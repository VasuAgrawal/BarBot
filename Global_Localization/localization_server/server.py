import time
import logging
import queue
import threading
import pprint
import shutil
import time
import copy
import math

import tornado
import tornado.gen
import tornado.ioloop
import tornado.locks
import tornado.iostream
import tornado.tcpserver
import numpy as np

from scipy.spatial.distance import pdist as scipy_pdist
from scipy.spatial.distance import squareform
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import packet
from protos.dwdistance_pb2 import DwDistance

# Globals are probably bad right?
messages = queue.Queue()
data_lock = threading.Lock()
pdist_measured = []
dwm_idx_mapping = dict()
dwm_idx_mapping_inv = dict()
is_beacon = dict()

beacon_pos_guess = dict()
beacon_pos_lock = threading.Lock()


# This class is simple enough. All it does is spawn a Tornado TCP server to
# listen to messages coming in from all of the differen beacons and adds all of
# the messages to a threadsafe queue. The queue gets processed by the
# PointSolver class to turn those measurements into useable positions.
class DataRecvServer(tornado.tcpserver.TCPServer):

    # Aggregate the message data from each stream into a DwDistance message, and
    # then process the messages separately later.
    @tornado.gen.coroutine
    def handle_stream(self, stream, address): 
        logging.info("Opened stream from address %s", address)
        while True:
            try:
                # Continually read data from the stream
                message_bytes = yield packet.read_packet_from_stream(stream)
            except tornado.iostream.StreamClosedError:
                return

            message = DwDistance()
            message.ParseFromString(message_bytes)
            logging.debug("Server received message:\n%s", message)
            messages.put(message)


    def start(self):
        self.listen(8888)
        tornado.ioloop.IOLoop.instance().start()



# It doesn't really make much sense to have the data aggregator be built in to
# the point solver. This class consumes all of the messages in the 'messages'
# queue, and turns that into an updated list of pairwise distances. The current
# paradigm for updating measurements is to average measurements if they come
# from beacon - beacon communication, and replace it if there is a wristband on
# either end of message.
class DataAggregator(object):
    def __init__(self, copy_threshold=1):
        """Initialize method.

        Args:
            copy_threshold: Number of seconds of data to accumulate before
                replacing the global data with the newest measurements.
        """
        self._copy_threshold = copy_threshold
        self._copy_time = time.time()

        self._dwm_idx_mapping = dict() # Map DWM IDs to lines in the array
        self._dwm_idx_mapping_inv = dict() # Map lines in the array to DWM IDs
        self._is_beacon = dict() # Map DWM IDs to beacon status
        self._pdist_measured = [] # Measured pairwise distances between DWM chip


    def data_adder(self):
        # Spin endlessly trying to pull messages from the message queue.  If we
        # get a job, we'll process the distance measurement and add eventually
        # replace the global distance with a new set of updated ones.

        while True:
            message = messages.get()

            # Check for unknown sender.
            if message.send_id not in self._dwm_idx_mapping:
                logging.info("Adding a new DWM (ID = %d) to the mapping",
                        message.send_id)

                # This is a new beacon that we don't recognize, so we need to
                # create a new row and column for it in the distances array.
                num_modules = len(self._dwm_idx_mapping)
                self._dwm_idx_mapping[message.send_id] = num_modules
                self._dwm_idx_mapping_inv[num_modules] = message.send_id

                # Add a new row of measurements
                self._pdist_measured.append([None] * num_modules)
                for row in self._pdist_measured:
                    row.append(None) # Add a new column of measurements

                # We'll also store the row IDs of all of the beacons, and store
                # whether that ID is a beacon or not.
                self._is_beacon[message.send_id] = message.beacon

            # If we don't have information on both ends of the message, we
            # ignore the message. This means 
            if (message.send_id not in self._dwm_idx_mapping):
                logging.warning("Received a message containing an unkown sender"
                        " with ID %d", message.send_id)
                continue
            if (message.recv_id not in self._dwm_idx_mapping):
                logging.warning("Received a message containing an unknown"
                        " receiver with ID %d", message.recv_id)
                continue

            send_idx = self._dwm_idx_mapping[message.send_id]
            recv_idx = self._dwm_idx_mapping[message.recv_id]

            # Add the message to the distances list. As mentioned above, for
            # beacons we're going to take a rolling weighted average, whereas
            # for the wristbands we're simply going to replace the old value
            # with the new.
            if (self._is_beacon[message.send_id] and
                    self._is_beacon[message.recv_id]): # Both beacons
                
                current_distance = self._pdist_measured[send_idx][recv_idx]
                new_dist = message.dist
                if current_distance is None:
                    current_distance = new_dist

                weight = .9
                weighted_dist = (weight * current_distance + 
                                (1 - weight) * new_dist)

                # Force a symmetric distances update.
                self._pdist_measured[send_idx][recv_idx] = weighted_dist
                self._pdist_measured[recv_idx][send_idx] = weighted_dist

            else: # At least one of the two is wristbands
                new_dist = message.dist
                self._pdist_measured[send_idx][recv_idx] = new_dist
                self._pdist_measured[recv_idx][send_idx] = new_dist


            self._update_global_data()

    def _update_global_data(self):

        if time.time() - self._copy_time > self._copy_threshold:
            global data_lock
            global pdist_measured
            global dwm_idx_mapping
            global dwm_idx_mapping_inv
            global is_beacon

            with data_lock:
                pdist_measured = copy.deepcopy(self._pdist_measured)
                dwm_idx_mapping = copy.deepcopy(self._dwm_idx_mapping)
                dwm_idx_mapping_inv = copy.deepcopy(self._dwm_idx_mapping_inv)
                is_beacon = copy.deepcopy(self._is_beacon)

            self._copy_time = time.time()


# Take these readings that we have and plot / log them in some way to make
# debugging easier.
class Visualizer(object):
    def __init__(self):
        self._pdist_measured = []
        self._beacon_pos_guess = []


    def format_measured(self):
        log_lines = []
        log_lines.append("Measured distances between nodes, in receive order:")
    
        if self._pdist_measured:
            console_size = shutil.get_terminal_size((80, 40))

            # Note that this assumes that the matrix given is square. Things
            # won't display nicely if it isn't (if at all).
            col_width = console_size.columns // len(self._pdist_measured)

            data_fmt = "%{}f".format(col_width)
            none_str = " " * (col_width - 4) + "None"

            for line in self._pdist_measured:
                elem_strs = [
                    none_str if elem is None else data_fmt % elem
                        for elem in line]
                row = "".join(elem_strs)
                log_lines.append(row)

        else:
            log_lines.append("No measurements to display!")

        return "\n".join(log_lines)


    def plot(self):
        plt.ion()
        fig = plt.figure(1)
        plt.clf()
        ax = fig.add_subplot(111, projection='3d')

        # First, we make a local copy of the data so that the lock is free. In
        # this case, we'll care about both the measurements as well as the
        # solved positions for each of the points.
        sorted_keys = sorted(self._beacon_pos_guess.keys())  
        logging.debug("Sorted keys: %s", sorted_keys)

        xs = []
        ys = []
        zs = []
        for key in sorted_keys:
            point = self._beacon_pos_guess[key]
            xs.append(point[0])
            ys.append(point[1])
            zs.append(point[2])

        if xs and ys and zs:
            ax.plot(xs, ys, zs)
            logging.info("Plotted points:\nX: %s\nY: %s\nZ: %s", xs, ys, zs)

            ax.set_xlim((min(xs), max(xs)))
            ax.set_ylim((min(ys), max(ys)))
            ax.set_zlim((min(zs), max(zs)))

            plt.show(False)
            plt.pause(.001)

    
    def run(self):
        while True:
            start = time.time()
            
            global data_lock
            with data_lock:
                global pdist_measured
                self._pdist_measured = copy.deepcopy(pdist_measured)

            global beacon_pos_lock
            with beacon_pos_lock:
                global beacon_pos_guess
                self._beacon_pos_guess = copy.deepcopy(beacon_pos_guess)

            self.plot()
            logging.debug(self.format_measured())
            time.sleep(max(1 - (time.time() - start), 0))


class Solver(object):
    def __init__(self):
        # Global data
        self._pdist_measured = []
        self._dwm_idx_mapping = dict()
        self._dwm_idx_mapping_inv = dict()
        self._is_beacon = dict()
      
        # Private data
        self._beacon_idx = []
        self._wristband_idx = []
        self._beacon_pdist = []
        self._num_beacons = 0

        self._beacon_pos_guess = dict()
        self._guess_points = []
  

    # This function is invoked when the conditions for the solve are not met. We
    # sleep for some amount of time and then try to solve again.
    def _data_delay(self):
        time.sleep(1)


    # Gets a local copy of the data from the global data that is being updated
    # by the aggregator and (potentially) others.
    def _fetch_new_data(self):
        global data_lock
        with data_lock:
            global pdist_measured
            global dwm_idx_mapping
            global is_beacon
            
            self._pdist_measured = copy.deepcopy(pdist_measured)
            self._dwm_idx_mapping = copy.deepcopy(dwm_idx_mapping)
            self._dwm_idx_mapping_inv = copy.deepcopy(dwm_idx_mapping_inv)
            self._is_beacon = copy.deepcopy(is_beacon)

        logging.debug("Mapping from DWM ID to idx:\n%s",
                pprint.pformat(self._dwm_idx_mapping))
        logging.debug("Mapping from idx to DWM ID:\n%s",
                pprint.pformat(self._dwm_idx_mapping_inv))

        # Check to make sure that there are enough beacons to even attempt a
        # solution. At minimum, 3 beacons and 1 wristband are needed to
        # identify the position of the wristband, given beacon positions.
        MIN_BEACONS = 4
        if len(self._dwm_idx_mapping) < MIN_BEACONS:
            logging.warning("Not enough DWM modules to attempt a solution!"
                    " Expected to see %d, saw %d", MIN_BEACONS,
                    len(self._dwm_idx_mapping))


        # Check to make sure that there are enough beacons to even attempt a
        # solution. At minimum, 3 beacons and 1 wristband are needed to
        # identify the position of the wristband, given beacon positions.
        MIN_BEACONS = 4
        if len(self._dwm_idx_mapping) < MIN_BEACONS:
            logging.warning("Not enough DWM modules to attempt a solution!"
                    " Expected to see %d, saw %d", MIN_BEACONS,
                    len(self._dwm_idx_mapping))
            return False
        return True

    # From the measured pairwise distances between everything, pick out the
    # pairwise distances that are between beacons only.
    def _make_beacon_pdist(self):
        # First, convert _pdist_measured to a numpy array for easier
        # processing by the GD code.
        for i, line in enumerate(self._pdist_measured):
            for j, dist in enumerate(line):
                if dist is None:
                    self._pdist_measured[i][j] = -1 # No distance available
        self._pdist_measured = np.array(self._pdist_measured,
                dtype=np.double)

        # Determine which rows/cols in the measured distance matrix
        # correspond to beacons and which correspond to wristbands.
        self._beacon_idx = []
        self._wristband_idx = []
        for dwm_id in self._is_beacon:
            if self._is_beacon[dwm_id]:
                self._beacon_idx.append(self._dwm_idx_mapping[dwm_id])
            else:
                self._wristband_idx.append(self._dwm_idx_mapping[dwm_id])

        logging.debug("Indices of beacon modules: %s", self._beacon_idx)
        logging.debug("Indices of wristband modules: %s",
            self._wristband_idx)

        beacon_rows = self._pdist_measured[self._beacon_idx, :]
        self._beacon_pdist = beacon_rows[:, self._beacon_idx]
        self._num_beacons = len(self._beacon_idx)

        # Check for the density of the beacon pairwise distance matrix. If there
        # are too many invalid entries, we won't be able to get an accurate
        # solution.
        num_valid_in_row = (self._beacon_pdist >= 0).astype(int).sum(1)
        logging.debug("There are %s valid measurements in each beacon row.",
                num_valid_in_row)

        # Each beacon needs to be connected to at least MIN_VALID number of
        # beacons, or else we won't be able to come to an accurate solution.
        MIN_VALID = 3
        valid = True
        for i, valid_count in enumerate(num_valid_in_row):
            if valid_count < MIN_VALID:
                # Figure out the DWM ID of this beacon.
                # We first figure out the row / col that this beacon corresponds
                # to in the measurements list, and then work backwards from the
                # mapping to figure out the actual DWM ID of the offender
                idx = self._beacon_idx[i]
                dwm_id = self._dwm_idx_mapping_inv[idx]
                logging.warning("Beacon with DWM ID %d is not connected to"
                        " enough other beacons! Expected to be connected to at"
                        " least %d, but only connected to %d!",
                        dwm_id, MIN_VALID, valid_count)
                valid = False
        return valid


    def _norm(self, x):
        # Apparently writing my own norm is *marginally* faster
        return np.sqrt(np.power(x, 2).sum())


    def _distance_loss(self, M1, M2):
        assert(M1.shape == M2.shape)
        m1_line = M1.reshape(M1.size)
        m2_line = M2.reshape(M2.size)
        loss = 0
        for i in range(len(m1_line)):
            if m1_line[i] >= 0 and m2_line[i] >= 0:
                loss += (m1_line[i] - m2_line[i]) ** 2
        return loss


    def _gd_init(self):
        average_dist = np.average(self._beacon_pdist)
        random_init = np.random.random((self._num_beacons, 3)).astype(
            np.double) * average_dist
        return random_init
   

    def GD(self, measured, init, limit = 100, thresh = 1e-10):
        # Perform a gradient descent update given a measurements matrix and some
        # initialization. This will either be a random initialization or
        # something based on the previous values.

        guess_points = np.copy(init)
        num_points = len(guess_points)
        itercount = 0
        loss = 0 # I wish ...
        while itercount < limit:
            measured_guess = squareform(scipy_pdist(guess_points))
            gradients = np.zeros(guess_points.shape, dtype=np.double)
            for i, xi in enumerate(guess_points):
                skip_count = 0
                for j, xj in enumerate(guess_points):
                    # Abuse the fact that the distance between a point and
                    # itself is zero, so we can skip this case.
                    if i == j: continue
                    if measured[i][j] < 0:
                        # This wasn't able to be measured, so we'll attempt to
                        # compensate by first skipping the gradient entirely.
                        # Otherwise, perhaps scale by however many we're
                        # missing, though that's not going to be correct too.
                        skip_count += 1
                        continue

                    diff = xi - xj
                    gradients[i] += (
                        -2 *
                        (measured[i][j] - measured_guess[i][j]) * 
                        (diff / self._norm(diff))
                    )
                gradients[i] *= 2 # Correctness
                gradients[i] *= num_points / (num_points - skip_count)

            guess_points -= .005 * gradients
            # loss = np.power(measured - measured_guess, 2).sum()
            loss = self._distance_loss(measured, measured_guess)
            if loss < thresh:
                break
            itercount += 1
        return (guess_points, loss)


    def solve(self):

        while True:
            if not self._fetch_new_data():
                self._data_delay()
                continue

            if not self._make_beacon_pdist():
                self._data_delay()
                continue

            # At this point, we know that we have a "solvable" set of distances
            # in the beacon pdist matrix. We invoke gradient descent to generate
            # a solution.
            start_time = time.time()

            if len(self._beacon_pos_guess) != self._num_beacons:
                # If we've added beacons since the last iteration, throw away
                # all of our guesses and start over a few times, picking the
                # best random initialization out of the lot.
                best_guess_points = None
                best_loss = math.inf
                for i in range(10):
                    guess_points, loss = self.GD(self._beacon_pdist,
                            self._gd_init())
                    if loss < best_loss:
                        best_loss = loss
                        best_guess_points = guess_points
                    logging.debug("Iteration with random init found loss of %f",
                            loss)

                # Store the best guess points
                self._guess_points = best_guess_points

            else:
                # We're simply refining our previous approach, so pass in the
                # previous best guess rather than an entirely new set. The
                # algorithm is deterministic with respect to the starting
                # coordinates, so there's no point in doing this again.
                self._guess_points, best_loss = self.GD(self._beacon_pdist,
                        self._guess_points)

            logging.debug("Guess points calculated with loss %f:\n%s", 
                    best_loss, self._guess_points)

            for i, guess in enumerate(self._guess_points):
                idx = self._beacon_idx[i]
                dwm_id = self._dwm_idx_mapping_inv[idx]
                self._beacon_pos_guess[dwm_id] = guess
            
            logging.info("Guess points calulated and related to indices:\n%s",
                         pprint.pformat(self._beacon_pos_guess))

            # Update the global guess points
            global beacon_pos_lock
            with beacon_pos_lock:
                global beacon_pos_guess
                beacon_pos_guess = self._beacon_pos_guess

            time.sleep(max(0, 1 - (time.time() - start_time)))
                

def main():
    logging.root.setLevel(logging.DEBUG)

    # All of these could totally be their own separate ROS nodes ...
    server = DataRecvServer()
    server_thread = threading.Thread(target=server.start)
    server_thread.start()
    logging.info("Started receiving server thread!")

    aggregator = DataAggregator()
    aggregator_thread = threading.Thread(target=aggregator.data_adder)
    aggregator_thread.start()
    logging.info("Started aggregator thread!")

    visualizer = Visualizer()
    visualizer_thread = threading.Thread(target=visualizer.run)
    visualizer_thread.start()
    logging.info("Starting visualizer thread!")

    solver = Solver()
    solver_thread = threading.Thread(target=solver.solve)
    solver_thread.start()
    logging.info("Started solver thread")

    solver_thread.join()
    visualizer_thread.join()
    aggregator_thread.join()
    server_thread.join()


if __name__ == "__main__":
     main()
