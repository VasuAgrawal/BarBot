import time
import logging
import queue
import threading
import pprint
import shutil
import time
import copy

import tornado
import tornado.gen
import tornado.ioloop
import tornado.locks
import tornado.iostream
import tornado.tcpserver
import numpy as np

import numpy as np
from scipy.spatial.distance import pdist as scipy_pdist
from scipy.spatial.distance import squareform
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import packet
from protos.dwdistance_pb2 import DwDistance

# Globals are probably bad right?
messages = queue.Queue()

class DataRecvServer(tornado.tcpserver.TCPServer):
    # Aggregate the message data from each stream into a DwDistance message, and
    # then process the messages separately later.
    @tornado.gen.coroutine
    def handle_stream(self, stream, address): 
        logging.debug("Opened stream from address", address)
        while True:
            try:
                # Continually read data from the stream
                message_bytes = yield packet.read_packet_from_stream(stream)
            except tornado.iostream.StreamClosedError:
                return

            message = DwDistance()
            message.ParseFromString(message_bytes)
            messages.put(message)


    def start(self):
        self.listen(8888)
        tornado.ioloop.IOLoop.instance().start()

class PointSolver(object):
    def __init__(self):
        self._distances = []
        self._mapping = dict()
        self._beacons = set()
        self._data_lock = threading.Lock()
        self._guess = None

    def print_measurements(self, measurements, labels = None):
        if measurements:
            print("Measurements following:")
            console_size = shutil.get_terminal_size((80, 40))
            # Fit the mesurements nicely into columns.
            left_width = 0
            col_width = (console_size.columns - left_width) // len(measurements)
            single = "%{}f".format(col_width)
            single_d = " ---------%{}d".format(col_width - 10)
            fmt = "%s" * len(measurements)
            none_str = " " * (col_width - 4) + "None"
            
            print(fmt % tuple([single_d % label for label in labels]))

            for i, line in enumerate(measurements):
                print(fmt % tuple([single % elem if elem is not None else none_str
                    for elem in line]))
            print()

    def data_adder(self):
        # Maintain a mapping from given beacon IDs to the index into our list that
        # the beacon actually is.
        mapping = dict()
        beacons = set()
        wristbands = set()
        distances = []
        copy_time = time.time()
        copy_threshold = 1 # Every second, update self with most recent data.

        while True:
            message = messages.get()

            # Add unknown senders to the mapping.
            if message.send_id not in mapping:
                num_items = len(mapping)
                mapping[message.send_id] = num_items
                distances.append([None] * num_items)
                for row in distances:
                    row.append(None)
                # Figure out how to classify the new tag
                if message.beacon:
                    beacons.add(message.send_id)
                else:
                    wristbands.add(message.send_id)

            # For now, we're only going to care about elements between beacons 
            if message.send_id in beacons and message.recv_id in beacons:
                send_idx = mapping[message.send_id]
                recv_idx = mapping[message.recv_id]

                current_distance = distances[send_idx][recv_idx]
                new_dist = message.dist
                if current_distance is None:
                    current_distance = new_dist
                # Since we're not going to maintain a complete history of all of
                # the distances we ever receive, we'll do a weighted average,
                # with weight going to the old measurement and 1-weight to the new.
                weight = .9
                weighted_dist = (weight) * current_distance + (1-weight) * new_dist 

                # Force the update to be symmetric
                distances[send_idx][recv_idx] = weighted_dist
                distances[recv_idx][send_idx] = weighted_dist

            if time.time() - copy_time > copy_threshold:
                # Copy over the local version of distances to the class version,
                # so that the solver can use a more updated set of data. This is
                # some buffering to avoid contention for the lock since
                # otherwise this function would have to lock on every message
                # received, which would probably kill throughput.
                with self._data_lock:
                    self._distances = copy.deepcopy(distances)
                    self._mapping = copy.deepcopy(mapping)
                    self._beacons = copy.deepcopy(beacons)

    def norm(self, x):
        # Apparently writing my own norm is *marginally* faster
        return np.sqrt(np.power(x, 2).sum())



    def plot(self, guess_points):
        plt.ion()
        xmin = guess_points[:, 0].min()
        ymin = guess_points[:, 1].min()
        xmax = guess_points[:, 0].max()
        ymax = guess_points[:, 1].max()

        guess_wrap = np.vstack((guess_points, guess_points[0]))

        ref_fig = plt.figure(1)
        plt.clf()
        ax = ref_fig.add_subplot(111, projection='3d')
        ax.plot(guess_wrap[:, 0], guess_wrap[:, 1], guess_wrap[:, 2])
        ax.set_xlim((xmin, xmax))
        ax.set_ylim((ymin, ymax))

        plt.show(False)
        plt.pause(.001)

    def data_delay(self):
        time.sleep(1)

    def distance_loss(self, M1, M2):
        assert(M1.shape == M2.shape)
        m1_line = M1.reshape(M1.size)
        m2_line = M2.reshape(M2.size)
        loss = 0
        for i in range(len(m1_line)):
            if m1_line[i] >= 0 and m2_line[i] >= 0:
                loss += (m1_line[i] - m2_line[i]) ** 2
        return loss


    def GD(self, measured, init, limit = 1000, thresh = 1e-10):
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
                        (diff / self.norm(diff))
                    )
                gradients[i] *= 2 # Correctness
                gradients[i] *= num_points / (num_points - skip_count)

            guess_points -= .005 * gradients
            # loss = np.power(measured - measured_guess, 2).sum()
            loss = self.distance_loss(measured, measured_guess)
            if loss < thresh:
                break
            itercount += 1
        return (guess_points, loss)

    def solve(self):
        distances = []
        mapping = dict()
        beacons = set()

        while True:
            with self._data_lock:
                distances = copy.deepcopy(self._distances)
                mapping = copy.deepcopy(self._mapping)
                beacons = copy.deepcopy(self._beacons)
           
            # First, make sure that there are enough distances to come up with
            # something for the beacon estimates.
            if len(beacons) < 3:
                print("Terminating solve, not enough beacons!")
                self.data_delay()
                continue
            
            start_time = time.time()

            # Now we have some subset of the pairwise distances between
            # everything. Using the GD as in math_test.py, we find some guess
            # for where the points are.

            # First, we trim down the list distances to only include things we
            # know are beacons. We also want to convert it to a numpy array, but
            # since we can't have `None` in an array we'll use negative numbers
            # instead, which works fine with distances.
            for i, line in enumerate(distances):
                for j, elem in enumerate(line):
                    if elem is None:
                        distances[i][j] = -1 # Pick some negative value
            
            # Convert to a numpy array, then select only columns which we know
            # to be beacons.
            measured = np.array(distances)
            labels = [False] * len(mapping)
            for key in mapping:
                labels[mapping[key]] = key in beacons
            labels = np.array(labels, dtype=np.bool)
            measured = measured[labels][:, labels]
          
            # Make sure we have enough valid datapoints to come to a good
            # solution. We'll do this by checking that there are at least 3
            # valid points for each.
            valid_cols = (measured >= 0).astype(int).sum(0)
            # valid = (measured >= 0).astype(int).sum(1).all()
            # num_valid = (measured >= 0).astype(int).sum()
            print("Number of valid measurements:", valid_cols)
            # if num_valid / measured.size < .5:
            if not (valid_cols >= 3).all():
                self.data_delay()
                continue
            print("There are enough valid points to process the measurements!")
            self.print_measurements(distances, labels)

            # Now we have a matrix very similar to the one we were using in math
            # test. There are a series of pairwise distances between some
            # points. We can now apply a gradient descent algorithm to determine
            # some guesses for where the points are, normalizing for points that
            # don't have as many points nearby by scaling their gradients
            # appropriately.
            num_points = len(measured)
            if self._guess is not None and len(self._guess) == num_points:
                # Just update based on the previous iteration
                guess_points = self._guess
                guess_points, loss = self.GD(measured, guess_points)

            else:
                # Pick the best loss over a few iterations.
                average_dist = np.average(measured)
                loss = None
                guess_points = None 
                for i in range(10):
                    gp = np.random.random((num_points,
                        3)).astype(np.double) * average_dist
                    gp, l = self.GD(measured, gp)
                    if loss is None or l < loss:
                        loss = l 
                        guess_points = gp

            print("Loss:", loss)
            measured_guess = squareform(scipy_pdist(guess_points))
            self._guess = guess_points

            # print("Measured distances", measured)
            # print("Guessed measurements", measured_guess)
            self.plot(guess_points)

            time.sleep(max(1 - (time.time() - start_time), 0))


def main():
    logging.root.setLevel(logging.INFO)
    server = DataRecvServer()
    server_thread = threading.Thread(target=server.start)
    server_thread.start()
    logging.info("Started receiving server thread!")

    # We could process in a separate thread, but it's more efficient to just
    # process here since there's nothing else going on.
    solver = PointSolver()
    processing_thread = threading.Thread(target=solver.data_adder)
    processing_thread.start()
    logging.info("Started processing thread!")

    solver_thread = threading.Thread(target=solver.solve)
    solver_thread.start()
    logging.info("Started solver thread!")

    solver_thread.join()
    processing_thread.join()
    server_thread.join()


if __name__ == "__main__":
     main()
