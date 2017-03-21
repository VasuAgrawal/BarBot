import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import pprint
import shutil

import packet
from protos.dwdistance_pb2 import DwDistance
import time
import logging
import socket
# from typing import 

class Beacon(object):
    def __init__(self, pos=None, tag: int = 0, frequency: int = 1,
            dropout: float = .1, noise: bool = True, noise_mean: float = 0.0, 
            noise_stdev: float = .1):
        """
        Args:
            pos: Position of the beacon. Randomly initialized if None
            tag: Beacon ID of some sort.
            frequency: units between updates.
            dropout: Packet loss in [0.0, 1.0]
            noise: Whether to add noise to distance measurement
            noise_mean: Mean of the noise to add to distance measurements
            noise_stdev: Standard deviation of noise to add. No noise if <= 0
        """
        self.tag=tag
        self.freq = frequency
        self.dropout = dropout
        self.noise = noise
        self.noise_mean = noise_mean
        self.noise_stdev = noise_stdev

        if pos is None:
            self.x = np.random.random() * 10 # X in [0, 10)
            self.y = np.random.random() * 10 # Y in [0, 10)
            self.z = np.random.random() * 2 + 5 # Z in [5, 7)
        else:
            self.x, self.y, self.z = pos

        self.pos = np.array([self.x, self.y, self.z])
        log_string = ", ".join([
                "Initialized beacon tag {} at {}".format(self.tag, self.pos),
                "with {} dropout".format(self.dropout),
                "data every {} units".format(self.freq),
                ("N({}, {}) noise".format(self.noise_mean, self.noise_stdev)
                    if self.noise else "no noise"),
            ])
        logging.debug(log_string)


    def distance(self, pos):
        # Report distance to some position, with some gaussian noise added
        pos = np.array(pos)
        distance = np.linalg.norm(self.pos - pos)
        # Add normal noise, usually between [-3*scale, 3*scale] meters
        noise = (0 if self.noise_stdev <= 0 else 
                np.random.normal(loc=self.noise_mean, scale=self.noise_stdev))
        return max(distance + noise, 0)


    def update(self, time, pos):
        # Determine whether or not to provide a measurement update, providing
        # that update if yes.
        if time % self.freq == 0 and not np.random.binomial(1, self.dropout):
            return self.distance(pos)
        return None


class GLS(object):
    
    def __init__(self, beacons=0, wristbands=0):
        self._num_beacons = beacons
        self._beacons = [Beacon(noise_stdev=.1, tag=tag, dropout=.5) 
                for tag in range(beacons)]
        self._sim_time = 0
        # Yes they're aliased, no, it doesn't matter.
        self._measurements = [[0] * self._num_beacons] * self._num_beacons
        # ignore the wristbands for now


    def print_measurements(self):
        print("Measurements at time", self._sim_time)
        console_size = shutil.get_terminal_size((80, 40))
        # Fit the mesurements nicely into columns.
        left_width = 0
        col_width = (console_size.columns - left_width) // self._num_beacons
        single = "%{}f".format(col_width)
        fmt = "%s" * self._num_beacons
        none_str = " " * (col_width - 4) + "None"
        for i, line in enumerate(self._measurements):
            print(fmt % tuple([single % elem if elem is not None else none_str
                for elem in line]))
        print()


    def step(self):
        # For every beacon, have it page every other beacon and get a distance
        # reading to it.
        self._measurements = []
        for src_idx, src in enumerate(self._beacons):
            self._measurements.append([])
            for dest_idx, dest in enumerate(self._beacons):
                self._measurements[src_idx].append(src.update(self._sim_time,
                    dest.pos))
        self.print_measurements()
        self._sim_time += 1


    def display(self, ax):
        points = list(zip(*[beacon.pos for beacon in self._beacons]))
        # ax.set_xlim([0, max(points[0])])
        # ax.set_ylim([0, max(points[1])])
        ax.set_zlim([0, max(points[2])])
        ax.scatter(*points)
        # Go through every single set of points 

    
    def solve(self):
        # So, as it turns out, there's a nice closed form solution you can
        # employ as found on StackOverflow.
        D = np.array(self._measurements)
        M = np.zeros(D.shape)
        for i in range(len(D)):
            for j in range(len(D[i])):
                M[i][j] = (D[0][j] ** 2 + D[i][0] ** 2 - D[i][j] ** 2) / 2
        # print("M:", M)

        # print("Rank of M", np.linalg.matrix_rank(M))
        U, s, V = np.linalg.svd(M)
        # print("U:", U)
        # print(U.shape, s.shape, V.shape)
        points = U @ np.diag(np.sqrt(s))
        print("Points, in theory", points[:, :3])
        # print("Points, in theory", U @ np.diag(np.sqrt(s)))
        # print(D)
        # print(M)

        # Randomly initialize a state vector to zeros.
        state = np.zeros(self._num_beacons * 3) # (x1, y1, z1, x2, y2, ...)
        
        # Loss function will be the difference in measured distances vs
        # calculated distances from predicted points
        # Loss:
        #   For every pair (i, j):
        #       J(

    def send_updates(self):
        # Have every beacon start a new connection, send whatever updates it can
        # to the server, and then close the connections. This isn't quite how
        # the beacons will work, but it should stress the server more this way.
        # connections = [tornado.tcpclient.TCPClient()
                # for i in range(self._num_threads)]
        try:
            connections = [socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    for i in range(self._num_beacons)]
            for conn in connections:
                conn.connect(("localhost", 8888))

            for send, line in enumerate(self._measurements):
                for recv, dist in enumerate(line):
                    if dist is not None:
                        msg = DwDistance()
                        msg.send_id = send
                        msg.recv_id = recv
                        msg.dist = dist
                        data = packet.make_packet_from_bytes(
                                msg.SerializeToString())
                        connections[send].send(data)
            for conn in connections:
                conn.close()
        except ConnectionRefusedError:
            pass


def main():
    logging.root.setLevel(logging.DEBUG)
    gls = GLS(beacons=10)
    while True:
        gls.step()
        gls.send_updates()
        time.sleep(1)
    # gls.solve()

    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')
    # gls.display(ax)
    # plt.show()


if __name__ == "__main__":
    main()
