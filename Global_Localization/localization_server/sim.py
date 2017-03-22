import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import mpl_toolkits.mplot3d.art3d as art3d
import pprint
import shutil

import math
import packet
from protos.dwdistance_pb2 import DwDistance
import time
import logging
import socket
# from typing import 

import threading

class DW(object):
    def __init__(self, pos=None, tag: int = 0, frequency: int = 1,
            dropout: float = .1, noise: bool = True, noise_mean: float = 0.0, 
            noise_stdev: float = .1, beacon: bool = False, period: int = 10):
        """
        Args:
            pos: Position of the beacon. Randomly initialized if None
            tag: Beacon ID of some sort.
            frequency: units between updates.
            dropout: Packet loss in [0.0, 1.0]
            noise: Whether to add noise to distance measurement
            noise_mean: Mean of the noise to add to distance measurements
            noise_stdev: Standard deviation of noise to add. No noise if <= 0
            beacon: Whether this chip is a beacon or not
            period: Period of rotation for movement simulation
        """
        self.tag=tag
        self.freq = frequency
        self.dropout = dropout
        self.noise = noise
        self.noise_mean = noise_mean
        self.noise_stdev = noise_stdev
        self.beacon = beacon
        self.period = period

        if pos is None:
            if self.beacon:
                self.x = np.random.random() * 10 # X in [0, 10)
                self.y = np.random.random() * 10 # Y in [0, 10)
                self.z = np.random.random() * 2 + 2 # Z in [2, 4)
            else:
                self.x = np.random.random() * 8 + 1 # X in [1, 9)
                self.y = np.random.random() * 8 + 1 # Y in [1, 9)
                self.z = 0
        else:
            self.x, self.y, self.z = pos

        # pos should normally be updated by the update method.
        self._pos = np.array([self.x, self.y, self.z])
        self.pos = self._pos # Just to have something here.

        log_string = ", ".join([
                "Initialized DWM1000 tag {} at {}".format(self.tag, self.pos),
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


    def update(self, time, dest_pos):
        if self.beacon:
            self.pos = self._pos
        else:
            # Calculate a new position based on time. For now, wristbands will
            # sweep a 1m radius circle in the XY plane.
            angle = 2 * math.pi * (time % self.period) / self.period
            y = math.sin(angle)
            x = math.cos(angle)
            self.pos = self._pos + [x, y, 0]

        # Determine whether or not to provide a measurement update, providing
        # that update if yes.
        if time % self.freq == 0 and not np.random.binomial(1, self.dropout):
            # Add a maximum distance to be slightly more realistic
            dist = self.distance(dest_pos)
            if dist < 5:
                return dist
            # return self.distance(dest_pos)
        return None


class GLS(object):
    
    def __init__(self, beacons=0, wristbands=0):
        self._num_beacons = beacons
        self._num_wristbands = wristbands
        self._num_modules = self._num_beacons + self._num_wristbands
        # self._beacons = [DW(noise_stdev=.1, tag=tag, dropout=.5, beacon=True)
                # for tag in range(self._num_beacons)]
        # self._wristbands = [DW(noise_stdev=.1, tag=tag + self._num_beacons,
            # dropout=.5, beacon=False) for tag in range(self._num_wristbands)]

        self._modules = [DW(noise_stdev=.1, tag=tag, dropout=.5,
            beacon=(tag < self._num_beacons)) for tag in
            range(self._num_modules)]
        
        self._beacons = self._modules[:self._num_beacons]
        self._wristbands = self._modules[self._num_beacons:]

        self._sim_time = 0
        # Yes they're aliased, no, it doesn't matter.
        self._measurements = [[0] * self._num_modules] * self._num_modules
        # ignore the wristbands for now

        # Initialize the display
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        plt.ion()


    def print_measurements(self):
        print("Measurements at time", self._sim_time)
        console_size = shutil.get_terminal_size((80, 40))
        # Fit the mesurements nicely into columns.
        left_width = 0
        col_width = (console_size.columns - left_width) // self._num_modules
        single = "%{}f".format(col_width)
        fmt = "%s" * self._num_modules
        none_str = " " * (col_width - 4) + "None"
        for i, line in enumerate(self._measurements):
            print(fmt % tuple([single % elem if elem is not None else none_str
                for elem in line]))
        print()


    def step(self):
        # For every beacon, have it page every other beacon and get a distance
        # reading to it.

        # Order in the measurements array is based on the order that beacons are
        # kept in the list. At this point, we don't care about the tags.
        self._measurements = []
        for src_idx, src in enumerate(self._modules):
            self._measurements.append([])
            for dest_idx, dest in enumerate(self._modules):
                self._measurements[src_idx].append(src.update(self._sim_time,
                    dest.pos))
        self.print_measurements()
        self._sim_time += 1


    def display(self):
        all_points = list(zip(*[module.pos for module in self._modules]))
        beacon_points = list(zip(*[beacon.pos for beacon in self._beacons]))
        wristband_points = list(zip(*[wb.pos for wb in self._wristbands]))
        pprint.pprint(wristband_points)
        
        self.ax.clear()
        for wb in self._wristbands:
            p = patches.Circle(wb._pos[0:2], 1, fill=False)
            self.ax.add_patch(p)
            art3d.pathpatch_2d_to_3d(p, z=0, zdir='z')

        self.ax.set_xlim([0, max(10, max(all_points[0]))])
        self.ax.set_ylim([0, max(10, max(all_points[1]))])
        self.ax.set_zlim([0, max(4,  max(all_points[2]))])
        self.ax.scatter(*beacon_points, c=[1.0, 0.0, 0.0, 1.0], alpha=1.0,
                marker='s')
        self.ax.scatter(*wristband_points, c=[0.0, 1.0, 0.0, 1.0], alpha=1.0,
                marker='o')

        self.ax.set_xlabel("X axis (m)")
        self.ax.set_ylabel("Y axis (m)")
        self.ax.set_zlabel("Z axis (m)")
        plt.title("Reference positions in simulation")
        
        # self.ax.line(*beacon_points)
        plt.draw()
        plt.pause(1)
        # plt.show(False)

    
    def solve(self):
        pass
        # # So, as it turns out, there's a nice closed form solution you can
        # # employ as found on StackOverflow.
        # D = np.array(self._measurements)
        # M = np.zeros(D.shape)
        # for i in range(len(D)):
            # for j in range(len(D[i])):
                # M[i][j] = (D[0][j] ** 2 + D[i][0] ** 2 - D[i][j] ** 2) / 2
        # # print("M:", M)

        # # print("Rank of M", np.linalg.matrix_rank(M))
        # U, s, V = np.linalg.svd(M)
        # # print("U:", U)
        # # print(U.shape, s.shape, V.shape)
        # points = U @ np.diag(np.sqrt(s))
        # print("Points, in theory", points[:, :3])
        # # print("Points, in theory", U @ np.diag(np.sqrt(s)))
        # # print(D)
        # # print(M)

        # # Randomly initialize a state vector to zeros.
        # state = np.zeros(self._num_beacons * 3) # (x1, y1, z1, x2, y2, ...)
        
        # # Loss function will be the difference in measured distances vs
        # # calculated distances from predicted points
        # # Loss:
        # #   For every pair (i, j):
        # #       J(

    def send_updates(self):
        # Have every beacon start a new connection, send whatever updates it can
        # to the server, and then close the connections. This isn't quite how
        # the beacons will work, but it should stress the server more this way.
        # connections = [tornado.tcpclient.TCPClient()
                # for i in range(self._num_threads)]
        try:
            connections = [socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    for i in range(self._num_modules)]
            for conn in connections:
                conn.connect(("localhost", 8888))

            
            for src_idx, src in enumerate(self._modules):
                line = self._measurements[src_idx]
                for recv_idx, recv in enumerate(self._modules):
                    dist = line[recv_idx]
                    if dist is not None: # Should be None along diagonal
                        msg = DwDistance()
                        msg.send_id = src.tag
                        msg.recv_id = recv.tag
                        msg.dist = dist
                        msg.beacon = src.beacon
                        # Ignore the timestamp for now, not sure how to add that
                        # in a good way from sim.

                        data = packet.make_packet_from_bytes(
                                msg.SerializeToString())
                        connections[src_idx].send(data)

            for conn in connections:
                conn.close()
        except ConnectionRefusedError:
            logging.error("Unable to connect to server!")


def main():
    np.random.seed(1)
    logging.root.setLevel(logging.DEBUG)
    gls = GLS(beacons=6, wristbands = 2)
    # threading.Thread(target=gls.display).start()
    while True:
        gls.step()
        gls.send_updates()
        gls.display()
        # time.sleep(1)


if __name__ == "__main__":
    main()
