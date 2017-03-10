import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

import logging
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
                ("~N({}, {}) noise".format(self.noise_mean, self.noise_stdev)
                    if self.noise else "no noise"),
            ])
        logging.debug(log_string)


    def distance(self, pos):
        # Report distance to some position, with some gaussian noise added
        pos = np.array(pos)
        distance = np.linalg.norm(self.pos - pos)
        # Add normal noise, usually between [-3*scale, 3*scale] meters
        noise = np.random.normal(loc=self.noise_mean, scale=self.noise_stdev)
        return distance + noise


    def update(self, time, pos):
        # Determine whether or not to provide a measurement update, providing
        # that update if yes.
        if time % self.freq == 0 and not np.random.binomial(1, self.dropout):
            return self.distance(pos)
        return None


def main():
    logging.root.setLevel(logging.DEBUG)
    # Generate 5 random beacon points between
    np.random.seed(1) # Seed with a constant value for testing
    NUM_BEACONS = 6
    beacons = [Beacon(tag=tag, dropout=.2)
            for tag in range(NUM_BEACONS)]


    for t in range(10):
        pos = [0, 0, 0]
        measurements = [beacon.update(t, pos) for beacon in beacons]
        print(measurements)


if __name__ == "__main__":
    main()

