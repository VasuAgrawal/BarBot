import sys
import numpy as np
import time
import matplotlib.pyplot as plt

def main():
    with open(sys.argv[1]) as f:
        data = np.array([float(line) for line in f.readlines()], dtype = float)
        x_axis = np.array(range(len(data)))
        with plt.xkcd():
            figure = plt.gcf()
            figure.set_size_inches(17, 11) # width, height
            plt.title("Distance to Boat (10 sample average, 10 Hz sample)")
            plt.xlabel("Time (s)")
            plt.ylabel("Distance (m)")
            plt.plot(x_axis, data)
            plt.savefig("Plot_%s.png" % int(time.time()), dpi=300)

main()
