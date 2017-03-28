import tornado
import tornado.tcpclient

import time
import packet
import random
from protos.dwdistance_pb2 import DwDistance

import numpy as np
from scipy.spatial.distance import pdist as scipy_pdist
from scipy.spatial.distance import squareform

# A spiky crown or something
ref_points = np.array([[66, 200, 0], [165, 240, 0], [133, 315, 0], 
    [233, 253, 0], [266, 315, 0], [261, 228, 0], [333, 200, 0], [266, 84, 0],
    [195, 124, 0], [122, 82, 0], [151, 182, 0]], dtype=np.double)

def pdist(X):
    # Unlike the norm, pdist is not a trivial calculation so it works out to be
    # more than 2x as fast to use the scipy pdist rather than using my own.
    return squareform(scipy_pdist(X))

pairwise = pdist(ref_points)

def my_callback(stream):
    while True:
        for send_idx, line in enumerate(pairwise):
            for recv_idx, dist in enumerate(line):
                if send_idx == recv_idx: continue
                if dist > 150: continue
                message = DwDistance()
                message.send_id = send_idx
                message.recv_id = recv_idx
                message.dist = dist
                message.beacon = True
       
                data = packet.make_packet_from_bytes(message.SerializeToString())
                stream.write(data)

        time.sleep(1)


def main():
    print("Connecting to server")
    client = tornado.tcpclient.TCPClient()
    client.connect("localhost", 8888, callback=my_callback)

    tornado.ioloop.IOLoop.instance().start()


if __name__ == "__main__":
    main()
