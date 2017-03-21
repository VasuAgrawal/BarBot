import tornado
import tornado.gen
import tornado.ioloop
import tornado.locks
import tornado.iostream
import tornado.tcpserver

import packet
from protos.dwdistance_pb2 import DwDistance

import time
import numpy as np

# Globals are probably bad right?
# Also, I need some way of figuring out how many beacons there are. Maybe that
# can be determined dynamically?
pdist = np.zeros((10, 10))
pdist_lock = tornado.locks.Lock()

class DataRecvServer(tornado.tcpserver.TCPServer):
    
    @tornado.gen.coroutine
    def handle_stream(self, stream, address): 
        # print("Opened stream from address", address)
        while True:
            try:
                # Continually read data from the stream
                message_bytes = yield packet.read_packet_from_stream(stream)
            except tornado.iostream.StreamClosedError:
                return

            message = DwDistance()
            message.ParseFromString(message_bytes)
            # At some point, come up with a more intelligent system that
            # involves a rolling average for the beacons, and just uses the most
            # recent point for the people.
            with (yield pdist_lock.acquire()):
                pdist[message.send_id][message.recv_id] = message.dist

@tornado.gen.coroutine
def parser():
    with (yield pdist_lock.acquire()):
        local_pdist = np.copy(pdist)
    print(local_pdist)
    print()

def main():
    # threading.Thread(target=parser).start()
    server = DataRecvServer()
    server.listen(8888)
    tornado.ioloop.PeriodicCallback(parser, 1000).start()
    tornado.ioloop.IOLoop.instance().start()

if __name__ == "__main__":
    main()
