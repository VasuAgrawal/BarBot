import tornado
import tornado.tcpclient

import time
import packet
import random
from protos.dwdistance_pb2 import DwDistance

def my_callback(stream):
    while True:
        message = DwDistance()
        message.send_id = random.randint(0, 9)
        message.recv_id = random.randint(0, 9)
        message.dist = round(random.random() * 10, 1)
       
        data = packet.make_packet_from_bytes(message.SerializeToString())
        print(data)
        stream.write(data)
        time.sleep(1)


def main():
    print("Connecting to server")
    client = tornado.tcpclient.TCPClient()
    client.connect("localhost", 8888, callback=my_callback)

    tornado.ioloop.IOLoop.instance().start()


if __name__ == "__main__":
    main()
