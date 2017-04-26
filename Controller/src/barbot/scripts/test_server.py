import socket
import sys

# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Connect the socket to the port where the server is listening
server_address = ('localhost', 10000)
print >>sys.stderr, 'binding to %s port %s' % server_address
sock.bind(server_address)
sock.listen(5)

conn, address = sock.accept()
print "Connected to client at ", address

try:
    while(True):
        location = raw_input("Enter waypoint: ")
        if(location == "quit"):
            sock.close()
            break;
        print(type(location))
        print("sending waypoint: %s", location)
        conn.sendall(location)

finally:
    print >>sys.stderr, 'closing socket'
    sock.close()