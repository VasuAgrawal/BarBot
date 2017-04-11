#include <iostream>
#include <string>

#include <google/protobuf/text_format.h>
#include "protos/dwdistance.pb.h"

#include <cstdlib>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <unistd.h>

// Makes one attempt at connecting to the server at the given destination. A
// failure is indicated with a negative number. Implement some try-again logic
// above this if you want.
int connect_to_server(std::string addr, std::string port) {
  struct addrinfo hints, *listp, *p;
  memset(&hints, 0, sizeof hints);
  hints.ai_family = AF_INET; // IPv4
  hints.ai_socktype = SOCK_STREAM; // TCP?
  hints.ai_flags = AI_NUMERICSERV;
  hints.ai_flags |= AI_ADDRCONFIG;

  int rv;
  if ((rv = getaddrinfo(addr.c_str(), port.c_str(), &hints, &listp)) != 0) {
    std::cerr << "Error in getaddrinfo: " << gai_strerror(rv) << std::endl;
    freeaddrinfo(listp);
    return -1;
  }

  int fd;
  for (p = listp; ; p = p->ai_next) {
    // Create a socket descriptor
    if ((fd = socket(p->ai_family, p->ai_socktype, p->ai_protocol)) < 0) {
      std::cerr << "Error in creating socket." << std::endl;
      continue;
    }

    // Attempt to connect to that socket descriptor.
    if (connect(fd, p->ai_addr, p->ai_addrlen) < 0) {
      std::cerr << "Error in connecting to socket." << std::endl;
      close(fd);
      continue;
    }
    
    // If we get here, we know that we've successfully connected probably.
    break;
  }

  // Check to make sure that we don't have a null thing.
  if (p == nullptr) {
    std::cerr << "Failed to connect" << std::endl;
    freeaddrinfo(listp);
    return -2;
  }

  freeaddrinfo(listp);
  return fd;
}

int main(int argc, char* argv[]) {
    int fd;
    std::string addr = "localhost";
    std::string port = "8888";
    if ((fd = connect_to_server(addr, port)) < 0) {
      std::cerr << "Unable to connect to server at " << addr << ":" << port 
        << std::endl;
      exit(fd);
    }

    GOOGLE_PROTOBUF_VERIFY_VERSION;
    DwDistance d;
    d.set_dist(20);
    d.set_send_id(1);
    d.set_recv_id(2);
    d.set_beacon(true);

    uint32_t byte_size = d.ByteSize();
    std::cout << "Byte Size: " << byte_size << std::endl;
    uint8_t *bytes = new uint8_t[4 + byte_size]; // 4 bytes for header

    // Little endian encoding of packet size.
    bytes[0] = (byte_size >> 0)  & 0xFF;
    bytes[1] = (byte_size >> 8)  & 0xFF;
    bytes[2] = (byte_size >> 16) & 0xFF;
    bytes[3] = (byte_size >> 24) & 0xFF;
    d.SerializeToArray(&bytes[4], byte_size);
    for (size_t i = 0; i < 4 + byte_size; ++i) {
      std::cout << std::to_string((int)bytes[i]) << ", ";
    }
    std::cout << std::endl;

    // Attempt to send the array over the socket
    if (write(fd, bytes, 4 + byte_size) < 0) {
      std::cerr << "Error in writing to socket!"  << std::endl;
    }

    std::string out;
    google::protobuf::TextFormat::PrintToString(d, &out);
    std::cout << out << std::endl;
    google::protobuf::ShutdownProtobufLibrary();

    delete[] bytes;
    close(fd);
}
