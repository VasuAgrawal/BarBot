CC=g++
CFLAGS=--std=c++11 `pkg-config --cflags --libs protobuf`

nettest: nettest.cpp protos/dwdistance.pb.cc
	$(CC) -o nettest nettest.cpp protos/dwdistance.pb.cc $(CFLAGS)
