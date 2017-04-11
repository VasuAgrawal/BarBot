#!/bin/bash

# Assumes protoc is installed in the system

# File that should be compiled
PROTOFILE=dwdistance.proto

protoc --plugin=protoc-gen-nanopb=nanopb/generator/protoc-gen-nanopb --nanopb_out=. $PROTOFILE

# Make protos for the localization server
LOC_SERV_LOC="../localization_server/protos"
mkdir -p $LOC_SERV_LOC
protoc --python_out=$LOC_SERV_LOC $PROTOFILE && \
    echo "Successfully compiled Protocol Buffers into 'protos' folder"
