#!/bin/bash

# Assumes protoc is installed in the system

# File that should be compiled
PROTOFILE=dwdistance.proto

# Make nanopb, just in case.
cd nanopb/generator/proto
make
cd ../../../

# Nanopb location out
NANOPB_OUT="../../Global_Localization/proto"
mkdir -p $NANOPB_OUT
protoc --plugin=protoc-gen-nanopb=nanopb/generator/protoc-gen-nanopb \
    --nanopb_out=$NANOPB_OUT $PROTOFILE && \
    echo "Successfully compiled $PROTOFILE into $NANOPB_OUT."

# Make protos for the localization server
LOC_SERV_LOC="../../Global_Localization/localization_server/protos"
mkdir -p $LOC_SERV_LOC
protoc --python_out=$LOC_SERV_LOC $PROTOFILE && \
    echo "Successfully compiled $PROTOFILE into $LOC_SERV_LOC."
