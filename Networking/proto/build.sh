#!/bin/bash

# Assumes protoc is installed in the system

# File that should be compiled
DWDISTANCE_FILE=dwdistance.proto
POSITIONS_FILE=positions.proto


# Make nanopb, just in case.
cd nanopb/generator/proto
make
cd ../../../

# Nanopb location out
NANOPB_OUT="../../Global_Localization/proto"
mkdir -p $NANOPB_OUT
protoc --plugin=protoc-gen-nanopb=nanopb/generator/protoc-gen-nanopb \
    --nanopb_out=$NANOPB_OUT $DWDISTANCE_FILE && \
    echo "Successfully compiled $DWDISTANCE_FILE into $NANOPB_OUT."

# Make protos for the localization server
LOC_SERV_LOC="../../Global_Localization/localization_server/protos"
mkdir -p $LOC_SERV_LOC
protoc --python_out=$LOC_SERV_LOC $DWDISTANCE_FILE $POSITIONS_FILE && \
    echo "Successfully compiled $DWDISTANCE_FILE, $POSITIONS_FILE into " \
         "$LOC_SERV_LOC."


CONTROLLER_LOC="../../Controller/src/barbot/scripts"
protoc --python_out=$CONTROLLER_LOC $POSITIONS_FILE && \
    echo "Successfully compiled $POSITIONS_FILE into $CONTROLLER_LOC."

SCHEDULER_LOC="../../Ordering/front_end_sample"
protoc --python_out=$SCHEDULER_LOC $POSITIONS_FILE && \
    echo "Successfully compiled $POSITIONS_FILE into $SCHEDULER_LOC."
