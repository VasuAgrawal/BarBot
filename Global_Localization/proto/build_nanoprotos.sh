#!/bin/bash

# Assumes protoc is installed in the system

# File that should be compiled
PROTOFILE=dwdistance.proto

protoc --plugin=protoc-gen-nanopb=nanopb/generator/protoc-gen-nanopb --nanopb_out=. $PROTOFILE
