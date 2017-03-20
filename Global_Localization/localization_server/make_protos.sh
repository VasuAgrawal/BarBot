#!/usr/bin/env bash
mkdir -p protos
protoc --python_out=protos dwdistance.proto && \
    echo "Successfully compiled Protocol Buffers into 'protos' folder"
