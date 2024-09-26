#!/usr/bin/env bash

PORT=$1

ffplay -analyzeduration 1 -fflags -nobuffer -probesize 32 -sync ext -i udp://192.168.0.135:$PORT
