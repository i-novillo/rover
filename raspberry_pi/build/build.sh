#!/bin/bash

# Clean-up 
find . -mindepth 1 -not -name 'build.sh' -exec rm -rf {} +

# Build environment
cmake ..
make