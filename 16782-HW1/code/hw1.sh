#!/bin/bash

# Ensure that a map number is provided as an argument
if [ $# -ne 1 ]; then
    echo "Usage: $0 <map number>"
    exit 1
fi

MAP_NUM=$1

# Step 1: Run cmake

if [ ! -d "build" ]; then
  # Create the 'build' directory if it doesn't exist
  mkdir build
fi

# Change to the 'build' directory
cd build

echo "Running cmake .."
cmake ..
if [ $? -ne 0 ]; then
    echo "cmake failed!"
    exit 1
fi

# Step 2: Run make
echo "Running make"
make
if [ $? -ne 0 ]; then
    echo "make failed!"
    exit 1
fi

# Step 3: Run the test with the given map number
echo "Running ./run_test with map${MAP_NUM}.txt"
./run_test "../maps/map${MAP_NUM}.txt"
if [ $? -ne 0 ]; then
    echo "run_test failed!"
    exit 1
fi

# Step 4: Call the Python visualizer
echo "Running Python visualizer with map${MAP_NUM}.txt"
python3 ../scripts/visualizer.py "../maps/map${MAP_NUM}.txt"
if [ $? -ne 0 ]; then
    echo "Python visualizer failed!"
    exit 1
fi

echo "Pipeline finished successfully."
