#!/bin/bash

# Ensure that if test 20 configurations
if [ $# -ne 1 ]; then
    echo "Usage: $0 <if full test>"
    exit 1
fi

FULL_TEST=$1

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

# Step 2: Run cmake build
echo "Running cmake --build . --config Release"
cmake --build . --config Release
if [ $? -ne 0 ]; then
    echo "cmake --build failed!"
    exit 1
fi

# Change to the 'scripts' directory
cd ../scripts
# Step 3: Run the test
echo "Running grader_hw2.py"
python grader_hw2.py $FULL_TEST

if [ $? -ne 0 ]; then
    echo "grader_hw2 failed!"
    exit 1
fi

# Step 4: Call the Python visualizer
# echo "Running Python visualizer with map${MAP_NUM}.txt"
# python3 ../scripts/visualizer.py "../maps/map${MAP_NUM}.txt"
# if [ $? -ne 0 ]; then
#     echo "Python visualizer failed!"
#     exit 1
# fi

echo "Pipeline finished successfully."
