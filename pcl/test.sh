#!/bin/bash

# Function to run the program
run_program() {
  core_id=$1  # Passed argument to identify the core/process
  ./build/localization > "output_${core_id}.csv"  # Redirect output to a unique file
}

export -f run_program  # Export the function for parallel to use

# Run the program on 4 cores, for example
seq 50 | parallel run_program

