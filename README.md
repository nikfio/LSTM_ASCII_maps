# NN-Roomba
Neural network-based path planning for mobile robots.

This repo is meant to build a path planning method based on neural networks. 
Simulation is conducted with MovingAI benchmarks.
Test in real environment are performed on the Roomba 600 with a Hokuyo LRF.

TASK LIST:
  1) build a optimal path planning solver to provide label data for supervised learning --> COMPLETED for simulation
     there it is implemented a supervisor based on the A* algorithm
  
  2) build libraries to train and validate a general neural network --> COMPLETD for simulation

  3) develop new methods / test neural network solutions: ONGOING 

	      @)now working on a LSTM neural network, first tests show improvements

		with respect to feed-forward nets and deep feed-forward nets
  
	      @)investigating and starting to implement a way to use topological-like

		map to reduce input dimension and computation time
  

Developing and testing on Ubuntu 16.04.3 LTS Xenial.

Build instructions:

  0) take care to change directories in /libs/NeuralNetwork/CMakeLists.txt 
     to your equivalent Caffe installation folder and related,
     if it does not work, ask for help.
     UPDATE: soon a working build of CAFFE will be uploaded inside this repo

  1) place terminal in build directory

  2) rm -r *

  3) cmake ..

  4) make
  
  - executables are in build/src/
  
  
Consider the same path-to-folder, directives for Simulation files location: 
  -keep *.map.scen files in path-to-folder/Scenarios
  -keep *.map files in path-to-folder/Maps


Usage:
  from build directory
  ./src/TestSupervisor path-to-Scenarios-folder [num of maps] [num test for each map] 

  ./src/TestNeuralNetwork path-to-Scenarios-folder [num of maps for training set] [num of maps for validation set] ...
					...        [amount of paths for Train set] [amount of paths for Validation set]



Please, if you want, signal if nasty errors like segmentation fault occur.


