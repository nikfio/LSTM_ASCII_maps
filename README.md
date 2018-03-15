# LSTM as global path planner in ASCII maps based simulation environment

Simulation is conducted with MovingAI benchmarks.

This repo builds agents to solve global path plannig:
	- high level
	- low resolution
	
In particular, this is not an offline search like A* or similar, but it is an online search agent.
The main feature and advantage is memory usage: just the current state occupies the agent memory, while A* has to potentially store the all map.

Developing and testing on Ubuntu 14.04 LTS Trusty.
Core libraries: cuda-8.0 working with GTX 1080Ti, CAFFE latest from the master branch of the main repository.

Build instructions:

  0) cmake files search for caffe in /usr/local, just like for cuda, fastest way is to copy your caffe build in there

  1) place terminal in build directory

  2) rm -r *  (I usually upload already built versions, so you have to delete everything of the existing build)

  3) cmake ..

  4) make
  
  - executables are in build/src/


Default launch:
  from build directory

 	 ./src/TestNeuralNetwork 
	 
For more specified launch it's better to view src/TestNeuralNetwork to see the various gflags options, otherwise type 

	./src/TestNeuralNetwork --help
	
If you have suggestions or comments, feel free to contact me.






