/* Implementation file of the functions defined in Train_and_Validate.hpp
 * section: functions dedicated to ONLINE TESTING
 * Important assumptions:
 * 
*/

// C++ 
#include <string>
#include <vector>
#include <cstdio>
#include <cstdlib>
#include <algorithm>
#include <csignal>
#include <glog/logging.h>


// caffe related
#include <caffe/caffe.hpp>
#include <caffe/blob.hpp>
#include <caffe/solver.hpp>
#include <caffe/solver_factory.hpp>
#include <caffe/sgd_solvers.hpp>
#include <caffe/util/io.hpp>
#include <caffe/layers/memory_data_layer.hpp>


// project related
#include "Build_set.hpp"
#include "Common_net.hpp"
#include "TrainValidateRNN.hpp"


using namespace std;
using namespace caffe;

using namespace PathPlanning;
using namespace Supervisor;

DECLARE_bool(gpu);
DECLARE_bool(diagonal);
DECLARE_string(trained);
DECLARE_string(online_model);
DECLARE_int32(online_paths);
DECLARE_int32(seq_size);

namespace NeuralNetwork {

BenchmarkSet OnlineSet;
vector<Scenario> OnlineArchive;

void TrainValidateRNN::OnlineTestSupervisor(string& filename) {

	//cout << "Results Online Test Supervisors side" << endl;
	float TotalLength = 0;

	for(int i = 0; i < OnlineSet.length.size(); i++) {
		
		TotalLength += OnlineSet.length[i];

	}
	
	float TotalPaths = OnlineSet.length.size();

	float MeanLength = TotalLength / TotalPaths;

	float TotalTime_astar = 0;
	for(int i = 0; i < OnlineSet.solved_time.size(); i++) {
		
		TotalTime_astar += OnlineSet.solved_time[i];

	}
	
	float AverageTime_astar = TotalTime_astar / TotalPaths;

	float TotalNodesExp_astar = 0;
	for(int i = 0; i < OnlineSet.expanded.size(); i++) {
		
		TotalNodesExp_astar += OnlineSet.expanded[i];

	}
	
	float AverageNodesExp_astar = TotalNodesExp_astar / TotalPaths; 

//	printf("ONLINE TEST: A* PERFORMANCE: Total paths = %.0f  Astar_unsolved = %d Astar_total_length = %.6f  Astar_mean_leangth = %.6f "
//	       " Astar_total_time = %.6f  Astar_mean_time = %.6f  \n  Astar_mean_nodes_expanded = %.6f \n ",
//		 TotalPaths, OnlineSet.unsolved, TotalLength, MeanLength, TotalTime_astar, AverageTime_astar, AverageNodesExp_astar);

	FILE * online_test = fopen(filename.c_str(), "w");
	if( online_test == NULL ) {
		cout << "Plot file opening failed.\n";
		exit(1);
	}

	int print_check = fprintf(online_test, "ONLINE TEST: A* PERFORMANCE: \n Total_paths = %.0f \n Astar_unsolved: %d \n"
						" Astar_total_length = %.6f \n Astar_mean_leangth = %.6f \n"
						" Astar_total_time = %.6f \n Astar_mean_time = %.6f  \n "
						" Astar_mean_nodes_expanded = %.6f \n ",
						  TotalPaths, OnlineSet.unsolved, 
						  TotalLength, MeanLength,
						  TotalTime_astar, AverageTime_astar,
						  AverageNodesExp_astar);

	if(print_check <= 0) {
	   printf("File: writetofile() failed\n");
	   exit(1);
	}
	fclose(online_test);

}

void TrainValidateRNN::UpdateOnlineNet()
{
	caffe::NetParameter net_param;
	net->ToProto(&net_param);
	net_param.mutable_state()->set_phase(caffe::Phase::TEST);
	online_net->CopyTrainedLayersFrom(net_param);	
}

void TrainValidateRNN::build_input_sample(Map& map, Pixel& direction_taken, Pixel& current, Pixel& target) {

	
	DistanceMap(dist_state, map, current, direction);

	vector<float> current_state(sequence_length, 0);

	for(int i = online_time_sequence - 1; i > 0; i--) {
		
		for(int j = 0; j < sequence_length; j++)  {
			online_blobData->mutable_cpu_data()[ (i - 1) * sequence_length + j]
								= states_tail[i * sequence_length + j];
			
			cout << online_blobData->mutable_cpu_data()[ (i - 1) * sequence_length + j] << "  ";
		}
		
		cout << endl;		
		
	
	} 
			
	for(int i = 0; i < dist_state.size(); i++) {
		online_blobData ->mutable_cpu_data()[last_out_pos + i] = dist_state[i];
		current_state[i] = dist_state[i];
	}

	online_blobData->mutable_cpu_data()[last_out_pos + dist_state.size()] = getModDistance(current, target);
	online_blobData->mutable_cpu_data()[last_out_pos + dist_state.size() + 1] = getRelativeAngle(current, target);
	
	current_state[dist_state.size()] = getModDistance(current, target);
	current_state[dist_state.size() + 1] = getRelativeAngle(current, target);

	// setting the new tail in the input state
	for(int i = 0; i < tail.size(); i++) {
				
		online_blobData->mutable_cpu_data()[last_out_pos + dist_state.size() + 2 + i] = tail[i];
		current_state[dist_state.size() + 2 + i] = tail[i];

	}

//	for(int i = 0; i < sequence_length; i++) {
//		cout << online_blobData->cpu_data()[last_out_pos + i] << "  " ;
//	}
//	cout << endl;
//	
//	getchar();

	// setting the new tail in the input state
	if( !tail.empty() ) {
		for(int i = tail.size() - 1; i > 0; i-- ) { 
			tail[i] = tail[i-1];
		}
		tail[0] = online_blobArgmax->cpu_data()[0];
		
	}
	

	for(int i = 0; i < online_time_sequence - 1; i++) {
		
//		cout << "STATE TAIL: " << i;
		for(int j = 0; j < sequence_length; j++)  {
			states_tail[i * sequence_length + j] = states_tail[(i+1) * sequence_length + j];
//			cout << "  " << states_tail[i * sequence_length + j] << "  ";
		}
//		getchar();
	} 
//	cout << endl;

//	getchar();

	for(int j = 0; j < sequence_length; j++)  {
		states_tail[last_out_pos * sequence_length + j] = current_state[j];
	}

	

	// populate clip blob 
	if( step % TIME_SEQUENCE == 0 ) {
		for(int j = 0; j < sequence_length; j++) {
			online_blobClip->mutable_cpu_data()[j] = 0;
		}
	}
	else {
		for(int j = 0; j < sequence_length; j++) {
			online_blobClip->mutable_cpu_data()[j] = 1;
		}
	}

	

};


TrainValidateRNN::TrainValidateRNN( string& direction_mode,
				   			 vector<string>& OnlinePaths )
{
	setDirections(direction, direction_mode);

	// set the allowed directions 
	string diag = "diagonal";
	setDirections(direction_diag, diag);

	// set tail of previous steps size
	tail.resize(FLAGS_seq_size - MIN_STATE_SIZE); 
	// and set random init
	for( int i = 0; i < tail.size(); i++) {
		tail[i] = rand() % direction.size();
	}

	// allow ranges map in DIAGONAL always
	dist_state.resize(8);

     sequence_length = FLAGS_seq_size;

	
	ScenarioList OnlineList(OnlinePaths, OnlinePaths.size());
	OnlineArchive = OnlineList.scenario_archive;
	build_benchmarking_set(OnlineList, OnlineSet, FLAGS_online_paths, direction_mode);
	
	Caffe::set_mode(Caffe::CPU);

	FLAGS_minloglevel = 2;
	// initialize net for online test
	online_net.reset(new caffe::Net<float>(FLAGS_online_model, caffe::TEST));
	online_net->CopyTrainedLayersFrom(FLAGS_trained);

	online_blobData = online_net->blob_by_name("data");
	online_blobClip = online_net->blob_by_name("clip");
	online_blobArgmax = online_net->blob_by_name("argmax");

	CHECK_EQ(sequence_length, online_blobData->shape(1)) << "online net: seq_size and data shape(1) must be equal";
	
	states_tail = vector<float>(online_blobData->shape(0) * sequence_length, 0);

	last_out_pos = online_blobData->shape(0) - 1;
	online_time_sequence = online_blobData->shape(0);
	
	time_t now = time(0);
	tm *local = localtime(&now);

	string online_test = "online_test" + online_net->name() + "_" + to_string(local->tm_mon) 
				     + "-" + to_string(local->tm_mday) + "-" + to_string(local->tm_hour)
				     + "_" + to_string(local->tm_min);

	LOG(INFO) << "ONLINE NET INITIALIZED: unrolling for " << online_time_sequence << " timesteps";

	OnlineTestSupervisor(online_test);

	OnlineNetTest(direction_mode, online_test);

};

float TrainValidateRNN::TrueDistance(Pixel& current, Pixel& target) {

	return static_cast<float>( sqrt(pow(target.x - current.x, 2)
				 + pow(target.y - current.y, 2)) );	

};

void TrainValidateRNN::OnlineNetTest(string& direction_mode, string& filename) {
	
	float TotalPaths = 0;
	float TotalLength_astar = 0;
	float TotalTime_astar = 0;
	float TotalNodesExp_astar = 0;	

	vector<CoordinateList> Net_paths_solved;
	int paths_attempted = 0;
	int unsolved_net = 0; int solved_net = 0;
	double online_start, online_end, path_start, path_end;
	vector<float> elapsed_time;
	vector<int> Net_expanded;

	float online_precision = 0;
	float Online_accu = 0;
	float Online_percentage = 0;
	string online_path = "online_path";

	int aux_int;
	FLAGS_minloglevel = 2;
	
	online_start = getTime();	
	while( paths_attempted < OnlineSet.OptimalPaths.size() ) {
		cout << "Attempt " << paths_attempted 
			<< " Map index: " << OnlineSet.map_index[index] << endl;

		Map map = OnlineSet.Maps[OnlineSet.map_index[index]];
				
		Pixel target  = OnlineSet.OptimalPaths[index].back(); 
		Pixel source  = OnlineSet.OptimalPaths[index].front(); 
		printMap_point(map, target, online_path, 'G');
		
		cout << " source: " << source << " target: " << target  << endl;
		//getchar();
		Pixel current = source; Pixel direction_taken = OnlineSet.OptimalPaths[index].at(1) - source;
		step=0;
		CoordinateList PathNet;
		PathNet.push_back(current);
		int node_exp_net = aux_int = 0;
		path_start = getTime(); 
		bool tolerance = false;
		while( !tolerance && step < MAX_STEPS) {
			
			build_input_sample(map, direction_taken, current, target);
			online_net->Forward();
			direction_taken = direction[online_blobArgmax->cpu_data()[0]];
			current = current + direction_taken;
			
//			cout << "directions taken ";
//			for(int i = 0; i < online_time_sequence; i++ ){
//			cout << "  " << online_blobArgmax->cpu_data()[i] << " ";
//			}
//			cout << "  ";

			node_exp_net++;
			float truedist = TrueDistance(current, target);

//			set a tolerance in goal pixel reaching
			if( truedist <= 1 ) 
				tolerance = true;
			else
				//cout << truedist << endl;
 
			if ( OutOfBounds(map, current) || detectCollision(map.coord[current.x][current.y]) ) {
//				 aux_int++;
//				 cout << "Auxiliary" << endl;
//				 bool intervention = take_free_rand(map, PathNet.back(), 50, current);
//				 if( !intervention ) 
//					break;
				 cout << "FAILURE! Aborting.." << endl;
				break;
			} 
//			cout << "Direction taken: " << current << "  target: " << target << endl;
			step++; 
			PathNet.push_back(current);

		}	
		path_end = getTime();
		
		
		if( !tolerance || step == MAX_STEPS) {
			unsolved_net++;	
			cout << current << " step: " << step << endl;
			string wrong_path = "failed_path_" + std::to_string(unsolved_net);
			PrintPathMod(PathNet, map, wrong_path, 'N');
			CoordinateList SupPath = OnlineSet.OptimalPaths[index];
			string sup_path = "supervised_failed_path_" + std::to_string(unsolved_net);
			PrintPath(SupPath, map, sup_path, 'A');
		}
		else {
			solved_net++;
			elapsed_time.push_back(path_end - path_start);
			Net_paths_solved.push_back(PathNet);
			Net_expanded.push_back(node_exp_net);
			cout << "NETWORK ACHIEVED TARGET: " << elapsed_time.back()
			     << "   Supervisor time: " << OnlineSet.solved_time[index] << endl;	
			string solved_path = "solved_path_" + std::to_string(solved_net);
			PrintPathMod(PathNet, map, solved_path, 'N');
			CoordinateList SupPath = OnlineSet.OptimalPaths[index];
			string sup_path = "supervised_solved_path_" + std::to_string(solved_net);
			PrintPath(SupPath, map, sup_path, 'A');	
			TotalTime_astar += OnlineSet.solved_time[index];	
			TotalLength_astar += OnlineSet.length[index];
			TotalNodesExp_astar += OnlineSet.expanded[index];
			 
			TotalPaths++;
		}


		index++;
		paths_attempted++;

	}
	online_end = getTime();

	float TotalTime_net = 0;
	 
	cout << "Solved by net: " << elapsed_time.size() << endl;
	if( !elapsed_time.empty() ) {
		for(int i = 0; i < elapsed_time.size(); i++) {
			TotalTime_net += elapsed_time[i];
		}
	}
	

	float TotalLength_net = 0;

	if( !Net_paths_solved.empty() ) {
		for(int i = 0; i < Net_paths_solved.size(); i++) {
			TotalLength_net += CalculateCost(Net_paths_solved[i], direction_mode);
		}
	}
	
	float TotalNodesExp_net = 0;
	
	if( !Net_expanded.empty() ) {
		for(int i = 0; i < Net_expanded.size(); i++) {
		
			TotalNodesExp_net += Net_expanded[i];

		}
	}
	
	float AverageTime_net = TotalTime_net / TotalPaths;	

	float AverageLength_net = TotalLength_net / TotalPaths;

	float AverageNodesExp_net = TotalNodesExp_net / TotalPaths;
	
	float MeanLength_astar = TotalLength_astar / TotalPaths;
	
	float AverageTime_astar = TotalTime_astar / TotalPaths;

	float AverageNodesExp_astar = TotalNodesExp_astar / TotalPaths; 

	printf("ONLINE TEST: comparison with net: A* PERFORMANCE: Total paths = %.0f  Astar_unsolved = %d Astar_total_length = %.6f  Astar_mean_leangth = %.6f "
	       " Astar_total_time = %.6f  Astar_mean_time = %.6f  \n  Astar_mean_nodes_expanded = %.6f \n ",
		 TotalPaths, OnlineSet.unsolved, TotalLength_astar, 
		 MeanLength_astar, TotalTime_astar, AverageTime_astar, AverageNodesExp_astar);

	FILE * online_test = fopen(filename.c_str(), "a");
	if( online_test == NULL ) {
		cout << "Plot file opening failed.\n";
		exit(1);
	}

	int print_check1 = fprintf(online_test, "ONLINE TEST: A* PERFORMANCE comparison with net: \n Total_paths = %.0f \n Astar_unsolved: %d \n"
						" Astar_total_length = %.6f \n Astar_mean_leangth = %.6f \n"
						" Astar_total_time = %.6f \n Astar_mean_time = %.6f  \n "
						" Astar_mean_nodes_expanded = %.6f \n ",
						  TotalPaths, OnlineSet.unsolved, 
						  TotalLength_astar, MeanLength_astar,
						  TotalTime_astar, AverageTime_astar,
						  AverageNodesExp_astar);

	printf("ONLINE TEST: NETWORK PERFORMANCE: Total paths: %.0f  Unsolved: %d "
						     "Total length = %.6f Mean leangth: %.6f "
						     "Total Time: %.6f Mean time: %.6f  \n "
						     " Average nodes expanded = %.6f \n ",
						      TotalPaths, unsolved_net,
						      TotalLength_net, AverageLength_net,					 
						      TotalTime_net, AverageTime_net,
						      AverageNodesExp_net);

	int print_check2 = fprintf(online_test, "ONLINE TEST: NETWORK PERFORMANCE: %s \n Total_paths = %.0f \n Net_unsolved = %d \n"
						" Net_total_length = %.6f \n Net_mean_length = %.6f \n" 
						" Net_total_time = %.6f \n Net_mean_time = %.6f \n "
						" Net_mean_nodes_expanded = %.6f \n ",
						 online_net->name().c_str(), TotalPaths, unsolved_net,
						 TotalLength_net, AverageLength_net, 
						 TotalTime_net, AverageTime_net,
						 AverageNodesExp_net);


	
	if(print_check2 <= 0) {
	   printf("File: writetofile() failed\n");
	   exit(1);
	}
	fclose(online_test);

};


bool TrainValidateRNN::take_free_rand(Map& map, Pixel& last_pos, int timeout, Pixel& new_point) {

	int attempts = 0;
	Pixel new_pix(0,0);

	srand(time(NULL));

	while(attempts < timeout) {

		int rand_dir = rand() % direction.size();
		new_pix = last_pos + direction[rand_dir];
		cout << "Random dir intervention: " << rand_dir << endl; 
		if ( !OutOfBounds(map, new_pix) && !detectCollision(map.coord[new_pix.x][new_pix.y]) ) {
			break;
		}

		attempts++;

	}

	new_point = new_pix;
	
	if( attempts == timeout ) 
		return false;
	else
		return true;

	cout << "Exiting intervention"<< endl;
	getchar();
	
};


void TrainValidateRNN::PrintPathMod(CoordinateList& PathFound, Map& map, string& filename, char symbol) {


	ofstream temp;
	temp.open(filename.c_str());
	if( temp.fail() ) {
		cout << "Print Map to file opening failed.\n";
		exit(1);
	}
	
	int i,j;
	for(j=0; j<map.height; j++) {
		for(i=0; i<map.width; i++) {
			int index = find_node_coord(i, j, PathFound); 
			if ( index != PathFound.size() ) {
				if( index == 0 )
					temp << 'S';
				else
					temp << symbol;
			}
			else
			{
				temp << map.coord[i][j].sym;
			}
		}
		temp << endl;
	}
	
	temp.close();
	


};





} // namespace NeuralNetwork

