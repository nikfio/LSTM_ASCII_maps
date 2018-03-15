#include <cstdlib>
#include <string>
#include <iostream>
#include <vector>
#include <cctype>
#include <math.h>
#include <boost/filesystem.hpp>
#include <algorithm>
#include <gflags/gflags.h>
#include <glog/logging.h>


// caffe related
#include <caffe/net.hpp>
#include <caffe/solver.hpp>

#include "Scenario.hpp"
#include "Supervisor.hpp"
#include "Build_set.hpp"
#include "Common_net.cpp"
#include "TrainValidateRNN.hpp"

using namespace std;
using namespace boost::filesystem;
using namespace caffe;
using namespace PathPlanning;
using namespace Supervisor;
using namespace NeuralNetwork;


DEFINE_bool(gpu, true, "Use GPU to brew CAFFE");
DEFINE_bool(train, true, "Execute train and validation");
DEFINE_bool(online, false, "Execute online testing");
DEFINE_bool(resume, false, "Execute training and validation process resuming a trained net .caffemodel");
DEFINE_bool(diagonal, true, "Allow diagonal move");
DEFINE_string(folders, "../movingAI/Random25/Scenarios/,../movingAI/Mazes_2w/Scenarios/,../movingAI/Mazes_4w/Scenarios/,../movingAI/Random40/Scenarios/,../movingAI/DragonAge/Scenarios/", 
			"Path to the folders containing files with .scen extension \n"
			"separated by comma\n");
DEFINE_int32(train_maps, 5, "Number of the maps where paths are picked for the training set");
DEFINE_int32(val_maps, 3, "Number of the maps where paths are picked for the validation set");
DEFINE_int64(train_steps, 1024, "Training set size in terms of total steps");
DEFINE_int64(val_steps, 1024, "Validation set size in terms of total steps");
DEFINE_int32(iter_size, 1, "number of forward - backward pass per train batch (gradients accumulated)");
DEFINE_int32(batch_updates, 1, "number of updates within the batch");
DEFINE_int32(val_freq, 3, "frequency of performing validation test");
DEFINE_int32(epochs, 500, "total number of epochs of the training process");
DEFINE_int32(online_maps, 1, "Number of the maps where paths are picked for the online testing set");
DEFINE_int32(online_paths, 10, "Number of online testing set size in terms of total paths");
DEFINE_string(solver_conf, "../RecurrentNets/LSTM/deep_lstm_solver.prototxt", " .prototxt file configuration of the solver selected");
DEFINE_string(trained, "", " .caffemodel file containing the trained layers of net selected for online testing");
DEFINE_string(online_model, "../RecurrentNets/LSTM/deep_stackout_lstm_online.prototxt", 
			    ".prototxt file of online version of the net - batch_size = 1 (one sample at a time)");
DEFINE_string(log, "../logs/", "directory where to write logs of the test");
DEFINE_int32(max_length, 300, "maximum path length in dataset building");
DEFINE_string(backend, "lmdb", "database backend type");
DEFINE_bool(to_db, false, "build a set and store to a database if true");
DEFINE_int32(seq_size, 10, "input state size");
DEFINE_int32(db_batch_size, 1024, "batch size in storing process");



int main(int argc,char* argv[]) {

gflags::SetVersionString(AS_STRING(CAFFE_VERSION));

gflags::SetUsageMessage("Command line instructions: \n"
			"Usage: ./TestNeuralNetwork <commands> <args> \n\n"
			"commands:\n"
			"--gpu		     Enable GPU mode for training process\n"
			"--train	     Execute train and validation process\n"
			"--online            Execute online testing comparing to other algorithms\n"	
			"--resume	     Resume a trained net and execute training\n"	
			"args:\n"
			"--folders=	      Path to the folder containing files with .scen extension\n"
					           "separated by comma\n"
			"--train_maps= 	      Number of the maps where paths are picked for the training set\n"
			"--validation_maps=   Number of the maps where paths are picked for the validation set\n"
			"--train_steps=       Training set size in terms of total steps\n"
			"--validation_steps=  Validation set size in terms of total steps\n"
			"--iter_size=         number of forward - backward pass batch (gradients accumulated)\n"
			"--batch_updates=     number of updates within the batch\n"
			"--val_freq=	       frequency of performing validation test\n"
			"--epochs=            total number of epochs\n"
			"--online_maps=	      Number of the maps where paths are picked for the online testing set\n"
			"--online_paths=      Number of online testing set size in terms of total paths\n"
			"--trained=  	      .prototxt trained layers\n"
			"--online_model=      .prototxt online model of the ned"
			"--log_file=          logs directory\n"
			"--max_length=        maximum path length in dataset building\n"
			"--backend=		  database backend type \n"
			"--to_db			  build a set and store to a database if true \n "
			"--seq_size=          input state size \n "
			
			"Example for training call:\n"
			"./src/TestNeuralNetwork -folders=""../movingAI/StarCraft/Scenarios/,../movingAI/Mazes_4w/Scenarios/""" 
			"--train_maps=4 --val_maps=4 --train_steps=10000 --val_steps=1000\n");

gflags::ParseCommandLineFlags(&argc, &argv, true);

google::InitGoogleLogging(argv[0]);
FLAGS_alsologtostderr = 1;
FLAGS_log_dir = FLAGS_log;

if( argc > 1 ) {
	gflags::ShowUsageWithFlagsRestrict(argv[0], "src/TestNeuralNetwork");
	exit(EXIT_FAILURE);
}

if( FLAGS_train && FLAGS_resume ) {
	cout << "Not allowed train and resume in a single run, choose one" << endl;
	exit(EXIT_FAILURE);
}


CHECK_GE(FLAGS_seq_size, MIN_STATE_SIZE) << "input state is must be" 
					    << " 8 ranges + distance + rel. angle = minimal_size "
 					    << " (default is  10) meaning that eventual steps tail size = "
					    << " seq_size - minimal_size ";

if( FLAGS_seq_size > MIN_STATE_SIZE ) {
	LOG(INFO) << " steps tail size: " << FLAGS_seq_size - MIN_STATE_SIZE; 
}

vector<string> folders_vec;
size_t pos = 0, comma_loc = 0;
string folder;

while( pos < FLAGS_folders.size() ) {

	comma_loc = FLAGS_folders.find(',', pos+1);
	folder.assign(FLAGS_folders, pos, comma_loc - pos);
	folders_vec.push_back(folder);
	if( comma_loc != string::npos )
		pos = comma_loc+1;
	else
		pos = FLAGS_folders.size();
 
}

vector< boost::filesystem::path > scen_folders; 

for(int i = 0; i < folders_vec.size(); i++) {

	boost::filesystem::path folder_path = folders_vec[i];
	scen_folders.push_back(folder_path);	

}
	
for(int i = 0; i < scen_folders.size(); i++) {

	if( !boost::filesystem::exists(scen_folders[i]) && !boost::filesystem::is_directory(scen_folders[i]) )  {
		LOG(ERROR) << "Not valid Scenarios folder " << i << endl;
		exit(EXIT_FAILURE);
	}	

}


#ifdef DEBUG
char confirm;
cout << "Train scenarios: " << FLAGS_train_maps << " num steps: " << FLAGS_train_steps << endl
     << "Train scenarios: " << FLAGS_val_maps << " num steps: " << FLAGS_val_steps << endl
     << "Confirm? (y/n) ";
cin >> confirm;
     if( confirm == 'n' ) {
	cout << "Aborting by user" << endl;
	exit(1);
     }
#endif


vector< vector<string> > scen_paths;
scen_paths.resize(scen_folders.size());
directory_iterator end_iter;
string scen_ext = ".scen";

for(int i = 0; i < scen_folders.size(); i++) {
	for(directory_iterator itr(scen_folders[i]); itr != directory_iterator(); ++itr) {
		if( is_regular_file(itr->status()) && itr->path().extension().string() == scen_ext ) {
//				cout << itr->path().string() << endl;
				scen_paths[i].push_back(itr->path().string());
		}

	}
}

srand(RAND_SEED);
for(int i = 0; i < scen_folders.size(); i++) {
	random_shuffle(scen_paths[i].begin(), scen_paths[i].end() );
}

vector<string> TrainPaths, ValidatePaths, OnlinePaths;

int total_maps = FLAGS_train_maps + FLAGS_val_maps;


for( int i=0, j = 0, k = 0; i < total_maps; ++i ) {
	
	j = i % scen_paths.size();
	k = i % scen_paths[j].size();

	if( i < FLAGS_train_maps ) {
		TrainPaths.push_back( scen_paths[j][k] );
	}
	else if ( i < total_maps ) {
		ValidatePaths.push_back( scen_paths[j][k] );
	}
	
}

for(int i=0, j=0, k=0; i < FLAGS_online_maps; i++) {

	j = i % scen_folders.size();

	k = rand() % scen_paths[j].size();	

	OnlinePaths.push_back( scen_paths[j][k] );

}

cout << "Maps chosen for training:" << endl;
for(int i=0; i < TrainPaths.size(); i++)
	cout << TrainPaths[i] << endl;

cout << "Maps chosen for validation:" << endl;
for(int i=0; i < ValidatePaths.size(); i++)
	cout << ValidatePaths[i] << endl;

if(FLAGS_online && OnlinePaths.size() > 0 ) {
	cout << "Maps chosen for online testing:" << endl;
	for(int i=0; i < OnlinePaths.size(); i++)
		cout << OnlinePaths[i] << endl;
}
		
ScenarioList TrainList(TrainPaths, TrainPaths.size(), 1000);
ScenarioList ValidateList(ValidatePaths, ValidatePaths.size(), 1000);

setPaths TrainSet, ValidateSet;

string direction_mode;

if(FLAGS_diagonal) {
	LOG(INFO) << "Allowed DIAGONAL directions";
	direction_mode = "diagonal";
}
else {
	LOG(INFO) << "NOT allowed DIAGONAL directions";
	direction_mode = "cardinal";
}

LOG(INFO) << "Paths composing datasets will have maximum " << FLAGS_max_length << " steps";

if(FLAGS_to_db) {

	build_set(TrainList, TrainSet, FLAGS_train_steps, direction_mode);

	build_set(ValidateList, ValidateSet, FLAGS_val_steps, direction_mode);

	TrainVal_sets Sets;
	Sets.TrainSet = TrainSet;
	Sets.ValidateSet = ValidateSet;

	Sets.TrainSize = FLAGS_train_steps;
	Sets.ValidateSize = FLAGS_val_steps;

	TrainValidateRNN TestTrainValidateLSTM( direction_mode,
									Sets,
									FLAGS_seq_size );

}


if(FLAGS_train) {

	/* Lines below launch 
		1) dataset building
		2) training process with runtime dataset from point (1)
     */

//	build_set(TrainList, TrainSet, FLAGS_train_steps, direction_mode);

//	build_set(ValidateList, ValidateSet, FLAGS_val_steps, direction_mode);

//	TrainVal_sets Sets;
//	Sets.TrainSet = TrainSet;
//	Sets.ValidateSet = ValidateSet;

//	Sets.TrainSize = FLAGS_train_steps;
//	Sets.ValidateSize = FLAGS_val_steps;
//	
//	TrainValidateRNN TestTrainValidateLSTM( direction_mode,
//									Sets );
	
	/* Line below launches training process from Dataset */
	TrainValidateRNN Test( direction_mode );


}
	

if(FLAGS_online) {

	

	TrainValidateRNN OnlineTestLSTM( direction_mode,
							   OnlinePaths );

}


return 0;

}
