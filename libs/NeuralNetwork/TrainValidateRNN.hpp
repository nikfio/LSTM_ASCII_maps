/* Header file to define principal functions to perform train and validation
 * of a neural network which properties are defined in the 
 * Neural-Network-model_name.prototxt file, meanwhile train configuration is
 * defined in solver_name.prototxt
 *
*/


/* IMPORTANT ASSUMPTIONS:
 * a sample is an entire path
 * a batch is a set of as many paths as batch_size
 * a step is movement in one of the allowed directions
 * 
 * this means that it is not taken into account if
 * different batches have the same total steps
 */

#ifndef TRAIN_VALIDATE_RNN_H
#define TRAIN_VALIDATE_RNN_H

// C++ related
#include <string>

// caffe related
#include <caffe/net.hpp>
#include <caffe/solver.hpp>
#include <caffe/util/db.hpp>


// project related
#include "Build_set.hpp"
#include "Common_net.hpp"


//#define DEBUG_TEST

#define MAX_STEPS 1000

using namespace caffe;

const int TIME_SEQUENCE = 1000;

const int MIN_STATE_SIZE = 10;

namespace NeuralNetwork
{

class TrainValidateRNN 
{

public:
// SUPERVISED TRAINING
TrainValidateRNN( string& direction_mode,
		 	   TrainVal_sets& sets );

// SUPERVISED TRAINING FROM DATABASE
TrainValidateRNN( string& direction_mode);

// BUILD DATABASE PROCESS
TrainValidateRNN( string& direction_mode,
		 	   TrainVal_sets& sets,
			   int sequence_length );


// ONLINE TESTING 
TrainValidateRNN( string& direction_mode,
		  	   vector<string>& OnlinePaths );	

void OnlineTestSupervisor(string& filename);

void OnlineNetTest(string& direction_mode, string& filename);

private:

bool TRAIN;

int sequence_length;

int train_batch_size;
int validate_batch_size;

CoordinateList direction;
CoordinateList direction_diag;

vector<float> dist_state;

int index = 0;
int step = 0;
int paths_parsed = 0;
int steps_parsed = 0;
int data_index = 0;
int batch = 0;
int online_index = 0;
int prev_step = 0;

vector<float> tail;

boost::shared_ptr<caffe::Blob<float> > blobData_;
boost::shared_ptr<caffe::Blob<float> > blobLabel_;
boost::shared_ptr<caffe::Blob<float> > blobClip_;


boost::shared_ptr<caffe::Solver<float> > solver;
boost::shared_ptr<caffe::Net<float> > net;
boost::shared_ptr<caffe::Net<float> > test_net;
boost::shared_ptr<caffe::Net<float> > online_net;

boost::shared_ptr<caffe::Blob<float> > blobData;
boost::shared_ptr<caffe::Blob<float> > blobClip;
boost::shared_ptr<caffe::Blob<float> > blobLabel;
boost::shared_ptr<caffe::Blob<float> > blobLoss;
boost::shared_ptr<caffe::Blob<float> > blobOut;
boost::shared_ptr<caffe::Blob<float> > blobSoft;
boost::shared_ptr<caffe::Blob<float> > blobAccu;
boost::shared_ptr<caffe::Blob<float> > blobArgmax; 

boost::shared_ptr<caffe::Blob<float> > test_blobData;
boost::shared_ptr<caffe::Blob<float> > test_blobClip;
boost::shared_ptr<caffe::Blob<float> > test_blobLabel;
boost::shared_ptr<caffe::Blob<float> > test_blobLoss;
boost::shared_ptr<caffe::Blob<float> > test_blobOut;
boost::shared_ptr<caffe::Blob<float> > test_blobSoft;
boost::shared_ptr<caffe::Blob<float> > test_blobAccu;
boost::shared_ptr<caffe::Blob<float> > test_blobArgmax;

boost::shared_ptr<caffe::Blob<float> > online_blobData;
boost::shared_ptr<caffe::Blob<float> > online_blobClip;
boost::shared_ptr<caffe::Blob<float> > online_blobLabel;
boost::shared_ptr<caffe::Blob<float> > online_blobLoss;
boost::shared_ptr<caffe::Blob<float> > online_blobOut;
boost::shared_ptr<caffe::Blob<float> > online_blobAccu; 
boost::shared_ptr<caffe::Blob<float> > online_blobArgmax; 

void build_input_batch(int set_batch_num, setPaths& set);

void build_input_sample(Map& map, Pixel& direction_taken, Pixel& current, Pixel& target); 

float TrueDistance(Pixel& current, Pixel& target);

bool take_free_rand(Map& map, Pixel& last_pos, int timeout, Pixel& new_point);

void UpdateTestNet();

void UpdateOnlineNet();

void PrintPathMod(CoordinateList& PathFound, Map& map, string& filename, char symbol);

vector< float > states_tail;

int last_out_pos;
int online_time_sequence;

};



} // namespace NeuralNetwork



#endif

