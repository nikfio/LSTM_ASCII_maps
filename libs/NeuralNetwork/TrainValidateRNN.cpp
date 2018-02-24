/* Implementation file of the functions defined in Train_and_Validate.hpp
 *
*/

// C++ 
#include <string>
#include <vector>
#include <cstdio>
#include <cstdlib>
#include <algorithm>
#include <ctime>


// caffe related
#include <caffe/caffe.hpp>
#include <caffe/solver.hpp>
#include <caffe/solver_factory.hpp>
#include <caffe/sgd_solvers.hpp>
#include <caffe/util/io.hpp>

#include "boost/scoped_ptr.hpp"
#include "caffe/proto/caffe.pb.h"
#include "caffe/util/db.hpp"
#include "caffe/util/rng.hpp"
#include "caffe/util/format.hpp"

// project related
#include "Build_set.hpp"
#include "Common_net.hpp"
#include "TrainValidateRNN.hpp"


using namespace std;
using namespace caffe;

using namespace PathPlanning;
using namespace Supervisor;

DECLARE_bool(gpu);
DECLARE_bool(resume);
DECLARE_bool(diagonal);
DECLARE_int32(epochs);
DECLARE_int32(val_freq);
DECLARE_string(solver_conf);				
DECLARE_int32(iter_size);
DECLARE_int32(batch_updates);
DECLARE_int32(max_length);
DECLARE_int32(seq_size);
DECLARE_int32(steps_tail);
DECLARE_string(trained);

namespace NeuralNetwork
{


void TrainValidateRNN::build_input_batch(int set_batch_num, setPaths& set) {

	blobData_ = TRAIN ? blobData : test_blobData;
	blobLabel_ = TRAIN ? blobLabel : test_blobLabel;
	blobClip_ = TRAIN ? blobClip : test_blobClip;
	int set_batch_size = TRAIN ? train_batch_size : validate_batch_size;
	
	float label;
	Pixel nextPix;
	Pixel currentDir;
	
	CoordinateList PathFound;
	bool batch_in = false;	

	while( batch < set_batch_num ) {

		Map map = set.Maps[set.map_index[index]];
				
		PathFound = set.OptimalPaths[index];
		Pixel target  = set.OptimalPaths[index].back(); 
		
//		cout << "Index: " << index << " source: " << set.OptimalPaths[index].front() << " target: " << target 
//		       << " length: " <<  set.OptimalPaths[index].size()
//			  << " map index: " <<  index % set.Maps.size() << endl;
	
		while( step < PathFound.size() - 1 ) {
			
			Pixel current = PathFound[step];
			
			nextPix = PathFound[step+1] - current;
			label = getDirectionIndex(direction, nextPix);
			
			
//			cout << "Step: " << step << " Index data: " << data_index << "  current: " << current
//			     << " next: " << PathFound[step % PathFound.size() + 1] <<  " next dir: " << nextPix << endl;		

			// setting the label (supervised output) for the next step
			blobLabel_->mutable_cpu_data()[data_index] = label;
				
			DistanceMap(dist_state, map, current, direction_diag);
			
			for(int i = 0; i < dist_state.size(); i++) {
				blobData_->mutable_cpu_data()[data_index * sequence_length + i] = dist_state[i];
			}

			
			blobData_->mutable_cpu_data()[data_index * sequence_length + dist_state.size()] = getModDistance(current, target);
																		
			blobData_->mutable_cpu_data()[data_index * sequence_length + dist_state.size() + 1] = getRelativeAngle(current, target);

			// setting the new tail in the input state
			for(int i = 0; i < tail.size(); i++) {
				
			blobData_->mutable_cpu_data()[data_index * sequence_length + dist_state.size() + 2 + i] = tail[i];

			}

//			for(int i = 0; i < sequence_length; i++) {
//				cout << blobData_->cpu_data()[data_index * sequence_length + i] <<  "  ";
//			}
//			cout << endl;
//			cout << "label: " << label << endl;
//			getchar();

			step++;
			steps_parsed++;
			data_index++;
			
			if( !tail.empty() ) {
				for(int i = tail.size() - 1; i > 0; i-- ) { 
					tail[i] = tail[i-1];
				}
				tail[0] = label;
			}


			if( data_index == set_batch_size ) {
				batch++;
				data_index = 0;
				batch_in = true;
				break;
			}
			
			
		}
	
	if(batch_in) {
		break;
	}
	else {
		index++;
		step = 0;
		paths_parsed++;
	}
	
	
	}
	
	
	if( batch == set_batch_num ) {
//		cout << "SET FINISHED: paths parsed: " << paths_parsed <<  " steps_parsed: " << steps_parsed << endl;
//		getchar();
		index = 0;
		step = 0;
		steps_parsed = 0;
		paths_parsed = 0;
		batch = 0;
	}
	

};


void TrainValidateRNN::UpdateTestNet()
{
	caffe::NetParameter net_param;
	net->ToProto(&net_param);
	net_param.mutable_state()->set_phase(caffe::Phase::TRAIN);
	test_net->CopyTrainedLayersFrom(net_param);	
};

/* here it is implemented a train method where the network next step
 * is given by the next position (Pixel) on the current predefined by 
 * Supervisor optimal path
 * This method is closer to a traditional training:
 * for each batch is built a vector of samples same size as batch_size
 */ 
TrainValidateRNN::TrainValidateRNN( string& direction_mode,
                                    TrainVal_sets& sets ) 
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
	
	 if (FLAGS_gpu) {
    		caffe::Caffe::set_mode(caffe::Caffe::GPU);
		LOG(INFO) << "GPU mode"; 
  	 } else {
    		caffe::Caffe::set_mode(caffe::Caffe::CPU);
		LOG(INFO) << "CPU mode";
  	}

	LOG(INFO) << FLAGS_solver_conf << endl;
	FLAGS_minloglevel = 1;
	SolverParameter solver_param;
	ReadProtoFromTextFileOrDie(FLAGS_solver_conf, &solver_param);
	solver.reset(caffe::SolverRegistry<float>::CreateSolver(solver_param));

	net = solver->net();

	blobData = net->blob_by_name("data");
	blobClip = net->blob_by_name("clip");
	blobLabel = net->blob_by_name("label");
	blobLoss = net->blob_by_name("loss");
     blobAccu = net->blob_by_name("accuracy");
	blobOut  = net->blob_by_name("out");
	blobSoft = net->blob_by_name("softmax");
	blobArgmax = net->blob_by_name("argmax");
	
	test_net = solver->test_nets()[0];

	test_blobData = test_net->blob_by_name("data");
	test_blobClip = test_net->blob_by_name("clip");
	test_blobLabel = test_net->blob_by_name("label");
     test_blobAccu = test_net->blob_by_name("accuracy");	
	test_blobOut  = test_net->blob_by_name("out");
	test_blobSoft = test_net->blob_by_name("softmax");
	test_blobArgmax = test_net->blob_by_name("argmax");

	CHECK_EQ(sequence_length, blobData->shape(1)) << "train net: seq_size and data shape(1) must be equal";
	CHECK_EQ(sequence_length, test_blobData->shape(1)) << "test net: sequence_length and data shape(1) must be equal";

	if(FLAGS_resume) {
		LOG(INFO) << "Copying weights from " << FLAGS_trained;
		net->CopyTrainedLayersFrom(FLAGS_trained);
	}

	train_batch_size = blobData->shape(0);
	validate_batch_size = test_blobData->shape(0);
	
	solver_param.set_iter_size(FLAGS_iter_size);
	
	time_t now = time(0);
	tm *local = localtime(&now);

	const string prefix = "../RecurrentNets/LSTM/" + net->name() + "_" + to_string(local->tm_mon+1) 
				     + "-" + to_string(local->tm_mday) + "-" + to_string(local->tm_hour);

	solver_param.set_snapshot_prefix(prefix);

	FLAGS_minloglevel = 0;
	LOG(INFO) << "Net loaded: " << net->name();
	LOG(INFO) << "TRAIN INFO: batch size: " << train_batch_size;
	LOG(INFO) << "Forward - backward per batch (gradients accumulated): " << FLAGS_iter_size;
	LOG(INFO) << "Number of updates per batch: " << FLAGS_batch_updates;
	LOG(INFO) << "Unrolling for " << TIME_SEQUENCE << " timesteps";
	
	string matlab_plot = "Test_" + net->name() + "_" + to_string(local->tm_mon+1) 
				     + "-" + to_string(local->tm_mday) + "-" + to_string(local->tm_hour) 
				     + "-" + to_string(local->tm_min);
				
	int print_check;
	FILE * plot = fopen(matlab_plot.c_str(), "w");
	if( plot == NULL ) {
		cout << "Plot file opening failed.\n";
		exit(1);
	}
	print_check = fprintf(plot, "TEST: %s DATE: %d %d %d:%d \n" 
				    " PARAMETERS: \n TRAIN_SIZE = %d \n VALID_SIZE = %d \n"
				    " train_set_build_time = %.4f \n" 
				    " val_set_build_time = %.4f \n"
				    " train_batch_size = %d \n iter_size = %d \n batch_updates = %d \n "
				    " base_learning_rate = %.5f \n"
				    " weight_decay = %f \n"
                        " state_input_size = %d \n " 
				    " max_path_length = %d \n "	
				    " loss_data = [ \n",
				     net->name().c_str(), local->tm_mon+1, 
				     local->tm_mday, local->tm_hour, local->tm_min,
				     sets.TrainSize, sets.ValidateSize,
					sets.TrainSet.ElapsedTime,
					sets.ValidateSet.ElapsedTime, 
				     train_batch_size, FLAGS_iter_size, FLAGS_batch_updates, 
				     solver->param().base_lr(),
				     solver->param().weight_decay(),
					sequence_length,
					FLAGS_max_length );
	
	if(print_check <= 0) {
	   printf("File: writetofile() failed\n");
	   exit(1);
	}
	fclose(plot);

	int epoch = 0;
	int validation_test = 0;

	int SHOW_EPOCH_LOG = 1;

	float Train_loss = 0.0f;
	float Test_loss = 0.0f;
	float Train_accu = 0.0f;
	float Test_accu = 0.0f;

	int train_total_length = 0;
	for(int i=0; i<sets.TrainSet.length.size(); i++) {
		train_total_length += sets.TrainSet.length[i];
	}
	
	int val_total_length = 0;
	for(int i=0; i<sets.ValidateSet.length.size(); i++) {
		val_total_length += sets.ValidateSet.length[i];
	}
	
	int train_batch_num = sets.TrainSize / train_batch_size;
	int validate_batch_num = sets.ValidateSize / validate_batch_size;
	LOG(INFO) << "Train: total length: " << train_total_length << " paths: " << sets.TrainSet.OptimalPaths.size()
	     << " batches: " << train_batch_num << endl;
	LOG(INFO) << "Validate: total length: " << val_total_length << " paths: "  << sets.ValidateSet.OptimalPaths.size() <<
	     " batches: "<< validate_batch_num << endl;

	FLAGS_minloglevel = 1;	

	
	// populate clip blobs - fixed time sequence means clip blob is constant 
	for( int i = 0; i < blobClip->shape(0) / sequence_length; i++) {
			
		if( i % TIME_SEQUENCE == 0 ) {
			for(int j = 0; j < sequence_length; j++) {
				blobClip->mutable_cpu_data()[i * sequence_length + j] = 0;
				test_blobClip->mutable_cpu_data()[i * sequence_length + j] = 0;
			}
		}
		else {
			for(int j = 0; j < sequence_length; j++) {
				blobClip->mutable_cpu_data()[i * sequence_length + j] = 1;
				test_blobClip->mutable_cpu_data()[i * sequence_length + j] = 1;
			}
		}
	}

	for(epoch = 1; epoch <= FLAGS_epochs; epoch++) { // training and validate started
		
		TRAIN = true;
		Train_loss = 0.0f;
		Train_accu = 0.0f;
		

		for(int k = 1; k <= train_batch_num; k++) { // training epoch				
			
			build_input_batch(train_batch_num, sets.TrainSet );

			solver->Step(FLAGS_batch_updates);		
			
			Train_loss += blobLoss->cpu_data()[0]; 
			Train_accu += blobAccu->cpu_data()[0];

//			char answer;
//			cout << "TRAINING: Want to check batch output? (y/n)" << endl;
//			cin >> answer;
//			if ( answer == 'y' ) {

//				for(int i = 0; i < train_batch_size; i++) {  

//					printf("Batch %d  sample %d  State input sequence: ", k, i );
//					for(int j = 0; j < sequence_length; j++) {
//						printf(" %.4f  ",  blobData->mutable_cpu_data()[i * sequence_length + j]);
//					}
//					cout << endl;
//					for(int j = 0; j < sequence_length; j++) {
//						printf(" %.0f  ",  blobClip->mutable_cpu_data()[i * sequence_length + j]);
//					}
//					cout << endl;

//					printf("Batch %d  sample %d  NET OUTS: ", k, i);   
//					for(int l=0; l < blobOut->shape(1); l++) { 
//						printf("(%.3f %.3f)  ",
//							blobOut->mutable_cpu_data()[i * blobOut->shape(1) + l],
//							blobSoft->mutable_cpu_data()[i * blobSoft->shape(1) + l]);
//					}
//					cout << endl;

//					printf("Batch %d  sample %d  LABEL OUT: ", k, i); 
//					printf(" %.0f  ", blobLabel->mutable_cpu_data()[i]);

//					cout << endl;

//					printf("Batch %d  sample %d  Current Batch Loss: %.5f  Current Batch Accuracy: %.5f \n", 
//							k, i, blobLoss->mutable_cpu_data()[0], blobAccu->mutable_cpu_data()[0] );
//		
//					cout << "Wanna pass forward? (y/n)" << endl;
//					cin >> answer;

//					if(answer == 'y')
//						break;

//				}
//					

//			}
			

		} // training epoch finished
		
		Train_loss /= train_batch_num;
		Train_accu /= train_batch_num;

		if ( SHOW_EPOCH_LOG ) {
			LOG(WARNING) << "TRAIN EPOCH: " << epoch 
				  << "  ACCURACY: "  << Train_accu * 100 << " %"
 				  << "  MEAN LOSS: " << Train_loss;
		}
		
		plot = fopen(matlab_plot.c_str(), "a");
		if( plot == NULL ) {
			cout << "Plot file opening failed.\n";
			exit(1);
		}
		fprintf(plot, "    %.4f    %.4f", Train_accu * 100, Train_loss); 
		fclose(plot);

		if( epoch % FLAGS_val_freq == 0 ) {

			TRAIN = false;
		
			validation_test++;
			
			Test_loss = 0.0f;
			Test_accu = 0.0f;
			
			UpdateTestNet();
			
			for(int k=1; k <= validate_batch_num; k++) { // validation test 
				
				build_input_batch(validate_batch_num, sets.ValidateSet );	

				test_net->Forward();	

				Test_accu += test_blobAccu->cpu_data()[0];

//				char answer;
//				cout << "VALIDATING: Want to check batch output? (y/n)" << endl;
//				cin >> answer;
//				if ( answer == 'y' ) {

//				for(int i = 0; i < validate_batch_size; i++) {  

//					printf("Batch %d  sample %d  State input sequence: ", k, i );
//					for(int j = 0; j < sequence_length; j++) {
//						printf(" %.4f  ",  test_blobData->mutable_cpu_data()[i * sequence_length + j]);
//					}
//					for(int j = 0; j < sequence_length; j++) {
//						printf(" %.4f  ",  test_blobClip->mutable_cpu_data()[i * sequence_length + j]);
//					}
//					cout << endl;

//					printf("Batch %d  sample %d  NET OUTS: ", k, i);   
//					for(int l=0; l < test_blobOut->shape(0); l++) { 
//		
//						printf("(%.3f %.3f)  ",
//							test_blobOut->mutable_cpu_data()[i * blobOut->shape(1) + l],
//							test_blobSoft->mutable_cpu_data()[i * blobSoft->shape(1) + l]);
//					}
//					cout << endl;

//					printf("Batch %d  sample %d  LABEL OUT: ", k, i); 
//					printf(" %.0f  ", test_blobLabel->mutable_cpu_data()[i]);

//					cout << endl;

//					printf("Batch %d  sample %d  Current Batch Accuracy: %.5f \n", 
//							k, i, test_blobAccu->mutable_cpu_data()[0] );
//		
//					cout << "Wanna pass forward? (y/n)" << endl;
//					cin >> answer;

//					if(answer == 'y')
//						break;

//				}
//					

//				}
				
											
		} // end validation test 
			

		Test_accu /=  validate_batch_num;
		
		LOG(WARNING) << "VALIDATION TEST: " << validation_test 
			  << "  ACCURACY: "  << Test_accu * 100 << " %";

		}
			
		plot = fopen(matlab_plot.c_str(), "a");
		if( plot == NULL ) {
			cout << "Plot file opening failed.\n";
			exit(1);
		}
		fprintf(plot, "    %.4f    %.4f  ; \n", Test_accu * 100, Test_loss); 
		fclose(plot);
		
	} // end of training and validate


};



} // namespace NeuralNetwork

