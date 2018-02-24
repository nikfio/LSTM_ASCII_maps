


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
DECLARE_bool(diagonal);
DECLARE_int32(epochs);
DECLARE_int32(val_freq);
DECLARE_string(solver_conf);				
DECLARE_int32(iter_size);
DECLARE_int32(batch_updates);
DECLARE_int32(max_length);
DECLARE_string(backend);
DECLARE_int64(train_steps);
DECLARE_int64(val_steps);
DECLARE_int32(db_batch_size);
DECLARE_int32(train_maps);
DECLARE_int32(val_maps);



namespace NeuralNetwork{


TrainValidateRNN::TrainValidateRNN( string& direction_mode,
		 	  				 TrainVal_sets& sets,
			   				 int sequence_length ) 

{

	LOG(INFO) << "Database storing specs requested: input state size: " << sequence_length 
			<< "  Maximum Path length: " << FLAGS_max_length;

	setDirections(direction, direction_mode);

	// set the allowed directions 
	string diag = "diagonal";
	setDirections(direction_diag, diag);

	// and set random init
	for( int i = 0; i < tail.size(); i++) {
		tail[i] = rand() % direction.size();
	}

	// allow ranges map in DIAGONAL always
	dist_state.resize(8);

     
	time_t now = time(0);
	tm *local = localtime(&now);

	string train_db_path = "Train_inputs-" + to_string(sequence_length) + "_" + to_string(sets.TrainSize)
							+ "-" + to_string(FLAGS_max_length) 
							+ "-" + to_string(FLAGS_train_maps) 
							+ "-" + to_string(local->tm_mon+1) 
				   			+ "-" + to_string(local->tm_mday) + "-" + to_string(local->tm_hour) 
				  			+ "-" + to_string(local->tm_min)
							+ "-" + FLAGS_backend;
	boost::scoped_ptr<caffe::db::DB> train_database(caffe::db::GetDB(FLAGS_backend));

	train_database->Open(train_db_path, caffe::db::NEW);
	boost::shared_ptr<caffe::db::Transaction> train_txn(train_database->NewTransaction());
	
	
	float label;
	Pixel nextPix;
	Pixel currentDir;
	
	CoordinateList PathFound;

	vector<float> store_data(sequence_length, 0);	

	while( steps_parsed < FLAGS_train_steps ) {
		
		Map map = sets.TrainSet.Maps[sets.TrainSet.map_index[index]]; 
				
		PathFound = sets.TrainSet.OptimalPaths[index];
		Pixel target  = sets.TrainSet.OptimalPaths[index].back(); 

//		string test_db = "test_db" + std::to_string(steps_parsed);
//		PrintPath(PathFound, map, test_db, 'A');
		
//		cout << "Map Index: " << sets.TrainSet.map_index[index] << " path parsed: " << paths_parsed << endl
//			<< " source: " << sets.TrainSet.OptimalPaths[index].front() 
//			<< " target: " << target 
//               << " length: " <<  sets.TrainSet.OptimalPaths[index].size() << endl;
	
		while( step < PathFound.size() - 1  ) {
			
			Pixel current = PathFound[step];
			nextPix = PathFound[step+1] - current;
			label = getDirectionIndex(direction, nextPix);

//			cout << "Step: " << step << " Index data: " << "  current: " << current
//			     << " next: " << PathFound[step % PathFound.size() + 1] 
//				<<  " next dir: " << nextPix << "  target: " << target <<  endl;
			
			DistanceMap(dist_state, map, current, direction_diag);
			
			for(int i = 0; i < dist_state.size(); i++) {
				store_data[i] = dist_state[i];
			}

			store_data[dist_state.size()] = getModDistance(current, target);
			store_data[dist_state.size() + 1] = getRelativeAngle(current, target);
	
			// setting the new tail in the input state
			for(int i = 0; i < tail.size(); i++) {
				
				store_data[dist_state.size() + 2 + i] = tail[i];

			}

//			for(int i = 0; i < store_data.size(); i++) {
//				cout << store_data[i] << " ";
//			}
//			cout <<  "label: " << label << endl;
//			
//			getchar();

			caffe::Datum datum;
			datum.set_channels(sequence_length);
			datum.set_height(1);
			datum.set_width(1);

			for(int i = 0; i < store_data.size(); i++) {
				datum.add_float_data(store_data[i]);
			}
			datum.set_label(label);

			string value;
			datum.SerializeToString(&value);	
			string key_str = caffe::format_int(steps_parsed, 8);
			train_txn->Put(key_str, value);
	
			step++;
			steps_parsed++;

			if( !tail.empty() ) {
				for(int i = tail.size() - 1; i > 0; i-- ) { 
					tail[i] = tail[i-1];
				}
				tail[0] = label;
			}
			
			if(steps_parsed % FLAGS_db_batch_size == 0) {
//				LOG(WARNING) << "Commit " << steps_parsed;
				train_txn->Commit();	
				train_txn.reset(train_database->NewTransaction()); 
			}
			
 			if( steps_parsed == FLAGS_train_steps )
				break;
			
			
		}
	
		index++;
		step = 0;
		paths_parsed++;
	
	
	
	}
	
	if(steps_parsed % FLAGS_db_batch_size){
		train_txn->Commit();	 
	}
	
	if( steps_parsed == FLAGS_train_steps ) {
		LOG(INFO) << "SET FINISHED: paths parsed: " << paths_parsed <<  " steps_parsed: " << steps_parsed << endl;
		index = 0;
		step = 0;
		steps_parsed = 0;
		paths_parsed = 0;
	}
	else {
		LOG(FATAL) << "Validation set: step stored check not valid: " 
				 << steps_parsed << " stored VS "<< FLAGS_train_steps << " required";
	}

	

	string validate_db_path = "Validate_inputs-" + to_string(sequence_length) + "_" + to_string(sets.ValidateSize)
							+ "-" + to_string(FLAGS_max_length) 
							+ "-" + to_string(local->tm_mon+1)
							+ "-" + to_string(FLAGS_val_maps) 
				   			+ "-" + to_string(local->tm_mday) + "-" + to_string(local->tm_hour) 
				  			+ "-" + to_string(local->tm_min)
							+ "-" + FLAGS_backend;
	boost::scoped_ptr<caffe::db::DB> validate_database(caffe::db::GetDB(FLAGS_backend));

	validate_database->Open(validate_db_path, caffe::db::NEW);
	boost::shared_ptr<caffe::db::Transaction> val_txn(validate_database->NewTransaction());

	while( steps_parsed < FLAGS_val_steps ) {
		
		Map map = sets.ValidateSet.Maps[sets.ValidateSet.map_index[index]]; 
				
		PathFound = sets.ValidateSet.OptimalPaths[index];
		Pixel target  = sets.ValidateSet.OptimalPaths[index].back(); 
		
//		cout << "Map Index: " << sets.ValidateSet.map_index[index] << " paths parsed: " << paths_parsed << endl
//			  << " source: " << sets.ValidateSet.OptimalPaths[index].front() << " target: " << target 
//		       << " length: " <<  sets.ValidateSet.OptimalPaths[index].size();
			  
	
		while( step < PathFound.size() - 1 ) {
			
			Pixel current = PathFound[step];
			
			nextPix = PathFound[step+1] - current;
			label = getDirectionIndex(direction, nextPix);
			
//			cout << "Step: " << step << "  current: " << current
//			     << " next: " << PathFound[step % PathFound.size() + 1] <<  " next dir: " << nextPix << endl;		
				
			DistanceMap(dist_state, map, current, direction_diag);
			
			for(int i = 0; i < dist_state.size(); i++) {
				store_data[i] = dist_state[i];
			}

			store_data[dist_state.size()] = getModDistance(current, target);
			store_data[dist_state.size() + 1] = getRelativeAngle(current, target);
		
			// setting the new tail in the input state
			for(int i = 0; i < tail.size(); i++) {
				
				store_data[dist_state.size() + 2 + i] = tail[i];

			}

//			for(int i = 0; i < store_data.size(); i++) {
//				cout << store_data[i] << " ";
//			}
//			cout <<  "label: " << label << endl;
//			
//			getchar();

	
			caffe::Datum datum;
			datum.set_channels(sequence_length);
			datum.set_height(1);
			datum.set_width(1);

			for(int i = 0; i < store_data.size(); i++) {
				datum.add_float_data(store_data[i]);
			}
			datum.set_label(label);

			string value;
			datum.SerializeToString(&value);	
			string key_str = caffe::format_int(steps_parsed, 8);
			val_txn->Put(key_str, value);
			
			step++;
			steps_parsed++;

			if( !tail.empty() ) {
				for(int i = tail.size() - 1; i > 0; i-- ) { 
					tail[i] = tail[i-1];
				}
				tail[0] = label;
			}
			
			if(steps_parsed % FLAGS_db_batch_size == 0) {
//				LOG(WARNING) << "Commit " << steps_parsed;
				val_txn->Commit();	
				val_txn.reset(validate_database->NewTransaction()); 
			}
			
			if( steps_parsed == FLAGS_val_steps )
				break;
			
			
		}
	
		index++;
		step = 0;
		paths_parsed++;
	
	
	
	}
	
	if(steps_parsed % FLAGS_db_batch_size ) {
		val_txn->Commit();	
	}
	
	if( steps_parsed == FLAGS_val_steps ) {
		LOG(INFO) << "SET FINISHED: paths parsed: " << paths_parsed <<  " steps_parsed: " << steps_parsed << endl;
		index = 0;
		step = 0;
		steps_parsed = 0;
		paths_parsed = 0;
	}
	else {
		LOG(FATAL) << "Validation set: step stored check not valid: " 
				 << steps_parsed << " stored VS "<< FLAGS_val_steps << " required";
	}

	
			
			
}


} // namespace neural_network






