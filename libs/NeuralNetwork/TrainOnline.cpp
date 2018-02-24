






void TrainValidateRNN::build_input_sample(int set_batch_num, setPaths& set, vector<Scenario>& Archive, CoordinateList& direction) {

	boost::shared_ptr<caffe::Blob<float> > blobData_ = TRAIN ? blobData : test_blobData;
	boost::shared_ptr<caffe::Blob<float> > blobLabel_ = TRAIN ? blobLabel : test_blobLabel;
	boost::shared_ptr<caffe::Blob<float> > blobClip_ = TRAIN ? blobClip : test_blobClip;

	float air_distance, relative_angle, label;
	//vector<float> directionStatus;
	Pixel nextPix;
	Pixel currentDir;
	
	double PathCost;
	string direction_mode;
	if( direction.size() == 8 ) {
		direction_mode = "diagonal";
	}
	else if( direction.size() == 4 ) { 
		direction_mode = "cardinal";
	}

	CoordinateList PathFound;
	//directionStatus.assign( direction.size(), 0);
	bool batch_in = false;
	
	while( steps_parsed == batch_size * set_batch_num ) {
		
		string name = set.scenario_name[index];
		Map map;
		
		int scenario_index = getScenarioFromArchive(Archive, name);
		map = Archive[scenario_index].getMap();
				
		PathFound = set.OptimalPaths[index];
		Pixel target  = set.OptimalPaths[index].back(); 
		//cout << index << "   " << PathFound.size() << endl;
		//cout << " source: " << set.OptimalPaths[index].front() << " target: " << target << " length: " << 			set.OptimalPaths[index].size() << endl;
	
		while( step < PathFound.size() - 1 ) {
			
			Pixel current = PathFound[step];
			nextPix = PathFound[step+1] - current;
			cout << "current: " << current << " next on path: " << PathFound[step+1] << " next: " << nextPix << endl;		
			label = getDirectionIndex(direction, nextPix);
	
			#ifdef DEBUG_LEV_2
			if( label == direction.size() ) {
				cout << "No available direction found, not possible, exiting..." << endl;
				exit(1);
			}
			#endif

			air_distance = euclidean(current, target);
			relative_angle = getRelativeAngle(current, target);

			//AdjacentStatus(directionStatus, current, scen.getMap(), direction);
			
			
			if( step == 0 ) {
				currentDir = PathFound[1] - current;
			}
			else {
				currentDir = current - PathFound[step-1];	
			}

			#ifdef DEBUG_LEV_2
			LocalMap occupancy;
			OccupancyGrid(map, occupancy, current, currentDir, 3, 3);
			string test_path = "test_path";
			PrintPath(PathFound, scen, test_path);		
			vector<float> occupancyStatus = LocalMapToData(occupancy);
			string test_occupancy = "test_occupancy";
			PrintOccupancyGrid(map, occupancy, test_occupancy);
			#endif

			
			vector<float> occupancyStatus = OccupancyData(map, current, currentDir,
									 OCCUPANCY_WIDTH, OCCUPANCY_HEIGHT);	

			// setting the label (supervised output) for the next step
			labels.push_back(label);
		
			for(int i = 0; i < occupancyStatus.size(); i++) {
				data.push_back(occupancyStatus[i]);
			}
			
			/*	 			
			for(int i = 0; i < directionStatus.size(); i++) {
				data.push_back(directionStatus[i]);
			}
			*/
			
			
			data.push_back(air_distance);
			data.push_back(relative_angle);
			// ground truth as input during training, allowed in recurrent nets like LSTM, let's try this way
			//data.push_back(label);
			
			step++;
			steps_parsed++;
			
			//cout << step << "   " << data.size() / sequence_length << "  " << data.size() << endl;
			
//			if( step % TIME_SEQUENCE ) {
//				for( int l = 0; l < sequence_length; l++) {
//					clip.push_back(1);
//				}
//			}
//			else {
//				for( int l = 0; l < sequence_length; l++) {
//					clip.push_back(0);
//				}
//			}

			if( data.size() <= sequence_length) {
				for( int l = 0; l < sequence_length; l++) {
					clip.push_back(0);
				}
			}
			else{
				for( int l = 0; l < sequence_length; l++) {
					clip.push_back(1);
				}
			}



			data_index++;
			
			batch_in = true;
			break;
			
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
	
	
	if( steps_parsed == batch_size * set_batch_num ) {
		//cout << "SET FINISHED: paths parsed: " << paths_parsed <<  " steps_parsed: " << steps_parsed << endl;
		index = 0;
		step = 0;
		steps_parsed = 0;
		paths_parsed = 0;
		batch = 0;
	}
	
	blobData_->set_cpu_data(data.data());
	blobLabel_->set_cpu_data(labels.data()); 
	blobClip_->set_cpu_data(clip.data()); 

};













/* here it is implemented a train method where the network next step
 * is given by the output of the network at the previous step
 * This method is closer to a real (online) training 
 * the next step of training analyze starting from where the network 
 * brought the robot at the previous step
 * if robot goes too far from target, warning is prompted
 */ 
/*
const caffe::SolverParameter & Train_Val_NetOut(string& direction_mode, int num_epochs, int batch_size, int validate_freq, TrainVal_sets& sets, string& solver_conf) {


	CoordinateList direction;
	setDirections(direction, direction_mode);

	boost::shared_ptr<caffe::Solver<float>> solver;
	boost::shared_ptr<caffe::Net<float>> net;

	caffe::SolverParameter solver_param;

	caffe::ReadProtoFromTextFileOrDie(solver_conf, &solver_param);
	solver.reset(caffe::SolverRegistry<float>::CreateSolver(solver_param));

	net = solver->net();

	boost::shared_ptr<caffe::MemoryDataLayer<float>> input =
		boost::dynamic_pointer_cast<caffe::MemoryDataLayer<float>>(net->layer_by_name("input"));

	
	caffe::shared_ptr<caffe::Blob<float>> output_blob = net->blob_by_name("softmax");

	int epoch = 0;
	int validation_test = 0;
	
	int sample, k, l;

	int SHOW_EPOCH_LOG = 1;
	int SHOW_SAMPLE_LOG = 50000;

	float y, y_prob, net_output;
	float loss;

	vector<int> train_indices;
	vector<int> validate_indices;

	for(int i = 0; i<sets.TrainSet.path_index.size(); i++) {
		train_indices.push_back(i);
	}

	for(int i = 0; i< sets.ValidateSet.path_index.size(); i++) {
		validate_indices.push_back(i);
	}

	srand(RAND_SEED);
	random_shuffle(train_indices.begin(), train_indices.end());
	random_shuffle(validate_indices.begin(), validate_indices.end());

	int train_precision = 0; int total_steps = 0; int set_index = 0;
	double percentage = 0;

	for(epoch = 0; epoch < num_epochs; epoch++) { // training and validate		

		for(sample = 0; sample <= train_indices.size() - batch_size; sample += batch_size) { // training epoch 
			
			vector<int> current_indices(train_indices.begin() + sample, train_indices.begin() + sample + batch_size);		
			caffe::Datum datum;
			for(k = 0; k < batch_size; k++) { // training batch
				
				set_index = current_indices[k];
				
				Pixel current_target = sets.TrainSet.OptimalPaths[set_index].back();
				Pixel current = sets.TrainSet.OptimalPaths[set_index].front();
				int steps = 0;
			
				while( current != current_target || steps < escape_num_steps ) { // sample
					
					steps++;
					total_steps++;

					datum = build_Datum(current, sets.TrainSet, set_index, direction); 
				
					if( datum.label() == OUT_OF_BOUNDS_STATE ) {
						#ifdef DEBUG_LEV_2
						cout << "Training: OutOfBounds, starting new path training" << endl;
						#endif
						break;
					}
					else if	( datum.label() == COLLISION_STATE ) {
						#ifdef DEBUG_LEV_2
						cout << "Training: Collision, starting new path training" << endl;
						#endif
						break;
					}	
					else if ( datum.label() == PATH_FINISHED_STATE ) {
						#ifdef DEBUG_LEV_2
						cout << "Training: Path finished, starting new path training" << endl;
						#endif
						break;
					}	
					
					vector<caffe::Datum> datum_vec( 1, datum);

					input->AddDatumVector(datum_vec);
					net->Forward(&loss);
							
					y=0;
					y_prob = -DBL_MAX;			
			
					for(l=0; l < output_blob->channels(); l++) { // get class with highest probability
						net_output = output_blob->cpu_data()[output_blob->channels() + l];
						//cout << net_output << "   " ;

						//printf("(%.3f %3f) ",
						//		output_blob->cpu_data()[k * output_blob->channels() + l],
						//		fc3->cpu_data()[k * output_blob->channels() + l]);

						if (y_prob < net_output)
						{
							y = l;
							y_prob = net_output;
						}
					}	
					//cout << endl;
			
					if( y == datum.label() ) {
						train_precision++;
					}
					
					//cout << "next dir by the network: " << y  << " label: "<< datum.label()<< endl;
					// updating state for the next step with thenetwork output
					current = current + direction[y];
					
			
				} // sample
										
	
			} // training batch 
		
			solver->Step(1);  // Updating the weights for one step
			
			percentage = ((double)train_precision / (double)total_steps) * 100;
			if ( sample % SHOW_SAMPLE_LOG == 0 ) {
				printf("TRAIN EPOCH: %d BATCH: %d TRAIN PRECISION: %d  /  %d  --> %.2f %% \n",
				epoch, sample, train_precision, total_steps, 
				percentage);
			}

			fflush(stdout);
		
			//solver->Step(1);  // Updating the weights for one step
		

		} // training epoch

		percentage = ( (double)train_precision / (double)total_steps ) * 100;
		if ( SHOW_EPOCH_LOG ) {
			printf("REPORT TRAIN EPOCH: %d TRAIN PRECISION: %d  /  %d --> %.2f %% \n",
			epoch, train_precision, total_steps, percentage );
		}

		fflush(stdout);

		if( epoch % validate_freq == 0 ) {
			validation_test++;
			int validate_steps = 0;
			int validate_precision = 0;

			for(sample = 0; sample <= validate_indices.size() - batch_size; sample += batch_size) { //  validation 

				vector<int> current_indices(validate_indices.begin() + sample, validate_indices.begin() + sample + batch_size);				

				for(k = 0; k < batch_size; k++) { // validation batch

			
					set_index = current_indices[k];
					Pixel current_target = sets.ValidateSet.OptimalPaths[set_index].back();
					Pixel current = sets.ValidateSet.OptimalPaths[set_index].front(); 
					int steps = 0;
			
					while( current != current_target || steps < escape_num_steps ) { // sample

						steps++;		
						validate_steps++;
	
						caffe::Datum datum = build_Datum(current, sets.ValidateSet, set_index,
 direction); 
						if( datum.label() == OUT_OF_BOUNDS_STATE ) {
							#ifdef DEBUG_LEV_2
							cout << "Training: OutOfBounds, starting new path training" << endl;
							#endif
							break;
						}
						else if	( datum.label() == COLLISION_STATE ) {
							#ifdef DEBUG_LEV_2
							cout << "Training: Collision, starting new path training" << endl;
							#endif
							break;
						}	
						else if ( datum.label() == PATH_FINISHED_STATE ) {
							#ifdef DEBUG_LEV_2
							cout << "Training: Path finished, starting new path training" << endl;	
							#endif
							break;
						}	
			
						vector<caffe::Datum> datum_vec( 1, datum);	
	
						input->AddDatumVector(datum_vec);
						net->Forward(&loss);
								
						y=0;
						y_prob = -DBL_MAX;			
				
						for(l=0; l < output_blob->channels(); l++) { // get class with highest probability
			
							net_output = output_blob->cpu_data()[output_blob->channels() + l];

							//printf("(%.3f %3f) ",
							//		output_blob->cpu_data()[k * output_blob->channels() + l],
							//		fc3->cpu_data()[k * output_blob->channels() + l]);
	
							if (y_prob < net_output) {
								y = l;
								y_prob = net_output;
							}
						}	
			
			
						if( y == datum.label() ) {
							validate_precision++;
						}
	
						// updating state for the next step with thenetwork output
						current = current + direction[y];
			
					 } // sample	
			
			} //  validation batch

		   } // validation

		   percentage = ((double)validate_precision / (double)validate_steps) * 100;
		   printf("REPORT VALIDATION TEST: %d VALIDATE PRECISION: %d  /  %d --> %.2f %% \n",
			       validation_test, validate_precision, validate_steps, 
				percentage );
	    }


	} // end of training and validate

	return solver->param();

};
*/



/* Online Training and Validating workin on one sample per iteration, but still doubts on its feasibility */
void TrainValidateRNN::TrainValidateRNN_online(string& direction_mode, int num_epochs, int batch_sample_size, int validate_freq,
	    	  TrainVal_sets& sets, string& solver_conf, vector<Scenario>& TrainArchive,
	          vector<Scenario>& ValidateArchive, caffe::SolverParameter solver_param,
	          vector<string>& OnlinePaths, int num_online_paths)  
{
	cout << OnlinePaths.size() << endl;
	getchar();
	ScenarioList OnlineList(OnlinePaths, OnlinePaths.size());
	
	build_benchmarking_set(OnlineList, OnlineSet, num_online_paths, direction_mode);

	cout << OnlineSet.length.size() << endl;
	getchar();

	Caffe::set_mode(Caffe::CPU);

	
	setDirections(direction, direction_mode);

	caffe::ReadProtoFromTextFileOrDie(solver_conf, &solver_param);
	solver.reset(caffe::SolverRegistry<float>::CreateSolver(solver_param));

	net = solver->net();

	blobData = net->blob_by_name("data");
	blobClip = net->blob_by_name("clip");
	blobLabel = net->blob_by_name("label");
	blobOutput = net->blob_by_name("softmax");
	outLayer = net->blob_by_name("out");
	blobLoss = net->blob_by_name("loss");
        blobAccu = net->blob_by_name("accuracy");

	batch_size = batch_sample_size;
	
	double drop_ratio1;
	cout << "Enter drop ratio on 1st layer used" ;
	cin >> drop_ratio1;
	cout << "Drop ratio 1st LSTM layer = " << drop_ratio1 << endl;
	getchar();
	string matlab_plot = "Test_LSTM";
	int print_check;
	FILE * plot = fopen(matlab_plot.c_str(), "w");
	if( plot == NULL ) {
		cout << "Plot file opening failed.\n";
		exit(1);
	}
	fprintf(plot, "NETWORK NAME: %s \n", net->name().c_str());
	print_check = fprintf(plot, "TEST PARAMETERS: \n batch_size = %d \n base_learning_rate = %.5f \n dropout_ratio1 = %.1f \n weight_decay = %f \n loss_data = [ \n",
					 batch_size, solver->param().base_lr(), drop_ratio1, solver->param().weight_decay());
	
	if(print_check <= 0) {
	   printf("File: writetofile() failed\n");
	   exit(1);
	}
	fclose(plot);
		
	string online_test = "online_test";
	OnlineTestSupervisor(online_test);

	// size of the local scope occupancy map plus relative info current node - target node
	sequence_length = OCCUPANCY_WIDTH * OCCUPANCY_HEIGHT + 2;

	int epoch = 0;
	int validation_test = 0;

	int SHOW_EPOCH_LOG = 2;
	int SHOW_BATCH_LOG = 500;

	float y, y_prob, net_output, test_net_output;

	int train_precision = 0;
	int total_steps = 0;
	double train_percentage=0;
	double test_percentage = 0;
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
	
	int train_batch_num = sets.TrainSize / batch_size;
	int validate_batch_num = sets.ValidateSize / batch_size;
	cout << "Train: total length: " << train_total_length << " paths: " << sets.TrainSet.path_index.size()
	     << " batches: " << train_batch_num << endl;
	cout << "Validate: total length: " << val_total_length << " paths: "  << sets.ValidateSet.path_index.size() <<
	     " batches: "<< validate_batch_num << endl;

	for(epoch = 1; epoch <= num_epochs; epoch++) { // training and validate started
		
			TRAIN = true;
			Train_loss = 0.0f;
			Train_accu = 0.0f;
			total_steps = 0;
			train_precision = 0;

			for(int k = 1; k <= train_batch_num; k++) { // training epoch				
			
				build_input_batch(train_batch_num, sets.TrainSet, TrainArchive, direction);
				
//				cout << "TRAINING: BATCH RECEIVED: " << k << " SIZE: " 
//			             << data.size() / sequence_length << " SAMPLES PARSED: " 
//				     << k * data.size() / sequence_length << endl;	
//				getchar();		
				
				solver->Step(1);
			
				#ifdef DEBUG_LEV_2
				y=0;
				y_prob = -DBL_MAX;	
				
				
//				for(int j=0; j < batch_size; j++) { // training batch	
//			
//					for(int l=0; l < blobOutput->channels(); l++) { // get class with highest probability
//						
//						net_output = blobOutput->mutable_cpu_data()[j * blobOutput->channels() + l];
//						 
////						printf("(%.3f %.3f)  ",
////								blobOutput->mutable_cpu_data()[j * blobOutput->channels() + l],
////								outLayer->mutable_cpu_data()[j * outLayer->channels() + l]);

//						if (y_prob < net_output)
//						{
//							y = l;
//							y_prob = net_output;
//						}
//					}

//					if( y == blobLabel->mutable_cpu_data()[j] ) {
//						train_precision++;
//					}

//					total_steps++;
////					cout << total_steps << "  NN out: " << y << " label: "  
////						<< blobLabel->mutable_cpu_data()[j] << endl;
//				
//				} // end of training batch
// 				train_percentage = ((double)train_precision / (double)total_steps) * 100;
				#endif			
			
				Train_loss += blobLoss->mutable_cpu_data()[0]; 
				Train_accu += blobAccu->mutable_cpu_data()[0];
	
//				if ( k % SHOW_BATCH_LOG == 0 ) {
//					printf("TRAIN EPOCH: %d BATCH: %d ACCURACY: %.2f %%  BATCH MEAN LOSS: %.4f  \n",
//					epoch, k, blobAccu->mutable_cpu_data()[0] * 100, blobLoss->mutable_cpu_data()[0]);
//				}
		
				data.clear();
				labels.clear();
				clip.clear();
				
			} // training epoch finished
		
		Train_loss /= train_batch_num;
		Train_accu /= train_batch_num;

		if ( SHOW_EPOCH_LOG ) {
			printf("TRAIN EPOCH: %d ACCURACY: %.4f %%   MEAN LOSS: %.4f  \n",
				epoch, Train_accu * 100, Train_loss);
		}

		plot = fopen(matlab_plot.c_str(), "a");
		if( plot == NULL ) {
			cout << "Plot file opening failed.\n";
			exit(1);
		}
		fprintf(plot, "    %.4f    %.4f", Train_accu * 100, Train_loss); 
		fclose(plot);

		fflush(stdout);


		if( epoch % validate_freq == 0 ) {

			TRAIN = false;
		
			validation_test++;
			int validate_steps = 0;
			int validate_precision = 0;
			Test_loss = 0.0f;
			Test_accu = 0.0f;
			test_percentage = 0;
		
			TestNet();
		
			for(int k=1; k <= validate_batch_num; k++) { // validation test 
				
				build_input_batch(validate_batch_num, sets.ValidateSet, ValidateArchive, direction);
				
				test_net->Forward();		
				
				#ifdef DEBUG_LEV_2
//				for(int j = 0; j < batch_size; j++) { // validation batch

//					y=0;
//					y_prob = -DBL_MAX;
//			
//					for(int l=0; l < test_blobOutput->channels(); l++) { // get class with highest probability
//		
//						test_net_output = test_blobOutput->mutable_cpu_data()[j * test_blobOutput->channels() + l];
//						printf("(%.3f %.3f)  ",
//							test_blobOutput->mutable_cpu_data()[j * test_blobOutput->channels() + l],
//							test_outLayer->mutable_cpu_data()[j * test_blobOutput->channels() + l]);

//						if (y_prob < test_net_output)
//						{
//							y = l;
//							y_prob = test_net_output;
//						}
//					}	
//					cout << endl;
//					validate_steps++;
//					if( y == test_blobLabel->mutable_cpu_data()[j] ) {
//						validate_precision++;
//					}
//					//cout << "NN out: " << y << " label: " << labels[j]  << "   " << test_blobLabel->mutable_cpu_data()[j] << endl;		

//				} // end validation batch
				#endif
				
			 	Test_loss += test_blobLoss->mutable_cpu_data()[0];
				Test_accu += test_blobAccu->mutable_cpu_data()[0];
				
				data.clear();
				clip.clear();
				labels.clear();				
				
			} // end validation test 
			

		Test_loss /=  validate_steps;
		Test_accu /=  validate_steps;

//		test_percentage = ((double)validate_precision / (double)validate_steps) * 100;		
		printf("REPORT VALIDATION TEST: %d ACCURACY: %.4f %%  MEAN LOSS: %.4f   \n",
	         	validation_test, Test_accu * 100, Test_loss );

		OnlineNetTest(direction_mode, online_test);

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














//				for(int j=0; j < batch_size; j++) { // training batch	
//			
//					for(int l=0; l < blobOutput->channels(); l++) { // get class with highest probability
//						
//						net_output = blobOutput->mutable_cpu_data()[j * blobOutput->channels() + l];
//						 
////						printf("(%.3f %.3f)  ",
////								blobOutput->mutable_cpu_data()[j * blobOutput->channels() + l],
////								outLayer->mutable_cpu_data()[j * outLayer->channels() + l]);

//						if (y_prob < net_output)
//						{
//							y = l;
//							y_prob = net_output;
//						}
//					}

//					if( y == blobLabel->mutable_cpu_data()[j] ) {
//						train_precision++;
//					}

//					total_steps++;
////					cout << total_steps << "  NN out: " << y << " label: "  
////						<< blobLabel->mutable_cpu_data()[j] << endl;
//				
//				} // end of training batch
// 				train_percentage = ((double)train_precision / (double)total_steps) * 100;
