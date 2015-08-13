/**
 * Test calibration target motion filter
 * Runs the calibration target motion filter
 * Uses historical data
 */

// Anantak includes
#include "Models/file_messages_keeper.h"
#include "Models/camera_calibration_target_motion_filter.h"
#include "Filter/sliding_window_filter.h"

/* Main function notes:
 * Here we use the filekeeper to open the saved files in the dataset. Using an iteration interval,
 * fresh data is sent over to the filter in a loop.
 */

int main(int argc, char** argv) {
  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);
  // Parse the commandline flags 
  google::ParseCommandLineFlags(&argc, &argv, true);
  // Verify the version of protobuf library 
  GOOGLE_PROTOBUF_VERIFY_VERSION;
  // Setting the glog output to stderr by default
  FLAGS_logtostderr = 1;
  
  
  // Sliding window filter - helps with publish subscribe and config reading
  std::unique_ptr<anantak::SlidingWindowFilter> filter_ = nullptr;
  std::string programs_setup_filename = "src/test/test_node.cfg";
  std::string component_name = "BeaconFilter00";
  
  // Build the sliding window filter component
  std::unique_ptr<anantak::SlidingWindowFilter> filter_ptr(
      new anantak::SlidingWindowFilter(programs_setup_filename, component_name));
  filter_ = std::move(filter_ptr);
  
  // Check if filter was created alright
  if (!filter_->is_initiated()) {
    LOG(FATAL) << "Could not initiate sliding window filter. Exiting.";
    return -1;
  }
  
  
  // Get filter configuration
  anantak::FilterConfig::Model model_config_;
  std::string model_name_ = "CameraCalibrationTargetMotionFilter.Camera00";
  if (!filter_->GetModelConfiguration(model_name_, &model_config_)) {
    LOG(FATAL) << "Model '" << model_name_ << "' was not found in filter configuration";
    return -1;
  }
  
  // Create the filter options
  anantak::CameraCalibrationTargetMotionFilter::Options
      target_motion_filter_options(model_config_.config_file());
  
  // Data storage file name
  std::string project_root_dir = anantak::GetProjectSourceDirectory() + "/";
  std::string plots_dir = project_root_dir + "src/Models/Plots/";
  std::vector<std::string> msgs_filenames {
    "data/beacon_tags.pb.data",
  };
  
  // Messages file reader
  bool run_in_realtime = true;   // false: batch mode, true: realtime mode
  
  // File reader  
  anantak::FileMessagesKeeper file_msgs_keeper(msgs_filenames, run_in_realtime);
  if (!file_msgs_keeper.LoadAllMsgsFromFiles()) {
    LOG(ERROR) << "Could not load data, exiting.";
    return -1;
  }
  
  // Iteration interval from the filter settings
  int64_t iteration_interval = target_motion_filter_options.IterationInterval(); // microsec
  LOG(INFO) << "Setting iteration interval = " << iteration_interval;
  
  // Setup memory to get new messages
  std::vector<std::unique_ptr<std::vector<anantak::SensorMsg>>> new_msgs;
  file_msgs_keeper.AllocateMemoryForNewMessages(500, &new_msgs);
  
  // Messages counter
  std::vector<int32_t> num_msgs;
  num_msgs.resize(msgs_filenames.size(), 0);
  
  // Create the filter
  anantak::CameraCalibrationTargetMotionFilter
      target_motion_filter(target_motion_filter_options, file_msgs_keeper.DataStartTime());
  
  // Timer to wait at end of each iteration when running in realtime mode
  float max_frequency = 0.;    // measure of how fast the filter is running
  anantak::Looper looper(iteration_interval);
  
  // Results message to be transmitted
  anantak::SensorMsg kin_msg_;
  
  //for (int i_iter=0; i_iter<1000; i_iter++) {
  
  int i_iter=0;
  while (file_msgs_keeper.MoreDataLeft()) {
    // Fetch new messages
    file_msgs_keeper.FetchNewMessages(iteration_interval, &new_msgs);
    
    // Report the number of messages recieved
    std::cout << "\x1B[31m" << "iteration " << i_iter << ": Messages ";
    for (int i_ds=0; i_ds<msgs_filenames.size(); i_ds++) {
      num_msgs[i_ds] += new_msgs[i_ds]->size();
      std::cout << new_msgs[i_ds]->size() << " ";
    }
    std::cout << "  Total: ";
    for (int i_ds=0; i_ds<msgs_filenames.size(); i_ds++) {
      std::cout << num_msgs[i_ds] << " ";
    }
    std::cout << "\033[0m" << "\n";
    
    // Send new messages to filter
    int64_t iteration_end_ts = file_msgs_keeper.CurrentDataTime();
    
    // Process the readings
    //    Save data to disk = !realtime, so saving only in batch mode. In realtime, we skip saving
    target_motion_filter.RunIteration(iteration_end_ts, new_msgs, !run_in_realtime);
    
    // Transmit the results message
    if (run_in_realtime) {
      kin_msg_.Clear();
      target_motion_filter.GetResultsMessage(&kin_msg_);
      if (!filter_->PublishMessage(kin_msg_, model_config_.results_subject())) {
        LOG(ERROR) << "Could not publish kinematic pose message on " << model_config_.results_subject();
      } else {
        VLOG(1) << "Published kinematic pose message on " << model_config_.results_subject();        
      }
    }
    
    // Wait iteration interval to complete if running in realtime
    if (run_in_realtime) {
      looper.Sleep();
      max_frequency = looper.max_frequency();
      VLOG(1) << "Filter max frequency = " << max_frequency;
    }
    
    i_iter++;
  }
  
  // Report on filter's operations
  
  // Shut down
  std::cout << "\x1B[31m" << "Shutting down " << "\033[0m" << "\n";
  // Destructors should kick in here
  
} // main

