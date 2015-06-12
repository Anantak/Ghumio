/**
 * Beacon model
 * Models the beacon motion
 * Uses historical data
 */

// Anantak includes
#include "Models/beacon_model.h"

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

  // Filenames
  std::string project_root_dir = anantak::GetProjectSourceDirectory() + "/";
  std::string plots_dir = project_root_dir + "src/Models/Plots/";

  std::vector<std::string> msgs_filenames {
    "data/beacon_tags.pb.data",
    "data/pixy_cam00.pb.data",
    //"data/pixy_cam01.pb.data",
    //"data/pixy_cam02.pb.data",
    //"data/beacon_imu.pb.data",
  };
  
  // Open msg files
  anantak::FileMessagesKeeper file_msgs_keeper(msgs_filenames, false);
  if (!file_msgs_keeper.LoadAllMsgsFromFiles()) {
    LOG(ERROR) << "Could not load data, exiting.";
    return -1;
  }
  
  int64_t iteration_interval = 1000000; // microsec
  
  // Setup memory to get new messages
  std::vector<std::unique_ptr<std::vector<anantak::SensorMsg>>> new_msgs;
  file_msgs_keeper.AllocateMemoryForNewMessages(500, &new_msgs);
  
  // Messages counter
  std::vector<int32_t> num_msgs;
  num_msgs.resize(msgs_filenames.size(), 0);
  
  // Create the Beacon filter
  const std::string beacon_filter_config_filename_ = "config/BeaconCameraCalibrator.Camera00.Intrinsics.cfg";
  anantak::BeaconFilter::Options beacon_filter_options_(beacon_filter_config_filename_);
  VLOG(1) << "Beacon filter options = \n" << beacon_filter_options_.ToString();
  anantak::BeaconFilter beacon_filter_(beacon_filter_options_,
      //1433182883550000
      //1433182885500000
      file_msgs_keeper.DataStartTime()  // Given as we running on historical data. If live, remove.
  );
  
  for (int i_iter=0; i_iter<120; i_iter++) {    // 120 max
    // Fetch new messages
    file_msgs_keeper.FetchNewMessages(iteration_interval, &new_msgs);
    
    // Report the number of messages recieved
    std::cout << "iteration " << i_iter << ": Messages ";
    for (int i_ds=0; i_ds<msgs_filenames.size(); i_ds++) {
      num_msgs[i_ds] += new_msgs[i_ds]->size();
      std::cout << new_msgs[i_ds]->size() << " ";
    }
    std::cout << "  ";
    for (int i_ds=0; i_ds<msgs_filenames.size(); i_ds++) {
      std::cout << num_msgs[i_ds] << " ";
    }
    std::cout << "\n";
    
    // Send new messages to filter
    //beacon_filter_.ProcessMessages(file_msgs_keeper.CurrentDataTime(), new_msgs);
    int64_t iteration_end_ts = file_msgs_keeper.CurrentDataTime();
    
    if (!beacon_filter_.StartIteration(iteration_end_ts)) {
      LOG(ERROR) << "Could not start iteration";
    }
    
    if (!beacon_filter_.CreateIterationStates(iteration_end_ts)) {
      LOG(ERROR) << "Could not create states for the iteration";
    }
    
    if (i_iter>=0) {
      if (!beacon_filter_.CreateIterationResiduals(new_msgs)) {
        LOG(ERROR) << "Could not create residuals for the iteration";
      }
    }
    
    if (!beacon_filter_.RunFiltering(iteration_end_ts)) {
      LOG(ERROR) << "Could not run filtering operations";
    }
    
  }
  
  // Report filter states

  
}
