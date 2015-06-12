/**
 *  Test Filter
 *
 *  Build a Filter
 */


/** Google libraries */
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <google/protobuf/message.h>

// Anantak includes
#include "common_config.h"
#include "Utilities/common_functions.h"
#include "Filter/sliding_window_filter.h"

/** Command line flags */
DEFINE_string(name, "", "Name of the component");
DEFINE_string(setup_file, "", "Programs setup file");

int main(int argc, char** argv) {
  
  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);
  // Parse the commandline flags 
  google::ParseCommandLineFlags(&argc, &argv, true);
  // Verify the version of protobuf library 
  GOOGLE_PROTOBUF_VERIFY_VERSION;
  // Setting the glog output to stderr by default
  FLAGS_logtostderr = 1;
  
  std::string programs_setup_filename = FLAGS_setup_file;
  std::string component_name = FLAGS_name;
  VLOG(1) << "Setup file = " << programs_setup_filename;
  VLOG(1) << "Component name = " << component_name;

  anantak::SlidingWindowFilter filter(programs_setup_filename, component_name);
  
  // Check if filter was created alright
  if (!filter.is_initiated()) {
    LOG(ERROR) << "An error occurred, could not initiate filter. Exiting.";
    return -1;
  }
  
  // Start looping
  //if (!filter.StartLooping()) {
  //  LOG(ERROR) << "There was a problem in filter looping. Exiting.";
  //  return -1;
  //}
  
  // Running with live data
  // Iteration begins
  //  Set iteration end timestamp
  //  Ask for new data
  //  Filter - create new states
  //  Wait till the data comes in
  //  When data comes in, process it in the filter
  //  If time left till cycle ends, wait
  //  next iteration
  
  anantak::Looper looper(filter.loop_frequency());
  int64_t iteration_interval = 1000000 / int64_t(filter.loop_frequency());
  VLOG(1) << "Loop frequency and iteration interval = "
      << filter.loop_frequency() << " " << iteration_interval;
  
  while (!filter.exit_loop()) {
    
    // Get current wall time
    int64_t iteration_end_ts = filter.wall_time();
    int64_t next_iteration_begin_ts = iteration_end_ts + iteration_interval;
    VLOG(1) << "Iteration end ts = " << iteration_end_ts;
    
    // Request new data ending at iteration end timestamp
    filter.RequestNewData(iteration_end_ts);
    
    // Create new states
    VLOG(1) << "Creating new states";
    
    // Wait for observations to arrive, put the observations into the queues
    filter.WaitForData(next_iteration_begin_ts);
    
    // Process the observations
    VLOG(1) << "Processing new observations";
    
    // Sleep if needed
    if (!filter.exit_loop()) looper.Sleep();
    filter.set_max_loop_frequency(looper.max_frequency());
  }
  
  return 0;
}
