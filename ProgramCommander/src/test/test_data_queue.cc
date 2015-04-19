/**
 *  Test Data Queue 
 *
 *  Build a DataQueue 
 */


/** Google libraries */
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <google/protobuf/message.h>

// Anantak includes
#include "DataQueue/data_queue.h"

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

  anantak::DataQueue data_queue(programs_setup_filename, component_name);
  data_queue.StartTrackingPerformance();
  if (!data_queue.StartLooping()) {
    LOG(ERROR) << "There was an error in starting DataQueue listening and routing.";
  }
  
  return 0;
}
