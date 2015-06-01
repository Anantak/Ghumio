/**
 *  Test Beacon serial monitor
 */

/** Google libraries */
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <google/protobuf/message.h>

// Anantak includes
#include "SerialBroadcaster/front_rangers_monitor.h"

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

  anantak::FrontRangersMonitor front_rangers_monitor(programs_setup_filename, component_name);
  // Check if component was created alright
  if (!front_rangers_monitor.is_initiated()) {
    LOG(ERROR) << "An error occurred, could not initiate front_rangers_monitor. Exiting.";
    return -1;
  }
  // Start looping
  if (!front_rangers_monitor.StartLooping()) {
    LOG(ERROR) << "There was a problem in front_rangers_monitor looping. Exiting.";
    return -1;
  }
  
  // All should self destruct cleanly
  return 0;
}
