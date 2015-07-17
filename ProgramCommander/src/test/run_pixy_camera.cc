/* Testing the pixy camera object
 */

// std includes
#include <signal.h>

/** Google libraries */
#include <glog/logging.h>
#include <gflags/gflags.h>

// Anantak includes
#include "PixyBroadcaster/pixy_camera.h"

/** Command line flags */
DEFINE_string(symlink, "/dev/PixyCam1", "Symlink to cam generated using a udev rule e.g. /dev/PixyCam1");

static bool run_flag = true;

void handle_SIGINT(int unused) {
  // On CTRL+C - abort! //
  run_flag = false;
}

int main(int argc, char** argv) {
  
  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);
  // Parse the commandline flags 
  google::ParseCommandLineFlags(&argc, &argv, true);
  // Verify the version of protobuf library 
  //GOOGLE_PROTOBUF_VERIFY_VERSION;
  // Setting the glog output to stderr by default
  FLAGS_logtostderr = 1;
  
  // Catch CTRL+C (SIGINT) signals //
  signal(SIGINT, handle_SIGINT);
  
  std::string symlink = FLAGS_symlink;
  VLOG(1) << "symlink = " << symlink;
  anantak::PixyCamera pixy_camera(symlink);
  if (!pixy_camera.IsInitialized()) {
    LOG(ERROR) << "Pixy camera was not initialized. Exit.";
    return -1;
  }
  
  while (run_flag) {
    pixy_camera.GetNewPixyMessage();
    VLOG(1) << "  Pixy cam message = " << pixy_camera.PixyCameraMessageToString();
  }
  
  VLOG(1) << "Ending pixy camera session.";
  
  return 0;
}
