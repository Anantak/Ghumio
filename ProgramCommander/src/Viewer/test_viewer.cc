/**
 * Taken from SimpleViewer demo application from libQGLViewer
 */

/** Qt include */
#include <qapplication.h>

/** Google libraries */
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <google/protobuf/message.h>

// Anantak includes
#include "common_config.h"
#include "Utilities/common_functions.h"
#include "Viewer/viewer.h"

int main(int argc, char** argv) {
  
  // Read command lines arguments.
  QApplication application(argc, argv);
  
  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);
  // Parse the commandline flags 
  google::ParseCommandLineFlags(&argc, &argv, true);
  // Verify the version of protobuf library 
  GOOGLE_PROTOBUF_VERIFY_VERSION;
  // Setting the glog output to stderr by default
  FLAGS_logtostderr = 1;
  
  // Instantiate a scene to be dispayed
  anantak::Scene scene;
  // Create the scene
  scene.LoadPosesFromFile("data/test_kinematic_states.pb.data");
  
  // Instantiate the viewer
  anantak::Viewer viewer(&scene);  
  viewer.setWindowTitle("Viewer");
  
  // Make the viewer window visible on screen
  viewer.show();
  
  // Run main loop
  return application.exec();
}
