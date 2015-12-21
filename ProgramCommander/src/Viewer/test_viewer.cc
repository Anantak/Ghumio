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

DEFINE_string(name, "BeaconDisplay", "Name of the component");
DEFINE_string(setup_file, "src/test/test_node.cfg", "Programs setup file");

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
  
  std::string programs_setup_filename = FLAGS_setup_file;
  std::string component_name = FLAGS_name;
  VLOG(1) << "Setup file = " << programs_setup_filename;
  VLOG(1) << "Component name = " << component_name;

  //// Instantiate a scene to be displayed
  //anantak::Scene scene;
  //// Create the scene
  //scene.LoadPosesFromFile("data/test_kinematic_states.pb.data");
  ////scene.LoadPosesFromFile("data/camera_calibration_target_final_poses.pb.data");
  //// Instantiate the viewer
  //anantak::Viewer viewer(&scene, scene.SceneRadius());  
  //viewer.setWindowTitle("Viewer");
  
  // Instantiate a dynamic scene to be displayed
  anantak::DynamicScene dynamic_scene(programs_setup_filename, component_name);
  // Instantiate the viewer
  anantak::Viewer viewer(&dynamic_scene, dynamic_scene.SceneRadius());  
  viewer.setWindowTitle("Viewer Dynamic Scene");
  
  // Make the viewer window visible on screen
  viewer.show();
  
  // Run main loop
  return application.exec();
}
