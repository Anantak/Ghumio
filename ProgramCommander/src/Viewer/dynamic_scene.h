/** Dynamic Scene
 * Holds the objects that are displayed in a dynamic scene.
 * Viewer draws the scene.
 */

#pragma once

#include "ComponentCommander/component.h"
#include "Models/ModelsLib1.h"

namespace anantak {

class DynamicScene {
 public:
  
  // Dynamic Scene properties
  struct Properties {
    float scene_radius;
    Properties():
      scene_radius(5.) {}
  };
  
  DynamicScene::Properties properties_;
  
  // Connection with the Anantak components system
  std::unique_ptr<anantak::Component> component_;
  
  // Dynamic target
  anantak::KinematicState     target_pose_;
  anantak::PoseStateDisplay   target_pose_display_;
  
  // Cameras
  anantak::CameraState          camera_;
  anantak::CameraStateDisplay   camera_display_;
  
  // Default constructor
  DynamicScene(std::string& programs_setup_filename, std::string& component_name);
  
  // Create a scene
  bool Create(std::string& programs_setup_filename, std::string& component_name);
  bool CalculateSceneRadius();
  
  // Update scene
  bool Update();
  
  // Load kinematic poses from a file
  bool LoadMessagesFromFile(const std::string& filename, std::vector<anantak::SensorMsg>* msgs);
  bool LoadPosesFromFile(const std::string& filename);
  
  // Accessors
  float SceneRadius() {return properties_.scene_radius;}
  
  // Destructor - usually everything should self destruct, special cases are destructed here
  virtual ~DynamicScene();
}; // DynamicScene
  

}  // namespace