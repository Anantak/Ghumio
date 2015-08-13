/** Scene
 * Holds the objects that are displayed in a scene.
 * Viewer draws the scene.
 */

#pragma once

#include "Models/ModelsLib1.h"

namespace anantak {

class Scene {
 public:
  
  // Scene properties
  struct Properties {
    float scene_radius;
    Properties():
      scene_radius(1.) {}
  };
  
  Scene::Properties properties_;
  
  // Objects in the scene
  std::vector<anantak::PoseStateDisplay>  scene_poses_;
  
  // Default constructor
  Scene();
  
  // Create a scene
  bool Create();
  bool CalculateSceneRadius();
  
  // Load kinematic poses from a file
  bool LoadMessagesFromFile(const std::string& filename, std::vector<anantak::SensorMsg>* msgs);
  bool LoadPosesFromFile(const std::string& filename);
  
  // Accessors
  float SceneRadius() {return properties_.scene_radius;}
  
  // Destructor - usually everything should self destruct, special cases are destructed here
  virtual ~Scene();
}; // Scene
  

}  // namespace