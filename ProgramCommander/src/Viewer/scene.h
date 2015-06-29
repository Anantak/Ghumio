/** Scene
 * Holds the objects that are displayed in a scene.
 * Viewer draws the scene.
 */

#pragma once

#include "Models/ModelsLib1.h"

namespace anantak {

class Scene {
 public:
  
  // Objects in the scene
  std::vector<anantak::PoseStateDisplay>  scene_poses_;
  
  // Default constructor
  Scene();
  
  // Create a scene
  bool Create();
  
  // Load kinematic poses from a file
  bool LoadMessagesFromFile(const std::string& filename, std::vector<anantak::SensorMsg>* msgs);
  bool LoadPosesFromFile(const std::string& filename);
  
  // Destructor - usually everything should self destruct, special cases are destructed here
  virtual ~Scene();
}; // Scene
  

}  // namespace