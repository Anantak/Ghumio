/** Dynamic Scene implementation
 */

#include "Viewer/dynamic_scene.h"
#include "DataQueue/message_file_reader.h"

/** Eigen includes */
#include <Eigen/Eigen>

namespace anantak {

// Scene properties
//Scene::Scene::Properties():
//  scene_radius(1.)
//{}

DynamicScene::DynamicScene(std::string& programs_setup_filename, std::string& component_name):
    properties_(), camera_(1)
{
  Create(programs_setup_filename, component_name);
}

DynamicScene::~DynamicScene() {}

bool DynamicScene::Create(std::string& programs_setup_filename, std::string& component_name) {
  
  //std::string programs_setup_filename = "src/test/test_node.cfg";
  //std::string component_name = "BeaconDisplay";
  
  // Build the component
  std::unique_ptr<anantak::Component> component_ptr(
      new anantak::Component(programs_setup_filename, component_name));
  component_ = std::move(component_ptr);
  
  // Check if component was created alright
  if (!component_->is_initiated()) {
    LOG(FATAL) << "Could not initiate component for " << component_name << ". Exiting.";
  }

  // Clear out the queue of the component
  component_->ReadMessages();
  
  // Load camera from file
  std::string camera_calibration_filename = "data/camera1_state.pb.data";
  if (!camera_.LoadFromFile(camera_calibration_filename)) {
    LOG(ERROR) << "Could not load camera";
    return false;
  }
  LOG(INFO) << "Loaded camera: " << camera_.ToString(1);
  
  // Create the camera display
  camera_display_.SetCamera(camera_);
  
  return true;
}

bool DynamicScene::Update() {
  
  if (component_) {
    component_->ReadMessages();
    if (component_->Observations().size() > 0) {
      const anantak::SensorMsg& sensor_msg = component_->Observations().back();
      VLOG(2) << "Got a " << sensor_msg.header().type() << " message";
      target_pose_.SetZero();
      target_pose_.Create(sensor_msg);
      VLOG(1) << "Target state = " << target_pose_.ToString(2);
      target_pose_display_.SetPose(target_pose_);
    }
  }
  
  return true;
}

//bool Scene::LoadPosesFromFile(const std::string& filename) {
//  std::vector<anantak::SensorMsg> msgs;
//  anantak::MessageFileReader file_reader;
//  if (!file_reader.LoadMessagesFromFile(filename, &msgs)) {
//    LOG(ERROR) << "Could not load messages from file " << filename;
//    return false;
//  }
//  // Add kinematic states
//  for (int i=0; i<msgs.size(); i++) {
//    anantak::KinematicState kin;
//    if (kin.Create(msgs[i])) {
//      scene_poses_.emplace_back();
//      scene_poses_.back().SetPose(kin);
//    }
//  }
//  // Add pose states
//  for (int i=0; i<msgs.size(); i++) {
//    anantak::PoseState pose;
//    if (pose.Create(msgs[i])) {
//      scene_poses_.emplace_back();
//      scene_poses_.back().SetPose(pose);
//    }
//  }
//  
//  // Calculate scene radius from loaded poses
//  CalculateSceneRadius();
//  
//  return true;
//}

//bool Scene::CalculateSceneRadius() {
//  float max_radius = 0.;
//  for (int i=0; i<scene_poses_.size(); i++) {
//    float radius = scene_poses_[i].p_.norm(); //VLOG(1) << "radius = " << radius;
//    max_radius = std::max(max_radius, radius);
//  }
//  properties_.scene_radius = max_radius;
//  return true;
//}

//bool Scene::Create() {
//  
//  // Create a pose
//  anantak::PoseState pose;
//  
//  Eigen::Vector3d axis; axis << 0, 0, 1;
//  double angle = 30.*kRadiansPerDegree;
//  Eigen::AngleAxisd aa(angle, axis.normalized());
//  Eigen::Quaterniond q(aa);
//  Eigen::Vector3d p; p << 0.,0.,0.;
//  pose.SetPose(q.conjugate(),p);
//  
//  Eigen::Matrix3d qcov; qcov = 10*kRadiansPerDegree *10*kRadiansPerDegree * Eigen::Matrix3d::Identity();
//  qcov(0,1) = qcov(1,0) = 10*kRadiansPerDegree *10*kRadiansPerDegree * 0.5;
//  Eigen::Matrix3d pcov; pcov = 0.05 * Eigen::Matrix3d::Identity();
//  pcov(0,1) = pcov(1,0) = 0.05*0.5;
//  Eigen::Matrix<double,6,6> cov; cov.setZero();
//  cov.block<3,3>(0,0) = qcov;
//  cov.block<3,3>(3,3) = pcov;
//  pose.SetCovariance(cov);
//  
//  // Create a kinematic pose
//  Eigen::Vector3d d_p; d_p << -1., 1., 0.1;
//  
//  anantak::KinematicState kin;
//  kin.SetPose(q.conjugate(), p + d_p);
//  kin.SetPoseCovariance(cov);
//  kin.v_ << .2,.2,.2;
//  kin.w_ << .0,.0,.3;
//  Eigen::Matrix<double,6,6> v_cov; v_cov.setZero();
//  v_cov = cov * 0.04;
//  kin.SetVelocityCovariance(v_cov);
//  
//  // Create a display
//  scene_poses_.emplace_back();
//  scene_poses_.back().SetPose(pose);
//  scene_poses_.emplace_back();
//  scene_poses_.back().SetPose(kin);
//  
//  VLOG(1) << scene_poses_[0].ToString(2);
//  VLOG(1) << scene_poses_[1].ToString(2);
//  
//  return true;
//}

}
