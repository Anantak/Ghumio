/** Scene implementation
 */

#include "Viewer/scene.h"
#include "DataQueue/message_file_reader.h"

/** Eigen includes */
#include <Eigen/Eigen>

namespace anantak {

Scene::Scene() {}

Scene::~Scene() {}

bool Scene::LoadPosesFromFile(const std::string& filename) {
  std::vector<anantak::SensorMsg> msgs;
  anantak::MessageFileReader file_reader;
  if (!file_reader.LoadMessagesFromFile(filename, &msgs)) {
    LOG(ERROR) << "Could not load messages from file " << filename;
    return false;
  }
  // Convert messages to kinematic states
  for (int i=0; i<msgs.size(); i++) {
    anantak::KinematicState kin;
    if (kin.Create(msgs[i])) {
      scene_poses_.emplace_back();
      scene_poses_.back().SetPose(kin);
    }
  }
  return true;
}

bool Scene::Create() {
  
  // Create a pose
  anantak::PoseState pose;
  
  Eigen::Vector3d axis; axis << 0, 0, 1;
  double angle = 30.*kRadiansPerDegree;
  Eigen::AngleAxisd aa(angle, axis.normalized());
  Eigen::Quaterniond q(aa);
  Eigen::Vector3d p; p << 0.,0.,0.;
  pose.SetPose(q.conjugate(),p);
  
  Eigen::Matrix3d qcov; qcov = 10*kRadiansPerDegree *10*kRadiansPerDegree * Eigen::Matrix3d::Identity();
  qcov(0,1) = qcov(1,0) = 10*kRadiansPerDegree *10*kRadiansPerDegree * 0.5;
  Eigen::Matrix3d pcov; pcov = 0.05 * Eigen::Matrix3d::Identity();
  pcov(0,1) = pcov(1,0) = 0.05*0.5;
  Eigen::Matrix<double,6,6> cov; cov.setZero();
  cov.block<3,3>(0,0) = qcov;
  cov.block<3,3>(3,3) = pcov;
  pose.SetCovariance(cov);
  
  // Create a kinematic pose
  Eigen::Vector3d d_p; d_p << -1., 1., 0.1;
  
  anantak::KinematicState kin;
  kin.SetPose(q.conjugate(), p + d_p);
  kin.SetPoseCovariance(cov);
  kin.v_ << .2,.2,.2;
  kin.w_ << .0,.0,.3;
  Eigen::Matrix<double,6,6> v_cov; v_cov.setZero();
  v_cov = cov * 0.04;
  kin.SetVelocityCovariance(v_cov);
  
  // Create a display
  scene_poses_.emplace_back();
  scene_poses_.back().SetPose(pose);
  scene_poses_.emplace_back();
  scene_poses_.back().SetPose(kin);
  
  VLOG(1) << scene_poses_[0].ToString(2);
  VLOG(1) << scene_poses_[1].ToString(2);
  
  return true;
}

}