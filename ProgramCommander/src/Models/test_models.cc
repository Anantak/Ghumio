
/** Google libraries */
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <google/protobuf/message.h>

// Anantak includes
#include "common_config.h"
#include "Utilities/common_functions.h"
#include "Models/ModelsLib1.h"
#include "DataQueue/message_file_writer.h"

using namespace anantak;

std::vector<anantak::KinematicState> kin_states_;

bool TestIntegrations() {
  
  Matrix3d m1;
  Identity3d(m1);
  VLOG(1) << "Identity function returns \n" << m1;
  
  // Create an integrand function from identity function
  std::function<bool(const double&, Matrix3d&)>
      identity_integral = std::bind(Identity3d, std::placeholders::_2);
  Matrix3d m2, m21, m22, m20;
  IntegralFunction<Matrix3d>(identity_integral, 10, m2);
  DoubleIntegralFunction<Matrix3d>(identity_integral, 10, m22, m21, m20);
  VLOG(1) << "Identity function integral returns \n" << m2;
  VLOG(1) << "Identity function double integral returns \n" << m22 << "\n" << m21;
  
  // Create an integrand function from ramp function
  double slope = 0.5;
  std::function<bool(const double&, Matrix3d&)>
      ramp_integral = std::bind(Ramp3d, std::placeholders::_1, slope, std::placeholders::_2);
  Matrix3d m3;
  IntegralFunction<Matrix3d>(ramp_integral, 10., m3);
  VLOG(1) << "Identity function integral returns \n" << m3;
}


bool TestKinematics() {

  Eigen::IOFormat CleanFmt(1, 0, " ", "\n", "", "");
  
  Eigen::Vector3d axis; axis << 0, 0, 1;
  double angle = 30.*kRadiansPerDegree;
  Eigen::AngleAxisd aa(angle, axis.normalized());
  Eigen::Quaterniond q(aa);
  Eigen::Vector3d p; p << 0.,0.,0.;
  
  Eigen::Matrix3d qcov; qcov = 10*kRadiansPerDegree *10*kRadiansPerDegree * Eigen::Matrix3d::Identity();
  qcov(0,1) = qcov(1,0) = 10*kRadiansPerDegree *10*kRadiansPerDegree * 0.5;
  Eigen::Matrix3d pcov; pcov = 0.05 * Eigen::Matrix3d::Identity();
  pcov(0,1) = pcov(1,0) = 0.05*0.5;
  Eigen::Matrix<double,6,6> cov; cov.setZero();
  cov.block<3,3>(0,0) = qcov;
  cov.block<3,3>(3,3) = pcov;
  
  Vector3d v; v << .2,.0,.0;
  Vector3d w; w << .0,.0,.3;
  Eigen::Matrix<double,6,6> v_cov; v_cov.setZero();
  v_cov = cov * 0.04;

  double dv_fac = 0.4;
  Vector3d dv = v * dv_fac;
  Vector3d dw = w * dv_fac;
  Eigen::Matrix<double,6,6> dv_cov; dv_cov.setZero();
  dv_cov = v_cov * dv_fac * dv_fac * 10;
  
  int n_steps = 3;
  const double dt = 1.5;   // seconds
  const double var_dt = 0.;  
  
  // Create a kinematic pose
  kin_states_.resize(n_steps, anantak::KinematicState());
  anantak::KinematicState& kin = kin_states_.at(0);
  kin.SetPose(q.conjugate(), p);
  kin.SetPoseCovariance(cov);
  kin.SetVelocity(w, v);
  kin.SetVelocityCovariance(v_cov);
  kin.SetAcceleration(dw, dv);
  kin.SetAccelerationCovariance(dv_cov);
  VLOG(1) << kin.ToString(2);
  
  
  // Move pose in time
  for (int i=0; i<n_steps-1; i++) {
    const anantak::KinematicState& kin0 = kin_states_.at(i);
    anantak::KinematicState& kin1 = kin_states_.at(i+1);
    kin1.SetTimestamp(kin0.Timestamp()+int64_t(dt*1e6));
    MoveInTime(kin0, dt, var_dt, &kin1);
    VLOG(1) << "Last State " << i << " = " << kin0.ToString(1);
    VLOG(1) << "Calculated state " << i+1 << " = " << kin1.ToString(1);  
    VLOG(1) << "\n kin_start cov = \n" << kin0.covariance_.format(CleanFmt) << "\n\n";
  }
  
  // Move in space
  anantak::KinematicState rel_kin_state;
  Eigen::Vector3d rel_p; rel_p << 0., -2., 0.;
  Eigen::Quaterniond rel_q(Eigen::Quaterniond::Identity());
  rel_kin_state.SetPose(rel_q, rel_p);
  
  for (int i=0; i<n_steps; i++) {
    kin_states_.emplace_back();
    const anantak::KinematicState& kin_start = kin_states_.at(i);
    anantak::KinematicState& kin_move = kin_states_.back();
    kin_move.SetTimestamp(kin_start.Timestamp());
    MoveInSpace(kin_start, rel_kin_state, &kin_move, true);
    VLOG(1) << "\n kin_start cov = \n" << kin_start.covariance_.format(CleanFmt);
    VLOG(1) << "\n kin_move cov = \n" << kin_move.covariance_.format(CleanFmt);
  }
  
  return true;
}

bool SaveStatesToFile(const std::string& filename) {  
  VLOG(2) << "Creating a file writer for " << filename;
  anantak::MessageFileWriter file_writer;
  if (!file_writer.Open(filename)) {
    LOG(ERROR) << "Could not open file. Quitting.";
    return false;
  }
  anantak::SensorMsg sensor_msg;
  int32_t num_msgs_written = 0;
  for (int i=0; i<kin_states_.size(); i++) {
    kin_states_[i].CopyToMessage(&sensor_msg);
    file_writer.WriteMessage(sensor_msg);
    num_msgs_written++;
  }
  LOG(INFO) << "Written " << num_msgs_written << " messages to file " << filename;
  file_writer.Close();
  VLOG(2) << "Closed file " << filename;
  return true;
}


int main(int argc, char** argv) {
  
  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);
  // Parse the commandline flags 
  google::ParseCommandLineFlags(&argc, &argv, true);
  // Verify the version of protobuf library 
  GOOGLE_PROTOBUF_VERIFY_VERSION;
  // Setting the glog output to stderr by default
  FLAGS_logtostderr = 1;
  
  //std::string project_root_dir = anantak::GetProjectSourceDirectory() + "/";
  //std::string data_dir = project_root_dir + "data/";
  //std::string data_dir = "data/";
  
  TestIntegrations();
  TestKinematics();
  SaveStatesToFile("data/test_kinematic_states.pb.data");
  
  return 0;
}