
/** Google libraries */
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <google/protobuf/message.h>

// Anantak includes
#include "common_config.h"
#include "Utilities/common_functions.h"
#include "Models/ModelsLib1.h"
#include "DataQueue/message_file_writer.h"
#include "DataQueue/message_file_reader.h"

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
    Matrix19d jac1; Vector19d jac2;
    //MoveInTime(kin0, dt, var_dt, &kin1, &jac1, &jac2);
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

bool TestCameraCalibration(const std::string& msgs_filename) {
  
  // Read messages from file
  std::vector<anantak::SensorMsg> msgs;
  anantak::MessageFileReader file_reader;
  file_reader.LoadMessagesFromFile(msgs_filename, &msgs);
  
  // Check if there are any messages
  if (msgs.size()<1) {
    LOG(ERROR) << "No messages were loaded from the file " << msgs_filename;
    return false;
  }
  if (!msgs.at(0).has_april_msg()) {
    LOG(ERROR) << "First message does not have an AprilTag message";
    return false;
  }
  if (!msgs.at(0).april_msg().has_camera_num()) {
    LOG(ERROR) << "First message's apriltag message does not have a camera num";
    return false;
  }
  
  // Get the camera number
  int32_t camera_num;
  camera_num = msgs.at(0).april_msg().camera_num();
  VLOG(1) << "First message has camera num = " << camera_num;
  
  // Make sure that all messages are from the same camera
  for (int i=0; i<msgs.size(); i++) {
    if (msgs.at(i).april_msg().camera_num() != camera_num) {
      LOG(WARNING) << "Message " << i << " camera number is not equal to " << camera_num;
    }
  }
  
  // Create target
  anantak::CameraCalibrationTarget
      target("config/camera_intrisics_apriltag_calibration_target.cfg");
  
  // Create camera
  anantak::CameraState camera(camera_num);
  std::vector<int32_t> image_size({640,480});
  
  // Create a calibrator
  CalibrationTargetCameraCalibrator camera_calibrator;
  
  // Starting camera calibration calculation
  camera_calibrator.InitiateCameraCalibration(target, msgs, image_size, &camera);
  
  // Save camera to file
  std::string savefile = "data/camera"+std::to_string(camera_num)+"_state.pb.data";
  camera.SaveToFile(savefile);
  
  return true;
}

bool TestMaths() {
  
  Matrix6x19d mat;
  mat.setZero();
  mat.diagonal().setOnes();
  VLOG(1) << "mat = \n" << mat;
  
  return true;
}

bool TestCubicSpline() {
  
  // Create a cubic pose spline
  int64_t dts = 100000;
  UniformCubicPoseSpline spline(100, dts);
  
  // Create a few control poses
  std::vector<PoseState> control_poses;
  int n=5;
  for (int i=0; i<n; i++) {
    PoseState cp0;
    cp0.SetTimestamp(i*dts + 1000000);
    Vector3d axis0; axis0 << 0., 1., 0.;
    Quaterniond q0(AngleAxisd(double(i)*20*kRadiansPerDegree, axis0.normalized())); // Body rotn
    Vector3d posn0; posn0 << double(i-n/2)*1., double(i-n/2)*.5, double(i-n/2)*.1; // Body posn
    cp0.SetPose(q0.conjugate(), posn0);
    control_poses.emplace_back(cp0);
  }
  
  // Add poses to spline
  for (int i=0; i<control_poses.size(); i++) {
    spline.AddControlPose(control_poses.at(i));    
  }
  
  // Store control poses in file for viewing
  kin_states_.clear();
  for (int i=0; i<control_poses.size(); i++) {
    KinematicState kin;
    kin.SetPose(control_poses.at(i));
    kin_states_.emplace_back(kin);
  }
  
  // Calculate interpolated poses using spline
  for (int i=0; i<8; i++) {
    KinematicState _kin;
    if (spline.InterpolatePose(1000000+3*dts + i*dts/4, &_kin)) {
      VLOG(1) << "Interpolated pose " << i << " " << _kin.ToString(2);
      kin_states_.emplace_back(_kin);
    } else {
      LOG(ERROR) << "Could not interpolate pose using spline";
    }
  }
  
  // Save poses to file
  SaveStatesToFile("data/test_kinematic_states.pb.data");  
}

bool TestCubicSplineInitiation() {
  
  int64_t dts = 100000;
  UniformCubicPoseSpline spline(100, dts);
  
  // Create two poses
  std::vector<PoseState> poses;
  int n=2;
  for (int i=0; i<n; i++) {
    PoseState cp0;
    cp0.SetTimestamp(i*120000 + 10010000);
    Vector3d axis0; axis0 << 0., 1., 0.;
    Quaterniond q0(AngleAxisd(double(i)*20*kRadiansPerDegree, axis0.normalized())); // Body rotn
    Vector3d posn0; posn0 << double(i-n/2)*1., double(i-n/2)*.5, double(i-n/2)*.1; // Body posn
    cp0.SetPose(q0.conjugate(), posn0);
    poses.emplace_back(cp0);
  }
  
  // Initiate spline with first two poses
  spline.InitiateSpline(poses.at(0), poses.at(1));
  // Extend spline
  spline.ExtendSpline(3*120000 + 10010000);
  
  // Store control poses in file for viewing
  kin_states_.clear();
  for (int i=0; i<poses.size(); i++) {
    KinematicState kin;
    kin.SetPose(poses.at(i));
    kin_states_.emplace_back(kin);
  }
  for (int i=0; i<spline.control_poses_->n_msgs(); i++) {
    KinematicState kin;
    kin.SetPose(spline.control_poses_->At(i));
    kin_states_.emplace_back(kin);
  }
  
  // Interpolate poses
  // Calculate interpolated poses using spline
  for (int i=0; i<12; i++) {
    KinematicState _kin;
    if (spline.InterpolatePose(10010000 + i*dts/4, &_kin)) {
      VLOG(1) << "Interpolated pose " << i << " " << _kin.ToString(2);
      kin_states_.emplace_back(_kin);
    } else {
      LOG(ERROR) << "Could not interpolate pose using spline";
    }
  }  
  
  // Save poses to file
  SaveStatesToFile("data/test_kinematic_states.pb.data");  
  
}

// Test interpolation and cubic spline
bool TestInterpolationAndCubicSpline() {
  
  int32_t num_poses = 20;
  double angles_extent = 50.*kRadiansPerDegree; // radians
  double posn_extent = 1; // meters
  int64_t dt = 1000000;
  int32_t num_interpolations_per_interval = 10;
  
  int64_t interpolation_dt = dt / num_interpolations_per_interval;
  kin_states_.clear();
  
  std::vector<PoseState> control_poses;
  
  // Starting pose is identity
  PoseState P0;
  Quaterniond q0; q0.setIdentity();
  Vector3d p0; p0.setZero();
  int64_t t0 = 1000000;
  P0.SetPose(q0, p0);
  P0.SetTimestamp(t0);
  control_poses.emplace_back(P0);
  
  // Generate some random control poses
  for (int i=1; i<num_poses; i++) {
    
    Vector3d d_angles = Vector3d::Random() * angles_extent;
    Vector3d d_posn   = Vector3d::Random() * posn_extent;
    
    VLOG(1) << "dAngles = " << d_angles.transpose();
    VLOG(1) << "dPosn = " << d_posn.transpose();
    
    Quaterniond d_q;
    d_q = AngleAxisd(d_angles(0), Vector3d::UnitX())
        * AngleAxisd(d_angles(1), Vector3d::UnitY())
        * AngleAxisd(d_angles(2), Vector3d::UnitZ());
    
    const PoseState& P_1 = control_poses[i-1];
    
    PoseState P;
    P.SetPose(P_1.Q()*d_q, P_1.P() + d_posn);
    P.SetTimestamp(P_1.Timestamp() + dt);
    control_poses.emplace_back(P);
    
  }
  
  // Store control poses in file for viewing
  if (false) {
    for (int i=0; i<control_poses.size(); i++) {
      KinematicState kin;
      kin.SetPose(control_poses.at(i));
      kin_states_.emplace_back(kin);
    }
  }
  
  // Create a Linear pose spline
  NonUniformLinearPoseSpline linear_spline(num_poses*10);
  for (int i=0; i<num_poses; i++) {
    linear_spline.AddControlPose(control_poses[i]);
  }
  
  
  std::vector<PoseState> interpolated_control_poses;
  
  // Interpolate control poses
  for (int i=0; i<num_poses; i++) {
    for (int j=0; j<num_interpolations_per_interval; j++) {
      int64_t t = control_poses[i].Timestamp() + j*interpolation_dt;
      PoseState P;
      linear_spline.InterpolatePose(t, &P);
      interpolated_control_poses.emplace_back(P);
    }
  }
  
  // Store control poses in file for viewing
  if (false) {
    for (int i=0; i<interpolated_control_poses.size(); i++) {
      KinematicState kin;
      kin.SetPose(interpolated_control_poses.at(i));
      kin_states_.emplace_back(kin);
    }
  }
  
  std::vector<KinematicState> spline_poses;
  
  // Create a Cubic pose spline using the control poses
  UniformCubicPoseSpline spline(num_poses*10, dt);
  
  // Add poses to spline
  for (int i=0; i<control_poses.size(); i++) {
    spline.AddControlPose(control_poses.at(i));    
  }
  
  for (int64_t t=t0+3*dt; t<control_poses.back().Timestamp(); t+=interpolation_dt) {
    KinematicState _kin;
    if (spline.InterpolatePose(t, &_kin)) {
      spline_poses.emplace_back(_kin);
      VLOG(3) << "Interpolated pose " << t << " " << _kin.ToString(2);
    } else {
      LOG(ERROR) << "Could not interpolate pose using spline";
    }
  }
  
  // Store spline poses
  if (true) {
    for (int i=0; i<spline_poses.size(); i++) {
      kin_states_.emplace_back(spline_poses[i]);
    }
  }
  
  // Save poses to file
  SaveStatesToFile("data/test_kinematic_states.pb.data");  
  
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
  
  //TestIntegrations();
  //TestKinematics();
  //SaveStatesToFile("data/test_kinematic_states.pb.data");
  
  TestCameraCalibration("data/camera_calibration_target_observations.pb.data");
  //TestMaths();
  
  //TestCubicSpline();
  //TestCubicSplineInitiation();
  //TestInterpolationAndCubicSpline();
  
  return 0;
}