/* Models library */

#pragma once

#ifndef ANANTAK_MODELSLIB1_H
#define ANANTAK_MODELSLIB1_H

/** std includes */
#include <vector>
#include <string>
#include <fstream>
#include <algorithm>
#include <memory>
#include <functional>

/** Google libraries */
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <google/protobuf/message.h>

/** Anantak includes */
#include "common_config.h"
#include "Filter/performance_tracker.h"
#include "Filter/circular_queue.h"
#include <Utilities/common_functions.h>

/** Protocol buffers */
#include "sensor_messages.pb.h"
#include "state_messages.pb.h"
#include "configurations.pb.h"

/** Eigen includes */
#include <Eigen/Eigen>

/** opengv */
#include <opengv/relative_pose/methods.hpp>
#include <opengv/relative_pose/CentralRelativeWeightingAdapter.hpp>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/absolute_pose/methods.hpp>
#include <opengv/absolute_pose/CentralAbsoluteAdapter.hpp>

/** ceres includes */
#include <ceres/ceres.h>

namespace anantak {

typedef Eigen::Matrix<double,1,1> Vector1d;
typedef Eigen::Matrix<double,2,1> Vector2d;
typedef Eigen::Matrix<double,3,1> Vector3d;
typedef Eigen::Matrix<double,4,1> Vector4d;
typedef Eigen::Matrix<double,5,1> Vector5d;
typedef Eigen::Matrix<double,6,1> Vector6d;
typedef Eigen::Matrix<double,7,1> Vector7d;
typedef Eigen::Matrix<double,9,1> Vector9d;
typedef Eigen::Matrix<double,18,1> Vector18d;
typedef Eigen::Matrix<double,19,1> Vector19d;
typedef Eigen::Matrix<double,20,1> Vector20d;

typedef Eigen::Matrix<double,1,1> Matrix1d;
typedef Eigen::Matrix<double,2,2> Matrix2d;
typedef Eigen::Matrix<double,3,3> Matrix3d;
typedef Eigen::Matrix<double,4,4> Matrix4d;
typedef Eigen::Matrix<double,5,5> Matrix5d;
typedef Eigen::Matrix<double,6,6> Matrix6d;
typedef Eigen::Matrix<double,9,9> Matrix9d;
typedef Eigen::Matrix<double,18,18> Matrix18d;
typedef Eigen::Matrix<double,19,19> Matrix19d;
typedef Eigen::Matrix<double,20,20> Matrix20d;

typedef Eigen::Matrix<double,1,3> Matrix1x3d;
typedef Eigen::Matrix<double,2,3> Matrix2x3d;
typedef Eigen::Matrix<double,2,4> Matrix2x4d;
typedef Eigen::Matrix<double,3,2> Matrix3x2d;
typedef Eigen::Matrix<double,6,18> Matrix6x18d;
typedef Eigen::Matrix<double,9,3> Matrix9x3d;

typedef Eigen::Map<Eigen::Matrix<double,1,1>> MapVector1d;
typedef Eigen::Map<Eigen::Vector3d> MapVector3d;
typedef Eigen::Map<Eigen::Vector4d> MapVector4d;

typedef Eigen::Map<Eigen::Matrix<double,1,1>> MapMatrix1d;
typedef Eigen::Map<Eigen::Matrix3d> MapMatrix3d;

typedef Eigen::Map<const Eigen::Vector3d> MapConstVector3d;
typedef Eigen::Map<const Eigen::Vector4d> MapConstVector4d;
typedef Eigen::Map<const Eigen::Matrix<double,20,1>> MapConstVector20d;

typedef Eigen::Map<const Eigen::Matrix3d> MapConstMatrix3d;
typedef Eigen::Map<const Eigen::Matrix<double,19,19>> MapConstMatrix19x19d;

typedef Eigen::Quaterniond Quaterniond;


// Templated typdef - starts in gcc 4.7
//template <typename Type> using IntegrandType = std::function<bool(const double&, Type&)>;

/** Global constants */

static const double kPi = 3.14159265358979323846;
static const double kPi_half = kPi*0.5;
static const double kPi_2 = kPi*2.0;
static const double kRadiansPerDegree = kPi/180.0;
static const double kDegreesPerRadian = 180.0/kPi;

static const double kEpsilon = Eigen::NumTraits<double>::epsilon();
static const double kOneLessEpsilon = double(1) - kEpsilon;
static const Matrix3d kIdentity3d(Eigen::Matrix3d::Identity());

/** Skew symmetric matrix from a vector */
inline Eigen::Matrix3d SkewSymm3d(const Vector3d& v) {
  Eigen::Matrix3d mat;
  mat <<     0,  -v[2],   v[1],
          v[2],      0,  -v[0],
         -v[1],   v[0],      0;
  return mat;
}


/** Integration functions - allow integrations via functions created from objects */

/** Integration function - integrates a function using Simpson's rule
 * Takes in a function that returns a matrix from a double value,
 * Lower limit = 0. Upper limit is an input. Returns the integrated matrix value */
template <typename MatrixType>
inline bool IntegralFunction(
    std::function<bool(const double&, MatrixType&)> integrand,
    const double& upper_limit,    // lower limit = 0
    MatrixType& I)
{
  // x0 = 0.; x2 = upper_limit;
  double x1 = upper_limit*0.5;
  MatrixType f0, f1, f2;
  if (!integrand(0., f0)) return false;
  if (!integrand(x1, f1)) return false;
  if (!integrand(upper_limit , f2)) return false;
  I = x1 / 3. * (f0 + 4.*f1 + f2);
  return true;
}

/** Double Integration function - integrates a function twice using Simpson's rule
 * Takes in a function that returns a matrix from a double value,
 * Lower limit = 0. Upper limit is an input.
 * Returns the double integrated and single integrated matrix value */
template <typename MatrixType>
inline bool DoubleIntegralFunction(
    std::function<bool(const double&, MatrixType&)> integrand,
    const double& upper_limit,    // lower limit = 0
    MatrixType& II, MatrixType& I, MatrixType& F)
{
  // x0 = 0.; x4 = upper_limit;
  double x2 = upper_limit*0.5;
  double x1 = x2*0.5;
  double x3 = x2 + x1;
  MatrixType f0, f1, f2, f3, f4;
  if (!integrand(0., f0)) return false;
  if (!integrand(x1, f1)) return false;
  if (!integrand(x2, f2)) return false;
  if (!integrand(x3, f3)) return false;
  if (!integrand(upper_limit , f4)) return false;
  MatrixType I1 = x1 / 3. * (f0 + 4.*f1 + f2);    // Area in first half
  MatrixType I2 = x1 / 3. * (f2 + 4.*f3 + f4);    // Area in second half
  I = I1 + I2;    // Total area
  II = x2 / 3. * (4.*I1 + I); // Integral of area
  F = f4;         // Ending function
  return true;
}

inline bool Identity3d(Matrix3d& m) {
  m = Eigen::Matrix3d::Identity();
  return true;
}

inline bool Ramp3d(const double& x, const double& s, Matrix3d& m) {
  m = Eigen::Matrix3d::Identity()*x*s;
  return true;
}


/** States - used to solve the problems */

/** Pose state
 *  3D pose
 */
class PoseState {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  Vector7d state_;
  Vector6d error_;
  Matrix6d covariance_;
  
  // Mappings
  MapVector4d qv_;  MapVector3d p_;  
  MapVector3d eq_;  MapVector3d ep_;  
  
  // Helpers
  Quaterniond q_;  Matrix3d R_;
  
  PoseState():
    qv_(state_.data()), p_(state_.data()+4), eq_(error_.data()), ep_(error_.data()+3)
  {
    SetZero();
  }
  
  // Default copy constructor
  PoseState(const PoseState& r) :
    qv_(state_.data()), p_(state_.data()+4), eq_(error_.data()), ep_(error_.data()+3)
  {
    state_= r.state_;
    error_ = r.error_;
    covariance_ = r.covariance_;
    q_ = r.q_;
    R_ = r.R_;
  }
  
  // Equal to assignment operator
  PoseState& operator= (const PoseState& r) {
    state_= r.state_;
    error_ = r.error_;
    covariance_ = r.covariance_;
    q_ = r.q_;
    R_ = r.R_;
  }
  
  // Reset
  bool SetZero() {
    state_.setZero();
    state_[3] = 1.;
    error_.setZero();
    covariance_.setZero();
    UpdateRotations();
    return true;
  }
  
  bool UpdateRotations() {
    q_ = Quaterniond(qv_);
    R_ = Matrix3d(q_);
    return true;
  }
  
  bool SetPose(const Eigen::Quaterniond& q, const Eigen::Vector3d& p) {
    qv_ = q.coeffs();
    p_ = p;
    UpdateRotations();    
    return true;
  }
  
  bool SetCovariance(const Eigen::Matrix<double,6,6>& cov, bool calc_inv_sqrt=true) {
    covariance_ = cov;
    // Calculate inverse sqrt matrix
    if (calc_inv_sqrt) {}
    return true;
  }
  
  // Const references
  const MapVector4d&  Qv() const {return qv_;}
  const Quaterniond&  Q() const {return q_;}
  const Matrix3d&     R() const {return R_;}
  const MapVector3d&  P() const {return p_;}
  
  // Copies
  const Matrix3d      VarQ() const {return covariance_.block<3,3>(0,0);}
  const Matrix3d      VarP() const {return covariance_.block<3,3>(3,3);}
  
  // Mutable pointers
  MapVector4d* Qv_ptr() {return &qv_;}
  MapVector3d* P_ptr() {return &p_;}
  
  bool IsZero() const {
    return (state_.isZero());
  }
  
  std::string ToString(int detail = 0) const {
    std::ostringstream ss; ss << "[" << qv_.transpose() << "] [" << p_.transpose() << "]";
    if (detail>0) {ss << "\n Covariace = \n" << covariance_;}
    return ss.str();
  }
  
}; // PoseState


/** Kinematic state - a pose, velocity and acceleration
 *  Capturing correlations is crucial
 */
class KinematicState {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  int64_t       timestamp_;   // t 
  Vector20d     state_;       // q, p, w, v, dw, dv, t
  Vector19d     error_;       // eq, ep, ew, ev, edw, edv, et
  Matrix19d     covariance_;  // eq, ep, ew, ev, edw, edv, et
  
  // Vector mappings
  MapVector4d qv_; MapVector3d w_;  MapVector3d dw_;
  MapVector3d p_;  MapVector3d v_;  MapVector3d dv_;   MapVector1d t_;
  MapVector3d eq_; MapVector3d ew_; MapVector3d edw_;
  MapVector3d ep_; MapVector3d ev_; MapVector3d edv_;  MapVector1d et_;
  
  // Helpers
  Quaterniond q_;  Matrix3d R_;
  
  KinematicState() :
    qv_(state_.data()),     p_(state_.data()+4),
    w_(state_.data()+7),    v_(state_.data()+10),
    dw_(state_.data()+13),  dv_(state_.data()+16),  t_(state_.data()+19),
    eq_(error_.data()),     ep_(error_.data()+3),
    ew_(error_.data()+6),   ev_(error_.data()+9),
    edw_(error_.data()+12), edv_(error_.data()+15), et_(error_.data()+18)
  {
    SetZero();
  }

  // Default copy constructor
  KinematicState(const KinematicState& r) :
    qv_(state_.data()),     p_(state_.data()+4),
    w_(state_.data()+7),    v_(state_.data()+10),
    dw_(state_.data()+13),  dv_(state_.data()+16),  t_(state_.data()+19),
    eq_(error_.data()),     ep_(error_.data()+3),
    ew_(error_.data()+6),   ev_(error_.data()+9),
    edw_(error_.data()+12), edv_(error_.data()+15), et_(error_.data()+18)
  {
    timestamp_ = r.timestamp_;
    state_= r.state_;
    error_ = r.error_;
    covariance_ = r.covariance_;
    q_ = r.q_;
    R_ = r.R_;
  }
  
  // Equal to assignment operator
  KinematicState& operator= (const KinematicState& r) {
    timestamp_ = r.timestamp_;
    state_= r.state_;
    error_ = r.error_;
    covariance_ = r.covariance_;
    q_ = r.q_;
    R_ = r.R_;
  }
  
  // Const references
  const MapVector4d&  Qv() const {return qv_;}
  const Quaterniond&  Q() const {return q_;}
  const Matrix3d&     R() const {return R_;}
  const MapVector3d&  P() const {return p_;}
  const MapVector3d&  W() const {return w_;}
  const MapVector3d&  V() const {return v_;}
  const MapVector3d&  dW() const {return dw_;}
  const MapVector3d&  dV() const {return dv_;}
  const MapVector1d&  T() const {return t_;}
  const int64_t& Timestamp() const {return timestamp_;}
  
  // Copies
  const Matrix3d      VarQ() const {return covariance_.block<3,3>(0,0);}
  const Matrix3d      VarP() const {return covariance_.block<3,3>(3,3);}
  const Matrix3d      VarW() const {return covariance_.block<3,3>(6,6);}
  const Matrix3d      VarV() const {return covariance_.block<3,3>(9,9);}
  const Matrix3d     VardW() const {return covariance_.block<3,3>(12,12);}
  const Matrix3d     VardV() const {return covariance_.block<3,3>(15,15);}
  const Matrix1d      VarT() const {return covariance_.block<1,1>(18,18);}
  
  // Mutable pointers
  MapVector4d* Qv_ptr() {return &qv_;}
  MapVector3d* P_ptr() {return &p_;}
  
  // Reset
  bool SetZero() {
    timestamp_ = 0;
    state_.setZero();
    state_[3] = 1.;
    error_.setZero();
    covariance_.setZero();
    UpdateRotations();
    return true;
  }
  
  bool UpdateRotations() {
    q_ = Quaterniond(qv_);
    R_ = Matrix3d(q_);
    return true;
  }
  
  bool SetTimestamp(const int64_t& tm) {
    timestamp_ = tm;
  }
  
  // State setters
  
  bool SetPose(const Eigen::Quaterniond& q, const Eigen::Vector3d& p) {
    qv_ = q.coeffs();
    p_ = p;
    UpdateRotations();
    return true;
  }
  
  bool SetVelocity(const Eigen::Vector3d& w, const Eigen::Vector3d& v) {
    w_ = w;
    v_ = v;
  }
  
  bool SetAcceleration(const Eigen::Vector3d& dw, const Eigen::Vector3d& dv) {
    dw_ = dw;
    dv_ = dv;
  }
  
  bool SetTime(const double& tm) {
    t_ = Vector1d(tm);
  }
  
  // Covariance setters
  
  bool SetPoseCovariance(const Eigen::Matrix<double,6,6>& cov) {
    covariance_.block<6,6>(0,0) = cov;
    return true;
  }
  
  bool SetVelocityCovariance(const Eigen::Matrix<double,6,6>& cov) {
    covariance_.block<6,6>(6,6) = cov;
    return true;
  }
  
  bool SetAccelerationCovariance(const Eigen::Matrix<double,6,6>& cov) {
    covariance_.block<6,6>(12,12) = cov;
    return true;
  }
  
  bool SetTimeCovariance(const double& cov) {
    covariance_.block<1,1>(18,18) = Vector1d(cov);
    return true;
  }
  
  // Creates a sensor message from this state
  bool CopyToMessage(anantak::SensorMsg* msg) const {
    msg->Clear();
    anantak::HeaderMsg* hdr_msg = msg->mutable_header();
    anantak::KinematicStateMessage* kin_msg = msg->mutable_kinematic_state_msg();
    
    // Build header message
    hdr_msg->set_timestamp(timestamp_);
    hdr_msg->set_type("KinematicState");
    hdr_msg->set_recieve_timestamp(timestamp_);
    hdr_msg->set_send_timestamp(timestamp_);
    
    // Build kinematic state message
    const double* st_data = state_.data();
    for (int i=0; i<state_.size(); i++) kin_msg->add_state(*(st_data+i));
    const double* cov_data = covariance_.data();
    for (int i=0; i<covariance_.size(); i++) kin_msg->add_covariance(*(cov_data+i));
  }
  
  // Creates this state from a sensor message
  bool Create(const anantak::SensorMsg& msg) {
    // Check the message
    if (!msg.has_header()) {LOG(ERROR) << "Msg has no header"; return false;}
    if (!msg.has_kinematic_state_msg()) {LOG(ERROR) << "Msg has no kinematic state"; return false;}
    if (msg.header().type()!="KinematicState") {LOG(ERROR) << "Msg type is not KinematicState"; return false;}
    const anantak::KinematicStateMessage& kin_msg = msg.kinematic_state_msg();
    if (kin_msg.state_size()!=state_.size()) {LOG(ERROR) << "Msg state size != state_.size"; return false;}
    if (kin_msg.covariance_size()!=covariance_.size()) {LOG(ERROR) << "Msg covariance size != covariance size"; return false;}
    
    // All good, proceed to populate the state
    timestamp_ = msg.header().timestamp();
    if (timestamp_<1) {LOG(WARNING) << "Msg timestamp < 1";}
    const MapConstVector20d _state(kin_msg.state().data());
    state_ = _state;
    const MapConstMatrix19x19d _cov(kin_msg.covariance().data());
    covariance_ = _cov;
    error_.setZero();
    UpdateRotations();
    
    return true;
  }
  
  std::string ToString(int detail = 0) const {
    std::ostringstream ss;
    if (detail>0) {ss << "\n pose = \n";}
    ss << qv_.transpose() << ", " << p_.transpose();
    if (detail>0) {ss << "\n velocity = \n" << w_.transpose() << ", " << v_.transpose();}
    if (detail>0) {ss << "\n acceleration = \n" << dw_.transpose() << ", " << dv_.transpose();}
    if (detail>1) {ss << "\n rotation covariance = \n" << VarQ() ;}
    if (detail>1) {ss << "\n position covariance = \n" << VarP() ;}
    return ss.str();
  }
  
  virtual ~KinematicState() {}
  
}; // KinematicState


/** PoseState display - helps in drawing the pose */
class PoseStateDisplay {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  // Information stored here
  bool has_pose_;
  bool has_velocity_;
  bool has_acceleration_;
  
  // Pose data
  Vector4d qv_;
  Matrix3d R_;
  Vector3d p_;
  Matrix6d pose_covariance_;
  
  // Pose transform matrix
  Matrix4d pose_T_;
  
  // Position uncertainty display
  Matrix3d position_cov_eigenvectors_;
  Vector3d position_cov_eigenvalues_;
  Vector3d position_cov_eigenvalues_sqrt_;
  
  // Rotation uncertainty display
  Matrix3d rotation_cov_eigenvectors_;
  Vector3d rotation_cov_eigenvalues_;
  Vector3d rotation_cov_eigenvalues_sqrt_;
  Matrix3d rotation_cov_x_disk_mat_;
  Matrix3d rotation_cov_y_disk_mat_;
  Matrix3d rotation_cov_z_disk_mat_;
  Eigen::Matrix<double,3,Eigen::Dynamic> rotation_cov_x_disk_points_;
  Eigen::Matrix<double,3,Eigen::Dynamic> rotation_cov_y_disk_points_;
  Eigen::Matrix<double,3,Eigen::Dynamic> rotation_cov_z_disk_points_;
  
  // Velocity data
  Vector3d v_;
  Vector3d w_;
  Matrix3d v_covariance_;  
  Matrix3d w_covariance_;
  Matrix3d v_cov_eigenvectors_;
  Vector3d v_cov_eigenvalues_;
  Vector3d v_cov_eigenvalues_sqrt_;
  Matrix3d w_cov_eigenvectors_;
  Vector3d w_cov_eigenvalues_;
  Vector3d w_cov_eigenvalues_sqrt_;
  double v_norm_;
  double w_norm_;
  Matrix4d v_arrow_T_;
  Matrix4d w_arrow_T_;
  Matrix4d v_circles_T_;
  Matrix4d w_circles_T_;
  
  // Default constructor
  PoseStateDisplay():
    has_pose_(false), has_velocity_(false), has_acceleration_(false) {}
  
  bool SetPose(const PoseState& pose) {
    // Copy pose values
    qv_ = pose.qv_;
    R_ = pose.R_;
    p_ = pose.p_;
    pose_covariance_ = pose.covariance_;
    CalculatePoseDiplayParameters();
    has_pose_ = true;
    return true;
  }
  
  bool SetPose(const KinematicState& kin) {
    // Copy pose values
    qv_ = kin.qv_;
    R_ = kin.R_;
    p_ = kin.p_;
    pose_covariance_ = kin.covariance_.block<6,6>(0,0);
    CalculatePoseDiplayParameters();
    
    v_ = kin.V();
    w_ = kin.W();
    v_covariance_ = kin.VarV();
    w_covariance_ = kin.VarW();
    CalculateVelocityDisplayParameters();
    
    has_pose_ = true;
    has_velocity_ = true;
    return true;
  }
  
  bool CalculatePoseDiplayParameters() {
    
    pose_T_.setZero(); pose_T_(3,3) = 1.;
    pose_T_.block<3,3>(0,0) = R_.transpose();   // transposing as we store the conjugate in state
    pose_T_.block<3,1>(0,3) = p_;
    
    // Calculate eigen vector decomposition of covariance matrix
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es;
    es.compute(pose_covariance_.block<3,3>(3,3));
    position_cov_eigenvectors_ = es.eigenvectors();
    position_cov_eigenvalues_ = es.eigenvalues();
    position_cov_eigenvalues_sqrt_ = position_cov_eigenvalues_.cwiseSqrt();
    es.compute(pose_covariance_.block<3,3>(0,0));
    rotation_cov_eigenvectors_ = es.eigenvectors();
    rotation_cov_eigenvalues_ = es.eigenvalues();
    rotation_cov_eigenvalues_sqrt_ = rotation_cov_eigenvalues_.cwiseSqrt();
    
    Matrix3d skewmat;
    skewmat << 0,0,0, 0,0,-1, 0,1,0;
    rotation_cov_x_disk_mat_ = skewmat * rotation_cov_eigenvalues_sqrt_.asDiagonal();
    skewmat << 0,0,1, 0,0,0, -1,0,0;
    rotation_cov_y_disk_mat_ = skewmat * rotation_cov_eigenvalues_sqrt_.asDiagonal();
    skewmat << 0,-1,0, 1,0,0, 0,0,0;
    rotation_cov_z_disk_mat_ = skewmat * rotation_cov_eigenvalues_sqrt_.asDiagonal();
    
    int num_points_on_circle = 16;
    
  	float dtheta = float(kPi_2) / float(num_points_on_circle);
    rotation_cov_x_disk_points_.resize(3,num_points_on_circle);
    rotation_cov_y_disk_points_.resize(3,num_points_on_circle);
    rotation_cov_z_disk_points_.resize(3,num_points_on_circle);
    for (int i=0; i<num_points_on_circle; i++) {
  		float theta = dtheta*float(i);
      float st = std::sin(theta);
      float ct = std::cos(theta);
      rotation_cov_z_disk_points_(0,i) = ct;
      rotation_cov_z_disk_points_(1,i) = st;
      rotation_cov_z_disk_points_(2,i) = 0.;
      rotation_cov_x_disk_points_(0,i) = 0.;
      rotation_cov_x_disk_points_(1,i) = st;
      rotation_cov_x_disk_points_(2,i) = ct;
      rotation_cov_y_disk_points_(0,i) = ct;
      rotation_cov_y_disk_points_(1,i) = 0.;
      rotation_cov_y_disk_points_(2,i) = st;
    }
    rotation_cov_x_disk_points_ = rotation_cov_x_disk_mat_ * rotation_cov_x_disk_points_;
    rotation_cov_y_disk_points_ = rotation_cov_y_disk_mat_ * rotation_cov_y_disk_points_;
    rotation_cov_z_disk_points_ = rotation_cov_z_disk_mat_ * rotation_cov_z_disk_points_;
    Eigen::Vector3d x_axis; x_axis << 1,0,0;
    Eigen::Vector3d y_axis; y_axis << 0,1,0;
    Eigen::Vector3d z_axis; z_axis << 0,0,1;
    rotation_cov_x_disk_points_.colwise() += x_axis;
    rotation_cov_y_disk_points_.colwise() += y_axis;
    rotation_cov_z_disk_points_.colwise() += z_axis;
    
    return true;
  }
  
  bool CalculateVelocityDisplayParameters() {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es;
    es.compute(v_covariance_);
    v_cov_eigenvectors_ = es.eigenvectors();
    v_cov_eigenvalues_ = es.eigenvalues();
    v_cov_eigenvalues_sqrt_ = v_cov_eigenvalues_.cwiseSqrt();
    es.compute(w_covariance_);
    w_cov_eigenvectors_ = es.eigenvectors();
    w_cov_eigenvalues_ = es.eigenvalues();
    w_cov_eigenvalues_sqrt_ = w_cov_eigenvalues_.cwiseSqrt();
    
    v_norm_ = v_.norm();
    w_norm_ = w_.norm();
    Eigen::Vector3d z_axis; z_axis << 0,0,1;
    Eigen::Quaterniond v_arrow_q_; v_arrow_q_.setFromTwoVectors(z_axis, v_);
    v_arrow_T_.setZero(); v_arrow_T_(3,3) = 1.;
    v_arrow_T_.block<3,3>(0,0) = Eigen::Matrix3d(v_arrow_q_);
    Eigen::Quaterniond w_arrow_q_; w_arrow_q_.setFromTwoVectors(z_axis, w_);
    w_arrow_T_.setZero(); w_arrow_T_(3,3) = 1.;
    w_arrow_T_.block<3,3>(0,0) = Eigen::Matrix3d(w_arrow_q_);
    v_circles_T_.setZero(); v_circles_T_(3,3) = 1.;
    v_circles_T_.block<3,3>(0,0) = v_cov_eigenvectors_;
    w_circles_T_.setZero(); w_circles_T_(3,3) = 1.;
    w_circles_T_.block<3,3>(0,0) = w_cov_eigenvectors_;
    return true;
  }
  
  std::string ToString(int detail = 0) {
    std::ostringstream ss; ss << "[" << qv_.transpose() << "] [" << p_.transpose() << "]";
    if (detail>0) {ss << "\n pose covariace = \n" << pose_covariance_;}
    if (detail>1) {ss << "\n posn cov EigenVectors = \n" << position_cov_eigenvectors_;}
    if (detail>1) {ss << "\n posn cov EigenValues = \n" << position_cov_eigenvalues_.transpose();}
    if (detail>1) {ss << "\n rotn cov EigenVectors = \n" << rotation_cov_eigenvectors_;}
    if (detail>1) {ss << "\n rotn cov EigenValues = \n" << rotation_cov_eigenvalues_.transpose();}
    return ss.str();
  }
};  // PoseStateDisplay 


/* Kinematic Pose operations
 *  MoveInTime (KinematicState, double) -> KinematicState, Vector18d
 *  MoveInSpace (KinematicState, PoseState) -> KinematicState, 
 *  AddRelativeMotion (KinematicState, ) -> 
 *  InterpolateInTime
 *
 * In pose operations, sometimes jacobians/variances will be needed and other times not. When
 * moving in time for propagation forward, jacobian wrt time is not needed. But when moving in time
 * within the interval where amount of move is via time delay, jacobian is needed. We would prefer
 * to interpolate the jacobians/variances from edges as approximations, so we will calculate the
 * jacobians even in case of propagation of states.
 *
 * We need to tell the calculation function what needs to be calculated. Best way to do this is by
 * using pointers. If a pointer to a jacobian matrix is there, do the calculation, otherwise not.
 *  
 */

/* Function to help in integrating kinematic motion through time
 *  <3,3>(0,0): B0_R_Bt(w0*t + dw0/2*t*t) - rotation from Bt frame to B0 frame
 *    B0 is the starting pose at t=0, So this is identity rotation
 *    Bt is the rotation due to rotation through angle of w0*t + dw0/2*t*t
 *  <3,3>(3,0): t * SkewSymm( B0_R_Bt(w0*t + dw0/2*t*t) * a0 )
 *  <3,3>(6,0): 0.5 * t * t * SkewSymm( B0_R_Bt(w0*t + dw0/2*t*t) * a0 )
 *  Maths here in Robotics notebook #5 pg 38-41
 */
inline bool KinMotionInTime(const Vector3d& w0, const Vector3d& dw0, const Vector3d& dv0,
                            const double& t, Matrix9x3d& m) {
  Vector3d aa = w0*t + dw0*0.5*t*t;
  double angle = aa.norm();
  if (std::abs(angle) < kEpsilon) {
    m.block<3,3>(0,0) = kIdentity3d;
    m.block<3,3>(3,0) = t * SkewSymm3d(dv0);
    m.block<3,3>(6,0) = 0.5 * t * m.block<3,3>(3,0);
  } else {
    Eigen::AngleAxisd angleaxis(angle, aa.normalized());
    m.block<3,3>(0,0) = angleaxis.toRotationMatrix();
    Vector3d B0_a_B = m.block<3,3>(0,0) * dv0;
    m.block<3,3>(3,0) = t * SkewSymm3d(B0_a_B);
    m.block<3,3>(6,0) = 0.5 * t * m.block<3,3>(3,0);
  }
  //VLOG(1) << "m = \n" << m;
  return true;
}


/* Move a kinematic state in time - assumes constant angular and linear accelerations
 *  In: Kinematic state to be moved in time, time interval of move, variance of interval
 *  Out: Modify another state that is ahead in time
 */ 
inline bool MoveInTime(const KinematicState& K0, const double t1, const double var_t1,
                       KinematicState* K1)
{  
  // Double integrate the rotation, t*SkewSymm(Rotation*a), 0.5*t*t*SkewSymm(Rotation*a)
  Matrix9x3d II_kinmotion, I_kinmotion, kinmotion;
  II_kinmotion.setZero(); I_kinmotion.setZero(); kinmotion.setZero(); 
  std::function<bool(const double&, Matrix9x3d&)>
      kinmotion_integral = std::bind(KinMotionInTime, K0.w_, K0.dw_, K0.dv_,
                                     std::placeholders::_1, std::placeholders::_2);
  DoubleIntegralFunction<Matrix9x3d>(kinmotion_integral, t1, II_kinmotion, I_kinmotion, kinmotion);
  VLOG(1) << "Kin motion integrals, \n II = \n" << II_kinmotion << "\n I = \n" << I_kinmotion
      << "\n R = \n" << kinmotion;
  
  // Helpers
  Matrix3d A_R_B0 = K0.R().transpose();
  Matrix3d t1_Identity = t1 * kIdentity3d;
  Matrix1d var_t1_mat(var_t1);
  
  // Calculate estimates
  Quaterniond Bt_q_A(Eigen::Quaterniond(kinmotion.block<3,3>(0,0).transpose())*K0.Q());
  Vector3d A_B0_p_Bt = A_R_B0 * ( K0.V()*t1 + II_kinmotion.block<3,3>(0,0)*K0.dV() );
  Vector3d A_p_Bt = K0.P() + A_B0_p_Bt;
  Vector3d Bt_w_B = K0.W() + K0.dW()*t1;
  Vector3d Bt_v_B = K0.V() + K0.dV()*t1;
  Vector3d Bt_dw_B = K0.dW();
  Vector3d Bt_dv_B = K0.dV();
  double tm = K0.T()[0] + t1;
  
  // Save estimates
  K1->SetPose(Bt_q_A, A_p_Bt);
  K1->SetVelocity(Bt_w_B, Bt_v_B);
  K1->SetAcceleration(Bt_dw_B, Bt_dv_B);
  K1->SetTime(tm);
  
  // Calculate jacobians
  //  Kin covariance has the following blocks: eq, ep, ew, ev, edw, edv, et
  //  Indexes are as follows:                   0,  3,  6,  9,  12,  15, 18
  Matrix19d dK1_dK0; dK1_dK0.setZero();
  Vector19d dK1_dt1; dK1_dt1.setZero();
  // A_eR_Bt: four terms, all positive
  dK1_dK0.block<3,3>(0,0) =   kIdentity3d;
  dK1_dK0.block<3,3>(0,6) =   t1 * A_R_B0;
  dK1_dK0.block<3,3>(0,12) =  0.5 * t1 * dK1_dK0.block<3,3>(0,6);
  dK1_dt1.block<3,1>(0,0) =   A_R_B0 * Bt_w_B;
  // A_ep_Bt: seven terms, three negative, four positive
  dK1_dK0.block<3,3>(3,0) =  -SkewSymm3d(A_B0_p_Bt);
  dK1_dK0.block<3,3>(3,3) =   kIdentity3d;
  dK1_dK0.block<3,3>(3,6) =  -A_R_B0 * II_kinmotion.block<3,3>(3,0);
  dK1_dK0.block<3,3>(3,9) =   dK1_dK0.block<3,3>(0,6);   // t1 * A_R_B0
  dK1_dK0.block<3,3>(3,12) = -A_R_B0 * II_kinmotion.block<3,3>(6,0);
  dK1_dK0.block<3,3>(3,15) =  A_R_B0 * II_kinmotion.block<3,3>(0,0);
  dK1_dt1.block<3,1>(3,0) =   A_R_B0 * ( K0.V() + I_kinmotion.block<3,3>(0,0)*K0.dV() );
  // Bt_ew_B: three terms
  dK1_dK0.block<3,3>(6,6) =   kIdentity3d;
  dK1_dK0.block<3,3>(6,12) =  t1_Identity;
  dK1_dt1.block<3,1>(6,0) =   K0.dW();
  // Bt_ev_B: three terms
  dK1_dK0.block<3,3>(9,9) =   kIdentity3d;
  dK1_dK0.block<3,3>(9,15) =  t1_Identity;
  dK1_dt1.block<3,1>(9,0) =   K0.dV();
  // Bt_edw_B: one term
  dK1_dK0.block<3,3>(12,12) = kIdentity3d;
  // Bt_edv_B: one term
  dK1_dK0.block<3,3>(15,15) = kIdentity3d;
  // et: two terms
  dK1_dK0(18,18) = 1.;
  dK1_dt1(18,0) =  1.;
  
  // Calculate variance and copy directly
  K1->covariance_ = dK1_dK0 * K0.covariance_ * dK1_dK0.transpose()
                  + dK1_dt1 *   var_t1_mat   * dK1_dt1.transpose();
  
  return true;
}

/* Move a kinematic state in space
 *  Absolute kin state A_K_B_B : B_R_A, A_p_B, B.A_x_B where x = w,v,dw,dv
 *    State's motion vectors are expressed in body B frame
 *  Relative kin state B_K_C_B : C_R_B, B_p_C, B.B_x_C where x = w,v,dw,dv
 *    Relative state's motion vectors are expressed in body B frame, not in body C frame
 *  Added kin state    A_K_C_B : C_R_A, A_p_C, B.A_x_C where x = w,v,dw,dv
 *    Added state is also in body B frame. Rotation to body C can be done as next step if needed.
 *  Maths here is in Robotics notebook#5 pg 57-58
 */
inline bool MoveInSpace(const KinematicState& AKBB, const KinematicState& BKCB,
                        KinematicState* AKCB, bool express_in_C_frame = false)
{
  // Helpers
  Matrix3d BAwBx = SkewSymm3d(AKBB.W());
  Matrix3d BAdw2Bx = SkewSymm3d(AKBB.dW()) + BAwBx*BAwBx;
  Matrix3d ARB = AKBB.R().transpose();
  Matrix3d BpCx = SkewSymm3d(BKCB.P());
  Matrix3d dAaC_dAvB = -SkewSymm3d(BAwBx*BKCB.P()) -BAwBx*BpCx -2.*SkewSymm3d(BKCB.V());
  
  // Estimates
  Quaterniond CRA = BKCB.Q() * AKBB.Q();  // CRB * BRA
  Vector3d ApC = AKBB.P() + ARB*BKCB.P();
  Vector3d AwC = AKBB.W() + BKCB.W();  // B.AwB + B.BwC
  Vector3d AvC = AKBB.V() + BKCB.V() + BAwBx*BKCB.P();
  Vector3d AdwC = AKBB.dW() + BKCB.dW() + BAwBx*BKCB.W();
  Vector3d AdvC = AKBB.dV() + BKCB.dV() + 2.*BAwBx*BKCB.V() + BAdw2Bx*BKCB.P();
  double tm = AKBB.T()[0];
  
  // Jacobians
  //  Kin covariance has the following blocks: eq, ep, ew, ev, edw, edv, et
  //  Indexes are as follows:                   0,  3,  6,  9,  12,  15, 18
  Matrix19d dAKCB_dAKBB; dAKCB_dAKBB.setZero();
  dAKCB_dAKBB.block<3,3>(0,0) =  kIdentity3d;
  dAKCB_dAKBB.block<3,3>(3,0) = -SkewSymm3d(ARB*BKCB.P());
  dAKCB_dAKBB.block<3,3>(3,3) =  kIdentity3d;
  dAKCB_dAKBB.block<3,3>(6,6) =  kIdentity3d;
  dAKCB_dAKBB.block<3,3>(9,6) = -BpCx;
  dAKCB_dAKBB.block<3,3>(9,9) =  kIdentity3d;
  dAKCB_dAKBB.block<3,3>(12,6) =  -SkewSymm3d(BKCB.W());
  dAKCB_dAKBB.block<3,3>(12,12) =  kIdentity3d;
  dAKCB_dAKBB.block<3,3>(15,6)  =  dAaC_dAvB;
  dAKCB_dAKBB.block<3,3>(15,12) = -BpCx;
  dAKCB_dAKBB.block<3,3>(15,15) =  kIdentity3d;
  
  Matrix19d dAKCB_dBKCB; dAKCB_dBKCB.setZero();
  dAKCB_dBKCB.block<3,3>(0,0) =  ARB;
  dAKCB_dBKCB.block<3,3>(3,3) =  ARB;
  dAKCB_dBKCB.block<3,3>(6,6) =  kIdentity3d;
  dAKCB_dBKCB.block<3,3>(9,3) =  BAwBx;
  dAKCB_dBKCB.block<3,3>(9,9) =  kIdentity3d;
  dAKCB_dBKCB.block<3,3>(12,6) =  BAwBx;
  dAKCB_dBKCB.block<3,3>(12,12) =  kIdentity3d;
  dAKCB_dBKCB.block<3,3>(15,3) =   BAdw2Bx;
  dAKCB_dBKCB.block<3,3>(15,9) =   2.*BAwBx;
  dAKCB_dBKCB.block<3,3>(15,15) =  kIdentity3d;
  
  // Rotate motions to C frame if asked to
  if (express_in_C_frame) {
    Matrix3d CRB = BKCB.R();
    // Jacobians
    dAKCB_dBKCB.block<3,3>(6,0) =  SkewSymm3d(AwC);
    dAKCB_dBKCB.block<3,3>(9,0) =  SkewSymm3d(AvC);
    dAKCB_dBKCB.block<3,3>(12,0) = SkewSymm3d(AdwC);
    dAKCB_dBKCB.block<3,3>(15,0) = SkewSymm3d(AdvC);
    dAKCB_dAKBB.block<3,19>(6,0) =  CRB*dAKCB_dAKBB.block<3,19>(6,0);
    dAKCB_dAKBB.block<3,19>(9,0) =  CRB*dAKCB_dAKBB.block<3,19>(9,0);
    dAKCB_dAKBB.block<3,19>(12,0) = CRB*dAKCB_dAKBB.block<3,19>(12,0);
    dAKCB_dAKBB.block<3,19>(15,0) = CRB*dAKCB_dAKBB.block<3,19>(15,0);
    dAKCB_dBKCB.block<3,19>(6,0) =  CRB*dAKCB_dBKCB.block<3,19>(6,0);
    dAKCB_dBKCB.block<3,19>(9,0) =  CRB*dAKCB_dBKCB.block<3,19>(9,0);
    dAKCB_dBKCB.block<3,19>(12,0) = CRB*dAKCB_dBKCB.block<3,19>(12,0);
    dAKCB_dBKCB.block<3,19>(15,0) = CRB*dAKCB_dBKCB.block<3,19>(15,0);
    AwC = CRB * AwC;
    AvC = CRB * AvC;
    AdwC = CRB * AdwC;
    AdvC = CRB * AdvC;
  }
  
  // Save estimates
  AKCB->SetPose(CRA, ApC);
  AKCB->SetVelocity(AwC, AvC);
  AKCB->SetAcceleration(AdwC, AdvC);
  AKCB->SetTime(tm);
  
  // Compute and save variances
  AKCB->covariance_ = dAKCB_dAKBB * AKBB.covariance_ * dAKCB_dAKBB.transpose()
                    + dAKCB_dBKCB * BKCB.covariance_ * dAKCB_dBKCB.transpose();
  
  return true;
}

inline bool InterpolateInTime(const KinematicState& K0, const KinematicState& K1, double ti,
                              KinematicState* Ki)
{
  return true;
}
 

class CameraState {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  // 4 pinhole + 5 distortion parameters: fx fy cx cy, k1 k2 k3, t1 t2
  // 7 pose: q, p
  // 1 for time delay: td
  
  Vector9d intrinsics_;   // Camera pinhole, radial and tangential distortion parameters
  Vector7d pose_;         // Camera pose, usually in machine frame. 
  Vector1d delay_;        // Camera image data path delay wrt some reference signal e.g. commander
  
  Vector9d intrinsics_error_;
  Vector6d pose_error_;
  Vector1d delay_error_;
  
  Matrix9d intrisics_covariance_;
  Matrix6d pose_covariance_;
  Matrix1d delay_covariance_;
  
  bool IsZero() const {
    return (intrinsics_.isZero());
  }
  
};


/** Calibration target view residual
 *
 *  Guess:
 *  Target's pose in camera frame + variance of the pose
 *  Camera intrinsics + variance of the parameters
 *
 *  Observation:
 *  AprilTag corners of the target seen in the image
 *
 *  Creation:
 *  Residual needs pre-initialized target pose state and camera state.
 *  Starting residuals are calculated
 *  Jacobians are calculated for corner views
 *  Variance of views are then calculated
 *
 *  Number of residuals is variable as it depends on the number of tags seen
 *  
 */
class CalibrationTargetViewResidual : public ceres::CostFunction {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  // Observations
  const SensorMsg*  message_;
  
  // States
  PoseState*    pose_;
  CameraState*  camera_;
  
  // Constructor
  CalibrationTargetViewResidual():
    message_(nullptr), pose_(nullptr), camera_(nullptr)
  {
    Reset();
  }
  
  // Default copy constructor
  CalibrationTargetViewResidual(const CalibrationTargetViewResidual& r) :
    message_(nullptr), pose_(nullptr), camera_(nullptr)
  {
    message_ = r.message_;
    pose_= r.pose_;
    camera_ = r.camera_;
  }
  
  // Equal to assignment operator
  CalibrationTargetViewResidual& operator= (const CalibrationTargetViewResidual& r) {
    message_ = r.message_;
    pose_= r.pose_;
    camera_ = r.camera_;
  }
  
  bool Reset() {
    message_ = nullptr;
    pose_ = nullptr;
    camera_ = nullptr;
    return true;
  }
  
  // Create residual using target pose state and camera state
  bool Create(const SensorMsg* message, PoseState* pose, CameraState* camera) {
    if (!message) {LOG(ERROR)<<"sensor message is null"; return false;}
    if (!pose) {LOG(ERROR)<<"pose is null"; return false;}
    if (!camera) {LOG(ERROR)<<"camera is null"; return false;}
    if (!message->has_april_msg()) {LOG(ERROR)<<"Message has no aprilag message"; return false;}
    if (pose->IsZero()) {LOG(WARNING)<<"pose is zero";}
    if (camera->IsZero()) {LOG(WARNING)<<"camera is zero";}
    
    message_ = message;
    pose_ = pose;
    camera_ = camera;
    
    if (!CalculateResiduals()) {
      LOG(ERROR) << "Could not calculate residuals";
      return false;
    }
    
    return true;
  }
  
  // Calculate residuals, jacobians and variances
  bool CalculateResiduals() {
    
    // How many tags were seen? 
    const AprilTagMessage& apriltag_msg = message_->april_msg();
    int32_t num_tags = apriltag_msg.tag_id_size();
    if (num_tags == 0) {LOG(ERROR)<<"num_tags seen == 0"; return false;}
    
    // Set number of residuals
    int32_t num_residuals = num_tags * 4 * 2;  // 4 corners per tag, 2 indices per corner
    set_num_residuals(num_residuals);
    
    // Set parameter block sizes
    mutable_parameter_block_sizes()->push_back(6);  // Target pose has 6 parameters to guess
    mutable_parameter_block_sizes()->push_back(9);  // Camera intrinsics has 9 parameters to guess
    
    // 
    
    return true;
  }
  
  
  // Destructor
  virtual ~CalibrationTargetViewResidual() {}
};  // CalibrationTargetViewResidual


///** Add poses
// * Adds two poses */
//bool AddPoses(
//    const Vector7d* WPA, const Vector7d* APB, Vector7d* WPB,  // Poses
//    Matrix6d* WJA, Matrix6d* AJB,                             // Jacobians
//    const Matrix6d* WQA, const Matrix6d* AQB, Matrix6d* WQB   // Covariance matrices
//) {
//  
//  if (!WPA || !APB || !WPB) {
//    LOG(ERROR) << "WPA or APB or WPB are NULL. Quit.";
//    return false;
//  }
//  
//  const Eigen::Quaterniond AqW(WPA->data());
//  const Eigen::Quaterniond BqA(APB->data());
//  const Eigen::Vector3d WpA(WPA->data()+4);
//  const Eigen::Vector3d ApB(APB->data()+4);
//  const Eigen::Matrix3d WrA(AqW.conjugate());
//  
//  Eigen::Quaterniond BqW = BqA * AqW;
//  Eigen::Matrix3d BrW(BqW);
//  Eigen::Vector3d WApB = WrA*ApB;
//  Eigen::Vector3d WpB = WpA + WApB;
//  
//  Eigen::Map<Eigen::Vector4d> BqW_map(WPB->data());
//  Eigen::Map<Eigen::Vector3d> WpB_map(WPB->data()+4);
//  BqW_map = BqW.coeffs();
//  WpB_map = WpB;
//  
//  if (!WJA || !AJB) {return true;}
//  
//  WJA->setZero();
//  WJA->block<3,3>(0,0) =  Eigen::Matrix3d::Identity();
//  WJA->block<3,3>(3,3) =  Eigen::Matrix3d::Identity();
//  WJA->block<3,3>(3,0) = -SkewSymmetricMatrix(WApB);
//
//  AJB->setZero();
//  AJB->block<3,3>(0,0) =  WrA;
//  AJB->block<3,3>(3,3) =  WrA;
//  
//  if (!WQA || !AQB || !WQB) {return true;}
//  
//  *WQB = (*WJA) * (*WQA) * (*WJA).transpose() + (*AJB) * (*AQB) * (*AJB).transpose();
//  
//  return true;
//}
//
///* Invert pose */
//bool InvertPose(
//    const Vector7d* APB, Vector7d* BPA,   // Poses
//    Matrix6d* AJB,                        // Jacobian
//    const Matrix6d* AQB, Matrix6d* BQA    // Covariance matrices
//) {
//  
//  if (!APB || !BPA) {
//    LOG(ERROR) << "APB or BPA are NULL. Quit.";
//    return false;
//  }
//  
//  const Eigen::Quaterniond BqA(APB->data());
//  const Eigen::Vector3d ApB(APB->data()+4);
//  const Eigen::Matrix3d BrA(BqA);
//  
//  Eigen::Quaterniond AqB = BqA.conjugate();
//  Eigen::Vector3d BpA = -BrA*ApB;
//  
//  Eigen::Map<Eigen::Vector4d> AqB_map(BPA->data());
//  Eigen::Map<Eigen::Vector3d> BpA_map(BPA->data()+4);
//  AqB_map = AqB.coeffs();
//  BpA_map = BpA;
//  
//  if (!AJB) return true;
//  
//  AJB->setZero();
//  AJB->block<3,3>(0,0) = -BrA;
//  AJB->block<3,3>(3,3) = -BrA;
//  AJB->block<3,3>(3,0) = -BrA*SkewSymmetricMatrix(ApB);
//  
//  if (!AQB || !BQA) return true;
//  
//  *BQA = (*AJB) * (*AQB) * (*AJB);
//  
//  return true;
//}
//
///** Diff poses */
//bool DiffPoses(
//    const Vector7d* WPA, const Vector7d* WPB, Vector7d* APB,  // Poses
//    Matrix6d* WJA, Matrix6d* WJB,                             // Jacobians
//    const Matrix6d* WQA, const Matrix6d* WQB, Matrix6d* AQB   // Covariance matrices    
//) {
//  if (!WPA || !APB || !WPB) {
//    LOG(ERROR) << "WPA or APB or WPB are NULL. Quit.";
//    return false;
//  }
//  
//  const Eigen::Quaterniond AqW(WPA->data());
//  const Eigen::Quaterniond BqW(WPB->data());
//  const Eigen::Vector3d WpA(WPA->data()+4);
//  const Eigen::Vector3d WpB(WPB->data()+4);
//  const Eigen::Matrix3d ArW(AqW);
//  
//  Eigen::Quaterniond BqA = BqW * AqW.conjugate();
//  Eigen::Vector3d WApB = WpB - WpA;
//  Eigen::Vector3d ApB = ArW*WApB;
//
//  Eigen::Map<Eigen::Vector4d> BqA_map(APB->data());
//  Eigen::Map<Eigen::Vector3d> ApB_map(APB->data()+4);
//  BqA_map = BqA.coeffs();
//  ApB_map = ApB;
//  
//  if (!WJA || !WJB) return true;
//  
//  WJA->setZero();
//  WJA->block<3,3>(0,0) = -ArW;
//  WJA->block<3,3>(3,3) = -ArW;
//  WJA->block<3,3>(3,0) =  ArW*SkewSymmetricMatrix(WApB);
//  
//  WJB->setZero();
//  WJB->block<3,3>(0,0) =  ArW;
//  WJB->block<3,3>(3,3) =  ArW;
//  
//  if (!WQA || !WQB || !AQB) return true;
//  
//  *AQB = (*WJA) * (*WQA) * (*WJA).transpose() + (*WJB) * (*WQB) * (*WJB).transpose();
//  
//  return true;
//}
//
//
///* Point projection on an image */
//bool ProjectPoint(
//    const Vector3d* xyz,    // Point in camera coordinates
//    const Vector4d* K,      // Camera marix components fx, fy, cx, cy 
//    const Vector3d* D, const Vector2d* T,  // Radial and tangential distortion coefficients
//    Vector2d*   uvim,         // Point in image
//    Matrix2x3d* duvim_dxyz,    // Jacobian of projection with point coordinates
//    Matrix2x4d* duvim_dK,      // Jacobian of projection with camera matrix
//    Matrix2x3d* duvim_dD,      // Jacobian of projection with radial distortion coefficients
//    Matrix2d*   duvim_dT       // Jacobian of projection with tangential distortion coefficients
//) {
//  if (!xyz || !K || !uv_im) {LOG(ERROR) << "!xyz || !K || !uv_im. Quit."; return false;}
//  if (!D || !T) {LOG(ERROR) << "!D || !T"; return false;}
//  
//  if (xyz->(2) <= Epsilon) {LOG(ERROR) << "xyz->(2) <= Epsilon. Quit."; return false;}
//  
//  // Direct calculations
//  Vector2d uv; uv << (*xyz)(0)/(*xyz)(2), (*xyz)(1)/(*xyz)(2);
//  double u = uv(0); double u2 = u*u;
//  double v = uv(1); double v2 = v*v;
//  double r = u2 + v2; double r2 = r*r; double r3 = r2*r;
//  double s = u*v;
//  double k1 = (*D)(0); double k2 = (*D)(1); double k3 = (*D)(2);
//  double t1 = (*T)(0); double t2 = (*T)(1);
//  Vector2d t_; t_ << 2.*t1*s + t2*(r + 2.*u2), t1*(r + 2.*v2) + 2.*t2*s;
//  double d_ = 1. + k1*r + k2*r2 + k3*r3;
//  Vector2d udvd = d_ * uv + t_;
//  Vector2d cxcy; cxcy << (*K)(2), (*K)(3);
//  Matrix2d fxfy; fxfy << (*K)(0), 0., 0., (*K)(1);
//  *uvim = cxcy + fxfy * udvd;
//  
//  if (!duvim_dxyz || !duvim_dK || !duvim_dD || !duvim_dT) return true;
//  
//  // Jacobians
//  Matrix1x3d rrr; rrr << r, r2, r3;
//  Matrix2x3d dudvd_dD = uv * rrr;
//  Matrix2d dudvd_dT; dudvd_dT << 2.*s, r + 2.*u2, r + 2.*v2, 2.*s;
//  Matrix1x3d kkkrrr; kkkrrr << k1, 2.*k2*r, 3.*k3*r2;
//  Matrix3x2d uvuvuv; uvuvuv << u,v,u,v,u,v; uvuvuv *= 2.;
//  double aa = 2.*t1*u + 2.*t2*v;
//  Matrix2d tuv; tuv << 6.*t2*u + 2.*t1*v, aa, aa, 2.*t2*u + 6.*t1*v;
//  Matrix2d dudvd_duv = uv*kkkrrr*uvuvuv + d_*Eigen::Vector2d::Identity() + tuv;
//  Matrix2x3d duv_dxyz; duv_dxyz << 1., 0., -u,  0., 1., -v;  duv_dxyz /= (*xyz)(2);
//  
//  *duvim_dxyz = fxfy * dudvd_duv * duv_dxyz;
//  *duvim_dK << udvd(0), 0., 1., 0.,   0., udvd(1), 0., 1.;
//  *duvim_dD = fxfy * dudvd_dD;
//  *duvim_dT = fxfy * dudvd_dT;
//  
//  return true;
//}
//
//
///* Kinematic state is pose, velocity and acceleration */
//
///* Move forward assuming constant accelerations */
//bool MoveInTime(
//    const Vector7d* P0, const Vector6d* V0, const Vector6d* A0,
//    const double dt,
//    Vector7d* P1, Vector6d* V1, Vector6d* A1,
//    Matrix6x18d* P1J0, Matrix6x18d* V1J0, Matrix6x18d* A1J0,
//    Matrix6d* P1Jdt, Matrix6d* V1Jdt, Matrix6d* A1Jdt)
//{
//  
//  return true;
//}
//
///* Move in space at this instant */
//bool MoveInSpace(
//    const Vector7d* P0, const Vector6d* V0, const Vector6d* A0,
//    const Vector7d* P, 
//    Vector7d* P1, Vector6d* V1, Vector6d* A1,
//    Matrix6x18d* P1J0, Matrix6x18d* V1J0, Matrix6x18d* A1J0,
//    Matrix6d* P1Jdt, Matrix6d* V1Jdt, Matrix6d* A1Jdt,
//    bool pose_is_absolute = true)     // true: P is absolute pose, false: P is relative
//{
//  
//  return true;
//}
//
///* Relative kinematics */
//bool DiffKinematics(
//    const Vector7d* P0, const Vector6d* V0, const Vector6d* A0,
//    const Vector7d* P1, const Vector6d* V1, const Vector6d* A1,
//    Vector7d* P01, Vector6d* V01, Vector6d* A01,
//    Matrix6x18d* P01J0, Matrix6x18d* V01J0, Matrix6x18d* A01J0,
//    Matrix6x18d* P01J1, Matrix6x18d* V01J1, Matrix6x18d* A01J1)
//{
//  
//  return true;
//}
//
//bool AddKinematics(
//    const Vector7d* P0, const Vector6d* V0, const Vector6d* A0,
//    const Vector7d* P01, const Vector6d* V01, const Vector6d* A01,
//    Vector7d* P1, Vector6d* V1, Vector6d* A1,
//    Matrix6x18d* P1J0, Matrix6x18d* V1J0, Matrix6x18d* A1J0,
//    Matrix6x18d* P1J01, Matrix6x18d* V1J01, Matrix6x18d* A1J01)
//{
//  
//  return true;
//}



} // namespace

#endif
