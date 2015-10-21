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
#include "Filter/timed_circular_queue.h"
#include <Utilities/common_functions.h>
#include "DataQueue/message_file_writer.h"
#include "DataQueue/message_file_reader.h"

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

/** OpenCV includes */
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>


namespace anantak {

typedef Eigen::Matrix<double,1,1> Vector1d;
typedef Eigen::Matrix<double,2,1> Vector2d;
typedef Eigen::Matrix<double,3,1> Vector3d;
typedef Eigen::Matrix<double,4,1> Vector4d;
typedef Eigen::Matrix<double,5,1> Vector5d;
typedef Eigen::Matrix<double,6,1> Vector6d;
typedef Eigen::Matrix<double,7,1> Vector7d;
typedef Eigen::Matrix<double,8,1> Vector8d;
typedef Eigen::Matrix<double,9,1> Vector9d;
typedef Eigen::Matrix<double,12,1> Vector12d;
typedef Eigen::Matrix<double,18,1> Vector18d;
typedef Eigen::Matrix<double,19,1> Vector19d;
typedef Eigen::Matrix<double,20,1> Vector20d;
typedef Eigen::Matrix<double,39,1> Vector39d;

typedef Eigen::Matrix<double,1,4> RowVector4d;

typedef Eigen::Matrix<double,1,1> Matrix1d;
typedef Eigen::Matrix<double,2,2> Matrix2d;
typedef Eigen::Matrix<double,3,3> Matrix3d;
typedef Eigen::Matrix<double,4,4> Matrix4d;
typedef Eigen::Matrix<double,5,5> Matrix5d;
typedef Eigen::Matrix<double,6,6> Matrix6d;
typedef Eigen::Matrix<double,9,9> Matrix9d;
typedef Eigen::Matrix<double,12,12> Matrix12d;
typedef Eigen::Matrix<double,18,18> Matrix18d;
typedef Eigen::Matrix<double,19,19> Matrix19d;
typedef Eigen::Matrix<double,20,20> Matrix20d;

typedef Eigen::Matrix<double,4,4,Eigen::RowMajor> Matrix4dRowType;

typedef Eigen::Matrix<double,1,3> Matrix1x3d;
typedef Eigen::Matrix<double,2,3> Matrix2x3d;
typedef Eigen::Matrix<double,2,4> Matrix2x4d;
typedef Eigen::Matrix<double,2,6> Matrix2x6d;
typedef Eigen::Matrix<double,2,9> Matrix2x9d;
typedef Eigen::Matrix<double,2,18> Matrix2x18d;
typedef Eigen::Matrix<double,2,19> Matrix2x19d;
typedef Eigen::Matrix<double,3,2> Matrix3x2d;
typedef Eigen::Matrix<double,3,4> Matrix3x4d;
typedef Eigen::Matrix<double,3,5> Matrix3x5d;
typedef Eigen::Matrix<double,3,6> Matrix3x6d;
typedef Eigen::Matrix<double,3,9> Matrix3x9d;
typedef Eigen::Matrix<double,3,12> Matrix3x12d;
typedef Eigen::Matrix<double,6,12> Matrix6x12d;
typedef Eigen::Matrix<double,6,18> Matrix6x18d;
typedef Eigen::Matrix<double,6,19> Matrix6x19d;
typedef Eigen::Matrix<double,9,3> Matrix9x3d;
typedef Eigen::Matrix<double,9,12> Matrix9x12d;
typedef Eigen::Matrix<double,19,6> Matrix19x6d;

typedef Eigen::Map<Eigen::Matrix<double,1,1>> MapVector1d;
typedef Eigen::Map<Eigen::Vector2d> MapVector2d;
typedef Eigen::Map<Eigen::Vector3d> MapVector3d;
typedef Eigen::Map<Eigen::Vector4d> MapVector4d;
typedef Eigen::Map<Eigen::Matrix<double,6,1>> MapVector6d;
typedef Eigen::Map<Eigen::Matrix<double,12,1>> MapVector12d;
typedef Eigen::Map<Eigen::Matrix<double,18,1>> MapVector18d;
typedef Eigen::Map<Eigen::Matrix<double,19,1>> MapVector19d;
typedef Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,1>> MapVectorXd;

typedef Eigen::Map<Eigen::Matrix<double,1,1>> MapMatrix1d;
typedef Eigen::Map<Eigen::Matrix<double,2,9>> MapMatrix2x9d;
typedef Eigen::Map<Eigen::Matrix3d> MapMatrix3d;
typedef Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>> MapMatrixXxYRowd;

typedef Eigen::Map<const Eigen::Matrix<double,1,1>> MapConstVector1d;
typedef Eigen::Map<const Eigen::Vector2d> MapConstVector2d;
typedef Eigen::Map<const Eigen::Vector3d> MapConstVector3d;
typedef Eigen::Map<const Eigen::Vector4d> MapConstVector4d;
typedef Eigen::Map<const Eigen::Matrix<double,6,1>> MapConstVector6d;
typedef Eigen::Map<const Eigen::Matrix<double,7,1>> MapConstVector7d;
typedef Eigen::Map<const Eigen::Matrix<double,9,1>> MapConstVector9d;
typedef Eigen::Map<const Eigen::Matrix<double,12,1>> MapConstVector12d;
typedef Eigen::Map<const Eigen::Matrix<double,18,1>> MapConstVector18d;
typedef Eigen::Map<const Eigen::Matrix<double,19,1>> MapConstVector19d;
typedef Eigen::Map<const Eigen::Matrix<double,20,1>> MapConstVector20d;
typedef Eigen::Map<const Eigen::Matrix<double,Eigen::Dynamic,1>> MapConstVectorXd;

typedef Eigen::Map<const Eigen::Matrix2d> MapConstMatrix2d;
typedef Eigen::Map<const Eigen::Matrix3d> MapConstMatrix3d;
typedef Eigen::Map<const Eigen::Matrix<double,6,6>> MapConstMatrix6d;
typedef Eigen::Map<const Eigen::Matrix<double,9,9>> MapConstMatrix9d;
typedef Eigen::Map<const Eigen::Matrix<double,12,12>> MapConstMatrix12d;
typedef Eigen::Map<const Eigen::Matrix<double,18,18>> MapConstMatrix18d;
typedef Eigen::Map<const Eigen::Matrix<double,19,19>> MapConstMatrix19d;

typedef Eigen::Quaterniond Quaterniond;
typedef Eigen::AngleAxisd  AngleAxisd;

// Templated typdef - starts in gcc 4.7
//template <typename Type> using IntegrandType = std::function<bool(const double&, Type&)>;

/** Global constants */

static const double kPi = 3.14159265358979323846;
static const double kPi_half = kPi*0.5;
static const double kPi_2 = kPi*2.0;
static const double kRadiansPerDegree = kPi/180.0;
static const double kDegreesPerRadian = 180.0/kPi;

static const double kEpsilon = Eigen::NumTraits<double>::epsilon();
static const double kOnePluskEpsilon = double(1.) + kEpsilon;
static const double kNegativeOnePluskEpsilon = -kOnePluskEpsilon;
static const double kOneLessEpsilon = double(1.) - kEpsilon;
static const Matrix3d kIdentity3d(Eigen::Matrix3d::Identity());
static const Matrix6d kIdentity6d(Matrix6d::Identity());
static const Matrix18d kIdentity18d(Matrix18d::Identity());

static const Eigen::IOFormat CleanFmt1(1, 0, " ", "\n", "", "");
static const Eigen::IOFormat CleanFmt2(2, 0, " ", "\n", "", "");
static const Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");

static const cv::Scalar CV_RED = cv::Scalar(0,0,255);
static const cv::Scalar CV_BLUE = cv::Scalar(255,0,0);
static const cv::Scalar CV_GREEN = cv::Scalar(0,255,0);
static const cv::Scalar CV_BLACK = cv::Scalar(0,0,0);
static const cv::Scalar CV_WHITE = cv::Scalar(255,255,255);
static const cv::Scalar CV_YELLOW = cv::Scalar(0,255,255);

// Skew symmetric matrix from a vector 
inline Eigen::Matrix3d SkewSymm3d(const Vector3d& v) {
  Eigen::Matrix3d mat;
  mat <<     0,  -v[2],   v[1],
          v[2],      0,  -v[0],
         -v[1],   v[0],      0;
  return mat;
}

// Check if a quaternion is normalized
inline bool CheckQuaternionNorm(const Quaterniond& q, const std::string name="") {
  double nrm = q.squaredNorm();
  if (std::abs(nrm-1.) > 1e-6) {
    LOG(ERROR) << "Quaternion " << name << " is not normalized: " << q.coeffs().transpose()
        << " " << nrm << 1.-nrm;
    return false;
  }
  return true;
}

// Small angle threshold for conversions between quaternion and angleaxis
static const double kSmallAngleThreshold = kEpsilon*kRadiansPerDegree;
static const double kSmallAngleThresholdSquared = kSmallAngleThreshold*kSmallAngleThreshold;
static const double kCosSmallAngleThresholdBy2 = std::cos(0.5*kSmallAngleThreshold);
static const double kSinSmallAngleThresholdBy2 = std::sin(0.5*kSmallAngleThreshold);
static const double kSinSqrSmallAngleThresholdBy2 = kSinSmallAngleThresholdBy2*kSinSmallAngleThresholdBy2;

// Rotation error to quaternion
inline Quaterniond ErrorAngleAxisToQuaternion(const Vector3d& err, bool use_approx = false) {
  if (!err.allFinite()) {LOG(ERROR) << "err vector is not finite " << err.transpose();}
  Eigen::Quaterniond err_quat;
  
  double theta_sqr = err.squaredNorm();
  if ((theta_sqr > kSmallAngleThresholdSquared) && !use_approx) {
    double theta = std::sqrt(theta_sqr);
    double theta_by_2 = 0.5 * theta;
    double k = std::sin(theta_by_2) / theta;
    err_quat.coeffs() << k*err[0], k*err[1], k*err[2], std::cos(theta_by_2);
  } else {
    Eigen::Vector3d quat_vec = 0.5 * err; // make a copy - wasteful?
    double quat_vec_sq_norm = quat_vec.squaredNorm();    
    if (quat_vec_sq_norm < 1.) {
      err_quat.coeffs() << quat_vec[0], quat_vec[1], quat_vec[2], std::sqrt(1.-quat_vec_sq_norm);
    } else {
      double q_w = 1./std::sqrt(1.+quat_vec_sq_norm);
      quat_vec *= q_w;
      err_quat.coeffs() << quat_vec[0], quat_vec[1], quat_vec[2], q_w;
    }
    //err_quat.coeffs() << 0.5*err[0], 0.5*err[1], 0.5*err[2], 1.;
    //err_quat.normalize();   // To be safe
  }
  return err_quat;
}

// Rotation error to quaternion
inline Vector3d QuaternionToErrorAngleAxis(Quaterniond& q, bool use_exact=false) {
  Vector3d aa;
  double w = q.w();
  if (w<kNegativeOnePluskEpsilon || w>kOnePluskEpsilon) {
    LOG(WARNING) << "Normalized the quaternion. Should not be needed. " << q.coeffs().transpose();
    q.normalize();
  }
  // This code uses the logic in ceres library's rotation.h
  double sin_sqr_theta_by_2 = q.vec().squaredNorm();
  if ((sin_sqr_theta_by_2 > kSinSqrSmallAngleThresholdBy2) || use_exact) {
    double sin_theta_by_2 = std::sqrt(sin_sqr_theta_by_2);
    // Ceres' method
    double cos_theta_by_2 = q.w();
    double theta_by_2 = ((cos_theta_by_2 < 0.)
                         ? std::atan2(-sin_theta_by_2, -cos_theta_by_2)
                         : std::atan2( sin_theta_by_2,  cos_theta_by_2));
    // Eigen's method
    //double theta_by_2 = std::acos(q.w());
    double k = theta_by_2 / sin_theta_by_2;
    aa = 2. * k * q.vec();
  } else {
    aa = 2. * q.vec();
  }
  return aa;
}

inline Vector3d QuaternionToEulerAngles(const Quaterniond& q) {
  Vector3d eu;
  Matrix3d mat(q);
  eu = mat.eulerAngles(2,0,2);
  eu *= kDegreesPerRadian;
  return eu;
}

// Same as above
inline Vector3d QuaternionToErrorAngleAxis(const Quaterniond& q, bool use_exact=false) {
  Quaterniond _q(q);
  return QuaternionToErrorAngleAxis(_q, use_exact);
}

// Rotation error to matrix3d
inline Eigen::Matrix3d ErrorAngleAxisToMatrix3d(const Vector3d& err) {
  Eigen::Quaterniond err_quat = ErrorAngleAxisToQuaternion(err);
  return err_quat.toRotationMatrix();
}


/** Calculate Inv Sqrt Cov for a square matrix */
template<typename MatType, typename VecType>
inline bool InverseSqrt(
    const Eigen::MatrixBase<MatType>& cov,
    Eigen::MatrixBase<MatType>* inv_sqrt_cov_ptr,
    Eigen::MatrixBase<VecType>* sqrt_var_ptr,
    Eigen::MatrixBase<MatType>* correl_ptr    = NULL,
    Eigen::MatrixBase<MatType>* check_mat_ptr = NULL) {
  
  // Check
  if (!inv_sqrt_cov_ptr) {
    LOG(ERROR) << "inv_sqrt_cov_ptr is NULL. Nowhere to save calcs. Quit.";
    return false;
  }
  
  // Correlations matrix and stdev
  VecType sqrt_var = cov.diagonal().cwiseSqrt();
  MatType inv_sqrt_var_mat = sqrt_var.cwiseInverse().asDiagonal();
  MatType correl = inv_sqrt_var_mat * cov * inv_sqrt_var_mat;
  //VLOG(1) << "Correlations mat = \n" << std::fixed << correl_mat.format(CleanFmt);
  
  // Calculate inverse-sqrt matrix using Cholesky decomposition of correlations matrix
  Eigen::LLT<MatType> cov_mat_llt(correl);
  MatType mat_L = cov_mat_llt.matrixL();
  MatType mat_L_inv = mat_L.inverse();
  //VLOG(1) << "  Matrix L of LLT decomposition = \n" << mat_L;
  //VLOG(1) << " check \n" << mat_L * mat_L.transpose() * correl_mat.inverse();
  //VLOG(1) << "  Matrix L inverse = \n" << mat_L_inv;
  // Check
  //VLOG(1) << " check \n" << mat_L_inv.transpose() * mat_L_inv * correl_mat;
  // Sqrt inverse mat
  
  *inv_sqrt_cov_ptr = mat_L_inv * inv_sqrt_var_mat;
  //VLOG(1) << "Sqrt inv mat = \n" << std::fixed << cov_mat_sqrt_inv_lld.format(CleanFmt);
  if (sqrt_var_ptr) {
    *sqrt_var_ptr = sqrt_var;
  }
  if (correl_ptr) {
    *correl_ptr = correl;
  }
  // Check now
  if (check_mat_ptr) {
    *check_mat_ptr = (*inv_sqrt_cov_ptr) * cov * (*inv_sqrt_cov_ptr).transpose();  // should be Identity
  }
  //Eigen::IOFormat CleanFmt(2, 0, ", ", "\n", "", "");
  //VLOG(1) << " Check for sqrt mat (I12) = \n" << check_mat.format(CleanFmt);
  return true;
}

/** Write a Eigen matrix to file in CSV format usually for plotting */
template <typename Derived>
inline bool WriteMatrixToCSVFile(const std::string& filename, const Eigen::DenseBase<Derived>& matrix,
                          bool path_is_relative = true) {
  std::string filepath(filename);
  if (path_is_relative) {filepath = anantak::GetProjectSourceDirectory() + "/" + filepath;}
  std::ofstream file(filepath);
  file << matrix.format(CSVFormat);
  VLOG(1) << "Written file " << filepath;
  return true;
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

class Point1dState {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  Vector1d  state_;
  Vector1d  error_;
  Matrix1d  covariance_;
  
  Point1dState() {Reset();}
  Point1dState(double v) {Reset(); state_ << v;}
  Point1dState(double v, double var) {Reset(); state_ << v; covariance_ << var;}
  
  bool Reset() { state_.setZero(); error_.setZero(); covariance_.setZero(); return true;}
  
  // Default copy constructor
  Point1dState(const Point1dState& r) :
    state_(r.state_), error_(r.error_), covariance_(r.covariance_) {}
  
  // Equal to assignment operator
  Point1dState& operator= (const Point1dState& r) {
    state_= r.state_; error_ = r.error_; covariance_ = r.covariance_;
  }
  
  double Value() const {return state_(0);}
  double Error() const {return error_(0);}
  double Var() const {return covariance_(0);}
  bool IsZero() const {return (std::abs(state_(0)+error_(0)) < kEpsilon);}
  
  std::string ToString(int detail=0) const {
    std::ostringstream ss;
    ss << state_.transpose() << "  ~  " << covariance_.transpose();
    return ss.str();
  };
  
  bool CopyToMessage(Point1dStateMessage* msg) const {
    msg->Clear();
    msg->set_state(state_(0));
    msg->set_covariance(covariance_(0));
    return true;
  }
  
  bool SetFromMessage(const Point1dStateMessage& msg) {
    Reset();
    state_(0) = msg.state();
    covariance_(0) = msg.covariance();
    return true;
  }
  
};  // Point1dState

class Point2dState {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  Vector2d  state_;
  Vector2d  error_;
  Matrix2d  covariance_;
  
  Point2dState() {Reset();}
  
  bool Reset() { state_.setZero(); error_.setZero(); covariance_.setZero(); return true;}
  
  // Const references
  const Matrix2d&     Covariance() const {return covariance_;}
  
};  // Point2dState

class PointState {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  Vector3d  state_;
  Vector3d  error_;
  Matrix3d  covariance_;
  
  PointState() {Reset();}
  
  bool Reset() { state_.setZero(); error_.setZero(); covariance_.setZero(); return true;}
  
  
};  // PointState

/** Pose state
 *  3D pose
 */
class PoseState {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  // Data stored
  Vector7d state_;        // Current pose state - this updates in time using error state
  Vector6d error_;        // Error state - this is modified by the optimizer
  Quaterniond q0_; Matrix3d R0_;  Vector3d p0_;   // First pose estimates - these do not change
  Matrix6d covariance_;   // Covariance of the error state
  double   information_;  // Single number representing inverse covariance. Used sometimes.
  int64_t  timestamp_;    // Timestamp of the pose
  
  // Mappings
  MapVector4d qv_;  MapVector3d p_;  
  MapVector3d eq_;  MapVector3d ep_;
  
  // Helpers
  Quaterniond q_;  Matrix3d R_;   // Current pose estimate
  Matrix6d inv_sqrt_cov_; 
  Matrix6d correl_; 
  Vector6d sqrt_var_;
  
  PoseState():
    qv_(state_.data()), p_(state_.data()+4), eq_(error_.data()), ep_(error_.data()+3) {
    SetZero();
  }
  
  // Reset
  bool SetZero() {
    state_.setZero();
    state_[3] = 1.;
    error_.setZero();
    covariance_.setZero();
    information_ = 0.;
    timestamp_ = 0;
    inv_sqrt_cov_.setZero(); 
    correl_.setZero(); 
    sqrt_var_.setZero();
    q_ = Quaterniond(qv_);
    R_ = Matrix3d(q_);
    p0_ = p_;
    q0_ = q_;
    R0_ = R_;
    ResetPoseOperationsHelpers();
    return true;
  }
  
  bool SetErrorZero() {
    error_.setZero();
    return true;
  }
  
  // Default copy constructor
  PoseState(const PoseState& r) :
    qv_(state_.data()), p_(state_.data()+4), eq_(error_.data()), ep_(error_.data()+3),
    state_(r.state_), error_(r.error_), covariance_(r.covariance_),
    information_(r.information_), timestamp_(r.timestamp_),
    q_(r.q_), R_(r.R_), q0_(r.q0_), R0_(r.R0_), p0_(r.p0_) {
    ResetPoseOperationsHelpers();
  }
  
  // Equal to assignment operator
  PoseState& operator= (const PoseState& r) {
    state_= r.state_; error_ = r.error_; covariance_ = r.covariance_;
    information_ = r.information_; timestamp_ = r.timestamp_;
    q_ = r.q_; R_ = r.R_; q0_ = r.q0_; R0_ = r.R0_; p0_ = r.p0_;
    ResetPoseOperationsHelpers();
  }
  
  bool SetStartingPose(const Eigen::Quaterniond& q0, const Eigen::Vector3d& p0) {
    q0_ = q0;
    R0_ = Matrix3d(q0_);
    p0_ = p0;
  }
  
  bool UpdateRotations() {
    qv_.normalize();
    q_ = Quaterniond(qv_);
    R_ = Matrix3d(q_);
    return true;
  }
  
  // Reset this pose state to given pose used as starting pose. Errors are set to zero.
  bool SetPose(const Eigen::Quaterniond& q, const Eigen::Vector3d& p) {
    qv_ = q.coeffs();
    UpdateRotations();
    p_ = p;
    SetStartingPose(q_, p_);
    SetErrorZero();
    ResetPoseOperationsHelpers();
    return true;
  }
  
  bool SetPose(const cv::Mat& rotn, const cv::Mat& tr) {
    cv::Size rotn_sz(1,3);
    cv::Size tr_sz(1,3);
    if (rotn.size()!=rotn_sz || tr.size()!=tr_sz) {
      LOG(ERROR) << "rotn or tr sizes are not correct " << rotn.size() << " " << tr.size();
      return false;
    }
    
    Vector3d _r; _r << rotn.at<double>(0), rotn.at<double>(1), rotn.at<double>(2);
    Vector3d _t; _t << tr.at<double>(0), tr.at<double>(1), tr.at<double>(2);
    Eigen::AngleAxisd _aa(_r.norm(), _r.normalized());
    Quaterniond _q(_aa);
    SetZero();
    qv_ = _q.conjugate().coeffs();  // conjugate as pose stores inverse rotation
    UpdateRotations();
    p_ = _t;
    SetStartingPose(q_, p_);
    SetErrorZero();
    ResetPoseOperationsHelpers();    
    return true;
  }
  
  bool SetTimestamp(int64_t ts) {
    timestamp_ = ts;
    return true;
  }
  
  bool SetCovariance(const Eigen::Matrix<double,6,6>& cov) {
    covariance_ = cov;
    ResetPoseOperationsHelpers();
    return CalculateInverseSqrtCovariance();
  }
  
  // Calculate covariance matrix from the ceres problem object
  bool SetCovariance(ceres::Problem* problem) {
    // Check if this intrisics object is a parameter block in this problem
    double* err_ptr = error_.data();
    if (!problem->HasParameterBlock(err_ptr)) {
      LOG(ERROR) << "This camera intrinsics object is not a parameer in this problem. Quit.";
      return false;
    }
    
    // Setup covariance calculation
    ceres::Covariance::Options covariance_options;
    ceres::Covariance covariance(covariance_options);
    std::vector<std::pair<const double*, const double*>> covariance_blocks;
    covariance_blocks.push_back(std::make_pair(err_ptr, err_ptr));
    
    // Compute the covariance matrices
    if (!covariance.Compute(covariance_blocks, problem)) {
      LOG(ERROR) << "Could not compute covariance matrices";
      return false;
    }
    
    // Get the calculated covariance. Returned matrix is row major order
    Matrix6d covmat;
    covariance.GetCovarianceBlock(err_ptr, err_ptr, covmat.data());
    covariance_ = covmat;
    ResetPoseOperationsHelpers();
    return CalculateInverseSqrtCovariance();
  }
  
  bool CalculateInverseSqrtCovariance() {
    if (!InverseSqrt(covariance_, &inv_sqrt_cov_, &sqrt_var_, &correl_)) {
      LOG(ERROR) << "Could not calculate inv sqrt mat";
      return false;
    }
    return true;
  }
  
  // Current estimates
  const MapVector4d&  Qv() const {return qv_;}
  const Quaterniond&  Q() const {return q_;}
  const Matrix3d&     R() const {return R_;}
  const MapVector3d&  P() const {return p_;}
  // First estimates
  const Quaterniond&  Q0() const {return q0_;}
  const Matrix3d&     R0() const {return R0_;}
  const Vector3d&     P0() const {return p0_;}
  // Other pose information
  const int64_t&      Timestamp() const {return timestamp_;}
  const Matrix6d&     Covariance() const {return covariance_;}
  const double&       Information() const {return information_;}
  
  // References
  double&             Information() {return information_;}
  // Copies
  const Matrix3d      VarQ() const {return covariance_.block<3,3>(0,0);}
  const Matrix3d      VarP() const {return covariance_.block<3,3>(3,3);}
  // Mutable pointers
  MapVector4d* Qv_ptr() {return &qv_;}
  MapVector3d* P_ptr() {return &p_;}
  
  // Recalculate the current state = First estimate + error state. Error state is left unchanged.
  bool Recalculate() {
    Quaterniond dq = anantak::ErrorAngleAxisToQuaternion(eq_); // In global frame
    q_ = q0_ * dq;
    q_.normalize();     // not taking the risk of quaternions drifting away from unit norm
    qv_ = q_.coeffs();  // x,y,z,w - this updates the state_
    R_ = Matrix3d(q_);
    p_ = p0_ + ep_;
    // covariance is left untouched
    ResetPoseOperationsHelpers();
    return true;
  }
  
  bool SetInformation(const double info) {
    if (std::isnan(info) || (info < 0.)) {
      LOG(ERROR) << "std::isnan(info) || (info < 0.). Skip.";
      return false;
    }
    information_ = info;
    return true;
  }
  
  bool IsZero() const {
    return (state_.block<3,1>(0,0).isZero() && p_.isZero());
  }
  
  // String representation
  std::string ToString(int detail = 0) const {
    std::ostringstream ss;
    ss << "[" << qv_.transpose() << "] [" << p_.transpose() << "] " << timestamp_;
    if (detail>0) {ss << "\n Covariance = \n" << covariance_;}
    if (detail>1) {
      ss << "\nSqrt var: ";
      ss << sqrt_var_.block<3,1>(0,0).transpose(); ss << ",  ";
      ss << sqrt_var_.block<3,1>(3,0).transpose();
      ss << "\nCorrelation: \n" << correl_.format(CleanFmt1);
      if (!error_.isZero()) {
        ss << "\ndPose: ";
        ss << error_.block<3,1>(0,0).transpose(); ss << ",  ";
        ss << error_.block<3,1>(3,0).transpose();
      }
    }
    return ss.str();
  }
  
  // String representation with angle and distance of the pose
  std::string NormString(int detail = 0) const {
    std::ostringstream ss;
    Eigen::AngleAxisd aa(q_);
    ss << "Axis: " << aa.axis().transpose() << " angle: " << aa.angle()*kDegreesPerRadian
       << ", Vec: " << p_.transpose() << " norm: " << p_.norm();
    return ss.str();    
  }
  
  /** Pose operations helpers **/
  
  bool add_pose_jacobian_is_calculated_;
  Matrix6d dAddPose_dPose;
  Matrix6d dAddPose_dAddPose;
  
  bool ResetPoseOperationsHelpers() {
    add_pose_jacobian_is_calculated_ = false;
    dAddPose_dPose.setZero();
    dAddPose_dAddPose.setZero();
    return true;
  }
  
  // Helper to add a point to this pose
  //  Maths in Notebook #6 pg 67
  inline bool AddPoint(const Vector3d& point,   // point is in pose frame
                       Vector3d* add_point,     // add_point is in reference frame of the pose
                       Matrix3x6d* jac, Matrix3d* add_jac  // Errors are in reference frame of pose
  ) const {
    Vector3d WApB = R_.transpose()*point;
    if (add_point) {
      (*add_point) = p_ + WApB;
    }
    if (jac) {
      jac->block<3,3>(0,0) = SkewSymm3d(WApB);    // Notice no negative sign here
      jac->block<3,3>(0,3) = kIdentity3d;
    }
    if (add_jac) {
      (*add_jac) = R_.transpose();
    }
    return true;
  }
  
  // Helper to get another pose as seen from this one. In other words, diff_pose = pose - this
  //  Maths in notebook#5 pg 33
  inline bool DiffPose(const PoseState& P1, PoseState* P10,
                       Matrix6d* dP10_dP0, Matrix6d* dP10_dP1) const {
    if (!P10) {LOG(ERROR)<<"P10 is null"; return false;}
    Quaterniond BqA = P1.q_ * q_.conjugate();
    Vector3d WApB = P1.p_ - p_;
    Vector3d ApB = R_*WApB;
    P10->SetPose(BqA, ApB);
    if (dP10_dP0) {
      dP10_dP0->setZero();
      dP10_dP0->block<3,3>(0,0) = R_;
      dP10_dP0->block<3,3>(3,3) = R_;
    }
    if (dP10_dP1) {
      dP10_dP1->setZero();
      dP10_dP1->block<3,3>(0,0) = -R_;
      dP10_dP1->block<3,3>(0,0) = -R_;
      dP10_dP1->block<3,3>(3,0) =  R_*SkewSymm3d(WApB);
    }
    return true;
  }
  
  inline bool DiffPose(const PoseState& P1, Vector6d* P10vec,
                       Matrix6d* dP10_dP0, Matrix6d* dP10_dP1) const {
    if (!P10vec) {LOG(ERROR)<<"P10vec is null"; return false;}
    PoseState P10;
    if (!DiffPose(P1, &P10, dP10_dP0, dP10_dP1)) {LOG(ERROR)<<"Could not diff poses"; return false;}
    // Convert the pose state to a residual vector representation
    P10vec->block<3,1>(0,0) = QuaternionToErrorAngleAxis(P10.Q()); // Small angle approximation
    P10vec->block<3,1>(3,0) = P10.P();
    return true;
  }
  
  // Helper to add a pose to this pose, that is expressed relative to this pose
  //  W_P_B = W_P_A + A_P_B    W_P_A = A_q_W  W_p_A   A_P_B = B_q_A  A_p_B   W_P_B = B_q_W  W_p_B
  inline bool AddPose(const Quaterniond& q, const Vector3d& p,
                      Quaterniond* add_q, Vector3d* add_p) const {
    if (add_q) *add_q = q * q_;
    if (add_p) *add_p = p_ + R_.transpose()*p;
    return true;
  }
  
  // Helper to calculate jacobian of this pose in add pose to this pose operation
  inline bool AddPoseJacobian(Matrix6d* jac, Matrix6d* add_jac) {
    if (!add_pose_jacobian_is_calculated_) {
      dAddPose_dPose = Matrix6d::Identity();
      dAddPose_dPose.block<3,3>(3,0) = -SkewSymm3d(R_.transpose()*p_);
      dAddPose_dAddPose.block<3,3>(0,0) = R_.transpose();
      dAddPose_dAddPose.block<3,3>(3,3) = R_.transpose();
      add_pose_jacobian_is_calculated_ = true;
    }
    if (jac) *jac = dAddPose_dPose;
    if (add_jac) *add_jac = dAddPose_dAddPose;
    return true;
  }
  
  // Update pose using another pose using approximate information method
  inline bool UpdatePoseUsingInformation(const PoseState& measurement) {
    // Checks
    if (std::isnan(measurement.information_)) {LOG(ERROR)<<"std::isnan(measurement.information_)"; return false;}
    if (measurement.information_ < kEpsilon) {LOG(ERROR)<<"measurement.information_ < kEpsilon"; return false;}
    if (!measurement.p_.allFinite()) {LOG(ERROR)<<"!measurement.p_.allFinite()"; return false;}
    if (!measurement.qv_.allFinite()) {LOG(ERROR)<<"!measurement.qv_.allFinite()"; return false;}
    if (std::isnan(information_)) {LOG(ERROR)<<"std::isnan(information_)"; *this=measurement; return false;}
    if (information_ < kEpsilon) {LOG(ERROR)<<"information_ < kEpsilon"; *this=measurement; return false;}
    if (!p_.allFinite()) {LOG(ERROR)<<"!p_.allFinite()"; *this=measurement; return false;}
    if (!qv_.allFinite()) {LOG(ERROR)<<"!qv_.allFinite()"; *this=measurement; return false;}
    // New information
    double i0 = measurement.information_;
    double i1 = information_ + i0;
    double info_by_i1 = information_/i1;
    double i0_by_i1 = i0/i1;
    // Position
    p_ = info_by_i1*p_ + i0_by_i1*measurement.p_;
    // Rotation
    Eigen::Matrix4d q_mat  = qv_ * qv_.transpose();
    Eigen::Matrix4d q_mat0 = measurement.qv_ * measurement.qv_.transpose();
    Eigen::Matrix4d rotn_q_mat = info_by_i1*q_mat + i0_by_i1*q_mat0;
    Eigen::EigenSolver<Eigen::Matrix4d> eigen_solver(rotn_q_mat);
    Eigen::Vector4d eval_real = eigen_solver.eigenvalues().real();
    Eigen::Vector4d::Index max_idx; eval_real.maxCoeff(&max_idx);
    Eigen::Vector4d evec_real = eigen_solver.eigenvectors().col(max_idx).real();
    qv_ = evec_real;
    // Information
    information_ = i1;
    // Ending pose opertions
    UpdateRotations();    
    ResetPoseOperationsHelpers();
    return true;
  }
  
  bool CopyToMessage(anantak::PoseStateMessage* pose_msg) const {
    const double* st_data = state_.data();
    for (int i=0; i<state_.size(); i++) pose_msg->add_state(*(st_data+i));
    const double* cov_data = covariance_.data();
    for (int i=0; i<covariance_.size(); i++) pose_msg->add_covariance(*(cov_data+i));    
    return true;
  }
  
  // Creates a sensor message from this state
  bool CopyToMessage(anantak::SensorMsg* msg) const {
    msg->Clear();
    // Build header message
    anantak::HeaderMsg* hdr_msg = msg->mutable_header();
    int64_t tm = 1;
    hdr_msg->set_timestamp(tm);
    hdr_msg->set_type("PoseState");
    hdr_msg->set_recieve_timestamp(tm);
    hdr_msg->set_send_timestamp(tm);
    // Build pose state message
    anantak::PoseStateMessage* pose_msg = msg->mutable_pose_state_msg();
    CopyToMessage(pose_msg);
    return true;
  }
  
  // Creates this state from a sensor message
  bool Create(const anantak::SensorMsg& msg) {
    // Check the message
    if (!msg.has_header()) {LOG(ERROR) << "Msg has no header"; return false;}
    if (!msg.has_pose_state_msg()) {LOG(ERROR) << "Msg has no pose state"; return false;}
    if (msg.header().type()!="PoseState") {LOG(ERROR) << "Msg type is not PoseState"; return false;}
    const anantak::PoseStateMessage& pose_msg = msg.pose_state_msg();
    if (pose_msg.state_size()!=state_.size()) {LOG(ERROR) << "Msg state size != state_.size"; return false;}
    if (pose_msg.covariance_size()!=covariance_.size()) {LOG(ERROR) << "Msg covariance size != covariance size"; return false;}
    
    // All good, proceed to populate the state
    const MapConstVector7d _state(pose_msg.state().data());
    state_ = _state;
    const MapConstMatrix6d _cov(pose_msg.covariance().data());
    covariance_ = _cov;
    error_.setZero();
    UpdateRotations();
    ResetPoseOperationsHelpers();
    
    return true;
  }
  
  bool ToVector(Vector8d* vec, int64_t ref_ts = 0, bool convert_q_to_aa = false) const {
    if (!vec) {LOG(ERROR)<<"Input vec is null"; return false;}
    vec->setZero();
    vec->block<7,1>(0,0) = state_;
    if (convert_q_to_aa) {
      // Convert quaternion to angle axis
      Quaterniond _q(q_);
      //Vector3d q_vec = QuaternionToErrorAngleAxis(_q, true);
      Vector3d q_vec = QuaternionToEulerAngles(_q);
      vec->block<3,1>(0,0) = q_vec;
      (*vec)(3,0) = 0.;
    }
    double ts = double(timestamp_ - ref_ts)*1e-6;
    (*vec)(7,0) = ts;
    return true;
  }
  
  bool FirstEstimateToVector(Vector8d* vec, int64_t ref_ts = 0, bool convert_q_to_aa = false) const {
    if (!vec) {LOG(ERROR)<<"Input vec is null"; return false;}
    vec->setZero();
    vec->block<4,1>(0,0) = q0_.coeffs();
    vec->block<3,1>(4,0) = p0_;
    if (convert_q_to_aa) {
      // Convert quaternion to angle axis
      Quaterniond _q0(q0_);
      //Vector3d q0_vec = QuaternionToErrorAngleAxis(_q0, true);
      Vector3d q0_vec = QuaternionToEulerAngles(_q0);
      vec->block<3,1>(0,0) = q0_vec;
      (*vec)(3,0) = 0.;
    }
    double ts = double(timestamp_ - ref_ts)*1e-6;
    (*vec)(7,0) = ts;
    return true;
  }
  
  virtual ~PoseState() {}
}; // PoseState


/** Kinematic state - a pose, velocity and acceleration
 *  Capturing correlations is crucial
 *  Notes:
 *  - We do not design this as a composition with PoseState because that would not allow us to
 *    have a single covariance matrix with pose, velocity and acceleration.
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
  MapVector6d edwdv_;
  
  // Helpers
  Quaterniond q_;  Matrix3d R_;
  
  KinematicState() :
    qv_(state_.data()),     p_(state_.data()+4),
    w_(state_.data()+7),    v_(state_.data()+10),
    dw_(state_.data()+13),  dv_(state_.data()+16),  t_(state_.data()+19),
    eq_(error_.data()),     ep_(error_.data()+3),
    ew_(error_.data()+6),   ev_(error_.data()+9),
    edw_(error_.data()+12), edv_(error_.data()+15), et_(error_.data()+18),
    edwdv_(error_.data()+12)
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
    edw_(error_.data()+12), edv_(error_.data()+15), et_(error_.data()+18),
    edwdv_(error_.data()+12)
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
  const Matrix6d  CovPose() const {return covariance_.block<6,6>(0,0);}
  const Matrix6d  CovVelocity() const {return covariance_.block<6,6>(6,6);}
  const Matrix6d  CovAcceleration() const {return covariance_.block<6,6>(12,12);}
  
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
    qv_.normalize();
    q_ = Quaterniond(qv_);
    R_ = Matrix3d(q_);
    return true;
  }
  
  bool IsZero() const {
    return (state_.block<3,1>(0,0).isZero() && state_.block<16,1>(4,0).isZero());
  }
  
  bool IsErrorZero() const {
    return error_.isZero();
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
  
  bool SetPose(const PoseState& pose) {
    qv_ = pose.qv_;
    p_ = pose.p_;
    UpdateRotations();
    return true;
  }
  
  bool SetVelocity(const Eigen::Vector3d& w, const Eigen::Vector3d& v) {
    w_ = w;
    v_ = v;
  }
  
  /* Set velocity with a diff wrt another state */
  bool SetVelocity(const PoseState& pose) {
    if (timestamp_==0) {LOG(ERROR) << "timestamp_==0"; return false;}
    if (pose.Timestamp()==0) {LOG(ERROR) << "pose.timestamp_==0"; return false;}
    double dt = double(pose.Timestamp() - timestamp_)*1e-6;
    if (dt<0) {LOG(WARNING)<<"dt<0";}
    Quaterniond _q = pose.Q()*q_.conjugate(); // B'_q_A * A_q_B = B'_q_B
    Eigen::AngleAxisd _aa(_q);
    VLOG(1) << "Velocity dq: " << _q.coeffs().transpose()
        << " aa: " << _aa.axis().transpose() << " angle: " << _aa.angle()*kDegreesPerRadian;
    //Vector3d _w = 2.*R_*_q.vec()/dt;  // Rotating omega from world frame to body frame 
    //Vector3d _w = _aa.axis()*_aa.angle()/dt;
    Vector3d _w = QuaternionToErrorAngleAxis(_q)/dt;           // This vector is expressed in local body frame already
    Vector3d _v = R_*(pose.P() - p_)/dt;    // Rotating velocity to the body frame from world frame
    SetVelocity(_w, _v);
    return true;
  }
  
  // Approximate calculation - only valid for small shifts
  bool ShiftSmallTime(const int64_t dtimestamp) {
    VLOG(1) << "Shifting time by: " << dtimestamp;
    double dt = double(dtimestamp)*1e-6;
    //Vector3d _aa_axis = R_.transpose() * w_ * dt;   // Rotating w from body frame to world frame
    Vector3d _aa_axis = w_ * dt;   // Rotating w from body frame to world frame
    Quaterniond _dq = ErrorAngleAxisToQuaternion(_aa_axis);
    Quaterniond _q(_dq * q_);
    qv_ = _q.coeffs();
    Vector3d _dp = R_.transpose() * v_ * dt;  // Approximate change in position in world frame - negelect rotation.
    p_ += _dp; 
    timestamp_ += dtimestamp;
    UpdateRotations();
    return true;
  }
  
  bool SetAcceleration(const Eigen::Vector3d& dw, const Eigen::Vector3d& dv) {
    dw_ = dw;
    dv_ = dv;
  }
  
  bool SetTime(const double tm) {
    t_(0) = tm;
  }
  
  // Covariance setters
  
  bool SetCovariance(const double rotn_sigma, const double posn_sigma,
                     const double omeg_sigma, const double velo_sigma,
                     const double alph_sigma, const double accl_sigma,
                     const double time_sigma) {
    double r2 = rotn_sigma * rotn_sigma; double p2 = posn_sigma * posn_sigma;
    double w2 = omeg_sigma * omeg_sigma; double v2 = velo_sigma * velo_sigma;
    double h2 = alph_sigma * alph_sigma; double a2 = accl_sigma * accl_sigma;
    double t2 = time_sigma * time_sigma;
    covariance_.setZero();
    covariance_.diagonal() << r2, r2, r2,  p2, p2, p2,
                              w2, w2, w2,  v2, v2, v2,
                              h2, h2, h2,  a2, a2, a2,
                              t2;
    return true;
  }
  
  bool SetCovariance(const Vector7d& sigmas) {
    return SetCovariance(sigmas[0], sigmas[1], sigmas[2], sigmas[3], sigmas[4], sigmas[5], sigmas[6]);
  }
  
  // Calculate covariance matrix from the ceres problem object
  bool SetCovariance(ceres::Problem* problem, bool no_time = true) {
    // Check if this intrisics object is a parameter block in this problem
    double* err_ptr = error_.data();
    if (!problem->HasParameterBlock(err_ptr)) {
      LOG(ERROR) << "This camera intrinsics object is not a parameter in this problem. Quit.";
      return false;
    }
    
    // Setup covariance calculation
    ceres::Covariance::Options covariance_options;
    ceres::Covariance covariance(covariance_options);
    std::vector<std::pair<const double*, const double*>> covariance_blocks;
    covariance_blocks.push_back(std::make_pair(err_ptr, err_ptr));
    
    // Compute the covariance matrices
    if (!covariance.Compute(covariance_blocks, problem)) {
      LOG(ERROR) << "Could not compute covariance matrices";
      return false;
    }
    
    // Get the calculated covariance. Returned matrix is row major order
    if (no_time) {
      Matrix18d covmat;
      covariance.GetCovarianceBlock(err_ptr, err_ptr, covmat.data());
      covariance_.block<18,18>(0,0) = covmat;      
    } else {
      Matrix19d covmat;
      covariance.GetCovarianceBlock(err_ptr, err_ptr, covmat.data());
      covariance_ = covmat;
    }
    VLOG(2) << "Calculated sqrt covariance diagonal of state at timestamp: " << timestamp_
        << "\n  " << covariance_.diagonal().cwiseSqrt().transpose();
    return true;
  }
  
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
  
  // Recalculate the state from error state after optimization. Set errors to zero.
  bool Recalculate() {
    Quaterniond dq = anantak::ErrorAngleAxisToQuaternion(eq_); // In global frame
    q_ *= dq;  // assuming q_ and dq are already normalized
    qv_ = q_.coeffs(); // x,y,z,w
    p_ += ep_;
    w_ += ew_;   v_ += ev_;
    dw_ += edw_; dv_ += edv_;
    t_ += et_;
    error_.setZero();
    // covariance is left untouched
    UpdateRotations();
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
    const MapConstMatrix19d _cov(kin_msg.covariance().data());
    covariance_ = _cov;
    error_.setZero();
    UpdateRotations();
    
    return true;
  }
  
  std::string ToString(int detail = 0) const {
    std::ostringstream ss;
    if (detail==0) {
      ss << timestamp_ << ", " << qv_.transpose() << ", " << p_.transpose();
    } else if (detail==1) {
      ss << timestamp_ << ", " << qv_.transpose() << ", " << p_.transpose() << "\n";
      ss << w_.transpose() << " " << v_.transpose() << ", " << dw_.transpose() << " " << dv_.transpose();
    } else if (detail==2) {
      ss << "timestamp: " << timestamp_ << "\n"
        << "  q:  " << qv_.transpose() << "  ~  " << VarQ().diagonal().transpose() << "\n"
        << "  p:  " << p_.transpose() << "  ~  " << VarP().diagonal().transpose() << "\n"
        << "  w:  " << w_.transpose() << "  ~  " << VarW().diagonal().transpose() << "\n"
        << "  v:  " << v_.transpose() << "  ~  " << VarV().diagonal().transpose() << "\n"
        << "  dw: " << dw_.transpose() << "  ~  " << VardW().diagonal().transpose() << "\n"
        << "  dv: " << dv_.transpose() << "  ~  " << VardV().diagonal().transpose() << "\n"
        << "  t:  " << t_.transpose() << "  ~  " << VarT().diagonal().transpose();      
    } else {
      if (detail>0) {ss << "\n timestamp = \n";}
      ss << timestamp_ << ", ";
      if (detail>0) {ss << "\n pose = \n";}
      ss << qv_.transpose() << ", " << p_.transpose();
      if (detail>0) {ss << "\n velocity = \n" << w_.transpose() << ", " << v_.transpose();}
      if (detail>0) {ss << "\n acceleration = \n" << dw_.transpose() << ", " << dv_.transpose();}
      if (detail>1) {ss << "\n rotation covariance = \n" << VarQ() ;}
      if (detail>1) {ss << "\n position covariance = \n" << VarP() ;}
    }
    return ss.str();
  }
  
  Vector39d ToVector(bool convert_q_to_aa = false) const {
    Vector39d v;
    v.block<20,1>(0,0) = state_;
    v.block<19,1>(20,0) = covariance_.diagonal().cwiseSqrt();
    if (convert_q_to_aa) {
      // Convert quaternion to angle axis
      Quaterniond _q(q_);
      //Vector3d q_vec = QuaternionToErrorAngleAxis(_q, true);
      Vector3d q_vec = QuaternionToEulerAngles(_q);
      v.block<3,1>(0,0) = q_vec;
      v(3,0) = 0.;
    }
    return v;
  }
  
  bool ToPose(PoseState* pose) const {
    pose->SetPose(q_, p_);
    Matrix6d _cov = covariance_.block<6,6>(0,0);
    pose->SetCovariance(_cov);
    return true;
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
/*inline bool KinMotionInTime(const Vector3d& w0, const Vector3d& dw0, const Vector3d& dv0,
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
}*/


/* Move a kinematic state in time - assumes constant angular and linear accelerations
 *  In: Kinematic state to be moved in time, time interval of move, variance of interval
 *  Out: Modify another state that is ahead in time
 *  Notes: Timestamp of the pose is not modified
 */ 
/*inline bool MoveInTime(const KinematicState& K0, const double t1, const double t1_var,
                       KinematicState* K1, Matrix19d* dK1_dK0, Vector19d* dK1_dt1) {
  
  if (!K1) {LOG(ERROR) << "K1 is null"; return false;}
  if (!dK1_dK0 || !dK1_dt1) {LOG(ERROR) << "dK1_dK0 / dK1_dt1 are null"; return false;}
  
  // Double integrate the rotation, t*SkewSymm(Rotation*a), 0.5*t*t*SkewSymm(Rotation*a)
  Matrix9x3d II_kinmotion, I_kinmotion, kinmotion;
  II_kinmotion.setZero(); I_kinmotion.setZero(); kinmotion.setZero(); 
  std::function<bool(const double&, Matrix9x3d&)>
      kinmotion_integral = std::bind(KinMotionInTime, K0.w_, K0.dw_, K0.dv_,
                                     std::placeholders::_1, std::placeholders::_2);
  DoubleIntegralFunction<Matrix9x3d>(kinmotion_integral, t1, II_kinmotion, I_kinmotion, kinmotion);
  VLOG(3) << "Kin motion integrals, \n II = \n" << II_kinmotion << "\n I = \n" << I_kinmotion
      << "\n R = \n" << kinmotion;
  
  // Helpers
  Matrix3d A_R_B0 = K0.R().transpose();
  Matrix3d t1_Identity = t1 * kIdentity3d;
  Matrix1d t1_var_mat; t1_var_mat << t1_var;
  
  // Calculate estimates
  Quaterniond Bt_q_A(Eigen::Quaterniond(kinmotion.block<3,3>(0,0).transpose())*K0.Q());
  Bt_q_A.normalize(); 
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
  dK1_dK0->setZero();
  dK1_dt1->setZero();
  // A_eR_Bt: four terms, all positive
  dK1_dK0->block<3,3>(0,0) =   kIdentity3d;
  dK1_dK0->block<3,3>(0,6) =   t1 * A_R_B0;
  dK1_dK0->block<3,3>(0,12) =  0.5 * t1 * dK1_dK0->block<3,3>(0,6);
  dK1_dt1->block<3,1>(0,0) =   A_R_B0 * Bt_w_B;
  // A_ep_Bt: seven terms, three negative, four positive
  dK1_dK0->block<3,3>(3,0) =  -SkewSymm3d(A_B0_p_Bt);
  dK1_dK0->block<3,3>(3,3) =   kIdentity3d;
  dK1_dK0->block<3,3>(3,6) =  -A_R_B0 * II_kinmotion.block<3,3>(3,0);
  dK1_dK0->block<3,3>(3,9) =   dK1_dK0->block<3,3>(0,6);   // t1 * A_R_B0
  dK1_dK0->block<3,3>(3,12) = -A_R_B0 * II_kinmotion.block<3,3>(6,0);
  dK1_dK0->block<3,3>(3,15) =  A_R_B0 * II_kinmotion.block<3,3>(0,0);
  dK1_dt1->block<3,1>(3,0) =   A_R_B0 * ( K0.V() + I_kinmotion.block<3,3>(0,0)*K0.dV() );
  // Bt_ew_B: three terms
  dK1_dK0->block<3,3>(6,6) =   kIdentity3d;
  dK1_dK0->block<3,3>(6,12) =  t1_Identity;
  dK1_dt1->block<3,1>(6,0) =   K0.dW();
  // Bt_ev_B: three terms
  dK1_dK0->block<3,3>(9,9) =   kIdentity3d;
  dK1_dK0->block<3,3>(9,15) =  t1_Identity;
  dK1_dt1->block<3,1>(9,0) =   K0.dV();
  // Bt_edw_B: one term
  dK1_dK0->block<3,3>(12,12) = kIdentity3d;
  // Bt_edv_B: one term
  dK1_dK0->block<3,3>(15,15) = kIdentity3d;
  // et: two terms
  (*dK1_dK0)(18,18) = 1.;
  (*dK1_dt1)(18,0) =  1.;
  
  // Calculate variance and copy to the state
  K1->covariance_ = (*dK1_dK0) * K0.covariance_ * dK1_dK0->transpose()
                  + (*dK1_dt1) *   t1_var_mat   * dK1_dt1->transpose();
  
  return true;
}*/



/** Move in time assuming constant acceleration during time t1.
 * t1 is deterministic, it has no variance.
 * Maths is in Notebook #5 pg 87 */
inline bool MoveInFixedTime(const KinematicState& K0,
                            const double t1,  // t1 has no variance
                            const Matrix6d* Cov_dw_dv, 
                            Vector18d* K1_K0, Matrix18d* dK1_dK0, Matrix18d* dK1_cov) {
  
  double t01 = t1*0.25;  Vector3d a01 = t01*K0.w_ + 0.5*t01*t01*K0.dw_;
  double t02 = t1*0.50;  Vector3d a02 = t02*K0.w_ + 0.5*t02*t02*K0.dw_;  Vector3d v02 = K0.v_ + t02*K0.dv_;
  double t03 = t1*0.75;  Vector3d a03 = t03*K0.w_ + 0.5*t03*t03*K0.dw_;
  double t04 = t1;       Vector3d a04 = t04*K0.w_ + 0.5*t04*t04*K0.dw_;  Vector3d v04 = K0.v_ + t04*K0.dv_;
  
  // Rotation matrices for times t0, t1/4, t1/2, t3/4, t1 - using small angle approximation
  Matrix3d R00 = kIdentity3d;
  Matrix3d R01 = kIdentity3d - SkewSymm3d(a01);
  Matrix3d R02 = kIdentity3d - SkewSymm3d(a02);
  Matrix3d R03 = kIdentity3d - SkewSymm3d(a03);
  Matrix3d R04 = kIdentity3d - SkewSymm3d(a04);
  
  Matrix3d IR_02 =  t01/3.*(R00.transpose() + 4.*R01.transpose() + R02.transpose());
  Matrix3d IR_04 =  IR_02 + t01/3.*(R02.transpose() + 4.*R03.transpose() + R04.transpose());
  
  Matrix3d IR1_02 =  t01/3.*(4.*t01*R01.transpose() + t02*R02.transpose());
  Matrix3d IR1_04 =  IR1_02 + t01/3.*(t02*R02.transpose() + 4.*t03*R03.transpose() + t04*R04.transpose());
  
  //  Kin error has the following blocks: eq, ep, ew, ev, edw, edv,  et
  //  Indexes are as follows:              0,  3,  6,  9,  12,  15,  18
  
  // Prediction: change in K0. K1 - K0 = K1_K0
  if (K1_K0) {
    K1_K0->setZero();                                     // dw dv do not change
    K1_K0->block<3,1>(0,0) = a04;                         // Bt_R_B0's angle axis of rotation
    K1_K0->block<3,1>(3,0) = IR_04*K0.v_ + IR1_04*K0.dv_; // B0_p_Bt
    K1_K0->block<3,1>(6,0) = t04*K0.dw_;                  // delta w
    K1_K0->block<3,1>(9,0) = t04*K0.dv_;                  // delta v
  }
  
  // Jacobian
  if (dK1_dK0) {
    
    Matrix3d B0_v00x = SkewSymm3d(K0.v_);
    Matrix3d B0_v02x = SkewSymm3d(R02.transpose() * v02);
    Matrix3d B0_v04x = SkewSymm3d(R04.transpose() * v04);
    
    Matrix3d IP_04  = t02/3.*(B0_v00x + 4.*B0_v02x + B0_v04x);
    Matrix3d IP1_04 = t02/3.*(4.*B0_v02x*IR_02 + B0_v04x*IR_04);
    Matrix3d IP2_04 = t02/3.*(4.*B0_v02x*IR1_02 + B0_v04x*IR1_04);
    
    Matrix3d t1_Identity3d = t04 * kIdentity3d;
    
    dK1_dK0->setZero();
    // B0_eq_Bt
    dK1_dK0->block<3,3>(0,0)   =  kIdentity3d;    // B0_eq
    dK1_dK0->block<3,3>(0,6)   = -IR_04;          // B0_ew
    dK1_dK0->block<3,3>(0,12)  = -IR1_04;         // B0_edw
    // B0_ep_Bt 
    dK1_dK0->block<3,3>(3,0)   = -IP_04;          // B0_eq_B0
    dK1_dK0->block<3,3>(3,3)   =  kIdentity3d;    // B0_ep
    dK1_dK0->block<3,3>(3,6)   =  IP1_04;         // B0_w
    dK1_dK0->block<3,3>(3,9)   =  IR_04;          // B0_v
    dK1_dK0->block<3,3>(3,12)  =  IP2_04;         // B0_dw
    dK1_dK0->block<3,3>(3,15)  =  IR1_04;         // B0_dv
    // B0_w
    dK1_dK0->block<3,3>(6,6)   =  kIdentity3d;    // B0_w
    dK1_dK0->block<3,3>(6,12)  =  t1_Identity3d;  // B0_dw
    // B0_v
    dK1_dK0->block<3,3>(9,9)   =  kIdentity3d;    // B0_v
    dK1_dK0->block<3,3>(9,15)  =  t1_Identity3d;  // B0_dv
    // B0_dw
    dK1_dK0->block<3,3>(12,12) =  kIdentity3d;    // B0_dw
    // B0_dv
    dK1_dK0->block<3,3>(15,15) =  kIdentity3d;    // B0_dv
    
    // Conditional covariance of prediction given B0 due to process error
    if (Cov_dw_dv && dK1_cov) {
      *dK1_cov = dK1_dK0->block<18,6>(0,12) * (*Cov_dw_dv) * dK1_dK0->block<18,6>(0,12).transpose();
    }
  }
  
  return true;
}

// Move in time from state to create another state 
inline bool MoveInFixedTime(const KinematicState& K0,
                            const int64_t t1_musec,  // t1 has no variance
                            KinematicState* K1) {
  
  if (!K0.error_.isZero()) {LOG(WARNING)<<"Projecting a state with non zero error! "
      << "Error will be neglected in projection.";}
  
  //KinematicState K0(_K0);  // Copy
  //if (!K0.error_.isZero()) {K0.Recalculate();}
  
  double dt = double(t1_musec)*1e-6;    // change in time
  Vector18d K1_K0;                      // K1 as seen from K0
  if (!MoveInFixedTime(K0, dt, nullptr, &K1_K0, nullptr, nullptr)) {
    LOG(ERROR) << "Could not move in time"; return false;}
  Vector3d Bt_dq_B0_vec(K1_K0.block<3,1>(0,0));
  Quaterniond Bt_dq_B0 = ErrorAngleAxisToQuaternion(Bt_dq_B0_vec);
  Quaterniond Bt_q_A = Bt_dq_B0 * K0.Q();
  Vector3d A_p_Bt = K0.R().transpose()*K1_K0.block<3,1>(3,0) + K0.P();
  Vector3d w = K0.W() + K1_K0.block<3,1>(6,0);
  Vector3d v = K0.V() + K1_K0.block<3,1>(9,0);
  Vector3d dw = K0.dW() + K1_K0.block<3,1>(12,0);
  Vector3d dv = K0.dV() + K1_K0.block<3,1>(15,0);
  int64_t timestamp = K0.Timestamp() + t1_musec;
  double t = K0.T()(0) + dt;
  
  K1->SetTimestamp(timestamp);
  K1->SetPose(Bt_q_A, A_p_Bt);
  K1->SetVelocity(w, v);
  K1->SetAcceleration(dw, dv);
  K1->SetTime(t);
  // Covariance is left alone
  
  return true;  
}

/** Calculates K1 - K0, or K1 seen from K0. Notebook #5 pg 87 */
inline bool DiffKinStates(const KinematicState& K0, const KinematicState& K1, Vector19d* diff) {
  if (!diff) {LOG(ERROR)<<"diff is null. Quit."; return false;}
  diff->setZero();
  // Diff the quaternion - result expressed in B0 frame
  Quaterniond dq = K1.Q()*K0.Q().conjugate();     // Bt_dq_B0 = Bt_q_A * A_q_B0   
  diff->block<3,1>(0,0) = QuaternionToErrorAngleAxis(dq);            // Small angle approximation for angle axis
  // Diff the position - result in B0 frame
  diff->block<3,1>(3,0) = K0.R()*(K1.P()-K0.P()); // B0_dp_Bt = B0_R_A*(A_p_Bt - A_p_B0)
  //  We do not need to use angular acceleration summation rule here (dw2 = dw1 + dw1xdw2)
  //  because we are just looking at the error in numbers
  diff->block<13,1>(6,0) = K1.state_.block<13,1>(7,0) - K0.state_.block<13,1>(7,0);
  return true;
}

inline bool DiffKinWithPose(const KinematicState& K0, const PoseState& P1, Vector6d* diff) {
  diff->setZero();
  // Diff the quaternion - result expressed in B0 frame
  Quaterniond dq = P1.Q()*K0.Q().conjugate();     // Bt_dq_B0 = Bt_q_A * A_q_B0   
  diff->block<3,1>(0,0) = QuaternionToErrorAngleAxis(dq);            // Small angle approximation for angle axis
  // Diff the position - result in B0 frame
  diff->block<3,1>(3,0) = K0.R()*(P1.P()-K0.P()); // B0_dp_Bt = B0_R_A*(A_p_Bt - A_p_B0)
  return true;  
}

/**Another interface to moving in time with deterministic time interval
inline bool MoveInTime(const KinematicState& K0, const int64_t dt,
                       KinematicState* K1, Matrix19d* dK1_dK0, Vector19d* dK1_dt1) {
  K1->SetTimestamp(K0.Timestamp() + dt);
  return MoveInTime(K0, double(dt)*1e-6, 0., K1, dK1_dK0, dK1_dt1);
}*/


/* Move a kinematic state in time relative to K0 - assumes constant angular and linear accelerations
 *  In: Kinematic state to be moved in time, time interval of move, variance of interval
 *  Out: Modify another state that is ahead in time
 *  Notes:
 *    - K0 covariance does not affect relative motion covariance. Only process noise affects it.
 *    - Timestamp of the pose is not modified
 *
inline bool RelativeMoveInTime(
    const KinematicState& K0, const Matrix6d& K0_accl_cov, const double t1, const double t1_var,
    KinematicState* K1, Matrix19x6d* dK1_dK0, Vector19d* dK1_dt1) {
  
  if (!K1) {LOG(ERROR) << "K1 is null"; return false;}
  if (!dK1_dK0 || !dK1_dt1) {LOG(ERROR) << "dK1_dK0 / dK1_dt1 are null"; return false;}
  
  // Double integrate the rotation, t*SkewSymm(Rotation*a), 0.5*t*t*SkewSymm(Rotation*a)
  Matrix9x3d II_kinmotion, I_kinmotion, kinmotion;
  II_kinmotion.setZero(); I_kinmotion.setZero(); kinmotion.setZero(); 
  std::function<bool(const double&, Matrix9x3d&)>
      kinmotion_integral = std::bind(KinMotionInTime, K0.w_, K0.dw_, K0.dv_,
                                     std::placeholders::_1, std::placeholders::_2);
  DoubleIntegralFunction<Matrix9x3d>(kinmotion_integral, t1, II_kinmotion, I_kinmotion, kinmotion);
  VLOG(3) << "Kin motion integrals, \n II = \n" << II_kinmotion << "\n I = \n" << I_kinmotion
      << "\n R = \n" << kinmotion;
  
  // Helpers
  Matrix3d A_R_B0 = K0.R().transpose();
  Matrix3d t1_Identity = t1 * kIdentity3d;
  Matrix1d t1_var_mat(t1_var);
  
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
  dK1_dK0->setZero();
  dK1_dt1->setZero();
  // A_eR_Bt: four terms, all positive
  //dK1_dK0->block<3,3>(0,-12) =   kIdentity3d;
  //dK1_dK0->block<3,3>(0,-6) =    t1 * A_R_B0;
  dK1_dK0->block<3,3>(0,0) =   0.5 * t1 * t1 * A_R_B0;
  dK1_dt1->block<3,1>(0,0) =   A_R_B0 * Bt_w_B;
  // A_ep_Bt: seven terms, three negative, four positive
  //dK1_dK0->block<3,3>(3,-12) =  -SkewSymm3d(A_B0_p_Bt);
  //dK1_dK0->block<3,3>(3,-9) =    kIdentity3d;
  //dK1_dK0->block<3,3>(3,-6) =   -A_R_B0 * II_kinmotion.block<3,3>(3,0);
  //dK1_dK0->block<3,3>(3,-3) =    dK1_dK0.block<3,3>(0,6);   // t1 * A_R_B0
  dK1_dK0->block<3,3>(3,0) = -A_R_B0 * II_kinmotion.block<3,3>(6,0);
  dK1_dK0->block<3,3>(3,3) =  A_R_B0 * II_kinmotion.block<3,3>(0,0);
  dK1_dt1->block<3,1>(3,0) =  A_R_B0 * ( K0.V() + I_kinmotion.block<3,3>(0,0)*K0.dV() );
  // Bt_ew_B: three terms
  //dK1_dK0.block<3,3>(6,-6) =   kIdentity3d;
  dK1_dK0->block<3,3>(6,0) =   t1_Identity;
  dK1_dt1->block<3,1>(6,0) =   K0.dW();
  // Bt_ev_B: three terms
  //dK1_dK0->block<3,3>(9,-3) =   kIdentity3d;
  dK1_dK0->block<3,3>(9,3) =  t1_Identity;
  dK1_dt1->block<3,1>(9,0) =   K0.dV();
  // Bt_edw_B: one term
  dK1_dK0->block<3,3>(12,0) = kIdentity3d;
  // Bt_edv_B: one term
  dK1_dK0->block<3,3>(15,3) = kIdentity3d;
  // et: two terms
  (*dK1_dK0)(18,6) =  1.;
  (*dK1_dt1)(18,0) =  1.;
  
  // Calculate variance and copy directly
  K1->covariance_ = (*dK1_dK0) *   K0_accl_cov  * dK1_dK0->transpose()
                  + (*dK1_dt1) *   t1_var_mat   * dK1_dt1->transpose();
  
  return true;
}*/


/* Change in kinematic state in a fixed time interval - with constant angular and linear accelerations
 *  In: Kinematic state to be moved in time, time interval of move
 *  Out: K1 is the forecast state
 *  Notes:
 *    - K0 covariance does not affect relative motion covariance. Only process noise affects it.
 *    - Timestamp of the pose is not modified
inline bool RelativeMoveInFixedTime(
    const KinematicState& K0, const Matrix6d& K0_accl_cov, const double t1, //const double t1_var,
    KinematicState* K1, Matrix18d* dK1_dK0, Vector18d* dK1_dt1) {
  
  if (!K1) {LOG(ERROR) << "K1 is null"; return false;}
  if (!dK1_dK0 || !dK1_dt1) {LOG(ERROR) << "dK1_dK0 / dK1_dt1 are null"; return false;}
  
  // Double integrate the rotation, t*SkewSymm(Rotation*a), 0.5*t*t*SkewSymm(Rotation*a)
  Matrix9x3d II_kinmotion, I_kinmotion, kinmotion;
  II_kinmotion.setZero(); I_kinmotion.setZero(); kinmotion.setZero(); 
  std::function<bool(const double&, Matrix9x3d&)>
      kinmotion_integral = std::bind(KinMotionInTime, K0.w_, K0.dw_, K0.dv_,
                                     std::placeholders::_1, std::placeholders::_2);
  DoubleIntegralFunction<Matrix9x3d>(kinmotion_integral, t1, II_kinmotion, I_kinmotion, kinmotion);
  VLOG(3) << "Kin motion integrals, \n II = \n" << II_kinmotion << "\n I = \n" << I_kinmotion
      << "\n R = \n" << kinmotion;
  
  // Helpers
  Matrix3d A_R_B0 = K0.R().transpose();
  Matrix3d t1_Identity = t1 * kIdentity3d;
  //Matrix1d t1_var_mat(t1_var);
  
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
  //  Kin covariance has the following blocks: eq, ep, ew, ev, edw, edv, // et
  //  Indexes are as follows:                   0,  3,  6,  9,  12,  15, // 18
  dK1_dK0->setZero();
  dK1_dt1->setZero();
  // A_eR_Bt: four terms, all positive
  //dK1_dK0->block<3,3>(0,-12) =   kIdentity3d;
  //dK1_dK0->block<3,3>(0,-6) =    t1 * A_R_B0;
  dK1_dK0->block<3,3>(0,0) =   0.5 * t1 * t1 * A_R_B0;
  dK1_dt1->block<3,1>(0,0) =   A_R_B0 * Bt_w_B;
  // A_ep_Bt: seven terms, three negative, four positive
  //dK1_dK0->block<3,3>(3,-12) =  -SkewSymm3d(A_B0_p_Bt);
  //dK1_dK0->block<3,3>(3,-9) =    kIdentity3d;
  //dK1_dK0->block<3,3>(3,-6) =   -A_R_B0 * II_kinmotion.block<3,3>(3,0);
  //dK1_dK0->block<3,3>(3,-3) =    dK1_dK0.block<3,3>(0,6);   // t1 * A_R_B0
  dK1_dK0->block<3,3>(3,0) = -A_R_B0 * II_kinmotion.block<3,3>(6,0);
  dK1_dK0->block<3,3>(3,3) =  A_R_B0 * II_kinmotion.block<3,3>(0,0);
  dK1_dt1->block<3,1>(3,0) =  A_R_B0 * ( K0.V() + I_kinmotion.block<3,3>(0,0)*K0.dV() );
  // Bt_ew_B: three terms
  //dK1_dK0.block<3,3>(6,-6) =   kIdentity3d;
  dK1_dK0->block<3,3>(6,0) =   t1_Identity;
  dK1_dt1->block<3,1>(6,0) =   K0.dW();
  // Bt_ev_B: three terms
  //dK1_dK0->block<3,3>(9,-3) =   kIdentity3d;
  dK1_dK0->block<3,3>(9,3) =  t1_Identity;
  dK1_dt1->block<3,1>(9,0) =   K0.dV();
  // Bt_edw_B: one term
  dK1_dK0->block<3,3>(12,0) = kIdentity3d;
  // Bt_edv_B: one term
  dK1_dK0->block<3,3>(15,3) = kIdentity3d;
  // et: two terms
  (*dK1_dK0)(18,6) =  1.;
  (*dK1_dt1)(18,0) =  1.;
  
  // Calculate variance and copy directly
  K1->covariance_ = (*dK1_dK0) *   K0_accl_cov  * dK1_dK0->transpose()
                  + (*dK1_dt1) *   t1_var_mat   * dK1_dt1->transpose();
  
  return true;
}*/


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
                        KinematicState* AKCB, bool express_in_C_frame = false) {
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
                              KinematicState* Ki) {
  return true;
}


/** Inter Kinematic State Residual
 *  Connects two kinematic states assuming constant acceleration. This residual:
 *  - Predicts the ending state
 *  - Calculates ending state variance
 *  - Provides residual evaluation
 *
 *  First state should be initiated. It will be used to move forward in time to second state.
 *  Second state will be overwritten by default, unless asked not to.
 *  Starting residual will be calculated using the two states.
 *  Variance of the residual is the predicted variance of the ending state.
 */
/** class InterKinematicStateResidual : public ceres::SizedCostFunction<19,19,19> { // R, K0, K1
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  struct Options {
    Matrix6d   acceleration_covariance;     // inadequate model fit for acceleration
    Matrix19d  kinematic_state_covariance;  // all zero but for acceleration
    Options(const double edw, const double edv) {
      double edw2 = edw*edw;
      double edv2 = edv*edv;
      Vector6d accl_cov_diag; accl_cov_diag << edw2, edw2, edw2, edv2, edv2, edv2;
      acceleration_covariance = accl_cov_diag.asDiagonal();
      kinematic_state_covariance.setZero();
      kinematic_state_covariance.block<6,6>(12,12) = acceleration_covariance;
    }
  };  // Options
  // Options set the process noise
  const InterKinematicStateResidual::Options* options_;
  
  // States
  KinematicState* K0_;          // Starting state
  KinematicState* K1_;          // Ending state
  double  t1_;                  // Time interval between states in seconds
  double  t1_var_;              // Variance of time interval
  
  // Residual and Jacobians
  Vector19d residual_;          // Residual = K1_prediction - K1_
  Matrix19d dK1_dK0_;           // Jacobian of residual (or K1_prediction) wrt dK0
  Matrix19d dK1_dK1_;           // Jacobian of residual wrt dK1
  Vector19d dK1_dt1_;           // Jacobian of residual (or K1_prediction) wrt dt1
  
  // Helper
  int64_t t1_musec_;            // Time interval between states in micro seconds
  bool overwrite_K1_;             // Should K1 be overwritten?
  
  // Constructor
  InterKinematicStateResidual(const InterKinematicStateResidual::Options* options):
      options_(options),
      K0_(nullptr), K1_(nullptr), t1_musec_(0), t1_(0.), t1_var_(0.), overwrite_K1_(true) {
    Reset();
  }
  
  bool Reset() {
    // options are not modified
    if (options_->acceleration_covariance.isZero()) {LOG(WARNING)<<"Options accl cov is zero!";}
    K0_ = nullptr; K1_ = nullptr;
    t1_musec_ = 0; t1_ = 0.; t1_var_ = 0.; overwrite_K1_ = true;
    residual_.setZero(); dK1_dK0_.setZero(); dK1_dK1_.setZero(); dK1_dt1_.setZero();
    return true;
  }
  
  // Options are reset separately from Create call
  bool SetOptions(const InterKinematicStateResidual::Options* options) {
    if (!options) {LOG(ERROR)<<"Options pointer is null"; return false;}
    if (options->acceleration_covariance.isZero()) {LOG(WARNING)<<"Options accl cov is zero.";}
    options_ = options;
    return true;
  }
  
  // Copy constructor
  InterKinematicStateResidual(const InterKinematicStateResidual& r) :
      options_(nullptr),
      K0_(nullptr), K1_(nullptr), t1_musec_(0), t1_(0.), t1_var_(0.), overwrite_K1_(true) {
    options_ = r.options_;
    K0_ = r.K0_; K1_ = r.K1_;
    t1_musec_ = r.t1_musec_; t1_ = r.t1_; t1_var_ = r.t1_var_; overwrite_K1_ = r.overwrite_K1_;
    residual_ = r.residual_; dK1_dK0_ = r.dK1_dK0_; dK1_dK1_ = r.dK1_dK1_; dK1_dt1_ = r.dK1_dt1_;
  }
  
  // Equal to assignment operator
  InterKinematicStateResidual& operator= (const InterKinematicStateResidual& r) {
    options_ = r.options_;
    K0_ = r.K0_; K1_ = r.K1_;
    t1_musec_ = r.t1_musec_; t1_ = r.t1_; t1_var_ = r.t1_var_; overwrite_K1_ = r.overwrite_K1_;
    residual_ = r.residual_; dK1_dK0_ = r.dK1_dK0_; dK1_dK1_ = r.dK1_dK1_; dK1_dt1_ = r.dK1_dt1_;
  }
  
  // Create the residual given two consecutive kinematic states
  bool Create(KinematicState* K0, KinematicState* K1, const int64_t t1_musec,
              const double t1_var=0, bool overwrite_K1=true, bool zero_K0_is_ok = false) {
    if (!options_) {LOG(ERROR)<<"Options have not been set. Quit."; return false;}
    if (!K0) {LOG(ERROR)<<"K0 is null"; return false;}
    if (!zero_K0_is_ok && K0->IsZero()) {LOG(ERROR)<<"K0 is zero. Set zero_K0_is_ok to true"; return false;}
    if (!K1) {LOG(ERROR)<<"K1 is null"; return false;}
    if (t1_musec<1) {LOG(WARNING)<<"t1<1";}
    if (t1_var<0.) {LOG(ERROR)<<"var_t1<0."; return false;}
    if (overwrite_K1 && !K1->IsZero()) {LOG(WARNING)<<"To be predicted state K1 is not zero";}
    
    K0_ = K0;
    K1_ = K1;
    t1_musec_ = t1_musec;
    t1_ = double(t1_musec_)*1e-6;
    t1_var_ = t1_var;
    overwrite_K1_ = overwrite_K1;
    
    if (!CalculateResiduals()) {
      LOG(ERROR) << "Could not calculate residuals";
      return false;
    }
    
    return true;
  }

  // Calculate residuals, jacobians and variances.
  //  First estimates are used for jacobians, residual is recalculated but jacobians are not
  bool CalculateResiduals() {
    
    // Predict: move in time from K0. Variance is accumulated only over the interval t1.
    //  Variance of K0 (serving as prior) does not effect the residual.
    //  Only the uncertainty in pose and velocity due to uncertainty in acceleration
    //  (process noise) and time interval affect residual variance
    KinematicState  K1_prediction;
    K1_prediction.SetTimestamp(K0_->Timestamp() + t1_musec_);
    
    const KinematicState& K0 = *K0_;
    if (!MoveInTime(K0, //options_->acceleration_covariance,
                    t1_, t1_var_,
                    &K1_prediction, &dK1_dK0_, &dK1_dt1_)) {
      LOG(ERROR) << "MoveInTime calculation was not successful";
      return false;
    }
    dK1_dK1_ = -1.*Matrix19d::Identity();
    
    // Calculate residual = Prediction - Measurement = K1_prediction - K1_
    if (overwrite_K1_) {
      // No residual with prediction
      residual_.setZero();
    } else {
      // Diff state wrt value in K1_
      const KinematicState& K1 = *K1_;
      if (!DiffKinStates(K1, K1_prediction, &residual_)) {
        LOG(ERROR) << "DiffKinStates to calculate residual had an error. Quit.";
        return false;
      }
      // Should we add the covariance of K1_ to residual?
      if (!K1_->covariance_.allFinite()) {
        LOG(WARNING) << "!K1_->covariance_.allFinite()\n" << K1_->covariance_;
      } else {
        K1_prediction.covariance_ += K1_->covariance_;
      }
    }
    
    // Calculate inverse sqrt covariance of residual
    Matrix19d inv_sqrt_cov;
    // If t1 is deterministic, do not include it in inverse sqrt cov calculation
    if (K1_prediction.covariance_(18,18) < kEpsilon) {
      // t1 is deterministic
      VLOG(1) << "t1 is deterministic " << K1_prediction.covariance_(18,18);
      Matrix18d cov_no_t1; cov_no_t1 = K1_prediction.covariance_.block<18,18>(0,0);
      Matrix18d inv_sqrt_cov_no_t1;
      Vector18d sqrt_var_no_t1;
      if (!InverseSqrt(cov_no_t1, &inv_sqrt_cov_no_t1, &sqrt_var_no_t1)) {
        LOG(ERROR) << "Could not calculate inverse sqrt covariance";
        return false;
      }
      inv_sqrt_cov.block<18,18>(0,0) = inv_sqrt_cov_no_t1;
      inv_sqrt_cov.block<18,1>(0,18).setZero();
      inv_sqrt_cov.block<1,19>(18,0).setZero();
      //sqrt_var.block<18,1>(0,0) = sqrt_var_no_t1;
      //sqrt_var(18,0) = 0;
    } else {
      // t1 is stochastic
      VLOG(1) << "t1 is stochastic: " << K1_prediction.covariance_(18,18);
      Vector19d sqrt_var;
      if (!InverseSqrt(K1_prediction.covariance_, &inv_sqrt_cov, &sqrt_var)) {
        LOG(ERROR) << "Could not calculate inverse sqrt covariance";
        return false;
      }
    }
    // Check inverse sqrt cov
    if (!inv_sqrt_cov.allFinite()) {
      LOG(ERROR) << "Found non-finite values in inv_sqrt_cov";
      LOG(ERROR) << "inv_sqrt_cov:\n" << inv_sqrt_cov;
      LOG(ERROR) << "K1_prediction.covariance_:\n" << K1_prediction.covariance_;
      return false;
    }
    
    // Scale residual and covariance by inverse sqrt covariance
    if (!overwrite_K1_) {
      residual_ = inv_sqrt_cov * residual_;
    }
    dK1_dK0_ = inv_sqrt_cov * dK1_dK0_;
    dK1_dK1_ = inv_sqrt_cov * dK1_dK1_;
    dK1_dt1_ = inv_sqrt_cov * dK1_dt1_;
    
    // Check for valid values
    if (!residual_.allFinite() || !dK1_dK0_.allFinite()  || !dK1_dK1_.allFinite() || !dK1_dt1_.allFinite()) {
      LOG(ERROR) << "Found non finite values in residual/Jacobians. Skip.";
      LOG(ERROR) << "residual_: " << residual_.transpose();
      LOG(ERROR) << "dK1_dK0_:\n" << dK1_dK0_;
      LOG(ERROR) << "dK1_dK1_:\n" << dK1_dK1_;
      LOG(ERROR) << "dK1_dt1_:\n" << dK1_dt1_;
      residual_.setZero();
      dK1_dK0_.setZero();
      dK1_dK1_.setZero();
      dK1_dt1_.setZero();
      return false;
    }
    
    // Copy to K1_ if asked to
    if (overwrite_K1_) {
      *K1_ = K1_prediction;
    }
    
    return true;
  }
  
  bool AddToProblem(ceres::Problem* problem) {
    ceres::CostFunction* cf = this;
    problem->AddResidualBlock(
      cf, NULL,
      K0_->error_.data(),                  // 19d
      K1_->error_.data()                   // 19d
    );
    return true;
  }
  
  // Update residual to incorporate the change in errors states
  //  Jacobians are not updated
  bool UpdateResiduals() {
    residual_ += dK1_dK0_*K0_->error_ + dK1_dK1_*K1_->error_;
  }
  
  // Evaluate function for ceres solver
  virtual bool Evaluate(
      double const* const* parameters,
      double* residuals,
      double** jacobians) const {
    
    // Mapping residuals and parameters to matrices
    MapConstVector19d    dK0(parameters[0]);
    MapConstVector19d    dK1(parameters[1]);
    //MapConstVector1d     dt1(parameters[2]);
    MapVector19d         residual(residuals);
    
    // Calculate residual from the error states using linearized model
    //residual = residual_ + dK1_dK0_*dK0 + dK1_dK1_*dK1 + dK1_dt1_*dt1;
    residual = residual_ + dK1_dK0_*dK0 + dK1_dK1_*dK1;
    
    // First estimates Jacobians are used as this shown to keep the estimator consistent
    if (jacobians != NULL) {
      if (jacobians[0] != NULL) {
        Eigen::Map<Eigen::Matrix<double,19,19,Eigen::RowMajor>> jac(jacobians[0]);
        jac = dK1_dK0_;
      }
      if (jacobians[1] != NULL) {
        Eigen::Map<Eigen::Matrix<double,19,19,Eigen::RowMajor>> jac(jacobians[1]);
        jac = dK1_dK1_;
      }
      //if (jacobians[2] != NULL) {
      //  Eigen::Map<Eigen::Matrix<double,19,1>> jac(jacobians[2]);
      //  jac = dK1_dt1_;
      ///}
    }
    
    return true;
  }
  
  // State operations utilities 
  
  // Interpolate states - this allows to reuse the calculations at the cost of accuracy
  bool InterpolateState(const double ti, const double ti_var,
                        KinematicState* Ki, Matrix19d* dKi_dK0, Vector19d* dKi_ti) const {
    
    LOG(ERROR) << "Interpolate state is yet to be implemented"; return false;
    
    return true;
  }
  
  // Interpolate pose - linearly interpolates K0 to K1, and the Jacobians
  bool InterpolatePose(const double ti, const double ti_var,
                       PoseState* Pi, Matrix6x19d* dPi_dK0, Vector6d* dPi_dti) const {
    
    if (ti < 0 || ti > t1_) {
      LOG(ERROR) << "Can not interpolate outside the interval. ti " << ti << " t1 " << t1_;
      return false;
    }
    
    double u = ti / t1_; double u1 = 1.-u;
    
    Quaterniond q = K0_->Q().slerp(u, K1_->Q());
    Vector3d p = u1*K0_->P() + u*K1_->P();
    Matrix6d cov = u1*K0_->CovPose() + u*K1_->CovPose();
    Pi->SetPose(q,p);
    Pi->SetCovariance(cov);
    
    *dPi_dK0 = u * dK1_dK0_.block<6,19>(0,0);
    dPi_dK0->diagonal().setOnes();
    
    Vector6d dP0_dt0;
    dP0_dt0.block<3,1>(0,0) = K0_->R().transpose() * K0_->W();
    dP0_dt0.block<3,1>(3,0) = K0_->R().transpose() * K0_->V();
    
    *dPi_dti = u1*dP0_dt0 + u*dK1_dt1_.block<6,1>(0,0);
    
    return true;
  }

  // Destructor
  virtual ~InterKinematicStateResidual() {}
};  // InterKinematicStateResidual  **/


/** Prior for a kinematic state
 *  Sets a prior on a kinematic pose. Uses a supplied kinematic state to create the prior
 */
class KinematicStatePrior : public ceres::CostFunction {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  // State
  KinematicState* K0_;
  
  // Starting error and jacobian
  Matrix18d  dK0_dK0_;
  Vector18d  dK0_;
  
  // Constructor
  KinematicStatePrior(): K0_(nullptr) {
    Reset();
  }
  
  bool Reset() {
    K0_ = nullptr; dK0_dK0_.setZero(); dK0_.setZero();
    return true;
  }
  
  // Copy constructor
  KinematicStatePrior(const KinematicStatePrior& r) {
    K0_ = r.K0_; dK0_dK0_ = r.dK0_dK0_; dK0_ = r.dK0_;
  }
  
  // Equal to assignment operator
  KinematicStatePrior& operator= (const KinematicStatePrior& r) {
    K0_ = r.K0_; dK0_dK0_ = r.dK0_dK0_; dK0_ = r.dK0_;
  }
  
  // Create from a Kinematic state
  bool Create(KinematicState* K0) {
    if (!K0) {LOG(ERROR)<<"K0 is null"; return false;}
    if (!K0->IsErrorZero()) {LOG(INFO)<<"K0 error is not zero, just FYI.";}
    
    K0_ = K0;
    
    // Number of residuals for the solver
    //set_num_residuals(12);
    set_num_residuals(18);
    
    // Set parameter block sizes for solver - kin state
    //mutable_parameter_block_sizes()->push_back(12); // Starting kin state has 18 parameters
    mutable_parameter_block_sizes()->push_back(18); // Starting kin state has 18 parameters
    
    // Create the residual and jacobians
    if (!CalculateResiduals()) {
      return false;
    }
    
    return true;
  }
  
  bool CalculateResiduals() {
    
    dK0_ = K0_->error_.block<18,1>(0,0);
    
    /*// Calculate inverse sqrt covariance of residual
    Matrix12d cov_resid = K0_->covariance_.block<12,12>(0,0);
    Matrix12d dK0_dK0_12;
    Vector12d sqrt_var_12;
    if (!InverseSqrt(cov_resid, &dK0_dK0_12, &sqrt_var_12)) {
      LOG(ERROR) << "Could not calculate inverse sqrt covariance";
      return false;
    }
    // Check inverse sqrt cov
    if (!dK0_dK0_.allFinite()) {
      LOG(ERROR) << "Found non-finite values in dK0_dK0_";
      LOG(ERROR) << "dK0_dK0_:\n" << dK0_dK0_;
      LOG(ERROR) << "K0 cov:\n" << cov_resid;
      return false;
    }
    dK0_dK0_.block<12,12>(0,0) = dK0_dK0_12;*/
    
    // Calculate inverse sqrt covariance of residual
    Matrix18d cov_resid = K0_->covariance_.block<18,18>(0,0);
    Matrix18d dK0_dK0;
    Vector18d sqrt_var;
    if (!InverseSqrt(cov_resid, &dK0_dK0, &sqrt_var)) {
      LOG(ERROR) << "Could not calculate inverse sqrt covariance";
      return false;
    }
    // Check inverse sqrt cov
    if (!dK0_dK0_.allFinite()) {
      LOG(ERROR) << "Found non-finite values in dK0_dK0_";
      LOG(ERROR) << "dK0_dK0_:\n" << dK0_dK0_;
      LOG(ERROR) << "K0 cov:\n" << cov_resid;
      return false;
    }
    dK0_dK0_ = dK0_dK0;
    
    return true;
  }
  
  bool AddToProblem(ceres::Problem* problem) {
    ceres::CostFunction* cf = this;
    problem->AddResidualBlock(
      cf, NULL,
      K0_->error_.data()                  // 18d
    );    
    return true;
  }
  
  // Mark K0 as constant on the problem
  bool MarkK0Constant(ceres::Problem* problem) {
    if (problem->HasParameterBlock(K0_->error_.data()))
      problem->SetParameterBlockConstant(K0_->error_.data());    
    return true;
  }
  
  // Evaluate function for ceres solver
  virtual bool Evaluate(
      double const* const* parameters,
      double* residuals,
      double** jacobians) const {
    
    // Mapping residuals and parameters to matrices
    //MapConstVector12d    dK0(parameters[0]);
    //MapVector12d         residual(residuals);
    MapConstVector18d    dK0(parameters[0]);
    MapVector18d         residual(residuals);
    
    // Calculate residual from the error states using linearized model
    //residual = dK0_dK0_.block<12,12>(0,0) * (dK0 - dK0_.block<12,1>(0,0));
    residual = dK0_dK0_*(dK0 - dK0_);
    
    // First estimates Jacobians are used as this shown to keep the estimator consistent
    if (jacobians != NULL) {
      if (jacobians[0] != NULL) {
        //Eigen::Map<Eigen::Matrix<double,12,12,Eigen::RowMajor>> jac(jacobians[0]);
        //jac = dK0_dK0_.block<12,12>(0,0);
        Eigen::Map<Eigen::Matrix<double,18,18,Eigen::RowMajor>> jac(jacobians[0]);
        jac = dK0_dK0_.block<18,18>(0,0);
      }
    }
    
    return true;
  }
  
  /*// Create from a Kinematic state
  bool Create2(KinematicState* K0) {
    if (!K0) {LOG(ERROR)<<"K0 is null"; return false;}
    if (!K0->IsErrorZero()) {LOG(INFO)<<"K0 error is not zero, just FYI.";}
    
    K0_ = K0;
    
    // Number of residuals for the solver
    set_num_residuals(18);
    // Set parameter block sizes for solver - kin state, camera intrinsics, extrinsics, timedelay
    mutable_parameter_block_sizes()->push_back(18); // Starting kin state has 18 parameters
    
    // Create the residual and jacobians
    if (!CalculateResiduals()) {
      return false;
    }
    
    return true;
  }
  
  bool CalculateResiduals2() {
    
    dK0_ = K0_->error_.block<18,1>(0,0);
    
    // Calculate inverse sqrt covariance of residual
    Matrix18d cov_resid = K0_->covariance_.block<18,18>(0,0);
    Vector18d sqrt_var;
    if (!InverseSqrt(cov_resid, &dK0_dK0_, &sqrt_var)) {
      LOG(ERROR) << "Could not calculate inverse sqrt covariance";
      return false;
    }
    // Check inverse sqrt cov
    if (!dK0_dK0_.allFinite()) {
      LOG(ERROR) << "Found non-finite values in dK0_dK0_";
      LOG(ERROR) << "dK0_dK0_:\n" << dK0_dK0_;
      LOG(ERROR) << "K0 cov:\n" << cov_resid;
      return false;
    }
    
    return true;
  }
  
  bool AddToProblem2(ceres::Problem* problem) {
    ceres::CostFunction* cf = this;
    problem->AddResidualBlock(
      cf, NULL,
      K0_->error_.data()                  // 18d
    );    
    return true;
  }
  
  // Evaluate function for ceres solver
  virtual bool Evaluate2(
      double const* const* parameters,
      double* residuals,
      double** jacobians) const {
    
    // Mapping residuals and parameters to matrices
    MapConstVector18d    dK0(parameters[0]);
    MapVector18d         residual(residuals);
    
    // Calculate residual from the error states using linearized model
    residual = dK0_dK0_*(dK0 - dK0_);
    
    // First estimates Jacobians are used as this shown to keep the estimator consistent
    if (jacobians != NULL) {
      if (jacobians[0] != NULL) {
        Eigen::Map<Eigen::Matrix<double,18,18,Eigen::RowMajor>> jac(jacobians[0]);
        jac = dK0_dK0_;
      }
    }
    
    return true;
  } */
  
};  // KinematicStatePrior


/** Inter Kinematic State Residual - fixed interstate time interval
 *  Connects two kinematic states assuming constant acceleration. This residual:
 *  - Predicts the ending state
 *  - Calculates ending state variance
 *  - Provides residual evaluation
 *
 *  First state should be initiated. It will be used to move forward in time to second state.
 *  Second state will be overwritten by default, unless asked not to.
 *  Starting residual will be calculated using the two states.
 *  Variance of the residual is the predicted variance of the ending state.
 */
class InterKinematicStateResidual : public ceres::CostFunction { // R, K0, K1
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  struct Options {
    Matrix6d   acceleration_covariance;     // inadequate model fit for acceleration
    Options(const double edw, const double edv) {
      double edw2 = edw*edw;
      double edv2 = edv*edv;
      Vector6d accl_cov_diag; accl_cov_diag << edw2, edw2, edw2, edv2, edv2, edv2;
      acceleration_covariance = accl_cov_diag.asDiagonal();
    }
  };  // Options
  // Options set the process noise
  const InterKinematicStateResidual::Options* options_;
  
  // States
  KinematicState* K0_;          // Starting state
  KinematicState* K1_;          // Ending state
  double  t1_;                  // Time interval between states in seconds
  double  t1_var_;              // Variance of time interval
  
  // Residual and Jacobians
  Vector18d residual_;          // Residual = K1_prediction - K1_
  Matrix18d dK1_dK0_;           // Jacobian of residual (or K1_prediction) wrt dK0
  Matrix18d dK1_dK1_;           // Jacobian of residual wrt dK1
  Vector18d dK1_dt1_;           // Jacobian of residual (or K1_prediction) wrt dt1
  
  // Helpers
  int64_t t1_musec_;            // Time interval between states in micro seconds
  bool overwrite_K1_;           // Should K1 be overwritten?
  Vector18d dK_prediction_;     // Predicted state as seen from K0
  Matrix18d J_prediction_K0_;   // Jacobian of dK_prediction wrt K0
  Matrix18d residual_cov_;      // Covariance of residual (defined as seen from K0)
  
  // Constructor
  InterKinematicStateResidual(const InterKinematicStateResidual::Options* options):
      options_(options),
      K0_(nullptr), K1_(nullptr), t1_musec_(0), t1_(0.), t1_var_(0.), overwrite_K1_(true) {
    Reset();
  }
  
  bool Reset() {
    // options are not modified
    if (options_->acceleration_covariance.isZero()) {LOG(WARNING)<<"Options accl cov is zero.";}
    K0_ = nullptr; K1_ = nullptr;
    t1_musec_ = 0; t1_ = 0.; t1_var_ = 0.; overwrite_K1_ = true;
    residual_.setZero(); dK1_dK0_.setZero(); dK1_dK1_.setZero(); dK1_dt1_.setZero();
    dK_prediction_.setZero(); J_prediction_K0_.setZero(); residual_cov_.setZero();
    return true;
  }
  
  // Options are reset separately from Create call
  bool SetOptions(const InterKinematicStateResidual::Options* options) {
    if (!options) {LOG(ERROR)<<"Options pointer is null"; return false;}
    if (options->acceleration_covariance.isZero()) {LOG(WARNING)<<"Options accl cov is zero.";}
    options_ = options;
    return true;
  }
  
  // Copy constructor
  InterKinematicStateResidual(const InterKinematicStateResidual& r) :
      options_(nullptr),
      K0_(nullptr), K1_(nullptr), t1_musec_(0), t1_(0.), t1_var_(0.), overwrite_K1_(true) {
    options_ = r.options_;
    K0_ = r.K0_; K1_ = r.K1_;
    t1_musec_ = r.t1_musec_; t1_ = r.t1_; t1_var_ = r.t1_var_; overwrite_K1_ = r.overwrite_K1_;
    residual_ = r.residual_; dK1_dK0_ = r.dK1_dK0_; dK1_dK1_ = r.dK1_dK1_; dK1_dt1_ = r.dK1_dt1_;
    dK_prediction_ = r.dK_prediction_; J_prediction_K0_ = r.J_prediction_K0_; residual_cov_ = r.residual_cov_;
  }
  
  // Equal to assignment operator
  InterKinematicStateResidual& operator= (const InterKinematicStateResidual& r) {
    options_ = r.options_;
    K0_ = r.K0_; K1_ = r.K1_;
    t1_musec_ = r.t1_musec_; t1_ = r.t1_; t1_var_ = r.t1_var_; overwrite_K1_ = r.overwrite_K1_;
    residual_ = r.residual_; dK1_dK0_ = r.dK1_dK0_; dK1_dK1_ = r.dK1_dK1_; dK1_dt1_ = r.dK1_dt1_;
    dK_prediction_ = r.dK_prediction_; J_prediction_K0_ = r.J_prediction_K0_; residual_cov_ = r.residual_cov_;
  }
  
  // Create the residual given two consecutive kinematic states
  bool Create(KinematicState* K0, KinematicState* K1, const int64_t t1_musec,
              bool overwrite_K1 = false, bool zero_K0_is_ok = false) {
    if (!options_) {LOG(ERROR)<<"Options have not been set. Quit."; return false;}
    if (!K0) {LOG(ERROR)<<"K0 is null"; return false;}
    if (!zero_K0_is_ok && K0->IsZero()) {LOG(ERROR)<<"K0 is zero. Set zero_K0_is_ok to true"; return false;}
    if (!K1) {LOG(ERROR)<<"K1 is null"; return false;}
    if (t1_musec<1) {LOG(WARNING)<<"t1<1";}
    //if (t1_var<0.) {LOG(ERROR)<<"var_t1<0."; return false;}
    if (overwrite_K1 && !K1->IsZero()) {LOG(WARNING)<<"To be predicted state K1 is not zero";}
    //if (!K0->IsErrorZero()) {LOG(ERROR)<<"K0 error is not zero. Should be."; return false;}
    //if (!K1->IsErrorZero()) {LOG(ERROR)<<"K1 error is not zero. Should be."; return false;}
    
    K0_ = K0;
    K1_ = K1;
    t1_musec_ = t1_musec;
    t1_ = double(t1_musec_)*1e-6;
    t1_var_ = 0.;   // Fixed at 0 as t1 is deterministic
    overwrite_K1_ = overwrite_K1;
    
    // Number of residuals for the solver
    //set_num_residuals(12);
    set_num_residuals(18);
    
    // Set parameter block sizes for solver - kin state, camera intrinsics, extrinsics, timedelay
    //mutable_parameter_block_sizes()->push_back(12); // Starting kin state has 18 parameters
    //mutable_parameter_block_sizes()->push_back(12); // Ending kin state has 18 parameters
    mutable_parameter_block_sizes()->push_back(18); // Starting kin state has 18 parameters
    mutable_parameter_block_sizes()->push_back(18); // Ending kin state has 18 parameters
    
    if (!CalculateResiduals()) {
      LOG(ERROR) << "Could not calculate residuals";
      return false;
    }
    
    return true;
  }
  
  /*// Calculate residuals, jacobians and variances.
  //  First estimates are used for jacobians, residual is recalculated but jacobians are not
  bool CalculateResiduals() {
    
    if (!K0_ || !K1_) {LOG(ERROR)<<"!K0_ || !K1_"; return false;}
    
    // Calculate residual
    // Adjust residual for any state errors already present. errors may not be zero at start.
    //  Notice that jacobian calculations neglect this. This ensures that jacobians use same state 
    //  estimates for the entire problem. This keeps the estimator consistent.
    residual_ = K1_->edwdv_ - K0_->edwdv_;
    residual_ += -dK1_dK0_ * K0_->edwdv_;
    residual_ += -dK1_dK1_ * K1_->edwdv_;
    
    // Jacobians
    dK1_dK0_ = -kIdentity6d;
    dK1_dK1_ =  kIdentity6d;
    
    // Covariance of residual
    residual_cov_ = options_->acceleration_covariance;
    
    // Report
    VLOG(1) << "Inter state residual: \n" << residual_.transpose();
    VLOG(1) << "Inter state residual sqrt cov diagonal: \n" << residual_cov_.diagonal().cwiseSqrt().transpose();
    
    // Calculate inverse sqrt covariance of residual
    Matrix6d inv_sqrt_cov;
    Vector6d sqrt_var;
    if (!InverseSqrt(residual_cov_, &inv_sqrt_cov, &sqrt_var)) {
      LOG(ERROR) << "Could not calculate inverse sqrt covariance";
      return false;
    }
    // Check inverse sqrt cov
    if (!inv_sqrt_cov.allFinite()) {
      LOG(ERROR) << "Found non-finite values in inv_sqrt_cov";
      LOG(ERROR) << "inv_sqrt_cov:\n" << inv_sqrt_cov;
      return false;
    }
    
    // Scale residual and covariance by inverse sqrt covariance
    residual_ = inv_sqrt_cov * residual_;
    dK1_dK0_  = inv_sqrt_cov * dK1_dK0_;
    dK1_dK1_  = inv_sqrt_cov * dK1_dK1_;
    //dK1_dt1_ = inv_sqrt_cov * dK1_dt1_;
    
    // Check for valid values
    if (!residual_.allFinite() || !dK1_dK0_.allFinite()  || !dK1_dK1_.allFinite() || !dK1_dt1_.allFinite()) {
      LOG(ERROR) << "Found non finite values in residual/Jacobians. Skip.";
      LOG(ERROR) << "residual_: " << residual_.transpose();
      LOG(ERROR) << "dK1_dK0_:\n" << dK1_dK0_;
      LOG(ERROR) << "dK1_dK1_:\n" << dK1_dK1_;
      LOG(ERROR) << "dK1_dt1_:\n" << dK1_dt1_;
      residual_.setZero();
      dK1_dK0_.setZero();
      dK1_dK1_.setZero();
      dK1_dt1_.setZero();
      return false;
    }
    
    // Copy to K1_ if asked to
    if (overwrite_K1_) {
      LOG(ERROR) << "Overwriting K1 is not implemented";
      return false;
    }
    
    return true;
  }
  
  // Add this residual to the problem
  bool AddToProblem(ceres::Problem* problem) {
    ceres::CostFunction* cf = this;
    problem->AddResidualBlock(
      cf, NULL,
      K0_->edwdv_.data(),                  // 6d
      K1_->edwdv_.data()                   // 6d
    );
    return true;
  }
  
  // Mark K0 as constant on the problem
  bool MarkK0Constant(ceres::Problem* problem) {
    if (problem->HasParameterBlock(K0_->edwdv_.data()))
      problem->SetParameterBlockConstant(K0_->edwdv_.data());    
    return true;
  }
  
  // Mark K1 as constant on the problem
  bool MarkK1Constant(ceres::Problem* problem) {
    if (problem->HasParameterBlock(K1_->edwdv_.data()))
      problem->SetParameterBlockConstant(K1_->edwdv_.data());    
    return true;
  }
  
  // Update the K1 pose and velocity errors using K0's acceleration errors
  bool UpdateK1Errors() {
    
    if (!K0_ || !K1_) {LOG(ERROR)<<"!K0_ || !K1_"; return false;}
    const KinematicState& K0 = *K0_;
    const KinematicState& K1 = *K1_;
    
    Vector18d dK_prediction;     // Predicted state as seen from K0
    Matrix18d J_prediction_K0;   // Jacobian of dK_prediction wrt K0, expressed in B0 frame
    if (!MoveInFixedTime(K0, t1_, &options_->acceleration_covariance,
                         &dK_prediction, &J_prediction_K0, nullptr)) {
      LOG(ERROR) << "MoveInTime calculation was not successful";
      return false;
    }
    
    // Change in pose and velocity of K1 expressed in B0 frame
    Vector12d B0_K1_dPdV = J_prediction_K0.block<12,6>(12,0) * K0_->edwdv_;
    
    // Transform the pose errors to reference frame A. A_eq_B1 = A_R_B0 * B0_eq_B1. Same with ep.
    B0_K1_dPdV.block<3,1>(0,0) = K0.R().transpose() * B0_K1_dPdV.block<3,1>(0,0);
    B0_K1_dPdV.block<3,1>(3,0) = K0.R().transpose() * B0_K1_dPdV.block<3,1>(3,0);
    
    // Change K1 errors
    K1_->error_.block<12,1>(0,0) = B0_K1_dPdV;
    
    return true;
  }
  
  // Update residual to incorporate the change in errors states
  //  Jacobians are not updated
  bool ResetK1ErrorToZero() {
    residual_ += dK1_dK1_ * K1_->edwdv_;
  }
  
  // Evaluate function for ceres solver
  virtual bool Evaluate(
      double const* const* parameters,
      double* residuals,
      double** jacobians) const {
    
    // Mapping residuals and parameters to matrices
    MapConstVector6d    dK0(parameters[0]);
    MapConstVector6d    dK1(parameters[1]);
    MapVector6d         residual(residuals);
    
    residual = residual_+ dK1_dK0_*dK0 + dK1_dK1_*dK1;
    
    // First estimates Jacobians are used as this shown to keep the estimator consistent
    if (jacobians != NULL) {
      if (jacobians[0] != NULL) {
        Eigen::Map<Eigen::Matrix<double,6,6,Eigen::RowMajor>> jac(jacobians[0]);
        jac = dK1_dK0_;
      }
      if (jacobians[1] != NULL) {
        Eigen::Map<Eigen::Matrix<double,6,6,Eigen::RowMajor>> jac(jacobians[1]);
        jac = dK1_dK1_;
      }
    }
    
    return true;
  }*/
  
  // Calculate residuals, jacobians and variances.
  //  First estimates are used for jacobians, residual is recalculated but jacobians are not
  bool CalculateResiduals() {
    
    if (!K0_ || !K1_) {LOG(ERROR)<<"!K0_ || !K1_"; return false;}
    const KinematicState& K0 = *K0_;
    const KinematicState& K1 = *K1_;
    
    if (!MoveInFixedTime(K0, t1_, &options_->acceleration_covariance,
                         &dK_prediction_, &J_prediction_K0_, &residual_cov_)) {
      LOG(ERROR) << "MoveInTime calculation was not successful";
      return false;
    }
    
    Matrix18d J_measurement_K1(kIdentity18d);
    J_measurement_K1.block<3,3>(0,0) = K0_->R();
    J_measurement_K1.block<3,3>(3,3) = K0_->R();
    
    Matrix18d J_measurement_K0(J_measurement_K1);
    J_measurement_K0 *= -1.;
    J_measurement_K0.block<3,3>(3,0) = K0_->R()*SkewSymm3d(K1_->P() - K0_->P());
    
    Vector19d dK_measurement;
    if (!DiffKinStates(K0, K1, &dK_measurement)) {
      LOG(ERROR) << "DiffKinStates to calculate residual had an error. Quit.";
      return false;
    }
    
    residual_ = dK_measurement.block<18,1>(0,0) - dK_prediction_;
    dK1_dK0_ = J_measurement_K0 - J_prediction_K0_;
    dK1_dK1_ = J_measurement_K1;
    
    // Adjust residual for any state errors already present. errors may not be zero at start.
    //  Notice that jacobian calculations neglect this. This ensures that jacobians use same state 
    //  estimates for the entire problem. This keeps the estimator consistent.
    residual_ += -dK1_dK0_*K0_->error_.block<18,1>(0,0);
    residual_ += -dK1_dK1_*K1_->error_.block<18,1>(0,0);
    
    // Report
    VLOG(1) << "Inter state residual: \n" << residual_.transpose();
    VLOG(1) << "Inter state residual sqrt cov diagonal: \n" << residual_cov_.diagonal().cwiseSqrt().transpose();
    
    // Calculate inverse sqrt covariance of residual
    Matrix18d inv_sqrt_cov;
    Vector18d sqrt_var;
    if (!InverseSqrt(residual_cov_, &inv_sqrt_cov, &sqrt_var)) {
      LOG(ERROR) << "Could not calculate inverse sqrt covariance";
      return false;
    }
    // Check inverse sqrt cov
    if (!inv_sqrt_cov.allFinite()) {
      LOG(ERROR) << "Found non-finite values in inv_sqrt_cov";
      LOG(ERROR) << "inv_sqrt_cov:\n" << inv_sqrt_cov;
      return false;
    }
    
    // Scale residual and covariance by inverse sqrt covariance
    residual_ = inv_sqrt_cov * residual_;
    dK1_dK0_  = inv_sqrt_cov * dK1_dK0_;
    dK1_dK1_  = inv_sqrt_cov * dK1_dK1_;
    //dK1_dt1_ = inv_sqrt_cov * dK1_dt1_;
    
    // Check for valid values
    if (!residual_.allFinite() || !dK1_dK0_.allFinite()  || !dK1_dK1_.allFinite() || !dK1_dt1_.allFinite()) {
      LOG(ERROR) << "Found non finite values in residual/Jacobians. Skip.";
      LOG(ERROR) << "residual_: " << residual_.transpose();
      LOG(ERROR) << "dK1_dK0_:\n" << dK1_dK0_;
      LOG(ERROR) << "dK1_dK1_:\n" << dK1_dK1_;
      LOG(ERROR) << "dK1_dt1_:\n" << dK1_dt1_;
      residual_.setZero();
      dK1_dK0_.setZero();
      dK1_dK1_.setZero();
      dK1_dt1_.setZero();
      return false;
    }
    
    // Copy to K1_ if asked to
    if (overwrite_K1_) {
      LOG(ERROR) << "Overwriting K1 is not implemented";
      return false;
    }
    
    return true;
  }
  
  // Add this residual to the problem
  bool AddToProblem(ceres::Problem* problem) {
    ceres::CostFunction* cf = this;
    problem->AddResidualBlock(
      cf, NULL,
      K0_->error_.data(),                  // 18d
      K1_->error_.data()                   // 18d
    );
    return true;
  }
  
  // Mark K0 as constant on the problem
  bool MarkK0Constant(ceres::Problem* problem) {
    if (problem->HasParameterBlock(K0_->error_.data()))
      problem->SetParameterBlockConstant(K0_->error_.data());    
    return true;
  }
  
  // Mark K1 as constant on the problem
  bool MarkK1Constant(ceres::Problem* problem) {
    if (problem->HasParameterBlock(K1_->error_.data()))
      problem->SetParameterBlockConstant(K1_->error_.data());    
    return true;
  }
  
  // Update residual to incorporate the change in errors states
  //  Jacobians are not updated
  bool ResetK1ErrorToZero() {
    //residual_.block<12,1>(0,0) += dK1_dK1_.block<12,12>(0,0) * K1_->error_.block<12,1>(0,0);
    residual_ += dK1_dK1_ * K1_->error_.block<18,1>(0,0);
  }
  
  // Evaluate function for ceres solver
  virtual bool Evaluate(
      double const* const* parameters,
      double* residuals,
      double** jacobians) const {
    
    // Mapping residuals and parameters to matrices
    //MapConstVector12d    dK0(parameters[0]);
    //MapConstVector12d    dK1(parameters[1]);
    //MapVector12d         residual(residuals);
    MapConstVector18d    dK0(parameters[0]);
    MapConstVector18d    dK1(parameters[1]);
    MapVector18d         residual(residuals);
    
    // Calculate residual from the error states using linearized model
    //residual = residual_.block<12,1>(0,0)
    //         + dK1_dK0_.block<12,12>(0,0) * dK0
    //         + dK1_dK1_.block<12,12>(0,0) * dK1;
    residual = residual_+ dK1_dK0_*dK0 + dK1_dK1_*dK1;
    
    // First estimates Jacobians are used as this shown to keep the estimator consistent
    if (jacobians != NULL) {
      if (jacobians[0] != NULL) {
        //Eigen::Map<Eigen::Matrix<double,12,12,Eigen::RowMajor>> jac(jacobians[0]);
        //jac = dK1_dK0_.block<12,12>(0,0);
        Eigen::Map<Eigen::Matrix<double,18,18,Eigen::RowMajor>> jac(jacobians[0]);
        jac = dK1_dK0_;
      }
      if (jacobians[1] != NULL) {
        //Eigen::Map<Eigen::Matrix<double,12,12,Eigen::RowMajor>> jac(jacobians[1]);
        //jac = dK1_dK1_.block<12,12>(0,0);
        Eigen::Map<Eigen::Matrix<double,18,18,Eigen::RowMajor>> jac(jacobians[1]);
        jac = dK1_dK1_;
      }
    }
    
    return true;
  }
  
  // State operations utilities 
  
  // Interpolate states - this allows to reuse the calculations at the cost of accuracy
  bool InterpolateState(const double ti, const double ti_var,
                        KinematicState* Ki, Matrix19d* dKi_dK0, Vector19d* dKi_ti) const {
    
    LOG(ERROR) << "Interpolate state is yet to be implemented"; return false;
    
    return true;
  }
  
  // Interpolate pose - linearly interpolates K0 to K1, and the Jacobians
  bool InterpolatePose(const double ti, const double ti_var,
                       PoseState* Pi, Matrix6x18d* dPi_dK0, Vector6d* dPi_dti) const {
    
    if (ti < 0 || ti > t1_) {
      LOG(ERROR) << "Can not interpolate outside the interval. ti " << ti << " t1 " << t1_;
      return false;
    }
    
    double u = ti / t1_; double u1 = 1.-u;
    
    Quaterniond q = K0_->Q().slerp(u, K1_->Q());
    Vector3d p = u1*K0_->P() + u*K1_->P();
    Matrix6d cov = u1*K0_->CovPose() + u*K1_->CovPose();
    Pi->SetPose(q,p);
    Pi->SetCovariance(cov);
    
    *dPi_dK0 = u * dK1_dK0_.block<6,18>(0,0);
    dPi_dK0->diagonal().setOnes();
    
    Vector6d dP0_dt0;
    dP0_dt0.block<3,1>(0,0) = K0_->R().transpose() * K0_->W();
    dP0_dt0.block<3,1>(3,0) = K0_->R().transpose() * K0_->V();
    
    *dPi_dti = u1*dP0_dt0 + u*dK1_dt1_.block<6,1>(0,0);
    
    return true;
  }

  bool InterpolateResidual(const double ti,
                           Vector18d* Ki_K0, Matrix18d* dKi_dK0, Matrix18d* dKi_cov) const {
    if (ti < 0 || ti > t1_) {
      LOG(ERROR) << "Can not interpolate outside the interval. ti " << ti << " t1 " << t1_;
      return false;
    }
    
    double u = ti/t1_; double u1 = 1.-u;
    
    *Ki_K0 = u*dK_prediction_;
    *dKi_dK0 = u*J_prediction_K0_;  // All off diagonal entries in Jacobian are integrals of time
                                    // or time itself. Thus linear interpolation is valid.
    dKi_dK0->diagonal().setOnes();
    *dKi_cov = dKi_dK0->block<18,6>(0,12)
               * options_->acceleration_covariance
               * dKi_dK0->block<18,6>(0,12).transpose();
    
    return true;
  }
  
  // Destructor
  virtual ~InterKinematicStateResidual() {}
};  // InterKinematicStateResidual 

class PinholeIntrinsicsState {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  // 4 pinhole + 5 distortion parameters: fx fy cx cy, k1 k2 k3, t1 t2
  
  Vector9d state_;   // Camera pinhole, radial and tangential distortion parameters
  Vector9d error_;
  Matrix9d covariance_;
  int32_t width_, height_;    // height and width of the camera image
  
  // Mappings
  MapVector4d Kv_; MapVector3d Dv_;  MapVector2d Tv_;
  
  // Helpers
  Matrix3d K_;
  Matrix3d K_inv_;
  Matrix9d inv_sqrt_cov_; 
  Matrix9d correl_; 
  Vector9d sqrt_var_; 
  
  PinholeIntrinsicsState():
      Kv_(state_.data()), Dv_(state_.data()+4), Tv_(state_.data()+7) {
    SetZero();
  }
  
  // Default copy constructor
  PinholeIntrinsicsState(const PinholeIntrinsicsState& r) :
      state_(r.state_), error_(r.error_), covariance_(r.covariance_),
      width_(r.width_), height_(r.height_),
      K_(r.K_), K_inv_(r.K_inv_),
      inv_sqrt_cov_(r.inv_sqrt_cov_), correl_(r.correl_), sqrt_var_(r.sqrt_var_),
      Kv_(state_.data()), Dv_(state_.data()+4), Tv_(state_.data()+7) {
    SetZero();
  }
  
  // Equal to assignment operator
  PinholeIntrinsicsState& operator= (const PinholeIntrinsicsState& r) {
    state_= r.state_; error_ = r.error_; covariance_ = r.covariance_;
    width_ = r.width_; height_ = r.height_;
    K_=r.K_; K_inv_=r.K_inv_;
    inv_sqrt_cov_=r.inv_sqrt_cov_; correl_=r.correl_; sqrt_var_=r.sqrt_var_;
  }
  
  bool SetZero() {
    state_.setZero();
    error_.setZero();
    covariance_.setZero();
    width_ = 0; height_ = 0;
    K_.setZero();
    K_inv_.setZero();
    inv_sqrt_cov_.setZero(); 
    correl_.setZero(); 
    sqrt_var_.setZero(); 
    return true;
  }
  
  bool IsZero() const {
    return (state_.isZero());
  }
  
  bool SetDistortionZero() {
    state_.block<5,1>(4,0).setZero();
    error_.block<5,1>(4,0).setZero();
  }
  
  bool UpdateCamerMatrix() {
    K_ << state_[0], 0., state_[2], 0., state_[1], state_[3], 0., 0., 1.;
    K_inv_ = K_.inverse();    
    return true;
  }
  
  inline double fx() const {return state_(0)+error_(0);}
  inline double fy() const {return state_(1)+error_(1);}
  inline double cx() const {return state_(2)+error_(2);}
  inline double cy() const {return state_(3)+error_(3);}
  inline const Matrix3d& K() const {return K_;}
  inline const Matrix3d& K_inv() const {return K_inv_;}
  inline int32_t width() const {return width_;}
  inline int32_t height() const {return height_;}
  
  bool SetImageSize(int32_t width, int32_t height) {
    width_ = width;
    height_ = height;
    return true;
  }
  
  bool SetIntrinsics(const cv::Mat& K, const cv::Mat& dist) {
    cv::Size K_size(3,3);
    cv::Size dist_size(5,1);
    if (K.size()!=K_size || dist.size()!=dist_size) {
      LOG(ERROR) << "cvMat sizes are not correct " << K.size() << " " << dist.size();
      return false;
    }
    state_[0] = K.at<double>(0,0);
    state_[1] = K.at<double>(1,1);
    state_[2] = K.at<double>(0,2);
    state_[3] = K.at<double>(1,2);
    state_[4] = dist.at<double>(0);
    state_[5] = dist.at<double>(1);
    state_[6] = dist.at<double>(4);
    state_[7] = dist.at<double>(2);
    state_[8] = dist.at<double>(3);
    UpdateCamerMatrix();
    VLOG(1) << "Set camera intrinsics = \n" << state_.transpose();
    return true;
  }
  
  /** Project a 3d point in camera coordinates on image */
  inline bool ProjectPoint(const Vector3d& CpF, Vector2d* uvim,
                    Matrix2x3d* duvim_dCpF, Matrix2x9d* duvim_dintrinsics) const {
    // Negative or zero depth is not allowed
    if (CpF(2) <= kEpsilon) {LOG(ERROR) << "CpF(2) <= kEpsilon. " << CpF(2); return false;}
    if (!uvim) {LOG(ERROR) << "uvim is NULL."; return false;}
    
    Matrix2d fxfy; fxfy << Kv_(0), 0., 0., Kv_(1);
    Vector2d cxcy; cxcy << Kv_(2), Kv_(3);
    const double k1 = Dv_(0); const double k2 = Dv_(1); const double k3 = Dv_(2);
    const double t1 = Tv_(0); const double t2 = Tv_(1);
    
    // Project point, then add distortion
    Vector2d uv; uv << CpF(0)/CpF(2), CpF(1)/CpF(2);
    const double u = uv(0); const double u2 = u*u;
    const double v = uv(1); const double v2 = v*v;
    const double r = u2 + v2; const double r2 = r*r; const double r3 = r2*r;
    const double s = u*v;
    Vector2d t_; t_ << 2.*t1*s + t2*(r + 2.*u2), t1*(r + 2.*v2) + 2.*t2*s;
    const double d_ = 1. + k1*r + k2*r2 + k3*r3;
    Vector2d udvd = d_ * uv + t_;
    *uvim = cxcy + fxfy * udvd;
    
    if (!duvim_dCpF && !duvim_dintrinsics) {return true;}
    
    // Calculate jacobians
    Matrix1x3d rrr; rrr << r, r2, r3;
    Matrix2x3d dudvd_dD = uv * rrr;
    Matrix2d dudvd_dT; dudvd_dT << 2.*s, r + 2.*u2, r + 2.*v2, 2.*s;
    Matrix1x3d kkkrrr; kkkrrr << k1, 2.*k2*r, 3.*k3*r2;
    Matrix3x2d uvuvuv; uvuvuv << u,v,u,v,u,v; uvuvuv *= 2.;
    const double aa = 2.*t1*u + 2.*t2*v;
    Matrix2d tuv; tuv << 6.*t2*u + 2.*t1*v, aa, aa, 2.*t2*u + 6.*t1*v;
    Matrix2d dudvd_duv = uv*kkkrrr*uvuvuv + d_*Matrix2d::Identity() + tuv;
    Matrix2x3d duv_dCpF; duv_dCpF << 1., 0., -u,  0., 1., -v;  duv_dCpF /= CpF(2);
    
    if (duvim_dCpF) {
      duvim_dCpF->block<2,3>(0,0) = fxfy * dudvd_duv * duv_dCpF;
    }
    
    if (duvim_dintrinsics) {
      duvim_dintrinsics->block<2,4>(0,0) << udvd(0), 0., 1., 0.,   0., udvd(1), 0., 1.;
      duvim_dintrinsics->block<2,3>(0,4) = fxfy * dudvd_dD;
      duvim_dintrinsics->block<2,2>(0,7) = fxfy * dudvd_dT;      
    }
    
    return true;
  }
  
  /* Point projection on an image */
  inline bool ProjectPoint(
      const Vector3d* xyz,    // Point in camera coordinates
      const Vector4d* K,      // Camera marix components fx, fy, cx, cy 
      const Vector3d* D, const Vector2d* T,  // Radial and tangential distortion coefficients
      Vector2d*   uvim,         // Point in image
      Matrix2x3d* duvim_dxyz,    // Jacobian of projection with point coordinates
      Matrix2x4d* duvim_dK,      // Jacobian of projection with camera matrix
      Matrix2x3d* duvim_dD,      // Jacobian of projection with radial distortion coefficients
      Matrix2d*   duvim_dT       // Jacobian of projection with tangential distortion coefficients
  ) const {
    if (!xyz || !K || !uvim) {LOG(ERROR) << "!xyz || !K || !uv_im. Quit."; return false;}
    if (!D || !T) {LOG(ERROR) << "!D || !T"; return false;}
    
    if ((*xyz)(2) <= kEpsilon) {LOG(ERROR) << "xyz->(2) <= Epsilon. Quit."; return false;}
    
    // Direct calculations
    Vector2d uv; uv << (*xyz)(0)/(*xyz)(2), (*xyz)(1)/(*xyz)(2);
    double u = uv(0); double u2 = u*u;
    double v = uv(1); double v2 = v*v;
    double r = u2 + v2; double r2 = r*r; double r3 = r2*r;
    double s = u*v;
    double k1 = (*D)(0); double k2 = (*D)(1); double k3 = (*D)(2);
    double t1 = (*T)(0); double t2 = (*T)(1);
    Vector2d t_; t_ << 2.*t1*s + t2*(r + 2.*u2), t1*(r + 2.*v2) + 2.*t2*s;
    double d_ = 1. + k1*r + k2*r2 + k3*r3;
    Vector2d udvd = d_ * uv + t_;
    Vector2d cxcy; cxcy << (*K)(2), (*K)(3);
    Matrix2d fxfy; fxfy << (*K)(0), 0., 0., (*K)(1);
    *uvim = cxcy + fxfy * udvd;
    
    if (!duvim_dxyz || !duvim_dK || !duvim_dD || !duvim_dT) return true;
    
    // Jacobians
    Matrix1x3d rrr; rrr << r, r2, r3;
    Matrix2x3d dudvd_dD = uv * rrr;
    Matrix2d dudvd_dT; dudvd_dT << 2.*s, r + 2.*u2, r + 2.*v2, 2.*s;
    Matrix1x3d kkkrrr; kkkrrr << k1, 2.*k2*r, 3.*k3*r2;
    Matrix3x2d uvuvuv; uvuvuv << u,v,u,v,u,v; uvuvuv *= 2.;
    double aa = 2.*t1*u + 2.*t2*v;
    Matrix2d tuv; tuv << 6.*t2*u + 2.*t1*v, aa, aa, 2.*t2*u + 6.*t1*v;
    Matrix2d dudvd_duv = uv*kkkrrr*uvuvuv + d_*Matrix2d::Identity() + tuv;
    Matrix2x3d duv_dxyz; duv_dxyz << 1., 0., -u,  0., 1., -v;  duv_dxyz /= (*xyz)(2);
    
    *duvim_dxyz = fxfy * dudvd_duv * duv_dxyz;
    *duvim_dK << udvd(0), 0., 1., 0.,   0., udvd(1), 0., 1.;
    *duvim_dD = fxfy * dudvd_dD;
    *duvim_dT = fxfy * dudvd_dT;
    
    return true;
  }
  
  // Calculate covariance matrix from the ceres problem object
  bool SetCovariance(ceres::Problem* problem) {
    // Check if this intrisics object is a parameter block in this problem
    double* err_ptr = error_.data();
    if (!problem->HasParameterBlock(err_ptr)) {
      LOG(ERROR) << "This camera intrinsics object is not a parameer in this problem. Quit.";
      return false;
    }
    
    // Setup covariance calculation
    ceres::Covariance::Options covariance_options;
    ceres::Covariance covariance(covariance_options);
    std::vector<std::pair<const double*, const double*>> covariance_blocks;
    covariance_blocks.push_back(std::make_pair(err_ptr, err_ptr));
    
    // Compute the covariance matricess
    if (!covariance.Compute(covariance_blocks, problem)) {
      LOG(ERROR) << "Could not compute covariance matrices";
      return false;
    }
    
    // Get the calculated covariance. Returned matrix is row major order
    Matrix9d covmat;
    covariance.GetCovarianceBlock(err_ptr, err_ptr, covmat.data());
    covariance_ = covmat;
    
    return CalculateInverseSqrtCovariance();
  }
  
  bool CalculateInverseSqrtCovariance() {
    if (!InverseSqrt(covariance_, &inv_sqrt_cov_, &sqrt_var_, &correl_)) {
      LOG(ERROR) << "Could not calculate inv sqrt mat";
      return false;
    }
    return true;
  }
  
  // Recalculate the state usually after optimization
  inline bool Recalculate() {
    state_ += error_;
    error_.setZero();
    UpdateCamerMatrix();
    // covariance and related matrices are left untouched
    return true;
  }
  
  // Create using a camera intrisics message
  bool SetFromMessage(const anantak::PinholeIntrinsicsStateMessage& state_msg) {
    if (!SetIntrinsics(state_msg)) {
      LOG(ERROR) << "Could not set intrinsics from message";
      return false;
    }
    if (!SetCovariance(state_msg)) {
      LOG(ERROR) << "Could not set intrinsics from message";
      return false;
    }
    return true;
  }
  
  // Create using a camera message
  bool SetIntrinsics(const anantak::PinholeIntrinsicsStateMessage& state_msg) {
    // Checks
    if (state_msg.image_size_size()<2) {LOG(ERROR)<<"image_size_size()<2. Bad message"; return false;}
    if (state_msg.pinhole_size()<4) {LOG(ERROR)<<"pinhole_size()<4. Bad message"; return false;}
    // Set pinhole
    state_.setZero();
    state_[0] = state_msg.pinhole(0);
    state_[1] = state_msg.pinhole(1);
    state_[2] = state_msg.pinhole(2);
    state_[3] = state_msg.pinhole(3);
    // Set image size
    width_ = state_msg.image_size(0);
    height_ = state_msg.image_size(1);
    // Set camera matrices
    K_ << state_[0], 0., state_[2], 0., state_[1], state_[3], 0., 0., 1.;
    K_inv_ = K_.inverse();
    // Radial distortion parameters
    if (state_msg.radial_size()!=3) {
      LOG(WARNING) << "state_msg.radial_size()!=3";        
    } else {
      state_[4] = state_msg.radial(0);
      state_[5] = state_msg.radial(1);
      state_[6] = state_msg.radial(2);
    }
    // Tangential distortion parameters
    if (state_msg.tangential_size()!=2) {
      LOG(WARNING) << "state_msg.tangential_size()!=2";        
    } else {
      state_[7] = state_msg.tangential(0);
      state_[8] = state_msg.tangential(1);
    }
    // Set error
    error_.setZero();
    return true;
  }
  
  bool SetCovariance(const anantak::PinholeIntrinsicsStateMessage& state_msg) {
    if (state_msg.covariance_size()!=covariance_.size()) {
      LOG(ERROR) << "Message covariance lenght does not match size of covariance_"
          << state_msg.covariance_size() << " " << covariance_.size();
      return false;
    }
    const MapConstMatrix9d msg_cov_data(state_msg.covariance().data());
    covariance_ = msg_cov_data;
    return CalculateInverseSqrtCovariance();
  }
  
  // Set camera intrinsics on a state message
  bool CopyToMessage(anantak::PinholeIntrinsicsStateMessage* state_msg) const {
    state_msg->clear_pinhole();
    // Image size
    state_msg->add_image_size(width_);
    state_msg->add_image_size(height_);
    // Pinhole
    state_msg->add_pinhole(state_[0]);
    state_msg->add_pinhole(state_[1]);
    state_msg->add_pinhole(state_[2]);
    state_msg->add_pinhole(state_[3]);
    // Radial
    state_msg->clear_radial();
    state_msg->add_radial(state_[4]);
    state_msg->add_radial(state_[5]);
    state_msg->add_radial(state_[6]);
    // Tangential
    state_msg->clear_tangential();
    state_msg->add_tangential(state_[7]);
    state_msg->add_tangential(state_[8]);
    // Covariance
    state_msg->clear_covariance();
    const double* cov_data = covariance_.data();
    for (int i=0; i<covariance_.size(); i++) {
      state_msg->add_covariance(*(cov_data+i));
    }    
    return true;
  }
  
  // Create from four given values
  bool Create(const double& fx, const double& fy, const double& cx, const double& cy,
      bool reset_covariance = true) {
    state_.setZero();
    state_[0] = fx;
    state_[1] = fy;
    state_[2] = cx;
    state_[3] = cy;
    // Set camera matrices
    K_ << state_[0], 0., state_[2], 0., state_[1], state_[3], 0., 0., 1.;
    K_inv_ = K_.inverse();
    // Radial distortion parameters are all zero
    if (reset_covariance) covariance_.setZero();
    // Set error
    error_.setZero();
    return true;
  }
  
  // Create approximate using image size and angle of view
  bool Create(const double& angle_of_view, const double& width, const double& height,
      const double& angle_of_view_stdev, const double& center_stdev) {
    double cx = width * 0.5;
    double cy = height * 0.5;
    double fx = cx / std::tan(angle_of_view * 0.5);
    double fy = fx;
    double mult = 2.0;
    double f0 = cx / std::tan((angle_of_view - mult*angle_of_view_stdev) * 0.5);
    double f1 = cx / std::tan((angle_of_view + mult*angle_of_view_stdev) * 0.5);
    double f_stdev = std::abs(f1 - f0)/mult*0.5;
    Eigen::Vector4d variance;
    double f_var = f_stdev * f_stdev;
    double c_var = center_stdev * center_stdev;
    variance << f_var, f_var, c_var, c_var;
    covariance_.setZero();
    covariance_.block<4,4>(0,0) = variance.asDiagonal();
    VLOG(1) << "Camera intrinsics from angle, size = " << angle_of_view << " " << width << " "
        << height << " are fx, fy, cx, cy = " << fx << " " << fy << " " << cx << " " << cy
        << " Covariance = \n" << covariance_;
    return Create(fx, fy, cx, cy, false);
  }
  
  // Create using CameraIntrinsicsState - note angles are in degrees
  bool Create(const anantak::CameraIntrinsicsInitMessage& msg) {
    if (msg.image_size_size()<2) {LOG(ERROR)<<"image_size_size()<2. Bad config."; return false;}
    width_ = msg.image_size(0);
    height_ = msg.image_size(1);
    return Create(msg.angle_of_view(), msg.image_size(0), msg.image_size(1),
                  msg.angle_of_view_stdev(), msg.center_stdev());
  }
  
  // String representation for printing
  std::string ToString(int detail=0) const {
    std::ostringstream ss;
    ss << "Image size: " << width_ << ", " << height_;
    if (detail>0) ss << "\nIntrinsics: ";
    ss << state_.block<4,1>(0,0).transpose(); ss << ",  ";
    ss << state_.block<3,1>(4,0).transpose(); ss << ",  ";
    ss << state_.block<2,1>(7,0).transpose();
    if (detail>0) {
      ss << "\nCovariance: \n" << covariance_.format(CleanFmt1);
    }
    if (detail>1) {
      ss << "\nSqrt var: ";
      ss << sqrt_var_.block<4,1>(0,0).transpose(); ss << ",  ";
      ss << sqrt_var_.block<3,1>(4,0).transpose(); ss << ",  ";
      ss << sqrt_var_.block<2,1>(7,0).transpose();
      ss << "\nCorrelation: \n" << correl_.format(CleanFmt1);
      if (!error_.isZero()) {
        ss << "\ndIntrinsics: ";
        ss << error_.block<4,1>(0,0).transpose(); ss << ",  ";
        ss << error_.block<3,1>(4,0).transpose(); ss << ",  ";
        ss << error_.block<2,1>(7,0).transpose();
      }
    }
    return ss.str();
  }
  
  // CV helpers
  cv::Size OpenCvSize() const {
    return cv::Size(width_, height_);
  }
  cv::Mat OpenCvCameraMatrix() const {
    cv::Mat cvK = (cv::Mat_<double>(3,3)
                   << state_[0], 0., state_[2],   0., state_[1], state_[3],   0., 0., 1.);
    return cvK;
  }
  cv::Mat OpenCvDistortionCoeffs() const {
    cv::Mat cvD = (cv::Mat_<double>(5,1)
                   << state_[4], state_[5], state_[7], state_[8], state_[6] );
    return cvD;
  }
  
  
};  // PinholeIntrinsicsState

inline Matrix3x4d AprilTag3dCorners(const double& tag_size = 1.0) {
  Matrix3x4d corners;
  corners << -1.0,  1.0,  1.0, -1.0,
             -1.0, -1.0,  1.0,  1.0,
              0.0,  0.0,  0.0,  0.0;
  corners *= tag_size*0.5;
  return corners;
}

inline bool ExtractAprilTag2dCorners(const AprilTagMessage& apriltag_msg, int i_tag,
                                     Matrix2x4d* image_coords) {
  if (image_coords) *image_coords <<
      apriltag_msg.u_1(i_tag), apriltag_msg.u_2(i_tag), apriltag_msg.u_3(i_tag), apriltag_msg.u_4(i_tag),
      apriltag_msg.v_1(i_tag), apriltag_msg.v_2(i_tag), apriltag_msg.v_3(i_tag), apriltag_msg.v_4(i_tag);  
  return true;
}

inline bool PlotTagOnImage(const Matrix2x4d& corners, const cv::Scalar& color, cv::Mat* image) {
  cv::Point2d p0(corners(0,0), corners(1,0));
  cv::Point2d p1(corners(0,1), corners(1,1));
  cv::Point2d p2(corners(0,2), corners(1,2));
  cv::Point2d p3(corners(0,3), corners(1,3));
  cv::line(*image, p0, p1, color, 1);
  cv::line(*image, p1, p2, color, 1);
  cv::line(*image, p2, p3, color, 1);
  cv::line(*image, p3, p0, color, 1);
  return true;
}

inline bool PlotAprilTagMessage(const AprilTagMessage& apriltag_msg, cv::Mat* image,
                                cv::Scalar color=CV_WHITE) {
  for (int i_tag=0; i_tag<apriltag_msg.tag_id_size(); i_tag++) {
    Matrix2x4d image_coords;
    ExtractAprilTag2dCorners(apriltag_msg, i_tag, &image_coords);
    PlotTagOnImage(image_coords, color, image);
  }
  return true;
}

/** Camera State - composition of intrinsics, static pose, delay */
class CameraState {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  // Camera identifier
  int32_t camera_num_;
  
  PinholeIntrinsicsState  intrinsics_;
  PoseState               extrinsics_;
  Point1dState            time_delay_;
  
  CameraState(int32_t camera_num):
      camera_num_(camera_num) {}
  
  // Default copy constructor
  CameraState(const CameraState& r) :
      camera_num_(r.camera_num_), 
      intrinsics_(r.intrinsics_), extrinsics_(r.extrinsics_), time_delay_(r.time_delay_) {}
  
  // Equal to assignment operator
  CameraState& operator= (const CameraState& r) {
    camera_num_ = r.camera_num_; 
    intrinsics_ = r.intrinsics_; extrinsics_ = r.extrinsics_; time_delay_ = r.time_delay_;
  }  
  
  // Const references
  const PinholeIntrinsicsState& Intrinsics() const {return intrinsics_;}
  const Matrix3d& K() const {return intrinsics_.K();}
  const Matrix3d& K_inv() const {return intrinsics_.K_inv();}
  const PoseState& Pose() const {return extrinsics_;}
  const Point1dState& TimeDelay() const {return time_delay_;}
  
  // Accessors
  inline int32_t CameraNumber() const {return camera_num_;}
  inline int32_t width() const {return intrinsics_.width();}
  inline int32_t height() const {return intrinsics_.height();}
  inline std::vector<int32_t> ImageSize() const {
    std::vector<int32_t> imsz({intrinsics_.width_, intrinsics_.height_}); return imsz;}
  
  // Image size
  inline bool SetImageSize(int32_t width, int32_t height) {return intrinsics_.SetImageSize(width, height);}
  
  // References
  PinholeIntrinsicsState& Intrinsics() {return intrinsics_;}
  
  // Set from a camera state message
  bool SetFromMessage(const SensorMsg& msg) {
    if (!msg.has_header()) {
      LOG(ERROR) << "Sensor message has no header";
      return false;
    }
    if (msg.header().type() != "CameraState") {
      LOG(ERROR) << "Message type is not CameraState. Found = " << msg.header().type();
      return false;
    }
    if (!msg.has_camera_state_msg()) {
      LOG(ERROR) << "Sensor message has no camera state msg";
      return false;
    }
    
    // Set intrinsics
    const CameraStateMessage& camera_msg = msg.camera_state_msg();
    camera_num_ = camera_msg.camera_num();
    if (!intrinsics_.SetFromMessage(camera_msg.intrinsics())) {
      LOG(ERROR) << "Could not set intrinsics from message";
      return false;
    }
    
    // Set time delay
    if (!time_delay_.SetFromMessage(camera_msg.time_delay())) {
      LOG(ERROR) << "Could not set time delay from message";
      return false;
    }
    
    return true;
  }
  
  // Set from init message
  bool SetIntrinsicsFromInitMessage(const anantak::CameraIntrinsicsInitMessage& init_msg) {
    return intrinsics_.Create(init_msg);
  }
  
  bool SetIntrinsicsFromHeuristics(const double& angle_of_view, const double& width, const double& height,
      const double& angle_of_view_stdev, const double& center_stdev) {
    return intrinsics_.Create(angle_of_view, width, height, angle_of_view_stdev, center_stdev);
  }
  
  // Copy state to a message
  bool CopyToMessage(SensorMsg* msg, const int64_t timestamp) const {
    msg->Clear();
    // Set header
    anantak::HeaderMsg* hdr_msg = msg->mutable_header();
    hdr_msg->set_timestamp(timestamp);
    hdr_msg->set_type("CameraState");
    hdr_msg->set_recieve_timestamp(timestamp);
    hdr_msg->set_send_timestamp(timestamp);
    // Set camera state
    anantak::CameraStateMessage* camera_msg = msg->mutable_camera_state_msg();
    camera_msg->set_camera_num(camera_num_);
    // Set intrinsics 
    anantak::PinholeIntrinsicsStateMessage* pinhole_msg = camera_msg->mutable_intrinsics();
    intrinsics_.CopyToMessage(pinhole_msg);
    // Set extrinsics
    anantak::PoseStateMessage* pose_msg = camera_msg->mutable_extrinsics();
    extrinsics_.CopyToMessage(pose_msg);
    // Set time delay
    anantak::Point1dStateMessage* time_delay_msg = camera_msg->mutable_time_delay();
    time_delay_.CopyToMessage(time_delay_msg);
    
    return true;
  }
  
  // Save a sensor message representation of this state to file
  bool SaveToFile(const std::string& savefile) const {
    anantak::MessageFileWriter file_writer;
    if (!file_writer.Open(savefile)) {
      LOG(ERROR) << "Could not open " << savefile;
      return false;
    }
    anantak::SensorMsg _msg;
    if (CopyToMessage(&_msg, anantak::GetWallTime())) file_writer.WriteMessage(_msg);
    file_writer.Close();
    LOG(INFO) << "Saved camera state to " << savefile;
    return true;
  }
  
  // Load from file
  bool LoadFromFile(const std::string& savefile) {
    anantak::MessageFileReader file_reader;
    std::vector<anantak::SensorMsg> msgs;
    if (!file_reader.LoadMessagesFromFile(savefile, &msgs)) {
      LOG(ERROR) << "Could not load messages from file " << savefile;
      return false;
    }
    if (msgs.size()<1) {
      LOG(ERROR) << "No messages were loaded from file " << savefile;
      return false;
    }
    if (!SetFromMessage(msgs.back())) {
      LOG(ERROR) << "Could not set camera from message in file " << savefile;
      return false;
    }
    return true;
  }
  
  // Get a string representation
  inline std::string ToString(int detail=0) const {
    std::ostringstream ss;
    if (detail>0) ss << "\nCamera number: ";
    ss << camera_num_ << "  ";
    ss << intrinsics_.ToString(detail);
    if (detail>0) ss << "\nExtrinsics: "; else ss << "  ";
    ss << extrinsics_.ToString(detail);
    if (detail>0) ss << "\nTime delay: "; else ss << "  ";
    ss << time_delay_.ToString(detail);
    return ss.str();
  }
  
}; // Camera state


// Display class for camera state
//  Helps draw the camera in a scene
class CameraStateDisplay {
 public:
  
  bool is_initialized_;
  Matrix3x6d vertices_;
  Matrix4dRowType transform_;
  
  CameraStateDisplay(): is_initialized_(false) {
    vertices_.setZero();
    transform_.setZero();
  }
  
  bool SetCamera(const CameraState& camera, const double meters_per_pixel = 1.) {
    Matrix3d K = camera.Intrinsics().K();
    double f = (K(0,0) + K(1,1))*0.5;
    double cu = K(0,2);
    double cv = K(1,2);
    double u = double(camera.Intrinsics().width_);
    double v = double(camera.Intrinsics().height_);
    u = 640.;
    v = 480.;
    //Vector3d v0; v0 << 0., 0., -f;
    //Vector3d v1; v1 << -cu, v-cv, 0.;
    //Vector3d v2; v2 << -cu, -cv, 0.;
    //Vector3d v3; v3 << u-cu, -cv, 0.;
    //Vector3d v4; v4 << u-cu, v-cv, 0.;
    //Vector3d v5; v5 << 0., v-cv, -f;
    vertices_.setZero();
    vertices_.col(0) << 0., 0., -f;
    vertices_.col(1) << -cu, v-cv, 0.;
    vertices_.col(2) << -cu, -cv, 0.;
    vertices_.col(3) << u-cu, -cv, 0.;
    vertices_.col(4) << u-cu, v-cv, 0.;
    vertices_.col(5) << 0., v-cv, -f;
    vertices_ *= meters_per_pixel;
    VLOG(1) << "Camera display vertices = \n" << vertices_;
    
    transform_.setZero();
    transform_(3,3) = 1.;
    transform_.block<3,3>(0,0) = camera.Pose().Q().toRotationMatrix();
    transform_.block<3,1>(0,3) = camera.Pose().P();
    VLOG(1) << "Camera display transform = \n" << transform_;
    
    is_initialized_ = true;
    
    return true;
  }
  
};  // CameraStateDisplay

/** Implements undistortion of camera messages */
class UndistortedCamera {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  // Cameras
  const CameraState* original_camera_;       // Original camera state
  CameraState camera_;  // New camera state
  
  // Undistortion components
  cv::Mat map_x_, map_y_;
  
  // Helpers
  bool is_initialized_;
  
  // Constructor takes in a camera state. This is copied in new camera but for distortion.
  UndistortedCamera(const CameraState& original_camera): is_initialized_(false),
      original_camera_(&original_camera), camera_(original_camera.camera_num_)
  {
    Initialize();
  }
  
  UndistortedCamera(int32_t camera_num): is_initialized_(false),
      original_camera_(nullptr), camera_(camera_num) {}
  
  bool Create(const CameraState& original_camera) {
    original_camera_ = &original_camera;
    Initialize();
  }
  
  // Initialize
  //  Build the undistorted camera, the undistortion map
  bool Initialize() {
    if (!original_camera_) {LOG(FATAL) << "Original camera is NULL"; return false;}
    
    // Build the new camera state
    camera_ = *original_camera_;   // copy original camera
    camera_.Intrinsics().SetDistortionZero();
    VLOG(2) << "Original camera: " << original_camera_->ToString();
    VLOG(2) << "New camera: " << camera_.ToString();
    
    // Create the undistortion map
    cv::Mat IdentityR = cv::Mat::eye(3, 3, CV_64F);
    cv::initUndistortRectifyMap(original_camera_->Intrinsics().OpenCvCameraMatrix(),
                                original_camera_->Intrinsics().OpenCvDistortionCoeffs(),
                                IdentityR,
                                camera_.Intrinsics().OpenCvCameraMatrix(),
                                camera_.Intrinsics().OpenCvSize(),
                                CV_32FC1,
                                map_x_, map_y_);
    
    is_initialized_ = true;
    return true;
  }
  
  // Undistort method for an opencv image
  bool Undistort(const cv::Mat& image, cv::Mat* undistorted_image) const {
    if (!is_initialized_) {LOG(ERROR) << "Undistorted camera is not initialized."; return false;}
    
    return true;
  }
  
  // Undistort method for an AprilTag message
  bool Undistort(const anantak::AprilTagMessage& apriltag_msg,
                 anantak::AprilTagMessage* undist_apriltag_msg) const {
    if (!is_initialized_) {LOG(ERROR) << "Undistorted camera is not initialized."; return false;}
    undist_apriltag_msg->Clear();
    undist_apriltag_msg->set_camera_num(apriltag_msg.camera_num());
    cv::Mat IdentityR = cv::Mat::eye(3, 3, CV_64F);
    for (int i_tag=0; i_tag<apriltag_msg.tag_id_size(); i_tag++) {
      Matrix2x4d image_coords;
      ExtractAprilTag2dCorners(apriltag_msg, i_tag, &image_coords);
      cv::Mat cvCin(1,4,CV_64FC2);
      for (int i=0; i<4; i++) {
        cvCin.at<cv::Vec2d>(0,i)[0] = image_coords(0,i);
        cvCin.at<cv::Vec2d>(0,i)[1] = image_coords(1,i);
      }
      cv::Mat cvCout = cvCin;
      cv::undistortPoints(cvCin, cvCout,
                          original_camera_->Intrinsics().OpenCvCameraMatrix(),
                          original_camera_->Intrinsics().OpenCvDistortionCoeffs(),
                          IdentityR,
                          camera_.Intrinsics().OpenCvCameraMatrix());
      // Build return message
      undist_apriltag_msg->add_tag_id(apriltag_msg.tag_id(i_tag));
      undist_apriltag_msg->add_u_1(cvCout.at<cv::Vec2d>(0,0)[0]);
      undist_apriltag_msg->add_v_1(cvCout.at<cv::Vec2d>(0,0)[1]);
      undist_apriltag_msg->add_u_2(cvCout.at<cv::Vec2d>(0,1)[0]);
      undist_apriltag_msg->add_v_2(cvCout.at<cv::Vec2d>(0,1)[1]);
      undist_apriltag_msg->add_u_3(cvCout.at<cv::Vec2d>(0,2)[0]);
      undist_apriltag_msg->add_v_3(cvCout.at<cv::Vec2d>(0,2)[1]);
      undist_apriltag_msg->add_u_4(cvCout.at<cv::Vec2d>(0,3)[0]);
      undist_apriltag_msg->add_v_4(cvCout.at<cv::Vec2d>(0,3)[1]);
    }
    
    return true;
  }
  
  
};  // UndistortedCamera


/* Tag corners on calibration target */
class CameraCalibrationTarget {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  std::map<std::string, Matrix3x4d> tag_map_;
  std::map<std::string, double> tag_size_;
  int32_t num_tags_;
  Matrix3d corner_position_covariance_;
  bool is_initialized_;
  
  // Get the config file in constructor, generate the tag-map
  CameraCalibrationTarget(const std::string& config_file):
      tag_map_(), tag_size_(), num_tags_(0), is_initialized_(false) {
    
    // Open config file, read tag offsets
    std::string config_file_path = anantak::GetProjectSourceDirectory() + "/" + config_file;
    std::unique_ptr<anantak::CameraAprilTagCalibrationTargetConfig> config =
        anantak::ReadProtobufFile<anantak::CameraAprilTagCalibrationTargetConfig>(config_file_path);
    if (!config) {
      LOG(ERROR) << "Could not parse the config file. Calibration target was not loaded.";
    } else {
      num_tags_ = config->tags_size();
      VLOG(1) << "Loaded calibration target with name = " << config->name() << " with "
          << num_tags_ << " tags";
      // Generate tag map
      for (int i=0; i<num_tags_; i++) {
        // Get the location of the tag on target
        Eigen::Vector3d tag_posn;
        const anantak::StaticApriltagStateMessage& tagmsg = config->tags(i);
        tag_posn << tagmsg.pose().state(4), tagmsg.pose().state(5), tagmsg.pose().state(6);
        // Get the corners of the tag in tag frame
        Matrix3x4d corners = anantak::AprilTag3dCorners(tagmsg.tag().size());
        corners.colwise() += tag_posn;
        tag_map_[tagmsg.tag().tag_id()] = corners;  // making a copy here, is only done once
        tag_size_[tagmsg.tag().tag_id()] = tagmsg.tag().size(); // tag size is stored here
        VLOG(3) << "Target tag " << i << " id: " << tagmsg.tag().tag_id() << " has corners: \n" << corners;
      }
      // Covariance of corners
      corner_position_covariance_ = kIdentity3d * 0.001 * 0.001;
      is_initialized_ = true;
    }
  }
  
  int32_t Size() const {return tag_map_.size();}
  double TagSize(int32_t i) const {
    if (i>=tag_map_.size()) {
      LOG(ERROR) << "i>=tag_map_.size() " << i << " " << tag_map_.size();
      return -1;
    }
    auto it = tag_size_.cbegin();
    std::advance(it, i);
    return it->second;
  }
  
  bool IsInitialized() const {return is_initialized_;}
  
  // Is this tag present on the calibration target?
  bool TagIsPresent(const std::string& tag_id) const {
    return (tag_map_.find(tag_id) != tag_map_.end());
  }
  
  // Function to return tag corners given the tag id
  const Matrix3x4d* TagCornersPtr(const std::string& tag_id) const {
    if (!TagIsPresent(tag_id)) {
      LOG(ERROR) << "Tag " << tag_id << " is not on calibration target.";
      return nullptr;
    }
    return &tag_map_.at(tag_id);
  }
  
  // Make sure that presence of the tag is checked before using this accessor
  const Matrix3x4d& TagCorners(const std::string& tag_id) const {
    return tag_map_.at(tag_id);
  }
  
  Matrix3x4d TagCorners(const std::string& tag_id, const double& size) {
    Matrix3x4d crnrs = tag_map_.at(tag_id);
    //crnrs += anantak::AprilTag3dCorners(size - tag_size_[tag_id]);
    return crnrs;
  }
  
  const Matrix3d& CornerPositionCovariance() const {
    return corner_position_covariance_;
  }
  
  // Create a target pose from the tag sightings in a single image
  //  Uses p3p algorithm on every tag, then information based fusion of tag poses
  //  ePnP algorithm does not work probably because of distortion, difference in tag poses is large
  bool CalculateTargetPose(
      const anantak::AprilTagMessage& apriltag_msg,
      const anantak::CameraState& camera,
      anantak::PoseState* target_pose_in_camera,
      int64_t timestamp=0) const {
    
    if (apriltag_msg.tag_id_size() == 0) {
      LOG(ERROR) << "No tags seen in the message. Skip.";
      return false;
    }
    
    int32_t num_tags = apriltag_msg.tag_id_size();
    target_pose_in_camera->information_ = kEpsilon*2;
    
    // Camera intrinsics
    const Eigen::Matrix3d& K = camera.K();
    const Eigen::Matrix3d& K_inv = camera.K_inv();
      
    // Generate bearing vectors for every tag, add them to opengv
    for (int i_tag=0; i_tag<num_tags; i_tag++) {
      // Get the tag id
      const std::string& tag_id = apriltag_msg.tag_id(i_tag);   // no need to copy
      
      // Before doing any more work make sure this tag is on the target
      if (!TagIsPresent(tag_id)) {
        VLOG(1) << "Found a tag that is not present on target: " << tag_id;
        continue;
      }
      
      // Calculate Camera to Tag pose
      const Matrix3x4d& Tjpf = tag_map_.at(tag_id);
      
      // Tag coordinates in camera
      Matrix2x4d image_coords;
      image_coords <<
          apriltag_msg.u_1(i_tag), apriltag_msg.u_2(i_tag), apriltag_msg.u_3(i_tag), apriltag_msg.u_4(i_tag),
          apriltag_msg.v_1(i_tag), apriltag_msg.v_2(i_tag), apriltag_msg.v_3(i_tag), apriltag_msg.v_4(i_tag);
          
      Matrix3x4d corners_2d;
      corners_2d.block<2,4>(0,0) = image_coords;
      corners_2d.block<1,4>(2,0) << 1., 1., 1., 1.;
      corners_2d = K_inv * corners_2d;
      corners_2d.colwise().normalize();
      // Report
      VLOG(3) << "    corners_in_cam " << tag_id << " = \n" << corners_2d;
      VLOG(3) << "              Tjpf " << tag_id << " = \n" << Tjpf;
      
      // Calculate tag pose in camera using P3P
      opengv::bearingVectors_t bearing_vecs;
      opengv::points_t points_vec;
      for (int i=0; i<4; i++) {
        bearing_vecs.push_back(corners_2d.col(i));
        points_vec.push_back(Tjpf.col(i));
      }
      opengv::absolute_pose::CentralAbsoluteAdapter adapter(bearing_vecs, points_vec);
      opengv::transformations_t pnp_transformations = opengv::absolute_pose::p3p_kneip(adapter);
      //VLOG(1) << "tag = " << apriltag_msg.tag_id(i_tag)
      //    << " n_tfmtns = " << pnp_transformations.size();
      //VLOG(3) << "corners = " << points_vec[0].transpose() << ", " << points_vec[1].transpose()
      //    << ", " << points_vec[2].transpose() << ", " << points_vec[3].transpose();
      Eigen::Vector4d dffs; dffs << 1e+14, 1e+14, 1e+14, 1e+14;
      Eigen::Matrix<double,3,16> Cpfs; // corner 3d coordinates in camera frame for 4 guesses
      int pnp_sz = pnp_transformations.size();
      for (int i=0; i<std::min(4, pnp_sz); i++) {
        if (pnp_transformations[i].allFinite()) {
          //VLOG(1) << "transformation " << i << "\n" << pnp_transformations[i];
          Eigen::Matrix3d pnp_rotn = pnp_transformations[i].block<3,3>(0,0);
          Eigen::Vector3d pnp_tran = pnp_transformations[i].col(3);
          // Calculate reprojection error
          double total_dff = 0.0;
          for (int j=0; j<4; j++) {
            // TpC, TrC, Tpf. CpT = -CrT*TpC. Cpf = CpT + CrT*Tpf = CrT*(Tpf-TpC)
            Eigen::Vector3d c = pnp_rotn.transpose() * (Tjpf.col(j) - pnp_tran);
            //VLOG(1) << "c = " << c.transpose();
            Eigen::Vector3d Kc = K*c;
            if (i<4 && j<4) {
              // store Kc (=Cpf) in Cpfs matrix
              Cpfs.block<3,4>(0,4*i).col(j) = Kc;
            }
            //VLOG(1) << "Kc = " << Kc.transpose();
            Eigen::Vector2d Kcn; Kcn << Kc(0)/Kc(2), Kc(1)/Kc(2);
            Eigen::Vector2d dff = Kcn - image_coords.col(j);
            //VLOG(1) << "Kcn = " << Kcn.transpose() << " dff = " << dff.squaredNorm();
            total_dff += dff.squaredNorm();
          }
          if (!std::isnan(total_dff)) dffs[i] = total_dff;
        }
      } // for all transformations
      //VLOG(1) << dffs.transpose();
      Eigen::Vector4d::Index min_idx; dffs.minCoeff(&min_idx);
      //VLOG(1) << "Min transformation at " << min_idx;
      double reproj_error = dffs[min_idx];
      
      Eigen::Matrix3d TjrCi = pnp_transformations[min_idx].block<3,3>(0,0);
      Eigen::Vector3d TjpCi = pnp_transformations[min_idx].col(3);
      //Cpf = Cpfs.block<3,4>(0,4*min_idx);
      //VLOG(2) << "Tag "<<tag_id<<" Transform = ("<<reproj_error<<")\n"<<TrC<<"\n"<<TpC;
      
      Eigen::Quaterniond TjqCi = Eigen::Quaterniond(TjrCi);
      Eigen::Matrix3d CirTj(TjrCi.transpose());
      Eigen::Vector3d CipTj = -TjrCi.transpose()*TjpCi;
      
      double information = 1./(TjpCi.norm() * std::max(reproj_error, 1.));
      
      if (std::isnan(information) || information<kEpsilon ||
          !TjrCi.allFinite() || !TjpCi.allFinite()) {
        LOG(ERROR) << "information is nan or <0. information = " << information;
        LOG(ERROR) << "\nTjrCi is not finite. TjrCi = \n" << TjrCi;
        LOG(ERROR) << "\nTjpCi is not finite. TjpCi = " << TjpCi.transpose();
        for (int i=0; i<pnp_transformations.size(); i++)
          LOG(ERROR) << "\npnp_transformations ("<<i<<") = \n" << pnp_transformations[i];
        LOG(ERROR) << "\ndffs = " << dffs.transpose() << "  min_idx = " << min_idx;
        continue;
      }
      
      // Create a temporary PoseState
      anantak::PoseState _pose;
      _pose.SetPose(TjqCi, CipTj);
      _pose.SetInformation(information);
      
      // Add the tag-pose information to target pose
      if (!target_pose_in_camera->UpdatePoseUsingInformation(_pose)) {
        LOG(ERROR) << "Could not add tag pose information to target pose. Skip.";
        continue;
      }
      
    } // for each tag seen
    
    target_pose_in_camera->SetTimestamp(timestamp);
    
    VLOG(3) << "Calculated target pose = " << target_pose_in_camera->ToString();
    
    return true;
  }
  
  virtual ~CameraCalibrationTarget() {}
  
};  // CameraCalibrationTarget


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
  
  static const int kMaxNumberOfResiduals = 280; // = 2*4*35. 35 tags, 4 corners and 2 rows
  
  struct Options {
    double image_sigma;
    Matrix2d image_corner_covariance;
    Options():
      image_sigma(1.0)
    {
      image_corner_covariance = image_sigma*image_sigma*Matrix2d::Identity();
    }
  };
  
  // Options
  CalibrationTargetViewResidual::Options options_;
  
  // Calibration target
  const CameraCalibrationTarget* target_;
  
  // Observations
  const SensorMsg*  message_;
  
  // States
  PoseState*    pose_;
  CameraState*  camera_;
  
  // Residual and Jacobians
  int32_t
      num_residuals_;
  Eigen::Matrix<double,Eigen::Dynamic,1,0,kMaxNumberOfResiduals,1>
      residual_;            // Starting residual
  Eigen::Matrix<double,Eigen::Dynamic,6,0,kMaxNumberOfResiduals,6>
      dcorner_dpose_;       // Jacobian of corners with target pose
  Eigen::Matrix<double,Eigen::Dynamic,9,0,kMaxNumberOfResiduals,9>
      dcorner_dintrinsics_; // Jacobian of corners with camera instrinsics
  
  // Constructor
  CalibrationTargetViewResidual():
    target_(nullptr), message_(nullptr), pose_(nullptr), camera_(nullptr)
  {
    Reset();
  }
  
  // Default copy constructor
  CalibrationTargetViewResidual(const CalibrationTargetViewResidual& r) :
    target_(nullptr), message_(nullptr), pose_(nullptr), camera_(nullptr), num_residuals_(0)
  {
    target_ = r.target_;
    message_ = r.message_;
    pose_= r.pose_;
    camera_ = r.camera_;
    num_residuals_ = r.num_residuals_;
    residual_ = r.residual_;
    dcorner_dpose_ = r.dcorner_dpose_;
    dcorner_dintrinsics_ = r.dcorner_dintrinsics_;
  }
  
  // Equal to assignment operator
  CalibrationTargetViewResidual& operator= (const CalibrationTargetViewResidual& r) {
    target_ = r.target_;
    message_ = r.message_;
    pose_= r.pose_;
    camera_ = r.camera_;
    num_residuals_ = r.num_residuals_;
    residual_ = r.residual_;
    dcorner_dpose_ = r.dcorner_dpose_;
    dcorner_dintrinsics_ = r.dcorner_dintrinsics_;
  }
  
  bool Reset() {
    target_ = nullptr;
    message_ = nullptr;
    pose_ = nullptr;
    camera_ = nullptr;
    num_residuals_ = 0;
    residual_.setZero();
    dcorner_dpose_.setZero();
    dcorner_dintrinsics_.setZero();
    return true;
  }
  
  // Create residual using target pose state and camera state
  bool Create(const CameraCalibrationTarget* target, const SensorMsg* message,
              PoseState* pose, CameraState* camera) {
    if (!target) {LOG(ERROR)<<"Calibration target is null"; return false;}
    if (!message) {LOG(ERROR)<<"Sensor message is null"; return false;}
    if (!pose) {LOG(ERROR)<<"Pose is null"; return false;}
    if (!camera) {LOG(ERROR)<<"Camera is null"; return false;}
    if (!message->has_april_msg()) {LOG(ERROR)<<"Message has no aprilag message"; return false;}
    if (pose->IsZero()) {LOG(WARNING)<<"pose is zero";}
    if (camera->intrinsics_.IsZero()) {LOG(WARNING)<<"camera is zero";}
    
    target_ = target;
    message_ = message;
    pose_ = pose;
    camera_ = camera;
    
    if (!CalculateResiduals()) {
      LOG(ERROR) << "Could not calculate residuals";
      return false;
    }
    
    return true;
  }
  
  // Calculate residuals, jacobians and variances.
  //  First estimates are used for jacobians, meaning that 
  //  residual is recalculated over time but jacobians never change
  bool CalculateResiduals() {
    
    // Target frame = T,  Camera frame = C,  Corner on target = F
    // Target pose in camera frame = CPT
    // Corner position in target frame = TpF
    // Corner position in camera frame = CpF
    
    // How many tags were seen? 
    const AprilTagMessage& apriltag_msg = message_->april_msg();
    if (apriltag_msg.tag_id_size() == 0) {LOG(ERROR)<<"num_tags seen == 0"; return false;}
    
    // Resize residual and jacobian stores to maximum size
    int32_t max_rows = apriltag_msg.tag_id_size()*2*4;  // 4 corners per tag, two indices per crnr
    residual_.resize(max_rows, 1); residual_.setZero();
    dcorner_dpose_.resize(max_rows, 6); dcorner_dpose_.setZero();
    dcorner_dintrinsics_.resize(max_rows, 9); dcorner_dintrinsics_.setZero();
    
    int32_t num_tags = 0;
    int32_t num_corners = 0;
    
    // Go through each tag and build up the residuals
    for (int i_tag=0; i_tag<apriltag_msg.tag_id_size(); i_tag++) {
      
      const std::string& tag_id = apriltag_msg.tag_id(i_tag);
      
      // Does this tag belong to the calibration target?
      if (!target_->TagIsPresent(tag_id)) {
        LOG(WARNING) << "Saw a tag not on target " << tag_id << ". Skipping it.";
        continue;
      }
      
      // Extract the tag corners seen in image
      Matrix2x4d image_coords;
      ExtractAprilTag2dCorners(apriltag_msg, i_tag, &image_coords);
      
      // Corners in target reference frame
      const Matrix3x4d& tag_corners = target_->TagCorners(tag_id);
      
      // Create a residual for each corner
      for (int i_crnr = 0; i_crnr<4; i_crnr++) {
        
        // Position of this corner wrt target
        const Vector3d TpF(tag_corners.col(i_crnr));
        
        // Position of this corner in camera frame
        Vector3d   CpF; CpF.setZero();
        Matrix3x6d dCpF_dCPT; dCpF_dCPT.setZero();
        Matrix3d   dCpF_dTpF; dCpF_dTpF.setZero();
        pose_->AddPoint(TpF, &CpF, &dCpF_dCPT, &dCpF_dTpF);
        
        // Projection of this point on camera image
        Vector2d uv; uv.setZero();
        Matrix2x3d duv_dCpF; duv_dCpF.setZero();
        Matrix2x9d duv_dintrinsics; duv_dintrinsics.setZero(); 
        camera_->intrinsics_.ProjectPoint(CpF, &uv, &duv_dCpF, &duv_dintrinsics);
        
        // Jacobians of projection wrt target pose in camera and corner position on target
        Matrix2x6d duv_dCPT = duv_dCpF * dCpF_dCPT;
        Matrix2x3d duv_dTpF = duv_dCpF * dCpF_dTpF;
        
        // Calculate starting residual
        Vector2d resid = uv - image_coords.col(i_crnr);
        
        // Calculate covariance of the residual
        //  In this residual we are estimating camera intrinsics and target pose. So the variance
        //  of these two should not be a part of the residual covariance.
        Matrix2d var_resid =
            duv_dTpF * target_->CornerPositionCovariance() * duv_dTpF.transpose()
          + options_.image_corner_covariance;
          //+ duv_dCPT * pose_->Covariance() * duv_dCPT.transpose()
          //+ duv_dintrinsics * camera_->intrinsics_.covariance_ * duv_dintrinsics.transpose()
        
        // Calculate inverse sqrt covariance
        Matrix2d inv_sqrt_cov; Vector2d sqrt_var;
        if (!InverseSqrt(var_resid, &inv_sqrt_cov, &sqrt_var)) {
          LOG(ERROR) << "Could not calculate inv sqrt mat";
          continue;
        }
        
        // Report calculations
        //VLOG(1) << "proj, seen, resid = " << " " << uv.transpose() << ",  "
        //    << image_coords.col(i_crnr).transpose() << ",  " << resid.transpose()
        //    << ", sqrt cov: " << sqrt_var.transpose() << ", " << var_resid(0,1)/sqrt_var(0)/sqrt_var(1);
        //VLOG(1) << "inv_sqrt_cov: " << inv_sqrt_cov.row(0) << " " << inv_sqrt_cov.row(1);
        
        // Normalize the residuals and Jacobians to create independent gaussians
        resid = inv_sqrt_cov * resid;
        duv_dCPT = inv_sqrt_cov * duv_dCPT;
        //duv_dTpF = inv_sqrt_cov * duv_dTpF;
        duv_dintrinsics = inv_sqrt_cov * duv_dintrinsics;
        
        // Check if there are any NANs in calculations
        if (!resid.allFinite() || !duv_dCPT.allFinite() ||
            !duv_dTpF.allFinite() || !duv_dintrinsics.allFinite()) {
          LOG(ERROR) << "Found non finite values in residual or jacobians";
          LOG(ERROR) << "resid = " << resid.transpose();
          LOG(ERROR) << "duv_dCPT = \n" << duv_dCPT;
          LOG(ERROR) << "duv_dTpF = \n" << duv_dTpF;
          LOG(ERROR) << "duv_dintrinsics = \n" << duv_dintrinsics;
          continue;
        }
        
        // Add the corner residual to the residual for this cost function
        residual_.block<2,1>(2*num_corners,0) = resid;
        dcorner_dpose_.block<2,6>(2*num_corners,0) = duv_dCPT;
        dcorner_dintrinsics_.block<2,9>(2*num_corners,0) = duv_dintrinsics;
        
        num_corners++;
        
      } // for all four corners
      
      // All was good, so increment the number of tags seen
      num_tags++;
    }
    VLOG(1) << "Used " << num_tags << " of " << apriltag_msg.tag_id_size() << " tags seen. "
        << " num corners: " << num_corners << " (should be " << num_tags*4 << ")";
    
    num_residuals_ = 2*num_corners;
    
    // Resize the residual and jacobians keeping calculated values if needed
    if (num_residuals_ != max_rows) {
      residual_.conservativeResize(num_residuals_, Eigen::NoChange);
      dcorner_dpose_.conservativeResize(num_residuals_, Eigen::NoChange);
      dcorner_dintrinsics_.conservativeResize(num_residuals_, Eigen::NoChange);
      LOG(INFO) << "Resized residuals/jacobians from " << max_rows << " to " << num_residuals_;
    }
    
    // Set number of residuals for solver
    set_num_residuals(num_residuals_);
    VLOG(1) << "Set number of residuals: " << num_residuals_;
    
    // Set parameter block sizes for solver
    mutable_parameter_block_sizes()->push_back(6);  // Target pose has 6 parameters to guess
    mutable_parameter_block_sizes()->push_back(9);  // Camera intrinsics has 9 parameters to guess
    
    return true;
  }
  
  // Add this residual to a problem
  bool AddToProblem(ceres::Problem *problem) {  // can not be const as we need non const *this
    ceres::CostFunction* tvr_cf = this;
    problem->AddResidualBlock(
      tvr_cf, NULL,
      pose_->error_.data(),
      camera_->intrinsics_.error_.data()
    );
    return true;
  }
  
  // Evaluate function for ceres solver
  virtual bool Evaluate(
      double const* const* parameters,
      double* residuals,
      double** jacobians) const {
    
    // Mapping residuals and parameters to matrices
    MapConstVector6d    dpose(parameters[0]);
    MapConstVector9d    dintrinsics(parameters[1]);
    MapVectorXd         residual(residuals, num_residuals_);
    
    // Calculate residual from the error states using linearized model
    residual = residual_ + dcorner_dpose_*dpose + dcorner_dintrinsics_*dintrinsics;
    
    // First estimates Jacobians are used as this shown to keep the estimator consistent
    if (jacobians != NULL) {
      if (jacobians[0] != NULL) {
        Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>>
            jac(jacobians[0], num_residuals_, 6);
        jac = dcorner_dpose_;
      }
      if (jacobians[1] != NULL) {
        Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>>
            jac(jacobians[1], num_residuals_, 9);
        jac = dcorner_dintrinsics_;
      }
    }
    
    return true;
  }
  
  // Destructor
  virtual ~CalibrationTargetViewResidual() {}
};  // CalibrationTargetViewResidual


// Spline that joins points with cubic segements
//  Keeps a circular queue of poses (coming from observations e.g.)
//  Provides first and second derivatives of current pose (expressed in body frame)
//  Provides pose interpolation 
class CubicPoseInterpolator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  std::unique_ptr<anantak::TimedCircularQueue<anantak::PoseState>> poses_;
  
  // Covariances are calculated only using equal diagonal covariance
  int64_t lookback_interval_;
  double sigma2_rotn_;
  double sigma2_posn_;
  double delta_time_;
  Matrix3d cov_helper_;
  double large_variance_;
  
  // Constructor takes in the size of circular queue to be maintained
  CubicPoseInterpolator(int32_t size, double sigma_rotn, double sigma_posn, double dt, int64_t lookback_interval) {
    // Allocate memory for the pose interpolator
    std::unique_ptr<anantak::TimedCircularQueue<anantak::PoseState>> poses_ptr(
        new anantak::TimedCircularQueue<anantak::PoseState>(size));
    poses_ = std::move(poses_ptr);
    LOG(INFO) << "Allocated memeory for TimedCircularQueue of CubicPoseInterpolator " << size;
    
    sigma2_rotn_ = sigma_rotn*sigma_rotn; sigma2_posn_ = sigma_posn*sigma_posn; 
    delta_time_ = dt;
    CalculateCovarianceHelper();
    
    lookback_interval_ = lookback_interval;
    large_variance_ = 1e4;
  }
  
  inline int32_t Size() const {return poses_->n_msgs();}
  
  // Calculate covariance matrix
  bool CalculateCovarianceHelper() {
    cov_helper_.setZero();
    cov_helper_(0,0) = 1.;
    cov_helper_(0,1) = cov_helper_(1,0) = 1.5;
    cov_helper_(0,2) = cov_helper_(2,0) = 1.;
    cov_helper_(1,1) = 6.5;
    cov_helper_(1,2) = cov_helper_(2,1) = 6.;
    cov_helper_(2,2) = 6.;
    VLOG(1) << "Pose interpolator covariance helper = \n" << cov_helper_;
    return true;
  }
  
  inline bool SetCovariance(const Matrix3d& R, double dt, int n_points_used, Matrix19d* cov) const {
    Matrix3d R2 = R*R.transpose();
    Matrix3d R_sigma2_posn  = R * sigma2_posn_;
    Matrix3d R2_sigma2_posn = R2 * sigma2_posn_;
    Matrix3d R_sigma2_rotn  = R * sigma2_rotn_;
    Matrix3d R2_sigma2_rotn = R2 * sigma2_rotn_;
    double dt2 = dt*dt; double dt3 = dt2*dt; double dt4 = dt3*dt;
    cov->setZero();
    
    if (n_points_used == 0) {
      cov->diagonal().setOnes();
      *cov *= large_variance_;
    }
    
    if (n_points_used == 1) {
      cov->block<3,3>(0,0)   = kIdentity3d * sigma2_rotn_;
      cov->block<3,3>(3,3)   = kIdentity3d * sigma2_posn_;
      
      cov->block<3,3>(9,9)   = kIdentity3d * large_variance_;
      cov->block<3,3>(15,15) = kIdentity3d * large_variance_;
      
      cov->block<3,3>(6,6)   = kIdentity3d * large_variance_;
      cov->block<3,3>(12,12) = kIdentity3d * large_variance_;
    }
    
    if (n_points_used == 2) {
      cov->block<3,3>(0,0)   = kIdentity3d * sigma2_rotn_;
      cov->block<3,3>(3,3)   = kIdentity3d * sigma2_posn_;
      
      cov->block<3,3>(3,9)   = cov_helper_(0,1) * R_sigma2_posn.transpose() / dt;
      cov->block<3,3>(9,3)   = cov_helper_(1,0) * R_sigma2_posn / dt;
      cov->block<3,3>(9,9)   = cov_helper_(1,1) * R2_sigma2_posn / dt2;
      
      cov->block<3,3>(0,6)   = cov_helper_(0,1) * R_sigma2_rotn.transpose() / dt;
      cov->block<3,3>(6,0)   = cov_helper_(1,0) * R_sigma2_rotn / dt;
      cov->block<3,3>(6,6)   = cov_helper_(1,1) * R2_sigma2_rotn / dt2;
      
      cov->block<3,3>(12,12) = kIdentity3d * large_variance_;
    }
    
    if (n_points_used == 3) {
      cov->block<3,3>(0,0)   = kIdentity3d * sigma2_rotn_;
      cov->block<3,3>(3,3)   = kIdentity3d * sigma2_posn_;
      
      cov->block<3,3>(3,9)   = cov_helper_(0,1) * R_sigma2_posn.transpose() / dt;
      cov->block<3,3>(3,15)  = cov_helper_(0,2) * R_sigma2_posn.transpose() / dt2;
      cov->block<3,3>(9,3)   = cov_helper_(1,0) * R_sigma2_posn / dt;
      cov->block<3,3>(9,9)   = cov_helper_(1,1) * R2_sigma2_posn / dt2;
      cov->block<3,3>(9,15)  = cov_helper_(1,2) * R2_sigma2_posn / dt3;
      cov->block<3,3>(15,3)  = cov_helper_(2,0) * R_sigma2_posn / dt2;
      cov->block<3,3>(15,9)  = cov_helper_(2,1) * R2_sigma2_posn / dt3;
      cov->block<3,3>(15,15) = cov_helper_(2,2) * R2_sigma2_posn / dt4;
      
      cov->block<3,3>(0,6)   = cov_helper_(0,1) * R_sigma2_rotn.transpose() / dt;
      cov->block<3,3>(0,12)  = cov_helper_(0,2) * R_sigma2_rotn.transpose() / dt2;
      cov->block<3,3>(6,0)   = cov_helper_(1,0) * R_sigma2_rotn / dt;
      cov->block<3,3>(6,6)   = cov_helper_(1,1) * R2_sigma2_rotn / dt2;
      cov->block<3,3>(6,12)  = cov_helper_(1,2) * R2_sigma2_rotn / dt3;
      cov->block<3,3>(12,0)  = cov_helper_(2,0) * R_sigma2_rotn / dt2;
      cov->block<3,3>(12,6)  = cov_helper_(2,1) * R2_sigma2_rotn / dt3;
      cov->block<3,3>(12,12) = cov_helper_(2,2) * R2_sigma2_rotn / dt4;
    }
    
    return true;
  }
  
  
  inline bool AddPose(const PoseState& pose) {
    int64_t ts = pose.Timestamp();
    
    // Checks
    if (ts==0) {LOG(ERROR) << "pose.Timestamp()==0 is not allowed"; return false; }
    if (ts<=poses_->LastTimestamp()) {LOG(ERROR) << "ts <= poses_->LastTimestamp() "
        << ts << " " << poses_->LastTimestamp() << ", diff = " << ts - poses_->LastTimestamp();
      return false;
    }
    
    // Store pose
    poses_->AddElement(pose);
    poses_->SetTimestamp(ts);
    
    return true;
  }
  
  // Add the given pose and calculate derivatives using poses going furthest to lookback_ts.
  // Result is returned in the kinematic state
  //  If no pose previous to this exists, derivatives are zero
  //  If one pose previous to this exists, only velocity is non-zero
  //  If two poses exist, velocity and acceleration can be calculated
  // Covariance of the provided pose is neglected. Cov matrix is generated using sigma_posn/rotn
  inline bool AddPose(const PoseState& pose, KinematicState* kin, KinematicState *ref_kin = nullptr) {
    
    const int64_t ts = pose.Timestamp();
    const int64_t lookback_ts = ts - lookback_interval_;
    
    // Checks
    if (ts==0) {LOG(ERROR) << "pose.Timestamp()==0 is not allowed"; return false; }
    if (ts<=poses_->LastTimestamp()) {LOG(ERROR) << "ts <= poses_->LastTimestamp() "
        << ts << " " << poses_->LastTimestamp() << ", diff = " << ts - poses_->LastTimestamp();}
    
    // Store pose
    poses_->AddElement(pose);
    poses_->SetTimestamp(ts);
    
    // Initiate values to be written to kinematic pose
    Quaterniond B_q_A = pose.Q();
    Vector3d A_p_B = pose.P();
    Matrix3d B_r_A = pose.R();
    
    Vector3d A_w3; A_w3.setZero();
    Vector3d A_v3; A_v3.setZero();
    Vector3d A_dw3; A_dw3.setZero();
    Vector3d A_dv3; A_dv3.setZero();
    double dt = delta_time_;
    int n_points_used = 1;
    
    // Get the second last pose and check if it can be admitted
    const PoseState* second_last_pose = poses_->NthLastElementPtr(2);
    if (second_last_pose) {
      if (second_last_pose->Timestamp() >= lookback_ts) {
        
        // Calculate velocities using ending pose and second-last pose. Notebook #6 ppg 4-9
        double t32 = double(ts - second_last_pose->Timestamp())*1e-6;
        if (t32<kEpsilon) {LOG(ERROR) << "t32<kEpsilon t32 = " << t32 << " " << ts << " "
            << second_last_pose->Timestamp(); return false;}
        Quaterniond dq32 = pose.Q() * second_last_pose->Q().conjugate();
        A_w3 = second_last_pose->R().transpose() * dq32.vec() * 2. / t32;
        A_v3 = (pose.P() - second_last_pose->P()) / t32;
        dt = t32;
        n_points_used++;
        
        // Get third last pose, check if it can be used
        const PoseState* third_last_pose = poses_->NthLastElementPtr(3);
        if (third_last_pose) {
          if (third_last_pose->Timestamp() >= lookback_ts) {
            
            // Calculate acceleration. Notebook #6 ppg 7-9
            double t21 = double(second_last_pose->Timestamp() - third_last_pose->Timestamp())*1e-6;
            Quaterniond dq21 = second_last_pose->Q() * third_last_pose->Q().conjugate();
            Vector3d A_w21 = third_last_pose->R().transpose() * dq21.vec() * 2. / t21;
            Vector3d A_v21 = (third_last_pose->P() - third_last_pose->P()) / t21;
            
            double t31_half = 0.5*(t32 + t21);
            A_dw3 = (A_w3 - A_w21)/t31_half;
            A_dv3 = (A_v3 - A_v21)/t31_half;
            
            A_w3 += A_dw3*t32*0.5;
            A_v3 += A_dv3*t32*0.5;
            
            dt = t31_half;
            n_points_used++;
          }
        }
        
        // We have updated values of velocities and accelerations 
      }
    }
    
    // Set the kin state using calculated values
    kin->SetTimestamp(ts);
    kin->SetPose(B_q_A, A_p_B);
    kin->SetVelocity(B_r_A*A_w3, B_r_A*A_v3);         // Transfer velocities to body frame
    kin->SetAcceleration(B_r_A*A_dw3, B_r_A*A_dv3);   // Transfer accelerations to body frame
    // Set algorithm time if a reference state is provided
    if (ref_kin) {
      double _t = double(ts - ref_kin->Timestamp())*1e-6 + ref_kin->T()(0);
      kin->SetTime(_t);
    }
    // Set covariance matrix of kinematic state
    SetCovariance(B_r_A, dt, n_points_used, &kin->covariance_);
    
    KinematicState test_K;
    VLOG(1) << "********Testing interpolation*******";
    Interpolate(ts, &test_K, ref_kin);
    VLOG(1) << "Interpolated state = " << test_K.ToString(2);
    VLOG(1) << "********Testing interpolation*******";
    
    return true;
  }
  
  // Interpolate or extrapolate using the poses
  bool Interpolate(int64_t timestamp, KinematicState* kin, const KinematicState* ref_kin = nullptr)
      const {
    
    int32_t size = poses_->n_msgs();
    const int64_t lookback_ts = timestamp - lookback_interval_;
    if (size<1) {LOG(ERROR)<<"There are no poses, can not interpolate"; return false;}
    if (!kin) {LOG(ERROR)<<"kin is null"; return false;}
    
    if (size==1) {
      kin->SetTimestamp(timestamp);
      kin->SetPose(poses_->Back());
      if (ref_kin) {
        double _t = double(timestamp - ref_kin->Timestamp())*1e-6 + ref_kin->T()(0);
        kin->SetTime(_t);
        kin->covariance_ = ref_kin->covariance_;
        kin->SetVelocity(ref_kin->W(), ref_kin->V());
        kin->SetAcceleration(ref_kin->dW(), ref_kin->dV());
      }
      return true;
    }
    
    VLOG(2) << "Interpolating using splines for timestamp: " << timestamp;
    
    // indexes start from the end with last element index=1. So first element index is size
    int32_t idx0 = 0;
    if (timestamp < poses_->FirstTimestamp()) {
      idx0 = size;
    }
    else if (timestamp > poses_->LastTimestamp()) {
      idx0 = 1;
    }
    else {
      for (int i=1; i<size; i++) {
        if (poses_->NthLastTimestamp(i+1)<=timestamp && timestamp<=poses_->NthLastTimestamp(i)) {
          idx0 = i;
          break;
        }
      }
    }
    
    if (idx0==0) {LOG(FATAL) << "idx0==0 ts " << timestamp
        << " first " << poses_->FirstTimestamp() << " last " << poses_->LastTimestamp(); return false;}
    
    // Interpolate
    const PoseState* last_pose = poses_->NthLastElementPtr(idx0);
    if (!last_pose) {LOG(FATAL) << "last_pose is null"; return false;}
    
    // Initiate values to be written to kinematic pose
    Quaterniond B_q_A = last_pose->Q();
    Vector3d A_p_B = last_pose->P();
    Matrix3d B_r_A = last_pose->R();
    
    Vector3d A_w3; A_w3.setZero();
    Vector3d A_v3; A_v3.setZero();
    Vector3d A_dw3; A_dw3.setZero();
    Vector3d A_dv3; A_dv3.setZero();
    double dt = 0.;
    int n_points_used = 1;
    
    // Get the second last pose and check if it can be admitted
    const PoseState* second_last_pose = poses_->NthLastElementPtr(idx0+1);
    if (second_last_pose) {
      if (second_last_pose->Timestamp() >= lookback_ts) {
        
        // Calculate velocities using ending pose and second-last pose. Notebook #6 ppg 4-9
        double t32 = double(last_pose->Timestamp() - second_last_pose->Timestamp())*1e-6;
        if (t32<kEpsilon) {LOG(ERROR) << "t32<kEpsilon t32 = " << t32 << " " << last_pose->Timestamp() << " "
            << second_last_pose->Timestamp(); return false;}
        Quaterniond dq32 = last_pose->Q() * second_last_pose->Q().conjugate();
        A_w3 = second_last_pose->R().transpose() * dq32.vec() * 2. / t32;
        A_v3 = (last_pose->P() - second_last_pose->P()) / t32;
        dt = t32;
        n_points_used++;
        
        // Get third last pose, check if it can be used
        const PoseState* third_last_pose = poses_->NthLastElementPtr(idx0+2);
        if (third_last_pose) {
          if (third_last_pose->Timestamp() >= lookback_ts) {
            
            // Calculate acceleration. Notebook #6 ppg 7-9
            double t21 = double(second_last_pose->Timestamp() - third_last_pose->Timestamp())*1e-6;
            Quaterniond dq21 = second_last_pose->Q() * third_last_pose->Q().conjugate();
            Vector3d A_w21 = third_last_pose->R().transpose() * dq21.vec() * 2. / t21;
            Vector3d A_v21 = (third_last_pose->P() - third_last_pose->P()) / t21;
            
            double t31_half = 0.5*(t32 + t21);
            A_dw3 = (A_w3 - A_w21)/t31_half;
            A_dv3 = (A_v3 - A_v21)/t31_half;
            
            A_w3 += A_dw3*t32*0.5;
            A_v3 += A_dv3*t32*0.5;
            
            dt = t31_half;
            n_points_used++;
          }
        }
        
        // We have updated values of velocities and accelerations 
      }
    }
    
    // Make adjustments for the pose timestamp
    double dtm = double(timestamp - last_pose->Timestamp()) * 1e-6;
    if (std::abs(dtm) > kEpsilon) {
      // Adjust pose
      Vector3d A_aa = A_w3*dtm + 0.5*A_dw3*dtm*dtm;
      Vector3d B_aa = Eigen::Matrix3d(B_q_A)*A_aa;
      Quaterniond B_dq = ErrorAngleAxisToQuaternion(B_aa);
      B_q_A = B_dq * B_q_A;
      Vector3d A_dp = A_v3*dtm + 0.5*A_dv3*dtm*dtm;
      A_p_B += A_dp;
      // Adjust velocities
      A_w3 += A_dw3*dtm;
      A_v3 += A_dv3*dtm;
      // No change to accelerations
    }
    
    // Set the kin state using calculated values
    kin->SetTimestamp(timestamp);
    kin->SetPose(B_q_A, A_p_B);
    kin->SetVelocity(B_r_A*A_w3, B_r_A*A_v3);         // Transfer velocities to body frame
    kin->SetAcceleration(B_r_A*A_dw3, B_r_A*A_dv3);   // Transfer accelerations to body frame
    // Set algorithm time if a reference state is provided
    if (ref_kin) {
      double _t = double(timestamp - ref_kin->Timestamp())*1e-6 + ref_kin->T()(0);
      kin->SetTime(_t);
    }
    // Set covariance matrix of kinematic state
    SetCovariance(B_r_A, dt, n_points_used, &kin->covariance_);
    
    return true;
  }
  
  
};  // CubicPoseInterpolator


/** Kinematic Calibration target view residual
 *    Connects a target view with a kinematic state and a camera state.
 *
 *  In: Observation of target as a AprilTag message - corner coordinates of tags.
 *  In: Camera state - this is the state of the camera that observed the target. This has three
 *      parameters (1) intrinsics (including distortion) (2) extrinsics (pose) (3) time delay.
 *      Camera is assumed to be static.
 *  In: Kinematic state - that marks the beginning of the interval where observation timestamp +
 *      camera time-delay lies.
 *
 *  Guess: Kinematic state - this is always to be guessed. So its variance is NOT used in residual.
 *  Guess: Camera state - we may/may not want to guess camera state depending on the problem. If a
 *      state is to be guessed, its variance should not be included in the residual variance. So
 *      we need to know what problem is being solved.
 *
 *  Notes:
 *    - This residual could be used to solve for: (1) Only kinematic state of target, and/or
 *      (2) portions of the camera state. We could only be solving for camera intrinsics. Or
 *      we have intrinsics known but we are solving for extrinsics and time delay.
 *    - Depending on what is being solved, variance of the residual will be adjusted. If only
 *      kinematic state is being solved, while keeping camera state fixed, we will include the
 *      camera state variance in the error variance.
 *    - Problems we could solve:
 *        (a) Kinematics of target only, camera is known
 *        (b) Kinematics + Intrinsics
 *        (c) Kinematics + Extrinsics
 *        (d) Kinematics + Delay
 *        (e) Kinematics + Intrinsics + Extrinsics + Delay
 */
class KinematicCalibrationTargetViewResidual : public ceres::CostFunction {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  static const int kMaxNumberOfResiduals = 280; // = 2*4*35. 35 tags, 4 corners and 2 indices
  
  struct Options {
    bool solve_for_intrinsics;  // default false. Solve for camera intrinsics?
    bool solve_for_extrinsics;  // default false. Solve for camera pose?
    bool solve_for_timedelay;   // default false. Solve for camera timedelay?
    
    double image_sigma;         // sigma of the corner seen in image. In pixels.
    
    Matrix6d   acceleration_covariance;     // inadequate model fit for acceleration
    
    Matrix2d image_corner_covariance; // Covariance matrix for tag corner view.
    Options(const double edw=0., const double edv=0.):
        image_sigma(1.0),
        solve_for_intrinsics(false), solve_for_extrinsics(false), solve_for_timedelay(false) {
      image_corner_covariance = image_sigma*image_sigma*Matrix2d::Identity();
      
      // Moving in time 
      double edw2 = edw*edw;
      double edv2 = edv*edv;
      Vector6d accl_cov_diag; accl_cov_diag << edw2, edw2, edw2, edv2, edv2, edv2;
      acceleration_covariance = accl_cov_diag.asDiagonal();
    }
  };
  
  // Options
  const KinematicCalibrationTargetViewResidual::Options* options_;
  
  // Calibration target
  const CameraCalibrationTarget* target_;
  
  // Observation
  int64_t           timestamp_;         // Timestamp of the observation
  PoseState         target_pose_est_;   // This is an calculated pose from the message
  
  // States
  KinematicState*   kin_;
  CameraState*      camera_;
  
  // Residual and Jacobians
  int32_t   num_residuals_;
  Eigen::Matrix<double,Eigen::Dynamic,1,0,kMaxNumberOfResiduals,1>
      residual_;            // Starting residual
  Eigen::Matrix<double,Eigen::Dynamic,18,0,kMaxNumberOfResiduals,18>
      dcorner_dkin_;        // Jacobian of corners with target kinematic state
  //Eigen::Matrix<double,Eigen::Dynamic,6,0,kMaxNumberOfResiduals,6>
  //    dcorner_dkin_;        // Jacobian of corners with target kinematic state
  Eigen::Matrix<double,Eigen::Dynamic,9,0,kMaxNumberOfResiduals,9>
      dcorner_dintrinsics_; // Jacobian of corners with camera instrinsics
  Eigen::Matrix<double,Eigen::Dynamic,6,0,kMaxNumberOfResiduals,6>
      dcorner_dextrinsics_; // Jacobian of corners with camera extrinsics
  Eigen::Matrix<double,Eigen::Dynamic,1,0,kMaxNumberOfResiduals,1>
      dcorner_dtimedelay_; // Jacobian of corners with camera time delay
  
  // Constructor - expects pointer to options. Options do not change after construction
  KinematicCalibrationTargetViewResidual(
      const KinematicCalibrationTargetViewResidual::Options* options):
      options_(options), target_(nullptr), kin_(nullptr), camera_(nullptr),
      timestamp_(0), target_pose_est_() {
    Reset();
  }
  
  // Reset residual - this does not change the options
  bool Reset() {
    // options_ are not modified
    target_ = nullptr; kin_ = nullptr; camera_ = nullptr;
    timestamp_ = 0; target_pose_est_.SetZero();
    num_residuals_ = 0; residual_.setZero();
    dcorner_dkin_.setZero(); dcorner_dintrinsics_.setZero();
    dcorner_dintrinsics_.setZero(); dcorner_dtimedelay_.setZero();
    return true;
  }
  
  // Default copy constructor
  KinematicCalibrationTargetViewResidual(const KinematicCalibrationTargetViewResidual& r) :
      options_(r.options_),
      target_(r.target_), kin_(r.kin_), camera_(r.camera_),
      timestamp_(0), target_pose_est_(r.target_pose_est_),
      num_residuals_(r.num_residuals_), residual_(r.residual_),
      dcorner_dkin_(r.dcorner_dkin_), dcorner_dintrinsics_(r.dcorner_dintrinsics_),
      dcorner_dextrinsics_(r.dcorner_dextrinsics_), dcorner_dtimedelay_(r.dcorner_dtimedelay_) {}
  
  // Equal to assignment operator
  KinematicCalibrationTargetViewResidual& operator= (const KinematicCalibrationTargetViewResidual& r) {
    options_ = r.options_;
    target_ = r.target_; kin_= r.kin_; camera_ = r.camera_;
    timestamp_ = r.timestamp_; target_pose_est_ = r.target_pose_est_;
    num_residuals_ = r.num_residuals_; residual_ = r.residual_;
    dcorner_dkin_ = r.dcorner_dkin_; dcorner_dintrinsics_ = r.dcorner_dintrinsics_;
    dcorner_dextrinsics_ = r.dcorner_dextrinsics_; dcorner_dtimedelay_ = r.dcorner_dtimedelay_;
  }
  
  // Create residual using target pose state and camera state
  bool Create(const CameraCalibrationTarget* target, KinematicState* kin, CameraState* camera,
              const SensorMsg& message,
              const InterKinematicStateResidual* inter_kin_resid = nullptr,
              const UndistortedCamera* undistorted_camera = nullptr,
              const CubicPoseInterpolator* pose_interpolator = nullptr,
              cv::Mat* plot_image = nullptr) {
    // Checks
    if (!target) {LOG(ERROR)<<"Calibration target is null"; return false;}
    if (!kin) {LOG(ERROR)<<"Kinematic state is null"; return false;}
    if (!camera) {LOG(ERROR)<<"Camera is null"; return false;}
    if (!message.has_header()) {LOG(ERROR)<<"Message has no header"; return false;}
    if (!message.has_april_msg()) {LOG(ERROR)<<"Message has no aprilag message"; return false;}
    if (kin->IsZero()) {LOG(WARNING)<<"pose is zero";}
    if (camera->intrinsics_.IsZero()) {LOG(WARNING)<<"camera is zero";}
    //if (!kin->IsErrorZero()) {LOG(WARNING)<<"Kin error is not zero. "
    //      << kin->ToString(2) << "\n Error: " << kin->error_.transpose();}
    
    target_ = target;
    kin_ = kin;
    camera_ = camera;
    timestamp_ = message.header().timestamp();
    
    // Number of residuals for the solver
    //set_num_residuals(6);
    set_num_residuals(18);
    // Set parameter block sizes for solver - kin state, camera intrinsics, extrinsics, timedelay
    //mutable_parameter_block_sizes()->push_back(12); // Target kinematic state has 12 parameters for constant velocity model
    mutable_parameter_block_sizes()->push_back(18); // Target kinematic state has 18 parameters 
    //mutable_parameter_block_sizes()->push_back(6); // Kinematic state has 6 acceleration parameters to solve
    
    // Calculate target pose if undistorted camera has been provided
    if (undistorted_camera) {
      anantak::AprilTagMessage undist_apriltag_msg;
      undistorted_camera->Undistort(message.april_msg(), &undist_apriltag_msg);
      if (!target_->CalculateTargetPose(undist_apriltag_msg, undistorted_camera->camera_,
                                        &target_pose_est_, timestamp_)) {
        LOG(WARNING) << "Could not calculate target pose";
      }
    }
    
    //if (!CalculateResiduals(message, inter_kin_resid, plot_image)) {
    if (!CalculateResiduals(inter_kin_resid, pose_interpolator)) {
      LOG(ERROR) << "Could not calculate residuals";
      return false;
    }
    
    return true;
  }
  
  // Return target pose as a vector
  bool TargetPoseToVector(Vector8d* vec) const {
    if (!vec) {LOG(ERROR)<<"Input vec is null"; return false;}
    vec->setZero();
    vec->block<7,1>(0,0) = target_pose_est_.state_;
    double ts = double(timestamp_ - kin_->Timestamp())*1e-6 + kin_->T()(0);
    (*vec)(7,0) = ts;
    return true;
  }
  
  // Calculate residuals using the estimated pose
  bool CalculateResiduals(const InterKinematicStateResidual* inter_kin_resid = nullptr,
                          const CubicPoseInterpolator* pose_interpolator = nullptr) {
    
    // Non-zero camera pose is not implemented yet
    if (!camera_->Pose().IsZero()) {
      LOG(ERROR) << "Non zero camera pose is not implemented yet."; return false;}
    
    // Calculate target velocity and acceleration using past poses interpolated with B-splines
    //  This will give a target kinematic state estimate
    //  Add this pose estimate to the interpolator
    //  Get an estimate of two derivatives (velo and accel) using 3rd order spline, do not go more than so many seconds back
    //    this looks back at the data it has in the lookback time limit,
    //    generates the spline. So if cubic spline was asked for but only two points are there,
    //      a linear spline will be built. velocity will be calculated, and acceleration will be set to zero.
    if (!pose_interpolator) {LOG(ERROR) << "Need a pose interpolator to operate"; return false;}
    KinematicState target_kin_est_;
    //pose_interpolator->AddPose(target_pose_est_, &target_kin_est_);
    pose_interpolator->Interpolate(timestamp_, &target_kin_est_);
    VLOG(1) << "Pose estimate = " << target_pose_est_.ToString();
    VLOG(1) << "Kinematic state estimate from pose observations = " << target_kin_est_.ToString(2);
    
    // Time gap between kinematic state and observation
    //  Observation timestamp is a measurement - assumed to have no variance
    //  Kinematic state timestamp is deterministic as it is generated and linked to wall time
    //  Camera time delay is an estimate with a variance
    double time_gap = double(timestamp_ - kin_->Timestamp())*1e-6 + camera_->TimeDelay().Value();
    double time_gap_var = camera_->TimeDelay().Var();   // camera delay is stochastic
    VLOG(1) << "time_gap: " << time_gap << "  ~  " << time_gap_var;
    if (time_gap < 0) {LOG(ERROR) << "time_gap between kinematic state and observation is negative "
        << time_gap; return false;}
    if (time_gap_var < 0) {LOG(ERROR) << "time_gap_var is negative " << time_gap_var; return false;}
    
    if (!kin_) {LOG(ERROR)<<"!kin_"; return false;}
    const KinematicState& K0 = *kin_;
    
    Vector18d dK_prediction;
    Matrix18d J_prediction_K0;
    Matrix18d prediction_cov;
    if (!inter_kin_resid) {
      // Full calculation
      if (!MoveInFixedTime(K0, time_gap, &options_->acceleration_covariance,
                           &dK_prediction, &J_prediction_K0, &prediction_cov)) {
        LOG(ERROR) << "MoveInTime calculation was not successful. Quit.";
        return false;
      }
    } else {
      // Interpolated approximation
      VLOG(1) << "Calculating jacobians using interpolation of inter-state residual.";
      // kin_ should be the same as the beginning state of inter state resid
      if (kin_ != inter_kin_resid->K0_) {
        LOG(FATAL) << "Expected to see kin_ same as inter state resid K0_";
        return false;
      }
      if (!inter_kin_resid->InterpolateResidual(time_gap,
                           &dK_prediction, &J_prediction_K0, &prediction_cov)) {
        LOG(ERROR) << "Interpolation calculation was not successful. Quit.";
        return false;
      }
    }
    
    Matrix18d J_measurement_P1(kIdentity18d);
    J_measurement_P1.block<3,3>(0,0) = kin_->R();
    J_measurement_P1.block<3,3>(3,3) = kin_->R();
    
    Matrix18d J_measurement_K0(J_measurement_P1);
    J_measurement_K0 *= -1.;
    J_measurement_K0.block<3,3>(3,0) =  kin_->R()*SkewSymm3d(target_kin_est_.P() - kin_->P());
    
    Vector19d dP_measurement;
    if (!DiffKinStates(K0, target_kin_est_, &dP_measurement)) {
      LOG(ERROR) << "DiffKinWithPose to calculate residual had an error. Quit.";
      return false;
    }
    
    residual_.resize(18,1); residual_.setZero();
    residual_ = dP_measurement.block<18,1>(0,0) - dK_prediction;
    
    dcorner_dkin_.resize(18,18); dcorner_dkin_.setZero();
    dcorner_dkin_ = J_measurement_K0 - J_prediction_K0;
    //dcorner_dkin_.resize(18,6); dcorner_dkin_.setZero();
    //dcorner_dkin_ = J_measurement_K0.block<18,6>(12,0) - J_prediction_K0.block<18,6>(12,0);
    
    // Assume a fixed variance of error in target pose estimate
    //Vector6d target_pose_est_var;
    //double ew = 2.*kRadiansPerDegree; ew *= ew;
    //double ev = 0.010; ev *= ev;
    //target_pose_est_var << ew, ew, ew, ev, ev, ev;
    //Matrix6d target_pose_est_cov = target_pose_est_var.asDiagonal();
    
    // Variance of the residual
    Matrix18d residual_cov = 
        J_measurement_P1 * target_kin_est_.covariance_.block<18,18>(0,0) * J_measurement_P1.transpose()
        + prediction_cov;
    
    // Report
    VLOG(1) << "Kin to pose residual: \n" << residual_.transpose();
    VLOG(1) << "Kin to pose residual sqrt cov diagonal: \n" << residual_cov.diagonal().cwiseSqrt().transpose();
    
    // Inv sqrt of the variance
    Matrix18d inv_sqrt_cov; Vector18d sqrt_var;
    if (!InverseSqrt(residual_cov, &inv_sqrt_cov, &sqrt_var)) {
      LOG(ERROR) << "Could not calculate inv sqrt mat";
      return false;
    }
    
    // Scale the residual and jacobian
    residual_ = inv_sqrt_cov * residual_;
    dcorner_dkin_ = inv_sqrt_cov * dcorner_dkin_;
    
    // Check if there are any NANs in the calculations
    if (!residual_.allFinite() || !dcorner_dkin_.allFinite()) {
      LOG(ERROR) << "Found non finite values in residual or jacobians";
      LOG(ERROR) << "residual_ = " << residual_.transpose();
      LOG(ERROR) << "dcorner_dkin_ = \n" << dcorner_dkin_;
      return false;
    }
    
    return true;
  }
  
  // Add this residual to a problem
  bool AddToProblem(ceres::Problem *problem, ceres::LossFunction* loss_func = nullptr) {
    // Add the residual 
    ceres::CostFunction* cf = this;
    problem->AddResidualBlock(
      cf, loss_func,
      kin_->error_.data()                  // 18d
      //kin_->edwdv_.data()                  // 6d
    );
    return true;
  }

  // Update residual to incorporate the change in errors states
  //  Jacobians are not updated
  bool ResetKinErrorToZero() {
    residual_ += dcorner_dkin_ * kin_->error_.block<18,1>(0,0);
    //residual_ += dcorner_dkin_ * kin_->edwdv_;
  }
  
  virtual bool Evaluate(
      double const* const* parameters,
      double* residuals,
      double** jacobians) const {
    
    // Mapping residuals and parameters to matrices
    MapVector18d        residual(residuals);
    MapConstVector18d   dkin(parameters[0]);
    
    // Calculate residual from the error states using linearized model
    residual = residual_ + dcorner_dkin_ * dkin;
    
    // First estimates Jacobians are used as this shown to keep the estimator consistent
    if (jacobians != NULL) {
      if (jacobians[0] != NULL) {
        Eigen::Map<Eigen::Matrix<double,18,18,Eigen::RowMajor>> jac(jacobians[0]);
        jac = dcorner_dkin_;
        //Eigen::Map<Eigen::Matrix<double,18,6,Eigen::RowMajor>> jac(jacobians[0]);
        //jac = dcorner_dkin_;
      }
    }
    
    return true;
  }
  
  /*virtual bool Evaluate(
      double const* const* parameters,
      double* residuals,
      double** jacobians) const {
    
    // Mapping residuals and parameters to matrices
    MapVector6d         residual(residuals);
    //MapConstVector12d   dkin(parameters[0]);
    MapConstVector18d   dkin(parameters[0]);
    
    // Calculate residual from the error states using linearized model
    //residual = residual_ + dcorner_dkin_.block<6,12>(0,0) * dkin;
    residual = residual_ + dcorner_dkin_ * dkin;
    
    // First estimates Jacobians are used as this shown to keep the estimator consistent
    if (jacobians != NULL) {
      if (jacobians[0] != NULL) {
        //Eigen::Map<Eigen::Matrix<double,6,12,Eigen::RowMajor>> jac(jacobians[0]);
        //jac = dcorner_dkin_.block<6,12>(0,0);
        Eigen::Map<Eigen::Matrix<double,6,18,Eigen::RowMajor>> jac(jacobians[0]);
        jac = dcorner_dkin_;
      }
    }
    
    return true;
  }
  
  // Calculate residuals using the estimated pose
  bool CalculateResiduals(const InterKinematicStateResidual* inter_kin_resid = nullptr,
                          CubicPoseInterpolator* pose_interpolator = nullptr) {
    
    // Non-zero camera pose is not implemented yet
    if (!camera_->Pose().IsZero()) {
      LOG(ERROR) << "Non zero camera pose is not implemented yet."; return false;}
    
    // Calculate target velocity and acceleration using past poses interpolated with B-splines
    //  This will give a target kinematic state estimate
    //  Add this pose estimate to the interpolator
    //  Get an estimate of two derivatives (velo and accel) using 3rd order spline, do not go more than so many seconds back
    //    this looks back at the data it has in the lookback time limit,
    //    generates the spline. So if cubic spline was asked for but only two points are there,
    //      a linear spline will be built. velocity will be calculated, and acceleration will be set to zero.
    if (!pose_interpolator) {LOG(ERROR) << "Need a pose interpolator to operate"; return false;}
    KinematicState K_;
    pose_interpolator->AddPose(target_pose_est_, &K_);
    
    // Time gap between kinematic state and observation
    //  Observation timestamp is a measurement - assumed to have no variance
    //  Kinematic state timestamp is deterministic as it is generated and linked to wall time
    //  Camera time delay is an estimate with a variance
    double time_gap = double(timestamp_ - kin_->Timestamp())*1e-6 + camera_->TimeDelay().Value();
    double time_gap_var = camera_->TimeDelay().Var();   // camera delay is stochastic
    VLOG(1) << "time_gap: " << time_gap << "  ~  " << time_gap_var;
    if (time_gap < 0) {LOG(ERROR) << "time_gap between kinematic state and observation is negative "
        << time_gap; return false;}
    if (time_gap_var < 0) {LOG(ERROR) << "time_gap_var is negative " << time_gap_var; return false;}
    
    if (!kin_) {LOG(ERROR)<<"!kin_"; return false;}
    const KinematicState& K0 = *kin_;
    
    Vector18d dK_prediction;
    Matrix18d J_prediction_K0;
    Matrix18d prediction_cov;
    if (!inter_kin_resid) {
      // Full calculation
      if (!MoveInFixedTime(K0, time_gap, &options_->acceleration_covariance,
                           &dK_prediction, &J_prediction_K0, &prediction_cov)) {
        LOG(ERROR) << "MoveInTime calculation was not successful. Quit.";
        return false;
      }
    } else {
      // Interpolated approximation
      VLOG(1) << "Calculating jacobians using interpolation of inter-state residual.";
      // kin_ should be the same as the beginning state of inter state resid
      if (kin_ != inter_kin_resid->K0_) {
        LOG(FATAL) << "Expected to see kin_ same as inter state resid K0_";
        return false;
      }
      if (!inter_kin_resid->InterpolateResidual(time_gap,
                           &dK_prediction, &J_prediction_K0, &prediction_cov)) {
        LOG(ERROR) << "Interpolation calculation was not successful. Quit.";
        return false;
      }
    }
    
    Matrix6d J_measurement_P1;
    J_measurement_P1.setZero();
    J_measurement_P1.block<3,3>(0,0) = kin_->R();
    J_measurement_P1.block<3,3>(3,3) = kin_->R();
    
    Matrix6x18d J_measurement_K0;
    J_measurement_K0.setZero();
    J_measurement_K0.block<3,3>(0,0) = -kin_->R();
    J_measurement_K0.block<3,3>(3,3) = -kin_->R();
    J_measurement_K0.block<3,3>(3,0) =  kin_->R()*SkewSymm3d(target_pose_est_.P() - kin_->P());
    
    Vector6d dP_measurement;
    if (!DiffKinWithPose(K0, target_pose_est_, &dP_measurement)) {
      LOG(ERROR) << "DiffKinWithPose to calculate residual had an error. Quit.";
      return false;
    }
    
    residual_.resize(6, 1); residual_.setZero();
    residual_ = dP_measurement - dK_prediction.block<6,1>(0,0);
    
    dcorner_dkin_.resize(6,18); dcorner_dkin_.setZero();
    dcorner_dkin_ = J_measurement_K0 - J_prediction_K0.block<6,18>(0,0);
    
    // Assume a fixed variance of error in target pose estimate
    Vector6d target_pose_est_var;
    double ew = 2.*kRadiansPerDegree; ew *= ew;
    double ev = 0.010; ev *= ev;
    target_pose_est_var << ew, ew, ew, ev, ev, ev;
    Matrix6d target_pose_est_cov = target_pose_est_var.asDiagonal();
    
    // Variance of the residual
    Matrix6d residual_cov = 
        J_measurement_P1 * target_pose_est_cov * J_measurement_P1.transpose()
      + prediction_cov.block<6,6>(0,0);
    
    // Report
    VLOG(1) << "Kin to pose residual: \n" << residual_.transpose();
    VLOG(1) << "Kin to pose residual sqrt cov diagonal: \n" << residual_cov.diagonal().cwiseSqrt().transpose();
    
    // Inv sqrt of the variance
    Matrix6d inv_sqrt_cov; Vector6d sqrt_var;
    if (!InverseSqrt(residual_cov, &inv_sqrt_cov, &sqrt_var)) {
      LOG(ERROR) << "Could not calculate inv sqrt mat";
      return false;
    }
    
    // Scale the residual and jacobian
    residual_ = inv_sqrt_cov * residual_;
    dcorner_dkin_ = inv_sqrt_cov * dcorner_dkin_;
    
    // Check if there are any NANs in the calculations
    if (!residual_.allFinite() || !dcorner_dkin_.allFinite()) {
      LOG(ERROR) << "Found non finite values in residual or jacobians";
      LOG(ERROR) << "residual_ = " << residual_.transpose();
      LOG(ERROR) << "dcorner_dkin_ = \n" << dcorner_dkin_;
      return false;
    }
    
    return true;
  }*/
  
  // Calculate residuals, jacobians and variances.
  //  Jacobians use first estimates. Residual is recalculated over time but jacobians never change.
  /*bool CalculateResiduals2(const SensorMsg& message,
                          const InterKinematicStateResidual* inter_kin_resid = nullptr,
                          cv::Mat* plot_image = nullptr) {
    
    // Target frame = T,  Camera frame = C,  Corner on target = F
    // Target pose in camera frame = CPT
    // Corner position in target frame = TpF
    // Corner position in camera frame = CpF
    
    // Non-zero camera pose is not implemented yet
    if (!camera_->Pose().IsZero()) {
      LOG(ERROR) << "Non zero camera pose is not implemented yet."; return false;}
    
    // Time gap between kinematic state and observation
    //  Observation timestamp is a measurement - assumed to have no variance
    //  Kinematic state timestamp is deterministic as it is generated and linked to wall time
    //  Camera time delay is an estimate with a variance
    double time_gap = double(timestamp_ - kin_->Timestamp())*1e-6 + camera_->TimeDelay().Value();
    double time_gap_var = camera_->TimeDelay().Var();   // camera delay is stochastic
    VLOG(1) << "time_gap: " << time_gap << "  ~  " << time_gap_var;
    if (time_gap < 0) {LOG(ERROR) << "time_gap between kinematic state and observation is negative "
        << time_gap; return false;}
    if (time_gap_var < 0) {LOG(ERROR) << "time_gap_var is negative " << time_gap_var; return false;}
    
    // Move forward in time from kin pose
    //  If inter_kin_resid is provided, it can be used for a quick interpolated calculation
    //  If not, full integration needs to be run. This will be more accurate, but time consuming.
    PoseState    CPT;        // Projected target pose at observation time
    Matrix6x18d  dCPT_dkin;  // Jacobian of pose at observation time with kin state
    Vector6d     dCPT_dtd;   // Jacobian of pose at observation time with time delay
    if (inter_kin_resid) {
      // kin_ should be the same as the beginning state of inter state resid
      if (kin_ != inter_kin_resid->K0_) {
        LOG(ERROR) << "Expected to see kin_ same as inter state resid K0_"; return false;}
      // Use interpolation for pose and jacobian calculation
      if (!inter_kin_resid->InterpolatePose(time_gap, time_gap_var, &CPT, &dCPT_dkin, &dCPT_dtd)) {
        LOG(ERROR) << "Could not interpolate kinematic poses. Quit.";
        return false;
      }
      VLOG(1) << "Interpolated pose = " << CPT.ToString();
    } else {
      // Run full integration
      const KinematicState& K0 = *kin_;
      KinematicState K1; Matrix19d dK1_dK0; Vector19d dK1_dt1;
      //if (!MoveInTime(K0, time_gap, time_gap_var, &K1, &dK1_dK0, &dK1_dt1)) {
      //  LOG(ERROR) << "Could not move in time";
      //  return false;
      //}
      K1.ToPose(&CPT);
      dCPT_dkin = dK1_dK0.block<6,18>(0,0);
      dCPT_dtd = dK1_dt1.block<6,1>(0,0);
    }
    VLOG(3) << "dCPT_dkin: \n" << dCPT_dkin;
    VLOG(3) << "dCPT_dtd (transpose): \n" << dCPT_dtd.transpose();
    
    // Move in space from machine pose to camera. Only if camera pose is not zero.
    //if (!camera_->Pose().IsZero()) {
    ///}
    
    // How many tags were seen? 
    const AprilTagMessage& apriltag_msg = message.april_msg();
    if (apriltag_msg.tag_id_size() == 0) {LOG(ERROR)<<"num_tags seen == 0"; return false;}
    
    // Resize residual and jacobian stores to maximum size seen in image
    int32_t max_rows = apriltag_msg.tag_id_size()*2*4;  // 4 corners per tag, two indices per crnr
    residual_.resize(max_rows, 1); residual_.setZero();
    dcorner_dkin_.resize(max_rows, 18); dcorner_dkin_.setZero();
    dcorner_dintrinsics_.resize(max_rows, 9); dcorner_dintrinsics_.setZero();
    dcorner_dextrinsics_.resize(max_rows, 6); dcorner_dextrinsics_.setZero();
    dcorner_dtimedelay_.resize(max_rows, 1); dcorner_dtimedelay_.setZero();
    
    int32_t num_tags = 0;
    int32_t num_corners = 0;
    
    double avg_corner_error = 0.;
    
    // Go through each tag to build up the residuals
    for (int i_tag=0; i_tag<apriltag_msg.tag_id_size(); i_tag++) {
      
      const std::string& tag_id = apriltag_msg.tag_id(i_tag);
      
      // Does this tag belong to the calibration target?
      if (!target_->TagIsPresent(tag_id)) {
        LOG(WARNING) << "Saw a tag not on target " << tag_id << ". Skipping it.";
        continue;
      }
      
      // Extract the tag corners seen in image
      Matrix2x4d image_coords, projected_coords;
      ExtractAprilTag2dCorners(apriltag_msg, i_tag, &image_coords);
      
      // Corners in target reference frame
      const Matrix3x4d& tag_corners = target_->TagCorners(tag_id);
      
      // Create a residual for each corner
      for (int i_crnr = 0; i_crnr<4; i_crnr++) {
        
        // Position of this corner wrt target
        const Vector3d TpF(tag_corners.col(i_crnr));
        
        // Position of this corner in camera frame
        Vector3d    CpF; CpF.setZero();
        Matrix3x6d  dCpF_dCPT; dCpF_dCPT.setZero();
        Matrix3d    dCpF_dTpF; dCpF_dTpF.setZero();
        CPT.AddPoint(TpF, &CpF, &dCpF_dCPT, &dCpF_dTpF);
        
        // Projection of this point on camera image
        Vector2d uv; uv.setZero();
        Matrix2x3d duv_dCpF; duv_dCpF.setZero();
        Matrix2x9d duv_dintrinsics; duv_dintrinsics.setZero(); 
        camera_->Intrinsics().ProjectPoint(CpF, &uv, &duv_dCpF, &duv_dintrinsics);
        
        // Final Jacobians of projection wrt kinematic state, intrinsics, extrinsics, time delay
        Matrix2x6d  duv_dCPT = duv_dCpF * dCpF_dCPT;
        Matrix2x3d  duv_dTpF = duv_dCpF * dCpF_dTpF;
        Matrix2x18d duv_dkin = duv_dCPT * dCPT_dkin;
        Vector2d    duv_dtd  = duv_dCPT * dCPT_dtd;
        
        // Calculate starting residual
        Vector2d resid = uv - image_coords.col(i_crnr);
        projected_coords.col(i_crnr) = uv;
        avg_corner_error += resid.squaredNorm();
        
        // Calculate covariance of the residual
        Matrix2d var_resid =
            options_->image_corner_covariance
            + duv_dTpF * target_->CornerPositionCovariance() * duv_dTpF.transpose();
        
        // Add variance due to intrinsics uncertainty if we are not solving for them
        if (!options_->solve_for_intrinsics) {
          var_resid +=
            duv_dintrinsics * camera_->Intrinsics().covariance_ * duv_dintrinsics.transpose();
        }
        
        // Add variance due extrinsics uncertainty if we are not solving for them
        //if (!options_->solve_for_extrinsics) {
        //  var_resid +=
        //    duv_dextrinsics * camera_->Extrinsics().covariance_ * duv_dextrinsics.transpose();
        ///}
        
        // Add variance due to timedelay uncertainty if we are not solving for it
        if (!options_->solve_for_timedelay) {
          var_resid +=
            duv_dtd * camera_->TimeDelay().covariance_ * duv_dtd.transpose();
        }
        
        
        // Calculate inverse sqrt covariance
        Matrix2d inv_sqrt_cov; Vector2d sqrt_var;
        if (!InverseSqrt(var_resid, &inv_sqrt_cov, &sqrt_var)) {
          LOG(ERROR) << "Could not calculate inv sqrt mat";
          continue;
        }
        
        // Report calculations
        if (i_tag<5 && i_crnr==0) {
          VLOG(1) << "proj, seen, resid = " << " " << uv.transpose() << ",  "
              << image_coords.col(i_crnr).transpose() << ",  " << resid.transpose()
              << ", sqrt cov: " << sqrt_var.transpose() << ", rho: " << var_resid(0,1)/sqrt_var(0)/sqrt_var(1);
          //VLOG(1) << "inv_sqrt_cov: " << inv_sqrt_cov.row(0) << " " << inv_sqrt_cov.row(1);
        }
        
        // Normalize the residuals and Jacobians to create independent gaussians
        resid = inv_sqrt_cov * resid;
        duv_dkin = inv_sqrt_cov * duv_dkin;
        duv_dintrinsics = inv_sqrt_cov * duv_dintrinsics;
        //duv_dextrinsics = inv_sqrt_cov * duv_dextrinsics;
        duv_dtd = inv_sqrt_cov * duv_dtd;
        
        // Check if there are any NANs in the calculations
        if (!resid.allFinite() || !duv_dkin.allFinite() || !duv_dintrinsics.allFinite() ||
            !duv_dtd.allFinite()) {
          LOG(ERROR) << "Found non finite values in residual or jacobians";
          LOG(ERROR) << "resid = " << resid.transpose();
          LOG(ERROR) << "duv_dkin = \n" << duv_dkin;
          LOG(ERROR) << "duv_dintrinsics = \n" << duv_dintrinsics;
          LOG(ERROR) << "duv_dtd = \n" << duv_dtd;
          continue;
        }
        
        // Append the corner residual to the cost function residual
        residual_.block<2,1>(2*num_corners,0) = resid;
        dcorner_dkin_.block<2,18>(2*num_corners,0) = duv_dkin;
        dcorner_dintrinsics_.block<2,9>(2*num_corners,0) = duv_dintrinsics;
        //dcorner_dextrinsics_.block<2,6>(2*num_corners,0) = duv_dextrinsics;
        dcorner_dtimedelay_.block<2,1>(2*num_corners,0) = duv_dtd;
        
        num_corners++;
        
      } // for all four corners
      
      // Plot the tag on the image
      if (plot_image) {
        PlotTagOnImage(image_coords, CV_WHITE, plot_image);
        PlotTagOnImage(projected_coords, CV_RED, plot_image);
      }
      
      // All was good, so increment the number of tags seen
      num_tags++;
    }
    avg_corner_error /= num_corners; avg_corner_error = std::sqrt(avg_corner_error);
    VLOG(1) << "Used " << num_tags << " of " << apriltag_msg.tag_id_size() << " tags seen. "
        << " num corners: " << num_corners << " (should be " << num_tags*4 << ") "
        << " avg corner error: " << avg_corner_error;
    
    num_residuals_ = 2*num_corners;
    
    // Resize the residual and jacobians keeping calculated values if needed
    if (num_residuals_ != max_rows) {
      residual_.conservativeResize(num_residuals_, Eigen::NoChange);
      dcorner_dkin_.conservativeResize(num_residuals_, Eigen::NoChange);
      dcorner_dintrinsics_.conservativeResize(num_residuals_, Eigen::NoChange);
      dcorner_dextrinsics_.conservativeResize(num_residuals_, Eigen::NoChange);
      dcorner_dtimedelay_.conservativeResize(num_residuals_, Eigen::NoChange);
      LOG(INFO) << "Resized residuals/jacobians from " << max_rows << " to " << num_residuals_;
    }
    
    // Set number of residuals for solver
    set_num_residuals(num_residuals_);
    VLOG(1) << "Set number of residuals for solver: " << num_residuals_;
    
    // Set parameter block sizes for solver - kin state, camera intrinsics, extrinsics, timedelay
    mutable_parameter_block_sizes()->push_back(18); // Target kinematic state has 18 parameters 
    mutable_parameter_block_sizes()->push_back(9);  // Camera intrinsics has 9 parameters 
    mutable_parameter_block_sizes()->push_back(6);  // Camera extrinsics has 6 parameters 
    mutable_parameter_block_sizes()->push_back(1);  // Camera timedelay has 1 parameters 
    
    return true;
  }
  
  // Add this residual to a problem
  bool AddToProblem2(ceres::Problem *problem, ceres::LossFunction* loss_func = nullptr) {
    // Add the residual 
    ceres::CostFunction* cf = this;
    problem->AddResidualBlock(
      cf, loss_func,
      kin_->error_.data(),                  // 18d
      camera_->intrinsics_.error_.data(),   // 9d
      camera_->extrinsics_.error_.data(),   // 6d
      camera_->time_delay_.error_.data()    // 1d
    );
    // Set states constant if not solving for them
    if (!options_->solve_for_intrinsics) {
      problem->SetParameterBlockConstant(camera_->intrinsics_.error_.data());
    }
    if (!options_->solve_for_extrinsics) {
      problem->SetParameterBlockConstant(camera_->extrinsics_.error_.data());
    }
    if (!options_->solve_for_timedelay) {
      problem->SetParameterBlockConstant(camera_->time_delay_.error_.data());
    }
    return true;
  }
  
  // Set the kinematic state of this residual to constant in problem
  bool SetKinematicStateConstant(ceres::Problem *problem) {
    problem->SetParameterBlockConstant(kin_->error_.data());
    return true;
  }
  
  // Evaluate function for ceres solver
  virtual bool Evaluate2(
      double const* const* parameters,
      double* residuals,
      double** jacobians) const {
    
    // Mapping residuals and parameters to matrices
    MapConstVector18d   dkin(parameters[0]);
    MapConstVector9d    dintrinsics(parameters[1]);
    MapConstVector6d    dextrinsics(parameters[2]);
    MapConstVector1d    dtimedelay(parameters[3]);
    MapVectorXd         residual(residuals, num_residuals_);
    
    // Calculate residual from the error states using linearized model
    residual = residual_
               + dcorner_dkin_ * dkin
               + dcorner_dintrinsics_ * dintrinsics
               + dcorner_dextrinsics_ * dextrinsics
               + dcorner_dtimedelay_ * dtimedelay;
    
    // First estimates Jacobians are used as this shown to keep the estimator consistent
    if (jacobians != NULL) {
      if (jacobians[0] != NULL) {
        Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>>
            jac(jacobians[0], num_residuals_, 18);
        jac = dcorner_dkin_;
      }
      if (jacobians[1] != NULL) {
        Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>>
            jac(jacobians[1], num_residuals_, 9);
        jac = dcorner_dintrinsics_;
      }
      if (jacobians[2] != NULL) {
        Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>>
            jac(jacobians[2], num_residuals_, 6);
        jac = dcorner_dextrinsics_;
      }
      if (jacobians[3] != NULL) {
        Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>>
            jac(jacobians[3], num_residuals_, 1);
        jac = dcorner_dtimedelay_;
      }
    }
    
    return true;
  }*/
  
  // Destructor
  virtual ~KinematicCalibrationTargetViewResidual() {}  
  
};  // KinematicCalibrationTargetViewResidual


// Calibration Target Camera Calibrator
//  Implements camera calibration routines
class CalibrationTargetCameraCalibrator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  struct Options {
    double pose_position_sigma;
    double pose_rotation_sigma;
    Options():
      pose_position_sigma(0.000), pose_rotation_sigma(0.*kRadiansPerDegree)
    {}
  };
  
  CalibrationTargetCameraCalibrator::Options options_;
  
  Matrix6d  starting_target_pose_cov;
  
  CalibrationTargetCameraCalibrator():
      options_()
  {
    starting_target_pose_cov.setZero();
    starting_target_pose_cov.block<3,3>(0,0) =
        options_.pose_rotation_sigma * options_.pose_rotation_sigma * kIdentity3d;
    starting_target_pose_cov.block<3,3>(3,3) =
        options_.pose_position_sigma * options_.pose_position_sigma * kIdentity3d;
  }
  
  // Use the observations of a calibration target to initiate camera using OpenCV methods
  bool InitiateCameraCalibration(
      const CameraCalibrationTarget& target,
      const std::vector<anantak::SensorMsg>& msgs,
      const std::vector<int32_t>& image_size,
      CameraState* camera,
      const std::string& savefile = "data/camera_calibration_target_starting_poses.pb.data")
  {  
    if (image_size.size()<2) {LOG(ERROR)<<"image_size size < 2"; return false;}
    
    VLOG(1) << "Starting calibration";
    
    // Create vector of vector of correspondeneces for each message stored in the grid
    std::vector<std::vector<cv::Point3f>> all_target_points;
    std::vector<std::vector<cv::Point2f>> all_image_points;
    
    for (int i_image=0; i_image < msgs.size(); i_image++) {
      const anantak::SensorMsg& msg = msgs.at(i_image);
      if (!msg.has_april_msg()) {
        LOG(ERROR) << "Msg does not have an AprilTag message. Skipping.";
        continue;
      }
      
      const anantak::AprilTagMessage& apriltag_msg = msg.april_msg();
      std::vector<cv::Point3f> target_points;
      std::vector<cv::Point2f> image_points;
      // Go through each tag sighting in message
      for (int i_tag=0; i_tag<apriltag_msg.tag_id_size(); i_tag++) {
        const std::string& id = apriltag_msg.tag_id(i_tag);
        // Make sure that tag is present in the target
        if (!target.TagIsPresent(id)) continue; 
        // Extract image points
        cv::Point2f ip0(apriltag_msg.u_1(i_tag), apriltag_msg.v_1(i_tag));
        cv::Point2f ip1(apriltag_msg.u_2(i_tag), apriltag_msg.v_2(i_tag));
        cv::Point2f ip2(apriltag_msg.u_3(i_tag), apriltag_msg.v_3(i_tag));
        cv::Point2f ip3(apriltag_msg.u_4(i_tag), apriltag_msg.v_4(i_tag));
        // Extract target points
        const Matrix3x4d& tag_corners = target.TagCorners(id);
        cv::Point3f tp0(tag_corners(0,0), tag_corners(1,0), tag_corners(2,0));
        cv::Point3f tp1(tag_corners(0,1), tag_corners(1,1), tag_corners(2,1));
        cv::Point3f tp2(tag_corners(0,2), tag_corners(1,2), tag_corners(2,2));
        cv::Point3f tp3(tag_corners(0,3), tag_corners(1,3), tag_corners(2,3));
        // Store all points
        image_points.push_back(ip0);
        image_points.push_back(ip1);
        image_points.push_back(ip2);
        image_points.push_back(ip3);
        target_points.push_back(tp0);
        target_points.push_back(tp1);
        target_points.push_back(tp2);
        target_points.push_back(tp3);
      }
      // Store
      all_target_points.push_back(target_points);
      all_image_points.push_back(image_points);
      VLOG(2) << "Stored " << target_points.size() << "/" << image_points.size() << " correspondences "
          << " for n tags: " << apriltag_msg.tag_id_size();
    }
    VLOG(1) << "Added " << all_target_points.size() << "/" << all_image_points.size() << " images";
    
    // Run calibration using OpenCV functions
    VLOG(1) << "Running opencv calibration...";
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    cv::Size image_sz(image_size[0], image_size[1]);
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;
    cv::calibrateCamera(all_target_points, all_image_points, image_sz,
                        camera_matrix, dist_coeffs, rvecs, tvecs);    
    // Report results
    LOG(INFO) << "OpenCV camera calibration results: \nCamera matrix = \n" << camera_matrix
        << "\nDistortion coeffs = \n" << dist_coeffs;
    
    // Store results in camera and pose states
    camera->intrinsics_.SetIntrinsics(camera_matrix, dist_coeffs);
    camera->SetImageSize(image_size[0], image_size[1]);
    std::vector<PoseState> target_poses;
    for (int i=0; i<rvecs.size(); i++) {
      target_poses.emplace_back();
      PoseState& _pose = target_poses.back();
      _pose.SetPose(rvecs[i], tvecs[i]);
      // Add variance to poses
      _pose.SetCovariance(starting_target_pose_cov);
      VLOG(1) << "Target pose " << i << ": " << _pose.ToString();
    }
    
    // Save the starting poses to a file, usually to plot
    anantak::MessageFileWriter file_writer;
    file_writer.Open(savefile);
    for (int i=0; i<target_poses.size(); i++) {
      anantak::SensorMsg _msg;
      if (target_poses[i].CopyToMessage(&_msg)) file_writer.WriteMessage(_msg);
    }
    file_writer.Close();
    LOG(INFO) << "Saved starting target poses to " << savefile;
    
    // Run optimization using anantak pose maths - mainly as a check and to calculate covariances
    return RunStartingCalibration(target, msgs, camera, &target_poses);
  }
  
  // Use observations to calibrate camera
  bool RunStartingCalibration(
      const CameraCalibrationTarget& target,
      const std::vector<anantak::SensorMsg>& msgs,
      CameraState* camera,
      std::vector<PoseState>* target_poses,
      const std::string& savefile = "data/camera_calibration_target_final_poses.pb.data")
  {
    VLOG(1) << "Running calibration \n  n_msgs: " << msgs.size() << ", n_poses: " << target_poses->size()
        << "\n  starting camera intrinsics: " << camera->intrinsics_.ToString();
    
    // Setup problem
    ceres::Problem::Options problem_options;
    problem_options.cost_function_ownership = ceres::TAKE_OWNERSHIP;
    problem_options.loss_function_ownership = ceres::TAKE_OWNERSHIP;
    ceres::Solver::Options solver_options;
    solver_options.max_num_iterations = 100;
    solver_options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary solver_summary;    
    ceres::Problem problem(problem_options);
    
    // Build residuals
    for (int i=0; i<target_poses->size(); i++) {
      PoseState& pose = target_poses->at(i);
      
      CalibrationTargetViewResidual* target_view_residual = new CalibrationTargetViewResidual();
      if (!target_view_residual->Create(&target, &msgs.at(i), &pose, camera)) {
        LOG(ERROR) << "Could not create residual from message " << i << "Skipping.";
        delete target_view_residual;
        target_view_residual = nullptr;
        continue;
      }
      
      // Add the residual to problem and transfer ownsership
      target_view_residual->AddToProblem(&problem);
      
      //if (false) {
      //problem.SetParameterBlockConstant(pose.error_.data());
      ///}
    }
    
    // Solve problem
    ceres::Solve(solver_options, &problem, &solver_summary);
    VLOG(1) << solver_summary.FullReport();
    
    // Extract covariances
    camera->intrinsics_.SetCovariance(&problem);
    for (int i=0; i<target_poses->size(); i++) {
      target_poses->at(i).SetCovariance(&problem);
    }
    
    // Recalculate 
    camera->intrinsics_.Recalculate();
    VLOG(1) << "Camera: " << camera->intrinsics_.ToString(2);
    for (int i=0; i<target_poses->size(); i++) {
      target_poses->at(i).Recalculate();
      VLOG(3) << "Target pose " << i << ": " << target_poses->at(i).ToString(2);
    }
    
    // Save the final poses to a file, usually to plot
    anantak::MessageFileWriter file_writer;
    file_writer.Open(savefile);
    for (int i=0; i<target_poses->size(); i++) {
      anantak::SensorMsg _msg;
      if (target_poses->at(i).CopyToMessage(&_msg)) file_writer.WriteMessage(_msg);
    }
    file_writer.Close();
    LOG(INFO) << "Saved starting target poses to " << savefile;
    
    return true;
  }
  
};  // CalibrationTargetCameraCalibrator 


// Spline that joins points with straight lines
class PiecewisePoseSpline {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  std::vector<int64_t>   timestamps_;
  std::vector<PoseState> poses_;
  
  PiecewisePoseSpline() {}
  
  // Add a new pose
  bool AddPose(const PoseState& pose) {
    
    if (pose.Timestamp()==0) {LOG(ERROR)<<"pose.Timestamp()==0"; return false;}
    
    if (poses_.size()>0) {
      if (pose.Timestamp()<=poses_.back().Timestamp()) {
        LOG(ERROR) << "Pose timestamps is less than last pose timestamp "
            << pose.Timestamp() << " " << poses_.back().Timestamp();
        return false;
      }
    }
    
    poses_.emplace_back(pose);
    
    return true;
  }
  
  // Interpolate or extrapolate using the poses
  bool Interpolate(int64_t timestamp, KinematicState* kin, const KinematicState* ref_kin = nullptr)
      const {
    
    if (poses_.size()<1) {LOG(ERROR)<<"There are no poses, can not interpolate"; return false;}
    if (!kin) {LOG(ERROR)<<"kin is null"; return false;}
    
    if (poses_.size()==1) {
      kin->SetTimestamp(timestamp);
      kin->SetPose(poses_.at(0));
      if (ref_kin) {
        double _t = double(timestamp - ref_kin->Timestamp())*1e-6 + ref_kin->T()(0);
        kin->SetTime(_t);
        kin->covariance_ = ref_kin->covariance_;
        kin->SetVelocity(ref_kin->W(), ref_kin->V());
        kin->SetAcceleration(ref_kin->dW(), ref_kin->dV());
      }
      return true;
    }
    
    VLOG(2) << "Interpolating using splines for timestamp: " << timestamp;
    
    int32_t idx0 = 0;
    int32_t idx1 = 0;
    if (timestamp < poses_.front().Timestamp()) {
      idx0 = 0; idx1 = 1;
    }
    else if (timestamp > poses_.back().Timestamp()) {
      idx0 = poses_.size()-2; idx1 = poses_.size()-1;
    }
    else {
      for (int i=0; i<poses_.size()-1; i++) {
        if (poses_.at(i).Timestamp()<=timestamp && timestamp<poses_.at(i+1).Timestamp()) {
          idx0 = i; idx1 = i+1;
          break;
        }
      }
    }
    
    if (idx0==0 && idx1==0) {LOG(FATAL) << "idx0==0 && idx1==0"; return false;}
    
    // Interpolate
    const PoseState& p0 = poses_.at(idx0);
    const PoseState& p1 = poses_.at(idx1);
    double f = double(timestamp - p0.Timestamp()) / double(p1.Timestamp() - p0.Timestamp());
    Quaterniond _dq = p1.Q() * p0.Q().conjugate();
    Vector3d _w = QuaternionToErrorAngleAxis(_dq);  // Small angle approximation
    Vector3d _da = _w * f;
    AngleAxisd aa(_da.norm(), _da.normalized());
    Quaterniond dq(aa);
    Quaterniond q = dq * p0.Q();
    Vector3d _v = p1.P() - p0.P();
    Vector3d dp = _v * f;
    Vector3d p = p0.P() + dp;
    double dt = double(p1.Timestamp() - p0.Timestamp())*1e-6;
    _w /= dt; _v /= dt;                     // in world frame
    //_w = p0.R() * _w;
    _v = p0.R() * _v;     // in body frame of p0 - these are assumed to be constant
    Vector3d zero; zero.setZero();
    kin->SetTimestamp(timestamp);
    kin->SetPose(q,p);
    kin->SetVelocity(_w,_v);
    kin->SetAcceleration(zero,zero);
    //kin->SetTime(0.); // Kin time is modified only if a reference state is provided
    // Covariance of kin is not modified
    
    if (ref_kin) {
      double _t = double(timestamp - ref_kin->Timestamp())*1e-6 + ref_kin->T()(0);
      kin->SetTime(_t);
      kin->covariance_ = ref_kin->covariance_;
    }
    
    return true;
  }
  
  virtual ~PiecewisePoseSpline() {}
  
};  //PiecewisePoseSpline


// Linear spline connecting poses for non-uniform control points
class NonUniformLinearPoseSpline {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef anantak::TimedCircularQueue<anantak::PoseState> PoseStateQueue;
  
  std::unique_ptr<PoseStateQueue>
    control_poses_;       // Control poses timed circular queue
  PoseStateQueue::FixedPoint
    segment_marker_;      // Fixed point on control poses marking beginning of segment
  
  int64_t starting_ts_;   // Starting timestamp of the spline (microseconds)
  int64_t ending_ts_;     // Starting timestamp of the spline (microseconds)
  
  PoseState* p0_;     // P[i] pose state
  PoseState* p1_;     // P[i+1] pose state 
  
  NonUniformLinearPoseSpline(int32_t size) {
    // Allocate memory for the queue
    std::unique_ptr<anantak::TimedCircularQueue<anantak::PoseState>> poses_ptr(
        new anantak::TimedCircularQueue<anantak::PoseState>(size));
    control_poses_ = std::move(poses_ptr);
    LOG(INFO) << "Allocated memory for TimedCircularQueue of NonUniformLinearPoseSpline " << size;
    
    // Add a fixed point
    segment_marker_ = control_poses_->CurrentFixedPoint();  // Marks the control pose before current segment
    
    starting_ts_ = 0;
    ending_ts_ = 0;
    
    p0_ = p1_ = nullptr;
  }
  
  bool Reset() {
    for (int i=0; i<control_poses_->n_msgs(); i++) {
      PoseState* pose = control_poses_->AtPtr(i);
      pose->SetZero();
    }
    control_poses_->Clear();
    segment_marker_.Reset();
    starting_ts_ = 0;
    ending_ts_ = 0;
    p0_ = p1_ = nullptr;
    return true;
  }
  
  inline int64_t StartingTimestamp() const {return starting_ts_;}
  inline int64_t EndingTimestamp() const {return ending_ts_;}
  inline int32_t NumControlPoints() const {return control_poses_->n_msgs();}
  
  // Add a new pose
  bool AddControlPose(const PoseState& pose) {
    
    if (pose.Timestamp()==0) {LOG(ERROR)<<"pose.Timestamp()==0"; return false;}
    
    if (control_poses_->n_msgs() > 0) {
      if (pose.Timestamp() <= control_poses_->LastTimestamp()) {
        LOG(ERROR) << "Pose ts is not greater than spline's last ts " << pose.Timestamp() << " "
            << control_poses_->LastTimestamp();
        return false;
      }
    }
    if (control_poses_->n_msgs() == 0) starting_ts_ = pose.Timestamp();
    control_poses_->AddElement(pose); // copy pose
    control_poses_->SetTimestamp(pose.Timestamp()); // Set queue timestamp
    VLOG(2) << "Added a control pose with timestamp " << pose.Timestamp();
    ending_ts_ = control_poses_->LastTimestamp();
    
    return true;
  }
  
  inline bool CheckPosePointers() {
    return (p1_ && p0_);
  }
  
  // Find the control interval for a given time
  inline bool PositionPosePointers(const int64_t ts) {
    if (ts < starting_ts_) {
      p0_ = control_poses_->AtPtr(0);
      p1_ = control_poses_->AtPtr(1);
      return CheckPosePointers();
    }
    if (ts > control_poses_->LastTimestamp()) {
      segment_marker_ = control_poses_->CurrentFixedPoint();
      p0_ = control_poses_->NthLastElementPtr(2);
      p1_ = control_poses_->NthLastElementPtr(1);
      return CheckPosePointers();
    }
    if (!control_poses_->PositionFixedPointBeforeTimestamp(segment_marker_, ts, true, false)) {
      LOG(ERROR) << "Could not position in control poses for the segment";
      return false;
    }
    //VLOG(2) << "Marker set at: " << segment_marker_.ToString() << " "
    //    << control_poses_->Timestamp(segment_marker_);
    p0_ = control_poses_->NthLastElementPtrBefore(segment_marker_, 1);  // Segment begin element
    p1_ = control_poses_->NthLastElementPtrBefore(segment_marker_, 0);  // Segment end element
    VLOG(2) << "Set p0 p1 ts: " << p0_->Timestamp() << " " << p1_->Timestamp();
    return CheckPosePointers();
  }
  
  // Interpolate or extrapolate using the poses
  bool InterpolatePose(int64_t timestamp, PoseState* pose, bool include_errors = false) {
    
    if (!pose) {LOG(ERROR)<<"pose is null"; return false;}
    
    if (control_poses_->n_msgs()<1) {
      LOG(ERROR)<<"There are no poses, can not interpolate";
      return false;
    }
    
    if (control_poses_->n_msgs()==1) {
      pose->SetTimestamp(timestamp);
      *pose = control_poses_->At(0);
      return true;
    }
    
    VLOG(2) << "Interpolating using linear spline for timestamp: " << timestamp;
    
    // Locate interpolation segment
    if (!PositionPosePointers(timestamp)) {
      LOG(ERROR) << "Could not position pointers for ts " << timestamp;
      return false;
    }
    
    // Interpolate pose linearly
    double f = double(timestamp - p0_->Timestamp()) / double(p1_->Timestamp() - p0_->Timestamp());
    Quaterniond _dq = p1_->Q() * p0_->Q().conjugate();
    Vector3d _w = QuaternionToErrorAngleAxis(_dq);
    Vector3d _da = _w * f;
    //AngleAxisd aa(_da.norm(), _da.normalized());
    //Quaterniond dq(aa);
    Quaterniond dq = ErrorAngleAxisToQuaternion(_da);
    Quaterniond q = dq * p0_->Q();
    Vector3d _v = p1_->P() - p0_->P();
    Vector3d dp = _v * f;
    Vector3d p = p0_->P() + dp;
    pose->SetTimestamp(timestamp);
    pose->SetPose(q,p);
    
    // Covariance of pose is not modified
    
    return true;
  }
  
  
};  // NonUniformLinearPoseSpline


// Options for all cubic pose spline classes
class PoseSpline_Options {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  Matrix6d pose_covariance_;  // represents the fit of cubic pose spline to object's trajectory
  
  double pose_prior_multiplier_;      // prior on a control pose sigma = multiplier * pose_cov_
  Matrix6d pose_prior_covariance_;    // inv sqrt matrix of prior covariance
  Matrix6d inv_sqrt_pose_prior_covariance_;   // inv sqrt matrix of prior covariance
  
  PoseSpline_Options(const double sigma_eq, const double sigma_ep,
                     const double sigma_prior_mult) {
    
    if (!CalculatePoseCovariance(sigma_eq, sigma_ep)) {
      LOG(FATAL) << "Could not calculate covariance of pose";
    }
    
    if (!CalculatePosePriorCovariance(sigma_prior_mult)) {
      LOG(FATAL) << "Could not calculate covariance of pose prior";
    }
  }
  
  bool CalculatePoseCovariance(const double sigma_eq, const double sigma_ep) {
    double var_eq = sigma_eq*sigma_eq;
    double var_ep = sigma_ep*sigma_ep;
    pose_covariance_.setZero();
    pose_covariance_.diagonal() << var_eq, var_eq, var_eq, var_ep, var_ep, var_ep;
    VLOG(1) << "PoseSpline_PoseResidual::Options cov: \n" << pose_covariance_;
    return true;
  }
  
  bool CalculatePosePriorCovariance(const double sigma_prior_mult) {
    pose_prior_multiplier_ = sigma_prior_mult;
    pose_prior_covariance_ = pose_prior_multiplier_ * pose_prior_multiplier_ * pose_covariance_;
    Vector6d sqrt_var;
    if (!InverseSqrt(pose_prior_covariance_, &inv_sqrt_pose_prior_covariance_, &sqrt_var)) {
      LOG(ERROR) << "Could not calculate inv sqrt covariance";
      return false;
    }
    if (!inv_sqrt_pose_prior_covariance_.allFinite()) {
      LOG(ERROR) << "Inverse sqrt prior covariance is not finite";
      return false;
    }
    return true;
  }
  
};  // PoseSpline_Options


// Creates a prior on a control pose
class PoseSpline_ControlPosePrior : public ceres::SizedCostFunction<6, 6> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  // Options
  const PoseSpline_Options* options_;
  
  // Pointer to state
  PoseState* P0_;
  
  // Residual and Jacobian
  Vector6d residual_;
  Matrix6d dP0_dP0_;
  
  PoseSpline_ControlPosePrior(const PoseSpline_Options* options) {
    SetOptions(options);
    Reset();
  }
  
  bool SetOptions(const PoseSpline_Options* options) {
    if (!options) {LOG(FATAL) << "Options are NULL";}
    options_ = options;
  }
  
  bool Reset() {    // options are not modified
    P0_ = nullptr;
    residual_.setZero();
    dP0_dP0_.setZero();
  }
  
  PoseSpline_ControlPosePrior(const PoseSpline_ControlPosePrior& r):
    options_(r.options_), P0_(r.P0_),
    residual_(r.residual_), dP0_dP0_(r.dP0_dP0_)
  {}
  
  PoseSpline_ControlPosePrior& operator= (const PoseSpline_ControlPosePrior& r) {
    options_ = r.options_; P0_ = r.P0_;
    residual_ = r.residual_; dP0_dP0_ = r.dP0_dP0_;
  }
  
  bool Create(PoseState* P0) {
    if (!P0) {LOG(ERROR) << "P0 is NULL!"; return false;}
    P0_ = P0;
    
    // Set residual, scaled already as it is zero
    residual_.setZero();
    
    // Set Jacobian, scaled already as it is identity
    dP0_dP0_ = options_->inv_sqrt_pose_prior_covariance_;
    
    // Include existing control pose error
    residual_ += -dP0_dP0_*P0_->error_;
    
    // Report
    VLOG(2) << "Pose prior residual = " << residual_.transpose();
    VLOG(2) << "inv_sqrt_pose_prior_covariance_ = " << dP0_dP0_.diagonal().transpose();
    
    return true;
  }
  
  // Add this residual to a problem
  bool AddToProblem(ceres::Problem *problem, ceres::LossFunction* loss_func = nullptr) {
    // Check integrity of data
    if (!P0_) {LOG(ERROR) << "!P0_"; return false;}
    
    VLOG(2) << "ControlPosePrior: Problem has " << problem->NumResidualBlocks() << " residual blocks.";
    
    // Add this residual to problem
    ceres::CostFunction* cf = this;
    problem->AddResidualBlock(
      cf, loss_func,
      P0_->error_.data()            // 6d
    );
    return true;
  }
  
  // Mark K0 as constant on the problem
  bool MarkP0Constant(ceres::Problem* problem) {
    if (problem->HasParameterBlock(P0_->error_.data()))
      problem->SetParameterBlockConstant(P0_->error_.data());    
    return true;
  }
  
  // Evaluate function for ceres solver
  virtual bool Evaluate(
      double const* const* parameters,
      double* residuals,
      double** jacobians) const {
    
    MapVector6d          residual(residuals);
    MapConstVector6d     dP0(parameters[0]);
    
    // Calculate residual from the error states using linearized model
    residual = residual_ + dP0_dP0_*dP0;
    
    // First estimates Jacobians are used as this shown to keep the estimator consistent
    if (jacobians != NULL) {
      if (jacobians[0] != NULL) {
        Eigen::Map<Eigen::Matrix<double,6,6,Eigen::RowMajor>> jac(jacobians[0]);
        jac = dP0_dP0_.block<6,6>(0,0);
      }
    }
    
    return true;
  }
  
};  // PoseSpline_ControlPosePrior


// Creates a constraint for a pose observation on the spline
class PoseSpline_PoseResidual : public ceres::SizedCostFunction<6, 6,6,6,6> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  // Options
  const PoseSpline_Options* options_;
  
  // Observation
  PoseState observation_;   // Should we copy the observation?
                            // This may be useful in reporting. But increases CPU load. **CHECK**
  
  // Pointers to spline control poses
  PoseState* p3_;     // P[i-3] pose state in control poses queue
  PoseState* p2_;     // P[i-2] pose state 
  PoseState* p1_;     // P[i-1] pose state 
  PoseState* p0_;     // P[i] pose state 
  
  // Residual
  Vector6d residual_;   // Residual between observation and spline calculated pose
  
  // Jacobians
  Matrix6d dObs_dp3_;   // Jacobian of error of observation by spline pose[i-3]
  Matrix6d dObs_dp2_;   // Jacobian of error of observation by spline pose[i-2]
  Matrix6d dObs_dp1_;   // Jacobian of error of observation by spline pose[i-1]
  Matrix6d dObs_dp0_;   // Jacobian of error of observation by spline pose[i]
  
  PoseSpline_PoseResidual(const PoseSpline_Options* options) {
    SetOptions(options);
    Reset();
  }
  
  bool SetOptions(const PoseSpline_Options* options) {
    if (!options) {LOG(FATAL) << "options pointer is null";}
    options_ = options;
  }
  
  bool Reset() {    // options_ is not reset
    observation_.SetZero();
    p3_ = nullptr; p2_ = nullptr; p1_ = nullptr; p0_ = nullptr;
    residual_.setZero();
    dObs_dp3_.setZero(); dObs_dp2_.setZero(); dObs_dp1_.setZero(); dObs_dp0_.setZero();
  }
  
  PoseSpline_PoseResidual(const PoseSpline_PoseResidual& r):
    options_(r.options_), observation_(r.observation_), 
    p3_(r.p3_), p2_(r.p2_), p1_(r.p1_), p0_(r.p0_),
    residual_(r.residual_),
    dObs_dp3_(r.dObs_dp3_), dObs_dp2_(r.dObs_dp2_), dObs_dp1_(r.dObs_dp1_), dObs_dp0_(r.dObs_dp0_)
  {}
  
  PoseSpline_PoseResidual& operator= (const PoseSpline_PoseResidual& r) {
    options_=r.options_; observation_=r.observation_; 
    p3_=r.p3_; p2_=r.p2_; p1_=r.p1_; p0_=r.p0_;
    residual_=r.residual_;
    dObs_dp3_=r.dObs_dp3_; dObs_dp2_=r.dObs_dp2_; dObs_dp1_=r.dObs_dp1_; dObs_dp0_=r.dObs_dp0_;
  }
  
  const PoseState& Observation() const {return observation_;}
  
  bool Create(const PoseState& observation,
              PoseState* p0, PoseState* p1, PoseState* p2, PoseState* p3,  // Spline control poses
              const Quaterniond* B01_q_A, const Vector3d* A_p_B01,  // predictions from spline
              const Matrix3x12d* dq_dq,   const Matrix3x12d* dp_dp  // jacobians from pose spline
  ) {
    // Check options
    if (!options_) {LOG(ERROR) << "options ptr is NULL"; return false;} 
    
    // Set observation
    if (observation.IsZero()) {LOG(ERROR) << "observation is zero"; return false;}
    observation_ = observation;   // is this copy needed?  **CHECK**
    observation_.UpdateRotations(); // Ensuring that quaternion is normalized
    
    // Set the control poses
    if (!p0 || !p1 || !p2 || !p3) {LOG(ERROR) << "!p0 || !p1 || !p2 || !p3"; return false;}
    p0_ = p0; p1_ = p1; p2_ = p2; p3_ = p3;
    
    // Calculate the residual from prediction. residual = prediction - observation, expressed in A frame
    //  Maths here is from Notebook #6 pg 55
    if (!B01_q_A || !A_p_B01) {LOG(ERROR) << "!B01_q_A || !A_p_B01"; return false;}
    Quaterniond B01_q_Obs = (*B01_q_A) * observation_.Q().conjugate();  // B01_q_A * A_q_Obs
    Matrix3d A_r_B01((*B01_q_A).conjugate());   // This transform to frame A is crucial
    residual_.block<3,1>(0,0) = A_r_B01 * QuaternionToErrorAngleAxis(B01_q_Obs);
    residual_.block<3,1>(3,0) = (*A_p_B01) - observation_.P();
    
    // Set the jacobians
    if (!dq_dq || !dp_dp) {LOG(ERROR) << "!dq_dq || !dp_dp"; return false;}
    dObs_dp0_.setZero();
    dObs_dp1_.setZero();
    dObs_dp2_.setZero();
    dObs_dp3_.setZero();
    
    dObs_dp0_.block<3,3>(0,0) = dq_dq->block<3,3>(0,0);
    dObs_dp1_.block<3,3>(0,0) = dq_dq->block<3,3>(0,3);
    dObs_dp2_.block<3,3>(0,0) = dq_dq->block<3,3>(0,6);
    dObs_dp3_.block<3,3>(0,0) = dq_dq->block<3,3>(0,9);
    
    dObs_dp0_.block<3,3>(3,3) = dp_dp->block<3,3>(0,0);
    dObs_dp1_.block<3,3>(3,3) = dp_dp->block<3,3>(0,3);
    dObs_dp2_.block<3,3>(3,3) = dp_dp->block<3,3>(0,6);
    dObs_dp3_.block<3,3>(3,3) = dp_dp->block<3,3>(0,9);
    
    // Covariance of the residual
    Matrix6d residual_cov = options_->pose_covariance_ + observation_.covariance_;
    
    // Inv sqrt of covariance
    Matrix6d inv_sqrt_cov; Vector6d sqrt_var;
    if (!InverseSqrt(residual_cov, &inv_sqrt_cov, &sqrt_var)) {
      LOG(ERROR) << "Could not calculate inv sqrt covariance";
      return false;
    }
    
    // Report
    VLOG(2) << "Observation residual: \n" << residual_.transpose();
    VLOG(2) << "Observation residual sqrt cov diagonal: \n" << sqrt_var.transpose();
    
    // Scale the residual and jacobians with inverse sqrt covariance
    residual_ = inv_sqrt_cov * residual_;
    dObs_dp0_ = inv_sqrt_cov * dObs_dp0_;
    dObs_dp1_ = inv_sqrt_cov * dObs_dp1_;
    dObs_dp2_ = inv_sqrt_cov * dObs_dp2_;
    dObs_dp3_ = inv_sqrt_cov * dObs_dp3_;
    
    // Check if there are any NANs in the calculations
    if (!residual_.allFinite() ||
        !dObs_dp0_.allFinite() || !dObs_dp1_.allFinite() ||
        !dObs_dp2_.allFinite() || !dObs_dp3_.allFinite()) {
      LOG(ERROR) << "Found non finite values in residual or jacobians";
      LOG(ERROR) << "residual_ = " << residual_.transpose();
      LOG(ERROR) << "Predicted quaternion = " << B01_q_A->coeffs().transpose();
      LOG(ERROR) << "Observation quaternion = " << observation_.Q().coeffs().transpose();
      LOG(ERROR) << "dObs_dp0_ = \n" << dObs_dp0_;
      LOG(ERROR) << "dObs_dp1_ = \n" << dObs_dp1_;
      LOG(ERROR) << "dObs_dp2_ = \n" << dObs_dp2_;
      LOG(ERROR) << "dObs_dp3_ = \n" << dObs_dp3_;
      return false;
    }
    
    // Adjust residual for starting control pose errors
    //residual_ += (-dObs_dp0_*p0_->error_ - dObs_dp1_*p1_->error_
    //              - dObs_dp2_*p2_->error_ - dObs_dp3_*p3_->error_);
    //residual_ += (dObs_dp0_*p0_->error_ + dObs_dp1_*p1_->error_
    //            + dObs_dp2_*p2_->error_ + dObs_dp3_*p3_->error_);
    
    return true;
  }
  
  // Add this residual to a problem
  bool AddToProblem(ceres::Problem *problem, ceres::LossFunction* loss_func = nullptr) {
    // Check integrity of data
    if (!p0_ || !p1_ || !p2_ || !p3_) {LOG(ERROR) << "!p0_ || !p1_ || !p2_ || !p3_"; return false;}
    int num_zero_jacobians = 0;
    if (dObs_dp0_.isZero()) {
      VLOG(2) << "dObs_dp0_.isZero() Obs ts " << observation_.Timestamp() << " p0 ts " << p0_->Timestamp(); num_zero_jacobians++;} 
    if (dObs_dp1_.isZero()) {
      VLOG(2) << "dObs_dp1_.isZero() Obs ts " << observation_.Timestamp() << " p1 ts " << p1_->Timestamp(); num_zero_jacobians++;} 
    if (dObs_dp2_.isZero()) {
      VLOG(2) << "dObs_dp2_.isZero() Obs ts " << observation_.Timestamp() << " p2 ts " << p2_->Timestamp(); num_zero_jacobians++;} 
    if (dObs_dp3_.isZero()) {
      VLOG(2) << "dObs_dp3_.isZero() Obs ts " << observation_.Timestamp() << " p3 ts " << p3_->Timestamp(); num_zero_jacobians++;}
    if (num_zero_jacobians > 1) {
      LOG(ERROR) << "Found more than one zero jacobian for the pose residual. Usually there is only one.";
      LOG(ERROR) << dObs_dp0_ << "\n" << dObs_dp1_ << "\n" << dObs_dp2_ << "\n" << dObs_dp3_;
      // return false; // Should we return false here? **CHECK**
    }
    
    // Add the residual to problem
    ceres::CostFunction* cf = this;
    problem->AddResidualBlock(
      cf, loss_func,
      p0_->error_.data(),           // 6d
      p1_->error_.data(),           // 6d
      p2_->error_.data(),           // 6d
      p3_->error_.data()            // 6d
    );
    
    // Report
    VLOG(2) << "Added pose residual to problem " << observation_.Timestamp() << " control points: "
        << p0_->Timestamp() << " " << p1_->Timestamp() << " "
        << p2_->Timestamp() << " " << p3_->Timestamp();
    
    return true;
  }
  
  // Mark p0 as constant on the problem
  bool MarkStartingControlPoseConstant(ceres::Problem* problem) {
    if (problem->HasParameterBlock(p3_->error_.data()))
      problem->SetParameterBlockConstant(p3_->error_.data());    
    return true;
  }
  
  // Mark all states constant on the problem
  bool MarkAllControlPosesConstant(ceres::Problem* problem) {
    if (problem->HasParameterBlock(p0_->error_.data()))
      problem->SetParameterBlockConstant(p0_->error_.data());    
    if (problem->HasParameterBlock(p1_->error_.data()))
      problem->SetParameterBlockConstant(p1_->error_.data());    
    if (problem->HasParameterBlock(p2_->error_.data()))
      problem->SetParameterBlockConstant(p2_->error_.data());    
    if (problem->HasParameterBlock(p3_->error_.data()))
      problem->SetParameterBlockConstant(p3_->error_.data());    
    return true;
  }
  
  int64_t EarliestControlPoseTimestamp() const {
    if (!p3_) return 0;
    return p3_->Timestamp();
  }
  
  // Removes the control pose errors from residual.
  //  This is done at starting to address existing errors in control poses
  bool RemovePoseErrors() {
    residual_ += (-dObs_dp0_*p0_->error_ - dObs_dp1_*p1_->error_
                  - dObs_dp2_*p2_->error_ - dObs_dp3_*p3_->error_);
    return true;
  }

  // Adds the control pose errors to residual.
  //  This is done before control pose errors are set to zero.
  bool ConsumePoseErrors() {
    residual_ += (dObs_dp0_*p0_->error_ + dObs_dp1_*p1_->error_
                 +dObs_dp2_*p2_->error_ + dObs_dp3_*p3_->error_);
    return true;
  }

  // Evaluate
  virtual bool Evaluate(
      double const* const* parameters,
      double* residuals,
      double** jacobians) const {
    
    // Mapping residuals and parameters to matrices
    MapVector6d         residual(residuals);
    MapConstVector6d    dp0(parameters[0]);   // Change in control pose [i]
    MapConstVector6d    dp1(parameters[1]);   // Change in control pose [i-1]
    MapConstVector6d    dp2(parameters[2]);   // Change in control pose [i-2]
    MapConstVector6d    dp3(parameters[3]);   // Change in control pose [i-3]
    
    // Calculate residual from the error states using linearized model
    residual = residual_ + dObs_dp0_*dp0 + dObs_dp1_*dp1 + dObs_dp2_*dp2 + dObs_dp3_*dp3;
    
    // First estimates Jacobians are used as this shown to keep the estimator consistent
    if (jacobians != NULL) {
      if (jacobians[0] != NULL) {
        Eigen::Map<Eigen::Matrix<double,6,6,Eigen::RowMajor>>
          jac(jacobians[0]);
        jac = dObs_dp0_;
      }
      if (jacobians[1] != NULL) {
        Eigen::Map<Eigen::Matrix<double,6,6,Eigen::RowMajor>>
          jac(jacobians[1]);
        jac = dObs_dp1_;
      }
      if (jacobians[2] != NULL) {
        Eigen::Map<Eigen::Matrix<double,6,6,Eigen::RowMajor>>
          jac(jacobians[2]);
        jac = dObs_dp2_;
      }
      if (jacobians[3] != NULL) {
        Eigen::Map<Eigen::Matrix<double,6,6,Eigen::RowMajor>>
          jac(jacobians[3]);
        jac = dObs_dp3_;
      }
    }
    
    return true;
  }
  
  virtual ~PoseSpline_PoseResidual() {}   // All should safely self destruct
  
};    // PoseSpline_PoseResidual


// Cubic pose spline - Uniform B-spline
//  Maths here is from Notebook #6 pg 35-40
class UniformCubicPoseSpline {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef anantak::TimedCircularQueue<anantak::PoseState> PoseStateQueue;
  
  std::unique_ptr<PoseStateQueue>
    control_poses_;       // Control poses timed circular queue
  PoseStateQueue::FixedPoint
    segment_marker_;      // Fixed point on control poses marking beginning of segment
  PoseStateQueue::FixedPoint
    prior_marker_;        // Fixed point marking till where priors have been created
  
  double dt_;             // Uniform pose spline's knot distance in seconds
  int64_t dts_;           // Uniform pose spline's knot distance in microseconds
  int64_t starting_ts_;   // Starting timestamp of the spline (microseconds)
  int64_t ending_ts_;     // Starting timestamp of the spline (microseconds)
  
  // Helpers
  Matrix4d M4_;       // Cubic spline basis matrix. Using K.Qin terminology.
  Matrix4d CB4_;      // Cubic spline cummulative basis matrix.
  
  PoseState* p0_;     // P[i] pose state 
  PoseState* p1_;     // P[i-1] pose state 
  PoseState* p2_;     // P[i-2] pose state 
  PoseState* p3_;     // P[i-3] pose state in control poses queue
  
  double recip_dt_, recip_dtdt_;
  
  // Construction requires:
  //  size: number of control poses to keep in the circular queue
  //  dt: distance between the knots
  UniformCubicPoseSpline(int32_t size, int64_t dts) {
    // Allocate memory for the queue
    std::unique_ptr<anantak::TimedCircularQueue<anantak::PoseState>> poses_ptr(
        new anantak::TimedCircularQueue<anantak::PoseState>(size));
    control_poses_ = std::move(poses_ptr);
    LOG(INFO) << "Allocated memory for TimedCircularQueue of UniformCubicPoseSpline " << size;
    
    // Add a fixed point
    segment_marker_ = control_poses_->CurrentFixedPoint();  // Marks the control pose before current segment
    prior_marker_ = control_poses_->CurrentFixedPoint();  // Marks last control pose with prior
    control_poses_->SetFixedPoint("ConstantBegin");   // Marks where constant control poses begin
    control_poses_->SetFixedPoint("VariableBegin");   // Marks where variable control poses begin
    
    dts_ = dts;
    dt_ = double(dts_)*1e-6;
    ending_ts_ = 0;
    starting_ts_ = 0;
    
    recip_dt_ = 1./dt_;
    recip_dtdt_ = recip_dt_ * recip_dt_;
    
    M4_ <<  1.,4.,1.,0.,  -3.,0.,3.,0.,  3.,-6.,3.,0.,  -1.,3.,-3.,1.;
    M4_ /= 6.;
    VLOG(1) << "Cubic B-spline M4 = \n" << M4_;
    
    Matrix4d  sum_mat; sum_mat << 1.,0.,0.,0., 1.,1.,0.,0.,  1.,1.,1.,0.,  1.,1.,1.,1.;
    CB4_ = M4_*sum_mat;
    VLOG(1) << "Cubic B-spline CB4 = \n" << CB4_;
    
    p3_ = p2_ = p1_ = p0_ = nullptr;
  }
  
  // Reset the spline
  bool Reset() {
    for (int i=0; i<control_poses_->n_msgs(); i++) {
      PoseState* pose = control_poses_->AtPtr(i);
      pose->SetZero();
    }
    control_poses_->Clear();
    segment_marker_.Reset();
    prior_marker_.Reset();
    control_poses_->GetFixedPoint("ConstantBegin").Reset();
    control_poses_->GetFixedPoint("VariableBegin").Reset();
    ending_ts_ = 0;
    starting_ts_ = 0;
    p3_ = p2_ = p1_ = p0_ = nullptr;
    return true;
  }
  
  inline int64_t StartingTimestamp() const {return starting_ts_;}
  inline int64_t EndingTimestamp() const {return ending_ts_;}
  inline int64_t CurrentStartingTimestamp() const {return control_poses_->FirstTimestamp()+3*dts_;}
  inline int64_t CurrentEndingTimestamp() const {return ending_ts_;}
  inline int32_t NumControlPoints() const {return control_poses_->n_msgs();}
  inline int64_t KnotDistance() const {return dts_;}
  
  inline bool U(const double u, RowVector4d* vec) {
    (*vec)[0] = 1.;
    (*vec)[1] = u;
    (*vec)[2] = (*vec)[1]*u;
    (*vec)[3] = (*vec)[2]*u;
    return true;
  }
  
  inline bool dU(const double u, RowVector4d* vec) {
    (*vec)[0] = 0.;
    (*vec)[1] = 1.;
    (*vec)[2] = 2.*u;
    (*vec)[3] = 3.*u*u;
    *vec *= recip_dt_;
    return true;
  }
  
  inline bool ddU(const double u, RowVector4d* vec) {
    (*vec)[0] = 0.;
    (*vec)[1] = 0.;
    (*vec)[2] = 2.;
    (*vec)[3] = 6.*u;
    *vec *= recip_dtdt_;
    return true;
  }
  
  inline bool CheckPosePointers() {
    return (p3_ && p2_ && p1_ && p0_);
  }
  
  // Find the control interval for a given time
  inline bool PositionPosePointers(const int64_t ts) {
    if (ts < starting_ts_) {
      LOG(ERROR) << "ts < starting_ts_ " << ts << " " << starting_ts_;
      return false;
    }
    if (ts > control_poses_->LastTimestamp()) {
      segment_marker_ = control_poses_->CurrentFixedPoint();
      p0_ = control_poses_->NthLastElementPtr(1);
      p1_ = control_poses_->NthLastElementPtr(2);
      p2_ = control_poses_->NthLastElementPtr(3);
      p3_ = control_poses_->NthLastElementPtr(4);
      VLOG(3) << "ts, p0 p1 p2 p3: " << ts << ", " << p0_->Timestamp() << " " << p1_->Timestamp()
          << " " << p2_->Timestamp() << " " << p3_->Timestamp();
      return CheckPosePointers();
    }
    if (!control_poses_->PositionFixedPointBeforeTimestamp(segment_marker_, ts, true, true)) {
      LOG(ERROR) << "Could not position in control poses for the segment";
      return false;
    }
    //VLOG(2) << "Marker set at: " << segment_marker_.ToString() << " "
    //    << control_poses_->Timestamp(segment_marker_);
    p0_ = control_poses_->NthLastElementPtrBefore(segment_marker_, 1);
    p1_ = control_poses_->NthLastElementPtrBefore(segment_marker_, 2);
    p2_ = control_poses_->NthLastElementPtrBefore(segment_marker_, 3);
    p3_ = control_poses_->NthLastElementPtrBefore(segment_marker_, 4);
    VLOG(3) << "ts, p0 p1 p2 p3: " << ts << ", " << p0_->Timestamp() << " " << p1_->Timestamp()
        << " " << p2_->Timestamp() << " " << p3_->Timestamp();
    return CheckPosePointers();
  }
  
  // Add control pose
  inline bool AddControlPose(const PoseState& pose) {
    if (control_poses_->n_msgs() > 0) {
      if (pose.Timestamp() <= control_poses_->LastTimestamp()) {
        LOG(ERROR) << "Pose ts is not greater than spline's last ts " << pose.Timestamp() << " "
            << control_poses_->LastTimestamp();
        return false;
      }
      if (pose.Timestamp() != control_poses_->LastTimestamp() + dts_) {
        LOG(ERROR) << "This is uniform B-spline. pose ts should be spline_ts + dts. Got "
            << pose.Timestamp() << " Expecting " << control_poses_->LastTimestamp() + dts_;
        return false;
      }
    }
    if (control_poses_->n_msgs() == 0) starting_ts_ = pose.Timestamp();
    control_poses_->AddElement(pose); // copy pose
    control_poses_->SetTimestamp(pose.Timestamp()); // Set queue timestamp
    control_poses_->BackPtr()->UpdateRotations();   // Making sure that quaternion is normalized
    VLOG(2) << "Added a control pose with timestamp " << pose.Timestamp();
    ending_ts_ = control_poses_->LastTimestamp();
    return true;
  }
  
  // Interpolated pose
  inline bool InterpolatePose(const int64_t ts, KinematicState* kin, bool include_errors = false) {
    if (!kin) {LOG(ERROR) << "kin is NULL"; return false;}
    Quaterniond q; Vector3d p;
    Vector3d w; Vector3d v;
    if (!CalculatePoseAndDerivatives(
          ts,
          &q, &p, nullptr, nullptr,
          &w, &v, nullptr, nullptr,   // w, v are expressed in reference frame
          nullptr, nullptr, nullptr, nullptr,
          include_errors
        )) {
      LOG(ERROR) << "Could not interpolate pose using spline. ts " << ts;
      return false;
    }
    kin->SetZero();             // Setting all data to zero
    kin->SetTimestamp(ts);
    kin->SetPose(q, p);
    Matrix3d R(q);              // R is B_r_A
    kin->SetVelocity(R*w, R*v); // w, v are expressed in body frame in kin pose
    kin->SetTime(double(ts-starting_ts_)*1e-6);
    // Acceleration to be added
    return true;
  }
  
  // Get pose at a given timestamp
  inline bool InterpolatePose(const int64_t ts, PoseState* pose, bool include_errors = false) {
    if (!pose) {LOG(ERROR) << "pose is NULL"; return false;}
    Quaterniond q; Vector3d p;
    if (!CalculatePoseAndDerivatives(
          ts,
          &q, &p, nullptr, nullptr,
          nullptr, nullptr, nullptr, nullptr,
          nullptr, nullptr, nullptr, nullptr,
          include_errors
        )) {
      LOG(ERROR) << "Could not interpolate pose using spline. ts " << ts;
      return false;
    }
    pose->SetTimestamp(ts);
    pose->SetPose(q, p);
    // Covariances are not changed
    return true;
  }
  
  // Set control pose pointers
  inline bool GetControlPoses(PoseState** p0, PoseState** p1, PoseState** p2, PoseState** p3) const {
    *p0 = p0_; *p1 = p1_; *p2 = p2_; *p3 = p3_;
    return true;
  }
  
  // Calculate pose, velocity and accelerations using last four control poses
  //  Maths in Notebook #6 pg 38
  inline bool CalculatePoseAndDerivatives(const int64_t ts,
      // Estimates                              // Jacobians - ordered as pose# 0,-1,-2,-3
      Quaterniond* B01_q_A, Vector3d* A_p_B01,  Matrix3x12d* dq_dq,  Matrix3x12d* dp_dp,
      Vector3d* A_w_B01,  Vector3d* A_v_B01,    Matrix3x12d* dw_dq,  Matrix3x12d* dv_dp,
      Vector3d* A_dw_B01, Vector3d* A_dv_B01,   Matrix3x12d* ddw_dq, Matrix3x12d* ddv_dp,
      bool include_errors = false
  ) {
    if (!PositionPosePointers(ts)) {
      LOG(ERROR) << "Could not position pointers for ts " << ts;
      return false;
    }
    
    double u = double(ts - p0_->Timestamp())*1e-6/dt_;
    if (u<0.) {LOG(FATAL) << "u<0. u=" << u << " ts " << ts
        << " p0 p1 p2 p3 " << p0_->Timestamp() << " " << p1_->Timestamp() << " "
        << p2_->Timestamp() << " " << p3_->Timestamp();
    }
    if (u>1.) {VLOG(2) << "u>1. u=" << u << " ts " << ts
        << " p0 p1 p2 p3 " << p0_->Timestamp() << " " << p1_->Timestamp() << " "
        << p2_->Timestamp() << " " << p3_->Timestamp();
    }
    //VLOG(2) << "Using u = " << u;
    
    if (!B01_q_A || !A_p_B01) {LOG(ERROR)<<"!B01_q_A || !A_p_B01"; return false;}
    
    RowVector4d u_vec, B, CB;
    U(u, &u_vec);
    B  = u_vec * M4_;
    CB = u_vec * CB4_;
    
    // Use first estimates by default
    Quaterniond B0_q_A(p0_->Q0());
    Quaterniond B1_q_A(p1_->Q0());
    Quaterniond B2_q_A(p2_->Q0());
    Quaterniond B3_q_A(p3_->Q0());
      
    Vector3d A_p_B3(p3_->P0());
    Vector3d A_p_B2(p2_->P0());
    Vector3d A_p_B1(p1_->P0());
    Vector3d A_p_B0(p0_->P0());
    
    // If asked, use current estimates
    if (include_errors) {
      B0_q_A = p0_->Q();
      B1_q_A = p1_->Q();
      B2_q_A = p2_->Q();
      B3_q_A = p3_->Q();
      
      A_p_B3 = p3_->P();
      A_p_B2 = p2_->P();
      A_p_B1 = p1_->P();
      A_p_B0 = p0_->P();
    }
    
    // Check if quaternions' norm is 1
    CheckQuaternionNorm(B0_q_A, "B0_q_A");
    CheckQuaternionNorm(B1_q_A, "B1_q_A");
    CheckQuaternionNorm(B2_q_A, "B2_q_A");
    CheckQuaternionNorm(B3_q_A, "B3_q_A");
      
    Quaterniond B0_q_B1 = B0_q_A * B1_q_A.conjugate();    // -1_w_0
    Quaterniond B1_q_B2 = B1_q_A * B2_q_A.conjugate();    // -2_w_-1
    Quaterniond B2_q_B3 = B2_q_A * B3_q_A.conjugate();    // -3_w_-2
    
    Vector3d B1_w_B0 = -QuaternionToErrorAngleAxis(B0_q_B1);  // -1_w_0
    Vector3d B2_w_B1 = -QuaternionToErrorAngleAxis(B1_q_B2);  // -2_w_-1
    Vector3d B3_w_B2 = -QuaternionToErrorAngleAxis(B2_q_B3);  // -3_w_-2
    
    Vector3d B1_wB_B0 = CB[3]*B1_w_B0;
    Vector3d B2_wB_B1 = CB[2]*B2_w_B1;
    Vector3d B3_wB_B2 = CB[1]*B3_w_B2;
    
    if (!B1_wB_B0.allFinite()) {LOG(ERROR) << "B1_wB_B0 = " << B1_wB_B0.transpose()
        << "B1_w_B0 = " << B1_w_B0.transpose() << " B0_q_B1 = " << B0_q_B1.coeffs().transpose();}
    if (!B2_wB_B1.allFinite()) {LOG(ERROR) << "B2_wB_B1 = " << B2_wB_B1.transpose()
        << "B2_w_B1 = " << B2_w_B1.transpose() << " B1_q_B2 = " << B1_q_B2.coeffs().transpose();}
    if (!B3_wB_B2.allFinite()) {LOG(ERROR) << "B3_wB_B2 = " << B3_wB_B2.transpose()
        << "B3_w_B2 = " << B3_w_B2.transpose() << " B2_q_B3 = " << B2_q_B3.coeffs().transpose();}
    
    Quaterniond B0_qB_B1 = ErrorAngleAxisToQuaternion(-B1_wB_B0);
    Quaterniond B1_qB_B2 = ErrorAngleAxisToQuaternion(-B2_wB_B1);
    Quaterniond B2_qB_B3 = ErrorAngleAxisToQuaternion(-B3_wB_B2);
    
    Quaterniond B3_qB_A = B3_q_A;                  // B-3_q_A
    Quaterniond B2_qB_A = B2_qB_B3 * B3_qB_A;      // B-2_qB_B-3 * B-3_q_A
    Quaterniond B1_qB_A = B1_qB_B2 * B2_qB_A;      // B-1_qB_B-2 * B-2_qB_B-3 * B-3_q_A
    
    
    if (B01_q_A) {
      *B01_q_A = B0_qB_B1 * B1_qB_A;      // B0_qB_B-1 * B-1_qB_B-2 * B-2_qB_B-3 * B-3_q_A
    }
    
    if (A_p_B01) {
      *A_p_B01 = B[0]*A_p_B3 + B[1]*A_p_B2 + B[2]*A_p_B1 + B[3]*A_p_B0;
    }
    
    if (!dq_dq && !dp_dp && !A_w_B01 && !A_v_B01) {return true;} 
    
    Matrix3d A_rB_B1(B1_qB_A.conjugate());
    Matrix3d A_rB_B2(B2_qB_A.conjugate());
    Matrix3d A_rB_B3(B3_qB_A.conjugate());
    
    Matrix3x9d jac_dqB01_by_dw123;
    jac_dqB01_by_dw123.block<3,3>(0,0) = A_rB_B1 * CB[3];
    jac_dqB01_by_dw123.block<3,3>(0,3) = A_rB_B2 * CB[2];
    jac_dqB01_by_dw123.block<3,3>(0,6) = A_rB_B3 * CB[1];
    
    Matrix3d B1_r_A(B1_q_A);
    Matrix3d B2_r_A(B2_q_A);
    Matrix3d B3_r_A(B3_q_A);
    
    Matrix9x12d jac_dw123_by_dtheta;
    jac_dw123_by_dtheta.setZero();
    jac_dw123_by_dtheta.block<3,3>(0,0) =  B1_r_A;
    jac_dw123_by_dtheta.block<3,3>(0,3) = -B1_r_A;
    jac_dw123_by_dtheta.block<3,3>(3,3) =  B2_r_A;
    jac_dw123_by_dtheta.block<3,3>(3,6) = -B2_r_A;
    jac_dw123_by_dtheta.block<3,3>(6,6) =  B3_r_A;
    jac_dw123_by_dtheta.block<3,3>(6,9) = -B3_r_A;
    
    if (dq_dq) {
      dq_dq->setZero();
      *dq_dq = jac_dqB01_by_dw123 * jac_dw123_by_dtheta;
      dq_dq->block<3,3>(0,9) += kIdentity3d;
    }
    
    if (dp_dp) {
      dp_dp->block<3,3>(0,0) = B[3] * kIdentity3d;
      dp_dp->block<3,3>(0,3) = B[2] * kIdentity3d;
      dp_dp->block<3,3>(0,6) = B[1] * kIdentity3d;
      dp_dp->block<3,3>(0,9) = B[0] * kIdentity3d;
    }
    
    if (!A_w_B01 && !A_v_B01) {return true;}
    
    RowVector4d du_vec, dCB;
    dU(u, &du_vec);
    dCB = du_vec * CB4_;
    
    if (A_w_B01) {
      *A_w_B01 = A_rB_B1*(dCB[3]*B1_w_B0) + A_rB_B2*(dCB[2]*B2_w_B1) + A_rB_B3*(dCB[1]*B3_w_B2);
      //VLOG(2) << "\nB1_w_B0 " << B1_w_B0.transpose() << "\nB2_w_B1 " << B2_w_B1.transpose()
      //    << "\nB3_w_B2 " << B3_w_B2.transpose() << "\ndCB " << dCB
      //    << "\nB0_q_B1 " << B0_q_B1.coeffs().transpose()
      //    << "\nB1_q_B2 " << B1_q_B2.coeffs().transpose()
      //    << "\nB2_q_B3 " << B2_q_B3.coeffs().transpose();
    }
    
    if (A_v_B01) {
      *A_v_B01 = dCB[3]*A_p_B0 + (dCB[2]-dCB[3])*A_p_B1 + (dCB[1]-dCB[2])*A_p_B2 - dCB[1]*A_p_B3;
    }
    
    // Jacobians of velocities
    // Accelerations
    // Jacobians of accelerations
    
    return true;
  }
  
  // Initiate spline with a single pose
  inline bool InitiateSpline(const PoseState& obs0) {
    
    // Spline control pose [i] timestamp
    int64_t ts0 = obs0.Timestamp();
    int64_t ts_0 = (ts0 / dts_) * dts_;   // [i]
    
    // Velocities are zero
    
    // Interpolated pose [i]
    Quaterniond q0 = obs0.Q();
    Vector3d p0 = obs0.P();
    
    // How many poses are needed to cover the second timestamp?
    VLOG(2) << "Initiating the cubic pose spline by creating " << 4
        << " control poses. First interpolation interval ts: " << ts_0;
    
    // Add starting control poses to spline
    for (int i=0; i<4; i++) {
      int64_t ts = ts_0 + (-3+i)*dts_;
      Quaterniond q = q0;
      Vector3d p = p0;
      PoseState* pose = control_poses_->NextMutableElement();
      pose->SetZero();
      pose->SetTimestamp(ts);
      pose->SetPose(q,p);
      control_poses_->SetTimestamp(ts);
      VLOG(1) << "Added control pose " << i << " " << pose->ToString();
    }
    
    starting_ts_ = ts_0;
    ending_ts_ = control_poses_->LastTimestamp();
    
    return true;
  }
  
  // Initiate spline with two starting poses
  //  Create first four control points using a line connecting two observations
  inline bool InitiateSpline(const PoseState& obs0, const PoseState& obs1) {
    
    // Spline control pose [i] timestamp
    int64_t ts0 = obs0.Timestamp();
    int64_t ts1 = obs1.Timestamp();
    int64_t ts_0 = (ts0 / dts_) * dts_;   // [i]
    
    // Velocities
    double delta_t = double(ts1-ts0)*1e-6;
    Quaterniond delta_q = obs1.Q() * obs0.Q().conjugate();
    Vector3d w = QuaternionToErrorAngleAxis(delta_q)/delta_t;        // Small angle approximation
    Vector3d v = (obs1.P() - obs0.P())/delta_t;
    
    // Interpolated pose [i]
    double delta_t0 = double(ts_0 - ts0)*1e-6;
    Vector3d delta_q0 = w*delta_t0;
    Quaterniond q0 = ErrorAngleAxisToQuaternion(delta_q0) * obs0.Q();
    Vector3d p0 = obs0.P() + v*delta_t0;
    
    // How many poses are needed to cover the second timestamp?
    const int64_t end_ts = (ts1 / dts_) * dts_;
    const int32_t num_extra_poses_to_create = (end_ts - ts_0) / dts_;
    VLOG(2) << "Initiating the cubic pose spline by creating " << 4+num_extra_poses_to_create
        << " control poses. First interpolation interval ts: " << ts_0;
    
    // Add starting control poses to spline
    for (int i=0; i<4+num_extra_poses_to_create; i++) {
      int64_t ts = ts_0 + (-3+i)*dts_;
      double dt = double(i-1)*dt_;
      Quaterniond q = ErrorAngleAxisToQuaternion(w*dt) * q0;
      Vector3d p = p0 + v*dt;
      PoseState* pose = control_poses_->NextMutableElement();
      pose->SetZero();
      pose->SetTimestamp(ts);
      pose->SetPose(q,p);
      control_poses_->SetTimestamp(ts);
      VLOG(1) << "Added control pose " << i << " " << pose->ToString();
    }
    
    starting_ts_ = ts_0;
    ending_ts_ = control_poses_->LastTimestamp();
    
    return true;
  }
  
  // Extend spline - adds control poses till provided timestamp is covered
  inline bool ExtendSpline(const int64_t ts) {
    
    // How many poses are needed to cover the given timestamp?
    const int64_t ts_0 = control_poses_->LastTimestamp();   // current timestamp [i]
    const int64_t end_ts = (ts / dts_) * dts_;
    const int32_t num_new_poses_to_create = (end_ts - ts_0) / dts_;
    VLOG(1) << "Extending the spline by creating " << num_new_poses_to_create
        << " new control poses. " << ts_0 << " to " << end_ts << " ("<<end_ts-ts_0<<") " << ts;
    
    if (num_new_poses_to_create==0) return true;
    
    // Add new control points
    for (int i=0; i<num_new_poses_to_create; i++) {
      const PoseState& P0 = control_poses_->NthLastElement(1);
      const PoseState& P1 = control_poses_->NthLastElement(2);
      int64_t ts = P0.Timestamp() + dts_;
      // Extend spline as line joining last two control poses - if last estimate is poor, this is poor
      //Quaterniond dq = P0.Q() * P1.Q().conjugate();
      //Vector3d dp = P0.P() - P1.P();
      //Quaterniond q = dq * P0.Q();  // Extend spline by using last omega. If angular pose is noisy this is unstable.
      //Vector3d p = P0.P() + dp;
      // Extend spline by copying last pose - if last estimate is poor, this is poor too
      //Quaterniond q = P0.Q();   // Extend spline assuming zero omega. If angular pose is noisy, this is better.
      //Vector3d p = P0.P();
      // Extend spline by copying second last pose - if last estimate is poor, this would do better
      Quaterniond q = P1.Q();
      Vector3d p = P1.P();
      PoseState* pose = control_poses_->NextMutableElement();
      pose->SetZero();
      pose->SetTimestamp(ts);
      pose->SetPose(q,p);
      control_poses_->SetTimestamp(ts);
      VLOG(2) << "Added control pose " << i << " " << pose->ToString();
    }
    
    ending_ts_ = control_poses_->LastTimestamp();
    return true;
  }
  
  // Create a pose residual from a pose observation
  inline bool CreatePoseResidual(const PoseState& observation, PoseSpline_PoseResidual* pose_residual) {
    if (!pose_residual) {LOG(ERROR) << "pose_residual is NULL"; return false;}
    const int64_t ts = observation.Timestamp();
    Quaterniond q; Vector3d p;
    Matrix3x12d dq_dq;  Matrix3x12d dp_dp;
    if (!CalculatePoseAndDerivatives(
          ts,
          &q, &p, &dq_dq, &dp_dp,
          nullptr, nullptr, nullptr, nullptr,   // w, v are expressed in reference frame
          nullptr, nullptr, nullptr, nullptr,
          false                                 // Using first estimates
        )) {
      LOG(ERROR) << "Could not interpolate pose using spline. ts " << ts;
      return false;
    }
    if (!pose_residual->Create(observation,   p0_, p1_, p2_, p3_,   &q, &p,   &dq_dq, &dp_dp)) {
      LOG(ERROR) << "Could not create pose residual from the observation";
      pose_residual->Reset();
      return false;
    }
    
    return true;
  }
  
  // Create priors on control poses and add them to a given queue
  inline bool CreateControlPosePriors(
      anantak::TimedCircularQueue<anantak::PoseSpline_ControlPosePrior>* priors_queue, int32_t* n) {
    if (!priors_queue) {LOG(ERROR) << "priors_queue is null"; return false;}
    if (!n) {LOG(ERROR) << "n is null"; return false;}
    
    for (auto fp = control_poses_->NextFixedPoint(prior_marker_);
         control_poses_->IsNotPastTheEnd(fp);
         control_poses_->IncrementFixedPoint(fp)) {
      PoseState* control_pose = control_poses_->MutableElementAtFixedPoint(fp);
      PoseSpline_ControlPosePrior* prior = priors_queue->NextMutableElement();
      prior->Reset();
      VLOG(1) << "Creating control pose prior at ts " << control_pose->Timestamp();
      if (!prior->Create(control_pose)) {
        LOG(ERROR) << "Could not create a pose prior. Skip.";
        prior->Reset();
        priors_queue->decrement();
      } else {
        priors_queue->SetTimestamp(control_pose->Timestamp());
        (*n)++;
      }
    }
    // Set priors marker at the end
    prior_marker_ = control_poses_->CurrentFixedPoint(); // Sets fixed point at the end
    
    return true;
  }
  
  // Create a velocity residual
  // Create an acceleration residual
  
  // Recalculate the control poses starting from VariableBegin marker
  bool RecalculateControlPoses(int32_t* num_recalculated) {
    
    for (auto fp = control_poses_->GetFixedPoint("VariableBegin");
         control_poses_->IsNotPastTheEnd(fp);
         control_poses_->IncrementFixedPoint(fp)) {
      PoseState* pose = control_poses_->MutableElementAtFixedPoint(fp);
      if (pose) {
        pose->Recalculate();
        (*num_recalculated)++;
      }
    }
    
    return true;
  }
  
  bool MarkControlPosesConstant(const int64_t constant_begin_ts, const int64_t variable_begin_ts,
                                ceres::Problem* problem, int32_t* num_marked) {
    
    // Locate constant begin timestamp
    if (!control_poses_->PositionFixedPointBeforeTimestamp("ConstantBegin", constant_begin_ts, true, true)) {
      LOG(WARNING) << "Could not locate constant_begin_ts in control_poses_ " << constant_begin_ts;
      return false;
    }
    VLOG(2) << "constant_begin_ts in control_poses_ found at " << control_poses_->FixedPointToString("ConstantBegin");
    
    // Locate variable begin timestamp
    if (!control_poses_->PositionFixedPointBeforeTimestamp("VariableBegin", variable_begin_ts, true, true)) {
      LOG(WARNING) << "Could not locate variable_begin_ts in control_poses_ " << variable_begin_ts;
      return false;
    }
    VLOG(2) << "variable_begin_ts in control_poses_ found at " << control_poses_->FixedPointToString("VariableBegin");
    
    // Mark control poses constant
    for (auto fp = control_poses_->GetFixedPoint("ConstantBegin");
         control_poses_->IsNotAtFixedPoint("VariableBegin", fp);
         control_poses_->IncrementFixedPoint(fp)) {
      double* error_data_ptr = control_poses_->MutableElementAtFixedPoint(fp)->error_.data();
      if (problem->HasParameterBlock(error_data_ptr))
        problem->SetParameterBlockConstant(error_data_ptr);    
      (*num_marked)++;
    }
    
    return true;
  }
  
  bool MarkAdditionalControlPosesConstant(const int64_t variable_begin_ts,
                                ceres::Problem* problem, int32_t* num_marked) {
    // Make a copy of the current Variable begin fp
    const auto last_variable_begin_fp = control_poses_->GetFixedPoint("VariableBegin");
    
    if (variable_begin_ts < control_poses_->Timestamp(last_variable_begin_fp)) {
      LOG(FATAL) << "New control poses variable begin ts is < last one! Not expected"
          << variable_begin_ts << " " << control_poses_->Timestamp(last_variable_begin_fp);
    }
    
    // Locate new variable begin ts
    if (!control_poses_->PositionFixedPointBeforeTimestamp("VariableBegin", variable_begin_ts, true, true)) {
      LOG(WARNING) << "Could not locate variable_begin_ts in control_poses_ " << variable_begin_ts;
      return false;
    }
    VLOG(2) << "variable_begin_ts in control_poses_ found at " << control_poses_->FixedPointToString("VariableBegin");
    
    // Mark control poses constant
    for (auto fp = last_variable_begin_fp;
         control_poses_->IsNotAtFixedPoint("VariableBegin", fp);
         control_poses_->IncrementFixedPoint(fp)) {
      double* error_data_ptr = control_poses_->MutableElementAtFixedPoint(fp)->error_.data();
      if (problem->HasParameterBlock(error_data_ptr))
        problem->SetParameterBlockConstant(error_data_ptr);    
      (*num_marked)++;
    }
    
    return true;
  }
  
}; // Uniform cubic pose spline


// Target view residual using the uniform cubic pose spline
class PoseSpline_CalibrationTargetViewResidual : public ceres::CostFunction {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  static const int kMaxNumberOfResiduals = 280; // = 2*4*35. 35 tags, 4 corners and 2 indices
  
  struct Options {
    bool solve_for_intrinsics;  // default false. Solve for camera intrinsics?
    bool solve_for_extrinsics;  // default false. Solve for camera pose?
    bool solve_for_timedelay;   // default false. Solve for camera timedelay?
    
    double image_sigma;         // sigma of the corner seen in image. In pixels.
    Matrix2d image_corner_covariance; // Covariance matrix for tag corner view.
    
    Options(const double im_sigma = 1.,
            const bool si = false,
            const bool se = false,
            const bool st = false):
        image_sigma(im_sigma),        // in pixels
        solve_for_intrinsics(si),
        solve_for_extrinsics(se),
        solve_for_timedelay(st) {
      image_corner_covariance = image_sigma*image_sigma*Matrix2d::Identity();
    }
  };
  
  // Options
  const PoseSpline_Options*   pose_spline_options_;
  const PoseSpline_CalibrationTargetViewResidual::Options*  options_;
  
  // Observation
  int64_t           timestamp_;         // Timestamp of the observation
  PoseState         target_pose_est_;   // This is the starting target pose from spline
  PoseState         observation_;       // Approximate pose estimate from the observation
  
  // Calibration target
  const CameraCalibrationTarget* target_;
  
  // State: Cubic pose spline
  UniformCubicPoseSpline*   cubic_pose_spline_;     // Cubic pose spline
  PoseState*    p0_;     // P[i] pose state 
  PoseState*    p1_;     // P[i-1] pose state 
  PoseState*    p2_;     // P[i-2] pose state 
  PoseState*    p3_;     // P[i-3] pose state in control poses queue
  
  // State: Camera
  const UndistortedCamera*  undistorted_camera_;   // Undistorting camera based off of camera_
  CameraState*              camera_;  // Holds intrinsics, extrisics and time delay of the camera
  
  // Residual and Jacobians
  int32_t
      num_residuals_;   // Number of corners seen * 2 (dimensions)
  Eigen::Matrix<double,Eigen::Dynamic,1,0,kMaxNumberOfResiduals,1>
      residual_;        // Starting residual
  Eigen::Matrix<double,Eigen::Dynamic,6,0,kMaxNumberOfResiduals,6>
      dcorner_dp0_;     // Jacobian of corner observation by spline pose[i]
  Eigen::Matrix<double,Eigen::Dynamic,6,0,kMaxNumberOfResiduals,6>
      dcorner_dp1_;     // Jacobian of corner observation by spline pose[i-1]
  Eigen::Matrix<double,Eigen::Dynamic,6,0,kMaxNumberOfResiduals,6>
      dcorner_dp2_;     // Jacobian of corner observation by spline pose[i-2]
  Eigen::Matrix<double,Eigen::Dynamic,6,0,kMaxNumberOfResiduals,6>
      dcorner_dp3_;     // Jacobian of corner observation by spline pose[i-3]
  
  //Eigen::Matrix<double,Eigen::Dynamic,9,0,kMaxNumberOfResiduals,9>
  //    dcorner_dintrinsics_; // Jacobian of corners with camera instrinsics
  //Eigen::Matrix<double,Eigen::Dynamic,6,0,kMaxNumberOfResiduals,6>
  //    dcorner_dextrinsics_; // Jacobian of corners with camera extrinsics
  //Eigen::Matrix<double,Eigen::Dynamic,1,0,kMaxNumberOfResiduals,1>
  //    dcorner_dtimedelay_; // Jacobian of corners with camera time delay
  
  PoseSpline_CalibrationTargetViewResidual(
      const PoseSpline_Options*  pose_spline_options,
      const PoseSpline_CalibrationTargetViewResidual::Options*  options) {
    SetOptions(pose_spline_options, options);
    Reset();
  }
  
  bool SetOptions(
      const PoseSpline_Options*  pose_spline_options,
      const PoseSpline_CalibrationTargetViewResidual::Options*  options) {
    if (!pose_spline_options) {LOG(FATAL) << "pose_spline_options pointer is null";}
    if (!options) {LOG(FATAL) << "options pointer is null";}
    pose_spline_options_ = pose_spline_options;
    options_ = options;
  }
  
  bool Reset() {    // options_ is not reset
    timestamp_ = 0; target_pose_est_.SetZero();
    target_ = nullptr; cubic_pose_spline_ = nullptr;
    p3_ = nullptr; p2_ = nullptr; p1_ = nullptr; p0_ = nullptr;
    undistorted_camera_ = nullptr; camera_ = nullptr;
    num_residuals_ = 0;
    residual_.setZero();
    dcorner_dp3_.setZero(); dcorner_dp2_.setZero(); dcorner_dp1_.setZero(); dcorner_dp0_.setZero();
    // Set the members of the cost function
    set_num_residuals(0);
    mutable_parameter_block_sizes()->clear();
  }
  
  PoseSpline_CalibrationTargetViewResidual(const PoseSpline_CalibrationTargetViewResidual& r):
    pose_spline_options_(r.pose_spline_options_), options_(r.options_),
    timestamp_(r.timestamp_), target_pose_est_(r.target_pose_est_),
    target_(r.target_), cubic_pose_spline_(r.cubic_pose_spline_),
    p3_(r.p3_), p2_(r.p2_), p1_(r.p1_), p0_(r.p0_),
    undistorted_camera_(r.undistorted_camera_), camera_(r.camera_),
    num_residuals_(r.num_residuals_), residual_(r.residual_),
    dcorner_dp3_(r.dcorner_dp3_), dcorner_dp2_(r.dcorner_dp2_),
    dcorner_dp1_(r.dcorner_dp1_), dcorner_dp0_(r.dcorner_dp0_)
  { set_num_residuals(r.num_residuals());
    *mutable_parameter_block_sizes() = r.parameter_block_sizes();
  }
  
  PoseSpline_CalibrationTargetViewResidual& operator= (const PoseSpline_CalibrationTargetViewResidual& r) {
    pose_spline_options_=r.pose_spline_options_; options_=r.options_;
    timestamp_=r.timestamp_; target_pose_est_=r.target_pose_est_;
    target_=r.target_; cubic_pose_spline_=r.cubic_pose_spline_;
    p3_=r.p3_; p2_=r.p2_; p1_=r.p1_; p0_=r.p0_;
    undistorted_camera_=r.undistorted_camera_; camera_=r.camera_;
    num_residuals_=r.num_residuals_; residual_=r.residual_;
    dcorner_dp3_=r.dcorner_dp3_; dcorner_dp2_=r.dcorner_dp2_;
    dcorner_dp1_=r.dcorner_dp1_; dcorner_dp0_=r.dcorner_dp0_;
    set_num_residuals(r.num_residuals());
    *mutable_parameter_block_sizes() = r.parameter_block_sizes();
  }
  
  virtual ~PoseSpline_CalibrationTargetViewResidual() {}
  
  // Create residual using target pose state and camera state
  bool Create(const SensorMsg& message,
              const CameraCalibrationTarget* target,
              const UndistortedCamera* undist_camera,
              UniformCubicPoseSpline* spline,
              CameraState* camera,
              cv::Mat* plot_image = nullptr) {
    // Checks
    if (!message.has_header()) {LOG(ERROR)<<"Message has no header"; return false;}
    if (!message.has_april_msg()) {LOG(ERROR)<<"Message has no aprilag message"; return false;}
    if (!target) {LOG(ERROR)<<"Calibration target is null"; return false;}
    if (!undist_camera) {LOG(ERROR)<<"Undistortion camera is null"; return false;}
    if (!spline) {LOG(ERROR)<<"Spline is null"; return false;}
    if (!camera) {LOG(ERROR)<<"Camera is null"; return false;}
    if (camera->intrinsics_.IsZero()) {LOG(WARNING)<<"camera is zero";}
    if (parameter_block_sizes().size() > 0) {
      LOG(FATAL) << "Number of parameters blocks > 0. Got: " << parameter_block_sizes().size();
    }
    
    target_ = target;
    undistorted_camera_ = undist_camera;
    cubic_pose_spline_ = spline;
    camera_ = camera;
    timestamp_ = message.header().timestamp();
    
    target_pose_est_.SetZero();
    observation_.SetZero();
    p0_ = p1_ = p2_ = p3_ = nullptr;
    
    // Calculate target pose if undistorted camera has been provided
    if (undistorted_camera_) {
      VLOG(2) << "Calculating approximate target pose";
      anantak::AprilTagMessage undist_apriltag_msg;
      undistorted_camera_->Undistort(message.april_msg(), &undist_apriltag_msg);
      if (!target_->CalculateTargetPose(undist_apriltag_msg, undistorted_camera_->camera_,
                                        &observation_, timestamp_)) {
        LOG(WARNING) << "Could not calculate target pose";
      }
    }
    
    if (!CalculateResiduals(message, plot_image)) {
      LOG(ERROR) << "Could not calculate residuals and jacobians";
      return false;
    }
    
    return true;
  }
  
  // Calculate residuals, jacobians and variances.
  //  Jacobians use first estimates. Residual is recalculated over time but jacobians never change.
  bool CalculateResiduals(const SensorMsg& message,
                          cv::Mat* plot_image = nullptr) {
    
    // Target frame = T,  Camera frame = C,  Corner on target = F
    // Target pose in camera frame = CPT
    // Corner position in target frame = TpF
    // Corner position in camera frame = CpF
    
    // Non-zero camera pose is not implemented yet
    if (!camera_->Pose().IsZero()) {
      LOG(ERROR) << "Non zero camera pose is not implemented yet."; return false;
    }
    if (!camera_->TimeDelay().IsZero()) {
      LOG(ERROR) << "Non zero camera time delay is not implemented yet."; return false;
    }
    
    // Predict the camera pose using spline
    Quaterniond  TqC;        // Rotation of target in camera frame
    Vector3d     CpT;        // Position of target in camera frame
    PoseState    CPT;        // Target pose in camera frame
    Matrix3x12d  CdqT_Pidq;  // Jacobian of target rotation wrt control poses' rotation
    Matrix3x12d  CdpT_Pidp;  // Jacobian of target position wrt control poses' position
    bool use_latest_estimates = false;  // Using first estimates. This keeps estimator consistent
    //bool use_latest_estimates = true; // Using lastest estimates would make the estimator inconsistent ** CHECK **
    VLOG(2) << "Calculating spline pose and derivatives";
    if (!cubic_pose_spline_->CalculatePoseAndDerivatives(
          timestamp_,
          &TqC, &CpT, &CdqT_Pidq, &CdpT_Pidp,
          nullptr, nullptr, nullptr, nullptr,
          nullptr, nullptr, nullptr, nullptr,
          use_latest_estimates
        )) {
      LOG(ERROR) << "Could not interpolate pose using spline. ts " << timestamp_;
      return false;
    }
    CPT.SetTimestamp(timestamp_);
    CPT.SetPose(TqC, CpT);
    target_pose_est_ = CPT;
    VLOG(2) << "Starting target pose: " << CPT.ToString();
    
    // Get the control pose pointers
    if (!cubic_pose_spline_->GetControlPoses(&p0_, &p1_, &p2_, &p3_)) {
      LOG(ERROR) << "Could not get control pose pointers from cubic spline";
      return false;
    }
    if (!p0_ || !p1_ || !p2_ || !p3_) {LOG(FATAL) << "Some or all control pose pointers are null";}
    
    // How many tags were seen? 
    const AprilTagMessage& apriltag_msg = message.april_msg();
    if (apriltag_msg.tag_id_size() == 0) {LOG(ERROR)<<"num_tags seen == 0"; return false;}
    
    // Resize residual and jacobian stores to maximum size seen in image
    int32_t max_rows = apriltag_msg.tag_id_size()*2*4;  // 4 corners per tag, 2 indices per corner
    residual_.resize(max_rows, 1); residual_.setZero();
    dcorner_dp0_.resize(max_rows, 6); dcorner_dp0_.setZero();
    dcorner_dp1_.resize(max_rows, 6); dcorner_dp1_.setZero();
    dcorner_dp2_.resize(max_rows, 6); dcorner_dp2_.setZero();
    dcorner_dp3_.resize(max_rows, 6); dcorner_dp3_.setZero();
    //VLOG(2) << "Saw " << apriltag_msg.tag_id_size() << " tags";
    
    //dcorner_dintrinsics_.resize(max_rows, 9); dcorner_dintrinsics_.setZero();
    //dcorner_dextrinsics_.resize(max_rows, 6); dcorner_dextrinsics_.setZero();
    //dcorner_dtimedelay_.resize(max_rows, 1); dcorner_dtimedelay_.setZero();
    
    int32_t num_tags = 0;
    int32_t num_corners = 0;
    
    double avg_corner_error = 0.;
    
    // Go through each tag to build up the residuals
    for (int i_tag=0; i_tag<apriltag_msg.tag_id_size(); i_tag++) {
      
      const std::string& tag_id = apriltag_msg.tag_id(i_tag);
      
      // Does this tag belong to the calibration target?
      if (!target_->TagIsPresent(tag_id)) {
        LOG(WARNING) << "Saw a tag not on target " << tag_id << ". Skipping it.";
        continue;
      }
      //VLOG(2) << "Building residuals and jacobian for tag " << tag_id;
      
      // Extract the tag corners seen in image
      Matrix2x4d image_coords, projected_coords;
      ExtractAprilTag2dCorners(apriltag_msg, i_tag, &image_coords);
      
      // Corners in target reference frame
      const Matrix3x4d& tag_corners = target_->TagCorners(tag_id);
      
      // Create a residual for each corner
      for (int i_crnr = 0; i_crnr<4; i_crnr++) {
        
        // Position of this corner wrt target
        const Vector3d TpF(tag_corners.col(i_crnr));
        
        // Position of this corner in camera frame
        Vector3d    CpF; CpF.setZero();
        Matrix3x6d  dCpF_dCPT; dCpF_dCPT.setZero();
        Matrix3d    dCpF_dTpF; dCpF_dTpF.setZero();
        if (!CPT.AddPoint(TpF, &CpF, &dCpF_dCPT, &dCpF_dTpF)) {
          LOG(FATAL) << "Could not add point. TpF: " << TpF.transpose() << " CpF: " << CpF.transpose();
          return false;
        }
        //VLOG(2) << "  Corner 3d coordinates in camera: " << CpF.transpose();
        
        // Projection of this point on camera image
        Vector2d uv; uv.setZero();
        Matrix2x3d duv_dCpF; duv_dCpF.setZero();
        if (!camera_->Intrinsics().ProjectPoint(CpF, &uv, &duv_dCpF, nullptr)) {
          LOG(FATAL) << "Could not project point. CpF: " << CpF.transpose();
          return false;          
        }
        //Matrix2x9d duv_dintrinsics; duv_dintrinsics.setZero(); 
        //camera_->Intrinsics().ProjectPoint(CpF, &uv, &duv_dCpF, &duv_dintrinsics);
        //VLOG(2) << "  Projected corner image coords: " << uv.transpose();
        
        // Jacobians of projection wrt predicted pose, intrinsics, extrinsics, time delay
        Matrix2x6d  duv_dCPT = duv_dCpF * dCpF_dCPT;
        Matrix2x3d  duv_dTpF = duv_dCpF * dCpF_dTpF;
        //Vector2d    duv_dtd  = duv_dCPT * dCPT_dtd;
        
        // Jacobians of projection wrt spline control poses
        Matrix6d dCPT_dP0; dCPT_dP0.setZero();
        dCPT_dP0.block<3,3>(0,0) = CdqT_Pidq.block<3,3>(0,0);
        dCPT_dP0.block<3,3>(3,3) = CdpT_Pidp.block<3,3>(0,0);
        Matrix2x6d  duv_dP0 = duv_dCPT * dCPT_dP0;
        
        Matrix6d dCPT_dP1; dCPT_dP1.setZero();
        dCPT_dP1.block<3,3>(0,0) = CdqT_Pidq.block<3,3>(0,3);
        dCPT_dP1.block<3,3>(3,3) = CdpT_Pidp.block<3,3>(0,3);
        Matrix2x6d  duv_dP1 = duv_dCPT * dCPT_dP1;
        
        Matrix6d dCPT_dP2; dCPT_dP2.setZero();
        dCPT_dP2.block<3,3>(0,0) = CdqT_Pidq.block<3,3>(0,6);
        dCPT_dP2.block<3,3>(3,3) = CdpT_Pidp.block<3,3>(0,6);
        Matrix2x6d  duv_dP2 = duv_dCPT * dCPT_dP2;
        
        Matrix6d dCPT_dP3; dCPT_dP3.setZero();
        dCPT_dP3.block<3,3>(0,0) = CdqT_Pidq.block<3,3>(0,9);        
        dCPT_dP3.block<3,3>(3,3) = CdpT_Pidp.block<3,3>(0,9);
        Matrix2x6d  duv_dP3 = duv_dCPT * dCPT_dP3;
        
        // Calculate starting residual
        Vector2d resid = uv - image_coords.col(i_crnr);
        projected_coords.col(i_crnr) = uv;
        avg_corner_error += resid.squaredNorm();
        //VLOG(2) << "  Seen image coordinates: " << image_coords.col(i_crnr).transpose();
        
        // Calculate covariance of the residual
        Matrix2d var_resid =
            options_->image_corner_covariance
            + duv_dTpF * target_->CornerPositionCovariance() * duv_dTpF.transpose() // Noise in knkowledge of target
            + duv_dCPT * pose_spline_options_->pose_covariance_ * duv_dCPT.transpose(); // Noise in spline trajectory model
        //VLOG(2) << "  residual variance: " << var_resid.row(0) << "  " << var_resid.row(1);
        
        //// Add variance due to intrinsics uncertainty if we are not solving for them
        //if (!options_->solve_for_intrinsics) {
        //  var_resid +=
        //    duv_dintrinsics * camera_->Intrinsics().covariance_ * duv_dintrinsics.transpose();
        ///}
        
        // Add variance due extrinsics uncertainty if we are not solving for them
        //if (!options_->solve_for_extrinsics) {
        //  var_resid +=
        //    duv_dextrinsics * camera_->Extrinsics().covariance_ * duv_dextrinsics.transpose();
        ///}
        
        //// Add variance due to timedelay uncertainty if we are not solving for it
        //if (!options_->solve_for_timedelay) {
        //  var_resid +=
        //    duv_dtd * camera_->TimeDelay().covariance_ * duv_dtd.transpose();
        ///}
        
        // Calculate inverse sqrt covariance
        Matrix2d inv_sqrt_cov; Vector2d sqrt_var;
        if (!InverseSqrt(var_resid, &inv_sqrt_cov, &sqrt_var)) {
          LOG(ERROR) << "Could not calculate inv sqrt mat";
          continue;
        }
        
        // Report calculations
        if (i_tag<5 && i_crnr==0) {
          VLOG(2) << "proj, seen, resid = " << " " << uv.transpose() << ",  "
              << image_coords.col(i_crnr).transpose() << ",  " << resid.transpose()
              << ", sqrt cov: " << sqrt_var.transpose() << ", rho: " << var_resid(0,1)/sqrt_var(0)/sqrt_var(1);
          //VLOG(1) << "inv_sqrt_cov: " << inv_sqrt_cov.row(0) << " " << inv_sqrt_cov.row(1);
        }
        
        // Normalize the residuals and Jacobians to create independent gaussians
        resid = inv_sqrt_cov * resid;
        duv_dP0 = inv_sqrt_cov * duv_dP0;
        duv_dP1 = inv_sqrt_cov * duv_dP1;
        duv_dP2 = inv_sqrt_cov * duv_dP2;
        duv_dP3 = inv_sqrt_cov * duv_dP3;
        //duv_dintrinsics = inv_sqrt_cov * duv_dintrinsics;
        //duv_dextrinsics = inv_sqrt_cov * duv_dextrinsics;
        //duv_dtd = inv_sqrt_cov * duv_dtd;
        
        // Check if there are any NANs in the calculations
        if (!resid.allFinite()
            || !duv_dP0.allFinite() || !duv_dP1.allFinite()
            || !duv_dP2.allFinite() || !duv_dP3.allFinite()
            //|| !duv_dintrinsics.allFinite() || !duv_dtd.allFinite()
        ) {
          LOG(ERROR) << "Found non finite values in residual or jacobians";
          LOG(ERROR) << "resid = " << resid.transpose();
          LOG(ERROR) << "duv_dP0 = \n" << duv_dP0;
          LOG(ERROR) << "duv_dP1 = \n" << duv_dP1;
          LOG(ERROR) << "duv_dP2 = \n" << duv_dP2;
          LOG(ERROR) << "duv_dP3 = \n" << duv_dP3;
          //LOG(ERROR) << "duv_dintrinsics = \n" << duv_dintrinsics;
          //LOG(ERROR) << "duv_dtd = \n" << duv_dtd;
          continue;
        }
        
        // Append the corner residual to the cost function residual
        residual_.block<2,1>(2*num_corners,0) = resid;
        dcorner_dp0_.block<2,6>(2*num_corners,0) = duv_dP0;
        dcorner_dp1_.block<2,6>(2*num_corners,0) = duv_dP1;
        dcorner_dp2_.block<2,6>(2*num_corners,0) = duv_dP2;
        dcorner_dp3_.block<2,6>(2*num_corners,0) = duv_dP3;
        //dcorner_dintrinsics_.block<2,9>(2*num_corners,0) = duv_dintrinsics;
        //dcorner_dextrinsics_.block<2,6>(2*num_corners,0) = duv_dextrinsics;
        //dcorner_dtimedelay_.block<2,1>(2*num_corners,0) = duv_dtd;
        
        num_corners++;
        
      } // for all four corners
      
      // Plot the tag on the image
      if (plot_image) {
        PlotTagOnImage(image_coords, CV_WHITE, plot_image);
        PlotTagOnImage(projected_coords, CV_RED, plot_image);
      }
      
      // All was good, so increment the number of tags seen
      num_tags++;
      
    }   // for all tags
    avg_corner_error /= num_corners; avg_corner_error = std::sqrt(avg_corner_error);
    VLOG(1) << "Used " << num_tags << " of " << apriltag_msg.tag_id_size() << " tags seen. "
        << " num corners: " << num_corners << " (should be " << num_tags*4 << ") "
        << " avg corner error: " << avg_corner_error;
    
    num_residuals_ = 2*num_corners;
    
    // Resize the residual and jacobians keeping calculated values if needed
    if (num_residuals_ < max_rows) {
      residual_.conservativeResize(num_residuals_, Eigen::NoChange);
      dcorner_dp0_.conservativeResize(num_residuals_, Eigen::NoChange);
      dcorner_dp1_.conservativeResize(num_residuals_, Eigen::NoChange);
      dcorner_dp2_.conservativeResize(num_residuals_, Eigen::NoChange);
      dcorner_dp3_.conservativeResize(num_residuals_, Eigen::NoChange);
      //dcorner_dintrinsics_.conservativeResize(num_residuals_, Eigen::NoChange);
      //dcorner_dextrinsics_.conservativeResize(num_residuals_, Eigen::NoChange);
      //dcorner_dtimedelay_.conservativeResize(num_residuals_, Eigen::NoChange);
      VLOG(1) << "Resized residuals/jacobians from " << max_rows << " to " << num_residuals_;
    }
    
    // Set number of residuals for solver
    set_num_residuals(num_residuals_);
    VLOG(1) << "Set number of residuals for solver: " << num_residuals_;
    
    // Set parameter block sizes for solver - camera intrinsics, extrinsics, timedelay
    mutable_parameter_block_sizes()->push_back(6);    // Control pose i has 6 parameters
    mutable_parameter_block_sizes()->push_back(6);    // Control pose i-1 has 6 parameters
    mutable_parameter_block_sizes()->push_back(6);    // Control pose i-2 has 6 parameters
    mutable_parameter_block_sizes()->push_back(6);    // Control pose i-3 has 6 parameters
    //mutable_parameter_block_sizes()->push_back(9);  // Camera intrinsics has 9 parameters 
    //mutable_parameter_block_sizes()->push_back(6);  // Camera extrinsics has 6 parameters 
    //mutable_parameter_block_sizes()->push_back(1);  // Camera timedelay has 1 parameters 
    
    return true;
  }
  
  // Evaluate function for ceres solver
  virtual bool Evaluate(
      double const* const* parameters,
      double* residuals,
      double** jacobians) const {
    
    // Mapping residuals and parameters to matrices
    MapVectorXd         residual(residuals, num_residuals_);
    MapConstVector6d    dp0(parameters[0]);   // Change in control pose [i]
    MapConstVector6d    dp1(parameters[1]);   // Change in control pose [i-1]
    MapConstVector6d    dp2(parameters[2]);   // Change in control pose [i-2]
    MapConstVector6d    dp3(parameters[3]);   // Change in control pose [i-3]
    //MapConstVector9d    dintrinsics(parameters[1]);
    //MapConstVector6d    dextrinsics(parameters[2]);
    //MapConstVector1d    dtimedelay(parameters[3]);
    
    // Calculate residual from the error states using linearized model
    residual = residual_
               + dcorner_dp0_*dp0
               + dcorner_dp1_*dp1
               + dcorner_dp2_*dp2
               + dcorner_dp3_*dp3;
               //+ dcorner_dintrinsics_ * dintrinsics
               //+ dcorner_dextrinsics_ * dextrinsics
               //+ dcorner_dtimedelay_ * dtimedelay;
    
    // First estimates Jacobians are used as this shown to keep the estimator consistent
    if (jacobians != NULL) {
      if (jacobians[0] != NULL) {
        MapMatrixXxYRowd jac(jacobians[0], num_residuals_, 6);
        jac = dcorner_dp0_;
      }
      if (jacobians[1] != NULL) {
        MapMatrixXxYRowd jac(jacobians[1], num_residuals_, 6);
        jac = dcorner_dp1_;
      }
      if (jacobians[2] != NULL) {
        MapMatrixXxYRowd jac(jacobians[2], num_residuals_, 6);
        jac = dcorner_dp2_;
      }
      if (jacobians[3] != NULL) {
        MapMatrixXxYRowd jac(jacobians[3], num_residuals_, 6);
        jac = dcorner_dp3_;
      }
      //if (jacobians[1] != NULL) {
      //  Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>>
      //      jac(jacobians[1], num_residuals_, 9);
      //  jac = dcorner_dintrinsics_;
      ///}
      //if (jacobians[2] != NULL) {
      //  Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>>
      //      jac(jacobians[2], num_residuals_, 6);
      //  jac = dcorner_dextrinsics_;
      ///}
      //if (jacobians[3] != NULL) {
      //  Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>>
      //      jac(jacobians[3], num_residuals_, 1);
      //  jac = dcorner_dtimedelay_;
      ///}
    }
    
    return true;
  }

  // Add this residual to a problem
  bool AddToProblem(ceres::Problem *problem, ceres::LossFunction* loss_func = nullptr) {
    // Check integrity of data
    if (!p0_ || !p1_ || !p2_ || !p3_) {LOG(ERROR) << "!p0_ || !p1_ || !p2_ || !p3_"; return false;}
    int num_zero_jacobians = 0;
    if (dcorner_dp0_.isZero()) {
      VLOG(2) << "dcorner_dp0_.isZero() ts " << timestamp_ << " p0 ts " << p0_->Timestamp(); num_zero_jacobians++;}
    if (dcorner_dp1_.isZero()) {
      VLOG(2) << "dcorner_dp1_.isZero() ts " << timestamp_ << " p1 ts " << p1_->Timestamp(); num_zero_jacobians++;}
    if (dcorner_dp2_.isZero()) {
      VLOG(2) << "dcorner_dp2_.isZero() ts " << timestamp_ << " p2 ts " << p2_->Timestamp(); num_zero_jacobians++;}
    if (dcorner_dp3_.isZero()) {
      VLOG(2) << "dcorner_dp3_.isZero() ts " << timestamp_ << " p3 ts " << p3_->Timestamp(); num_zero_jacobians++;}
    if (num_zero_jacobians > 1) {
      LOG(ERROR) << "Found more than one zero jacobian for the pose residual. Usually there is only one.";
      LOG(ERROR) << dcorner_dp0_ << "\n" << dcorner_dp1_ << "\n" << dcorner_dp2_ << "\n" << dcorner_dp3_;
      // return false; // Should we return false here? **CHECK**
    }
    
    // Check if this residual is already added to the problem
    //std::vector<ceres::ResidualBlockId> residual_blocks;
    //problem->GetResidualBlocks(&residual_blocks);
    //VLOG(1) << "Problem has " << residual_blocks.size() << " residual blocks.";
    //for (int i=0; i<residual_blocks.size(); i++) {
    //  if (problem->GetCostFunctionForResidualBlock(residual_blocks.at(i)) == this) {
    //    LOG(FATAL) << "this cost function has already been added to this problem!";
    //  }
    ///}
    
    // Add the residual to problem
    ceres::CostFunction* cf = this;
    problem->AddResidualBlock(
      cf, loss_func,
      p0_->error_.data(),           // 6d
      p1_->error_.data(),           // 6d
      p2_->error_.data(),           // 6d
      p3_->error_.data()            // 6d
    );
    
    // Report
    VLOG(2) << "Added pose residual to problem " << timestamp_ << " control points: "
        << p0_->Timestamp() << " " << p1_->Timestamp() << " "
        << p2_->Timestamp() << " " << p3_->Timestamp();
    
    return true;
  }
  
  // Mark p0 as constant on the problem
  bool MarkStartingControlPoseConstant(ceres::Problem* problem) {
    if (problem->HasParameterBlock(p3_->error_.data()))
      problem->SetParameterBlockConstant(p3_->error_.data());    
    return true;
  }
  
  // Mark all states constant on the problem
  bool MarkAllControlPosesConstant(ceres::Problem* problem) {
    if (problem->HasParameterBlock(p0_->error_.data()))
      problem->SetParameterBlockConstant(p0_->error_.data());    
    if (problem->HasParameterBlock(p1_->error_.data()))
      problem->SetParameterBlockConstant(p1_->error_.data());    
    if (problem->HasParameterBlock(p2_->error_.data()))
      problem->SetParameterBlockConstant(p2_->error_.data());    
    if (problem->HasParameterBlock(p3_->error_.data()))
      problem->SetParameterBlockConstant(p3_->error_.data());    
    return true;
  }
  
  int64_t EarliestControlPoseTimestamp() const {
    if (!p3_) return 0;
    return p3_->Timestamp();
  }
  
  // Removes the control pose errors from residual.
  //  This is done at starting to address existing errors in control poses
  bool RemovePoseErrors() {
    residual_ += (-dcorner_dp0_*p0_->error_ - dcorner_dp1_*p1_->error_
                  -dcorner_dp2_*p2_->error_ - dcorner_dp3_*p3_->error_);
    return true;
  }

  // Adds the control pose errors to residual.
  //  This is done before control pose errors are set to zero.
  bool ConsumePoseErrors() {
    residual_ += ( dcorner_dp0_*p0_->error_ + dcorner_dp1_*p1_->error_
                  +dcorner_dp2_*p2_->error_ + dcorner_dp3_*p3_->error_);
    return true;
  }

  // Accessor for the starting target pose estimate
  const PoseState& StartingTargetPose() const {
    return target_pose_est_;
  }
  
  const PoseState& Observation() const {
    return observation_;
  }
  
};  // PoseSpline_CalibrationTargetViewResidual


// Extrinsics calibration between two cameras
//  One reference camera produces poses of a target at wall-timestamps
//  Second camera produces poses of the same target at same wall-timestamps
//    also produces jacobian of the poses wrt time
// We want to guess the relative pose and time delay of the second camera wrt first one
//  Residual gets two pose states one for each camera, checks that they have the same timestamp
//  Jacobian wrt time for second camera is also provided, should not be zero
//  Residual is defined as the difference 


/** Message filters */

typedef std::function<bool(const anantak::SensorMsg&)> SensorMessageFilterType;




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
//
//
///* Kinematic state is pose, velocity and acceleration */
//
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
