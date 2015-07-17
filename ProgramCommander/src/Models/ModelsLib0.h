/**
 *  Loads a monocular camera calibration, set of raw image-names and their timings.
 *  Undistorts each image, tracks features across images using libviso2.
 *  Packs each sparse feature set in a protocol buffer and saves the file.
 *  Potentially we can also add AprilTag detection to this too.
 *
 */

#pragma once

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
#include "Filter/observation.h"
#include "DataQueue/message_file_writer.h"
#include "DataQueue/message_file_reader.h"
#include "DataQueue/message_file_stats.h"
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

/** libviso2 includes */

/** Command line flags */

namespace anantak {

/** Type definitions */
typedef std::vector<anantak::SensorMsg>
    SensorMsgVectorType;
typedef Eigen::Matrix<int64_t, Eigen::Dynamic, 1>
    TimestampVecType;
typedef std::vector<int64_t>
    StdVectorOfTimestampsType;
typedef Eigen::Quaterniond
    QuaternionType;
typedef Eigen::Vector4d
    QuatColVecType;
typedef std::vector<Eigen::Quaterniond>
    QuaternionVectorType;
typedef Eigen::Matrix3d
    RotationMatType;
typedef Eigen::Vector3d
    YawPitchRollType;
typedef Eigen::Matrix<double, Eigen::Dynamic, 3>
    YawPitchRollVectorType;
typedef Eigen::AngleAxisd
    AngleAxisType;
typedef std::vector<AngleAxisType>
    AngleAxisVectorType;
typedef Eigen::Matrix4d
    QuaternionMultMatrixType;
typedef Eigen::Vector3d
    Vector3dType;
typedef std::vector<Vector3dType, Eigen::aligned_allocator<Vector3dType>>
    StdVectorOfVector3dType;
typedef std::vector<RotationMatType, Eigen::aligned_allocator<RotationMatType>>
    StdVectorOfMatrix3dType;
typedef Eigen::Vector2d
    Vector2dType;

/** Global constants */

const double Pi = 3.14159265358979323846;
const double Pi_half = Pi*0.5;
const double Pi_2 = Pi*2.0;
const double RadiansPerDegree = Pi/180.0;
const double DegreesPerRadian = 180.0/Pi;

const double SmallAngularVelocityThreshold = 1.0*RadiansPerDegree; // radians per second

static const double Epsilon = Eigen::NumTraits<double>::epsilon();
static const double OneLessEpsilon = double(1) - Epsilon;

/** File utilities **/

/** Load data from files into a messages vector
 *    IMU data
 *    Sparse Features data
 *    April tags data
 */
bool LoadMsgsFromFile(const std::string& filename, std::vector<anantak::SensorMsg>* storage) {
  anantak::MessageFileReader pb_file_reader;
  if (!pb_file_reader.Open(filename)) {
    LOG(ERROR) << "File " << filename << " could not be opened.";
    return false;
  }
  // Read messages one-by-one
  anantak::SensorMsg msg;
  int n_msgs_in_file = 0;
  // Message is read 'in place'. Message's fields will be written 'over' the existing message. 
  while (pb_file_reader.ReadMessage(&msg)) {
    anantak::SensorMsg msg_copy(msg);
    storage->push_back(msg_copy);
    msg.Clear();
    n_msgs_in_file++;
  }
  //pb_file_reader.Close(); /** no closing is necessary as file is already closed */
  VLOG(1) << "Read " << n_msgs_in_file << " messages from " << filename;  
  return true;
}

/** Write a Eigen matrix to file in CSV format usually for plotting */
const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");
template <typename Derived>
void WriteMatrixToCSVFile(const std::string& filename, const Eigen::DenseBase<Derived>& matrix) {
  std::ofstream file(filename.c_str());
  file << matrix.format(CSVFormat);
  VLOG(1) << "Written file " << filename;
}

/** Interpolation functions */

/** LinearInterpolation represents the intepolated location of a point in another vector of points
 *    index represents the index of the last point in vector
 *    fraction is in range [0,1) that represents distance as fraction of index, index+1 pts in vec
 */
struct LinearInterpolation {
  int32_t index;    /**< index of the last point in reference vector */
  double fraction;  /**< fraction in [0,1) of the point in interval of points index, index+1 */
  LinearInterpolation(): index(-1), fraction(0) {};
  LinearInterpolation(const LinearInterpolation& li): index(li.index), fraction(li.fraction) {};
  LinearInterpolation(int32_t idx, double frac): index(idx), fraction(frac) {};
};
typedef std::vector<LinearInterpolation> LinearInterpVectorType;

template<typename MsgType>
bool InterpolateTimestamps(const std::vector<MsgType>& ref, const std::vector<int64_t>& ts,
    std::vector<LinearInterpolation>* interp) {
  int64_t min_ref = ref.front().timestamp;
  int64_t max_ref = ref.back().timestamp;
  interp->clear();
  int32_t curr_ref_idx = 0;
  for (int i=0; i<ts.size(); i++) {
    LinearInterpolation li;
    if (ts[i]<min_ref || ts[i]> max_ref) {interp->push_back(li); continue;}
    int32_t j = curr_ref_idx;
    bool located = false;
    while (j<ref.size()-1 && !located) {
      if (ref[j].timestamp<=ts[i] && ts[i]<ref[j+1].timestamp) {
        located = true;
        li.index = j;
        li.fraction = double(ts[i] - ref[j].timestamp) /
            double(ref[j+1].timestamp - ref[j].timestamp); //VLOG(1) << " " << li.fraction;
        interp->push_back(li);
      } else {
        j++;
      }
    } // while
    curr_ref_idx = j;
  }
  return true;
}

/* Linear interpolation using circular queue */
template<typename MsgType>
bool InterpolateTimestamps(const anantak::CircularQueue<MsgType>& ref, const std::vector<int64_t>& ts,
    std::vector<LinearInterpolation>* interp) {
  int32_t sz = ref.n_msgs();
  int64_t min_ref = ref.at(0).timestamp;
  int64_t max_ref = ref.at(sz-1).timestamp;
  interp->clear();
  int32_t curr_ref_idx = 0;
  for (int i=0; i<ts.size(); i++) {
    LinearInterpolation li;
    if (ts[i]<min_ref || ts[i]> max_ref) {interp->push_back(li); continue;}
    int32_t j = curr_ref_idx;
    bool located = false;
    while (j<sz-1 && !located) {
      if (ref.at(j).timestamp<=ts[i] && ts[i]<ref.at(j+1).timestamp) {
        located = true;
        li.index = j;
        li.fraction = double(ts[i] - ref.at(j).timestamp) /
            double(ref.at(j+1).timestamp - ref.at(j).timestamp); //VLOG(1) << " " << li.fraction;
        interp->push_back(li);
      } else {
        j++;
      }
    } // while
    curr_ref_idx = j;
  }
  return true;
}

/** Creates LinearInterpolation vector for a ts against given ref
 *  Assumes that both vectors and monotonically increasing. */
bool InterpolateTimestamps(const std::vector<int64_t>& ref, const std::vector<int64_t>& ts,
    LinearInterpVectorType* interp) {
  int64_t min_ref = ref.front();
  int64_t max_ref = ref.back();
  interp->clear();
  int32_t curr_ref_idx = 0;
  for (int i=0; i<ts.size(); i++) {
    LinearInterpolation li;
    if (ts[i]<min_ref || ts[i]> max_ref) {interp->push_back(li); continue;}
    int32_t j = curr_ref_idx;
    bool located = false;
    while (j<ref.size()-1 && !located) {
      if (ref[j]<=ts[i] && ts[i]<ref[j+1]) {
        located = true;
        li.index = j;
        li.fraction = double(ts[i] - ref[j]) / double(ref[j+1] - ref[j]);
        interp->push_back(li);
      } else {
        j++;
      }
    } // while
    curr_ref_idx = j;
  }
  return true;
}

/** Creates LinearInterpolation vector for a ts against given ref with upper limit timestamp.
 *  Assumes that both vectors and monotonically increasing. */
bool InterpolateTimestamps(const std::vector<int64_t>& ref, const std::vector<int64_t>& ts,
    const int64_t& upper_limit, LinearInterpVectorType* interp) {
  int64_t min_ref = ref.front();
  int64_t max_ref = std::min(ref.back(), upper_limit);
  interp->clear();
  int32_t curr_ref_idx = 0;
  for (int i=0; i<ts.size(); i++) {
    LinearInterpolation li;
    if (ts[i]<min_ref || ts[i]> max_ref) {interp->push_back(li); continue;}
    int32_t j = curr_ref_idx;
    bool located = false;
    while (j<ref.size()-1 && !located) {
      if (ref[j]<=ts[i] && ts[i]<ref[j+1]) {
        located = true;
        li.index = j;
        li.fraction = double(ts[i] - ref[j]) / double(ref[j+1] - ref[j]);
        interp->push_back(li);
      } else {
        j++;
      }
    } // while
    curr_ref_idx = j;
  }
  return true;
}

/** Creates LinearInterpolation vector for a ts against given ref, with lower and upper limit ts.
 *  Assumes that both vectors and monotonically increasing. */
bool InterpolateTimestamps(const std::vector<int64_t>& ref, const std::vector<int64_t>& ts,
    const int64_t& lower_limit, const int64_t& upper_limit, LinearInterpVectorType* interp) {
  int64_t min_ref = std::max(ref.front(), lower_limit);
  int64_t max_ref = std::min(ref.back(), upper_limit);
  interp->clear();
  int32_t curr_ref_idx = 0;
  for (int i=0; i<ts.size(); i++) {
    LinearInterpolation li;
    if (ts[i]<min_ref || ts[i]> max_ref) {interp->push_back(li); continue;}
    int32_t j = curr_ref_idx;
    bool located = false;
    while (j<ref.size()-1 && !located) {
      if (ref[j]<=ts[i] && ts[i]<ref[j+1]) {
        located = true;
        li.index = j;
        li.fraction = double(ts[i] - ref[j]) / double(ref[j+1] - ref[j]);
        interp->push_back(li);
      } else {
        j++;
      }
    } // while
    curr_ref_idx = j;
  }
  return true;
}

/** Quaternion functions */

/** NOTES:
 * Rotation Matrix rotates an object in a given coordinate frame.
 * Transformation Matrix rotates coordinate frame. This is transpose of the Rotation Matrix.
 * In a sequence of rotations,
 *  Rotation Matrices pre-multiply
 *  But Transformation Matrces will post multiply. Simply transpose product of rotation matrices.
 * Quaternions can be converted to rotation matrices using Eigen formulas.
 *  This uses standard Hamiltonian notation.
 *  But JPL conventions are different. They use -ij=k that gives left-hand-rule rotations.
 *  JPL rotation matrix would be transpose of the rotmat from standard convention.
 *  Also, product of quaternions under JPL if is p(x)q is same as q(x)p under standard convention
 * In a sequence of quaternion rotations (under any notation)
 *  quaternions pre-multiply if the object is rotating. So p followed by q will come from q(x)p
 *  but if coordinate axis is rotating, they post-multiply. so p followed by q comes from p(x)q
 * We use standard Hamiltonian notation. Under this:
 *  Both Transformation Matrices and their quaternions post multiply, so follow same mult seq.
 *  So we are able to use the standard Eigen functions of conversion from/to rotmat/quat/angleax
 * Transformation Matrix IrC to angle axis using Eigen gives an axis and angle
 *  Axis and angle represent right-hand-rule object rotation of the object C in frame I, or ->
 *  Rotate the object C from its frame by this angle-axis to see how it appears in ref frame
 *  Negative of the angle represents the rotation of frame C to frame I, or ->
 *  Keep the object C fixed and rotate the coordinates -angleaxis to see from ref frame
 * Transformation Matrix from angle axis comes from:
 *  I3 + sin(theta).SkewSymm(k_hat) + (1-cos(theta)).SkewSymm(k_hat)*SkewSymm(k_hat).transpose
 *  This equation is used for all rotation integrals
 * References:
 *   [1] http://www-users.cs.umn.edu/~trawny/Publications/Quaternions_3D.pdf 
 *   [2] https://www.cs.iastate.edu/~cs577/handouts/quaternion.pdf - Section 4.
 */

QuaternionMultMatrixType LeftMultiplicationMatrix(const QuaternionType& q) {
  QuaternionMultMatrixType mat;
  mat <<  q.w(), -q.z(),  q.y(),  q.x(),
          q.z(),  q.w(), -q.x(),  q.y(),
         -q.y(),  q.x(),  q.w(),  q.z(),
         -q.x(), -q.y(), -q.z(),  q.w();
  return mat;
}

QuaternionMultMatrixType RightMultiplicationMatrix(const QuaternionType& q) {
  QuaternionMultMatrixType mat;
  mat <<  q.w(),  q.z(), -q.y(),  q.x(),
         -q.z(),  q.w(),  q.x(),  q.y(),
          q.y(), -q.x(),  q.w(),  q.z(),
         -q.x(), -q.y(), -q.z(),  q.w();
  return mat;
}

template <typename Derived>
QuaternionMultMatrixType LeftMultiplicationMatrix(const Eigen::MatrixBase<Derived>& q) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 4)
  QuaternionMultMatrixType mat;
  mat <<  q(3), -q(2),  q(1),  q(0),
          q(2),  q(3), -q(0),  q(1),
         -q(1),  q(0),  q(3),  q(2),
         -q(0), -q(1), -q(2),  q(3);
  return mat;
}

template <typename Derived>
QuaternionMultMatrixType RightMultiplicationMatrix(const Eigen::MatrixBase<Derived>& q) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 4)
  QuaternionMultMatrixType mat;
  mat <<  q(3),  q(2), -q(1),  q(0),
         -q(2),  q(3),  q(0),  q(1),
          q(1), -q(0),  q(3),  q(2),
         -q(0), -q(1), -q(2),  q(3);
  return mat;
}

template <typename Derived>
Eigen::Matrix3d SkewSymmetricMatrix(const Eigen::MatrixBase<Derived>& v) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3)
  Eigen::Matrix3d mat;
  mat <<     0,  -v[2],   v[1],
          v[2],      0,  -v[0],
         -v[1],   v[0],      0;
  return mat;
}

/** Interpolate two quaternions with constant angular velocity */
QuaternionType InterpolateQuaternionsLinearly(const QuaternionType& q1, const QuaternionType& q2,
      double distance) {
  return q1.slerp(distance, q2);
}

/* AprilTag functions */

inline Eigen::Matrix<double,3,4> AprilTag3dCorners(const double& tag_size = 1.0) {
  Eigen::Matrix<double,3,4> corners;
  corners << -1.0,  1.0,  1.0, -1.0,
             -1.0, -1.0,  1.0,  1.0,
              0.0,  0.0,  0.0,  0.0;
  corners *= tag_size*0.5;
  return corners;
}

struct AprilTagView {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::string tag_id;
  double tag_size;
  int32_t image_idx;
  Eigen::Matrix<double,2,4> image_coords;  // tag corner coordinates in image
  // After initial pose calculation
  double reproj_error;   // reprojection error
  Eigen::Matrix3d TrC;   // 3x3
  QuaternionType  TqC;   // quaternion
  Eigen::Vector3d TpC;   // 3x1
  Eigen::Matrix<double,3,4> Cpf;    // Tag corner 3d coordinates in camera frame
  
  // Constructors
  AprilTagView() {
    SetZero();
  }
  AprilTagView(const AprilTagView& view) {
    tag_id = view.tag_id; tag_size = view.tag_size; image_idx = view.image_idx;
    image_coords = view.image_coords; reproj_error = view.reproj_error;
    TrC = view.TrC; TqC = view.TqC; TpC = view.TpC; Cpf = view.Cpf;
  }
  // Set to zero
  bool SetZero() {
    tag_id = ""; tag_size = 0.; image_idx = 0; image_coords.setZero();
    reproj_error = 0.; TrC.setIdentity(); TqC.setIdentity(); TpC.setZero(); Cpf.setZero();
  }
  // Set from AprilTagMessage
  bool SetFromAprilTagMessage(const int32_t& i_tag, const anantak::AprilTagMessage& apriltag_msg,
      const double& tag_sz, const int32_t& img_idx) {
    tag_id = apriltag_msg.tag_id(i_tag);
    tag_size = tag_sz;
    image_idx = img_idx;
    image_coords <<
        apriltag_msg.u_1(i_tag), apriltag_msg.u_2(i_tag), apriltag_msg.u_3(i_tag), apriltag_msg.u_4(i_tag),
        apriltag_msg.v_1(i_tag), apriltag_msg.v_2(i_tag), apriltag_msg.v_3(i_tag), apriltag_msg.v_4(i_tag);
    return true;
  }
  
  // Calculate tag pose in camera
  template <typename Derived>
  bool CalculateTagPoseInCamera(const Eigen::MatrixBase<Derived>& K,
      const Eigen::MatrixBase<Derived>& K_inv) {
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived,3,3)
    Eigen::Matrix<double,3,4> apriltag_3d_corners = anantak::AprilTag3dCorners(tag_size);
    Eigen::Matrix<double,3,4> corners_2d;
    corners_2d.block<2,4>(0,0) = image_coords;
    corners_2d.block<1,4>(2,0) << 1., 1., 1., 1.;
    corners_2d = K_inv * corners_2d;
    corners_2d.colwise().normalize();
    //VLOG(1) << "    corners_2d = \n" << corners_2d;
    //Eigen::Vector3d test_corner; test_corner << image_coords(0,0)-K(0,2), image_coords(1,0)-K(1,2), K(0,0); test_corner.normalize(); VLOG(1) << "    test corner = " << test_corner;
    // Solve for camera pose wrt AprilTag using opengv's p3p algorithm
    opengv::bearingVectors_t bearing_vecs;
    opengv::points_t points_vec;
    for (int i=0; i<4; i++) {
      bearing_vecs.push_back(corners_2d.col(i));
      points_vec.push_back(apriltag_3d_corners.col(i));
    }
    opengv::absolute_pose::CentralAbsoluteAdapter adapter(bearing_vecs, points_vec);
    opengv::transformations_t pnp_transformations = opengv::absolute_pose::p3p_kneip(adapter);
    //VLOG(1) << "tag = " << apriltag_msg.tag_id(i_tag)
    //    << " n_tfmtns = " << pnp_transformations.size();
    //VLOG(3) << "corners = " << points_vec[0].transpose() << ", " << points_vec[1].transpose()
    //    << ", " << points_vec[2].transpose() << ", " << points_vec[3].transpose();
    Eigen::Vector4d dffs;
    Eigen::Matrix<double,3,16> Cpfs; // corner 3d coordinates in camera frame for 4 guesses
    for (int i=0; i<pnp_transformations.size(); i++) {
      //VLOG(1) << "transformation " << i << "\n" << pnp_transformations[i];
      Eigen::Matrix3d pnp_rotn = pnp_transformations[i].block<3,3>(0,0);
      Eigen::Vector3d pnp_tran = pnp_transformations[i].col(3);
      // Calculate reprojection error
      double total_dff = 0.0;
      for (int j=0; j<4; j++) {
        // TpC, TrC, Tpf. CpT = -CrT*TpC. Cpf = CpT + CrT*Tpf = CrT*(Tpf-TpC)
        Eigen::Vector3d c = pnp_rotn.transpose() * (apriltag_3d_corners.col(j) - pnp_tran);
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
      dffs[i] = total_dff;
    } // for all transformations
    //VLOG(1) << dffs.transpose();
    Eigen::Vector4d::Index min_idx; dffs.minCoeff(&min_idx);
    //VLOG(1) << "Min transformation at " << min_idx;
    reproj_error = dffs[min_idx];
    TrC = pnp_transformations[min_idx].block<3,3>(0,0);
    TpC = pnp_transformations[min_idx].col(3);
    QuaternionType q_TqC(TrC);
    TqC = q_TqC;
    Cpf = Cpfs.block<3,4>(0,4*min_idx);
    VLOG(2) << "Tag "<<tag_id<<" Transform = ("<<reproj_error<<")\n"<<TrC<<"\n"<<TpC;
    return true;
  } // CalculateTagPoseInCamera
  
  // Check if sighting is valid
  bool IsValid() {
    return (TrC.allFinite() && TpC.allFinite() && image_coords.allFinite() && Cpf.allFinite());
  }
};
std::ostream& operator<<(std::ostream& os, const AprilTagView& view) {
  Eigen::Map<const Eigen::Matrix<double,8,1>> xy(view.image_coords.data());
  Eigen::AngleAxisd aa(view.TqC);
  return os << view.tag_id << ": \n   Corners = " << xy.transpose()
      << "\n   TpC = " << view.TpC.transpose() << "\n   TqC = " << aa.axis().transpose()
      << ", " << aa.angle()*DegreesPerRadian << "\n   err = " << view.reproj_error;
}

/* Small angle approximations used with errors in rotations */

// Error to quaternion
template<typename Vec3dType>
Eigen::Quaterniond ErrorAngleAxisToQuaterion(const Eigen::MatrixBase<Vec3dType>& err,
    const double& small_angle_threshold = 1e-6) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Vec3dType, 3)
  Eigen::Vector3d quat_vec = err;
  quat_vec *= 0.5;
  double quat_vec_sq_norm = quat_vec.squaredNorm();
  Eigen::Quaterniond err_quat;
  if (quat_vec_sq_norm < 1.) {
    err_quat.coeffs() << quat_vec[0], quat_vec[1], quat_vec[2], std::sqrt(1.-quat_vec_sq_norm);
  } else {
    double q_w = 1./std::sqrt(1.+quat_vec_sq_norm);
    quat_vec *= q_w;
    err_quat.coeffs() << quat_vec[0], quat_vec[1], quat_vec[2], q_w;
  }
  //double err_angle = err.norm();
  //Eigen::Quaterniond err_quat;
  //if (err_angle < small_angle_threshold) {
  //  err_quat.coeffs() << 0.5*err[0], 0.5*err[1], 0.5*err[2], 1.;  // x,y,z,w
  ///} else {
  //  Eigen::AngleAxisd aa(err_angle, err/err_angle);
  //  err_quat = aa;
  ///}
  //err_quat.normalize();
  return err_quat;
}

// Error to matrix
template<typename Vec3dType>
Eigen::Matrix3d ErrorAngleAxisToMatrix3d(const Eigen::MatrixBase<Vec3dType>& err,
    const double& small_angle_threshold = 1e-8) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Vec3dType, 3)
  Eigen::Quaterniond err_quat = ErrorAngleAxisToQuaterion(err, small_angle_threshold);
  return err_quat.toRotationMatrix();
  //double err_angle = err.norm();
  //Eigen::Matrix3d err_mat;
  //if (err_angle < small_angle_threshold) {
  //  err_mat = Eigen::Matrix3d::Identity();
  ///} else {
  //  Eigen::AngleAxisd aa(err_angle, err/err_angle);
  //  err_mat = aa.toRotationMatrix();
  ///}
  //return err_mat;    
}

// Calculate Inv Sqrt Cov for a square matrix
template<typename MatType, typename VecType>
bool CalculateInverseSqrtMatrix(
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

/* IMU functions */

/* State
 * A state is e.g. IMU state at a reading timestamp, a tag's pose, IMU-to-camera pose etc.
 * States go through a lifecycle:
 *  Allocated - only memory is allocated and cleared, this is unusable as yet
 *  Created - state has been created from a reading
 *  Estimated - state has been estimated populating its estimates, jacobians and noises
 *  ReadyForOptimization - Error states have been created and initial values written
 *  Optimized - Error states have been calculated using an optimizer
 *  ReEstimated - Estimates have been recalculated from optimized error states
 *  ReadyForOptimization - State is ready to be re-optimized N number of times
 *  Marginalized - State has been marginalized and will not be used in optimization again
 *  Allocated - All memory is cleared for reuse.
 */

/* State base class
 * All states are derived classes
 */
class State {
 public:
  // Constructors
  State() : _lifecycle(kAllocated) {}
  
  enum Lifecycle {
    kAllocated, kCreated, kEstimated, kReadyForOptimization,
    kOptimized, kReCalculated, kMarginalized
  } _lifecycle;
  
  virtual ~State() {}    
};

class ScalarState : public State {
 public:
  // Constructors
  ScalarState() : State(), state_(0.), error_(0.), covariance_(0.) {}
  
  // Set to zero
  bool SetZero() {
    state_ = 0.;
    error_ = 0.;
    covariance_ = 0.;
    return true;
  }
  
  bool SetErrorZero() {
    error_ = 0.;
    return true;
  }
  
  bool SetValue(double v) {
    state_ = v;
    return true;
  }
  
  // Is this zero?
  bool IsZero() const {
    return (std::abs(state_)<Epsilon);
  }
  
  virtual ~ScalarState() {}
  
  bool Create(const double& v) {
    state_ = v;
    SetErrorZero();
    covariance_ = 0.;
    return true;
  }
  
  // Recalculate the state from error state after optimization. Set errors to zero.
  bool Recalculate() {
    state_ += error_;
    error_ = 0.;
    return true;
  }
  
  // Return a copy of stored value
  double Value() const {
    return state_;
  }
  
  // State
  double state_;
  // Error
  double error_;
  // Variance
  double covariance_;
  
};  // ScalarState
std::ostream& operator<<(std::ostream& os, const ScalarState& s) {
  return os << s.state_ << " var:" << s.covariance_; 
}

/* Vector3d state
 * This is a 3d vector, e.g. used to estimate gravity in world frame
 */
class Vector3dState : public State {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef Eigen::Map<Eigen::Vector3d> MapVector3dType;
  
  // Constructors
  Vector3dState() : State(), Gp_(state_), dGp_(error_), covariance_(Eigen::Matrix3d::Zero()) {
    SetZero();
  }
  
  // Set to zero
  bool SetZero() {
    MapVector3dType s(state_);
    MapVector3dType e(error_);
    s.setZero();
    e.setZero();
    covariance_.setZero();
    return true;
  }
  
  bool SetErrorZero() {
    MapVector3dType e(error_);
    e.setZero();
    return true;
  }
  
  bool Create(Eigen::Vector3d *v) {
    Gp_ = *v;   // copy vector
    covariance_.setZero();
    SetErrorZero();
    return true;
  }
  
  bool SetFromVector3d(Eigen::Vector3d *v) {
    Gp_ = *v;   // copy vector
    return true;
  }
  
  // Is this zero?
  bool IsZero() const {
    return (Gp_.isZero() && covariance_.isZero());
  }
  
  virtual ~Vector3dState() {}
  
  // Recalculate the state from error state after optimization. Set errors to zero.
  bool Recalculate() {
    Gp_ += dGp_;
    dGp_.setZero();
    return true;
  }
  
  // Helper that sends back copies
  Eigen::Vector3d Position() const {
    return Eigen::Vector3d(Gp_);
  }
  
  // State
  double state_[3];   /**< Gp */
  // Error
  double error_[3];   /**< dGp */
  // Covariance
  Eigen::Matrix3d covariance_;
  
  // Helper maps
  MapVector3dType Gp_;
  MapVector3dType dGp_;
  
}; // Vector3dState

/* Pose3d state
 * This is a quaternion and a position
 */
class Pose3dState : public State {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef Eigen::Map<Eigen::Vector3d> MapVector3dType;
  typedef Eigen::Map<const Eigen::Vector3d> MapConstVector3dType;
  typedef Eigen::Map<Eigen::Vector4d> MapVector4dType;
  typedef Eigen::Map<Eigen::Matrix<double,7,1>> MapVector7dType;
  typedef Eigen::Map<const Eigen::Matrix<double,7,1>> MapConstVector7dType;
  typedef Eigen::Map<Eigen::Matrix<double,6,1>> MapVector6dType;
  typedef Eigen::Matrix<double,6,6> Matrix6dType;
  typedef Eigen::Map<const Eigen::Matrix<double,6,6,Eigen::RowMajor>> MapConstMatrix6dRowType;
  
  // Constructors
  Pose3dState() : State(), LqvG_(state_), GpL_(state_+4), dGaL_(error_), dGpL_(error_+3),
      timestamp_(0), information_(0.) {
    SetZero();
  }

  // Default copy constructor
  Pose3dState(const Pose3dState& r) :
      State(), LqvG_(state_), GpL_(state_+4), dGaL_(error_), dGpL_(error_+3),
      timestamp_(0), information_(0.) {
    timestamp_ = r.timestamp_;
    LqvG_ = r.LqvG_;
    GpL_ = r.GpL_;
    dGaL_ = r.dGaL_;
    dGpL_ = r.dGpL_;
    covariance_ = r.covariance_;
    information_ = r.information_;
  }
  
  // Equal to assignment operator
  Pose3dState& operator= (const Pose3dState& r) {
    timestamp_ = r.timestamp_;
    LqvG_ = r.LqvG_;
    GpL_ = r.GpL_;
    dGaL_ = r.dGaL_;
    dGpL_ = r.dGpL_;
    covariance_ = r.covariance_;
    information_ = r.information_;
  }
  
  // Set to zero
  bool SetZero() {
    MapVector7dType s(state_);
    MapVector6dType e(error_);
    s.setZero(); s[3] = 1.;
    e.setZero();
    timestamp_ = 0;
    information_ = 0.;
    covariance_.setZero();
    return true;
  }
  
  // Set error to zero
  bool SetErrorZero() {
    MapVector6dType e(error_);
    e.setZero();
    return true;
  }
  
  // Is this zero?
  bool IsZero() const {
    MapConstVector3dType LaG(state_);
    return (LaG.isZero() && GpL_.isZero() && covariance_.isZero());
  }
  
  virtual ~Pose3dState() {}
  
  // Create from given values
  bool Create(const Eigen::Quaterniond* q, const Eigen::Vector3d* v) {
    LqvG_ = q->coeffs();
    GpL_ = *v;
    covariance_.setZero();
    SetErrorZero();
    return true;
  }
  
  // Create from given values
  bool Create(const Eigen::Quaterniond* q, const Eigen::Vector3d* v, const int64_t& ts) {
    timestamp_ = ts;
    return Create(q, v);
  }
  
  // Create from message
  bool Create(const anantak::Pose3dStateMessage& msg, const int64_t& ts=0) {
    timestamp_ = ts;
    information_ = 0;
    if (msg.state_size()!=7) {
      LOG(ERROR) << "Message pose state size != 7. Using default." << msg.state_size();
      SetZero();
      return false;
    }
    MapVector7dType s(state_);
    MapConstVector7dType msg_s(msg.state().data());
    s = msg_s;
    SetErrorZero();
    if (msg.covariance_size()!=36) {
      LOG(ERROR) << "Message pose cov size != 36. Using default." << msg.covariance_size();
      covariance_.setZero();
      return false;
    }
    MapConstMatrix6dRowType msg_cov(msg.covariance().data());
    covariance_ = msg_cov;
    return true;
  }
  
  // Set timestamp
  bool SetTimestamp(const int64_t& ts) {
    timestamp_ = ts;
    return true;
  }
  
  // Recalculate the state from error state after optimization. Set errors to zero.
  bool Recalculate() {
    Eigen::Quaterniond LqG(LqvG_.data()); // x,y,z,w
    Eigen::Quaterniond dGqL = anantak::ErrorAngleAxisToQuaterion(dGaL_);
    LqG *= dGqL;  // assuming LqvG_ (and hence LqG) are already normalized
    LqvG_ = LqG.coeffs(); // x,y,z,w
    GpL_ += dGpL_;
    dGaL_.setZero();
    dGpL_.setZero();
    return true;
  }
  
  // Return copies of state values
  Eigen::Quaterniond Quaternion() const {
    return Eigen::Quaterniond(LqvG_.data()); // x,y,z,w
  }
  Eigen::Vector3d Position() const {
    return Eigen::Vector3d(GpL_);
  }
  double Timestamp() const {
    return timestamp_;
  }
  
  // G = global or reference frame, L = local frame
  
  // State
  double state_[7];   /**< LqG, GpL */    
  // Error
  double error_[6];   /**< dGqL, dGpL */
  // Covariance
  Matrix6dType covariance_;
  // Information - single dimensionless measure holding information on the pose
  double information_;
  
  // Timestamp  -  not required, but could be needed in some cases
  int64_t timestamp_;
  
  // Helper maps
  MapVector4dType LqvG_; // quaternion as a vector x,y,z,w
  MapVector3dType GpL_;
  MapVector3dType dGaL_; // angleaxis as a vector
  MapVector3dType dGpL_;
  
}; // Pose3dState
std::ostream& operator<<(std::ostream& os, const Pose3dState& pose) {
  bool full = false;
  Eigen::AngleAxisd aa(pose.Quaternion().conjugate());
  if (full) 
    return os << "   Position = " << pose.Position().transpose()
        << "\n   Rotation = " << aa.axis().transpose()
        << ", " << aa.angle()*DegreesPerRadian << "\n   CovMat = \n"
        << pose.covariance_;
  else 
    return os << "Posn = " << pose.Position().transpose()
        << ", Rotn = " << aa.axis().transpose() << ", " << aa.angle()*DegreesPerRadian
        << ", Info = " << pose.information_;
}


/* Velocity3d state
 * This is a angular and linear velocity
 */
class Velocity3dState : public State {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef Eigen::Map<Eigen::Vector3d> MapVector3dType;
  typedef Eigen::Map<const Eigen::Vector3d> MapConstVector3dType;
  typedef Eigen::Map<Eigen::Matrix<double,6,1>> MapVector6dType;
  typedef Eigen::Matrix<double,6,6> Matrix6dType;
  
  // Constructors
  Velocity3dState() : State(), Gw_(state_), Gv_(state_+3), dGw_(error_), dGv_(error_+3),
      timestamp_(0), information_(0.) {
    SetZero();
  }
  
  // Set to zero
  bool SetZero() {
    MapVector6dType s(state_);
    MapVector6dType e(error_);
    s.setZero();
    e.setZero();
    timestamp_ = 0;
    information_ = 0.;
    covariance_.setZero();
    return true;
  }
  
  // Set error to zero
  bool SetErrorZero() {
    MapVector6dType e(error_);
    e.setZero();
    return true;
  }
  
  // Is this zero?
  bool IsZero() const {
    return (Gw_.isZero() && Gv_.isZero() && covariance_.isZero());
  }
  
  virtual ~Velocity3dState() {}
  
  // Create from given values
  bool Create(const Eigen::Vector3d* w, const Eigen::Vector3d* v) {
    Gw_ = *w;
    Gv_ = *v;
    covariance_.setZero();
    information_ = 0.;
    SetErrorZero();
    return true;
  }
  
  // Create from given values
  bool Create(const Eigen::Vector3d* w, const Eigen::Vector3d* v, const int64_t& ts) {
    timestamp_ = ts;
    return Create(w, v);
  }
  
  // Set timestamp
  bool SetTimestamp(const int64_t& ts) {
    timestamp_ = ts;
    return true;
  }
  
  // Recalculate the state from error state after optimization. Set errors to zero.
  bool Recalculate() {
    Gw_ += dGw_;
    Gv_ += dGv_;
    dGw_.setZero();
    dGv_.setZero();
    return true;
  }
  
  // Return copies of state values
  Eigen::Vector3d Omega() const {
    return Eigen::Vector3d(Gw_);
  }
  Eigen::Vector3d Velocity() const {
    return Eigen::Vector3d(Gv_);
  }
  double Timestamp() const {
    return timestamp_;
  }
  
  // G = global or reference frame, L = local frame
  
  // State
  double state_[6];   /**< Gw, Gv */    
  // Error
  double error_[6];   /**< dGw, dGv */
  // Covariance
  Matrix6dType covariance_;
  // Information - single dimensionless measure holding information on the pose
  double information_;
  
  // Timestamp  -  not required, but needed many times
  int64_t timestamp_;
  
  // Helper maps
  MapVector3dType Gw_;  // Angular velocity vector
  MapVector3dType Gv_;  // Linear velocity vector
  MapVector3dType dGw_; 
  MapVector3dType dGv_;
  
}; // Velocity3dState
std::ostream& operator<<(std::ostream& os, const Velocity3dState& velo) {
  bool full = false;
  if (full) 
    return os << "   Velocity = " << velo.Velocity().transpose()
        << "\n   Omega = " << velo.Omega().transpose()
        << "\n   CovMat = \n" << velo.covariance_;
  else 
    return os << "   Velocity = " << velo.Velocity().transpose()
        << "   Omega = " << velo.Omega().transpose()
        << "   Info = " << velo.information_;
}


/* Unit vector 3d state
 * Represents a plane though the origin by a unit vector normal to the plane.
 * Error is a rotation of the unit vector that only has two degrees of freedom.
 * Maths here comes from Robotics notebook 4 ppg 93-94
 */
/*class UnitVector3dState : public State {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef Eigen::Map<Eigen::Vector3d> MapVector3dType;
  typedef Eigen::Map<const Eigen::Vector3d> MapConstVector3dType;

  // Constructors
  UnitVector3dState() : State(), Gn_(state_), dGan_(error_), timestamp_(0) {
    SetZero();
  }
  
  // Set to zero
  bool SetZero() {
    Gn_.setZero();
    dGan_.setZero();
    timestamp_ = 0;
    covariance_sqrt_.setZero();
    return true;
  }
  
  // Set error to zero
  bool SetErrorZero() {
    dGan_.setZero();
    return true;
  }
  
  // Is this zero?
  bool IsZero() const {
    return (Gn_.isZero() && dGan_.isZero() && covariance_sqrt_.isZero());
  }
  
  virtual ~UnitVector3dState() {}
  
  // Create from given values
  bool Create(const Eigen::Vector3d* v) {
    Gn_ = *v;         // copy the provided vector
    Gn_.normalize();  // this is a crucial step
    covariance_sqrt_.setZero();
    SetErrorZero();
    return true;
  }
  
  // Create from given values
  bool Create(const Eigen::Vector3d* v, const int64_t& ts) {
    timestamp_ = ts;
    return Create(v);
  }
  
  // Set timestamp
  bool SetTimestamp(const int64_t& ts) {
    timestamp_ = ts;
    return true;
  }
  
  // Recalculate the state from error state after optimization. Set errors to zero.
  bool Recalculate() {
    Eigen::Matrix3d rotn = anantak::ErrorAngleAxisToMatrix3d(dGan_);
    Gn_ = rotn * Gn_;
    dGan_.setZero();
    return true;
  }
  
  // Return copies of state values
  Eigen::Vector3d Position() const {
    return Eigen::Vector3d(Gn_);
  }
  double Timestamp() const {
    return timestamp_;
  }
  
  // G = global or reference frame, n = normal to the plane
  
  // State
  double state_[3];   // Gn
  // Error
  double error_[3];   // dGqn
  // Covariance
  Eigen::Matrix3d covariance_sqrt_;
  
  // Timestamp  -  not required, but could be needed in some cases
  int64_t timestamp_;
  
  // Helper maps
  MapVector3dType Gn_;
  MapVector3dType dGan_; // angleaxis as a vector
  
}; // UnitVector3dState*/

/* Unit vector 3d state
 * Represents a plane though the origin by a unit vector normal to the plane.
 * Error is a rotation of the unit vector that only has two degrees of freedom.
 * Maths here comes from Robotics notebook 4 ppg 93-94
 */
class UnitVector3dState : public State {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef Eigen::Map<Eigen::Vector3d> MapVector3dType;
  typedef Eigen::Map<Eigen::Vector2d> MapVector2dType;
  typedef Eigen::Matrix<double,3,2> Matrix3x2Type;
  
  // Constructors
  UnitVector3dState() : State(), Gn_(state_), dGn_(error_), timestamp_(0) {
    SetZero();
  }
  
  // Set to zero
  bool SetZero() {
    Gn_.setZero();
    dGn_.setZero();
    Gn_dGn_.setZero();
    timestamp_ = 0;
    covariance_sqrt_.setZero();
    return true;
  }
  
  // Set error to zero
  bool SetErrorZero() {
    dGn_.setZero();
    return true;
  }
  
  // Is this zero?
  bool IsZero() const {
    return (Gn_.isZero() && dGn_.isZero());  // Just these two are enough for this decision
  }
  
  virtual ~UnitVector3dState() {}
  
  /*// Calculate Jacobian wrt derivative state
  bool CalculateJacobian() {
    double alpha_;          
    double theta_;          
    double xy_proj = std::sqrt(Gn_[0]*Gn_[0] + Gn_[1]*Gn_[1]);
    double alpha_ = std::atan2(Gn_[2], xy_proj); // Angle of vector wrt XY plane.
    double theta_ = std::atan2(Gn_[1], Gn_[0]);  // Angle of XY plane projection plane with X axis.
    double s1 = std::sin(alpha_);
    double c1 = std::cos(alpha_);
    double s2 = std::sin(theta_);
    double c2 = std::cos(theta_);
    Gn_dGn_ << -s1*c2, -c1*s2,  -s1*s2, c1*c2,  c1, 0.;   // Starting Jacobian      
  }*/
  
  // Calculate Jacobian wrt derivative state
  bool CalculateJacobian() {
    Eigen::Matrix3d skew_Gn = anantak::SkewSymmetricMatrix(Gn_);
    Gn_dGn_ = skew_Gn.block<3,2>(0,0);
  }
  
  // Create from given values
  bool Create(const Eigen::Vector3d* v) {
    Gn_ = *v;             // copy the provided vector
    Gn_.normalize();      // this is a crucial step
    CalculateJacobian();
    covariance_sqrt_.setZero();
    SetErrorZero();
    return true;
  }
  
  // Create from given values
  bool Create(const Eigen::Vector3d* v, const int64_t& ts) {
    timestamp_ = ts;
    return Create(v);
  }
  
  // Set timestamp
  bool SetTimestamp(const int64_t& ts) {
    timestamp_ = ts;
    return true;
  }
  
  // Recalculate the state from error state after optimization. Set errors to zero.
  bool Recalculate() {
    Gn_ += Gn_dGn_ * dGn_;    // Usually use the starting Jacobian. User could recalculate it.
    Gn_.normalize();          // this is a crucial step
    dGn_.setZero();
    return true;
  }
  
  // Return copies of state values
  Eigen::Vector3d Position() const {
    return Eigen::Vector3d(Gn_);
  }
  double Timestamp() const {
    return timestamp_;
  }
  
  // G = global or reference frame, n = normal to the plane
  
  // State
  double state_[3];   // unit vector - has a magnitude of 1. So has 2 degrees of freedom.
  // Error
  double error_[2];   // 2-vector that characterises the error. Only two degrees of freedom exist.
  // Covariance
  Eigen::Matrix2d covariance_sqrt_;   // Covariance of the error, dGn_
  
  // Angles for the vector
  Matrix3x2Type Gn_dGn_;  // Gn_ += Gn_dGn_ * dGn_
  
  // Timestamp  -  not required, but could be needed in some cases
  int64_t timestamp_;
  
  // Helper maps
  MapVector3dType Gn_;    // unit vector
  MapVector2dType dGn_;   // infinitesimal change in unit vector
  
}; // UnitVector3dState


/* Plane 3d state
 * Implements a state representing a 3d plane by a unit normal and a distance scalar.
 * This has three degrees of freedom: 2 for unit normal and 1 for distance.
 */
class Plane3dState : public State {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  // Constructor
  Plane3dState() : State(), normal_(), distance_() {}
  
  // Set state to zero
  bool SetZero() {
    normal_.SetZero();
    distance_.SetZero();
    return true;
  }
  
  // Set error to zero
  bool SetErrorZero() {
    normal_.SetErrorZero();
    distance_.SetErrorZero();
    return true;
  }
  
  // Is this zero?
  bool IsZero() {
    return (normal_.IsZero() && distance_.IsZero());
  }
  
  virtual ~Plane3dState() {}
  
  // Create from a vector and a value
  bool Create(const Eigen::Vector3d* v, const double& value) {
    normal_.Create(v);
    distance_.SetValue(value);
    return true;
  }
  
  // Create from a vector, timestamp and a value
  bool Create(const Eigen::Vector3d* v, const double& value, const int64_t& ts) {
    normal_.Create(v, ts);
    distance_.SetValue(value);
    return true;
  }
  
  // Recalculate
  bool Recalculate() {
    normal_.Recalculate();
    distance_.Recalculate();
    return true;
  }
  
  // A plane is comprised of a unit vector and a scalar - this is a composition of two states
  anantak::UnitVector3dState  normal_;
  anantak::ScalarState        distance_;    // could be negative/positive
  
};  // Plane3dState


/* Static April Tag state
 * April tag is a square tag with a unique id. This could be any other square tag.
 */
class StaticAprilTagState : public State {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  // Constructor
  StaticAprilTagState() : State(), tag_id_(16, ' '), pose_(), size_() {}
  
  bool SetZero() {
    tag_id_ = std::string(16, ' ');
    pose_.SetZero();
    size_.SetZero();
    return true;
  }
  
  // Set error to zero
  bool SetErrorZero() {
    pose_.SetErrorZero();
    size_.SetErrorZero();
    return true;
  }
  
  // Is this zero?
  bool IsZero() const {
    return (pose_.IsZero() && size_.IsZero());
  }
  
  virtual ~StaticAprilTagState() {}
  
  // Set from Vector3d and Quaternion
  bool Create(const std::string* id, const Eigen::Quaterniond* q, const Eigen::Vector3d* v,
      const double* s) {
    // copy all provided values
    tag_id_ = *id; 
    pose_.Create(q, v);
    size_.Create(*s);
    return true;
  }
  
  bool Create(const anantak::StaticApriltagStateMessage& msg) {
    tag_id_ = msg.tag().tag_id(); 
    pose_.Create(msg.pose());
    size_.Create(msg.tag().size());
    size_.covariance_ = msg.tag().covariance();
    return true;    
  }
  
  bool Recalculate() {
    pose_.Recalculate();
    size_.Recalculate();
    return true;
  }
  
  // Static tag elements
  std::string tag_id_;
  anantak::Pose3dState pose_;
  anantak::ScalarState size_;
  
};  // StaticAprilTagState


/* Camera calibration matrix state - distortion free
 * No radial or tangential distortion terms are assumed
 */
class CameraIntrinsicsState : public State {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef Eigen::Map<Eigen::Vector4d> MapVector4dType;
  typedef Eigen::Map<const Eigen::Vector4d> MapConstVector4dType;
  typedef Eigen::Matrix<double,4,4,Eigen::RowMajor> Matrix4dRowType;
  typedef Eigen::Map<Eigen::Matrix<double,4,4,Eigen::RowMajor>> MapMatrix4dType;
  typedef Eigen::Map<Eigen::Matrix<double,6,6,Eigen::RowMajor>> MapMatrix6dType;
  typedef Eigen::Map<Eigen::Matrix<double,8,8,Eigen::RowMajor>> MapMatrix8dType;
  typedef Eigen::Map<const Eigen::Matrix<double,4,4,Eigen::RowMajor>> MapConstMatrix4dType;
  typedef Eigen::Map<const Eigen::Matrix<double,6,6,Eigen::RowMajor>> MapConstMatrix6dType;
  typedef Eigen::Map<const Eigen::Matrix<double,8,8,Eigen::RowMajor>> MapConstMatrix8dType;
  
  // Constructors
  CameraIntrinsicsState() : State() {
    SetZero();
  }
  
  // Set to zero
  bool SetZero() {
    MapVector4dType s(state_);
    MapVector4dType e(error_);
    s.setZero();
    e.setZero();
    covariance_.setZero();
    K_.setZero();
    K_inv_.setZero();
    return true;
  }
  
  // Set error to zero
  bool SetErrorZero() {
    MapVector4dType e(error_);
    e.setZero();
    return true;
  }
  
  // Is this zero?
  bool IsZero() const {
    MapConstVector4dType s(state_);
    MapConstVector4dType e(error_);
    return (s.isZero() && e.isZero());
  }
  
  virtual ~CameraIntrinsicsState() {}
  
  // Create from a camera matrix
  bool Create(const Eigen::Matrix3d *K) {
    K_ = *K;
    state_[0] = K_(0,0);
    state_[1] = K_(1,1);
    state_[2] = K_(0,2);
    state_[3] = K_(1,2);
    SetErrorZero();
    covariance_.setZero();
    K_inv_ = K_.inverse();
    return true;
  }
  
  // Create from four given values
  bool Create(const double& fx, const double& fy, const double& cx, const double& cy,
      bool reset_covariance = true) {
    state_[0] = fx;
    state_[1] = fy;
    state_[2] = cx;
    state_[3] = cy;
    SetErrorZero();
    if (reset_covariance) covariance_.setZero();
    K_ << state_[0], 0., state_[2], 0., state_[1], state_[3], 0., 0., 1.;
    K_inv_ = K_.inverse();
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
    covariance_ = variance.asDiagonal();
    VLOG(1) << "Camera intrinsics from angle, size = " << angle_of_view << " " << width << " "
        << height << " are fx, fy, cx, cy = " << fx << " " << fy << " " << cx << " " << cy
        << " Covariance = \n" << covariance_;
    return Create(fx, fy, cx, cy, false);
  }
  
  // Create using CameraIntrinsicsState - note angles are in degrees
  bool Create(const anantak::CameraIntrinsicsInitMessage& msg) {
    return Create(msg.angle_of_view(), msg.image_size(0), msg.image_size(1),
                  msg.angle_of_view_stdev(), msg.center_stdev());
  }
  
  // Create using a camera intrisics message
  bool Create(const anantak::CameraIntrinsicsStateMessage& intrinsics_msg) {
    // Set state
    state_[0] = intrinsics_msg.pinhole(0);
    state_[1] = intrinsics_msg.pinhole(1);
    state_[2] = intrinsics_msg.pinhole(2);
    state_[3] = intrinsics_msg.pinhole(3);
    // Set error
    SetErrorZero();
    // Set covariance
    int32_t cov_size = intrinsics_msg.covariance_size();
    if (cov_size == 0) {
      covariance_.setZero();
    } else if (cov_size == 16) {
      const MapConstMatrix4dType cov(intrinsics_msg.covariance().data());
      covariance_ = cov;
    } else if (cov_size == 36) {
      const MapConstMatrix4dType cov(intrinsics_msg.covariance().data());
      covariance_ = cov.block<4,4>(0,0);
    } else if (cov_size == 64) {
      const MapConstMatrix4dType cov(intrinsics_msg.covariance().data());
      covariance_ = cov.block<4,4>(0,0);
    } else {
      LOG(ERROR) << "Covariance field in camera intrinsics state message has wierd size "
          << cov_size << ". Expected 4x4, 6x6 or 8x8. Setting covariance to 0";
      covariance_.setZero();
    }
    // Set camera matrices
    K_ << state_[0], 0., state_[2], 0., state_[1], state_[3], 0., 0., 1.;
    K_inv_ = K_.inverse();    
    return true;
  }
  
  // Set camera intrinsics on a state message
  bool SetCameraIntrinsics(anantak::CameraIntrinsicsStateMessage* state_msg) const {
    state_msg->clear_pinhole();
    state_msg->add_pinhole(state_[0]);
    state_msg->add_pinhole(state_[1]);
    state_msg->add_pinhole(state_[2]);
    state_msg->add_pinhole(state_[3]);    
    return true;
  }
  
  // Set camera intrinsics on a state message
  bool SetCovariance(anantak::CameraIntrinsicsStateMessage* state_msg) const {
    state_msg->clear_covariance();
    const double* cov_data = covariance_.data();
    for (int i=0; i<16; i++) {
      state_msg->add_covariance(*(cov_data+i));
    }
    return true;
  }
  
  // Recalculate the state from error state after optimization. Set errors to zero.
  bool Recalculate() {
    MapVector4dType s(state_);
    MapVector4dType e(error_);
    s += e;
    e.setZero();
    K_ << state_[0], 0., state_[2], 0., state_[1], state_[3], 0., 0., 1.;
    K_inv_ = K_.inverse();
    return true;
  }
  
  // Return a copy of the camera matrix from this state
  Eigen::Matrix3d CameraMatrix() const {
    Eigen::Matrix3d camera_matrix;
    camera_matrix << state_[0], 0., state_[2], 0., state_[1], state_[3], 0., 0., 1.;
    return camera_matrix;
  }
  
  const Eigen::Matrix3d K() const {
    return K_;
  }
  
  const Eigen::Matrix3d K_inv() const {
    return K_inv_;
  }
  
  // Helper function to show results without changing state
  std::string ToString(bool detailed=true) const {
    MapConstVector4dType s(state_);
    MapConstVector4dType e(error_);
    Eigen::Vector4d se = s + e;
    std::string str = "fx " + std::to_string(se[0]) + ", fy " + std::to_string(se[1]) +
                      ", cx " + std::to_string(se[2]) + ", cy " + std::to_string(se[3]);
    if (detailed) {
      std::stringstream ss;
      ss << covariance_.format(Eigen::IOFormat());
      str += ", covmat = \n"+ss.str();
    }
    return str;
  }
  
  // Calculate covariance matrix from the ceres problem object
  bool CalculateCovariance(ceres::Problem* problem) {
    // Check if this intrisics object is a parameter block in this problem
    if (!problem->HasParameterBlock(error_)) {
      LOG(ERROR) << "This camera intrinsics object is not a parameer in this problem. Quit.";
      return false;
    }
    
    // Setup covariance calculation
    ceres::Covariance::Options covariance_options;
    ceres::Covariance covariance(covariance_options);
    std::vector<std::pair<const double*, const double*>> covariance_blocks;
    covariance_blocks.push_back(std::make_pair(error_, error_));
    
    // Compute the covariance matricess
    if (!covariance.Compute(covariance_blocks, problem)) {
      LOG(ERROR) << "Could not compute covariance matrices";
      return false;
    }
    
    // Get the calculated covariance. Returned matrix is row major order
    Matrix4dRowType covmat;
    covariance.GetCovarianceBlock(error_, error_, covmat.data());
    covariance_ = covmat;
    
    return true;
  }
  
  // State
  double state_[4];   /**< fx, fy, cx, cy */
  // Error
  double error_[4];   /**< dfx, dfy, dcx, dcy */
  // Covariance
  Eigen::Matrix4d covariance_;
  
  // Camera matrix storage - helps to avoid recalculations outside
  Eigen::Matrix3d K_;
  Eigen::Matrix3d K_inv_;
  
  // Helper matrices
  
}; // CameraIntrinsicsState

/* Camera distortion coefficients state
 * Three radial terms and two tangential terms, same as default OpenCV model
 */
class CameraDistortionState5 : public State {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  // State
  double state_[5];   /**< k1, k2, k3, t1, t2; Why does OpenCV do it as k1 k2 t1 t2 k3*/
  // Error
  double error_[5];   /**< dk1, dk2, dk3, dt1, dt2 */
  // Covariance
  Eigen::Matrix<double,5,5> covariance_;
  
  
};  // CameraDistortionState4


/* Camera state
 * Composition of camera pinhole model, distortion state and extrinsics
 */
class CameraState : public State {
 public:
  CameraIntrinsicsState   pinhole_;
  CameraDistortionState5  distortion_;
  Pose3dState             extrinsics_;
  
};  // CameraState


/* Update Pose3dState using covariances
 * Do a Kalman update to a Pose3d using another reading of the pose using the covariances */
bool UpdatePose3dUsingCovariances(const anantak::Pose3dState* measurement,
    anantak::Pose3dState* pose) {
  typedef Eigen::Matrix<double,6,6> Matrix6dType;
  typedef Eigen::Matrix<double,6,1> Vector6dType;
  typedef Eigen::Map<Eigen::Matrix<double,6,1>> MapVector6dType;
  
  // Basic checks
  if (!measurement->covariance_.allFinite()) {
    LOG(ERROR) << "Measurement covariance is not finite. Skipping the reading";
    return false;
  }
  
  // Gdq = LqG^-1 * LqG_m
  Eigen::Quaterniond err_quat = pose->Quaternion().conjugate() * measurement->Quaternion();
  Eigen::Vector3d err_rotn = 2.*err_quat.vec();   // small angle approximation
  
  // Gdp = GpL_m - GpL
  Eigen::Vector3d err_posn = measurement->GpL_ - pose->GpL_;
  
  // Residual vector
  Vector6dType delta_pose;
  delta_pose.block<3,1>(0,0) = err_rotn;
  delta_pose.block<3,1>(3,0) = err_posn;
  
  // Cov(Gdq, Gdp) = Cov(pose) + Cov(measurement)
  Matrix6dType S = pose->covariance_ + measurement->covariance_;
  Matrix6dType S_inv = S.inverse();   // most expensive step here
  
  // Check
  if (!S_inv.allFinite()) {
    LOG(ERROR) << "Residual covariance matrix is not finite. Skipping reading";
    return false;
  }
  
  // Kalman gain K = P*S^-1
  Matrix6dType K = pose->covariance_ * S_inv;
  
  // Changes to be applied to pose
  delta_pose = K * delta_pose;
  
  // Update the pose
  MapVector6dType pose_error(pose->error_);
  pose_error = delta_pose;
  pose->Recalculate();
  Matrix6dType I_K = Matrix6dType::Identity() - K;
  pose->covariance_ = I_K*pose->covariance_*I_K.transpose() + K*measurement->covariance_*K.transpose();
  
} // UpdatePose3dState

/* Update Pose3dState using information - expensive operation as it uses the eigen solver
 *    example of info: new_info = 1./(std::max(1.,view.reproj_error)*view.TpC[2]); */
bool UpdatePose3dUsingInformation(const anantak::Pose3dState* measurement,
    anantak::Pose3dState* pose) {
  if (std::isnan(measurement->information_)) return false;
  if (measurement->information_ < Epsilon) return false;
  if (!measurement->GpL_.allFinite()) return false;
  if (!measurement->LqvG_.allFinite()) return false;
  if (std::isnan(pose->information_)) {*pose=*measurement; return false;}
  if (pose->information_ < Epsilon) {*pose=*measurement; return false;}
  if (!pose->GpL_.allFinite()) {*pose=*measurement; return false;}
  if (!pose->LqvG_.allFinite()) {*pose=*measurement; return false;}
  double i0 = measurement->information_;
  double i1 = pose->information_ + i0;
  double info_by_i1 = pose->information_/i1;
  double i0_by_i1 = i0/i1;
  pose->GpL_ = info_by_i1*pose->GpL_ + i0_by_i1*measurement->GpL_;
  Eigen::Matrix4d q_mat  = pose->LqvG_ * pose->LqvG_.transpose();
  Eigen::Matrix4d q_mat0 = measurement->LqvG_ * measurement->LqvG_.transpose();
  Eigen::Matrix4d rotn_q_mat = info_by_i1*q_mat + i0_by_i1*q_mat0;
  Eigen::EigenSolver<Eigen::Matrix4d> eigen_solver(rotn_q_mat);
  Eigen::Vector4d eval_real = eigen_solver.eigenvalues().real();
  Eigen::Vector4d::Index max_idx; eval_real.maxCoeff(&max_idx);
  Eigen::Vector4d evec_real = eigen_solver.eigenvectors().col(max_idx).real();
  pose->LqvG_ = evec_real;
  pose->information_ = i1;
  return true;
}

/* Add two Pose3d using information. c = a + b
 * First pose is L1qG, GpL1. Second pose is L2qL1, L1pL2. We need L2qG, GpL2
 * L2qG = L2qL1 * L1qG; GpL2 = GpL1 + G.L1pL2 = GpL1 + GrL1*L1pL2 */
bool AddPose3dUsingInformation(const anantak::Pose3dState* a, const anantak::Pose3dState* b,
    anantak::Pose3dState* c) {
  if (std::isnan(a->information_) || std::isnan(b->information_)) return false;
  if (a->information_ < 0. || b->information_ < 0.) return false;
  if (!a->GpL_.allFinite() || !a->LqvG_.allFinite()) return false;
  if (!b->GpL_.allFinite() || !b->LqvG_.allFinite()) return false;
  
  // Calculate pose a + pose b
  Eigen::Quaterniond L1qG(a->Quaternion());
  Eigen::Matrix3d GrL1(L1qG.conjugate());
  Eigen::Quaterniond L2qG = b->Quaternion() * L1qG;
  c->GpL_ = a->GpL_ + GrL1*b->GpL_;
  c->LqvG_ = L2qG.coeffs();
  
  // Calculate information of combined pose
  if (a->information_ == 0. || b->information_ == 0.) {
    c->information_ = 0.;
  } else {
    c->information_ = 1./(1./a->information_ + 1./b->information_);
  }
  return true;
}

/* Diff two Pose3d using information. c = a - b
 * First pose is L1qG, GpL1. Second pose is L2qG, GpL2. We need L1qL2, L2pL1
 * L1qL2 = L1qG * GqL2; L2pL1 = L2pG + L2.GpL1 = -L2rG*GpL2 + L2rG*GpL1 = L2rG*(GpL1 - GpL2) */
bool DiffPose3dUsingInformation(const anantak::Pose3dState* a, const anantak::Pose3dState* b,
    anantak::Pose3dState* c) {
  if (std::isnan(a->information_) || std::isnan(b->information_)) return false;
  if (a->information_ < 0. || b->information_ < 0.) return false;
  if (!a->GpL_.allFinite() || !a->LqvG_.allFinite()) return false;
  if (!b->GpL_.allFinite() || !b->LqvG_.allFinite()) return false;
  
  // Calculate pose a + pose b
  Eigen::Quaterniond GqL2(b->Quaternion().conjugate());
  Eigen::Matrix3d GrL2(GqL2);
  Eigen::Quaterniond L1qL2 = a->Quaternion() * GqL2;
  c->GpL_ = GrL2.transpose()*(a->GpL_ - b->GpL_);
  c->LqvG_ = L1qL2.coeffs();
  
  // Calculate information of combined pose
  if (a->information_ == 0. || b->information_ == 0.) {
    c->information_ = 0.;
  } else {
    c->information_ = 1./(1./a->information_ + 1./b->information_);
  }
  return true;
}

/* Invert Pose3d using information. c = -a
 * Pose is LqG, GpL. We need GqL, -LrG*GpL */
bool InvertPose3dUsingInformation(const anantak::Pose3dState* a, anantak::Pose3dState* c) {
  if (std::isnan(a->information_)) return false;
  if (a->information_ < 0.) return false;
  if (!a->GpL_.allFinite() || !a->LqvG_.allFinite()) return false;
  
  // Calculate inverse of pose a
  Eigen::Quaterniond GqL = a->Quaternion().conjugate();
  Eigen::Matrix3d GrL(GqL);
  c->GpL_ = -GrL.transpose()*a->GpL_;
  c->LqvG_ = GqL.coeffs();
  
  // No change in information
  c->information_ = a->information_;
  
  return true;
}


/* Interpolate two Pose3d using information
 * Not sure how to address the effect of time difference */
bool InterpolatePose3dUsingInformation(const anantak::Pose3dState* a, const anantak::Pose3dState* b,
    anantak::Pose3dState* c) {
  if (std::isnan(a->information_) || std::isnan(b->information_)) return false;
  if (a->information_ < 0. || b->information_ < 0.) return false;
  if (!a->GpL_.allFinite() || !a->LqvG_.allFinite()) return false;
  if (!b->GpL_.allFinite() || !b->LqvG_.allFinite()) return false;
  if (a->timestamp_ == 0 || b->timestamp_ == 0 || c->timestamp_ == 0) return false;
  if (a->timestamp_ >= b->timestamp_) return false;
  if (c->timestamp_ > b->timestamp_) return false;
  if (c->timestamp_ < a->timestamp_) return false;
  
  // Interpolate pose linearly
  double frac = double(c->timestamp_ - a->timestamp_) / double(b->timestamp_ - a->timestamp_);
  double frac1 = 1.-frac;
  Eigen::Quaterniond q1(a->Quaternion());
  q1.slerp(frac, b->Quaternion());
  c->LqvG_ = q1.coeffs();
  c->GpL_ = frac1*a->GpL_ + frac*b->GpL_;
  
  // Interpolate information
  c->information_ = frac1*a->information_ + frac*b->information_;
  
  return true;
}


/* CameraIntrinsicsNormalPrior
 * Implements the residual for normal camera intrinsics prior */
class CameraIntrinsicsNormalPrior : public ceres::SizedCostFunction<4, 4> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  // Type declarations
  typedef Eigen::Map<Eigen::Matrix<double,4,1>> MapVector4dType;
  typedef Eigen::Map<const Eigen::Matrix<double,4,1>> MapConstVector4dType;
  typedef Eigen::Map<Eigen::Matrix<double,4,4,Eigen::RowMajor>> MapMatrix4x4RowType;
  typedef Eigen::Matrix<double,4,4> Matrix4x4;
  typedef Eigen::Matrix<double,4,1> Matrix4x1;
  typedef Eigen::Matrix<double,4,4,Eigen::RowMajor> Matrix4x4RowType;
  
  Matrix4x4 inv_sqrt_cov_;
  Matrix4x1 sqrt_var_;
  Matrix4x4 correl_;
  Matrix4x4 check_mat_;
  
  CameraIntrinsicsState* cam_;
  Matrix4x4RowType dresidual_dmeasurement_;
  
  // Default constructor
  CameraIntrinsicsNormalPrior() : inv_sqrt_cov_(Matrix4x4::Zero()), sqrt_var_(Matrix4x1::Zero()),
  correl_(Matrix4x4::Zero()), check_mat_(Matrix4x4::Zero()),
  cam_(NULL), dresidual_dmeasurement_(Matrix4x4RowType::Zero()) {}
  
  CameraIntrinsicsNormalPrior(const CameraIntrinsicsNormalPrior& p) : inv_sqrt_cov_(p.inv_sqrt_cov_),
  sqrt_var_(p.sqrt_var_), correl_(p.correl_), check_mat_(p.check_mat_),
  cam_(p.cam_), dresidual_dmeasurement_(p.dresidual_dmeasurement_) {}
  
  CameraIntrinsicsNormalPrior& operator= (const CameraIntrinsicsNormalPrior& p) {
    inv_sqrt_cov_ = p.inv_sqrt_cov_; sqrt_var_ = p.sqrt_var_;
    correl_ = p.correl_; check_mat_ = p.check_mat_; cam_ = p.cam_;
    dresidual_dmeasurement_ = p.dresidual_dmeasurement_;
  }
  
  // SetZero
  bool SetZero() {
    inv_sqrt_cov_ = Matrix4x4::Zero();
    sqrt_var_ = Matrix4x1::Zero();
    correl_ = Matrix4x4::Zero();
    check_mat_ = Matrix4x4::Zero();
    cam_ = NULL;
    dresidual_dmeasurement_ = Matrix4x4RowType::Zero();
    return true;
  }
  
  // Create with an existing Pose3dState
  bool Create(CameraIntrinsicsState* cam, bool detailed=false) {
    if (!cam) {LOG(ERROR) << "Cam ptr is NULL!"; return false;}
    if (detailed) {
      if (!CalculateInverseSqrtMatrix(cam->covariance_, &inv_sqrt_cov_, &sqrt_var_, &correl_, &check_mat_)) {
        LOG(ERROR) << "Could not calculate inverse sqrt of covariance matrix";
        return false;
      }
    } else {
      if (!CalculateInverseSqrtMatrix(cam->covariance_, &inv_sqrt_cov_, &sqrt_var_)) {
        LOG(ERROR) << "Could not calculate inverse sqrt of covariance matrix";
        return false;
      }        
    }
    cam_ = cam;
    if (!CalculateStartingResidualandJacobians()) {
      LOG(ERROR) << "Could not calculate jacobian";
      return false;
    }
    return true;
  }
  
  // Calculate Jacobians
  bool CalculateStartingResidualandJacobians() {
    
    //dresidual_dmeasurement_.setZero();
    //dresidual_dmeasurement_.block<3,3>(0,0) = -Eigen::Matrix3d::Identity();
    //dresidual_dmeasurement_.block<3,3>(3,3) = -Eigen::Matrix3d::Identity();
    
    // Scale residual and jacobians for noise sqrt variance
    //dresidual_dmeasurement_ = inv_sqrt_cov_ * dresidual_dmeasurement_;
    dresidual_dmeasurement_ = inv_sqrt_cov_;
    
    return true;
  }
  
  // Evaluate function for ceres solver
  virtual bool Evaluate(
      double const* const* parameters,
      double* residuals,
      double** jacobians) const {
    
    // Assuming Check() has been performed before
    
    // Mapping residuals and parameters to matrices
    MapConstVector4dType   dmeasurement(parameters[0]);
    MapVector4dType        residual(residuals);
    
    /* Calculate residual from the error states. Linear model of the residual wrt error states
     * is used to recalculate the residual. */
    residual = dresidual_dmeasurement_ * dmeasurement;
    
    /* Jacobians are not recalculated with changing states. We use first estimates Jacobians
     * as this shown to keep the estimator consistent. */
    if (jacobians != NULL) {
      if (jacobians[0] != NULL) {
        MapMatrix4x4RowType jac(jacobians[0]);
        jac = dresidual_dmeasurement_;
      }
    }
    
    return true;
  }  // Evaluate
  
};  // CameraIntrinsicsNormalPrior
std::ostream& operator<<(std::ostream& os, const CameraIntrinsicsNormalPrior& prior) {
  Eigen::IOFormat CleanFmt(2, 0, ", ", "\n", "", "");
  return os << "Inv sqrt cov = \n" << prior.inv_sqrt_cov_.format(CleanFmt)
      << "\n Sqrt var = " << prior.sqrt_var_.transpose().format(CleanFmt)
      << "\n Correl = \n" << prior.correl_.format(CleanFmt)
      << "\n check = \n" << prior.check_mat_.format(CleanFmt);
}


/* Predict Pose3d state
 * Use the last state to predict the next one. As this is static, new state is same as previous
 */
bool PredictPose3dState(const Pose3dState& state0, const int64_t& interval,
    Pose3dState* state1) {
  
  // Set the target timestamp
  state1->timestamp_ = state0.timestamp_ + interval;
  
  // Copy values
  state1->LqvG_ = state0.LqvG_;
  state1->GpL_ = state0.GpL_;
  state1->dGaL_ = state0.dGaL_;
  state1->dGpL_ = state0.dGpL_;
  state1->covariance_ = state0.covariance_;
  state1->information_ = state0.information_;
  
  return true;
}

/* Pose3dNormalPrior
 * Implements the residual for normal pose3d prior */
class Pose3dNormalPrior : public ceres::SizedCostFunction<6, 6> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  // Type declarations
  typedef Eigen::Map<Eigen::Matrix<double,6,1>> MapVector6dType;
  typedef Eigen::Map<const Eigen::Matrix<double,6,1>> MapConstVector6dType;
  typedef Eigen::Map<Eigen::Matrix<double,6,6,Eigen::RowMajor>> MapMatrix6x6RowType;
  typedef Eigen::Matrix<double,6,6> Matrix6x6;
  typedef Eigen::Matrix<double,6,1> Matrix6x1;
  typedef Eigen::Matrix<double,6,6,Eigen::RowMajor> Matrix6x6RowType;
  
  Matrix6x6 inv_sqrt_cov_;
  Matrix6x1 sqrt_var_;
  Matrix6x6 correl_;
  Matrix6x6 check_mat_;
  
  Pose3dState* pose_;
  Matrix6x6RowType dresidual_dmeasurement_;
  
  // Default constructor
  Pose3dNormalPrior() : inv_sqrt_cov_(Matrix6x6::Zero()), sqrt_var_(Matrix6x1::Zero()),
  correl_(Matrix6x6::Zero()), check_mat_(Matrix6x6::Zero()),
  pose_(NULL), dresidual_dmeasurement_(Matrix6x6RowType::Zero()) {}
  
  Pose3dNormalPrior(const Pose3dNormalPrior& p) : inv_sqrt_cov_(p.inv_sqrt_cov_),
  sqrt_var_(p.sqrt_var_), correl_(p.correl_), check_mat_(p.check_mat_),
  pose_(p.pose_), dresidual_dmeasurement_(p.dresidual_dmeasurement_) {}
  
  Pose3dNormalPrior& operator= (const Pose3dNormalPrior& p) {
    inv_sqrt_cov_ = p.inv_sqrt_cov_; sqrt_var_ = p.sqrt_var_;
    correl_ = p.correl_; check_mat_ = p.check_mat_; pose_ = p.pose_;
    dresidual_dmeasurement_ = p.dresidual_dmeasurement_;
  }
  
  // SetZero
  bool SetZero() {
    inv_sqrt_cov_ = Matrix6x6::Zero();
    sqrt_var_ = Matrix6x1::Zero();
    correl_ = Matrix6x6::Zero();
    check_mat_ = Matrix6x6::Zero();
    pose_ = NULL;
    dresidual_dmeasurement_ = Matrix6x6RowType::Zero();
    return true;
  }
  
  // Create with an existing Pose3dState
  bool Create(Pose3dState* pose, bool detailed=false) {
    if (!pose) {LOG(ERROR) << "Pose ptr is NULL!"; return false;}
    if (detailed) {
      if (!CalculateInverseSqrtMatrix(pose->covariance_, &inv_sqrt_cov_, &sqrt_var_, &correl_, &check_mat_)) {
        LOG(ERROR) << "Could not calculate inverse sqrt of covariance matrix";
        return false;
      }
    } else {
      if (!CalculateInverseSqrtMatrix(pose->covariance_, &inv_sqrt_cov_, &sqrt_var_)) {
        LOG(ERROR) << "Could not calculate inverse sqrt of covariance matrix";
        return false;
      }        
    }
    pose_ = pose;
    if (!CalculateStartingResidualandJacobians()) {
      LOG(ERROR) << "Could not calculate jacobian";
      return false;
    }
    return true;
  }
  
  // Calculate Jacobians
  bool CalculateStartingResidualandJacobians() {
    
    //dresidual_dmeasurement_.setZero();
    //dresidual_dmeasurement_.block<3,3>(0,0) = -Eigen::Matrix3d::Identity();
    //dresidual_dmeasurement_.block<3,3>(3,3) = -Eigen::Matrix3d::Identity();
    
    // Scale residual and jacobians for noise sqrt variance
    //dresidual_dmeasurement_ = inv_sqrt_cov_ * dresidual_dmeasurement_;
    dresidual_dmeasurement_ = inv_sqrt_cov_;
    
    return true;
  }
  
  // Evaluate function for ceres solver
  virtual bool Evaluate(
      double const* const* parameters,
      double* residuals,
      double** jacobians) const {
    
    // Assuming Check() has been performed before
    
    // Mapping residuals and parameters to matrices
    MapConstVector6dType   dmeasurement(parameters[0]);
    MapVector6dType        residual(residuals);
    
    /* Calculate residual from the error states. Linear model of the residual wrt error states
     * is used to recalculate the residual. */
    residual = dresidual_dmeasurement_ * dmeasurement;
    
    /* Jacobians are not recalculated with changing states. We use first estimates Jacobians
     * as this shown to keep the estimator consistent. */
    if (jacobians != NULL) {
      if (jacobians[0] != NULL) {
        MapMatrix6x6RowType jac(jacobians[0]);
        jac = dresidual_dmeasurement_;
      }
    }
    
    return true;
  }  // Evaluate
  
};  // Pose3dNormalPrior
std::ostream& operator<<(std::ostream& os, const Pose3dNormalPrior& prior) {
  Eigen::IOFormat CleanFmt(2, 0, ", ", "\n", "", "");
  return os << "Inv sqrt cov = \n" << prior.inv_sqrt_cov_.format(CleanFmt)
      << "\n Sqrt var = " << prior.sqrt_var_.transpose().format(CleanFmt)
      << "\n Correl = \n" << prior.correl_.format(CleanFmt)
      << "\n check = \n" << prior.check_mat_.format(CleanFmt);
}

/* April tag functions */

// April Tag reading
struct AprilTagReadingType {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  typedef Eigen::Map<Eigen::Matrix<double,8,1>> MapVector8dType;
  typedef Eigen::Map<const Eigen::Matrix<double,6,1>> MapConstVector6dType;
  typedef Eigen::Map<const Eigen::Matrix<double,1,1>> MapConstVector1dType;
  typedef Eigen::Map<const Eigen::Matrix<double,4,1>> MapConstVector4dType;
  typedef Eigen::Map<Eigen::Matrix<double,8,6,Eigen::RowMajor>> MapMatrix8x6RowType;
  typedef Eigen::Map<Eigen::Matrix<double,8,4,Eigen::RowMajor>> MapMatrix8x4RowType;
  typedef Eigen::Map<Eigen::Matrix<double,8,1>> MapMatrix8x1RowType;
  typedef Eigen::Matrix<double,8,6,Eigen::RowMajor> Matrix8x6RowType;
  typedef Eigen::Matrix<double,6,1> Matrix6x1RowType;
  typedef Eigen::Matrix<double,6,4,Eigen::RowMajor> Matrix6x4RowType;
  typedef Eigen::Matrix<double,6,6,Eigen::RowMajor> Matrix6x6RowType;
  typedef Eigen::Matrix<double,8,8,Eigen::RowMajor> Matrix8x8RowType;
  typedef Eigen::Matrix<double,3,4> Matrix3x4Type;
  typedef Eigen::Matrix<double,2,4,Eigen::RowMajor> Matrix2x4RowType;
  typedef Eigen::Matrix<double,2,2,Eigen::RowMajor> Matrix2x2RowType;
  typedef Eigen::Matrix<double,2,3,Eigen::RowMajor> Matrix2x3RowType;
  typedef Eigen::Matrix<double,3,6,Eigen::RowMajor> Matrix3x6RowType;
  
  int64_t timestamp;    // timestamp of the observation - serves as image index
  int32_t camera_num;   // camera from which this was seen
  std::string tag_id;
  Eigen::Matrix<double,2,4> image_coords;  // tag corner coordinates in image
  
  // After estimation is done
  bool pose_has_been_estimated;
  double reproj_error;
  double information;
  Eigen::Quaterniond TjqCi;
  Eigen::Matrix3d TjrCi;
  Eigen::Vector3d CipTj;
  Eigen::Vector3d TjpCi;
  Matrix6x6RowType Cov_dCiposeTj;
  Matrix6x6RowType Cov_dTjposeCi;
  
  AprilTagReadingType() : timestamp(0), camera_num(0), tag_id(16, ' '),
    pose_has_been_estimated(false), reproj_error(0.), information(0.),
    TjqCi(Eigen::Quaterniond::Identity()), TjrCi(Eigen::Matrix3d::Identity()),
    CipTj(Eigen::Vector3d::Zero()), TjpCi(Eigen::Vector3d::Zero()) {
    image_coords.setZero();
    Cov_dCiposeTj.setZero();
    Cov_dTjposeCi.setZero();
  }
  
  bool Reset() {
    timestamp = 0;
    camera_num = 0;
    tag_id = std::string(16, ' ');
    image_coords.setZero();
    pose_has_been_estimated = false;
    reproj_error = 0.;
    information = 0.;
    TjqCi = Eigen::Quaterniond::Identity();
    TjrCi = Eigen::Matrix3d::Identity();
    CipTj = Eigen::Vector3d::Zero();
    TjpCi = Eigen::Vector3d::Zero();
    Cov_dCiposeTj.setZero();
    Cov_dTjposeCi.setZero();
    return true;
  }
  
  bool IsZero() const {
    return (timestamp==0);
  }
  
  // Set from AprilTagMessage
  bool SetFromAprilTagMessage(const int64_t& ts, const int32_t& cam_num, const int32_t& i_tag,
      const anantak::AprilTagMessage& apriltag_msg) {
    Reset();
    // make sure msg has a message at i_tag index
    if (apriltag_msg.tag_id_size()<i_tag+1) {
      LOG(ERROR) << "AprilTagMessage has " << apriltag_msg.tag_id_size() << " tags, asked for "
          << i_tag;
      Reset();
      return false;
    }
    timestamp = ts;
    camera_num = cam_num;
    tag_id = apriltag_msg.tag_id(i_tag);
    image_coords <<
        apriltag_msg.u_1(i_tag), apriltag_msg.u_2(i_tag), apriltag_msg.u_3(i_tag), apriltag_msg.u_4(i_tag),
        apriltag_msg.v_1(i_tag), apriltag_msg.v_2(i_tag), apriltag_msg.v_3(i_tag), apriltag_msg.v_4(i_tag);
    return true;
  }
  
  const int64_t& Timestamp() const {return timestamp;}
  
  // Calculate tag pose estimates
  bool EstimateCameraToTagPose(
      const anantak::CameraIntrinsicsState* camera_,  // for camera matrix and its covariance
      const double& tag_size_ ) {
    
    // Set estimated flag to true
    pose_has_been_estimated = true;
    
    // Calculate Camera to Tag pose      
    Matrix3x4Type Tjpf = anantak::AprilTag3dCorners(tag_size_);
    Eigen::Matrix3d K = camera_->K_;
    Eigen::Matrix3d K_inv = camera_->K_inv_;
    
    Matrix3x4Type corners_2d;
    corners_2d.block<2,4>(0,0) = image_coords;
    corners_2d.block<1,4>(2,0) << 1., 1., 1., 1.;
    corners_2d = K_inv * corners_2d;
    corners_2d.colwise().normalize();
    //VLOG(1) << "    corners_2d = \n" << corners_2d;
    //Eigen::Vector3d test_corner; test_corner << image_coords(0,0)-K(0,2), image_coords(1,0)-K(1,2), K(0,0); test_corner.normalize(); VLOG(1) << "    test corner = " << test_corner;
    // Solve for camera pose wrt AprilTag using opengv's p3p algorithm
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
    reproj_error = dffs[min_idx];
    
    TjrCi = pnp_transformations[min_idx].block<3,3>(0,0);
    TjpCi = pnp_transformations[min_idx].col(3);
    //Cpf = Cpfs.block<3,4>(0,4*min_idx);
    //VLOG(2) << "Tag "<<tag_id<<" Transform = ("<<reproj_error<<")\n"<<TrC<<"\n"<<TpC;
    
    TjqCi = Eigen::Quaterniond(TjrCi);
    Eigen::Matrix3d CirTj(TjrCi.transpose());
    CipTj = -TjrCi.transpose()*TjpCi;
    
    information = 1./(TjpCi[2] * std::max(reproj_error, 1.));
    
    if (std::isnan(information) || information<Epsilon ||
        !TjrCi.allFinite() || !TjpCi.allFinite()) {
      LOG(ERROR) << "information is nan or <0. information = " << information;
      LOG(ERROR) << "\nTjrCi is not finite. TjrCi = \n" << TjrCi;
      LOG(ERROR) << "\nTjpCi is not finite. TjpCi = " << TjpCi.transpose();
      LOG(ERROR) << "\nImage coords = \n" << image_coords;
      for (int i=0; i<pnp_transformations.size(); i++)
        LOG(ERROR) << "\npnp_transformations ("<<i<<") = \n" << pnp_transformations[i];
      LOG(ERROR) << "\ndffs = " << dffs.transpose() << "  min_idx = " << min_idx;
      return false;
    }
    
    return true;
  }
  
  bool CalculateCameraPoseFromTagPose(
      const anantak::Pose3dState* tag_pose,
      anantak::Pose3dState* cam_pose) {
    
    // tag_pose provides TjqT0, T0pTj and Cov(dT0qpTj)
    // CirT0 = CirTj * TjrT0
    // T0pCi = T0pTj + T0.TjpCi = T0pTj + T0rTj*TjpCi
    
    // Estimate the mean
    Eigen::Quaterniond TjqT0 = tag_pose->Quaternion();
    Eigen::Vector3d T0pTj = tag_pose->Position();
    Eigen::Quaterniond CiqT0 = TjqCi.conjugate() * TjqT0;
    Eigen::Matrix3d T0rTj(TjqT0.conjugate());
    Eigen::Vector3d T0pCi = T0pTj + T0rTj*TjpCi;
    
    // Copy into cam pose
    cam_pose->LqvG_ = CiqT0.coeffs();
    cam_pose->GpL_  = T0pCi;
    cam_pose->information_ = information;
    
    return true;
  }
  
  bool CalculateTagPoseFromCameraPose(
      const anantak::Pose3dState* cam_pose,
      anantak::Pose3dState* tag_pose) {
    
    // cam_pose provides CiqT0, T0pCi and Cov(dT0qpCi)
    // TjrT0 = TjrCi * CirT0
    // T0pTj = T0pCi + T0.CipTj = T0pCi + T0rCi*CipTj
    
    // Estimate the mean
    Eigen::Quaterniond CiqT0 = cam_pose->Quaternion();
    Eigen::Vector3d T0pCi = cam_pose->Position();
    Eigen::Quaterniond TjqT0 = TjqCi * CiqT0;
    Eigen::Matrix3d T0rCi(CiqT0.conjugate());
    Eigen::Vector3d T0pTj = T0pCi + T0rCi*CipTj;
    
    // Copy into tag pose
    tag_pose->LqvG_ = TjqT0.coeffs();
    tag_pose->GpL_  = T0pTj;
    tag_pose->information_ = information;
    
    return true;
  }
  
  // Calculate tag pose estimates with Jacobians
  bool EstimateCameraToTagPoseWithJacobians(
      const anantak::CameraIntrinsicsState* camera_,  // for camera matrix and its covariance
      const anantak::StaticAprilTagState* tagTj_,     // for tag size and its covariance
      const double sigma_image_) {
    
    // Set estimated flag to true
    pose_has_been_estimated = true;
    
    // Calculate Camera to Tag pose      
    Matrix3x4Type Tjpf = anantak::AprilTag3dCorners(tagTj_->size_.Value());
    Eigen::Matrix3d K = camera_->K_;
    Eigen::Matrix3d K_inv = camera_->K_inv_;
    
    Matrix3x4Type corners_2d;
    corners_2d.block<2,4>(0,0) = image_coords;
    corners_2d.block<1,4>(2,0) << 1., 1., 1., 1.;
    corners_2d = K_inv * corners_2d;
    corners_2d.colwise().normalize();
    //VLOG(1) << "    corners_2d = \n" << corners_2d;
    //Eigen::Vector3d test_corner; test_corner << image_coords(0,0)-K(0,2), image_coords(1,0)-K(1,2), K(0,0); test_corner.normalize(); VLOG(1) << "    test corner = " << test_corner;
    // Solve for camera pose wrt AprilTag using opengv's p3p algorithm
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
    Eigen::Vector4d dffs;
    Eigen::Matrix<double,3,16> Cpfs; // corner 3d coordinates in camera frame for 4 guesses
    for (int i=0; i<pnp_transformations.size(); i++) {
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
      dffs[i] = total_dff;
    } // for all transformations
    //VLOG(1) << dffs.transpose();
    Eigen::Vector4d::Index min_idx; dffs.minCoeff(&min_idx);
    //VLOG(1) << "Min transformation at " << min_idx;
    reproj_error = dffs[min_idx];
    
    TjrCi = pnp_transformations[min_idx].block<3,3>(0,0);
    TjpCi = pnp_transformations[min_idx].col(3);
    //Cpf = Cpfs.block<3,4>(0,4*min_idx);
    //VLOG(2) << "Tag "<<tag_id<<" Transform = ("<<reproj_error<<")\n"<<TrC<<"\n"<<TpC;
    
    TjqCi = Eigen::Quaterniond(TjrCi);
    Eigen::Matrix3d CirTj(TjrCi.transpose());
    CipTj = -TjrCi.transpose()*TjpCi;
    
    // Calculate Jacobians
    Matrix6x6RowType    dtagview_dCiposeTj_;
    Matrix6x1RowType    dtagview_dTj_size_;
    Matrix6x4RowType    dtagview_dK_;             // K is the camera matrix
    
    Eigen::Matrix3d I3(Eigen::Matrix3d::Identity());
    
    //VLOG(1) << "Tjpf = \n" << Tjpf;
    
    double fx = K(0,0);
    double fy = K(1,1);
    double cx = K(0,2);
    double cy = K(1,2);
    
    // Uncertainty in tag size
    Matrix6x6RowType tag_size_cov_mat;
    tag_size_cov_mat.setZero();
    
    // Use first three corners only, fourth one will be collinear and unnecessary
    for (int i_crnr=0; i_crnr<3; i_crnr++) {
      
      Eigen::Vector3d Tjpfk = Tjpf.col(i_crnr);
      Eigen::Vector3d CiTjpfk = CirTj * Tjpfk;
      Eigen::Vector3d Cipfk = CipTj + CiTjpfk;
      double z_recip = 1./Cipfk[2];
      double x_by_z = Cipfk[0] * z_recip;
      double y_by_z = Cipfk[1] * z_recip;
      Eigen::Vector2d uv;
      uv << fx*x_by_z + cx, fy*y_by_z + cy;
      
      //VLOG(1) << "uv calc, seen corners = \n" << uv.transpose() << " " << tag_view_->image_coords.col(i_crnr).transpose();
      
      Matrix2x4RowType duv_dK;
      duv_dK << x_by_z, 0., 1., 0.,   0., y_by_z, 0., 1.;
      
      Matrix2x2RowType fxfy_by_z;
      fxfy_by_z << fx*z_recip, 0.,   0., fy*z_recip;
      
      Matrix2x3RowType duv_dCiposnf;
      duv_dCiposnf << 1., 0., -x_by_z,   0., 1., -y_by_z;
      duv_dCiposnf = fxfy_by_z * duv_dCiposnf;
      
      Matrix3x6RowType dCiposnf_dCiposeTj;
      dCiposnf_dCiposeTj.setZero();
      dCiposnf_dCiposeTj.block<3,3>(0,0) = -anantak::SkewSymmetricMatrix(CiTjpfk);
      dCiposnf_dCiposeTj.block<3,3>(0,3) =  I3;
      
      Eigen::Vector3d dCiposnf_dsizeTj = Tjpfk / tagTj_->size_.Value();
      dCiposnf_dsizeTj = CirTj * dCiposnf_dsizeTj;
      
      dtagview_dK_.block<2,4>(2*i_crnr,0) =  duv_dK;
      
      dtagview_dCiposeTj_.block<2,6>(2*i_crnr,0) = duv_dCiposnf * dCiposnf_dCiposeTj;
      
      dtagview_dTj_size_.block<2,1>(2*i_crnr,0) = duv_dCiposnf * dCiposnf_dsizeTj;
      
      Eigen::Matrix3d tag_size_cov = tagTj_->size_.covariance_ * I3;
      tag_size_cov = CirTj * tag_size_cov * CirTj.transpose();
      tag_size_cov_mat.block<2,2>(2*i_crnr,2*i_crnr) = duv_dCiposnf * tag_size_cov * duv_dCiposnf.transpose();
    }
    
    // Calculate variance of CiPTj
    // dtagview = dtagview_dK_ * dK + dtagview_dCiposeTj_ * dCiposeTj + dtagview_dTj_size_ * dTj_size
    // dtagview_dCiposeTj_ * dCiposeTj = dtagview - dtagview_dK_ * dK - dtagview_dTj_size_ * dTj_size
    
    // dtagview_dCiposeTj_ * Cov(dCiposeTj) * dtagview_dCiposeTj_' =
    //   Cov(dtagview)  +  dtagview_dK_ * Cov(dK) * dtagview_dK_'  +  tag_size_cov_mat
    
    //double var_corner = std::max(sigma_image_ * sigma_image_, reproj_error);
    //Matrix6x6RowType Cov_dtagview = var_corner * Matrix6x6RowType::Identity();
    Matrix6x6RowType Cov_dtagview = reproj_error * Matrix6x6RowType::Identity();
    Matrix6x6RowType Cov_dK = dtagview_dK_ * camera_->covariance_ * dtagview_dK_.transpose();
    Cov_dCiposeTj = Cov_dtagview + Cov_dK + tag_size_cov_mat;
    
    // No guarantee if inverse will always exist, but here is the supporting argument:
    //  Three points of tag are used that are almost never in a straight line
    //  They will be in a straight line only if tag is viewed from the side, but this should not happen.
    //  Now, if corners are never in a straight line, they should not be collinear, so inverse should exist.
    //  A rigorous argument might be possible, defered to a later time.
    Matrix6x6RowType dtagview_dCiposeTj_inv = dtagview_dCiposeTj_.inverse();
    Cov_dCiposeTj = dtagview_dCiposeTj_inv * Cov_dCiposeTj * dtagview_dCiposeTj_inv.transpose();
    
    Matrix6x6RowType inv_mat;
    inv_mat.setZero();
    inv_mat.block<3,3>(0,0) = -TjrCi;
    inv_mat.block<3,3>(3,0) = -TjrCi*anantak::SkewSymmetricMatrix(CipTj);
    inv_mat.block<3,3>(3,3) = -TjrCi;
    Cov_dTjposeCi = inv_mat * Cov_dCiposeTj * inv_mat.transpose();
    
    //VLOG(1) << "Estimated Camera to Tag pose:";
    //VLOG(1) << "  TjqCi, CipTj = \n" << TjqCi.coeffs().transpose() << ", " << CipTj.transpose();
    //VLOG(1) << "  Cov_dCiposeTj = \n" << Cov_dCiposeTj;
    
    return true;
  }
  
  bool CalculateCameraPoseFromTagPoseWithJacobians(
      const anantak::Pose3dState* tag_pose,
      anantak::Pose3dState* cam_pose) {
    
    // tag_pose provides TjqT0, T0pTj and Cov(dT0qpTj)
    // CirT0 = CirTj * TjrT0
    // T0pCi = T0pTj + T0.TjpCi = T0pTj + T0rTj*TjpCi
    
    // Estimate the mean
    Eigen::Quaterniond TjqT0 = tag_pose->Quaternion();
    Eigen::Vector3d T0pTj = tag_pose->Position();
    Eigen::Quaterniond CiqT0 = TjqCi.conjugate() * TjqT0;
    Eigen::Matrix3d T0rTj(TjqT0.conjugate());
    Eigen::Vector3d T0pCi = T0pTj + T0rTj*TjpCi;
    
    // Estimate the covariance
    Matrix6x6RowType dT0poseCi_dT0poseTj;
    Eigen::Vector3d T0TjpCi = T0rTj * TjpCi;
    dT0poseCi_dT0poseTj.setZero();
    dT0poseCi_dT0poseTj.block<3,3>(0,0) =  Eigen::Matrix3d::Identity();
    dT0poseCi_dT0poseTj.block<3,3>(3,0) = -anantak::SkewSymmetricMatrix(T0TjpCi);
    dT0poseCi_dT0poseTj.block<3,3>(3,3) =  Eigen::Matrix3d::Identity();
    
    Matrix6x6RowType dT0poseCi_dTjposeCi;
    dT0poseCi_dTjposeCi.setZero();
    dT0poseCi_dTjposeCi.block<3,3>(0,0) =  T0rTj;
    dT0poseCi_dTjposeCi.block<3,3>(3,3) =  T0rTj;
    
    // Copy into cam pose
    cam_pose->LqvG_ = CiqT0.coeffs();
    cam_pose->GpL_  = T0pCi;
    cam_pose->covariance_ =  dT0poseCi_dT0poseTj * tag_pose->covariance_ * dT0poseCi_dT0poseTj.transpose();
    cam_pose->covariance_ += dT0poseCi_dTjposeCi * Cov_dTjposeCi * dT0poseCi_dTjposeCi.transpose();
    
    return true;
  }
  
  bool CalculateTagPoseFromCameraPoseWithJacobians(
      const anantak::Pose3dState* cam_pose,
      anantak::Pose3dState* tag_pose) {
    
    // cam_pose provides CiqT0, T0pCi and Cov(dT0qpCi)
    // TjrT0 = TjrCi * CirT0
    // T0pTj = T0pCi + T0.CipTj = T0pCi + T0rCi*CipTj
    
    // Estimate the mean
    Eigen::Quaterniond CiqT0 = cam_pose->Quaternion();
    Eigen::Vector3d T0pCi = cam_pose->Position();
    Eigen::Quaterniond TjqT0 = TjqCi * CiqT0;
    Eigen::Matrix3d T0rCi(CiqT0.conjugate());
    Eigen::Vector3d T0pTj = T0pCi + T0rCi*CipTj;
    
    // Estimate the covariance
    Matrix6x6RowType dT0poseTj_dT0poseCi;
    Eigen::Vector3d T0CipTj = T0rCi * CipTj;
    dT0poseTj_dT0poseCi.setZero();
    dT0poseTj_dT0poseCi.block<3,3>(0,0) =  Eigen::Matrix3d::Identity();
    dT0poseTj_dT0poseCi.block<3,3>(3,0) = -anantak::SkewSymmetricMatrix(T0CipTj);
    dT0poseTj_dT0poseCi.block<3,3>(3,3) =  Eigen::Matrix3d::Identity();
    
    Matrix6x6RowType dT0poseTj_dCiposeTj;
    dT0poseTj_dCiposeTj.setZero();
    dT0poseTj_dCiposeTj.block<3,3>(0,0) =  T0rCi;
    dT0poseTj_dCiposeTj.block<3,3>(3,3) =  T0rCi;
    
    // Copy into tag pose
    tag_pose->LqvG_ = TjqT0.coeffs();
    tag_pose->GpL_  = T0pTj;
    tag_pose->covariance_ =  dT0poseTj_dT0poseCi * cam_pose->covariance_ * dT0poseTj_dT0poseCi.transpose();
    tag_pose->covariance_ += dT0poseTj_dCiposeTj * Cov_dCiposeTj * dT0poseTj_dCiposeTj.transpose();
    
    return true;
  }
  
};  // AprilTagReadingType
std::ostream& operator<<(std::ostream& os, const AprilTagReadingType& reading) {
  bool full = false;
  //Eigen::Quaterniond TjqCi;
  //Eigen::Matrix3d TjrCi;
  //Eigen::Vector3d CipTj;
  //Eigen::Vector3d TjpCi;
  //Matrix6x6RowType Cov_dCiposeTj;
  //Matrix6x6RowType Cov_dTjposeCi;
  Eigen::AngleAxisd aa(reading.TjqCi.conjugate());
  if (full) 
    return os << "   Position = " << reading.CipTj.transpose()
        << ", Rotation = " << aa.axis().transpose()
        << ", " << aa.angle()*DegreesPerRadian << "\n   CovMat = \n"
        << reading.Cov_dCiposeTj;
  else 
    return os << "   Position = " << reading.CipTj.transpose()
        << ", Rotation = " << aa.axis().transpose()
        << ", " << aa.angle()*DegreesPerRadian << "   CovDiagSqrt = "
        << reading.Cov_dCiposeTj.diagonal().cwiseSqrt().transpose()
        << ",  reproj_error = " << reading.reproj_error;
}

/* Static April tags map
 * Represents a map of square tags. Each 
 * Implements functions to
 *  Assimilate new tag readings from an image, while adding new tags seen and updating old ones
 */
class StaticAprilTagsMap {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef Eigen::Matrix<double,6,6> Matrix6x6;
  
  struct Options {
    int32_t max_tags_in_map;    // number of maximum tags to keep
    int32_t max_tags_in_image;  // number of maximum tags expected in a single image
    
    bool    all_tags_have_same_size;          // true/false
    double  default_april_tag_size;           // in meters
    double  default_april_tag_size_sigma;     // in meters
    double  default_image_tag_corner_sigma;   // in pixels
    
    double  infinite_info_rotn_sigma;         // in radians
    double  infinite_info_posn_sigma;         // in meters
    double  zero_info_rotn_sigma;             // in radians
    double  zero_info_posn_sigma;             // in meters
    
    double  infinite_information;     // arbitrary relative units
    double  zero_information;         // arbitrary relative units
    
    Options() {
      max_tags_in_map = 500;
      max_tags_in_image = 50;
      
      all_tags_have_same_size = true;
      default_april_tag_size = 0.4780;          // meters
      default_april_tag_size_sigma = 0.010/3.0; // meters
      default_image_tag_corner_sigma = 1.0;     // pixels
      
      infinite_info_rotn_sigma = 1e-6;  // very small value to show certainty
      infinite_info_posn_sigma = 1e-6;  // very small value to show certainty
      zero_info_rotn_sigma = 1e+6;      // very large value to show uncertainty
      zero_info_posn_sigma = 1e+6;      // very large value to show uncertainty
      
      infinite_information = 1e+14;
      zero_information = 0.;
    }
    
  };  // StaticAprilTagsMap::Options
  
  // Data holders
  StaticAprilTagsMap::Options options_;
  
  // Tag data
  std::unique_ptr<anantak::CircularQueue<anantak::StaticAprilTagState>> tag_poses_;
  std::unique_ptr<anantak::CircularQueue<anantak::Pose3dNormalPrior>> tag_pose_priors_;
  
  // Single image data - these are cleared for every image
  std::unique_ptr<anantak::CircularQueue<anantak::AprilTagReadingType>> april_tag_readings_;
  std::unique_ptr<anantak::CircularQueue<anantak::Pose3dState>> cam_poses_collected_;
  
  // Pointer to the Origin tag - needed to set its state as constant
  anantak::StaticAprilTagState* origin_tag_;
  
  // Helpers
  Eigen::Matrix<double,6,6> infinite_info_pose_covariance_;
  Eigen::Matrix<double,6,6> zero_info_pose_covariance_;
  double tag_size_variance_;
  
  // Constructors. A map of April tags could be initiated in multiple ways.
  //  From a file - storing tag poses and sizes
  //  From a single tag size - meaning no tag but all tags have the same size
  //  From nothing - meaning no tags and no sizes available
  StaticAprilTagsMap():
    options_() {
    if (!Initiate()) {
      LOG(ERROR) << "Could not initiate Tag map.";
    }
  }
  
  StaticAprilTagsMap(const StaticAprilTagsMap::Options& options) {
    options_ = options;  // make a local copy
    if (!Initiate()) {
      LOG(ERROR) << "Could not initiate Tag map.";
    }
  }
  
  bool Initiate() {
    
    // Allocate memory for tag poses
    std::unique_ptr<anantak::CircularQueue<anantak::StaticAprilTagState>> cq_tag_poses_ptr(new
        anantak::CircularQueue<anantak::StaticAprilTagState>(options_.max_tags_in_map));
    tag_poses_ = std::move(cq_tag_poses_ptr);
    
    // Allocate memory for tag pose priors
    std::unique_ptr<anantak::CircularQueue<anantak::Pose3dNormalPrior>> cq_tag_pose_priors_ptr(new
        anantak::CircularQueue<anantak::Pose3dNormalPrior>(options_.max_tags_in_map));
    tag_pose_priors_ = std::move(cq_tag_pose_priors_ptr);
    
    // Allocate memory for storing initiation data - tag readings
    std::unique_ptr<anantak::CircularQueue<anantak::AprilTagReadingType>> cq_tag_rdng_ptr(new
        anantak::CircularQueue<anantak::AprilTagReadingType>(options_.max_tags_in_image));
    april_tag_readings_ = std::move(cq_tag_rdng_ptr);
    
    // Allocate memory for collacted camera poses
    std::unique_ptr<anantak::CircularQueue<anantak::Pose3dState>> cq_cam_poses_collected(new
        anantak::CircularQueue<anantak::Pose3dState>(options_.max_tags_in_image));
    cam_poses_collected_ = std::move(cq_cam_poses_collected);      
    
    // Helpers
    double t_var = options_.infinite_info_rotn_sigma * options_.infinite_info_rotn_sigma;
    double p_var = options_.infinite_info_posn_sigma * options_.infinite_info_posn_sigma;
    Eigen::Matrix<double,6,1> inf_pose_var_diag;
    inf_pose_var_diag << t_var, t_var, t_var, p_var, p_var, p_var;
    infinite_info_pose_covariance_ = inf_pose_var_diag.asDiagonal();
    
    t_var = options_.zero_info_rotn_sigma * options_.zero_info_rotn_sigma;
    p_var = options_.zero_info_posn_sigma * options_.zero_info_rotn_sigma;
    Eigen::Matrix<double,6,1> zero_pose_var_diag;
    zero_pose_var_diag << t_var, t_var, t_var, p_var, p_var, p_var;
    zero_info_pose_covariance_ = zero_pose_var_diag.asDiagonal();
    
    tag_size_variance_ = options_.default_april_tag_size_sigma * options_.default_april_tag_size_sigma;
    
    origin_tag_ = NULL;
    
    return true;
  }
  
  // Destructor
  virtual ~StaticAprilTagsMap() {}
  
  // Is this tag pointer origin tag?
  bool IsOriginTag(const anantak::StaticAprilTagState* tag_ptr) const {
    return (tag_ptr == origin_tag_);
  }
  
  // Number of tags
  int32_t NumTags() const {
    return tag_poses_->n_msgs();
  }
  
  // Look for the given tag_id in tag_poses_. Return a pointer to tag pose state
  bool FindTagInTagMap(const std::string& tag_id, anantak::StaticAprilTagState **tagTj) const {
    // Simple linear search through the tag map. Inefficient. We can do better
    bool tag_found = false;
    int i=0;
    *tagTj = NULL;
    while (!tag_found && i<tag_poses_->n_msgs()) {
      tag_found = (tag_id == tag_poses_->at(i).tag_id_);
      if (tag_found) {
        *tagTj = tag_poses_->at_ptr(i);
      }
      i++;
    }
    return tag_found;
  }
  
  // Find the number of common tags with another tag map
  int32_t FindNumberOfCommonTags(const StaticAprilTagsMap& tag_map) const {
    // Bit inefficient, better search could be written
    int32_t num_common_tags = 0;
    for (int i=0; i<tag_map.tag_poses_->n_msgs(); i++) {
      anantak::StaticAprilTagState *tagTj;
      //VLOG(1) << "   checking tag = " << tag_map.tag_poses_->at(i).tag_id_;
      if (FindTagInTagMap(tag_map.tag_poses_->at(i).tag_id_, &tagTj)) {
        num_common_tags++;
        //VLOG(1) << "    Found it";
      } //else {
        //VLOG(1) << "    did not find it";
      ///}
    }
    return num_common_tags;
  }
  
  // Calculate map to map pose
  bool CalculateMapToMapPose(const StaticAprilTagsMap& tag_map, anantak::Pose3dState* pose) const {
    if (!pose) {
      LOG(ERROR) << "Got a nullptr for return pose";
      return false;
    }
    int32_t num_updates = 0;
    pose->SetZero();
    for (int i=0; i<tag_map.tag_poses_->n_msgs(); i++) {
      anantak::StaticAprilTagState *tagTj;
      if (FindTagInTagMap(tag_map.tag_poses_->at(i).tag_id_, &tagTj)) {
        // Skip this tag if both tags are origin tags.
        if (tag_map.tag_poses_->at_ptr(i)==tag_map.origin_tag_ && tagTj==origin_tag_)
            continue;
        // Calculate the pose difference from pose of tag in this map to pose of the tag in other
        anantak::Pose3dState invpose;
        invpose.SetZero();
        if (!InvertPose3dUsingInformation(&tag_map.tag_poses_->at(i).pose_, &invpose)) {
          LOG(ERROR) << "Could not invert pose";
          continue;
        }
        anantak::Pose3dState dpose;
        dpose.SetZero();
        if (!AddPose3dUsingInformation(&tagTj->pose_, &invpose, &dpose)) {
          LOG(ERROR) << "Could not calculate diff between poses for tag " << tagTj->tag_id_;
          continue;
        } else {
          VLOG(1) << "  Diff in pose for tag " << tagTj->tag_id_ << " = " << dpose;
          VLOG(1) << "    T1 = " << tagTj->pose_;
          VLOG(1) << "    T2 = " << tag_map.tag_poses_->at(i).pose_;
          num_updates++;
          // Add the pose to composite pose
          UpdatePose3dUsingInformation(&dpose, pose);
        }
      }  // if tag was found 
    } // for i
    if (num_updates==0) return false;
    return true;
  }
  
  // Combine another map into this map
  bool CombineMap(const StaticAprilTagsMap& tag_map, const anantak::Pose3dState& map_pose) {
    for (int i=0; i<tag_map.tag_poses_->n_msgs(); i++) {
      anantak::StaticAprilTagState *tagTj;
      if (FindTagInTagMap(tag_map.tag_poses_->at(i).tag_id_, &tagTj)) {
        // Skip this tag if both tags are origin tags.
        if (tag_map.tag_poses_->at_ptr(i)==tag_map.origin_tag_ && tagTj==origin_tag_)
            continue;
        // Add the map pose to this one
        anantak::Pose3dState new_pose;
        new_pose.SetZero();
        if (!AddPose3dUsingInformation(&map_pose, &tag_map.tag_poses_->at(i).pose_, &new_pose)) {
          LOG(ERROR) << "Could not add map pose to tag " << tagTj->tag_id_;
        } else {
          VLOG(2) << "  Add map pose to tag " << tagTj->tag_id_ << " = " << new_pose;
          VLOG(2) << "    T1 = " << tag_map.tag_poses_->at(i).pose_;
          VLOG(2) << "    T2 = " << map_pose;
          // Add this pose to composite pose
          UpdatePose3dUsingInformation(&new_pose, &tagTj->pose_);
          VLOG(1) << "Updated tag " << tagTj->tag_id_ << " " << tagTj->pose_;
        }
      }  // if tag was found
      else {
        // This is a new tag. Add it to the tag map.
        anantak::StaticAprilTagState *tag = tag_poses_->next_mutable_element();
        *tag = tag_map.tag_poses_->at(i);  // using implicitly defined operator=
        if (!AddPose3dUsingInformation(&map_pose, &tag_map.tag_poses_->at(i).pose_, &tag->pose_)) {
          LOG(ERROR) << "Could not add map pose to tag " << tag->tag_id_;
        }
        // Set covariances
        tag->pose_.covariance_ = zero_info_pose_covariance_;
        tag->size_.covariance_ = tag_size_variance_;
        // Add a tag pose prior for this tag
        Pose3dNormalPrior* tag_prior = tag_pose_priors_->next_mutable_element();
        tag_prior->SetZero();
        tag_prior->Create(&tag->pose_);
        VLOG(1) << "Added tag " << tag->tag_id_ << " " << tag->pose_;
      }
    } // for each tag in the other map
    return true;
  }
  
  // Look for tag size
  double FindTagSize(const std::string& tag_id) const {
    if (options_.all_tags_have_same_size) {
      return options_.default_april_tag_size;
    } else {
      LOG(ERROR) << "Could not find tag size";
      return 0.;
    }
    return options_.default_april_tag_size;
  }
  
  // Extract Tag readings from a tag message
  bool ExtractTagReadingsFromMessage(const anantak::SensorMsg& tag_msg) {
    // Add new readings
    if (tag_msg.has_header() && tag_msg.has_april_msg()) {
      const anantak::AprilTagMessage& apriltag_msg = tag_msg.april_msg();
      //VLOG(3) << "Number of AprilTags in msg = " << apriltag_msg.tag_id_size();
      // Extract the views one-by-one
      for (int i_tag=0; i_tag < apriltag_msg.tag_id_size(); i_tag++) {
        anantak::AprilTagReadingType *tag_rdng = april_tag_readings_->next_mutable_element();
        tag_rdng->SetFromAprilTagMessage(tag_msg.header().timestamp(),
            apriltag_msg.camera_num(), i_tag, apriltag_msg);
      }
    } else {
      LOG(ERROR) << "Strange: Apriltag sensor messages has no header or apriltag message!";
      return false;
    }
    return true;
  }
  
  // Process a tag message
  //  Gets intrinsics state for the image, Tag view readings and a pointer to a camera pose
  //  returns calculated pose of the image in tag map
  //  returns false if it could not calculate image pose usually if tags are all new
  bool ProcessTagMessage(
      const anantak::SensorMsg& tag_msg,
      const anantak::CameraIntrinsicsState& cam_intrinsics,
      anantak::Pose3dState* cam_pose,
      const bool reestimate_existing_tags = true) {
    
    // If there are no readings, return false
    // For every reading, check if the tag exists
    //    if so, estimate the camera pose
    //    if not add tag to the list of tag pose to be estimated
    // If there are no camera poses calculated, return false
    // Use all camera poses to calculate one camera pose
    // Use camera pose to estimate new tag poses
    // return true
    
    // Clean up the storage queues
    april_tag_readings_->Clear();
    cam_poses_collected_->Clear();
    
    // Extract readings from tag msg
    if (!ExtractTagReadingsFromMessage(tag_msg)) {
      LOG(ERROR) << "Tag readings could not be extracted from message";
      return false;
    }
    
    // If no message seen, return
    if (april_tag_readings_->n_msgs() == 0) {
      VLOG(2) << "No tag was seen in image";
      return false;
    }
    
    // We have some tag readings. Assimilate them.
    for (int i_rdng=0; i_rdng<april_tag_readings_->n_msgs(); i_rdng++) {
      AprilTagReadingType* tag_reading = april_tag_readings_->at_ptr(i_rdng);
      
      // If there are no tags in tag map, add the origin tag
      if (tag_poses_->n_msgs() == 0) {
        // Use the first tag as the origin tag
        //  Origin tag has a pose of zero. This serves as the origin.
        
        // Make sure size is available
        double tag_size = FindTagSize(tag_reading->tag_id);
        if (tag_size<Epsilon) {
          LOG(INFO) << "Could not find tag size, skipping this tag. tag_id = " << tag_reading->tag_id;
        } else {
          // Add the a new tag pose
          StaticAprilTagState* origin_tag = tag_poses_->next_mutable_element();
          Eigen::Quaterniond T0qT0 = Eigen::Quaterniond::Identity();
          Eigen::Vector3d T0pT0 = Eigen::Vector3d::Zero();
          origin_tag->SetZero();
          origin_tag->Create(&tag_reading->tag_id, &T0qT0, &T0pT0, &tag_size);
          // Set covariances
          origin_tag->pose_.covariance_ = infinite_info_pose_covariance_;
          origin_tag->size_.covariance_ = tag_size_variance_;
          // Set information
          origin_tag->pose_.information_ = options_.infinite_information;
          // Set origin_tag_ pointer
          origin_tag_ = origin_tag;
          // Add a tag pose prior for this tag
          Pose3dNormalPrior* origin_tag_prior = tag_pose_priors_->next_mutable_element();
          origin_tag_prior->SetZero();
          origin_tag_prior->Create(&origin_tag->pose_);
          VLOG(1) << "Initiated tag map with origin tag id = " << tag_reading->tag_id;
        }  
      }
      
      // Check if this tag exists in the tag map. 
      anantak::StaticAprilTagState* tag = NULL;
      bool tag_exists = FindTagInTagMap(tag_reading->tag_id, &tag);
      
      if (tag_exists) {
        
        // Create a new collected camera pose
        anantak::Pose3dState* collected_cam_pose = cam_poses_collected_->next_mutable_element();
        collected_cam_pose->SetZero();
        
        // Calculate camera pose for the tag if not already estimated
        if (!tag_reading->pose_has_been_estimated)
          tag_reading->EstimateCameraToTagPose(&cam_intrinsics, tag->size_.state_);
        tag_reading->CalculateCameraPoseFromTagPose(&(tag->pose_), collected_cam_pose);
        
        //std::cout << "Collected camera pose rdng # " << i_rdng << " = \n";
        //std::cout << *collected_cam_pose << "\n";
        //std::cout << "  From tag pose " << tag->tag_id_ << " = \n";
        //std::cout << tag->pose_ << "\n";
        
      } else {
        // This is a new tag. This will be created after camera pose has been calculated.
        // Nothing to do here.
        //VLOG(1) << "Saw a new tag = " << tag_reading->tag_id;
      }
      
    } // for each tag reading
    
    // Check if any camera pose was seen
    if (cam_poses_collected_->n_msgs()==0) {
      VLOG(1) << "All tags seen in image were not present in the tag map. Cannot calculate camera pose.";
      return false;
    }
    
    // Average out the collected camera poses
    else {
      // Initiate cam_pose so that readings can be added to it
      cam_pose->SetZero();
      Eigen::Quaterniond CiqT0 = Eigen::Quaterniond::Identity();
      Eigen::Vector3d T0pCi = Eigen::Vector3d::Zero();
      cam_pose->Create(&CiqT0, &T0pCi);
      cam_pose->covariance_ = zero_info_pose_covariance_;
      cam_pose->information_ = options_.zero_information;
      
      // Add each collected reading to the camera pose
      for (int i=0; i<cam_poses_collected_->n_msgs(); i++) {
        UpdatePose3dUsingInformation(cam_poses_collected_->at_ptr(i), cam_pose);
      }
      
      // Report the camera pose state
      //VLOG(1) << "\nCalculated camera pose = ";
      //std::cout << *cam_pose << "\n";
    }
    
    // Update all tags using the newly calculated camera pose
    for (int i_rdng=0; i_rdng<april_tag_readings_->n_msgs(); i_rdng++) {
      AprilTagReadingType* tag_reading = april_tag_readings_->at_ptr(i_rdng);
      
      // Check if this tag exists in the tag map. 
      anantak::StaticAprilTagState* tag = NULL;
      bool tag_exists = FindTagInTagMap(tag_reading->tag_id, &tag);
      
      // If not there, add a new tag with zero information
      if (!tag_exists) {
        
        // Make sure that the tag size is available
        double tag_size = FindTagSize(tag_reading->tag_id);
        if (tag_size<Epsilon) {
          // Skip this tag
          LOG(INFO) << "Could not find tag size, skipping this tag. tag_id = " << tag_reading->tag_id;
        } else {
          // Create a new tag in the tag map
          tag = tag_poses_->next_mutable_element();
          Eigen::Quaterniond TjqT0 = Eigen::Quaterniond::Identity();
          Eigen::Vector3d T0pTj = Eigen::Vector3d::Zero();
          tag->SetZero();
          tag->Create(&tag_reading->tag_id, &TjqT0, &T0pTj, &tag_size);
          // Set covariances
          tag->pose_.covariance_ = zero_info_pose_covariance_;
          tag->size_.covariance_ = tag_size_variance_;
          // Set information
          tag->pose_.information_ = options_.zero_information;
          // Add a tag pose prior for this tag
          Pose3dNormalPrior* tag_prior = tag_pose_priors_->next_mutable_element();
          tag_prior->SetZero();
          tag_prior->Create(&tag->pose_);
          VLOG(1) << "Added a tag to map with id = " << tag_reading->tag_id;
        }
        
      } // if !tag_exists
      
      // Estimate pose of new tags. But estimate existing tags only if asked.
      if (reestimate_existing_tags || (!reestimate_existing_tags && !tag_exists)) {
        
        // Use the camera pose to calculate a 'collected' tag pose
        anantak::Pose3dState collected_tag_pose;
        collected_tag_pose.SetZero();
        if (!tag_reading->pose_has_been_estimated)
          tag_reading->EstimateCameraToTagPose(&cam_intrinsics, tag->size_.state_);
        tag_reading->CalculateTagPoseFromCameraPose(cam_pose, &collected_tag_pose);
        
        //std::cout << "Tag: " << tag->tag_id_;
        //std::cout << "  starting tag pose = \n" << tag->pose_ << "\n";
        //std::cout << "  tag reading pose = \n" << *tag_reading << "\n";
        
        // Update tag pose in tag map
        UpdatePose3dUsingInformation(&collected_tag_pose, &(tag->pose_));
        
        // Report the tag pose calculated
        //std::cout << "  collected tag pose = \n" << collected_tag_pose << "\n";
        //std::cout << "  updated tag pose = \n" << tag->pose_ << "\n";
      } 
    }
    
    return true;
  }
  
  // Extract priors for each tag in tag map from the solved problem
  bool CalculatePriors(ceres::Problem* problem, bool detailed=false) {
    
    // Check
    if (tag_poses_->n_msgs() <= 1) {
      LOG(INFO) << "There are <=1 tags in tagmap. There is no covariance to calculate!";
      return false;
    }
    if (tag_poses_->n_msgs() != tag_pose_priors_->n_msgs()) {
      LOG(ERROR) << "tag_poses_->n_msgs() != tag_pose_priors_->n_msgs(). Should not be."
          << tag_poses_->n_msgs() << ", " << tag_pose_priors_->n_msgs();
      return false;
    }
    
    // Calculate covariance for each tag pose
    ceres::Covariance::Options covariance_options;
    ceres::Covariance covariance(covariance_options);
    std::vector<std::pair<const double*, const double*>> covariance_blocks;
    
    // Leave the first tag as it is the origin tag
    for (int i=1; i<tag_poses_->n_msgs(); i++) {
      double* tag_pose_error_ptr = tag_poses_->at(i).pose_.error_;
      if (problem->HasParameterBlock(tag_pose_error_ptr)) {
        covariance_blocks.push_back(std::make_pair(tag_pose_error_ptr, tag_pose_error_ptr));
      }
    }
    
    // Compute the covariance matricess
    bool success = covariance.Compute(covariance_blocks, problem);
    if (!success) {
      LOG(ERROR) << "Could not compute covariance matrices";
      return false;
    }
    
    // Leave the first tag as it is the origin tag
    for (int i=1; i<tag_poses_->n_msgs(); i++) {
      double* tag_pose_error_ptr = tag_poses_->at(i).pose_.error_;
      if (problem->HasParameterBlock(tag_pose_error_ptr)) {
        
        // Get the covariance matrix from the problem
        double* tag_pose_cov_ptr = tag_poses_->at(i).pose_.covariance_.data();
        covariance.GetCovarianceBlock(tag_pose_error_ptr, tag_pose_error_ptr, tag_pose_cov_ptr);
        
        // Update priors with new covariance matrices
        if (!tag_pose_priors_->at(i).Create(&tag_poses_->at(i).pose_, detailed)) {
          LOG(ERROR) << "Could not update prior for tag i = " << i << " name = "
              << tag_poses_->at(i).tag_id_ << ". Continue with rest.";
        }
        
      }
    }
    
    return true;
  }
  
  // Add tag pose priors to a problem
  bool AddPriors(ceres::Problem* problem, bool set_origin_fixed = true) {
    
    // Add priors
    int32_t n_tpp = 0;
    for (int i=0; i<tag_pose_priors_->n_msgs(); i++) {
      anantak::Pose3dNormalPrior* tag_pose_prior = tag_pose_priors_->at_ptr(i);
      ceres::CostFunction* tag_pose_prior_cf = tag_pose_prior;
      problem->AddResidualBlock(
        tag_pose_prior_cf, NULL,
        tag_pose_prior->pose_->error_
      );
      n_tpp++;
    }
    VLOG(1) << "Added " << n_tpp << " tag pose priors to problem";
    
    // Set Tag0 pose as constant - this forms reference for the tag map
    if (set_origin_fixed) {
      problem->SetParameterBlockConstant(origin_tag_->pose_.error_);
      VLOG(1) << "Set origin tag as fixed";
    }
    
    return true;
  }
  
  // Report priors
  bool ReportPriors() {
    for (int i=0; i<tag_pose_priors_->n_msgs(); i++) {
      VLOG(1) << "Tag pose prior for tag = " << tag_poses_->at(i).tag_id_ << "\n"
          << tag_pose_priors_->at(i);
    }
    return false;
  }
  
  // Recalculate all static tag poses
  bool Recalculate() {
    for (int i=0; i<tag_poses_->n_msgs(); i++) tag_poses_->at(i).Recalculate();
    return true;
  }
  
  // Save tag poses and sizes to a file
  bool SaveToFile(const std::string& save_filename, const std::string& predicate) {
    // Create a matrix with all tags data and save
    Eigen::Matrix<double,8,Eigen::Dynamic> tags_mat;
    int32_t num_tags = tag_poses_->n_msgs();
    tags_mat.resize(8,num_tags);
    for (int i=0; i<num_tags; i++) {
      Eigen::Map<Eigen::Matrix<double,7,1>> tag_pose(tag_poses_->at(i).pose_.state_);
      tags_mat.block<7,1>(0,i) = tag_pose;
      tags_mat(7,i) = tag_poses_->at(i).size_.Value();
    }
    anantak::WriteMatrixToCSVFile(save_filename+"."+predicate+".tagposes", tags_mat.transpose());
    return true;
  }
  
  bool SaveTagMapToFile(const std::string& save_filename, const std::string& predicate) {
    return SaveToFile(save_filename, predicate);
  }
  
};  // StaticAprilTagsMap


struct ImuReadingType {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  int64_t timestamp;
  QuaternionType quaternion;
  Vector3dType acceleration;
  ImuReadingType(): timestamp(0), quaternion(Eigen::Quaterniond::Identity()),
      acceleration(Eigen::Vector3d::Zero()) {}
  ImuReadingType(const ImuReadingType& rdng): timestamp(rdng.timestamp),
      quaternion(rdng.quaternion), acceleration(rdng.acceleration) {}
  ImuReadingType& operator= (const ImuReadingType& rdng) {
    timestamp = rdng.timestamp;
    quaternion = rdng.quaternion;
    acceleration = rdng.acceleration;
  }
  // This constructor is used to create a reading by rotating an existing one to a new frame
  // Reading is WqI. We need I0qI. I0qI = I0qW * WqI. So we premultiply the given rotation.
  ImuReadingType(const ImuReadingType& rdng, const QuaternionType& I0qW) :
      timestamp(rdng.timestamp), quaternion(rdng.quaternion), acceleration(rdng.acceleration) {
    quaternion = I0qW * quaternion;
  }
  bool SetZero() {timestamp=0; quaternion=Eigen::Quaterniond::Identity();
      acceleration=Eigen::Vector3d::Zero();}
};
std::ostream& operator<<(std::ostream& os, const ImuReadingType& rdng) {
  return os << rdng.timestamp << ", " << rdng.quaternion.coeffs().transpose() << ", "
      << rdng.acceleration.transpose();
}


bool InterpolateImuReading(const ImuReadingType* r0, const ImuReadingType* r1, ImuReadingType* r) {
  if (r->timestamp <= 0) {
    LOG(ERROR) << "r.timestamp <= 0. Please initiate timestamp of target reading";
    return false;
  }
  if (r1->timestamp <= r0->timestamp) {
    LOG(ERROR) << "r1.timestamp <= r0.timestamp. can not interpolate!";
    return false;
  }
  double frac = double(r->timestamp - r0->timestamp) / double(r1->timestamp - r0->timestamp);
  r->quaternion = InterpolateQuaternionsLinearly(r0->quaternion, r1->quaternion, frac);
  r->acceleration = (1.0-frac)*r0->acceleration + (frac)*r1->acceleration;
  return true;
}

/** Interpolate IMU readings */
bool InterpolateImuReadings(
    const std::vector<ImuReadingType>& ref,       /**< Imu readings reference */
    const std::vector<int64_t>& timestamps,       /**< Timestamps for interpolation */
    std::vector<LinearInterpolation>* interp,     /**< Interpolation type */
    std::vector<ImuReadingType>* interp_readings  /**< Interpolated readings */
    ) {
  // Interpolate timestamps
  (*interp).clear();
  InterpolateTimestamps<ImuReadingType>(ref, timestamps, interp);
  int32_t num_ref = ref.size();
  QuaternionType unitq;
  interp_readings->clear();
  for (int i=0; i<(*interp).size(); i++) {
    int32_t idx = (*interp)[i].index;
    double frac = (*interp)[i].fraction;
    ImuReadingType rdng;
    rdng.timestamp = timestamps[i];
    if (idx>-1 && idx<num_ref-1) {
      rdng.quaternion = InterpolateQuaternionsLinearly(
          ref[idx].quaternion, ref[idx+1].quaternion, frac);
      rdng.acceleration = (1.0-frac)*ref[idx].acceleration + (frac)*ref[idx+1].acceleration;
      /*VLOG_EVERY_N(1,10) << idx << " " << frac;
      VLOG_EVERY_N(1,10) << " interp q [" << ref[idx].quaternion.coeffs().transpose() << "] "
          << rdng.quaternion.coeffs().transpose() << " ["
          << ref[idx+1].quaternion.coeffs().transpose() << "]";
      VLOG_EVERY_N(1,10) << " interp a [" << ref[idx].acceleration.transpose() << "] "
          << rdng.acceleration.transpose() << " ["
          << ref[idx+1].acceleration.transpose() << "]";*/
    } else {
      rdng.quaternion = unitq;
      rdng.acceleration = Eigen::Vector3d::Zero();
    }
    interp_readings->push_back(rdng);
  }
  return true;
}  

bool InterpolateImuReadings(
    const anantak::CircularQueue<ImuReadingType>& ref,  /**< Imu readings reference */
    const std::vector<int64_t>& timestamps,             /**< Timestamps for interpolation */
    std::vector<LinearInterpolation>* interp,           /**< Interpolation type */
    std::vector<ImuReadingType>* interp_readings        /**< Interpolated readings */
    ) {
  // Interpolate timestamps
  //(*interp).clear(); // Not necessary as InterpolateTimestamps clears interp
  InterpolateTimestamps<ImuReadingType>(ref, timestamps, interp);
  const int32_t num_ref = ref.n_msgs();
  QuaternionType unitq;
  interp_readings->clear();
  for (int i=0; i<(*interp).size(); i++) {
    int32_t idx = (*interp)[i].index;
    double frac = (*interp)[i].fraction;
    ImuReadingType rdng;
    rdng.timestamp = timestamps[i];
    if (idx>-1 && idx<num_ref-1) {
      rdng.quaternion = InterpolateQuaternionsLinearly(
          ref.at(idx).quaternion, ref.at(idx+1).quaternion, frac);
      rdng.acceleration = (1.0-frac)*ref.at(idx).acceleration + (frac)*ref.at(idx+1).acceleration;
      /*VLOG_EVERY_N(1,10) << idx << " " << frac;
      VLOG_EVERY_N(1,10) << " interp q [" << ref.at(idx).quaternion.coeffs().transpose() << "] "
          << rdng.quaternion.coeffs().transpose() << " ["
          << ref.at(idx+1).quaternion.coeffs().transpose() << "]";
      VLOG_EVERY_N(1,10) << " interp a [" << ref.at(idx).acceleration.transpose() << "] "
          << rdng.acceleration.transpose() << " ["
          << ref.at(idx+1).acceleration.transpose() << "]";*/
    } else {
      rdng.quaternion = unitq;
      rdng.acceleration = Eigen::Vector3d::Zero();
    }
    interp_readings->push_back(rdng);
  }
  return true;
}  

struct ImuReadingsIntegralType {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  int64_t timestamps[2];    /**< Begin and end timestamps of the integration period */
  double dt;                /**< Integral period in seconds */
  Eigen::Matrix3d P;        /**< Double integral of rotation */
  Eigen::Matrix3d V;        /**< Integral of rotation */
  Eigen::Vector3d y;        /**< Double integral of (rotated) acceleration */
  Eigen::Vector3d s;        /**< Integral of (rotated) acceleration */
  // Constructors
  ImuReadingsIntegralType(): timestamps({0,0}), dt(0.),
    P(Eigen::Matrix3d::Zero()), V(Eigen::Matrix3d::Zero()), y(Eigen::Vector3d::Zero()),
    s(Eigen::Vector3d::Zero()) {}
  ImuReadingsIntegralType(const ImuReadingsIntegralType& iri):
    timestamps({iri.timestamps[0],iri.timestamps[1]}), dt(iri.dt), P(iri.P), V(iri.V), y(iri.y),
    s(iri.s) {}
  ImuReadingsIntegralType& operator= (const ImuReadingsIntegralType& iri) {
    timestamps[0] = iri.timestamps[0]; timestamps[1] = iri.timestamps[1];
    dt = iri.dt; P = iri.P; V = iri.V; y = iri.y; s = iri.s;
  }
  // Premultiply a 3x3 matrix - usually to change reference frame
  template<typename Mat3dType>
  bool LeftMultiplyRotationMatrix(const Eigen::MatrixBase<Mat3dType>& rotn) {
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Mat3dType, 3, 3)
    P = rotn*P;
    V = rotn*V;
    y = rotn*y;
    s = rotn*s;
    return true;
  }
};
std::ostream& operator<<(std::ostream& os, const ImuReadingsIntegralType& I) {
  return os << " dt = " << I.dt << "\n V = \n" << I.V << "\n P = \n" << I.P << "\n s = "
      << I.s.transpose() << "\n y = " << I.y.transpose();
}


/** Slerp-style interpolation to calculate [1/4,2/4,3/4] rotations between given quaternions */
template<typename Mat3dType>
bool QuadSlerp(
    const Eigen::Quaterniond& q1,         /**< Starting quaternion */
    const Eigen::Quaterniond& q2,         /**< Ending quaternion */
    Eigen::MatrixBase<Mat3dType>* m_25,   /**< Rotation matrix for 25% quaternion */
    Eigen::MatrixBase<Mat3dType>* m_50,   /**< Rotation matrix for 50% quaternion */
    Eigen::MatrixBase<Mat3dType>* m_75    /**< Rotation matrix for 75% quaternion */
    ) {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Mat3dType, 3, 3)
  double d = q1.dot(q2);
  double absD = std::abs(d);
  double t25 = 0.25; double t50 = 0.50; double t75 = 0.75;
  double scale025, scale125, scale050, scale150, scale075, scale175;
  
  if (absD >= OneLessEpsilon) {
    scale025 = double(1) - t25; scale125 = t25;
    scale050 = double(1) - t50; scale150 = t50;
    scale075 = double(1) - t75; scale175 = t75;
  } else {
    double theta = std::acos(absD);     // theta is the angle between the 2 quaternions
    double sinThetaRecip = double(1)/std::sin(theta);
    scale025 = std::sin((double(1) - t25) * theta) * sinThetaRecip;
    scale125 = std::sin((t25 * theta)) * sinThetaRecip;
    scale050 = std::sin((double(1) - t50) * theta) * sinThetaRecip;
    scale150 = std::sin((t50 * theta)) * sinThetaRecip;
    scale075 = std::sin((double(1) - t75) * theta) * sinThetaRecip;
    scale175 = std::sin((t75 * theta)) * sinThetaRecip;
  }
  if(d<0) { scale125 = -scale125; scale150 = -scale150; scale175 = -scale175; }
  
  *m_25 = Eigen::Quaterniond(scale025 * q1.coeffs() + scale125 * q2.coeffs()).toRotationMatrix();
  *m_50 = Eigen::Quaterniond(scale050 * q1.coeffs() + scale150 * q2.coeffs()).toRotationMatrix();
  *m_75 = Eigen::Quaterniond(scale075 * q1.coeffs() + scale175 * q2.coeffs()).toRotationMatrix();
  
  return true;
}

/** Interpolate vectors 3d to give three mid points */
template<typename Vec3dType>
bool QuadInterp(
    const Eigen::MatrixBase<Vec3dType>& v1,
    const Eigen::MatrixBase<Vec3dType>& v2,
    Eigen::MatrixBase<Vec3dType>* v_25,   /**< Vector for 25% */
    Eigen::MatrixBase<Vec3dType>* v_50,   /**< Vector for 50% */
    Eigen::MatrixBase<Vec3dType>* v_75    /**< Vector for 75% */      
    ) {
  *v_25 = 0.75 * v1 + 0.25 * v2;
  *v_50 = 0.50 * v1 + 0.50 * v2;
  *v_75 = 0.25 * v1 + 0.75 * v2;
  return true;
}

/** Integrate rotations and accelerations using numerical integration
 * Rotations are assumed to be in the same reference frame WqIi or I0qIi
 * Accelerations are assumed to be in the imu frame, Iia. So these will be rotated to W or Ii.
 * Kinematics assume rotations with constant angular velocity and accelerations are linear
 * Inputs are:
 *    vector of timestamps [0..n-1] - last timestamp is used for d_time calculation
 *    vector of rotations [0..n-1] - last rotation is used for d_rotation calculation
 *    vector of accelerations a[0..n-1] - last acceleration is only used for d_accel calculation
 *    [i,j] are starting and ending indexes representing [0..n-1] in existing vectors
 * Integration follows Simpson's rule  Integral(f()) ~= h/3*(f[0] + 4*f[1] + f[2])
 */
bool IntegrateImuKinematics(
    const std::vector<ImuReadingType>& imu_readings,
    const ImuReadingType& starting,
    const ImuReadingType& ending,
    const int32_t& start_idx,                  /**< Starting index in imu readings */
    const int32_t& end_idx,                    /**< Ending index in imu readings */
    const double& accel_factor,                /**< accel = reading * factor */
    Eigen::Matrix3d* V,         /**< Integral(rotation, dt) */
    Eigen::Matrix3d* P,         /**< Double_Integral(rotation, dt) */
    Eigen::Vector3d* s,         /**< Integral(rotation*acceleration, dt) */
    Eigen::Vector3d* y          /**< Double_Integral(rotation*acceleration, dt) */
    ) {
  
  // Basic checks
  if (starting.timestamp > ending.timestamp) {
    LOG(ERROR) << "starting.timestamp > ending.timestamp! " << starting.timestamp << " "
        << ending.timestamp;
    return false;
  }
  if (starting.timestamp > imu_readings[start_idx].timestamp) {
    LOG(ERROR) << "starting.timestamp > imu_readings[start_idx].timestamp " << starting.timestamp
        << " " << imu_readings[start_idx].timestamp;
    return false;
  }
  if (ending.timestamp < imu_readings[end_idx].timestamp) {
    LOG(ERROR) << "ending.timestamp < imu_readings[end_idx].timestamp " << ending.timestamp
        << " " << imu_readings[end_idx].timestamp;
    return false;
  }
  
  int32_t n_intervals = 0;
  if (start_idx > end_idx) {
    n_intervals = 1;
  } else {
    n_intervals = 2 + end_idx - start_idx;
  }
  
  /* For every time interval, calculate 3 mid-point quaternions - this assumes constant angvelo. 
   * Convert each quaternion to its matrix form, then all operations are matrix operations.
   * Calculate the rotation matrices and acceleration vectors. Transfer to reference frame.
   * Integrate. */
  
  // Initiate the integrals to zero
  *V = Eigen::Matrix3d::Zero();
  *P = Eigen::Matrix3d::Zero();
  *s = Eigen::Vector3d::Zero();
  *y = Eigen::Vector3d::Zero();
  
  // Integration variables
  const ImuReadingType *reading0, *reading1;
  double dt;
  Eigen::Matrix3d rotn00, rotn25, rotn50, rotn75;
  Eigen::Vector3d accl00, accl25, accl50, accl75;
  Eigen::Matrix3d rotn100, irotn_half1, irotn_half2, V_mid, V_end, iirotn;
  Eigen::Vector3d accl100, iaccl_half1, iaccl_half2, s_mid, s_end, iiaccl;
  
  reading0 = &starting;
  rotn00 = reading0->quaternion.toRotationMatrix();
  accl00 = rotn00 * reading0->acceleration;   // accl00 is now in reference frame
  
  for (int32_t i_intvl=0; i_intvl<n_intervals; i_intvl++) {
    
    if (i_intvl==n_intervals-1) {reading1 = &ending;}
    else {reading1 = &imu_readings[i_intvl+start_idx];}
    
    dt = double(reading1->timestamp - reading0->timestamp)*1e-6;
    if (dt<0) {
      LOG(ERROR) << "dt<0!" << dt << " " << reading0->timestamp << " " << reading1->timestamp;
      return false;
    }
    
    // Interpolate rotations assuming constant angular velocity and accelerations linearly
    QuadSlerp(reading0->quaternion, reading1->quaternion, &rotn25, &rotn50, &rotn75);
    QuadInterp(reading0->acceleration, reading1->acceleration, &accl25, &accl50, &accl75);
    
    rotn100 = reading1->quaternion.toRotationMatrix();
    accl25 = rotn25 * accl25; // accl25 is now in reference frame
    accl50 = rotn50 * accl50; // accl50 is now in reference frame
    accl75 = rotn75 * accl75; // accl75 is now in reference frame
    accl100 = rotn100 * reading1->acceleration;  // accl100 is now in reference frame
    
    // Use Simpson's rule to integrate in two half intervals for single integral
    double dt_by_12 = dt / 12.0;
    irotn_half1 = dt_by_12 * (rotn00 + 4.0*rotn25 + rotn50);
    irotn_half2 = dt_by_12 * (rotn50 + 4.0*rotn75 + rotn100);
    iaccl_half1 = dt_by_12 * (accl00 + 4.0*accl25 + accl50);
    iaccl_half2 = dt_by_12 * (accl50 + 4.0*accl75 + accl100);
    
    V_mid = irotn_half1;
    V_end = V_mid + irotn_half2;
    s_mid = iaccl_half1;
    s_end = s_mid + iaccl_half2;
    
    // Use Simpson's rule to integrate for double integral
    iirotn = 2.0 * dt_by_12 * (4.0*V_mid + V_end);
    iiaccl = 2.0 * dt_by_12 * (4.0*s_mid + s_end);
    
    *V += V_end;
    *s += s_end;
    *P += iirotn;
    *y += iiaccl;
    
    // Transfer last rotn and accl to beginning for the next iteration
    reading0 = reading1;
    rotn00 = rotn100;
    accl00 = accl100;
  }
  
  *s *= accel_factor;
  *y *= accel_factor;
  
  return true;
}

bool IntegrateImuKinematics(
    const std::vector<ImuReadingType>& imu_readings,
    const int32_t& start_idx,                  /**< Starting index in imu readings */
    const int32_t& end_idx,                    /**< Ending index in imu readings */
    const double& accel_factor,                /**< accel = reading * factor */
    Eigen::Matrix3d* V,         /**< Integral(rotation, dt) */
    Eigen::Matrix3d* P,         /**< Double_Integral(rotation, dt) */
    Eigen::Vector3d* s,         /**< Integral(rotation*acceleration, dt) */
    Eigen::Vector3d* y          /**< Double_Integral(rotation*acceleration, dt) */
    ) {
  return IntegrateImuKinematics(
    imu_readings,
    imu_readings[start_idx],
    imu_readings[end_idx],
    start_idx+1,
    end_idx-1,
    accel_factor,
    V, P, s, y
  );
}

bool IntegrateImuKinematics(
    const std::vector<ImuReadingType>& imu_readings,
    const ImuReadingType& starting,
    const ImuReadingType& ending,
    const int32_t& start_idx,                  /**< Starting index in imu readings */
    const int32_t& end_idx,                    /**< Ending index in imu readings */
    const double& accel_factor,                /**< accel = reading * factor */
    ImuReadingsIntegralType* integral_type
    ) {
  integral_type->timestamps[0] = starting.timestamp;
  integral_type->timestamps[1] = ending.timestamp;
  integral_type->dt = double(ending.timestamp - starting.timestamp)*1e-6;
  return IntegrateImuKinematics(
    imu_readings,
    starting,
    ending,
    start_idx,
    end_idx,
    accel_factor,
    &integral_type->V, &integral_type->P, &integral_type->s, &integral_type->y
  );
}

bool IntegrateImuKinematics(
    const std::vector<ImuReadingType>& imu_readings,
    const int32_t& start_idx,                  /**< Starting index in imu readings */
    const int32_t& end_idx,                    /**< Ending index in imu readings */
    const double& accel_factor,                /**< accel = reading * factor */
    ImuReadingsIntegralType* integral_type
    ) {
  return IntegrateImuKinematics(
    imu_readings,
    imu_readings[start_idx],
    imu_readings[end_idx],
    start_idx+1,
    end_idx-1,
    accel_factor,
    integral_type
  );
}

bool IntegrateImuReading(
    const std::vector<ImuReadingType>& imu_readings,
    const int32_t& idx,                  /**< Starting index in imu readings */
    const double& accel_factor,          /**< accel = reading * factor */
    const double* gravity,
    const double* accel_biases,
    const double* position_0,
    const double* velocity_0,
    double* position_1,
    double* velocity_1
    ) {
  Eigen::Map<const Eigen::Vector3d> grav(gravity);
  Eigen::Map<const Eigen::Vector3d> abias(accel_biases);
  Eigen::Map<const Eigen::Vector3d> posn0(position_0);
  Eigen::Map<const Eigen::Vector3d> velo0(velocity_0);
  Eigen::Map<Eigen::Vector3d> posn1(position_1);
  Eigen::Map<Eigen::Vector3d> velo1(velocity_1);
  
  ImuReadingsIntegralType ri;
  IntegrateImuKinematics(imu_readings, idx, idx+1, accel_factor, &ri);
  
  posn1 = posn0 + ri.dt*velo0 + 0.5*ri.dt*ri.dt*grav + ri.y - ri.P*abias;
  velo1 = velo0 + ri.dt*grav + ri.s - ri.V*abias;
  
  return true;
}


/** ConvexImuResidualFunction
 *  Keeps Imu rotations constant. Assumes a constant angular velocity between readings.
 *  Parameters are:
 *    0 Position[i] - Vector3d - all velo/posn in base frame
 *    1 Position[i+1] - Vector3d
 *    2 Velocity[i] - Vector3d
 *    3 Velocity[i+1] - Vector3d
 *    4 Gravity  - Vector3d - in world frame
 *    5 Accelerometer biases - Vector3d - in IMU body frame
 *  Inputs are:
 *    P matrix -  All integrals are assumed to be in the Base frame
 *    V matrix -
 *    s vector - input value already contains estimated gravity magnitude
 *    y vector -
 *    We are keeping calculation of above integrals out of this class. Other design could be
 *    that we provide the image timestamps and imu readings to this. But this is more flexible.
 *  Used to estimate imu starting poses and accelerometer bias */
class ConvexImuResidualFunction : public ceres::SizedCostFunction<7,3,3,3,3,3,3> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  typedef Eigen::Map<const Eigen::Vector3d> MapVector3dConstType;
  typedef Eigen::Map<Eigen::Matrix<double,7,1>> MapVector7dType;
  typedef Eigen::Map<Eigen::Matrix<double,7,3,Eigen::RowMajor>> MapMatrix7x3Type;
  
  ConvexImuResidualFunction(
      const Eigen::Matrix3d *P, const Eigen::Matrix3d *V,
      const Eigen::Vector3d *y, const Eigen::Vector3d *s,
      const double& dt, const double& g_mag,
      const double& sigma_a, const double& sigma_g ) :
    dt_(dt), g_mag_(g_mag), dt2_by_2_(0.5*dt*dt), g_mag2_(g_mag*g_mag),
    sigma_a_(sigma_a), sigma_g_(sigma_g),
    I3_(Eigen::Matrix3d::Identity()), world_R_base_(Eigen::Matrix3d::Identity()) {
    P_ = P; V_ = V; y_ = y; s_ = s;
    Initialize();
  }
  ConvexImuResidualFunction(
      const ImuReadingsIntegralType *I, const double& g_mag,
      const double& sigma_a, const double& sigma_g ) :
    dt_(I->dt), g_mag_(g_mag), dt2_by_2_(0.5*I->dt*I->dt), g_mag2_(g_mag*g_mag),
    sigma_a_(sigma_a), sigma_g_(sigma_g),
    I3_(Eigen::Matrix3d::Identity()), world_R_base_(Eigen::Matrix3d::Identity()) {
    P_ = &I->P; V_ = &I->V; y_ = &I->y; s_ = &I->s;
    Initialize();
  }
  ConvexImuResidualFunction(
      const ImuReadingsIntegralType *I,
      const Eigen::Matrix3d *world_R_base,   // rotation of starting pose in world/global frame
      const double& g_mag, const double& sigma_a, const double& sigma_g ) :
    dt_(I->dt), g_mag_(g_mag), dt2_by_2_(0.5*I->dt*I->dt), g_mag2_(g_mag*g_mag),
    sigma_a_(sigma_a), sigma_g_(sigma_g),
    I3_(Eigen::Matrix3d::Identity()), world_R_base_(*world_R_base) {
    P_ = &I->P; V_ = &I->V; y_ = &I->y; s_ = &I->s;
    Initialize();
  }
  
  bool Initialize() {
    dt_I3_ = dt_ * I3_; dt2_by_2_I3_ = dt2_by_2_ * I3_;
    double sqrt_dt, sqrt_dt2by2; sqrt_dt = std::sqrt(dt_); sqrt_dt2by2 = std::sqrt(dt2_by_2_);
    double sigma_p, sigma_v; sigma_v = sigma_a_*sqrt_dt; sigma_p = sigma_a_*sqrt_dt2by2;
    inv_sigma_ << sigma_p, sigma_p, sigma_p, sigma_v, sigma_v, sigma_v, sigma_g_;
    inv_sigma_ = inv_sigma_.cwiseInverse();
    //VLOG(1) << "inv_sigma = " << inv_sigma_.transpose();
    BrW_ = world_R_base_.transpose();
    
    // Precalculate jacobians
    dr_dposn0_.setZero();
    dr_dposn0_.block<3,3>(0,0) = I3_ / sigma_p;
    dr_dposn1_.setZero();
    dr_dposn1_.block<3,3>(0,0) = -I3_ / sigma_p;
    dr_dvelo0_.setZero();
    dr_dvelo0_.block<3,3>(0,0) = dt_I3_ / sigma_p;
    dr_dvelo0_.block<3,3>(3,0) = I3_ / sigma_v;
    dr_dvelo1_.setZero();
    dr_dvelo1_.block<3,3>(3,0) = -I3_ / sigma_v;
    dr_dgrav_.setZero();
    dr_dgrav_.block<3,3>(0,0) = dt2_by_2_I3_ * BrW_ / sigma_p;
    dr_dgrav_.block<3,3>(3,0) = dt_I3_ * BrW_ / sigma_v;
    //dr_dgrav_.block<1,3>(6,0) << 2.*grav.transpose()/sigma_g_; // depends on the grav estimate
    dr_dbias_.setZero();
    dr_dbias_.block<3,3>(0,0) = -(*P_) / sigma_p;
    dr_dbias_.block<3,3>(3,0) = -(*V_) / sigma_v;
    
    return true;
  }
  
  virtual ~ConvexImuResidualFunction() {}
  
  virtual bool Evaluate(
      double const* const* parameters,
      double* residuals,
      double** jacobians) const {
    
    MapVector3dConstType posn0(parameters[0]);    // All velo/posn are in Base frame
    MapVector3dConstType posn1(parameters[1]);
    MapVector3dConstType velo0(parameters[2]);
    MapVector3dConstType velo1(parameters[3]);    
    MapVector3dConstType  grav(parameters[4]);    // Gravity is in world frame
    MapVector3dConstType  bias(parameters[5]);    // Accel biases are in IMU body frame
    MapVector7dType resid(residuals);
    
    Eigen::Vector3d grav_base = BrW_*grav;        // grav_base is in base frame
    resid.block<3,1>(0,0) = posn0 + dt_*velo0 + dt2_by_2_*grav_base + (*y_) - (*P_)*bias - posn1;
    resid.block<3,1>(3,0) = velo0 + dt_*grav_base + (*s_) - (*V_)*bias - velo1;
    resid(6) = grav_base.squaredNorm() - g_mag2_;
    resid = resid.cwiseProduct(inv_sigma_);
    
    if (jacobians != NULL) {
      if (jacobians[0] != NULL) {
        MapMatrix7x3Type dr_dposn0(jacobians[0]);
        dr_dposn0 = dr_dposn0_;
      }
      if (jacobians[1] != NULL) {
        MapMatrix7x3Type dr_dposn1(jacobians[1]);
        dr_dposn1 = dr_dposn1_;
      }
      if (jacobians[2] != NULL) {
        MapMatrix7x3Type dr_dvelo0(jacobians[2]);
        dr_dvelo0 = dr_dvelo0_;
      }
      if (jacobians[3] != NULL) {
        MapMatrix7x3Type dr_dvelo1(jacobians[3]);
        dr_dvelo1 = dr_dvelo1_;
      }
      if (jacobians[4] != NULL) {
        MapMatrix7x3Type dr_dgrav(jacobians[4]);
        dr_dgrav = dr_dgrav_;
        dr_dgrav.block<1,3>(6,0) << 2.*grav.transpose()*inv_sigma_[6];
      }
      if (jacobians[5] != NULL) {
        MapMatrix7x3Type dr_dbias(jacobians[5]);
        dr_dbias = dr_dbias_;
      }
    }
    
    return true;
  } // Evaluate
  
 protected:
  const Eigen::Matrix3d *P_, *V_;   /**< V = Integral(rotation), P = Integral(V) */
  const Eigen::Vector3d *y_, *s_;   /**< s = Integral(rotated_acceleration), y = Integral(s) */
  const double dt_, dt2_by_2_;      /**< dt = time of this interval */
  const double g_mag_, g_mag2_;     /**< Gravity magnitude and its square */
  const double sigma_a_, sigma_g_;  /**< Accelerometer noise stdev, gravity_mag^2 noise stdev */
  Eigen::Matrix3d I3_, dt_I3_, dt2_by_2_I3_;
  Eigen::Matrix<double,7,1> inv_sigma_;
  // Precalculated jacobians - in convex formulation most coefficients are constant.
  Eigen::Matrix<double,7,3> dr_dposn0_, dr_dposn1_, dr_dvelo0_, dr_dvelo1_, dr_dgrav_, dr_dbias_;
  Eigen::Matrix3d world_R_base_, BrW_;
}; // ConvexImuResidualFunction

// Same as above, but with a scale added for imu accel readings. Should be close to 1.
class ConvexImuResidualFunctionScaled : public ceres::SizedCostFunction<7,3,3,3,3,3,3,1> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  typedef Eigen::Map<const Eigen::Vector3d> MapVector3dConstType;
  typedef Eigen::Map<Eigen::Matrix<double,7,1>> MapVector7dType;
  typedef Eigen::Map<Eigen::Matrix<double,7,3,Eigen::RowMajor>> MapMatrix7x3Type;
  
  ConvexImuResidualFunctionScaled(
      const Eigen::Matrix3d *P, const Eigen::Matrix3d *V,
      const Eigen::Vector3d *y, const Eigen::Vector3d *s,
      const double& dt, const double& g_mag,
      const double& sigma_a, const double& sigma_g ) :
    dt_(dt), g_mag_(g_mag), dt2_by_2_(0.5*dt*dt), g_mag2_(g_mag*g_mag),
    sigma_a_(sigma_a), sigma_g_(sigma_g),
    I3_(Eigen::Matrix3d::Identity()), world_R_base_(Eigen::Matrix3d::Identity()) {
    P_ = P; V_ = V; y_ = y; s_ = s;
    Initialize();
  }
  ConvexImuResidualFunctionScaled(
      const ImuReadingsIntegralType *I, const double& g_mag,
      const double& sigma_a, const double& sigma_g ) :
    dt_(I->dt), g_mag_(g_mag), dt2_by_2_(0.5*I->dt*I->dt), g_mag2_(g_mag*g_mag),
    sigma_a_(sigma_a), sigma_g_(sigma_g),
    I3_(Eigen::Matrix3d::Identity()), world_R_base_(Eigen::Matrix3d::Identity()) {
    P_ = &I->P; V_ = &I->V; y_ = &I->y; s_ = &I->s;
    Initialize();
  }
  ConvexImuResidualFunctionScaled(
      const ImuReadingsIntegralType *I,
      const Eigen::Matrix3d *world_R_base,   // rotation of starting pose in world/global frame
      const double& g_mag, const double& sigma_a, const double& sigma_g ) :
    dt_(I->dt), g_mag_(g_mag), dt2_by_2_(0.5*I->dt*I->dt), g_mag2_(g_mag*g_mag),
    sigma_a_(sigma_a), sigma_g_(sigma_g),
    I3_(Eigen::Matrix3d::Identity()), world_R_base_(*world_R_base) {
    P_ = &I->P; V_ = &I->V; y_ = &I->y; s_ = &I->s;
    Initialize();
  }
  
  bool Initialize() {
    dt_I3_ = dt_ * I3_; dt2_by_2_I3_ = dt2_by_2_ * I3_;
    double sqrt_dt, sqrt_dt2by2; sqrt_dt = std::sqrt(dt_); sqrt_dt2by2 = std::sqrt(dt2_by_2_);
    double sigma_p, sigma_v; sigma_v = sigma_a_*sqrt_dt; sigma_p = sigma_a_*sqrt_dt2by2;
    inv_sigma_ << sigma_p, sigma_p, sigma_p, sigma_v, sigma_v, sigma_v, sigma_g_;
    inv_sigma_ = inv_sigma_.cwiseInverse();
    //VLOG(1) << "inv_sigma = " << inv_sigma_.transpose();
    BrW_ = world_R_base_.transpose();
    
    // Precalculate jacobians
    dr_dposn0_.setZero();
    dr_dposn0_.block<3,3>(0,0) = I3_ / sigma_p;
    dr_dposn1_.setZero();
    dr_dposn1_.block<3,3>(0,0) = -I3_ / sigma_p;
    dr_dvelo0_.setZero();
    dr_dvelo0_.block<3,3>(0,0) = dt_I3_ / sigma_p;
    dr_dvelo0_.block<3,3>(3,0) = I3_ / sigma_v;
    dr_dvelo1_.setZero();
    dr_dvelo1_.block<3,3>(3,0) = -I3_ / sigma_v;
    dr_dgrav_.setZero();
    dr_dgrav_.block<3,3>(0,0) = dt2_by_2_I3_ * BrW_ / sigma_p;
    dr_dgrav_.block<3,3>(3,0) = dt_I3_ * BrW_ / sigma_v;
    //dr_dgrav_.block<1,3>(6,0) << 2.*grav.transpose()/sigma_g_; // depends on the grav estimate
    dr_dbias_.setZero();
    dr_dbias_.block<3,3>(0,0) = -(*P_) / sigma_p;
    dr_dbias_.block<3,3>(3,0) = -(*V_) / sigma_v;
    
    dr_dscale_.setZero();
    dr_dscale_.block<3,1>(0,0) = (*y_) / sigma_p;
    dr_dscale_.block<3,1>(3,0) = (*s_) / sigma_v;
    
    return true;
  }
  
  virtual ~ConvexImuResidualFunctionScaled() {}
  
  virtual bool Evaluate(
      double const* const* parameters,
      double* residuals,
      double** jacobians) const {
    
    MapVector3dConstType posn0(parameters[0]);    // All velo/posn are in Base frame
    MapVector3dConstType posn1(parameters[1]);
    MapVector3dConstType velo0(parameters[2]);
    MapVector3dConstType velo1(parameters[3]);    
    MapVector3dConstType  grav(parameters[4]);    // Gravity is in world frame
    MapVector3dConstType  bias(parameters[5]);    // Accel biases are in IMU body frame
    const double scale = parameters[6][0];
    MapVector7dType resid(residuals);
    
    Eigen::Vector3d grav_base = BrW_*grav;        // grav_base is in base frame
    resid.block<3,1>(0,0) = posn0 + dt_*velo0 + dt2_by_2_*grav_base + scale*(*y_) - (*P_)*bias - posn1;
    resid.block<3,1>(3,0) = velo0 + dt_*grav_base + scale*(*s_) - (*V_)*bias - velo1;
    resid(6) = grav_base.squaredNorm() - g_mag2_;
    resid = resid.cwiseProduct(inv_sigma_);
    
    if (jacobians != NULL) {
      if (jacobians[0] != NULL) {
        MapMatrix7x3Type dr_dposn0(jacobians[0]);
        dr_dposn0 = dr_dposn0_;
      }
      if (jacobians[1] != NULL) {
        MapMatrix7x3Type dr_dposn1(jacobians[1]);
        dr_dposn1 = dr_dposn1_;
      }
      if (jacobians[2] != NULL) {
        MapMatrix7x3Type dr_dvelo0(jacobians[2]);
        dr_dvelo0 = dr_dvelo0_;
      }
      if (jacobians[3] != NULL) {
        MapMatrix7x3Type dr_dvelo1(jacobians[3]);
        dr_dvelo1 = dr_dvelo1_;
      }
      if (jacobians[4] != NULL) {
        MapMatrix7x3Type dr_dgrav(jacobians[4]);
        dr_dgrav = dr_dgrav_;
        dr_dgrav.block<1,3>(6,0) << 2.*grav.transpose()*inv_sigma_[6];
      }
      if (jacobians[5] != NULL) {
        MapMatrix7x3Type dr_dbias(jacobians[5]);
        dr_dbias = dr_dbias_;
      }
      if (jacobians[6] != NULL) {
        MapVector7dType dr_dscale(jacobians[6]);
        dr_dscale = dr_dscale_;
      }
    }
    
    return true;
  } // Evaluate
  
 protected:
  const Eigen::Matrix3d *P_, *V_;   /**< V = Integral(rotation), P = Integral(V) */
  const Eigen::Vector3d *y_, *s_;   /**< s = Integral(rotated_acceleration), y = Integral(s) */
  const double dt_, dt2_by_2_;      /**< dt = time of this interval */
  const double g_mag_, g_mag2_;     /**< Gravity magnitude and its square */
  const double sigma_a_, sigma_g_;  /**< Accelerometer noise stdev, gravity_mag^2 noise stdev */
  Eigen::Matrix3d I3_, dt_I3_, dt2_by_2_I3_;
  Eigen::Matrix<double,7,1> inv_sigma_;
  // Precalculated jacobians - in convex formulation most coefficients are constant.
  Eigen::Matrix<double,7,3> dr_dposn0_, dr_dposn1_, dr_dvelo0_, dr_dvelo1_, dr_dgrav_, dr_dbias_;
  Eigen::Matrix<double,7,1> dr_dscale_;
  Eigen::Matrix3d world_R_base_, BrW_;
}; // ConvexImuResidualFunctionScaled

/* Functions for full VIO */

// Options to be supplied for IMU integration - this may be generated from a protobuf
struct ImuIntegrationOptions {
  double accel_to_gravity_multiplier;  // gravity in m/s^2 = multiplier * accel reading
  double gravity_magnitude;     // gravity in m/s^2
  double accel_factor; // accelerometer reading * accel_factor gives acceleration in m/s^2
  double small_angle_threshold; // radians. If angle is less, use small angle approximation
  // Noise terms - variance of noise
  double qr;  // Gyroscope measurement noise
  double qa;  // Accelerometer measurement noise
  double qwg; // Gyro bias drift process noise
  double qwa; // Accel bias drift process noise
  double qg;  // Gravity magnitude measurement noise
  
  // default constructor
  ImuIntegrationOptions() {
    accel_to_gravity_multiplier = 8192.;  // LSB/gravity
    gravity_magnitude = 9.8;  // m/s^2
    small_angle_threshold = 0.002; // radians
    accel_factor = gravity_magnitude / accel_to_gravity_multiplier; // m/s^2/LSB
    
    double grav_mag_lb = 9.7; // m/s^2 - lower bound of gravity mag
    double grav_mag_ub = 9.9; // m/s^2 - upper bound of gravity mag
    double grav_range_stdev = 2.0; // range of stdev's between grav ub and lb
    
    double sigma_gravity = (grav_mag_ub - grav_mag_lb)/grav_range_stdev;
    double sigma_accel = 400.0*1e-6*gravity_magnitude; // m/s^2/sqrt(Hz) from the datasheet
    double sigma_gyro = 5.*1e-3*RadiansPerDegree; // rad/s/sqrt(Hz) from the datasheet
    
    // Noise stdev
    qr = sigma_gyro;
    qa = sigma_accel;
    qwg = sigma_gyro*1e-1;    // datasheet does not provide bias stability information. So this is a guess.
    qwa = sigma_accel*1e-1;   // datasheet does not provide bias stability information. So this is a guess.
    qg = sigma_gravity;
    
    // Converting to variance from stdev
    qr *= qr;
    qa *= qa;
    qwg *= qwg;
    qwa *= qwa;
    qg *= qg;
    
  }
  // implicit copy constructor should be available
  // implicit equal assign operator should be available
};  // ImuIntegrationOptions

/* Imu state
 * Implements methods for an imu state.
 */
class ImuState : public State {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef Eigen::Map<Eigen::Vector3d> MapVector3dType;
  typedef Eigen::Map<Eigen::Vector4d> MapVector4dType;
  typedef Eigen::Map<Eigen::Matrix<double,16,1>> MapVector16dType;
  typedef Eigen::Map<Eigen::Matrix<double,15,1>> MapVector15dType;
  
  // Default constructor
  ImuState(): State(), IaG_(state_),
    IqvG_(state_),  GpI_(state_+4),  GvI_(state_+7),  bg_(state_+10),  ba_(state_+13),
    dGqI_(error_), dGpI_(error_+3), dGvI_(error_+6), dbg_(error_+9),  dba_(error_+12) {
    SetZero();
  }
  
  // Set to zero
  bool SetZero() {
    timestamp_ = 0;
    MapVector16dType s(state_);
    MapVector15dType e(error_);
    s.setZero(); s[3] = 1.;
    e.setZero();
    return true;
  }
  
  // Set Error state to zero
  bool SetErrorZero() {
    MapVector15dType e(error_);
    e.setZero();
    return true;
  }
  
  // Is this zero?
  bool IsZero() const {
    return (IaG_.isZero() && GpI_.isZero() && GvI_.isZero() && bg_.isZero() && ba_.isZero());
  }
  
  // Set timestamp
  bool SetTimestamp(const int64_t& ts) {
    timestamp_ = ts;
    return true;
  }
  
  // Recalculate the state from error state after optimization. Set errors to zero.
  bool Recalculate() {
    Eigen::Quaterniond IqG(IqvG_.data()); // x,y,z,w
    Eigen::Quaterniond dGqI = anantak::ErrorAngleAxisToQuaterion(dGqI_);
    IqG *= dGqI;  // assuming IqvG_ (and hence IqG) are already normalized
    IqvG_ = IqG.coeffs();
    GpI_ += dGpI_;
    GvI_ += dGvI_;
    bg_ += dbg_;
    ba_ += dba_;
    SetErrorZero();
    return true;
  }
  
  // Helpers that send copies of state values
  Eigen::Quaterniond Quaternion() const {
    return Eigen::Quaterniond(IqvG_.data()); // x,y,z,w
  }
  Eigen::Vector3d Position() const {
    return Eigen::Vector3d(GpI_);
  }
  
  // Destructor
  virtual ~ImuState() {}
  
  // Timestamp
  int64_t timestamp_;
  // State
  double state_[16];
  // Error
  double error_[15];
  
  // Helper maps
  MapVector4dType IqvG_; // quaternion as a vector x,y,z,w
  MapVector3dType IaG_;
  MapVector3dType GpI_;
  MapVector3dType GvI_;
  MapVector3dType bg_;
  MapVector3dType ba_;
  MapVector3dType dGqI_; // angleaxis as a vector
  MapVector3dType dGpI_;
  MapVector3dType dGvI_;
  MapVector3dType dbg_;
  MapVector3dType dba_;
}; // ImuState


struct ImuEstimatesIntegralType {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef Eigen::Map<Eigen::Vector3d> MapVector3dType;
  typedef Eigen::Map<Eigen::Vector4d> MapVector4dType;
  typedef Eigen::Map<const Eigen::Vector3d> MapConstVector3dType;
  typedef Eigen::Map<const Eigen::Vector4d> MapConstVector4dType;
  
  // Starting state
  int64_t ts0;
  Eigen::Quaterniond r0;    /**< Rotation in form GqL. G=reference frame, L=local frame */
  Eigen::Vector3d p0;
  Eigen::Vector3d v0;
  Eigen::Vector3d bg0;
  Eigen::Vector3d ba0;
  Eigen::Vector3d g0;
  double td0;               /**< Time delay */
  
  // Ending state
  int64_t ts1;
  Eigen::Quaterniond r1;    /**< Rotation in form GqL. G=reference frame, L=local frame */
  Eigen::Vector3d p1;
  Eigen::Vector3d v1;
  Eigen::Vector3d bg1;
  Eigen::Vector3d ba1;
  Eigen::Vector3d g1;
  double td1;               /**< Time delay */
  
  // Estimate integrals
  double dt;                /**< Integral period in seconds */
  Eigen::Matrix3d P;        /**< Double integral of rotation */
  Eigen::Matrix3d V;        /**< Integral of rotation */
  Eigen::Vector3d y;        /**< Double integral of rotated (acceleration - accel_bias) */
  Eigen::Vector3d s;        /**< Integral of rotated (acceleration - accel_bias) */
  Eigen::Matrix3d M;        /**< Integral( SkewSymm(rotated(acceleration - accel_bias))*integ(rotn) ) */
  Eigen::Matrix3d MM;       /**< Integral of M */
  
  // Noise covariance matrices
  Eigen::Matrix3d N0;       /**< Noise in rotation */
  Eigen::Matrix3d N1;       /**< Noise in acceleration */
  Eigen::Matrix3d N2;       /**< Noise due to M */
  Eigen::Matrix3d N3;       /**< Noise due to gravity uncertainty */
  Eigen::Matrix3d N1N1;     /**< Integral of N1 */
  Eigen::Matrix3d N2N2;     /**< Integral of N2 */
  Eigen::Matrix3d N3N3;     /**< Integral of N3 */
  Eigen::Matrix3d N4;       /**< Integral of N4 */
  Eigen::Matrix3d N5;       /**< Integral of N5 */
  
  // Constructor
  ImuEstimatesIntegralType(): ts0(0), ts1(0), td0(0.), td1(0.),
    r0(Eigen::Quaterniond::Identity()), p0(Eigen::Vector3d::Zero()), v0(Eigen::Vector3d::Zero()),
    bg0(Eigen::Vector3d::Zero()), ba0(Eigen::Vector3d::Zero()), g0(Eigen::Vector3d::Zero()),
    r1(Eigen::Quaterniond::Identity()), p1(Eigen::Vector3d::Zero()), v1(Eigen::Vector3d::Zero()),
    bg1(Eigen::Vector3d::Zero()), ba1(Eigen::Vector3d::Zero()), g1(Eigen::Vector3d::Zero()),
    dt(0.), P(Eigen::Matrix3d::Zero()), V(Eigen::Matrix3d::Zero()),
    y(Eigen::Vector3d::Zero()), s(Eigen::Vector3d::Zero()), M(Eigen::Matrix3d::Zero()),
    MM(Eigen::Matrix3d::Zero()), N0(Eigen::Matrix3d::Zero()), N1(Eigen::Matrix3d::Zero()),
    N2(Eigen::Matrix3d::Zero()), N3(Eigen::Matrix3d::Zero()), N1N1(Eigen::Matrix3d::Zero()),
    N2N2(Eigen::Matrix3d::Zero()), N3N3(Eigen::Matrix3d::Zero()),
    N4(Eigen::Matrix3d::Zero()), N5(Eigen::Matrix3d::Zero()) {}
    
  // Set to zero
  bool SetZero() { ts0=0; ts1=0; td0=0.; td1=0.;
    r0=Eigen::Quaterniond::Identity(); p0=Eigen::Vector3d::Zero(); v0=Eigen::Vector3d::Zero();
    bg0=Eigen::Vector3d::Zero(); ba0=Eigen::Vector3d::Zero(); g0=Eigen::Vector3d::Zero();
    r1=Eigen::Quaterniond::Identity(); p1=Eigen::Vector3d::Zero(); v1=Eigen::Vector3d::Zero();
    bg1=Eigen::Vector3d::Zero(); ba1=Eigen::Vector3d::Zero(); g1=Eigen::Vector3d::Zero();
    dt=0.; P=Eigen::Matrix3d::Zero(); V=Eigen::Matrix3d::Zero();
    y=Eigen::Vector3d::Zero(); s=Eigen::Vector3d::Zero(); M=Eigen::Matrix3d::Zero();
    MM=Eigen::Matrix3d::Zero(); N0=Eigen::Matrix3d::Zero(); N1=Eigen::Matrix3d::Zero();
    N2=Eigen::Matrix3d::Zero(); N3=Eigen::Matrix3d::Zero(); N1N1=Eigen::Matrix3d::Zero();
    N2N2=Eigen::Matrix3d::Zero(); N3N3=Eigen::Matrix3d::Zero();
    N4=Eigen::Matrix3d::Zero(); N5=Eigen::Matrix3d::Zero();
    return true;
  }
  
  // Set from a state
  bool SetFromState(const ImuState& state, const Vector3dState& grav) {
    // Set starting state
    ts0 = state.timestamp_;
    r0.coeffs() << -state.IqvG_[0], -state.IqvG_[1], -state.IqvG_[2], state.IqvG_[3];  // conjugate as we need GqI
    p0 = state.GpI_; v0 = state.GvI_; bg0 = state.bg_; ba0 = state.ba_;
    g0 = grav.Gp_; td0 = 0.;
    // Set ending state
    ts1 = ts0; r1 = r0; p1 = p0; v1 = v0; bg1 = bg0; ba1 = ba0; g1 = g0; td1 = td0;
    // All integrals are set to zero
    dt=0.; P=Eigen::Matrix3d::Zero(); V=Eigen::Matrix3d::Zero();
    y=Eigen::Vector3d::Zero(); s=Eigen::Vector3d::Zero(); M=Eigen::Matrix3d::Zero();
    MM=Eigen::Matrix3d::Zero(); N0=Eigen::Matrix3d::Zero(); N1=Eigen::Matrix3d::Zero();
    N2=Eigen::Matrix3d::Zero(); N3=Eigen::Matrix3d::Zero(); N1N1=Eigen::Matrix3d::Zero();
    N2N2=Eigen::Matrix3d::Zero(); N3N3=Eigen::Matrix3d::Zero();
    N4=Eigen::Matrix3d::Zero(); N5=Eigen::Matrix3d::Zero();
    return true;
  }
  
  // Copy ending state to given states
  bool CopyEndingStateTo(ImuState* state) {
    // State's timestamp is not modified.
    state->IqvG_ = r1.conjugate().coeffs();   // r1 is GqI. State is IqG vector form
    state->GpI_ = p1; state->GvI_ = v1; state->bg_ = bg1; state->ba_ = ba1;
    // td1 copy - TODO to be done later
    return true;
  }
  
  // Premultiply a 3x3 matrix - usually to change the reference frame
  template<typename Mat3dType>
  bool LeftMultiplyRotationMatrix(const Eigen::MatrixBase<Mat3dType>& rotn) {
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Mat3dType, 3, 3)
    P = rotn*P; V = rotn*V; y = rotn*y; s = rotn*s; M = rotn*M; MM = rotn*MM;
    // Quaternion and rotation both follow same multiplication sequence
    Eigen::Quaterniond rotn_q(rotn); r0 = rotn_q*r0; r1 = rotn_q*r1;
    p0 = rotn*p0; v0 = rotn*v0; g0 = rotn*g0;
    p1 = rotn*p1; v1 = rotn*v1; g1 = rotn*g1;
    // Biases are in body frame and do not rotate
    // Noises are assumed to be spherical, so rotation will not change their covariance matrix
    // Time delays do not change
    return true;
  }
  
  // Rescale by a factor - usually to linearly (approximately) interpolate
  bool ScaleByFactor(const double& f) {
    dt*=f; P*=f; V*=f; y*=f; s*=f; M*=f; MM*=f;
    N0*=f; N1*=f; N2*=f; N3*=f; N1N1*=f; N2N2*=f; N3N3*=f; N4*=f; N5*=f;
    double f1 = 1. - f;
    ts1 = int64_t(ts0*f + ts1*f1);
    r1.slerp(f, r0); // r0 remains the same
    p1 = p0*f1 + p1*f;
    v1 = v0*f1 + v1*f;
    bg1 = bg0*f1 + bg1*f;
    ba1 = ba0*f1 + ba1*f;
    g1 = g0*f1 + g1*f;
    // Time delays do not change?
    return true;
  }
  
  // Add another integral at the end
  bool AddIntegral(const ImuEstimatesIntegralType& i) {
    dt+=i.dt; P+=i.P; V+=i.V; y+=i.y; s+=i.s; M+=i.M; MM+=i.MM;
    N0+=i.N0; N1+=i.N1; N2+=i.N2; N3+=i.N3; N1N1+=i.N1N1; N2N2+=i.N2N2; N3N3+=i.N3N3; N4+=i.N4; N5+=i.N5; 
    // Assumes that i.r0 = r, meaning starting rotation of i is ending rotation of this integral
    // Rotations are GqL. So the rotation to be added is L1qL2 and this is GqL1. We need GqL2.
    // GqL1 is i.r0 and GqL2 is i.r1 so L1qL2 = GqL1^-1 * GqL2 = i.r0^-1 * i.r1
    ts1 += (i.ts1 - i.ts0);
    r1 *= (i.r0.conjugate() * i.r1); // r0 remains the same
    p1 += (i.p1 - i.p0);
    v1 += (i.v1 - i.v0);
    bg1 += (i.bg1 - i.bg0);
    ba1 += (i.ba1 - i.ba0);
    g1 += (i.g1 - i.g0);
    // Time delays do not change?
    return true;
  }
  
  // Interpolate provided integral linearly and populate this one
  bool SetUsingInterpolation(const ImuEstimatesIntegralType& i0, const int64_t& ts) {
    if (ts<i0.ts0 || ts>i0.ts1) return false; // is this needed?
    
    // fractions used for interpolation
    double f0 = double(ts-i0.ts0)/double(i0.ts1-i0.ts0);
    double f1 = 1. - f0;
    
    // Set starting state
    ts0=i0.ts0; r0=i0.r0; p0=i0.p0; v0=i0.v0; bg0=i0.bg0; ba0=i0.ba0; g0=i0.g0; td0=i0.td0;
    
    // Set ending state using interpolation
    ts1 = ts;
    r1  = i0.r0; r1.slerp(f0,i0.r1);
    p1  = f1*i0.p0  + f0*i0.p1 ;
    v1  = f1*i0.v0  + f0*i0.v1 ;
    bg1 = f1*i0.bg0 + f0*i0.bg1;
    ba1 = f1*i0.ba0 + f0*i0.ba1;
    g1  = f1*i0.g0  + f0*i0.g1 ;
    td1 = f1*i0.td0 + f0*i0.td1;
    
    // Set integrals using 0 as starting point as all integrals are 0 at ts0
    dt   = f0*i0.dt;
    P    = f0*i0.P;
    V    = f0*i0.V;
    y    = f0*i0.y;
    s    = f0*i0.s;
    M    = f0*i0.M;
    MM   = f0*i0.MM;
    N0   = f0*i0.N0;
    N1   = f0*i0.N1;
    N2   = f0*i0.N2;
    N3   = f0*i0.N3;
    N1N1 = f0*i0.N1N1;
    N2N2 = f0*i0.N2N2;
    N3N3 = f0*i0.N3N3;
    N4   = f0*i0.N4;
    N5   = f0*i0.N5;
    
    // TODO - Add interpolation noise
    
    return true;
  }
  
  // Interpolate between endings of provided integrals linearly and populate this one
  bool SetUsingInterpolation(const ImuEstimatesIntegralType& i0,
        const ImuEstimatesIntegralType& i1, const int64_t& ts) {
    if (ts<i0.ts1 || ts>i1.ts1) return false; // is this needed?
    
    if (abs(double(i0.ts0-i1.ts0)) > Epsilon)
      LOG(WARNING) << "i0, i1 do not start at same timestamp. diff = " << i0.ts0-i1.ts0;
    
    // fractions used for interpolation
    double f0 = double(ts-i0.ts1)/double(i1.ts1-i0.ts1);
    double f1 = 1. - f0;
    
    // Set starting state using i0's starting state.
    // It is assumed that i1's starting state is the same as i0's
    ts0=i0.ts0; r0=i0.r0; p0=i0.p0; v0=i0.v0; bg0=i0.bg0; ba0=i0.ba0; g0=i0.g0; td0=i0.td0;
    
    // Set ending state using interpolation
    ts1 = ts;
    r1  = i0.r1; r1.slerp(f0,i1.r1);
    p1  = f1*i0.p1  + f0*i1.p1 ;
    v1  = f1*i0.v1  + f0*i1.v1 ;
    bg1 = f1*i0.bg1 + f0*i1.bg1;
    ba1 = f1*i0.ba1 + f0*i1.ba1;
    g1  = f1*i0.g1  + f0*i1.g1 ;
    td1 = f1*i0.td1 + f0*i1.td1;
    
    // Set integrals using interpolation from i0.ts1 to i1.ts1
    dt   = f1*i0.dt   + f0*i1.dt  ;
    P    = f1*i0.P    + f0*i1.P   ;
    V    = f1*i0.V    + f0*i1.V   ;
    y    = f1*i0.y    + f0*i1.y   ;
    s    = f1*i0.s    + f0*i1.s   ;
    M    = f1*i0.M    + f0*i1.M   ;
    MM   = f1*i0.MM   + f0*i1.MM  ;
    N0   = f1*i0.N0   + f0*i1.N0  ;
    N1   = f1*i0.N1   + f0*i1.N1  ;
    N2   = f1*i0.N2   + f0*i1.N2  ;
    N3   = f1*i0.N3   + f0*i1.N3  ;
    N1N1 = f1*i0.N1N1 + f0*i1.N1N1;
    N2N2 = f1*i0.N2N2 + f0*i1.N2N2;
    N3N3 = f1*i0.N3N3 + f0*i1.N3N3;
    N4   = f1*i0.N4   + f0*i1.N4  ;
    N5   = f1*i0.N5   + f0*i1.N5  ;
    
    // TODO - Add interpolation noise      
    
    return true;
  }
  
  // Calculate state noise variance matrix, negelecting all correlations
  bool StateNoiseVariance(Eigen::Matrix<double,15,1> *noise_cov) const {
    
    // Diagonals of the covariance matrices. All off-diagonal terms are zero.
    noise_cov->block<3,1>(0,0)  = N0.diagonal();
    noise_cov->block<3,1>(3,0)  = N1N1.diagonal() + N2N2.diagonal() + N3N3.diagonal();
    noise_cov->block<3,1>(6,0)  = N1.diagonal() + N2.diagonal() + N3.diagonal();
    noise_cov->block<3,1>(9,0)  = N4.diagonal(); //options_.qwg * dt * Eigen::Vector3d::Ones();
    noise_cov->block<3,1>(12,0) = N5.diagonal(); //options_.qwa * dt * Eigen::Vector3d::Ones();
    
    return true;
  }
};
std::ostream& operator<<(std::ostream& os, const ImuEstimatesIntegralType& I) {
  return os << " dt = " << I.dt << "\n V = \n" << I.V << "\n P = \n" << I.P << "\n s = "
      << I.s.transpose() << "\n y = " << I.y.transpose();
}

// Integrate IMU kinematics
bool IntegrateImuKinematics(
    const ImuReadingType& reading0,
    const ImuReadingType& reading1,
    const ImuIntegrationOptions& options,
    ImuEstimatesIntegralType* integrals) {
  
  // Basic checks
  if (reading0.timestamp > reading1.timestamp) {
    LOG(ERROR) << "reading0.timestamp > reading1.timestamp! " << reading0.timestamp << " "
        << reading1.timestamp;
    return false;
  }
  
  /* For every time interval, calculate 3 mid-point quaternions - this assumes constant angvelo. 
   * Convert each quaternion to its matrix form, then all operations are matrix operations.
   * Calculate the rotation matrices and acceleration vectors. Transfer to reference frame.
   * Integrate. */
  
  // Readings advance the information
  int64_t int64dt = reading1.timestamp - reading0.timestamp;
  double dt = double(int64dt)*1e-6;
  if (dt<0) {
    LOG(ERROR) << "dt<0! " << dt << " " << reading0.timestamp << " " << reading1.timestamp;
    return false;
  }
  integrals->dt += dt;
  integrals->ts1 += int64dt;
  
  // Values in integrals are assumed to be initialized.
  
  // Integration variables
  Eigen::Quaterniond quat00, quat100, I1qI11, I0qI1;
  Eigen::Vector3d gyro_bias_error;
  Eigen::Vector3d accl0_imu, accl1_imu;
  Eigen::Matrix3d rotn00, rotn25, rotn50, rotn75, rotn100;
  Eigen::Vector3d accl00, accl25, accl50, accl75, accl100;
  Eigen::Matrix3d irotn_half1, irotn_half2, irotn25, irotn50, irotn75, irotn100, iirotn;
  Eigen::Vector3d iaccl_half1, iaccl_half2, iaccl50, iaccl100, iiaccl;
  
  // Integration variables for M, MM
  Eigen::Matrix3d dM25, dM50, dM75, dM100; // dM00 = Zero;
  Eigen::Matrix3d M_half1, M_half2, M50, M100, iM;
  
  // Integration variables for noises
  Eigen::Matrix3d s_s_accl50, sqr_s_s_accl50_diag;
  
  // Begin integration from the current ending estimate
  quat00 = integrals->r1;
  rotn00 = quat00.toRotationMatrix();
  accl0_imu = reading0.acceleration*options.accel_factor - integrals->ba1;
  accl00 = rotn00 * accl0_imu;   // accl00 is now in reference frame
  
  // Calculate rotation adjusting for gyro biases
  gyro_bias_error = dt * integrals->bg1;
  I1qI11 = anantak::ErrorAngleAxisToQuaterion(gyro_bias_error, 1e-6); // bias correction
  // Readings are GqI0 and GqI1. Need GqI11 = GqI0 * I0qI1 * I1qI11. I0qI1 = GqI0^-1 * GqI1.
  I0qI1 = reading0.quaternion.conjugate() * reading1.quaternion;      // reading
  quat100 = quat00 * I0qI1 * I1qI11;    // reading + bias correction
  
  // Imu ending acceleration adjsted for biases
  accl1_imu = reading1.acceleration*options.accel_factor - integrals->ba1;
  
  // Interpolate rotations assuming constant angular velocity and accelerations linearly
  QuadSlerp(quat00, quat100, &rotn25, &rotn50, &rotn75);
  QuadInterp(accl0_imu, accl1_imu, &accl25, &accl50, &accl75);
  
  rotn100 = quat100.toRotationMatrix();
  accl25 = rotn25 * accl25; // accl25 is now in reference frame
  accl50 = rotn50 * accl50; // accl50 is now in reference frame
  accl75 = rotn75 * accl75; // accl75 is now in reference frame
  accl100 = rotn100 * accl1_imu;  // accl100 is now in reference frame
  
  // Use Simpson's rule to integrate in two half intervals for single integral
  double dt_by_12 = dt / 12.0;
  irotn_half1 = dt_by_12 * (rotn00 + 4.0*rotn25 + rotn50);
  irotn_half2 = dt_by_12 * (rotn50 + 4.0*rotn75 + rotn100);
  iaccl_half1 = dt_by_12 * (accl00 + 4.0*accl25 + accl50);
  iaccl_half2 = dt_by_12 * (accl50 + 4.0*accl75 + accl100);
  
  irotn25 = 0.5*irotn_half1;    // using linear interpolation here. OK if dt is small.
  irotn50 = irotn_half1;
  irotn75 = irotn_half1 + 0.5*irotn_half2;  // using linear interpolation as above.
  irotn100 = irotn_half1 + irotn_half2;
  iaccl50 = iaccl_half1;
  iaccl100 = iaccl_half1 + iaccl_half2;
  
  // Use Simpson's rule to integrate for double integral
  iirotn = 2.0 * dt_by_12 * (4.0*irotn50 + irotn100);
  iiaccl = 2.0 * dt_by_12 * (4.0*iaccl50 + iaccl100);
  
  integrals->V += irotn100;     // integral of rotation wrt time
  integrals->P += iirotn;       // double integral of rotation wrt time
  integrals->s += iaccl100;     // integral of acceleration wrt time
  integrals->y += iiaccl;       // double integral of acceleration wrt time
  
  // Integrating for M, MM
  s_s_accl50 = anantak::SkewSymmetricMatrix(accl50);
  dM25 = anantak::SkewSymmetricMatrix(accl25) * irotn25;
  dM50 = s_s_accl50 * irotn50;
  dM75 = anantak::SkewSymmetricMatrix(accl75) * irotn75;
  dM100 = anantak::SkewSymmetricMatrix(accl100) * irotn100;
  M_half1 = dt_by_12 * (       4.0*dM25 + dM50);
  M_half2 = dt_by_12 * (dM50 + 4.0*dM75 + dM100);
  M50 = M_half1;
  M100 = M_half1 + M_half2;
  iM = 2.0 * dt_by_12 * (4.0*M50 + M100);
  integrals->M  += M100;
  integrals->MM += iM;
  
  // Integrating for noises
  double dt2_by_2 = 0.5*dt*dt;
  double dt3_by_6 = dt*dt2_by_2/3.;
  double dt4_by_24 = 0.25*dt*dt3_by_6;
  
  double qn0_1 =       dt*options.qr +  dt2_by_2*options.qwg;
  double qn0_2 = dt2_by_2*options.qr +  dt3_by_6*options.qwg;
  double qn0_3 = dt3_by_6*options.qr + dt4_by_24*options.qwg;
  double qn1_1 =       dt*options.qa +  dt2_by_2*options.qwa;
  double qn1_2 = dt2_by_2*options.qa +  dt3_by_6*options.qwa;
  double qn3_1 =       dt*options.qg;
  double qn3_2 = dt2_by_2*options.qg;
  double qn4   =       dt*options.qwg;
  double qn5   =       dt*options.qwa;
  
  s_s_accl50 *= s_s_accl50.transpose();
  sqr_s_s_accl50_diag = s_s_accl50.diagonal().asDiagonal();
  
  integrals->N0 += qn0_1 * Eigen::Matrix3d::Identity();
  integrals->N1 += qn1_1 * Eigen::Matrix3d::Identity();
  integrals->N2 += qn0_2 * sqr_s_s_accl50_diag;  // Approximation to keep computations simple
  integrals->N3 += qn3_1 * Eigen::Matrix3d::Identity();
  integrals->N1N1 += qn1_2 * Eigen::Matrix3d::Identity();
  integrals->N2N2 += qn0_3 * sqr_s_s_accl50_diag;  // Approximation to keep computations simple
  integrals->N3N3 += qn3_2 * Eigen::Matrix3d::Identity();
  integrals->N4 += qn4 * Eigen::Matrix3d::Identity();
  integrals->N5 += qn5 * Eigen::Matrix3d::Identity();
  
  // Propagate state
  integrals->r1 = quat100;
  integrals->p1 += integrals->v1*dt + integrals->g1*dt2_by_2 + iiaccl;
      // uses starting velocity, so has to come before velocity is updated
  integrals->v1 += integrals->g1*dt + iaccl100;
  //integrals->bg1 = integrals->bg1;  // no change as per kinematic assumptions
  //integrals->ba1 = integrals->ba1;  // no change as per kinematic assumptions
  //integrals->g1 = integrals->g1;    // no change as per kinematic assumptions
  //integrals->td1 = integrals->td1;  // time delay errors are yet to be implemented
  
  return true;
}

/* ImuResidualFunction: used in sliding window optimization problems for IMU readings
 * IMU's state vector
 *    x = [ IqG, GpI, GvI, bg, ba ], [ Gg ]
 * IMU's error state vector
 *    dx = [ Gdq, Gdp, Gdv, dbg, dba ], [ Gdg ]
 * Solving is done using error states.
 * Residuals are calculated using states for positions, velocities and gravity.
 * Residuals are assumed to be Gaussian. Mean of each residual is the predicted value and the
 * Variance is estimated using linearized approximation of measurement/propagation equations.
 * Linearization expresses each residual as a linear function of error state vector
 *    Residual ~= Jacobian(x)*dx + linearization_error
 * Hessian is expressed from Jacobian that assumes we are already close to the solution
 *    Hessian ~= Jacobian(x) * Cov(dx) * Jacobian(x).transpose()
 * Kinematic assumptions:
 *    Linear accel - linearly varies between t0 and t1
 *    Angular velocity - remains constant between t0 and t1
 *    
 */
class ImuResidualFunction : public ceres::SizedCostFunction<15, 15,15,3> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef Eigen::Map<Eigen::Vector3d> MapVector3dType;
  typedef Eigen::Map<Eigen::Vector4d> MapVector4dType;
  typedef Eigen::Matrix<double,15,1> Vector15dType;
  typedef Eigen::Matrix<double,16,1> Vector16dType;
  typedef Eigen::Map<Eigen::Matrix<double,15,1>> MapVector15dType;
  typedef Eigen::Map<Eigen::Matrix<double,16,1>> MapVector16dType;
  typedef Eigen::Map<const Eigen::Matrix<double,15,1>> MapConstVector15dType;
  typedef Eigen::Map<const Eigen::Matrix<double, 3,1>> MapConstVector3dType;
  typedef Eigen::Matrix<double,16,15,Eigen::RowMajor> Matrix16x15RowType;
  typedef Eigen::Matrix<double,16, 3,Eigen::RowMajor> Matrix16x3RowType;
  typedef Eigen::Matrix<double,15,15,Eigen::RowMajor> Matrix15x15RowType;
  typedef Eigen::Matrix<double,15, 3,Eigen::RowMajor> Matrix15x3RowType;
  typedef Eigen::Map<Eigen::Matrix<double,15,15,Eigen::RowMajor>> MapMatrix15x15RowType;
  typedef Eigen::Map<Eigen::Matrix<double,15, 3,Eigen::RowMajor>> MapMatrix15x3RowType;
  typedef Eigen::Map<Eigen::Matrix<double,16,15,Eigen::RowMajor>> MapMatrix16x15RowType;
  typedef Eigen::Map<Eigen::Matrix<double,16, 3,Eigen::RowMajor>> MapMatrix16x3RowType;
  
  enum ResidualStateType {kOpen, kClosed};
  
  /* This defines connection between two error states due to IMU readings. At allocation time
   * this object just allocates uninitiated memory. At creation, it gets pointer to two states
   * that this constraint connects along with the starting imu reading. Repeatedly new readings
   * can be added to this as long as the readings lie between the two states' timestamps.
   * Before the optimization is to be done, a 'closing' integral that spans the last reading to
   * ending state is calculated and added to the integrals that connects the states.
   *
   * Owner class follows this route: Allocate memory in a circular queue for this class.
   * When new readings come in, ImuStates are created for the entire period. Then interpolation
   * of readings is done. New ImuResidualFunctions are Created. Next all readings are added
   * to the constraints in increasing order of time in this way: When we cross to next period
   * current state is 'GetReadyToOptimize'. This populates state1's estimates. Optimization
   * is done, this populates the error state. All states are recalculated. Marginalization is
   * done finally.
   *
   * When new data comes in, owner class checks for every reading: is the reading within the
   * optimization window? If not neglect it audibly. If so, is it before the ending timestamp of
   * last iteration? If not begin creating states. If so, add reading to existing state where
   * this reading belongs, while 'GetReadyToOptimize' when moving to the next readings interval.
   */
  
  // Options and settings
  ImuIntegrationOptions options_;
  bool options_have_been_set_;
  
  // States and readings
  ImuState* state0_;   /**< Beginning state for this constraint */
  ImuState* state1_;   /**< Ending state for this constraint */
  Vector3dState* gravity_; /**< Gravity state for this constraint */
  ImuReadingType last_integrated_reading_;  /**< Latest IMU reading in the residual */
  int32_t num_readings_;
  ResidualStateType residual_state_;
  
  // Integrals
  ImuEstimatesIntegralType integral_till_last_reading_;  /**< calc everytime a new reading comes */
  ImuEstimatesIntegralType integral_;  /**< complete integrals of readings from state0 to state1 */
  
  // Residual vector
  Vector15dType R_;
  
  // First estimates Jacobians
  Matrix15x15RowType dR_dS0_, dR_dS1_;
  Matrix15x3RowType  dR_dG_;
  Vector15dType inv_sqrt_var_;
  
  // Helpers for calculations
  double dt_;
  
  // Interpolation.
  // IMU residual is used to interpolate integrals for readings from other sensors. We keep a
  // history of integrals that will be used to perform interpolation. Only a max number of
  // integrals are kept in the memory. Beyond that last integral is used for interpolation.
  int32_t max_integrals_history_;  // no readings are stored by default
  int32_t num_integrals_stored_;   // zero to start with
  std::vector<ImuEstimatesIntegralType> integrals_history_; // size is max_integrals_history
  
  // Default constructor
  ImuResidualFunction(int32_t max_integrals_history=0):
    state0_(NULL), state1_(NULL), gravity_(NULL), last_integrated_reading_(),
    num_readings_(0), residual_state_(kOpen),
    integral_till_last_reading_(), integral_(),
    options_(), options_have_been_set_(false),
    dt_(0.),
    max_integrals_history_(max_integrals_history), num_integrals_stored_(0) {
    R_.setZero(); dR_dS0_.setZero(); dR_dS1_.setZero(); dR_dG_.setZero(); inv_sqrt_var_.setZero();
    integrals_history_.resize(max_integrals_history_);
  }
  
  ImuResidualFunction(const ImuResidualFunction& r):
    state0_(r.state0_), state1_(r.state1_), gravity_(r.gravity_),
    last_integrated_reading_(r.last_integrated_reading_),
    num_readings_(r.num_readings_), residual_state_(r.residual_state_),
    integral_till_last_reading_(r.integral_till_last_reading_), integral_(r.integral_),
    options_(r.options_), options_have_been_set_(r.options_have_been_set_),
    dt_(r.dt_), R_(r.R_), dR_dS0_(r.dR_dS0_), dR_dS1_(r.dR_dS1_), dR_dG_(r.dR_dG_),
    inv_sqrt_var_(r.inv_sqrt_var_),
    max_integrals_history_(r.max_integrals_history_),
    num_integrals_stored_(r.num_integrals_stored_),
    integrals_history_(r.integrals_history_) {}
  
  ImuResidualFunction& operator=(const ImuResidualFunction& r) {
    // check for self-assignment
    if(&r == this) return *this;
    state0_=r.state0_; state1_=r.state1_; gravity_=r.gravity_;
    last_integrated_reading_=r.last_integrated_reading_;
    num_readings_=r.num_readings_; residual_state_=r.residual_state_;
    integral_till_last_reading_=r.integral_till_last_reading_; integral_=r.integral_;
    options_=r.options_; options_have_been_set_=r.options_have_been_set_;
    dt_=r.dt_; R_=r.R_; dR_dS0_=r.dR_dS0_; dR_dS1_=r.dR_dS1_; dR_dG_=r.dR_dG_;
    inv_sqrt_var_=r.inv_sqrt_var_;
    max_integrals_history_=r.max_integrals_history_;
    num_integrals_stored_=r.num_integrals_stored_;
    integrals_history_=r.integrals_history_;
    return *this;
  }
  
  // Setup options - this is usually done once at allocation, but could be reset if needed
  bool SetOptions(const ImuIntegrationOptions& options) {
    options_ = options;  // copy, not sure if all options should be kept locally
    options_have_been_set_ = true;
    return true;
  }
  
  // Reset everything but options. We are preparing to reuse the allocated memory.
  bool Reset() {
    state0_=NULL; state1_=NULL; gravity_=NULL;
    last_integrated_reading_.SetZero();
    num_readings_ = 0; residual_state_ = kOpen;
    integral_till_last_reading_.SetZero();
    integral_.SetZero();
    R_.setZero(); dR_dS0_.setZero(); dR_dS1_.setZero(); dR_dG_.setZero(); inv_sqrt_var_.setZero();
    dt_ = 0.;
    num_integrals_stored_ = 0;
    // max_integrals_history_ does not change
    for (int i=0; i<max_integrals_history_; i++) {integrals_history_[i].SetZero();}
    return true;
  }
  
  // Setup the constraint with a starting state, gravity and reading.
  //  Ending state will be provided at closure.
  bool Create(ImuState* state0, Vector3dState* gravity, const ImuReadingType& rdng0) {
    // Make sure provided data makes sense
    if (state0->IsZero()) {
      LOG(ERROR) << "state0 is zero, can not initiate imu constraint from a zero state.";
      return false;
    }
    if (gravity->IsZero()) {
      LOG(ERROR) << "gravity is zero, can not initiate imu constraint from a zero gravity state.";
      return false;
    }
    if (state0->timestamp_ > rdng0.timestamp) { // this should have an adjustment for reading ts
      LOG(ERROR) << "state0 timestamp > reading0 timestamp. Not allowed. " << state0->timestamp_
          << ", " << rdng0.timestamp;
      return false;
    }
    
    // Reset the residual
    Reset();
    
    // Setup new data
    state0_ = state0; gravity_ = gravity;
    last_integrated_reading_ = rdng0; // copy reading
    // Setup integrals
    integral_till_last_reading_.SetFromState(*state0, *gravity);
    
    return true;
  }
  
  bool Create(ImuState* state0, Vector3dState* gravity, const ImuReadingType& rdng0,
      const ImuIntegrationOptions& options) {
    SetOptions(options);
    return Create(state0, gravity, rdng0);
  }
  
  // Setup the constraint with two flanking states and a starting reading
  bool Create(ImuState* state0, ImuState* state1, Vector3dState* gravity,
      const ImuReadingType& rdng0) {
    
    // Make sure provided data makes sense
    if (state0->timestamp_ >= state1->timestamp_) {
      LOG(ERROR) << "state0 timestamp >= state1 timestamp. Not allowed. " << state0->timestamp_
          << ", " << state1->timestamp_;
      return false;
    }
    
    if (!Create(state0, gravity, rdng0)) {
      return false;
    }
    
    state1_ = state1;
    
    // Other calculations
    dt_ = (state1_->timestamp_ - state0_->timestamp_)*1e-6;
    
    return true;
  }
  
  bool Create(ImuState* state0, ImuState* state1, Vector3dState* gravity,
      const ImuReadingType& rdng0, const ImuIntegrationOptions& options) {
    SetOptions(options);
    return Create(state0, state1, gravity, rdng0);
  }
  
  /*bool CreateData() {
    // Get the the state data pointers
    double* x0_ = state0_->state_;
    double* x1_ = state1_->state_;
    double* xg_ = gravity_->state_;
    
    // Changing the mapped arrays. Extract from Eigen documentation:
    // It is possible to change the array of a Map object after declaration, using the C++
    // "placement new" syntax. Despite appearances, this does not invoke the memory allocator,
    // because the syntax specifies the location for storing the result. This syntax makes it
    // possible to declare a Map object without first knowing the mapped array's location 
    
    // Current state vector
    new (&I0qG_) MapVector4dType(x0_);
    new (&GpI0_) MapVector3dType(x0_+4);  new (&GvI0_) MapVector3dType(x0_+7);
    new  (&bg0_) MapVector3dType(x0_+10); new  (&ba0_) MapVector3dType(x0_+13);
    // Next state vector      
    new (&I1qG_) MapVector4dType(x1_);
    new (&GpI1_) MapVector3dType(x1_+4);  new (&GvI1_) MapVector3dType(x1_+7);
    new  (&bg1_) MapVector3dType(x1_+10); new  (&ba1_) MapVector3dType(x1_+13);
    // Gravity vector      
    new (&Gg_) MapVector3dType(xg_);
    
    return true;
  }*/
  
  // Add a new reading
  bool AddReading(const ImuReadingType& rdng) {
    
    // Check the reading timestamp. Reading timestamp is changed by timedelay
    if (state1_) {
      if (rdng.timestamp <= last_integrated_reading_.timestamp ||
          rdng.timestamp + int64_t(integral_till_last_reading_.td0*1e6) > state1_->timestamp_) {
        LOG(ERROR) << "Can not add a reading before latest one or beyond end state. last_ts = "
            << last_integrated_reading_.timestamp << " reading.ts (delay) = " << rdng.timestamp
            << " (" << int64_t(integral_till_last_reading_.td0*1e6) << ")"
            << " end state ts = " << state1_->timestamp_;
        return false;
      }
    } else {
      if (rdng.timestamp <= last_integrated_reading_.timestamp) {
        LOG(ERROR) << "Can not add a reading before latest one "
            << last_integrated_reading_.timestamp;
        return false;
      }
    }
    
    // Calculate integrals from last_integrated_reading to this reading
    if (!IntegrateImuKinematics(last_integrated_reading_, rdng, options_,
                                &integral_till_last_reading_)) {
      LOG(ERROR) << "Could not integrate reading. Dropping it.";
      return false;
    }
    num_readings_++;
    
    // Update integral_till_last_reading_ 
    last_integrated_reading_ = rdng;  // copy
    
    // Store the latest integral into integrals history if history is to be kept
    // Integrals history is kept to allow interpolation of states for other sensors' readings
    if (max_integrals_history_ > 0) {
      if (num_integrals_stored_ < max_integrals_history_) {
        integrals_history_[num_integrals_stored_] = integral_till_last_reading_; // copy
        num_integrals_stored_++;
      }
    }
    
    return true;
  }
  
  // Calculates the last integral, full integral and propagates state if asked to.
  bool AddEndStateReading(const ImuReadingType& rdng, bool propagate_state = false) {
    
    // Make sure that state1 was provided when the constraint was created
    if (!state1_) {
      LOG(ERROR) << "Need to provide ending state. Use the other AddEndStateReading().";
      return false;
    }
    
    // Check if reading timestamp + delay matches the State1 timestamp?
    if (abs(rdng.timestamp + int64_t(integral_till_last_reading_.td0*1e6)
            - state1_->timestamp_)>10) {
      LOG(WARNING) << "Ending state reading does not match state1 timestamp " <<
          rdng.timestamp + int64_t(integral_till_last_reading_.td0*1e6) - state1_->timestamp_;
    }
    
    // Calculate integral_ for the whole interval
    // First make a copy of integral since state0_, then integrate to the end
    integral_ = integral_till_last_reading_;  // make a copy 
    if (!IntegrateImuKinematics(last_integrated_reading_, rdng, options_, &integral_)) {
      LOG(ERROR) << "Could not integrate last reading. Can not continue.";
      return false;
    }
    
    // Propagate state if asked to
    if (propagate_state) integral_.CopyEndingStateTo(state1_);
    
    // Calculate Jacobians from first estimates
    if (!CalculateStartingResidualandJacobians()) {
      return false;
    }
    
    residual_state_ = kClosed;
    
    return true;
  }
  
  // Add ending state reading with ending state provided
  bool AddEndStateReading(const ImuReadingType& rdng, ImuState* state1, bool propagate_state = false) {
    
    // Check is state1_ is already set
    if (state1_) {
      LOG(ERROR) << "Ending state is already provided. Can not provide again.";
      return false;
    }
    
    // Make sure provided data makes sense
    if (state0_->timestamp_ >= state1->timestamp_) {
      LOG(ERROR) << "state0 timestamp >= state1 timestamp. Not allowed. " << state0_->timestamp_
          << ", " << state1->timestamp_;
      return false;
    }
    
    state1_ = state1;
    
    // Other calculations
    dt_ = (state1_->timestamp_ - state0_->timestamp_)*1e-6;
    
    return AddEndStateReading(rdng, propagate_state);
  }
  
  // Interpolate integrals for other sensors' readings. Interpolation requires the following:
  // Residual must be closed, interpolation timestamp must be within the start/end ts of residual.
  bool InterpolateIntegrals(const int64_t& interp_ts, ImuEstimatesIntegralType* interp_integral)
      const {
    
    if (residual_state_ != kClosed) {
      LOG(ERROR) << "Cannot interpolate using an open residual.";
      return false;
    }
    
    if (interp_ts < integral_.ts0 || interp_ts > integral_.ts1) {
      LOG(ERROR) << "Interpolation timestamp is outside the residual interval. " << interp_ts
          << " is not in [" << integral_.ts0 << ", " << integral_.ts1 << "]";
      return false;
    }
    
    if (num_integrals_stored_ == 0) {
      bool ret = interp_integral->SetUsingInterpolation(integral_, interp_ts);
      if (!ret) LOG(ERROR) << "Could not interpolate integral.";
      return ret;
    }
    
    // Find where the interp_ts lies.
    int32_t interp_ts_interval = 0;
    bool interp_ts_interval_found = false;
    while (!interp_ts_interval_found && interp_ts_interval<num_integrals_stored_) {
      interp_ts_interval_found = (interp_ts <= integrals_history_[interp_ts_interval].ts1);
      if (!interp_ts_interval_found) interp_ts_interval++;
    }
    if (!interp_ts_interval_found) {
      interp_ts_interval_found = (interp_ts <= integral_.ts1);
      if (interp_ts_interval_found) interp_ts_interval++;
    }
    
    // Interpolate the given integral
    if (interp_ts_interval_found) {
      //VLOG(1) << "    Interpolation of integrals: found interval " << interp_ts_interval;
      if (interp_ts_interval==0) {
        interp_integral->SetUsingInterpolation(integrals_history_[0], interp_ts);
      } else
      if (interp_ts_interval>0 && interp_ts_interval<num_integrals_stored_) {
        interp_integral->SetUsingInterpolation(
            integrals_history_[interp_ts_interval-1],
            integrals_history_[interp_ts_interval],
            interp_ts);
      } else
      if (interp_ts_interval == num_integrals_stored_) {
        interp_integral->SetUsingInterpolation(
            integrals_history_[num_integrals_stored_-1],
            integral_,
            interp_ts);
      } else {
        LOG(ERROR) << "Strange, can not find interp_ts interval. " << interp_ts_interval << " "
            << num_integrals_stored_ << " It should have been here. Is it NAN/Inf? " << interp_ts;
        return false;
      }
    } else {
      LOG(ERROR) << "Something is wrong. Could not find interp_ts. Should have.";
      return false;
    }
    
    return true;
  }
  
  // Calculate error state Jacobians using initial estimates - kept constant during optimization. 
  bool CalculateStartingResidualandJacobians() {
    /* Error state Jacobians are d_residual / d_error_state. We assume that in the local vicinity
     * of current state residual can be approximated by a linear model of error states. We also
     * assume that starting state estimate is close to actual state. Under these assumptions
     * changes in residual can be calculated using jacobians, rather than full valuation of
     * non-linear function. Variance of the residual can then be expressed from the covariance
     * of the error states. */
    
    //MapMatrix16x15RowType dR_dS0_, dR_dS1_;
    //MapMatrix16x3RowType  dR_dG_;
    
    /* Residual defines the difference between state1 and state0 given the readings. We have used
     * readings to predict state1 from state0. By linearizing around the estimate we can specify
     * this residual as a function of error states. The 16-vector of residual is:
     *    r_16x1 = r_rotn r_posn r_velo r_bg r_ba r_g, 3+3+3+3+3+1 = 16
     * The error states are
     *    e_15x1 = e_rotn e_posn e_velo e_bg e_ba, 3+3+3+3+3 = 15
     */
    
    // Indexes of blocks - helpers
    const int rotn_idx = 0;
    const int posn_idx = 3;
    const int velo_idx = 6;
    const int bg_idx   = 9;
    const int ba_idx   = 12;
    //const int g_idx    = 15;      
    
    /* Usually residual is defined as residual = observation - prediction. Here we do the reverse.
     *  residual = prediction - observation
     * This is fine as long as we are consistent in valuing the residuals and they are gaussian.
     *  Maths here is in Notebook#4 pg77-78
     * Estimates values are stored as:
     *  Wq^Ii+1' = integral_.r1
     *  Wp^Ii+1' = integral_.p1
     *  Wv^Ii+1' = integral_.v1
     *  Ii+1'bg^ = integral_.bg1
     *  Ii+1'ba^ = integral_.ba1
     * State1 values are defined as:
     *  Ii+1q^W = state1_->Quaternion()
     *  Wp^Ii+1 = state1_->GpI_
     *  Wv^Ii+1 = state1_->GvI_
     *  Ii+1bg^ = state1_->bg_
     *  Ii+1ba^ = state1_->ba_
     * Negative residuals are defined in the following way:
     *   Ii+1'rIi+1 = Ii+1'rW * WrIi+1
     *  -WIi+1'pIi+1 = WpIi+1' - WpIi+1
     *  -WIi+1'vIi+1 = WvIi+1' - WvIi+1
     *  -Ii+1'bgIi+1 = Ii+1'bg - Ii+1bg   .. No rotation of bg/ba here as difference in vectors
     *  -Ii+1'baIi+1 = Ii+1'ba - Ii+1ba   .. is correctly considered across different ref frames
     * Negative residuals' are evaluated from their estimates and errors as:
     *   WIi+1'aIi+1 = WIi+1'a^Ii+1 + WIi+1'r~Ii+1    .. rotation is written as angle axis here
     *  -WIi+1'pIi+1 = -WIi+1'p^Ii+1 + -WIi+1'p~Ii+1
     *  -WIi+1'vIi+1 = -WIi+1'v^Ii+1 + -WIi+1'v~Ii+1
     *  -Ii+1'bgIi+1 = -Ii+1'bg^Ii+1 + -Ii+1'bg~Ii+1
     *  -Ii+1'baIi+1 = -Ii+1'ba^Ii+1 + -Ii+1'ba~Ii+1
     *  where
     *    Ii+1'aIi+1 = AngleAxis(Ii+1'rIi+1)
     *    Ii+1'a^Ii+1 = AngleAxis(Ii+1'r^Ii+1)
     *    WIi+1'aIi+1 = Wr^Ii+1' * Ii+1'aIi+1
     *    WIi+1'a^Ii+1 = Wr^Ii+1' * Ii+1'a^Ii+1
     *    WIi+1'r~Ii+1 = Wr^Ii+1' * Ii+1'r~Ii+1
     * Estimates of residuals come from above definitions:
     *   WIi+1'a^Ii+1 = Wr^Ii+1' * AngleAxis( Ii+1'r^W * Wr^Ii+1 )
     *  -WIi+1'p^Ii+1 = Wp^Ii+1' - Wp^Ii+1
     *  -WIi+1'v^Ii+1 = Wv^Ii+1' - Wv^Ii+1
     *  -Ii+1'bg^Ii+1 = Ii+1'bg^ - Ii+1bg^  .. No rotation of bg/ba here as difference in vectors
     *  -Ii+1'ba^Ii+1 = Ii+1'ba^ - Ii+1ba^  .. is correctly considered across different ref frames
     * Residual errors are:
     *   WIi+1'r~Ii+1 = Jacobian(r,x^Ii)*x~Ii - Wr~Ii+1 + Wr^Ii+1'*noises
     *  -WIi+1'p~Ii+1 = Jacobian(p,x^Ii)*x~Ii - Wp~Ii+1 + noises
     *  -WIi+1'v~Ii+1 = Jacobian(v,x^Ii)*x~Ii - Wv~Ii+1 + noises
     *  -Ii+1'bg~Ii+1 = Jacobian(bg,x^Ii)*x~Ii - Ii+1bg + noises
     *  -Ii+1'ba~Ii+1 = Jacobian(ba,x^Ii)*x~Ii - Ii+1ba + noises
     *  where
     *    Jacobian(*,x^Ii) is jacobian of variable * wrt state at Ii
     *    x^Ii = [ IiqW, WpIi, WvIi, Iibg Iiba ]
     *    x~Ii = [ Wr~Ii, Wp~Ii, Wv~Ii, Iibg~, Iiba~ ]
     */
    
    // Calculate starting residual from predicted value and ending state (state1)
    R_.setZero();
    Eigen::Quaterniond WqI1_ = integral_.r1;
    Eigen::Quaterniond WqI1  = state1_->Quaternion().conjugate();
    Eigen::Matrix3d WrI1_ = WqI1_.toRotationMatrix();
    Eigen::Quaterniond I1_qI1 = WqI1_.conjugate() * WqI1;
    /*Eigen::AngleAxisd I1_aI1(I1_qI1);
    double aa_angle = I1_aI1.angle();
    Eigen::Vector3d aa_axis = I1_aI1.axis();
    if (aa_angle <= -anantak::Pi_2) aa_angle += anantak::Pi_2;
    if (aa_angle >=  anantak::Pi_2) aa_angle -= anantak::Pi_2;
    if (aa_angle > anantak::Pi_half && aa_angle < anantak::Pi) {
      aa_angle = anantak::Pi - aa_angle;
      aa_axis *= -1.;
    } else if (aa_angle < -anantak::Pi_half && aa_angle > -anantak::Pi) {
      aa_angle = anantak::Pi + aa_angle;
      aa_axis *= -1.;
    } else if (aa_angle <= -anantak::Pi || aa_angle >= anantak::Pi) {
      LOG(ERROR) << "Angle axis is <= -anantak::Pi or >= anantak::Pi. Did not expect that."
          << aa_angle << " " << aa_axis.transpose();
      return false;
    }
    Eigen::Vector3d I1_avI1 =  aa_angle * aa_axis;*/
    Eigen::Vector3d I1_avI1 = 2.*I1_qI1.vec();    // approximation assuming small angle
    R_.block<3,1>(rotn_idx,0) = WrI1_ * I1_avI1;
    R_.block<3,1>(posn_idx,0) = integral_.p1 - state1_->GpI_;
    R_.block<3,1>(velo_idx,0) = integral_.v1 - state1_->GvI_;
    R_.block<3,1>(bg_idx  ,0) = integral_.bg1 - state1_->bg_;
    R_.block<3,1>(ba_idx  ,0) = integral_.ba1 - state1_->ba_;
    
    // Gravity residual
    //double grav_state_mag = gravity_->Gp_.norm();
    //R_(g_idx   ,0) = grav_state_mag - options_.gravity_magnitude;
    
    // Ending state
    //int64_t ts1;
    //Eigen::Quaterniond r1;    /**< Rotation in form GqL. G=reference frame, L=local frame */
    //Eigen::Vector3d p1;
    //Eigen::Vector3d v1;
    //Eigen::Vector3d bg1;
    //Eigen::Vector3d ba1;
    //Eigen::Vector3d g1;
    //double td1;               /**< Time delay */
    
    // Estimate integrals
    //double dt;                /**< Integral period in seconds */
    //Eigen::Matrix3d P;        /**< Double integral of rotation */
    //Eigen::Matrix3d V;        /**< Integral of rotation */
    //Eigen::Vector3d y;        /**< Double integral of rotated (acceleration - accel_bias) */
    //Eigen::Vector3d s;        /**< Integral of rotated (acceleration - accel_bias) */
    //Eigen::Matrix3d M;        /**< Integral( SkewSymm(rotated(acceleration - accel_bias))*integ(rotn) ) */
    //Eigen::Matrix3d MM;       /**< Integral of M */
    
    // Noise covariance matrices
    //Eigen::Matrix3d N0;       /**< Noise in rotation */
    //Eigen::Matrix3d N1;       /**< Noise in acceleration */
    //Eigen::Matrix3d N2;       /**< Noise due to M */
    //Eigen::Matrix3d N3;       /**< Noise due to gravity uncertainty */
    //Eigen::Matrix3d N1N1;     /**< Integral of N1 */
    //Eigen::Matrix3d N2N2;     /**< Integral of N2 */
    //Eigen::Matrix3d N3N3;     /**< Integral of N3 */
    
    //double qn0_1 =       dt*options.qr +  dt2_by_2*options.qwg;
    //double qn0_2 = dt2_by_2*options.qr +  dt3_by_6*options.qwg;
    //double qn0_3 = dt3_by_6*options.qr + dt4_by_24*options.qwg;
    //double qn1_1 =       dt*options.qa +  dt2_by_2*options.qwa;
    //double qn1_2 = dt2_by_2*options.qa +  dt3_by_6*options.qwa;
    //double qn3_1 =       dt*options.qg;
    //double qn3_2 = dt2_by_2*options.qg;
    
    //s_s_accl50 *= s_s_accl50.transpose();
    //sqr_s_s_accl50_diag = s_s_accl50.diagonal().asDiagonal();
    
    //integrals->N0 += qn0_1 * Eigen::Matrix3d::Identity();
    //integrals->N1 += qn1_1 * Eigen::Matrix3d::Identity();
    //integrals->N2 += qn0_2 * sqr_s_s_accl50_diag;  // Approximation to keep computations simple
    //integrals->N3 += qn3_1 * Eigen::Matrix3d::Identity();
    //integrals->N1N1 += qn1_2 * Eigen::Matrix3d::Identity();
    //integrals->N2N2 += qn0_3 * sqr_s_s_accl50_diag;  // Approximation to keep computations simple
    //integrals->N3N3 += qn3_2 * Eigen::Matrix3d::Identity();
    
    // Set all entries in Jacobians to zero
    dR_dS0_.setZero();  // 15x15
    dR_dS1_.setZero();  // 15x15
    dR_dG_.setZero();   // 15x3
    
    Eigen::Matrix3d I3(Eigen::Matrix3d::Identity());
    
    // Fill blocks of Jacobian wrt state0
    dR_dS0_.block<3,3>(rotn_idx,rotn_idx) =  I3;
    dR_dS0_.block<3,3>(rotn_idx,  bg_idx) = -integral_.V;
    
    dR_dS0_.block<3,3>(posn_idx,rotn_idx) = -anantak::SkewSymmetricMatrix(integral_.y);
    dR_dS0_.block<3,3>(posn_idx,posn_idx) =  I3;
    dR_dS0_.block<3,3>(posn_idx,velo_idx) =  integral_.dt*I3;
    dR_dS0_.block<3,3>(posn_idx,  bg_idx) =  integral_.MM;
    dR_dS0_.block<3,3>(posn_idx,  ba_idx) = -integral_.P;
    
    dR_dS0_.block<3,3>(velo_idx,rotn_idx) = -anantak::SkewSymmetricMatrix(integral_.s);
    dR_dS0_.block<3,3>(velo_idx,velo_idx) =  I3;
    dR_dS0_.block<3,3>(velo_idx,  bg_idx) =  integral_.M;
    dR_dS0_.block<3,3>(velo_idx,  ba_idx) = -integral_.V;
    
    dR_dS0_.block<3,3>(  bg_idx,  bg_idx) =  I3;
    
    dR_dS0_.block<3,3>(  ba_idx,  ba_idx) =  I3;
    
    // Fill blocks of Jacobian wrt state1
    dR_dS1_.block<3,3>(rotn_idx,rotn_idx) = -I3;
    dR_dS1_.block<3,3>(posn_idx,posn_idx) = -I3;
    dR_dS1_.block<3,3>(velo_idx,velo_idx) = -I3;
    dR_dS1_.block<3,3>(  bg_idx,  bg_idx) = -I3;
    dR_dS1_.block<3,3>(  ba_idx,  ba_idx) = -I3;
    
    // Fill blocks of Jacobian wrt gravity
    dR_dG_.block<3,3>(posn_idx,0) = 0.5*integral_.dt*integral_.dt*I3;
    dR_dG_.block<3,3>(velo_idx,0) = integral_.dt*I3;
    //dR_dG_.block<1,3>(   g_idx,0) = gravity_->Gp_.transpose()/grav_state_mag;
    
    // Divide Jacobian blocks by their noise' sqrt(variance)
    /* Scale residual with variance of the error states. Variance is calculated using the linear
     * model of the residual wrt error states. Only stochastic variables contribute to variance.
     * State is considered 'given' as in conditional variance is being calculated. */
    
    // Diagonals of the covariance matrices. All off-diagonal terms are zero.
    Eigen::Vector3d rotn_noise_cov = integral_.N0.diagonal();
    Eigen::Vector3d posn_noise_cov = integral_.N1N1.diagonal() + integral_.N2N2.diagonal() + integral_.N3N3.diagonal();
    Eigen::Vector3d velo_noise_cov = integral_.N1.diagonal() + integral_.N2.diagonal() + integral_.N3.diagonal();
    Eigen::Vector3d bg_noise_cov = integral_.N4.diagonal(); //options_.qwg * integral_.dt * Eigen::Vector3d::Ones();
    Eigen::Vector3d ba_noise_cov = integral_.N5.diagonal(); //options_.qwa * integral_.dt * Eigen::Vector3d::Ones();
    
    // Inverse square root of covariance matrix
    inv_sqrt_var_.block<3,1>(rotn_idx,0) = rotn_noise_cov.cwiseSqrt().cwiseInverse();
    inv_sqrt_var_.block<3,1>(posn_idx,0) = posn_noise_cov.cwiseSqrt().cwiseInverse();
    inv_sqrt_var_.block<3,1>(velo_idx,0) = velo_noise_cov.cwiseSqrt().cwiseInverse();
    inv_sqrt_var_.block<3,1>(  bg_idx,0) = bg_noise_cov.cwiseSqrt().cwiseInverse();
    inv_sqrt_var_.block<3,1>(  ba_idx,0) = ba_noise_cov.cwiseSqrt().cwiseInverse();
    //inv_sqrt_var_(g_idx,0) = 1./std::sqrt(options_.qg);
    
    // Modify Jacobians with variances
    dR_dS0_ = inv_sqrt_var_.asDiagonal() * dR_dS0_;
    dR_dS1_ = inv_sqrt_var_.asDiagonal() * dR_dS1_;
    dR_dG_  = inv_sqrt_var_.asDiagonal() * dR_dG_;
    
    // Quick sanity check
    if (abs(integral_.dt - dt_)>anantak::Epsilon) {
      LOG(ERROR) << "integral_dt does not match dt_" << integral_.dt << " " << dt_;
      return false;
    }
    
    return true;
  }
  
  // Check if this constraint is ok to use for optimization
  bool Check() const {
    if (!options_have_been_set_) {
      LOG(ERROR) << "Options have not been set";
      return false;
    }
    bool states_present = state0_ && state1_;
    if (!states_present) {
      LOG(ERROR) << "state0 or state1 is NULL";
      return false;
    }
    bool states_initiated = (!state0_->IsZero() && !state1_->IsZero());
    if (!states_initiated) {
      LOG(ERROR) << "States have not been initiated yet";
      return false;
    }
    if (residual_state_!=kClosed) {
      LOG(ERROR) << "Residual is not closed yet, can not be used for optimization.";
      return false;
    }
    return true;
  }
  
  bool GetReadyToOptimize() {
    if (!Check()) {LOG(ERROR)<<"Check failed, can not evaluate"; return false;}
    // Set State1 errors to zero. State0 errors are assumed to be zeroed by previous residual.
    state1_->SetErrorZero();
    return true;
  }
  
  bool IsOpen() const {
    return (residual_state_ == kOpen);
  }
  
  bool IsClosed() const {
    return (residual_state_ == kClosed);
  }
  
  // Optimizer valuation - called by Ceres solver
  virtual bool Evaluate(
      double const* const* parameters,
      double* residuals,
      double** jacobians) const {
    
    // Assuming Check() has been performed before
    
    // Mapping residuals and parameters to matrices
    // Curr error state dx0 = [ GdqI0, GdpI0, GdvI0, dbg0, dba0 ] = [dq0, dp0, dv0, dbg0, dba0]
    // Next error state dx1 = [ GdqI1, GdpI1, GdvI1, dbg1, dba1 ] = [dq1, dp1, dv1, dbg1, dba1]
    // Gravity error [ Gdg ] = [dgrav]
    MapConstVector15dType   E0(parameters[0]);
    MapConstVector15dType   E1(parameters[1]);
    MapConstVector3dType    EG(parameters[2]);
    MapVector15dType        R(residuals);
    
    /* Calculate residual from the error states. Linear model of the residual wrt error states
     * is used to recalculate the residual. */
    R = R_ + dR_dS0_*E0 + dR_dS1_*E1 + dR_dG_*EG;
    
    /* Jacobians are not recalculated with changing states. We use first estimates Jacobians
     * as this shown to keep the estimator consistent. */
    if (jacobians != NULL) {
      if (jacobians[0] != NULL) {
        MapMatrix15x15RowType dR_dS0(jacobians[0]);
        dR_dS0 = dR_dS0_;
      }
      if (jacobians[1] != NULL) {
        MapMatrix15x15RowType dR_dS1(jacobians[1]);
        dR_dS1 = dR_dS1_;
      }
      if (jacobians[2] != NULL) {
        MapMatrix15x3RowType dR_dG(jacobians[2]);
        dR_dG = dR_dG_;
      }
    }
    
    return true;
  }
  
  // Destructor
  virtual ~ImuResidualFunction() {}
  
}; // ImuResidualFunction  

/* ImuOmegaBiasResidual
 * This is to fix the value of angular bias to zero
 */
class ImuOmegaBiasResidual : public ceres::SizedCostFunction<3, 15> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef Eigen::Map<Eigen::Matrix<double,3,1>> MapVector3dType;
  typedef Eigen::Map<const Eigen::Matrix<double,15,1>> MapConstVector15dType;
  typedef Eigen::Matrix<double,3,15,Eigen::RowMajor> Matrix3x15RowType;
  typedef Eigen::Map<Eigen::Matrix<double,3,15,Eigen::RowMajor>> MapMatrix3x15RowType;
  
  ImuState* state_;
  double sigma_gyro_bias_;
  
  Matrix3x15RowType jacobian_;
  
  ImuOmegaBiasResidual(ImuState* state, double sigma_gyro_bias) {
    //if (!state) return false;
    //if (sigma_gyra_bias < Epsilon) return false;
    state_=state;
    sigma_gyro_bias_ = sigma_gyro_bias;
    jacobian_.setZero();
    jacobian_.block<3,3>(0,9) = Eigen::Matrix3d::Identity() / sigma_gyro_bias_;
  }
  
  virtual ~ImuOmegaBiasResidual() {}
  
  // Evaluate function for ceres solver
  virtual bool Evaluate(
      double const* const* parameters,
      double* residuals,
      double** jacobians) const {
    
    // Assuming Check() has been performed before
    
    // Mapping residuals and parameters to matrices
    MapConstVector15dType   dimu(parameters[0]);
    MapVector3dType        residual(residuals);
    
    /* Calculate residual from the error states. Linear model of the residual wrt error states
     * is used to recalculate the residual. */
    residual = jacobian_*dimu;
    
    /* Jacobians are not recalculated with changing states. We use first estimates Jacobians
     * as this shown to keep the estimator consistent. */
    if (jacobians != NULL) {
      if (jacobians[0] != NULL) {
        MapMatrix3x15RowType jac(jacobians[0]);
        jac = jacobian_;
      }
    }
    
    return true;
  }
  
};  // ImuOmegaBiasResidual

/* ImuAccelBiasResidual
 * This is to fix the value of angular bias to zero
 */
class ImuAccelBiasResidual : public ceres::SizedCostFunction<3, 15> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef Eigen::Map<Eigen::Matrix<double,3,1>> MapVector3dType;
  typedef Eigen::Map<const Eigen::Matrix<double,15,1>> MapConstVector15dType;
  typedef Eigen::Matrix<double,3,15,Eigen::RowMajor> Matrix3x15RowType;
  typedef Eigen::Map<Eigen::Matrix<double,3,15,Eigen::RowMajor>> MapMatrix3x15RowType;
  
  ImuState* state_;
  double sigma_accel_bias_;
  
  Matrix3x15RowType jacobian_;
  
  ImuAccelBiasResidual(ImuState* state, double sigma_gyro_bias) {
    //if (!state) return false;
    //if (sigma_gyra_bias < Epsilon) return false;
    state_=state;
    sigma_accel_bias_ = sigma_gyro_bias;
    jacobian_.setZero();
    jacobian_.block<3,3>(0,12) = Eigen::Matrix3d::Identity() / sigma_accel_bias_;
  }
  
  virtual ~ImuAccelBiasResidual() {}
  
  // Evaluate function for ceres solver
  virtual bool Evaluate(
      double const* const* parameters,
      double* residuals,
      double** jacobians) const {
    
    // Assuming Check() has been performed before
    
    // Mapping residuals and parameters to matrices
    MapConstVector15dType   dimu(parameters[0]);
    MapVector3dType        residual(residuals);
    
    /* Calculate residual from the error states. Linear model of the residual wrt error states
     * is used to recalculate the residual. */
    residual = jacobian_*dimu;
    
    /* Jacobians are not recalculated with changing states. We use first estimates Jacobians
     * as this shown to keep the estimator consistent. */
    if (jacobians != NULL) {
      if (jacobians[0] != NULL) {
        MapMatrix3x15RowType jac(jacobians[0]);
        jac = jacobian_;
      }
    }
    
    return true;
  }
  
};  // ImuAccelBiasResidual


/* Vector3d Prior
 * Setup a prior on a 3d vector */
class Vector3dPrior : public ceres::SizedCostFunction<3, 3> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Type declarations
  typedef Eigen::Map<Eigen::Matrix<double,3,1>> MapVector3dType;
  typedef Eigen::Map<const Eigen::Matrix<double,3,1>> MapConstVector3dType;
  typedef Eigen::Matrix<double,3,3,Eigen::RowMajor> Matrix3x3RowType;
  typedef Eigen::Map<Eigen::Matrix<double,3,3,Eigen::RowMajor>> MapMatrix3x3RowType;
  
  struct Options {
    double sigma_position;
    Options() {
      sigma_position = 0.010;               // in whatever units
    }
  };
  
  // Options
  Vector3dPrior::Options options_;
  
  // Data holders
  Eigen::Vector3d expected_position_;       /**< WpI value of position */
  anantak::Vector3dState *measurement_;       /**< One measurement of the pose */
  
  // Starting residual
  Eigen::Vector3d residual_;      /**< Starting residual */
  
  // Jacobian matrices
  Matrix3x3RowType dresidual_dmeasurement_;
  
  // Default constructor
  Vector3dPrior():
    options_(),
    expected_position_(Eigen::Vector3d::Zero()),
    measurement_(NULL) {
    residual_.setZero();
    dresidual_dmeasurement_.setZero();
  }
  
  // Default copy constructor
  Vector3dPrior(const Vector3dPrior& r) {
    options_=r.options_;
    expected_position_=r.expected_position_;
    measurement_=r.measurement_;
    residual_=r.residual_;
    dresidual_dmeasurement_=r.dresidual_dmeasurement_;
  }
  
  // Destructor
  virtual ~Vector3dPrior() {}
  
  // Reset residual
  bool Reset() {
    expected_position_ = Eigen::Vector3d::Zero();
    measurement_=NULL;
    residual_.setZero();
    dresidual_dmeasurement_.setZero();
    return true;
  }
  
  bool Create(const Eigen::Vector3d *posn, Vector3dPrior::Options *options) {
    // Reset the residual
    Reset();
    
    options_ = *options;
    
    residual_ =  *posn;
    dresidual_dmeasurement_ = -Eigen::Matrix3d::Identity();
    
    // Address noises
    double inv_sqrt_var_position = 1./options_.sigma_position;
    Eigen::Vector3d inv_sqrt_var_mat_diag;
    inv_sqrt_var_mat_diag << inv_sqrt_var_position, inv_sqrt_var_position, inv_sqrt_var_position;
    
    // Scale residual and jacobians for noise sqrt variance
    residual_ = inv_sqrt_var_mat_diag.asDiagonal() * residual_;
    dresidual_dmeasurement_ = inv_sqrt_var_mat_diag.asDiagonal() * dresidual_dmeasurement_;
    
    return true;
  }
  
  // Create the prior using a given quaternion (IqW) and position (WqI)
  bool Create(const Eigen::Vector3d *posn, anantak::Vector3dState *measured_pose,
      Vector3dPrior::Options *options) {
    // Any checks?
    
    // Reset the residual
    Reset();
    
    // Assign new values
    expected_position_ = *posn;   // copy
    measurement_ = measured_pose; 
    options_ = *options;
    if (!CalculateStartingResidualandJacobians()) {
      LOG(ERROR) << "Could not calculate Jacobians. Exit.";
      return false;
    }
    return true;
  }
  
  // Create prior - only needs the expected value and a variance
  bool Create(const anantak::Vector3dState *expected_pose, anantak::Vector3dState *measured_pose, 
      Vector3dPrior::Options *options) {
    Eigen::Vector3d posn(expected_pose->Position());
    return Create(&posn, measured_pose, options);
  }
  
  // Calculate Jacobians
  bool CalculateStartingResidualandJacobians() {
    
    Eigen::Vector3d WpI = measurement_->Position();
    Eigen::Vector3d WpI_ = expected_position_;
    Eigen::Vector3d WdpI = WpI - WpI_;
    
    residual_ =  WdpI;
    
    dresidual_dmeasurement_ = -Eigen::Matrix3d::Identity();
    
    // Address noises
    double inv_sqrt_var_position = 1./options_.sigma_position;
    Eigen::Vector3d inv_sqrt_var_mat_diag;
    inv_sqrt_var_mat_diag << inv_sqrt_var_position, inv_sqrt_var_position, inv_sqrt_var_position;
    
    // Scale residual and jacobians for noise sqrt variance
    residual_ = inv_sqrt_var_mat_diag.asDiagonal() * residual_;
    dresidual_dmeasurement_ = inv_sqrt_var_mat_diag.asDiagonal() * dresidual_dmeasurement_;
    
    return true;
  }
  
  bool Check() {
    return true;
  }
  
  // Get ready for optimization
  bool GetReadyToOptimize() {
    if (!Check()) {
      LOG(ERROR) << "Check failed. Can not continue.";
      return false;
    }
    // Set the errors to zero
    measurement_->SetErrorZero();
    
    return true;
  }
  
  // Evaluate function for ceres solver
  virtual bool Evaluate(
      double const* const* parameters,
      double* residuals,
      double** jacobians) const {
    
    // Assuming Check() has been performed before
    
    // Mapping residuals and parameters to matrices
    MapConstVector3dType   dmeasurement(parameters[0]);
    MapVector3dType        residual(residuals);
    
    /* Calculate residual from the error states. Linear model of the residual wrt error states
     * is used to recalculate the residual. */
    residual = residual_ + dresidual_dmeasurement_*dmeasurement;
    
    /* Jacobians are not recalculated with changing states. We use first estimates Jacobians
     * as this shown to keep the estimator consistent. */
    if (jacobians != NULL) {
      if (jacobians[0] != NULL) {
        MapMatrix3x3RowType jac(jacobians[0]);
        jac = dresidual_dmeasurement_;
      }
    }
    
    return true;
  }
  
};  // Vector3dPrior

/* Vector3d magnitude constraint
 * Implements the constraint that magnitude of a 3d vector is close to the given value */
class Vector3dMagnitudeResidual : public ceres::SizedCostFunction<1, 3,1> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Type declarations
  typedef Eigen::Matrix<double,1,1> Vector1dType;
  typedef Eigen::Map<Eigen::Matrix<double,1,1>> MapVector1dType;
  typedef Eigen::Map<Eigen::Matrix<double,3,1>> MapVector3dType;
  typedef Eigen::Map<const Eigen::Matrix<double,1,1>> MapConstVector1dType;
  typedef Eigen::Map<const Eigen::Matrix<double,3,1>> MapConstVector3dType;
  typedef Eigen::Matrix<double,3,3,Eigen::RowMajor> Matrix3x3RowType;
  typedef Eigen::Matrix<double,1,3,Eigen::RowMajor> Matrix1x3RowType;
  typedef Eigen::Matrix<double,1,1,Eigen::RowMajor> Matrix1x1RowType;
  typedef Eigen::Map<Eigen::Matrix<double,3,3,Eigen::RowMajor>> MapMatrix3x3RowType;
  typedef Eigen::Map<Eigen::Matrix<double,1,3,Eigen::RowMajor>> MapMatrix1x3RowType;
  typedef Eigen::Map<Eigen::Matrix<double,1,1,Eigen::RowMajor>> MapMatrix1x1RowType;
  
  struct Options {
    double sigma_magnitude;
    Options() {
      sigma_magnitude = 1.;               // in whatever units
    }
  };
  
  // Options
  Vector3dMagnitudeResidual::Options options_;
  
  // Data holders
  anantak::Vector3dState *measurement_;       /**< One measurement of the vector */
  anantak::ScalarState *magnitude_;           /**< Magnitude state */
  
  // Starting residual
  Vector1dType residual_;                  /**< Starting residual */
  
  // Jacobian matrices
  Matrix1x3RowType dresidual_dmeasurement_;
  Matrix1x1RowType dresidual_dmagnitude_;
  
  // Default constructor
  Vector3dMagnitudeResidual():
    options_(),
    measurement_(NULL),
    magnitude_(NULL) {
    residual_.setZero();
    dresidual_dmeasurement_.setZero();
    dresidual_dmagnitude_.setZero();
  }
  
  // Default copy constructor
  Vector3dMagnitudeResidual(const Vector3dMagnitudeResidual& r) {
    options_=r.options_;
    measurement_=r.measurement_;
    magnitude_=r.magnitude_;
    residual_=r.residual_;
    dresidual_dmeasurement_=r.dresidual_dmeasurement_;
    dresidual_dmagnitude_=r.dresidual_dmagnitude_;
  }
  
  // Destructor
  virtual ~Vector3dMagnitudeResidual() {}
  
  // Reset residual
  bool Reset() {
    measurement_=NULL;
    magnitude_=NULL;
    residual_.setZero();
    dresidual_dmeasurement_.setZero();
    dresidual_dmagnitude_.setZero();
    return true;
  }
  
  // Create 
  bool Create(anantak::Vector3dState *measurement, anantak::ScalarState *magnitude,
      Vector3dMagnitudeResidual::Options *options) {
    // Any checks?
    
    // Reset the residual
    Reset();
    
    // Assign new values
    measurement_ = measurement;
    magnitude_ = magnitude;
    options_ = *options;    // copy
    if (!CalculateStartingResidualandJacobians()) {
      LOG(ERROR) << "Could not calculate Jacobians. Exit.";
      return false;
    }
    return true;
  }
  
  // Calculate Jacobians
  bool CalculateStartingResidualandJacobians() {
    
    double starting_mag = measurement_->Position().norm();
    residual_(0,0) = measurement_->Position().norm() - magnitude_->Value();
    
    dresidual_dmeasurement_ = (1./starting_mag) * measurement_->Position().transpose();
    dresidual_dmagnitude_(0,0) = -1.;
    
    // Address noises
    double inv_sqrt_var_magnitude = 1./options_.sigma_magnitude;
    
    // Scale residual and jacobians for noise sqrt variance
    residual_ *= inv_sqrt_var_magnitude;
    dresidual_dmeasurement_ *= inv_sqrt_var_magnitude;
    dresidual_dmagnitude_ *= inv_sqrt_var_magnitude;
    
    return true;
  }
  
  bool Check() {
    return true;
  }
  
  // Get ready for optimization
  bool GetReadyToOptimize() {
    if (!Check()) {
      LOG(ERROR) << "Check failed. Can not continue.";
      return false;
    }
    // Set the errors to zero
    measurement_->SetErrorZero();
    magnitude_->SetErrorZero();
    return true;
  }
  
  // Evaluate function for ceres solver
  virtual bool Evaluate(
      double const* const* parameters,
      double* residuals,
      double** jacobians) const {
    
    // Assuming Check() has been performed before
    
    // Mapping residuals and parameters to matrices
    MapConstVector3dType   dmeasurement(parameters[0]);
    MapConstVector1dType   dmagnitude(parameters[1]);
    MapVector1dType        residual(residuals);
    
    /* Calculate residual from the error states. Linear model of the residual wrt error states
     * is used to recalculate the residual. */
    residual = residual_ + dresidual_dmeasurement_*dmeasurement + dresidual_dmagnitude_*dmagnitude;
    
    /* Jacobians are not recalculated with changing states. We use first estimates Jacobians
     * as this shown to keep the estimator consistent. */
    if (jacobians != NULL) {
      if (jacobians[0] != NULL) {
        MapMatrix1x3RowType jac(jacobians[0]);
        jac = dresidual_dmeasurement_;
      }
      if (jacobians[1] != NULL) {
        MapMatrix1x1RowType jac(jacobians[1]);
        jac = dresidual_dmagnitude_;
      }
    }
    
    return true;
  }
  
};  // Vector3dMagnitudeResidual


/* Vector3d fixed zero projection residual
 * Projection of one vector onto another (unit) vector is zero
 */
class Vector3dProjectionResidual0 : public ceres::SizedCostFunction<1, 3> {
 public:
  typedef Eigen::Map<Eigen::Matrix<double,1,1>> MapVector1dType;
  typedef Eigen::Map<const Eigen::Matrix<double,3,1>> MapConstVector3dType;
  typedef Eigen::Matrix<double,1,3,Eigen::RowMajor> Matrix1x3RowType;
  typedef Eigen::Map<Eigen::Matrix<double,1,3,Eigen::RowMajor>> MapMatrix1x3RowType;
  
  Eigen::Vector3d unit_vec_;
  double sigma_;
  
  Matrix1x3RowType jacobian_;
  
  Vector3dProjectionResidual0(const Eigen::Vector3d* unit_vec, const double sigma):
    unit_vec_((*unit_vec).normalized()), sigma_(sigma) {
    jacobian_ = unit_vec_.transpose()/sigma_;
  }
  
  virtual ~Vector3dProjectionResidual0() {}
  
  // Evaluate function for ceres solver
  virtual bool Evaluate(
      double const* const* parameters,
      double* residuals,
      double** jacobians) const {
    
    // Assuming Check() has been performed before
    
    // Mapping residuals and parameters to matrices
    MapConstVector3dType   vector(parameters[0]);
    MapVector1dType        residual(residuals);
    
    /* Calculate residual from the error states. Linear model of the residual wrt error states
     * is used to recalculate the residual. */
    residual = jacobian_ * vector;
    
    /* Jacobians are not recalculated with changing states. We use first estimates Jacobians
     * as this shown to keep the estimator consistent. */
    if (jacobians != NULL) {
      if (jacobians[0] != NULL) {
        MapMatrix1x3RowType jac(jacobians[0]);
        jac = jacobian_;
      }
    }
    
    return true;
  }    
  
};  // Vector3dProjectionResidual0

/* Pose3dPrior
 * Setup a prior on a noisy pose */
class Pose3dPrior : public ceres::SizedCostFunction<6, 6> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Type declarations
  typedef Eigen::Map<Eigen::Matrix<double,6,1>> MapVector6dType;
  typedef Eigen::Map<const Eigen::Matrix<double,6,1>> MapConstVector6dType;
  typedef Eigen::Matrix<double,6,6,Eigen::RowMajor> Matrix6x6RowType;
  typedef Eigen::Map<Eigen::Matrix<double,6,6,Eigen::RowMajor>> MapMatrix6x6RowType;
  
  struct Options {
    double sigma_theta;
    double sigma_position;
    bool use_vec;
    Eigen::Vector3d sigma_theta_vec;
    Eigen::Vector3d sigma_position_vec;
    Options(): use_vec(false), sigma_theta_vec(Eigen::Vector3d::Zero()),
      sigma_position_vec(Eigen::Vector3d::Zero()) {
      sigma_theta = 10.*RadiansPerDegree;   // in Radians
      sigma_position = 0.010;               // in meters
    }
  };
  
  // Options
  Pose3dPrior::Options options_;
  
  // Data holders
  Eigen::Quaterniond expected_rotation_;    /**< IqW value of rotation */
  Eigen::Vector3d expected_position_;       /**< WpI value of position */
  anantak::Pose3dState *measurement_;       /**< One measurement of the pose */
  
  // Starting residual
  Eigen::Matrix<double,6,1> residual_;      /**< Starting residual */
  
  // Jacobian matrices
  Matrix6x6RowType dresidual_dmeasurement_;
  
  // Default constructor
  Pose3dPrior():
    options_(),
    expected_rotation_(Eigen::Quaterniond::Identity()), expected_position_(Eigen::Vector3d::Zero()),
    measurement_(NULL) {
    residual_.setZero();
    dresidual_dmeasurement_.setZero();
  }
  
  // Default copy constructor
  Pose3dPrior(const Pose3dPrior& r) {
    options_=r.options_;
    expected_rotation_=r.expected_rotation_; expected_position_=r.expected_position_;
    measurement_=r.measurement_;
    residual_=r.residual_;
    dresidual_dmeasurement_=r.dresidual_dmeasurement_;
  }
  
  // Destructor
  virtual ~Pose3dPrior() {}
  
  // Reset residual
  bool Reset() {
    expected_rotation_ = Eigen::Quaterniond::Identity();
    expected_position_ = Eigen::Vector3d::Zero();
    measurement_=NULL;
    residual_.setZero();
    dresidual_dmeasurement_.setZero();
    return true;
  }
  
  // Create the prior using a given quaternion (IqW) and position (WqI)
  bool Create(const Eigen::Quaterniond *quat, const Eigen::Vector3d *posn,
      anantak::Pose3dState *measured_pose, Pose3dPrior::Options *options) {
    // Any checks?
    
    // Reset the residual
    Reset();
    
    // Assign new values
    expected_rotation_ = *quat;   // copy
    expected_position_ = *posn;   // copy
    measurement_ = measured_pose; 
    options_ = *options;
    if (!CalculateStartingResidualandJacobians()) {
      LOG(ERROR) << "Could not calculate Jacobians. Exit.";
      return false;
    }
    return true;
  }
  
  // Create prior - only needs the expected value and a variance
  bool Create(const anantak::Pose3dState *expected_pose, anantak::Pose3dState *measured_pose, 
      Pose3dPrior::Options *options) {
    Eigen::Quaterniond quat(expected_pose->Quaternion());
    Eigen::Vector3d posn(expected_pose->Position());
    return Create( &quat, &posn, measured_pose, options);
  }
  
  // Calculate Jacobians
  bool CalculateStartingResidualandJacobians() {
    
    Eigen::Quaterniond IqW = measurement_->Quaternion();
    Eigen::Vector3d WpI = measurement_->Position();
    Eigen::Quaterniond IqW_ = expected_rotation_;
    Eigen::Vector3d WpI_ = expected_position_;
    
    Eigen::Quaterniond WdqI = IqW_.conjugate() * IqW;
    Eigen::Vector3d WdpI = WpI - WpI_;
    
    residual_.block<3,1>(0,0) =  2.*WdqI.vec();   // Approximation that assumes small angle
    residual_.block<3,1>(3,0) =  WdpI;
    
    dresidual_dmeasurement_.setZero();
    dresidual_dmeasurement_.block<3,3>(0,0) = -WdqI.toRotationMatrix().transpose();
    dresidual_dmeasurement_.block<3,3>(3,3) = -Eigen::Matrix3d::Identity();
    
    // Address noises
    Eigen::Matrix<double,6,1> inv_sqrt_var_mat_diag;
    if (options_.use_vec) {
      inv_sqrt_var_mat_diag.block<3,1>(0,0) = options_.sigma_theta_vec.cwiseInverse();
      inv_sqrt_var_mat_diag.block<3,1>(3,0) = options_.sigma_position_vec.cwiseInverse();
    } else {
      double inv_sqrt_var_theta = 1./options_.sigma_theta;
      double inv_sqrt_var_position = 1./options_.sigma_position;
      inv_sqrt_var_mat_diag << inv_sqrt_var_theta, inv_sqrt_var_theta, inv_sqrt_var_theta,
          inv_sqrt_var_position, inv_sqrt_var_position, inv_sqrt_var_position;
    }
    
    // Scale residual and jacobians for noise sqrt variance
    residual_ = inv_sqrt_var_mat_diag.asDiagonal() * residual_;
    dresidual_dmeasurement_ = inv_sqrt_var_mat_diag.asDiagonal() * dresidual_dmeasurement_;
    
    return true;
  }
  
  bool Check() {
    return true;
  }
  
  // Get ready for optimization
  bool GetReadyToOptimize() {
    if (!Check()) {
      LOG(ERROR) << "Check failed. Can not continue.";
      return false;
    }
    // Set the errors to zero
    measurement_->SetErrorZero();
    
    return true;
  }
  
  // Evaluate function for ceres solver
  virtual bool Evaluate(
      double const* const* parameters,
      double* residuals,
      double** jacobians) const {
    
    // Assuming Check() has been performed before
    
    // Mapping residuals and parameters to matrices
    MapConstVector6dType   dmeasurement(parameters[0]);
    MapVector6dType        residual(residuals);
    
    /* Calculate residual from the error states. Linear model of the residual wrt error states
     * is used to recalculate the residual. */
    residual = residual_ + dresidual_dmeasurement_*dmeasurement;
    
    /* Jacobians are not recalculated with changing states. We use first estimates Jacobians
     * as this shown to keep the estimator consistent. */
    if (jacobians != NULL) {
      if (jacobians[0] != NULL) {
        MapMatrix6x6RowType jac(jacobians[0]);
        jac = dresidual_dmeasurement_;
      }
    }
    
    return true;
  }
  
};  // Pose3dPrior 

  /* Noisy Pose Residual
 * There is a mean pose. But several measurements of the pose are noisy measurements of that pose.
 * Both expectation as well as measurement are variable states.
 */
class NoisyPoseResidual : public ceres::SizedCostFunction<6, 6,6> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Type declarations
  typedef Eigen::Map<Eigen::Matrix<double,6,1>> MapVector6dType;
  typedef Eigen::Map<const Eigen::Matrix<double,6,1>> MapConstVector6dType;
  typedef Eigen::Matrix<double,6,6,Eigen::RowMajor> Matrix6x6RowType;
  typedef Eigen::Map<Eigen::Matrix<double,6,6,Eigen::RowMajor>> MapMatrix6x6RowType;
  
  struct Options {
    double sigma_theta;
    double sigma_position;
    bool rate_of_change;    /**< True if this is a rate of change of pose residual */
    Options() {
      sigma_theta = 2.*RadiansPerDegree;   // in Radians
      sigma_position = 0.005;              // in meters
      rate_of_change = false;              // By default not a rate of change
    }
  };
  
  // Options
  NoisyPoseResidual::Options options_;
  
  // Data holders
  anantak::Pose3dState *expectation_;       /**< Expected value of the 3d pose */
  anantak::Pose3dState *measurement_;       /**< One measurement of the pose */
  
  // Starting residual
  Eigen::Matrix<double,6,1> residual_;      /**< Starting residual */
  
  // Jacobian matrices
  Matrix6x6RowType dresidual_dexpectation_;
  Matrix6x6RowType dresidual_dmeasurement_;
  
  // Default constructor
  NoisyPoseResidual():
    options_(),
    expectation_(NULL), measurement_(NULL) {
    residual_.setZero();
    dresidual_dexpectation_.setZero(); dresidual_dmeasurement_.setZero();
  }
  
  // Default copy constructor
  NoisyPoseResidual(const NoisyPoseResidual& r) {
    options_=r.options_;
    expectation_=r.expectation_; measurement_=r.measurement_;
    residual_=r.residual_;
    dresidual_dexpectation_=r.dresidual_dexpectation_;
    dresidual_dmeasurement_=r.dresidual_dmeasurement_;
  }
  
  // Destructor
  virtual ~NoisyPoseResidual() {}
  
  // Reset residual
  bool Reset() {
    expectation_=NULL; measurement_=NULL;
    residual_.setZero();
    dresidual_dexpectation_.setZero();
    dresidual_dmeasurement_.setZero();
    return true;
  }
  
  // Create residual
  bool Create(anantak::Pose3dState *expectation, anantak::Pose3dState *measurement, 
      NoisyPoseResidual::Options *options) {
    
    if (options->rate_of_change) {
      if (expectation->timestamp_ == 0) LOG(WARNING) << "expectation ts is zero " << expectation->timestamp_;
      if (measurement->timestamp_ == 0) LOG(WARNING) << "measurement ts is zero " << measurement->timestamp_;
      // measurement comes after expectation
      double dt = double(measurement->timestamp_ - expectation->timestamp_)*1e-6;
      if (dt < Epsilon) {
        LOG(ERROR) << "rate_of_change pose residual can not have zero/negative change in ts " 
            << expectation->timestamp_ << " " << measurement->timestamp_ << " " << dt;
        return false;
      }
    }
    
    // Reset the residual
    Reset();
    
    // Assign new values
    expectation_ = expectation;
    measurement_ = measurement;
    options_ = *options;    // copy
    if (!CalculateStartingResidualandJacobians()) {
      LOG(ERROR) << "Could not calculate Jacobians. Exit.";
      return false;
    }
    return true;
  }
  
  // Calculate Jacobians
  bool CalculateStartingResidualandJacobians() {
    
    Eigen::Quaterniond IqW = measurement_->Quaternion();
    Eigen::Vector3d WpI = measurement_->Position();
    Eigen::Quaterniond IqW_ = expectation_->Quaternion();
    Eigen::Vector3d WpI_ = expectation_->Position();
    
    Eigen::Quaterniond WdqI = IqW_.conjugate() * IqW;
    Eigen::Vector3d WdpI = WpI - WpI_;
    
    residual_.block<3,1>(0,0) =  2.*WdqI.vec();   // Approximation that assumes small angle
    residual_.block<3,1>(3,0) =  WdpI;
    
    dresidual_dexpectation_.setZero();
    dresidual_dexpectation_.block<3,3>(0,0) =  Eigen::Matrix3d::Identity();
    dresidual_dexpectation_.block<3,3>(3,3) =  Eigen::Matrix3d::Identity();
    
    dresidual_dmeasurement_.setZero();
    dresidual_dmeasurement_.block<3,3>(0,0) = -WdqI.toRotationMatrix().transpose();
    dresidual_dmeasurement_.block<3,3>(3,3) = -Eigen::Matrix3d::Identity();
    
    // Address noises
    double inv_sqrt_var_theta = 1./options_.sigma_theta;
    double inv_sqrt_var_position = 1./options_.sigma_position;
    
    // If rate of change, variance depends on elapsed time
    if (options_.rate_of_change) {
      double dt = double(measurement_->timestamp_ - expectation_->timestamp_)*1e-6;
      if (dt<Epsilon) {
        LOG(ERROR) << "dt<=0! not possible";
        return false;
      }
      double inv_sqrt_dt = 1./std::sqrt(dt);
      inv_sqrt_var_theta *= inv_sqrt_dt;
      inv_sqrt_var_position *= inv_sqrt_dt;
    }
    
    Eigen::Matrix<double,6,1> inv_sqrt_var_mat_diag;
    inv_sqrt_var_mat_diag << inv_sqrt_var_theta, inv_sqrt_var_theta, inv_sqrt_var_theta,
        inv_sqrt_var_position, inv_sqrt_var_position, inv_sqrt_var_position;
    
    // Scale residual and jacobians for noise sqrt variance
    residual_ = inv_sqrt_var_mat_diag.asDiagonal() * residual_;
    dresidual_dexpectation_ = inv_sqrt_var_mat_diag.asDiagonal() * dresidual_dexpectation_;
    dresidual_dmeasurement_ = inv_sqrt_var_mat_diag.asDiagonal() * dresidual_dmeasurement_;
    
    return true;
  }
  
  bool Check() {
    return true;
  }
  
  // Get ready for optimization
  bool GetReadyToOptimize() {
    if (!Check()) {
      LOG(ERROR) << "Check failed. Can not continue.";
      return false;
    }
    // Set the errors to zero
    measurement_->SetErrorZero();
    
    return true;
  }
  
  // Evaluate function for ceres solver
  virtual bool Evaluate(
      double const* const* parameters,
      double* residuals,
      double** jacobians) const {
    
    // Assuming Check() has been performed before
    
    // Mapping residuals and parameters to matrices
    MapConstVector6dType   dexpectation(parameters[0]);
    MapConstVector6dType   dmeasurement(parameters[1]);
    MapVector6dType        residual(residuals);
    
    /* Calculate residual from the error states. Linear model of the residual wrt error states
     * is used to recalculate the residual. */
    residual = residual_ + dresidual_dexpectation_*dexpectation + dresidual_dmeasurement_*dmeasurement;
    
    /* Jacobians are not recalculated with changing states. We use first estimates Jacobians
     * as this shown to keep the estimator consistent. */
    if (jacobians != NULL) {
      if (jacobians[0] != NULL) {
        MapMatrix6x6RowType jac(jacobians[0]);
        jac = dresidual_dexpectation_;
      }
      if (jacobians[1] != NULL) {
        MapMatrix6x6RowType jac(jacobians[1]);
        jac = dresidual_dmeasurement_;
      }
    }
    
    return true;
  }
  
};  // NoisyPoseResidual 

/* IMU to Absolute Pose Sensor Residual
 * Say there are two absolute pose sensors that are rigidly connected and are detecting their
 * poses in wrt their reference map. E.g. Imu's absolute pose states and a tag camera. Imu
 * measures its pose in its own world reference frame. Tag camera measures its pose wrt tag map.
 * This residual will connect the poses of second absolute camera with the first one.
 * Notation:
 *  Sensor 1 is I measuring its pose wrt W frame: IiqW WpIi, i is index of measurement
 *  Sensor 2 is C measuring its pose wrt T0 frame: CiqT0 T0pCi, is is the measurement index
 *  I to C pose is: CqI IpC
 *  T0 to W pose is: T0qW WpT0
 *  All quaternions (q) are 4x1, positions (p) are 3x1, d_quaternions (dq) and dp are 3x1
 * States are:
 *  IiqW WpIi  CqI IpC  T0qW WpT0 - 7x1 7x1 7x1
 * Error states are:
 *  WdqIi WdpIi  IdqC IdpC  WdqT0 WdpT0 - 6x1 6x1 6x1
 * Measurements are:
 *  CiqT0 T0pCi - 7x1
 * Residuals are:
 *  T0dqCi T0dpCi - 6x1
 */
class ImuToAbsPoseSensorResidual : public ceres::SizedCostFunction<6, 6,6,6> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Type declarations
  typedef Eigen::Map<Eigen::Matrix<double,6,1>> MapVector6dType;
  typedef Eigen::Map<const Eigen::Matrix<double,6,1>> MapConstVector6dType;
  typedef Eigen::Matrix<double,6,6,Eigen::RowMajor> Matrix6x6RowType;
  typedef Eigen::Map<Eigen::Matrix<double,6,6,Eigen::RowMajor>> MapMatrix6x6RowType;
  
  struct Options {
    bool use_first_estimates;   /**< Should we reevaluate Jacobians at every estimate? */
    // Noise parameters
    double q_theta;             /**< Stdev of T0dqCi in radians/sqrt(Hz) */
    double q_posn;              /**< Stdev of TodpCi in meters/sqrt(HZ) */
    Options() {
      use_first_estimates = true;       /**< Generally we use first estimate jacobians */
      q_theta = 5.*RadiansPerDegree/3.; /**< 3 sigmas inside 5 degrees */
      q_posn = 0.010/3.;                /**< 3 sigmas lie inside 1 cm */
    }
  };
  
  // Options
  ImuToAbsPoseSensorResidual::Options options_;
  
  // Data holders
  anantak::ImuState *poseI_;          /**< Pose of Imu sensor in W frame */
  anantak::Pose3dState *poseC_;       /**< Pose of second sensor in T0 frame */
  anantak::Pose3dState *poseItoC_;    /**< Relative pose between sensors */
  anantak::Pose3dState *poseWtoT0_;   /**< Relative pose of reference frames */
  
  // Starting Residuals
  Eigen::Matrix<double,6,1> T0dq_dpCi_;   /**< Starting residual */
  
  // Jacobian matrices
  Matrix6x6RowType dposeC_dposeI_, dposeC_dposeWtoT0_, dposeC_dposeItoC_;
  
  // Default constructor
  ImuToAbsPoseSensorResidual():
    options_(),
    poseI_(NULL), poseC_(NULL), poseItoC_(NULL), poseWtoT0_(NULL) {
    dposeC_dposeI_.setZero(); dposeC_dposeWtoT0_.setZero(); dposeC_dposeItoC_.setZero();
    T0dq_dpCi_.setZero();
  }
  
  // Default copy constructor
  ImuToAbsPoseSensorResidual(const ImuToAbsPoseSensorResidual& r) {
    options_=r.options_;
    poseI_=r.poseI_; poseC_=r.poseC_; poseItoC_=r.poseItoC_; poseWtoT0_=r.poseWtoT0_;
    T0dq_dpCi_=r.T0dq_dpCi_; 
    dposeC_dposeI_=r.dposeC_dposeI_; dposeC_dposeWtoT0_=r.dposeC_dposeWtoT0_;
    dposeC_dposeItoC_=r.dposeC_dposeItoC_;
    
  }
  
  // Destructor
  virtual ~ImuToAbsPoseSensorResidual() {}
  
  // Reset residual
  bool Reset() {
    poseI_=NULL; poseC_=NULL; poseItoC_=NULL; poseWtoT0_=NULL;
    T0dq_dpCi_.setZero(); 
    dposeC_dposeI_.setZero(); dposeC_dposeWtoT0_.setZero(); dposeC_dposeItoC_.setZero();
    return true;
  }
  
  // Create residual
  bool Create(anantak::ImuState *poseI, anantak::Pose3dState *poseC,
      anantak::Pose3dState *poseItoC, anantak::Pose3dState *poseWtoT0,
      ImuToAbsPoseSensorResidual::Options *options) {
    if (poseI->IsZero()) {
      LOG(WARNING) << "Provided imu pose is zero";
    }
    if (poseC->IsZero()) {
      LOG(WARNING) << "Provided sensor pose is zero";
    }
    // Should we check if any of the other poses are zero too?
    // Should we return false if so?
    
    // Reset the residual
    Reset();
    
    // Assign new values
    poseI_ = poseI;
    poseC_ = poseC;
    poseItoC_ = poseItoC;
    poseWtoT0_ = poseWtoT0;
    options_ = *options;
    if (!CalculateStartingResidualandJacobians()) {
      LOG(ERROR) << "Could not calculate Jacobians. Exit.";
      return false;
    }
    return true;
  }
  
  // Calculate Jacobians
  bool CalculateStartingResidualandJacobians() {
    
    /* All maths here is written in robotics notebook#4, ppg 50-52 */
    
    // Collect the available estimates
    Eigen::Quaterniond IiqW(poseI_->Quaternion());   // State has IiqW
    Eigen::Quaterniond CqI(poseItoC_->Quaternion());   // Stored CqI
    Eigen::Quaterniond T0qW(poseWtoT0_->Quaternion());  // State has T0qW
    Eigen::Quaterniond CiqT0(poseC_->Quaternion());      // Stored is CiqT0
    
    Eigen::Vector3d WpIi(poseI_->GpI_);
    Eigen::Vector3d IpC(poseItoC_->GpL_);
    Eigen::Vector3d WpT0(poseWtoT0_->GpL_);
    Eigen::Vector3d T0pCi(poseC_->GpL_);
    
    Eigen::Matrix3d IirW(IiqW);
    Eigen::Matrix3d CrI(CqI);
    Eigen::Matrix3d T0rW(T0qW);
    Eigen::Matrix3d CirT0(CiqT0);
    
    // Calculate residuals
    Eigen::Quaterniond T0qCi_est = T0qW * IiqW.conjugate() * CqI.conjugate();
    Eigen::Quaterniond T0dqCi_inv = T0qCi_est * CiqT0;
    Eigen::Vector3d T0pCi_est = T0rW * (IirW.transpose()*IpC + WpIi - WpT0);
    
    T0dq_dpCi_.block<3,1>(0,0) = -2.*T0dqCi_inv.vec();    // Approximation in conversion to angleaxis
    T0dq_dpCi_.block<3,1>(3,0) = T0pCi - T0pCi_est;
    
    // Calculate Jacobians
    Eigen::Matrix3d T0rW_est = CirT0.transpose() * CrI * IirW;
    Eigen::Matrix3d T0rCirI_est = CirT0.transpose() * CrI;
    Eigen::Matrix3d T0pCi_est_sksym = anantak::SkewSymmetricMatrix(T0pCi_est);
    Eigen::Matrix3d T0rWrIi_est = T0rW * IirW.transpose();
    Eigen::Matrix3d T0IpC_est_sksym = anantak::SkewSymmetricMatrix(T0rWrIi_est*IpC);
    
    dposeC_dposeI_.setZero();
    dposeC_dposeI_.block<3,3>(0,0) =  T0rW_est;
    dposeC_dposeI_.block<3,3>(3,0) = -T0IpC_est_sksym * T0rW;
    dposeC_dposeI_.block<3,3>(3,3) =  T0rW;
    
    dposeC_dposeItoC_.setZero();
    dposeC_dposeItoC_.block<3,3>(0,0) = T0rCirI_est;
    dposeC_dposeItoC_.block<3,3>(3,3) = T0rWrIi_est;
    
    dposeC_dposeWtoT0_.setZero();
    dposeC_dposeWtoT0_.block<3,3>(0,0) = -T0rW_est;
    dposeC_dposeWtoT0_.block<3,3>(0,3) =  T0pCi_est_sksym * T0rW;
    dposeC_dposeWtoT0_.block<3,3>(3,3) = -T0rW;
    
    // Scale residual and Jacobians by 1./sqrt(variances)
    double inv_sqrt_var_theta = 1./options_.q_theta;
    double inv_sqrt_var_posn  = 1./options_.q_posn;
    Eigen::Matrix<double,6,1> inv_sqrt_var_vec;
    inv_sqrt_var_vec << inv_sqrt_var_theta, inv_sqrt_var_theta, inv_sqrt_var_theta,
                        inv_sqrt_var_posn, inv_sqrt_var_posn, inv_sqrt_var_posn;
    dposeC_dposeI_ = inv_sqrt_var_vec.asDiagonal() * dposeC_dposeI_;
    dposeC_dposeItoC_ = inv_sqrt_var_vec.asDiagonal() * dposeC_dposeItoC_;
    dposeC_dposeWtoT0_ = inv_sqrt_var_vec.asDiagonal() * dposeC_dposeWtoT0_;
    T0dq_dpCi_ = inv_sqrt_var_vec.asDiagonal() * T0dq_dpCi_;
    
    return true;
  }
  
  bool Check() {
    return true;
  }
  
  // Get ready for Optimization
  bool GetReadyToOpimize() {
    if (!Check()) {
      LOG(ERROR) << "Check failed. Can not continue.";
      return false;
    }
    // Set the errors to zero
    // Imu error state is set to zero by other residuals
    poseC_->SetErrorZero();
    poseItoC_->SetErrorZero();
    poseWtoT0_->SetErrorZero();
    
    return true;
  }
  
  // Evaluate function for ceres solver
  virtual bool Evaluate(
      double const* const* parameters,
      double* residuals,
      double** jacobians) const {
    
    // Assuming Check() has been performed before
    
    // Mapping residuals and parameters to matrices
    MapConstVector6dType   dposeI(parameters[0]);
    MapConstVector6dType   dposeItoC(parameters[1]);
    MapConstVector6dType   dposeWtoT0(parameters[2]);
    MapVector6dType        dposeC(residuals);
    
    /* Calculate residual from the error states. Linear model of the residual wrt error states
     * is used to recalculate the residual. */
    dposeC = T0dq_dpCi_ + dposeC_dposeI_*dposeI + dposeC_dposeItoC_*dposeItoC + dposeC_dposeWtoT0_*dposeWtoT0;
    
    /* Jacobians are not recalculated with changing states. We use first estimates Jacobians
     * as this shown to keep the estimator consistent. */
    if (jacobians != NULL) {
      if (jacobians[0] != NULL) {
        MapMatrix6x6RowType jac(jacobians[0]);
        jac = dposeC_dposeI_;
      }
      if (jacobians[1] != NULL) {
        MapMatrix6x6RowType jac(jacobians[1]);
        jac = dposeC_dposeItoC_;
      }
      if (jacobians[2] != NULL) {
        MapMatrix6x6RowType jac(jacobians[2]);
        jac = dposeC_dposeWtoT0_;
      }
    }
    
    return true;
  }
}; // ImuToAbsPoseSensorResidual

/* Imu to Camera Pose residual
 * We solve the Hand-eye calibration type problem but where motion is included. As rotation
 * and motion are along different axes the rotation between sensors should be observable.
 * Displacement should be observable in the plane of motion only as rotation is only along
 * one axis.
 * Maths here comes from Robotics notebook #4 pg 58
 */
class ImuToCameraPoseResidual : public ceres::SizedCostFunction<6, 15,6> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Type declarations
  typedef Eigen::Map<Eigen::Matrix<double,6,1>> MapVector6dType;
  typedef Eigen::Map<const Eigen::Matrix<double,6,1>> MapConstVector6dType;
  typedef Eigen::Map<const Eigen::Matrix<double,15,1>> MapConstVector15dType;
  typedef Eigen::Matrix<double,6,6,Eigen::RowMajor> Matrix6x6RowType;
  typedef Eigen::Matrix<double,6,15,Eigen::RowMajor> Matrix6x15RowType;
  typedef Eigen::Map<Eigen::Matrix<double,6,6,Eigen::RowMajor>> MapMatrix6x6RowType;
  typedef Eigen::Map<Eigen::Matrix<double,6,15,Eigen::RowMajor>> MapMatrix6x15RowType;
  
  struct Options {
    bool use_first_estimates;   /**< Should we reevaluate Jacobians at every estimate? */
    // Noise parameters
    double q_theta;             /**< Stdev of T0dqCi in radians/sqrt(Hz) */
    double q_posn;              /**< Stdev of T0dpCi in meters/sqrt(HZ) */
    Options() {
      use_first_estimates = true;       /**< Generally we use first estimate jacobians */
      q_theta = 5.*RadiansPerDegree/3.; /**< 3 sigmas lie inside 5 degrees */
      q_posn = 0.010/3.;                /**< 3 sigmas lie inside 1 cm */
    }
  };
  
  // Options
  ImuToCameraPoseResidual::Options options_;
  
  // Data holders
  anantak::ImuState *poseI_;          /**< Pose of Imu sensor in W frame */
  anantak::Pose3dState *poseC_;       /**< Pose of second sensor in T0 frame */
  anantak::Pose3dState *poseItoC_;    /**< Relative pose between sensors */
  
  // Starting Residuals
  Eigen::Matrix<double,6,1> C0dq_dpCi_;   /**< Starting residual */
  
  // Jacobian matrices
  Matrix6x6RowType dposeC_dposeItoC_;
  Matrix6x15RowType dposeC_dposeI_;
  
  // Default constructor
  ImuToCameraPoseResidual():
    options_(),
    poseI_(NULL), poseC_(NULL), poseItoC_(NULL) {
    dposeC_dposeI_.setZero(); dposeC_dposeItoC_.setZero();
    C0dq_dpCi_.setZero();
  }
  
  // Default copy constructor
  ImuToCameraPoseResidual(const ImuToCameraPoseResidual& r) {
    options_=r.options_;
    poseI_=r.poseI_; poseC_=r.poseC_; poseItoC_=r.poseItoC_;
    C0dq_dpCi_=r.C0dq_dpCi_; 
    dposeC_dposeI_=r.dposeC_dposeI_;
    dposeC_dposeItoC_=r.dposeC_dposeItoC_;
  }
  
  // Destructor
  virtual ~ImuToCameraPoseResidual() {}
  
  // Reset residual
  bool Reset() {
    poseI_=NULL; poseC_=NULL; poseItoC_=NULL;
    C0dq_dpCi_.setZero();
    dposeC_dposeI_.setZero(); dposeC_dposeItoC_.setZero();
    return true;
  }
  
  // Create residual
  bool Create(anantak::ImuState *poseI, anantak::Pose3dState *poseC,
      anantak::Pose3dState *poseItoC,
      ImuToCameraPoseResidual::Options *options) {
    if (poseI->IsZero()) {
      LOG(WARNING) << "Provided imu pose is zero";
    }
    if (poseC->IsZero()) {
      LOG(WARNING) << "Provided sensor pose is zero";
    }
    // Should we check if any of the other poses are zero too?
    // Should we return false if so?
    
    // Reset the residual
    Reset();
    
    // Assign new values
    poseI_ = poseI;
    poseC_ = poseC;
    poseItoC_ = poseItoC;
    options_ = *options;  // copy options
    if (!CalculateStartingResidualandJacobians()) {
      LOG(ERROR) << "Could not calculate Jacobians. Exit.";
      return false;
    }
    return true;
  }
  
  // Calculate Jacobians
  bool CalculateStartingResidualandJacobians() {
    
    /* All maths here is written in robotics notebook#4, pg 58 */
    
    // Collect the available estimates
    Eigen::Quaterniond IiqI0(poseI_->Quaternion());   // State has IiqI0
    Eigen::Quaterniond CqI(poseItoC_->Quaternion());   // Stored CqI
    Eigen::Quaterniond CiqC0(poseC_->Quaternion());      // Stored is CiqC0
    
    Eigen::Vector3d I0pIi(poseI_->GpI_);
    Eigen::Vector3d IpC(poseItoC_->GpL_);
    Eigen::Vector3d C0pCi(poseC_->GpL_);
    
    Eigen::Matrix3d IirI0(IiqI0);
    Eigen::Matrix3d CrI(CqI);
    Eigen::Matrix3d CirC0(CiqC0);
    
    // Calculate starting residuals
    Eigen::Matrix3d I3(Eigen::Matrix3d::Identity());
    Eigen::Quaterniond C0dqCi_est = CiqC0.conjugate() * CqI * IiqI0 * CqI.conjugate();
    Eigen::Matrix3d C0drCi_est(C0dqCi_est);
    Eigen::Vector3d I0pCi_est = (IirI0.transpose() - I3)*IpC + I0pIi;
    Eigen::Vector3d C0pCi_est = CrI*I0pCi_est;
    Eigen::Vector3d I0IipCi_est = IirI0.transpose() * IpC;
    
    C0dq_dpCi_.block<3,1>(0,0) = 2.*C0dqCi_est.vec();    // Approximation in conversion to angleaxis
    C0dq_dpCi_.block<3,1>(3,0) = C0pCi_est - C0pCi;
    
    // Calculate Jacobians
    Eigen::Matrix3d pre_mult_mat = -C0drCi_est * CrI;
    dposeC_dposeItoC_.setZero();
    dposeC_dposeItoC_.block<3,3>(0,0) =  pre_mult_mat * (IirI0.transpose() - I3);
    dposeC_dposeItoC_.block<3,3>(3,0) =  CrI * anantak::SkewSymmetricMatrix(I0pCi_est);
    dposeC_dposeItoC_.block<3,3>(3,3) =  CrI * (IirI0.transpose() - I3);
    
    dposeC_dposeI_.setZero();
    dposeC_dposeI_.block<3,3>(0,0) =  pre_mult_mat;
    dposeC_dposeI_.block<3,3>(3,0) = -CrI * anantak::SkewSymmetricMatrix(I0IipCi_est);
    dposeC_dposeI_.block<3,3>(3,3) =  CrI;
    
    // Scale residual and Jacobians by 1./sqrt(variances)
    double inv_sqrt_var_theta = 1./options_.q_theta;
    double inv_sqrt_var_posn  = 1./options_.q_posn;
    Eigen::Matrix<double,6,1> inv_sqrt_var_vec;
    inv_sqrt_var_vec << inv_sqrt_var_theta, inv_sqrt_var_theta, inv_sqrt_var_theta,
                        inv_sqrt_var_posn, inv_sqrt_var_posn, inv_sqrt_var_posn;
    dposeC_dposeI_ = inv_sqrt_var_vec.asDiagonal() * dposeC_dposeI_;
    dposeC_dposeItoC_ = inv_sqrt_var_vec.asDiagonal() * dposeC_dposeItoC_;
    C0dq_dpCi_ = inv_sqrt_var_vec.asDiagonal() * C0dq_dpCi_;
    
    //VLOG(1) << "  dposeC_dposeItoC_= \n" << dposeC_dposeItoC_;
    //VLOG(1) << "  dposeC_dposeI_= \n" << dposeC_dposeI_;
    
    return true;
  }
  
  // Check if the residual is ready for optimization
  bool Check() {
    return true;
  }
  
  // Get ready for Optimization
  bool GetReadyToOptimize() {
    if (!Check()) {
      LOG(ERROR) << "Check failed. Can not continue.";
      return false;
    }
    // Set the errors to zero
    // Imu error state is set to zero elsewhere
    poseC_->SetErrorZero();     // But we are not going to calculate this. 
    poseItoC_->SetErrorZero();  // This is the main quantity we want to estimate.
    
    return true;
  }
  
  // Evaluate function for ceres solver
  virtual bool Evaluate(
      double const* const* parameters,
      double* residuals,
      double** jacobians) const {
    
    // Assuming Check() has been performed before
    
    // Mapping residuals and parameters to matrices
    MapConstVector15dType  dposeI(parameters[0]);
    MapConstVector6dType   dposeItoC(parameters[1]);
    MapVector6dType        dposeC(residuals);
    
    /* Calculate residual from the error states. Linear model of the residual wrt error states
     * is used to recalculate the residual. */
    dposeC = C0dq_dpCi_ + dposeC_dposeI_*dposeI + dposeC_dposeItoC_*dposeItoC;
    
    /* Jacobians are not recalculated with changing states. We use first estimates Jacobians
     * as this shown to keep the estimator consistent. Also make calculations fast.*/
    if (jacobians != NULL) {
      if (jacobians[0] != NULL) {
        MapMatrix6x15RowType jac(jacobians[0]);
        jac = dposeC_dposeI_;
      }
      if (jacobians[1] != NULL) {
        MapMatrix6x6RowType jac(jacobians[1]);
        jac = dposeC_dposeItoC_;
      }
    }
    
    return true;
  }
  
};  // ImuToCameraPoseResidual


/* Rigid rotation residual
 * Two rotating sensors that are rigidly connected. Each measures its rotation in its own
 * reference frame. We have rotation readings from both sensors. We have an initial guess of
 * the rotation between the sensors and the rotation between the reference frames. Guess changes
 * to the sensor-sensor rotation and frame-frame rotation to reduce the difference in rotations.
 * We would typically set one of the two variables as constant.
 */
class RigidRotationResidual : public ceres::SizedCostFunction<3, 6,6,6,6> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef Eigen::Map<const Eigen::Vector3d> MapConstVector3dType;
  typedef Eigen::Map<const Eigen::Matrix<double,6,1>> MapConstVector6dType;
  typedef Eigen::Map<Eigen::Matrix<double,3,1>> MapVector3dType;
  typedef Eigen::Matrix<double,3,3,Eigen::RowMajor> Matrix3x3RowType;
  typedef Eigen::Map<Eigen::Matrix<double,3,3,Eigen::RowMajor>> MapMatrix3x3RowType;
  typedef Eigen::Matrix<double,3,6,Eigen::RowMajor> Matrix3x6RowType;
  typedef Eigen::Map<Eigen::Matrix<double,3,6,Eigen::RowMajor>> MapMatrix3x6RowType;
  
  struct Options {
    double sigma_theta;
    Options(): sigma_theta(1.*RadiansPerDegree) {}   // in Radians
  };
  
  // Options
  RigidRotationResidual::Options options_;
  
  // Data holders
  double *poseI_quat_;           /**< Pose of Imu sensor in W frame */
  double *poseC_quat_;           /**< Pose of second sensor in T0 frame */
  double *poseItoC_quat_;        /**< Relative pose between sensors */
  double *poseT0toW_quat_;       /**< Relative pose between sensors' reference frames */
  
  // Starting Residuals
  Eigen::Matrix<double,3,1> CidposeC_;   /**< Starting residual */
  
  // Jacobian matrices
  Matrix3x6RowType  dCidposeC_dposeI_;
  Matrix3x6RowType  dCidposeC_dposeC_;
  Matrix3x6RowType  dCidposeC_dposeItoC_;
  Matrix3x6RowType  dCidposeC_dposeT0toW_;
  
  // Default constructor
  RigidRotationResidual():
    options_(),
    poseI_quat_(NULL), poseC_quat_(NULL), poseItoC_quat_(NULL), poseT0toW_quat_(NULL) {
    CidposeC_.setZero();
    dCidposeC_dposeI_.setZero(); dCidposeC_dposeC_.setZero(); dCidposeC_dposeItoC_.setZero();
    dCidposeC_dposeT0toW_.setZero();
  }
  
  // Default copy constructor
  RigidRotationResidual(const RigidRotationResidual& r) {
    options_=r.options_;
    poseI_quat_=r.poseI_quat_; poseC_quat_=r.poseC_quat_;
    poseItoC_quat_=r.poseItoC_quat_; poseT0toW_quat_=r.poseT0toW_quat_;
    CidposeC_=r.CidposeC_; 
    dCidposeC_dposeI_=r.dCidposeC_dposeI_;
    dCidposeC_dposeC_=r.dCidposeC_dposeC_;
    dCidposeC_dposeItoC_=r.dCidposeC_dposeItoC_;
    dCidposeC_dposeT0toW_=r.dCidposeC_dposeT0toW_;
  }
  
  // Destructor
  virtual ~RigidRotationResidual() {}
  
  // Reset residual
  bool Reset() {
    poseI_quat_=NULL; poseC_quat_=NULL; poseItoC_quat_=NULL; poseT0toW_quat_=NULL;
    CidposeC_.setZero();
    dCidposeC_dposeI_.setZero(); dCidposeC_dposeC_.setZero(); dCidposeC_dposeItoC_.setZero();
    dCidposeC_dposeT0toW_.setZero();
    return true;
  }
  
  // Create residual
  bool Create(double *poseI_quat, double *poseC_quat, double *poseItoC_quat,
      double *poseT0toW_quat, RigidRotationResidual::Options *options) {
    if (!poseI_quat) return false;
    if (!poseC_quat) return false;
    if (!poseItoC_quat) return false;
    if (!poseT0toW_quat) return false;
    if (!options) return false;
    // Should we check if any of the other poses are zero too?
    // Should we return false if so?
    
    // Reset the residual
    Reset();
    
    // Assign new values
    poseI_quat_ = poseI_quat;
    poseC_quat_ = poseC_quat;
    poseItoC_quat_ = poseItoC_quat;
    poseT0toW_quat_ = poseT0toW_quat;
    options_ = *options;  // copy options
    if (!CalculateStartingResidualandJacobians()) {
      LOG(ERROR) << "Could not calculate Jacobians. Exit.";
      return false;
    }
    return true;
  }
  
  // Calculate Jacobians
  bool CalculateStartingResidualandJacobians() {
    
    /* All maths here is written in robotics notebook#4, ppg 85-86 */
    
    // Collect the available estimates
    Eigen::Quaterniond IiqW(poseI_quat_);      // State has IiqW
    Eigen::Quaterniond CqI(poseItoC_quat_);    // Stored CqI
    Eigen::Quaterniond CiqT0(poseC_quat_);     // Stored is CiqT0
    Eigen::Quaterniond WqT0(poseT0toW_quat_);  // Stored is WqT0
    
    Eigen::Matrix3d IirW(IiqW);
    Eigen::Matrix3d CrI(CqI);
    Eigen::Matrix3d CirT0(CiqT0);
    Eigen::Matrix3d WrT0(WqT0);
    
    // Calculate starting residuals
    Eigen::Matrix3d I3(Eigen::Matrix3d::Identity());
    Eigen::Quaterniond CiqW = CiqT0 * WqT0.conjugate();
    Eigen::Quaterniond CidqC_est = CiqW * IiqW.conjugate() * CqI.conjugate();
    Eigen::Matrix3d CidrC_est(CidqC_est);
    
    CidposeC_.block<3,1>(0,0) = 2.*CidqC_est.vec();    // Approximation in conversion to angleaxis
    
    // Calculate Jacobians
    
    dCidposeC_dposeI_.setZero();
    dCidposeC_dposeI_.block<3,3>(0,0) = -IirW;
    dCidposeC_dposeI_.block<3,3>(0,0) = CrI*dCidposeC_dposeI_.block<3,3>(0,0);
    
    dCidposeC_dposeC_.setZero();
    dCidposeC_dposeC_.block<3,3>(0,0) = IirW;
    dCidposeC_dposeC_.block<3,3>(0,0) = CrI*dCidposeC_dposeC_.block<3,3>(0,0)*WrT0;
    
    dCidposeC_dposeItoC_.setZero();
    dCidposeC_dposeItoC_.block<3,3>(0,0) = -I3;
    dCidposeC_dposeItoC_.block<3,3>(0,0) = CrI*dCidposeC_dposeItoC_.block<3,3>(0,0);
    
    dCidposeC_dposeT0toW_.setZero();
    dCidposeC_dposeT0toW_.block<3,3>(0,0) = -I3;
    dCidposeC_dposeT0toW_.block<3,3>(0,0) = dCidposeC_dposeC_.block<3,3>(0,0)*dCidposeC_dposeT0toW_.block<3,3>(0,0);
    
    // Scale residual and Jacobians by 1./sqrt(variances)
    double inv_sqrt_var_theta = 1./options_.sigma_theta;
    Eigen::Matrix<double,3,1> inv_sqrt_var_vec;
    inv_sqrt_var_vec << inv_sqrt_var_theta, inv_sqrt_var_theta, inv_sqrt_var_theta;
    CidposeC_             = inv_sqrt_var_vec.asDiagonal() * CidposeC_;
    dCidposeC_dposeI_     = inv_sqrt_var_vec.asDiagonal() * dCidposeC_dposeI_;
    dCidposeC_dposeC_     = inv_sqrt_var_vec.asDiagonal() * dCidposeC_dposeC_;
    dCidposeC_dposeItoC_  = inv_sqrt_var_vec.asDiagonal() * dCidposeC_dposeItoC_;
    dCidposeC_dposeT0toW_ = inv_sqrt_var_vec.asDiagonal() * dCidposeC_dposeT0toW_;
    
    return true;
  }
  
  // Evaluate function for ceres solver
  virtual bool Evaluate(
      double const* const* parameters,
      double* residuals,
      double** jacobians) const {
    
    // Assuming Check() has been performed before
    
    // Mapping residuals and parameters to matrices
    MapConstVector6dType   dposeI(parameters[0]);
    MapConstVector6dType   dposeC(parameters[1]);
    MapConstVector6dType   dposeItoC(parameters[2]);
    MapConstVector6dType   dposeT0toW(parameters[3]);
    MapVector3dType        dCidposeC(residuals);
    
    /* Calculate residual from the error states. Linear model of the residual wrt error states
     * is used to recalculate the residual. */
    dCidposeC = CidposeC_
                + dCidposeC_dposeI_*dposeI
                + dCidposeC_dposeC_*dposeC
                + dCidposeC_dposeItoC_*dposeItoC
                + dCidposeC_dposeT0toW_*dposeT0toW;
    
    /* Jacobians are not recalculated with changing states. We use first estimates Jacobians
     * as this shown to keep the estimator consistent. Also make calculations fast.*/
    if (jacobians != NULL) {
      if (jacobians[0] != NULL) {
        MapMatrix3x6RowType jac(jacobians[0]);
        jac = dCidposeC_dposeI_;
      }
      if (jacobians[1] != NULL) {
        MapMatrix3x6RowType jac(jacobians[1]);
        jac = dCidposeC_dposeC_;
      }
      if (jacobians[2] != NULL) {
        MapMatrix3x6RowType jac(jacobians[2]);
        jac = dCidposeC_dposeItoC_;
      }
      if (jacobians[3] != NULL) {
        MapMatrix3x6RowType jac(jacobians[3]);
        jac = dCidposeC_dposeT0toW_;
      }
    }
    
    return true;
  }
  
};  // RigidRotationResidual

/* Varies the position of IMU and Imu-to-sensor pose - derived from rigid pose contraint
 * Applies a constraint that a sensor pose is rigidly related to the imu */
class RigidPoseWithImuPositionResidual : public ceres::SizedCostFunction<6, 3,6> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Type declarations
  typedef Eigen::Map<Eigen::Matrix<double,6,1>> MapVector6dType;
  typedef Eigen::Map<const Eigen::Matrix<double,6,1>> MapConstVector6dType;
  typedef Eigen::Map<const Eigen::Matrix<double,15,1>> MapConstVector15dType;
  typedef Eigen::Map<const Eigen::Matrix<double,3,1>> MapConstVector3dType;
  typedef Eigen::Matrix<double,6,6,Eigen::RowMajor> Matrix6x6RowType;
  typedef Eigen::Matrix<double,6,3,Eigen::RowMajor> Matrix6x3RowType;
  typedef Eigen::Matrix<double,6,15,Eigen::RowMajor> Matrix6x15RowType;
  typedef Eigen::Map<Eigen::Matrix<double,6,3,Eigen::RowMajor>> MapMatrix6x3RowType;
  typedef Eigen::Map<Eigen::Matrix<double,6,6,Eigen::RowMajor>> MapMatrix6x6RowType;
  typedef Eigen::Map<Eigen::Matrix<double,6,15,Eigen::RowMajor>> MapMatrix6x15RowType;
  
  struct Options {
    double sigma_theta;
    double sigma_position;
    bool rate_of_change;    /**< True if this is a rate of change of pose residual */
    Options() {
      sigma_theta = 2.*RadiansPerDegree;   // in Radians
      sigma_position = 0.005;              // in meters
      rate_of_change = false;              // By default not a rate of change
    }
  };
  
  // Options
  RigidPoseWithImuPositionResidual::Options options_;
  
  // Data holders
  anantak::ImuState *poseI_;          /**< Pose of Imu sensor in W frame */
  anantak::Pose3dState *poseC_;       /**< Pose of second sensor in T0 frame */
  anantak::Pose3dState *poseItoC_;    /**< Relative pose between sensors */
  anantak::Pose3dState *poseT0toW_;   /**< Relative pose between sensors' reference frames */
  
  // Starting Residuals
  Eigen::Matrix<double,6,1> CidposeC_;   /**< Starting residual */
  
  // Jacobian matrices
  Matrix6x15RowType dCidposeC_dposeI_;
  Matrix6x3RowType  dCidposeC_drotnI_;
  Matrix6x3RowType  dCidposeC_dposnI_;
  Matrix6x6RowType  dCidposeC_dposeC_;
  Matrix6x6RowType  dCidposeC_dposeItoC_;
  Matrix6x6RowType  dCidposeC_dposeT0toW_;
  
  // Default constructor
  RigidPoseWithImuPositionResidual():
    options_(),
    poseI_(NULL), poseC_(NULL), poseItoC_(NULL), poseT0toW_(NULL) {
    CidposeC_.setZero();
    dCidposeC_dposeI_.setZero(); dCidposeC_dposeC_.setZero(); dCidposeC_dposeItoC_.setZero();
    dCidposeC_drotnI_.setZero(); dCidposeC_dposnI_.setZero(); 
    dCidposeC_dposeT0toW_.setZero();
  }
  
  // Default copy constructor
  RigidPoseWithImuPositionResidual(const RigidPoseWithImuPositionResidual& r) {
    options_=r.options_;
    poseI_=r.poseI_; poseC_=r.poseC_; poseItoC_=r.poseItoC_; poseT0toW_=r.poseT0toW_;
    CidposeC_=r.CidposeC_; 
    dCidposeC_dposeI_=r.dCidposeC_dposeI_;
    dCidposeC_drotnI_=r.dCidposeC_drotnI_;
    dCidposeC_dposnI_=r.dCidposeC_dposnI_;
    dCidposeC_dposeC_=r.dCidposeC_dposeC_;
    dCidposeC_dposeItoC_=r.dCidposeC_dposeItoC_;
    dCidposeC_dposeT0toW_=r.dCidposeC_dposeT0toW_;
  }
  
  // Destructor
  virtual ~RigidPoseWithImuPositionResidual() {}
  
  // Reset residual
  bool Reset() {
    poseI_=NULL; poseC_=NULL; poseItoC_=NULL; poseT0toW_=NULL;
    CidposeC_.setZero();
    dCidposeC_dposeI_.setZero(); dCidposeC_dposeC_.setZero(); dCidposeC_dposeItoC_.setZero();
    dCidposeC_drotnI_.setZero(); dCidposeC_dposnI_.setZero(); 
    dCidposeC_dposeT0toW_.setZero();
    return true;
  }
  
  // Create residual
  bool Create(anantak::ImuState *poseI, anantak::Pose3dState *poseC,
      anantak::Pose3dState *poseItoC, anantak::Pose3dState *poseT0toW,
      RigidPoseWithImuPositionResidual::Options *options) {
    if (poseI->IsZero()) {
      LOG(WARNING) << "Provided imu pose is zero";
    }
    if (poseC->IsZero()) {
      LOG(WARNING) << "Provided sensor pose is zero";
    }
    // Should we check if any of the other poses are zero too?
    // Should we return false if so?
    
    // Reset the residual
    Reset();
    
    // Assign new values
    poseI_ = poseI;
    poseC_ = poseC;
    poseItoC_ = poseItoC;
    poseT0toW_ = poseT0toW;
    options_ = *options;  // copy options
    if (!CalculateStartingResidualandJacobians()) {
      LOG(ERROR) << "Could not calculate Jacobians. Exit.";
      return false;
    }
    return true;
  }
  
  // Calculate Jacobians
  bool CalculateStartingResidualandJacobians() {
    
    /* All maths here is written in robotics notebook#4, ppg 85-86 */
    
    // Collect the available estimates
    Eigen::Quaterniond IiqW(poseI_->Quaternion());      // State has IiqW
    Eigen::Quaterniond CqI(poseItoC_->Quaternion());    // Stored CqI
    Eigen::Quaterniond CiqT0(poseC_->Quaternion());     // Stored is CiqT0
    Eigen::Quaterniond WqT0(poseT0toW_->Quaternion());  // Stored is WqT0
    
    Eigen::Vector3d WpIi(poseI_->GpI_);
    Eigen::Vector3d IpC(poseItoC_->GpL_);
    Eigen::Vector3d T0pCi(poseC_->GpL_);
    Eigen::Vector3d T0pW(poseT0toW_->GpL_);
    
    Eigen::Matrix3d IirW(IiqW);
    Eigen::Matrix3d CrI(CqI);
    Eigen::Matrix3d CirT0(CiqT0);
    Eigen::Matrix3d WrT0(WqT0);
    
    // Calculate starting residuals
    Eigen::Matrix3d I3(Eigen::Matrix3d::Identity());
    Eigen::Quaterniond CiqW = CiqT0 * WqT0.conjugate();
    Eigen::Vector3d T0WpCi_est = T0pCi - T0pW;
    Eigen::Vector3d WpCi_est = WrT0*T0WpCi_est;
    Eigen::Quaterniond CidqC_est = CiqW * IiqW.conjugate() * CqI.conjugate();
    Eigen::Matrix3d CidrC_est(CidqC_est);
    Eigen::Vector3d WIipCi_est = WpCi_est - WpIi;
    Eigen::Vector3d IipCi_est = IirW*WIipCi_est;
    Eigen::Vector3d CdpCi_est = CrI*(IipCi_est - IpC);
    
    CidposeC_.block<3,1>(0,0) = 2.*CidqC_est.vec();    // Approximation in conversion to angleaxis
    CidposeC_.block<3,1>(3,0) = CdpCi_est;
    
    // Calculate Jacobians
    Matrix6x6RowType CrICrI;
    CrICrI.setZero();
    CrICrI.block<3,3>(0,0) = CrI;
    CrICrI.block<3,3>(3,3) = CrI;
    
    Matrix6x6RowType WrT0WrT0;
    WrT0WrT0.setZero();
    WrT0WrT0.block<3,3>(0,0) = WrT0;
    WrT0WrT0.block<3,3>(3,3) = WrT0;
    
    dCidposeC_dposeI_.setZero();
    dCidposeC_dposeI_.block<3,3>(0,0) = -IirW;
    dCidposeC_dposeI_.block<3,3>(3,0) =  IirW * anantak::SkewSymmetricMatrix(WIipCi_est);
    dCidposeC_dposeI_.block<3,3>(3,3) = -IirW;
    dCidposeC_dposeI_ = CrICrI*dCidposeC_dposeI_;
    
    dCidposeC_drotnI_ = dCidposeC_dposeI_.block<6,3>(0,0);
    dCidposeC_dposnI_ = dCidposeC_dposeI_.block<6,3>(0,3);
    
    dCidposeC_dposeC_.setZero();
    dCidposeC_dposeC_.block<3,3>(0,0) = IirW;
    dCidposeC_dposeC_.block<3,3>(3,3) = IirW;
    dCidposeC_dposeC_ = CrICrI*dCidposeC_dposeC_*WrT0WrT0;
    
    dCidposeC_dposeItoC_.setZero();
    dCidposeC_dposeItoC_.block<3,3>(0,0) = -I3;
    dCidposeC_dposeItoC_.block<3,3>(3,0) =  anantak::SkewSymmetricMatrix(CdpCi_est);
    dCidposeC_dposeItoC_.block<3,3>(3,3) = -I3;
    dCidposeC_dposeItoC_ = CrICrI*dCidposeC_dposeItoC_;
    
    dCidposeC_dposeT0toW_.setZero();
    dCidposeC_dposeT0toW_.block<3,3>(0,0) = -I3;
    dCidposeC_dposeT0toW_.block<3,3>(3,0) =  anantak::SkewSymmetricMatrix(T0WpCi_est);
    dCidposeC_dposeT0toW_.block<3,3>(3,3) = -I3;
    dCidposeC_dposeT0toW_ = dCidposeC_dposeC_*dCidposeC_dposeT0toW_;
    
    // Scale residual and Jacobians by 1./sqrt(variances)
    double inv_sqrt_var_theta = 1./options_.sigma_theta;
    double inv_sqrt_var_posn  = 1./options_.sigma_position;
    Eigen::Matrix<double,6,1> inv_sqrt_var_vec;
    inv_sqrt_var_vec << inv_sqrt_var_theta, inv_sqrt_var_theta, inv_sqrt_var_theta,
                        inv_sqrt_var_posn, inv_sqrt_var_posn, inv_sqrt_var_posn;
    CidposeC_             = inv_sqrt_var_vec.asDiagonal() * CidposeC_;
    dCidposeC_dposeI_     = inv_sqrt_var_vec.asDiagonal() * dCidposeC_dposeI_;
    dCidposeC_drotnI_     = inv_sqrt_var_vec.asDiagonal() * dCidposeC_drotnI_;
    dCidposeC_dposnI_     = inv_sqrt_var_vec.asDiagonal() * dCidposeC_dposnI_;
    dCidposeC_dposeC_     = inv_sqrt_var_vec.asDiagonal() * dCidposeC_dposeC_;
    dCidposeC_dposeItoC_  = inv_sqrt_var_vec.asDiagonal() * dCidposeC_dposeItoC_;
    dCidposeC_dposeT0toW_ = inv_sqrt_var_vec.asDiagonal() * dCidposeC_dposeT0toW_;
    
    //VLOG(1) << "  dCidposeC_dposeI_= \n" << dCidposeC_dposeI_;
    //VLOG(1) << "  dCidposeC_dposeC_= \n" << dCidposeC_dposeC_;
    //VLOG(1) << "  dCidposeC_dposeItoC_= \n" << dCidposeC_dposeItoC_;
    
    return true;
  }
  
  // Check if the residual is ready for optimization
  bool Check() {
    return true;
  }
  
  // Get ready for Optimization
  bool GetReadyToOptimize() {
    if (!Check()) {
      LOG(ERROR) << "Check failed. Can not continue.";
      return false;
    }
    // Set the errors to zero
    // Imu error state is set to zero elsewhere
    poseC_->SetErrorZero();     // But we are not going to calculate this. 
    poseItoC_->SetErrorZero();  // This is the main quantity we want to estimate.
    poseT0toW_->SetErrorZero();
    return true;
  }
  
  // Evaluate function for ceres solver
  virtual bool Evaluate(
      double const* const* parameters,
      double* residuals,
      double** jacobians) const {
    
    // Assuming Check() has been performed before
    
    // Mapping residuals and parameters to matrices
    MapConstVector3dType   posnI(parameters[0]);
    MapConstVector6dType   dposeItoC(parameters[1]);
    MapVector6dType        dCidposeC(residuals);
    
    Eigen::Vector3d dposnI = posnI - poseI_->Position();
    
    /* Calculate residual from the error states. Linear model of the residual wrt error states
     * is used to recalculate the residual. */
    dCidposeC = CidposeC_
                + dCidposeC_dposnI_*dposnI
                + dCidposeC_dposeItoC_*dposeItoC;
    
    /* Jacobians are not recalculated with changing states. We use first estimates Jacobians
     * as this shown to keep the estimator consistent. Also make calculations fast.*/
    if (jacobians != NULL) {
      if (jacobians[0] != NULL) {
        MapMatrix6x3RowType jac(jacobians[0]);
        jac = dCidposeC_dposnI_;
      }
      if (jacobians[1] != NULL) {
        MapMatrix6x6RowType jac(jacobians[1]);
        jac = dCidposeC_dposeItoC_;
      }
    }
    
    return true;
  }
}; // RigidPoseWithImuPositionResidual


/* Rigid Pose With IMU constraint
 * Applies a constraint that a sensor pose is rigidly related to the imu */
class RigidPoseWithImuResidual : public ceres::SizedCostFunction<6, 15,6,6,6> {
 public:
  // Type declarations
  typedef Eigen::Map<Eigen::Matrix<double,6,1>> MapVector6dType;
  typedef Eigen::Map<const Eigen::Matrix<double,6,1>> MapConstVector6dType;
  typedef Eigen::Map<const Eigen::Matrix<double,15,1>> MapConstVector15dType;
  typedef Eigen::Matrix<double,6,6,Eigen::RowMajor> Matrix6x6RowType;
  typedef Eigen::Matrix<double,6,15,Eigen::RowMajor> Matrix6x15RowType;
  typedef Eigen::Map<Eigen::Matrix<double,6,6,Eigen::RowMajor>> MapMatrix6x6RowType;
  typedef Eigen::Map<Eigen::Matrix<double,6,15,Eigen::RowMajor>> MapMatrix6x15RowType;
  
  struct Options {
    double sigma_theta;
    double sigma_position;
    bool rate_of_change;    /**< True if this is a rate of change of pose residual */
    Options() {
      sigma_theta = 2.*RadiansPerDegree;   // in Radians
      sigma_position = 0.005;              // in meters
      rate_of_change = false;              // By default not a rate of change
    }
  };
  
  // Options
  RigidPoseWithImuResidual::Options options_;
  
  // Data holders
  anantak::ImuState *poseI_;          /**< Pose of Imu sensor in W frame */
  anantak::Pose3dState *poseC_;       /**< Pose of second sensor in T0 frame */
  anantak::Pose3dState *poseItoC_;    /**< Relative pose between sensors */
  anantak::Pose3dState *poseT0toW_;   /**< Relative pose between sensors' reference frames */
  
  // Starting Residuals
  Eigen::Matrix<double,6,1> CidposeC_;   /**< Starting residual */
  
  // Jacobian matrices
  Matrix6x15RowType dCidposeC_dposeI_;
  Matrix6x6RowType  dCidposeC_dposeC_;
  Matrix6x6RowType  dCidposeC_dposeItoC_;
  Matrix6x6RowType  dCidposeC_dposeT0toW_;
  
  // Default constructor
  RigidPoseWithImuResidual():
    options_(),
    poseI_(NULL), poseC_(NULL), poseItoC_(NULL), poseT0toW_(NULL) {
    CidposeC_.setZero();
    dCidposeC_dposeI_.setZero(); dCidposeC_dposeC_.setZero(); dCidposeC_dposeItoC_.setZero();
    dCidposeC_dposeT0toW_.setZero();
  }
  
  // Default copy constructor
  RigidPoseWithImuResidual(const RigidPoseWithImuResidual& r) {
    options_=r.options_;
    poseI_=r.poseI_; poseC_=r.poseC_; poseItoC_=r.poseItoC_; poseT0toW_=r.poseT0toW_;
    CidposeC_=r.CidposeC_; 
    dCidposeC_dposeI_=r.dCidposeC_dposeI_;
    dCidposeC_dposeC_=r.dCidposeC_dposeC_;
    dCidposeC_dposeItoC_=r.dCidposeC_dposeItoC_;
    dCidposeC_dposeT0toW_=r.dCidposeC_dposeT0toW_;
  }
  
  // Destructor
  virtual ~RigidPoseWithImuResidual() {}
  
  // Reset residual
  bool Reset() {
    poseI_=NULL; poseC_=NULL; poseItoC_=NULL; poseT0toW_=NULL;
    CidposeC_.setZero();
    dCidposeC_dposeI_.setZero(); dCidposeC_dposeC_.setZero(); dCidposeC_dposeItoC_.setZero();
    dCidposeC_dposeT0toW_.setZero();
    return true;
  }
  
  // Create residual
  bool Create(anantak::ImuState *poseI, anantak::Pose3dState *poseC,
      anantak::Pose3dState *poseItoC, anantak::Pose3dState *poseT0toW,
      RigidPoseWithImuResidual::Options *options) {
    if (poseI->IsZero()) {
      LOG(WARNING) << "Provided imu pose is zero";
    }
    if (poseC->IsZero()) {
      LOG(WARNING) << "Provided sensor pose is zero";
    }
    // Should we check if any of the other poses are zero too?
    // Should we return false if so?
    
    // Reset the residual
    Reset();
    
    // Assign new values
    poseI_ = poseI;
    poseC_ = poseC;
    poseItoC_ = poseItoC;
    poseT0toW_ = poseT0toW;
    options_ = *options;  // copy options
    if (!CalculateStartingResidualandJacobians()) {
      LOG(ERROR) << "Could not calculate Jacobians. Exit.";
      return false;
    }
    return true;
  }
  
  // Calculate Jacobians
  bool CalculateStartingResidualandJacobians() {
    
    /* All maths here is written in robotics notebook#4, ppg 85-86 */
    
    // Collect the available estimates
    Eigen::Quaterniond IiqW(poseI_->Quaternion());      // State has IiqW
    Eigen::Quaterniond CqI(poseItoC_->Quaternion());    // Stored CqI
    Eigen::Quaterniond CiqT0(poseC_->Quaternion());     // Stored is CiqT0
    Eigen::Quaterniond WqT0(poseT0toW_->Quaternion());  // Stored is WqT0
    
    Eigen::Vector3d WpIi(poseI_->GpI_);
    Eigen::Vector3d IpC(poseItoC_->GpL_);
    Eigen::Vector3d T0pCi(poseC_->GpL_);
    Eigen::Vector3d T0pW(poseT0toW_->GpL_);
    
    Eigen::Matrix3d IirW(IiqW);
    Eigen::Matrix3d CrI(CqI);
    Eigen::Matrix3d CirT0(CiqT0);
    Eigen::Matrix3d WrT0(WqT0);
    
    // Calculate starting residuals
    Eigen::Matrix3d I3(Eigen::Matrix3d::Identity());
    Eigen::Quaterniond CiqW = CiqT0 * WqT0.conjugate();
    Eigen::Vector3d T0WpCi_est = T0pCi - T0pW;
    Eigen::Vector3d WpCi_est = WrT0*T0WpCi_est;
    Eigen::Quaterniond CidqC_est = CiqW * IiqW.conjugate() * CqI.conjugate();
    Eigen::Matrix3d CidrC_est(CidqC_est);
    Eigen::Vector3d WIipCi_est = WpCi_est - WpIi;
    Eigen::Vector3d IipCi_est = IirW*WIipCi_est;
    Eigen::Vector3d CdpCi_est = CrI*(IipCi_est - IpC);
    
    CidposeC_.block<3,1>(0,0) = 2.*CidqC_est.vec();    // Approximation in conversion to angleaxis
    CidposeC_.block<3,1>(3,0) = CdpCi_est;
    
    // Calculate Jacobians
    Matrix6x6RowType CrICrI;
    CrICrI.setZero();
    CrICrI.block<3,3>(0,0) = CrI;
    CrICrI.block<3,3>(3,3) = CrI;
    
    Matrix6x6RowType WrT0WrT0;
    WrT0WrT0.setZero();
    WrT0WrT0.block<3,3>(0,0) = WrT0;
    WrT0WrT0.block<3,3>(3,3) = WrT0;
    
    dCidposeC_dposeI_.setZero();
    dCidposeC_dposeI_.block<3,3>(0,0) = -IirW;
    dCidposeC_dposeI_.block<3,3>(3,0) =  IirW * anantak::SkewSymmetricMatrix(WIipCi_est);
    dCidposeC_dposeI_.block<3,3>(3,3) = -IirW;
    dCidposeC_dposeI_ = CrICrI*dCidposeC_dposeI_;
    
    dCidposeC_dposeC_.setZero();
    dCidposeC_dposeC_.block<3,3>(0,0) = IirW;
    dCidposeC_dposeC_.block<3,3>(3,3) = IirW;
    dCidposeC_dposeC_ = CrICrI*dCidposeC_dposeC_*WrT0WrT0;
    
    dCidposeC_dposeItoC_.setZero();
    dCidposeC_dposeItoC_.block<3,3>(0,0) = -I3;
    dCidposeC_dposeItoC_.block<3,3>(3,0) =  anantak::SkewSymmetricMatrix(CdpCi_est);
    dCidposeC_dposeItoC_.block<3,3>(3,3) = -I3;
    dCidposeC_dposeItoC_ = CrICrI*dCidposeC_dposeItoC_;
    
    dCidposeC_dposeT0toW_.setZero();
    dCidposeC_dposeT0toW_.block<3,3>(0,0) = -I3;
    dCidposeC_dposeT0toW_.block<3,3>(3,0) =  anantak::SkewSymmetricMatrix(T0WpCi_est);
    dCidposeC_dposeT0toW_.block<3,3>(3,3) = -I3;
    dCidposeC_dposeT0toW_ = dCidposeC_dposeC_*dCidposeC_dposeT0toW_;
    
    // Scale residual and Jacobians by 1./sqrt(variances)
    double inv_sqrt_var_theta = 1./options_.sigma_theta;
    double inv_sqrt_var_posn  = 1./options_.sigma_position;
    Eigen::Matrix<double,6,1> inv_sqrt_var_vec;
    inv_sqrt_var_vec << inv_sqrt_var_theta, inv_sqrt_var_theta, inv_sqrt_var_theta,
                        inv_sqrt_var_posn, inv_sqrt_var_posn, inv_sqrt_var_posn;
    CidposeC_             = inv_sqrt_var_vec.asDiagonal() * CidposeC_;
    dCidposeC_dposeI_     = inv_sqrt_var_vec.asDiagonal() * dCidposeC_dposeI_;
    dCidposeC_dposeC_     = inv_sqrt_var_vec.asDiagonal() * dCidposeC_dposeC_;
    dCidposeC_dposeItoC_  = inv_sqrt_var_vec.asDiagonal() * dCidposeC_dposeItoC_;
    dCidposeC_dposeT0toW_ = inv_sqrt_var_vec.asDiagonal() * dCidposeC_dposeT0toW_;
    
    //VLOG(1) << "  dCidposeC_dposeI_= \n" << dCidposeC_dposeI_;
    //VLOG(1) << "  dCidposeC_dposeC_= \n" << dCidposeC_dposeC_;
    //VLOG(1) << "  dCidposeC_dposeItoC_= \n" << dCidposeC_dposeItoC_;
    
    return true;
  }
  
  // Check if the residual is ready for optimization
  bool Check() {
    return true;
  }
  
  // Get ready for Optimization
  bool GetReadyToOptimize() {
    if (!Check()) {
      LOG(ERROR) << "Check failed. Can not continue.";
      return false;
    }
    // Set the errors to zero
    // Imu error state is set to zero elsewhere
    poseC_->SetErrorZero();     // But we are not going to calculate this. 
    poseItoC_->SetErrorZero();  // This is the main quantity we want to estimate.
    poseT0toW_->SetErrorZero();
    return true;
  }
  
  // Evaluate function for ceres solver
  virtual bool Evaluate(
      double const* const* parameters,
      double* residuals,
      double** jacobians) const {
    
    // Assuming Check() has been performed before
    
    // Mapping residuals and parameters to matrices
    MapConstVector15dType  dposeI(parameters[0]);
    MapConstVector6dType   dposeC(parameters[1]);
    MapConstVector6dType   dposeItoC(parameters[2]);
    MapConstVector6dType   dposeT0toW(parameters[3]);
    MapVector6dType        dCidposeC(residuals);
    
    /* Calculate residual from the error states. Linear model of the residual wrt error states
     * is used to recalculate the residual. */
    dCidposeC = CidposeC_
                + dCidposeC_dposeI_*dposeI
                + dCidposeC_dposeC_*dposeC
                + dCidposeC_dposeItoC_*dposeItoC
                + dCidposeC_dposeT0toW_*dposeT0toW;
    
    /* Jacobians are not recalculated with changing states. We use first estimates Jacobians
     * as this shown to keep the estimator consistent. Also make calculations fast.*/
    if (jacobians != NULL) {
      if (jacobians[0] != NULL) {
        MapMatrix6x15RowType jac(jacobians[0]);
        jac = dCidposeC_dposeI_;
      }
      if (jacobians[1] != NULL) {
        MapMatrix6x6RowType jac(jacobians[1]);
        jac = dCidposeC_dposeC_;
      }
      if (jacobians[2] != NULL) {
        MapMatrix6x6RowType jac(jacobians[2]);
        jac = dCidposeC_dposeItoC_;
      }
      if (jacobians[3] != NULL) {
        MapMatrix6x6RowType jac(jacobians[3]);
        jac = dCidposeC_dposeT0toW_;
      }
    }
    
    return true;
  }
}; // RigidPoseWithImuResidual


/* Rigid Pose With IMU Change constraint
 * Applies a constraint that a sensor pose is rigidly related to the imu
 * Maths here is in Robotics notebook#4 ppg 89-90 */
class RigidPoseWithImuChangeResidual : public ceres::SizedCostFunction<6, 15,6,15,6,6> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Type declarations
  typedef Eigen::Map<Eigen::Matrix<double,6,1>> MapVector6dType;
  typedef Eigen::Map<const Eigen::Matrix<double,6,1>> MapConstVector6dType;
  typedef Eigen::Map<const Eigen::Matrix<double,15,1>> MapConstVector15dType;
  typedef Eigen::Matrix<double,6,6,Eigen::RowMajor> Matrix6x6RowType;
  typedef Eigen::Matrix<double,6,15,Eigen::RowMajor> Matrix6x15RowType;
  typedef Eigen::Map<Eigen::Matrix<double,6,6,Eigen::RowMajor>> MapMatrix6x6RowType;
  typedef Eigen::Map<Eigen::Matrix<double,6,15,Eigen::RowMajor>> MapMatrix6x15RowType;
  
  struct Options {
    double sigma_theta;
    double sigma_position;
    bool rate_of_change;    /**< True if this is a rate of change of pose residual */
    Options() {
      sigma_theta = 60.*RadiansPerDegree;  // in Radians/s /sqrt(Hz)
      sigma_position = 0.100;              // in meters/s /sqrt(Hz)
      rate_of_change = true;               // By default this is a rate of change
    }
  };
  
  // Options
  RigidPoseWithImuChangeResidual::Options options_;
  
  // Data holders
  anantak::ImuState *poseI_;          /**< Pose of Imu sensor in W frame */
  anantak::Pose3dState *poseC_;       /**< Pose of second sensor in T0 frame */
  anantak::ImuState *poseI1_;         /**< Pose of Imu sensor in W frame */
  anantak::Pose3dState *poseC1_;      /**< Pose of second sensor in T0 frame */
  anantak::Pose3dState *poseT0toW_;   /**< Relative pose between sensors' reference frames */
  
  // Starting Residuals
  Eigen::Matrix<double,6,1> CdposeC1_;   /**< Starting residual */
  
  // Jacobian matrices
  Matrix6x15RowType dCdposeC1_dposeI_;
  Matrix6x6RowType  dCdposeC1_dposeC_;
  Matrix6x15RowType dCdposeC1_dposeI1_;
  Matrix6x6RowType  dCdposeC1_dposeC1_;
  Matrix6x6RowType  dCdposeC1_dposeT0toW_;
  
  // Default constructor
  RigidPoseWithImuChangeResidual():
    options_(),
    poseI_(NULL), poseC_(NULL), poseI1_(NULL), poseC1_(NULL) {
    CdposeC1_.setZero();
    dCdposeC1_dposeI_.setZero(); dCdposeC1_dposeC_.setZero();
    dCdposeC1_dposeI1_.setZero(); dCdposeC1_dposeC1_.setZero();
    dCdposeC1_dposeT0toW_.setZero();
  }
  
  // Default copy constructor
  RigidPoseWithImuChangeResidual(const RigidPoseWithImuChangeResidual& r) {
    options_=r.options_;
    poseI_=r.poseI_; poseC_=r.poseC_; poseI1_=r.poseI1_; poseC1_=r.poseC1_;
    CdposeC1_=r.CdposeC1_; 
    dCdposeC1_dposeI_=r.dCdposeC1_dposeI_;
    dCdposeC1_dposeC_=r.dCdposeC1_dposeC_;
    dCdposeC1_dposeI1_=r.dCdposeC1_dposeI1_;
    dCdposeC1_dposeC1_=r.dCdposeC1_dposeC1_;
    dCdposeC1_dposeT0toW_=r.dCdposeC1_dposeT0toW_;
  }
  
  // Destructor
  virtual ~RigidPoseWithImuChangeResidual() {}
  
  // Reset residual
  bool Reset() {
    poseI_=NULL; poseC_=NULL; poseI1_=NULL; poseC1_=NULL;
    CdposeC1_.setZero();
    dCdposeC1_dposeI_.setZero(); dCdposeC1_dposeC_.setZero();
    dCdposeC1_dposeI1_.setZero(); dCdposeC1_dposeC1_.setZero();
    dCdposeC1_dposeT0toW_.setZero();
    return true;
  }
  
  // Create residual
  bool Create(anantak::ImuState *poseI, anantak::Pose3dState *poseC,
      anantak::ImuState *poseI1, anantak::Pose3dState *poseC1, anantak::Pose3dState *poseT0toW,
      RigidPoseWithImuChangeResidual::Options *options) {
    if (poseI->IsZero()) {
      LOG(WARNING) << "Provided imu pose is zero";
    }
    if (poseC->IsZero()) {
      LOG(WARNING) << "Provided sensor pose is zero";
    }
    if (poseI1->IsZero()) {
      LOG(WARNING) << "Provided imu pose is zero";
    }
    if (poseC1->IsZero()) {
      LOG(WARNING) << "Provided sensor pose is zero";
    }
    // Make sure that timestamps are correct
    if (double(poseI->timestamp_) < Epsilon) {
      LOG(ERROR) << "Cannot have zero timestamp for pose change residual";
      return false;
    }
    if (double(poseI1->timestamp_) < Epsilon) {
      LOG(ERROR) << "Cannot have zero timestamp for pose change residual";
      return false;
    }
    if (poseI->timestamp_ >= poseI1->timestamp_) {
      LOG(ERROR) << "Can not have poseI->timestamp_ >= poseI1->timestamp_";
      return false;
    }
    // Should we check if any of the other poses are zero too?
    // Should we return false if so?
    
    // Reset the residual
    Reset();
    
    // Assign new values
    poseI_ = poseI;
    poseC_ = poseC;
    poseI1_ = poseI1;
    poseC1_ = poseC1;
    poseT0toW_ = poseT0toW;
    options_ = *options;  // copy options
    if (!CalculateStartingResidualandJacobians()) {
      LOG(ERROR) << "Could not calculate Jacobians. Exit.";
      return false;
    }
    return true;
  }
  
  // Calculate Jacobians
  bool CalculateStartingResidualandJacobians() {
    
    /* Maths here is written in robotics notebook#4, ppg 89-90 */
    
    // Collect the available estimates
    Eigen::Quaterniond IqW(poseI_->Quaternion());       // State has IqW
    Eigen::Quaterniond CqT0(poseC_->Quaternion());      // Stored is CqT0
    Eigen::Quaterniond I1qW(poseI1_->Quaternion());     // State has I1qW
    Eigen::Quaterniond C1qT0(poseC1_->Quaternion());    // Stored is C1qT0
    Eigen::Quaterniond WqT0(poseT0toW_->Quaternion());  // Stored is WqT0
    
    Eigen::Vector3d WpI(poseI_->GpI_);
    Eigen::Vector3d T0pC(poseC_->GpL_);
    Eigen::Vector3d WpI1(poseI1_->GpI_);
    Eigen::Vector3d T0pC1(poseC1_->GpL_);
    Eigen::Vector3d T0pW(poseT0toW_->GpL_);
    
    Eigen::Matrix3d IrW(IqW);
    Eigen::Matrix3d CrT0(CqT0);
    Eigen::Matrix3d I1rW(I1qW);
    Eigen::Matrix3d C1rT0(C1qT0);
    Eigen::Matrix3d WrT0(WqT0);
    
    const Eigen::Matrix3d I3(Eigen::Matrix3d::Identity());
    
    // First pose  WpC
    Eigen::Quaterniond CqW = CqT0 * WqT0.conjugate();
    Eigen::Vector3d T0WpC = T0pC - T0pW; 
    Eigen::Vector3d WpC = WrT0 * T0WpC;
    
    Matrix6x6RowType dWposeC_dT0poseC;
    dWposeC_dT0poseC.setZero();
    dWposeC_dT0poseC.block<3,3>(0,0) =  WrT0;
    dWposeC_dT0poseC.block<3,3>(3,3) =  WrT0;
    
    Matrix6x6RowType dWposeC_dT0poseW;
    dWposeC_dT0poseW.setZero();
    dWposeC_dT0poseW.block<3,3>(0,0) = -WrT0;
    dWposeC_dT0poseW.block<3,3>(3,0) =  WrT0*anantak::SkewSymmetricMatrix(T0WpC);
    dWposeC_dT0poseW.block<3,3>(3,3) = -WrT0;
    
    // Second pose  WpC1
    Eigen::Quaterniond C1qW = C1qT0 * WqT0.conjugate();
    Eigen::Vector3d T0WpC1 = T0pC1 - T0pW; 
    Eigen::Vector3d WpC1 = WrT0 * T0WpC1;
    
    Matrix6x6RowType dWposeC1_dT0poseC1;
    dWposeC1_dT0poseC1.setZero();
    dWposeC1_dT0poseC1.block<3,3>(0,0) =  WrT0;
    dWposeC1_dT0poseC1.block<3,3>(3,3) =  WrT0;
    
    Matrix6x6RowType dWposeC1_dT0poseW;
    dWposeC1_dT0poseW.setZero();
    dWposeC1_dT0poseW.block<3,3>(0,0) = -WrT0;
    dWposeC1_dT0poseW.block<3,3>(3,0) =  WrT0*anantak::SkewSymmetricMatrix(T0WpC1);
    dWposeC1_dT0poseW.block<3,3>(3,3) = -WrT0;
    
    // First pose  IpC
    Eigen::Quaterniond CqI = CqW * IqW.conjugate();
    Eigen::Vector3d WIpC = WpC - WpI;
    Eigen::Vector3d IpC = IrW * WIpC;
    
    Matrix6x6RowType dIposeC_dWposeC;
    dIposeC_dWposeC.setZero();
    dIposeC_dWposeC.block<3,3>(0,0) =  IrW;
    dIposeC_dWposeC.block<3,3>(3,3) =  IrW;
    
    Matrix6x15RowType dIposeC_dWposeI;
    dIposeC_dWposeI.setZero();
    dIposeC_dWposeI.block<3,3>(0,0) = -IrW;
    dIposeC_dWposeI.block<3,3>(3,0) =  IrW*anantak::SkewSymmetricMatrix(WIpC);
    dIposeC_dWposeI.block<3,3>(3,3) = -IrW;
    
    // Second pose  I1pC1
    Eigen::Quaterniond C1qI1 = C1qW * I1qW.conjugate();
    Eigen::Vector3d WI1pC1 = WpC1 - WpI1;
    Eigen::Vector3d I1pC1 = I1rW * WI1pC1;
    
    Matrix6x6RowType dI1poseC1_dWposeC1;
    dI1poseC1_dWposeC1.setZero();
    dI1poseC1_dWposeC1.block<3,3>(0,0) =  I1rW;
    dI1poseC1_dWposeC1.block<3,3>(3,3) =  I1rW;
    
    Matrix6x15RowType dI1poseC1_dWposeI1;
    dI1poseC1_dWposeI1.setZero();
    dI1poseC1_dWposeI1.block<3,3>(0,0) = -I1rW;
    dI1poseC1_dWposeI1.block<3,3>(3,0) =  I1rW*anantak::SkewSymmetricMatrix(WI1pC1);
    dI1poseC1_dWposeI1.block<3,3>(3,3) = -I1rW;
    
    // Finally the error pose  CdpC1
    Eigen::Quaterniond C1dqC = C1qI1 * CqI.conjugate();
    Eigen::Vector3d ICdpC1 = I1pC1 - IpC;
    Eigen::Matrix3d CrI(CqI);
    Eigen::Vector3d CdpC1 = CrI * ICdpC1;
    
    Matrix6x6RowType dCdposeC1_dI1poseC1;
    dCdposeC1_dI1poseC1.setZero();
    dCdposeC1_dI1poseC1.block<3,3>(0,0) =  CrI;
    dCdposeC1_dI1poseC1.block<3,3>(3,3) =  CrI;
    
    Matrix6x6RowType dCdposeC1_dIposeC;
    dCdposeC1_dIposeC.setZero();
    dCdposeC1_dIposeC.block<3,3>(0,0) = -CrI;
    dCdposeC1_dIposeC.block<3,3>(3,0) =  CrI*anantak::SkewSymmetricMatrix(ICdpC1);
    dCdposeC1_dIposeC.block<3,3>(3,3) = -CrI;
    
    // Starting residual
    CdposeC1_.block<3,1>(0,0) = 2.*C1dqC.vec(); // Approximation
    CdposeC1_.block<3,1>(3,0) = CdpC1;
    
    // Starting Jacobians
    dCdposeC1_dposeI_ = dCdposeC1_dIposeC * dIposeC_dWposeI;
    dCdposeC1_dposeC_ = dCdposeC1_dIposeC * dIposeC_dWposeC * dWposeC_dT0poseC;
    dCdposeC1_dposeI1_ = dCdposeC1_dI1poseC1 * dI1poseC1_dWposeI1;
    dCdposeC1_dposeC1_ = dCdposeC1_dI1poseC1 * dI1poseC1_dWposeC1 * dWposeC1_dT0poseC1;
    dCdposeC1_dposeT0toW_ = dCdposeC1_dIposeC * dIposeC_dWposeC * dWposeC_dT0poseW
                          + dCdposeC1_dI1poseC1 * dI1poseC1_dWposeC1 * dWposeC1_dT0poseW;
    
    double dt = double(poseI1_->timestamp_ - poseI_->timestamp_)*1e-6;
    if (dt < Epsilon) {
      LOG(ERROR) << "dt <= 0";
      return false;
    }
    
    // Scale residual and Jacobians by 1./sqrt(variances)
    double inv_sqrt_var_theta = 1./(options_.sigma_theta*dt);
    double inv_sqrt_var_posn  = 1./(options_.sigma_position*dt);
    Eigen::Matrix<double,6,1> inv_sqrt_var_vec;
    inv_sqrt_var_vec << inv_sqrt_var_theta, inv_sqrt_var_theta, inv_sqrt_var_theta,
                        inv_sqrt_var_posn, inv_sqrt_var_posn, inv_sqrt_var_posn;
    CdposeC1_             = inv_sqrt_var_vec.asDiagonal() * CdposeC1_;
    dCdposeC1_dposeI_     = inv_sqrt_var_vec.asDiagonal() * dCdposeC1_dposeI_;
    dCdposeC1_dposeC_     = inv_sqrt_var_vec.asDiagonal() * dCdposeC1_dposeC_;
    dCdposeC1_dposeI1_    = inv_sqrt_var_vec.asDiagonal() * dCdposeC1_dposeI1_;
    dCdposeC1_dposeC1_    = inv_sqrt_var_vec.asDiagonal() * dCdposeC1_dposeC1_;
    dCdposeC1_dposeT0toW_ = inv_sqrt_var_vec.asDiagonal() * dCdposeC1_dposeT0toW_;
    
    return true;
  }
  
  // Check if the residual is ready for optimization
  bool Check() {
    return true;
  }
  
  // Get ready for Optimization
  bool GetReadyToOptimize() {
    if (!Check()) {
      LOG(ERROR) << "Check failed. Can not continue.";
      return false;
    }
    // Set the errors to zero is done elsewhere
    return true;
  }
  
  // Evaluate function for ceres solver
  virtual bool Evaluate(
      double const* const* parameters,
      double* residuals,
      double** jacobians) const {
    
    // Assuming Check() has been performed before
    
    // Mapping residuals and parameters to matrices
    MapVector6dType        CdposeC1(residuals);
    MapConstVector15dType  dposeI(parameters[0]);
    MapConstVector6dType   dposeC(parameters[1]);
    MapConstVector15dType  dposeI1(parameters[2]);
    MapConstVector6dType   dposeC1(parameters[3]);
    MapConstVector6dType   dposeT0toW(parameters[4]);
    
    /* Calculate residual from the error states. Linear model of the residual wrt error states
     * is used to recalculate the residual. */
    CdposeC1 = CdposeC1_
              + dCdposeC1_dposeI_*dposeI
              + dCdposeC1_dposeC_*dposeC
              + dCdposeC1_dposeI1_*dposeI
              + dCdposeC1_dposeC1_*dposeC1
              + dCdposeC1_dposeT0toW_*dposeT0toW;
    
    /* Jacobians are not recalculated with changing states. We use first estimates Jacobians
     * as this shown to keep the estimator consistent. Also make calculations fast.*/
    if (jacobians != NULL) {
      if (jacobians[0] != NULL) {
        MapMatrix6x15RowType jac(jacobians[0]);
        jac = dCdposeC1_dposeI_;
      }
      if (jacobians[1] != NULL) {
        MapMatrix6x6RowType jac(jacobians[1]);
        jac = dCdposeC1_dposeC_;
      }
      if (jacobians[2] != NULL) {
        MapMatrix6x15RowType jac(jacobians[2]);
        jac = dCdposeC1_dposeI1_;
      }
      if (jacobians[3] != NULL) {
        MapMatrix6x6RowType jac(jacobians[3]);
        jac = dCdposeC1_dposeC1_;
      }
      if (jacobians[4] != NULL) {
        MapMatrix6x6RowType jac(jacobians[4]);
        jac = dCdposeC1_dposeT0toW_;
      }
    }
    
    return true;
  }
}; // RigidPoseWithImuChangeResidual


/* AprilTag VIO Residual - for static tags only
 * April tag view is 4 corners in an image. So residuals are:
 *  Four corner image coordinates, 8x1 residual
 * April tag view from any camera is linked to:
 *  Previous IMU-state-in-W-frame, 15x1 error
 *  IMU gravity state-in-W-frame, 3x1 error
 *  IMU-to-camera pose, 6x1 error
 *  Tag-map-to-W-frame pose, 6x1 error - this can be broken 3x1, 3x1 to keep one part constant
 *  Tag-in-Tag-map pose, 6x1 error for tag pose + 1x1 error in tag size = 7x1 error
 *  Camera matrix, 4x1 error [fu,fv,cu,cv]
 * This residual builds up the residuals and jacobians for tag views in VIO.
 */
/*class AprilTagVioResidual: public ceres::SizedCostFunction<8, 15,3,6,3,3,6,1,4> {
 public:
  
  // Type declarations
  typedef Eigen::Map<Eigen::Matrix<double,8,1>> MapVector8dType;
  typedef Eigen::Map<const Eigen::Matrix<double,1,1>> MapConstVector1dType;
  typedef Eigen::Map<const Eigen::Matrix<double,3,1>> MapConstVector3dType;
  typedef Eigen::Map<const Eigen::Matrix<double,4,1>> MapConstVector4dType;
  typedef Eigen::Map<const Eigen::Matrix<double,6,1>> MapConstVector6dType;
  typedef Eigen::Map<const Eigen::Matrix<double,15,1>> MapConstVector15dType;
  typedef Eigen::Matrix<double,8,1> Matrix8x1RowType;
  typedef Eigen::Matrix<double,8,3,Eigen::RowMajor> Matrix8x3RowType;
  typedef Eigen::Matrix<double,8,4,Eigen::RowMajor> Matrix8x4RowType;
  typedef Eigen::Matrix<double,8,6,Eigen::RowMajor> Matrix8x6RowType;
  typedef Eigen::Matrix<double,8,15,Eigen::RowMajor> Matrix8x15RowType;
  typedef Eigen::Map<Eigen::Matrix<double,8,1>> MapMatrix8x1RowType;
  typedef Eigen::Map<Eigen::Matrix<double,8,3,Eigen::RowMajor>> MapMatrix8x3RowType;
  typedef Eigen::Map<Eigen::Matrix<double,8,4,Eigen::RowMajor>> MapMatrix8x4RowType;
  typedef Eigen::Map<Eigen::Matrix<double,8,6,Eigen::RowMajor>> MapMatrix8x6RowType;
  typedef Eigen::Map<Eigen::Matrix<double,8,15,Eigen::RowMajor>> MapMatrix8x15RowType;
  typedef Eigen::Matrix<double,2,1> Matrix2x1Type;
  typedef Eigen::Matrix<double,2,3> Matrix2x3Type;
  typedef Eigen::Matrix<double,2,4> Matrix2x4Type;
  typedef Eigen::Matrix<double,2,6> Matrix2x6Type;
  typedef Eigen::Matrix<double,3,4> Matrix3x4Type;
  typedef Eigen::Matrix<double,3,6> Matrix3x6Type;
  typedef Eigen::Matrix<double,6,3> Matrix6x3Type;
  typedef Eigen::Matrix<double,6,6> Matrix6x6Type;
  typedef Eigen::Matrix<double,6,15> Matrix6x15Type;
  typedef Eigen::Matrix<double,8,8> Matrix8x8Type;
  typedef Eigen::Matrix<double,15,1> Matrix15x1Type;
  typedef Eigen::Map<Eigen::Matrix<double,2,4>> MapMatrix2x4Type;
  
  // Options object used by this residual
  struct Options {
    bool use_first_estimates;   // Should we re-evaluate Jacobians at every estimate? 
    // Noise parameters
    double q_image;             // Stdev of u,v location of corner in the image 
    Options() {
      use_first_estimates = true;       // Generally we use first estimate jacobians 
      q_image = 0.5;                    // 1-sigma conrner location in pixels 
    }
  };
  
  // Data members
  AprilTagVioResidual::Options options_;
  
  // Observation
  const anantak::AprilTagReadingType *tag_view_;
  
  // States that this residual constrains
  anantak::ImuState *poseI_;                // Pose of IMU in W frame 
  anantak::Vector3dState* gravity_;         // Gravity vector in W frame 
  anantak::Pose3dState *poseItoC_;          // Camera pose in IMU body frame 
  anantak::Pose3dState *poseWtoT0_;         // Tag map pose in W frame 
  anantak::StaticAprilTagState *tagTj_;     // Pose and size of tag Tj in T0 frame 
  anantak::CameraIntrinsicsState *camera_;  // Camera intrinsics state 
  
  // IMU residual used to interpolate the IMU integrals
  const anantak::ImuResidualFunction *imu_constraint_;  // IMU residual used to interpolate 
  
  // Starting residual
  Eigen::Matrix<double,8,1> tagview_residual_;   // Starting residual 
  
  // Jacobian matrices
  Matrix8x15RowType   dtagview_dposeI_;
  Matrix8x3RowType    dtagview_dgravity_;
  Matrix8x6RowType    dtagview_dposeItoC_;
  Matrix8x3RowType    dtagview_drotnWtoT0_;
  Matrix8x3RowType    dtagview_dposnWtoT0_;
  Matrix8x6RowType    dtagview_dposeT0toTj_;
  Matrix8x1RowType    dtagview_dTj_size_;
  Matrix8x4RowType    dtagview_dK_;             // K is the camera matrix
  
  // Default constructor
  AprilTagVioResidual() :
    options_(),
    poseI_(NULL), gravity_(NULL), poseItoC_(NULL), poseWtoT0_(NULL),
    tagTj_(NULL), camera_(NULL), tag_view_(NULL), imu_constraint_(NULL) {
    tagview_residual_.setZero();
    dtagview_dposeI_.setZero();
    dtagview_dgravity_.setZero();
    dtagview_dposeItoC_.setZero();
    dtagview_drotnWtoT0_.setZero();
    dtagview_dposnWtoT0_.setZero();
    dtagview_dposeT0toTj_.setZero();
    dtagview_dTj_size_.setZero();
    dtagview_dK_.setZero();
  }
  
  // Default copy constructor
  AprilTagVioResidual(const AprilTagVioResidual& r) {
    options_=r.options_;
    poseI_=r.poseI_; gravity_=r.gravity_; poseItoC_=r.poseItoC_; poseWtoT0_=r.poseWtoT0_;
    tagTj_=r.tagTj_; camera_ = r.camera_; tag_view_ = r.tag_view_;
    imu_constraint_ = r.imu_constraint_;
    tagview_residual_ = r.tagview_residual_;
    dtagview_dposeI_ = r.dtagview_dposeI_;
    dtagview_dgravity_ = r.dtagview_dgravity_;
    dtagview_dposeItoC_ = r.dtagview_dposeItoC_;
    dtagview_drotnWtoT0_ = r.dtagview_drotnWtoT0_;
    dtagview_dposnWtoT0_ = r.dtagview_dposnWtoT0_;
    dtagview_dposeT0toTj_ = r.dtagview_dposeT0toTj_;
    dtagview_dTj_size_ = r.dtagview_dTj_size_;
    dtagview_dK_ = r.dtagview_dK_;
  }
  
  // Destructor
  virtual ~AprilTagVioResidual() {}
  
  // Reset residual
  bool Reset() {
    // options_ are not reset
    poseI_ = NULL; gravity_ = NULL; poseItoC_ = NULL; poseWtoT0_ = NULL; tagTj_ = NULL;
    camera_ = NULL; tag_view_ = NULL; imu_constraint_ = NULL;
    tagview_residual_.setZero();
    dtagview_dposeI_.setZero();
    dtagview_dgravity_.setZero();
    dtagview_dposeItoC_.setZero();
    dtagview_drotnWtoT0_.setZero();
    dtagview_dposnWtoT0_.setZero();
    dtagview_dposeT0toTj_.setZero();
    dtagview_dTj_size_.setZero();
    dtagview_dK_.setZero();
    return true;
  }
  
  // Create residual - this allows to reuse the memory already assigned for this residual
  bool Create(const anantak::AprilTagReadingType *tag_view,
      anantak::ImuState *poseI, anantak::Vector3dState* gravity,
      anantak::Pose3dState *poseItoC, anantak::Pose3dState *poseWtoT0,
      anantak::StaticAprilTagState *tagTj, anantak::CameraIntrinsicsState *camera,
      const anantak::ImuResidualFunction *imu_constraint,
      bool is_zero_tag = false) {
    if (tag_view->IsZero()) {
      LOG(ERROR) << "Provided tag view is zero";
      return false;
    }
    if (poseI->IsZero()) {
      LOG(WARNING) << "Provided imu pose is zero";
    }
    if (gravity->IsZero()) {
      LOG(ERROR) << "Provided gravity is zero. Can not continue.";
      return false;
    }
    if (poseItoC->IsZero()) {
      LOG(WARNING) << "Provided ItoC pose is zero";
    }
    if (poseWtoT0->IsZero()) {
      LOG(WARNING) << "Provided WtoT0 pose is zero";
    }
    if (tagTj->IsZero() && !is_zero_tag) {
      LOG(WARNING) << "Provided non-origin tagTj pose is zero";
    }
    if (camera->IsZero()) {
      LOG(ERROR) << "Provided camera is zero. Can not continue";
      return false;
    }
    if (!imu_constraint->IsClosed()) {
      LOG(ERROR) << "Provided IMU constraint is not closed. Can not use it for interpolation";
      return false;
    }
    
    // Reset the residual
    Reset();
    
    // Assign pointers to states
    tag_view_ = tag_view;     // Observation of tag corners
    poseI_ = poseI;           // IMU pose in World frame
    gravity_ = gravity;       // IMU gravity vector in World frame
    poseItoC_ = poseItoC;     // IMU to Camera pose state
    poseWtoT0_ = poseWtoT0;   // TagMap pose in World state
    tagTj_ = tagTj;           // Tag pose in TagMap state
    camera_ = camera;         // Camera intrinsics state
    imu_constraint_ = imu_constraint;   // IMU constraint used for interpolation
    
    // Calculate starting Residuals and Jacobians
    if (!CalculateStartingResidualandJacobians()) {
      LOG(ERROR) << "Could not calculate Jacobians. Exit.";
      return false;
    }
    return true;
  }
  
  // Create residual - along with options
  bool Create(const anantak::AprilTagReadingType *tag_view,
      anantak::ImuState *poseI, anantak::Vector3dState* gravity,
      anantak::Pose3dState *poseItoC, anantak::Pose3dState *poseWtoT0,
      anantak::StaticAprilTagState *tagTj, anantak::CameraIntrinsicsState *camera,
      const anantak::ImuResidualFunction *imu_constraint,
      AprilTagVioResidual::Options *options,
      bool is_zero_tag = false) {
    if (options->q_image < Epsilon) {
      LOG(WARNING) << "Provided image corner sqrt variance is zero.";
    }
    options_ = *options;  // copy options as these are kept for life of the residual
    return Create(tag_view, poseI, gravity, poseItoC, poseWtoT0, tagTj, camera, imu_constraint,
                  is_zero_tag);
  }
  
  // Calculate starting estimates of Residual and Jacobians.
  bool CalculateStartingResidualandJacobians() {
    
    // We have IqW,WqI, IqC,CpI, T0qW,WpT0, TjqT0,T0pTj, K from states
    // We have the observation of the corners
    // We have a pointer to the residual that will be used to interpolate imu integrals
    //
    // t time instants are where IMU states are established
    // i time instants are where camera images with tag views are seen
    // j is index of a tag in tag map
    // 
    // IirW,WpIi IMU state at the camera timestamp is calculated by propagation of IMU state at t
    // This is done by linear interpolation (added interpolation noise) using IMU residual.
    
    ImuEstimatesIntegralType imu_integral;  // create a blank integral
    if (!imu_constraint_->InterpolateIntegrals(tag_view_->timestamp, &imu_integral)) {
      LOG(ERROR) << "Could not interpolate integral. Can not continue.";
      return false;
    }
    //VLOG(1) << "  WpIi imu = " << imu_constraint_->integral_.p1.transpose();
    
    Eigen::Matrix3d IirW = imu_integral.r1.conjugate().toRotationMatrix(); // Integral stores WrI
    Eigen::Vector3d WpIi = imu_integral.p1;
    
    Eigen::Vector3d Wg = gravity_->Position();  // Gravity in World frame
    
    Eigen::Matrix3d CrI = poseItoC_->Quaternion().toRotationMatrix();
    Eigen::Vector3d IpC = poseItoC_->Position();
    
    Eigen::Matrix3d T0rW = poseWtoT0_->Quaternion().toRotationMatrix();
    Eigen::Vector3d WpT0 = poseWtoT0_->Position();
    
    Eigen::Matrix3d TjrT0 = tagTj_->pose_.Quaternion().toRotationMatrix();
    Eigen::Vector3d T0pTj = tagTj_->pose_.Position();
    
    Matrix3x4Type Tpf = anantak::AprilTag3dCorners(tagTj_->size_.Value());
    
    Eigen::Matrix3d K = camera_->CameraMatrix();
    
    // Estimates are calculated as follows:
    //  UVij = [X/Z, Y/Z]ij
    //  XYZij = K*CipfTj
    //  CipfTj = CirT0 * ( T0pTj - T0pCi + T0rTj*Tpf )  .. for each corner feature
    //  CirT0 = CrI * IirW * WrT0
    //  T0pCi = T0rW * ( WpIi + WrIi*IpC - WpT0 )
    
    Eigen::Matrix3d CirT0 = CrI * IirW * T0rW.transpose();
    Eigen::Vector3d WIipCi = IirW.transpose()*IpC;            // we break calculations to reuse
    Eigen::Vector3d WT0pCi = WpIi + WIipCi - WpT0;            // intermediate results later
    Eigen::Vector3d T0pCi = T0rW * WT0pCi;
    
    // Report
    //VLOG(1) << "  IpC   = " << IpC.transpose();
    //VLOG(1) << "  WpIi  = " << WpIi.transpose();
    //VLOG(1) << "  T0pCi = " << T0pCi.transpose();
    
    Matrix3x4Type T0Tjpf = TjrT0.transpose() * Tpf;
    Eigen::Vector3d T0CipTj = T0pTj - T0pCi;
    Matrix3x4Type T0CipfTj = T0Tjpf.colwise() + T0CipTj;
    Matrix3x4Type CipfTj = CirT0 * T0CipfTj;
    
    Matrix3x4Type XYZij = K * CipfTj;
    Matrix2x4Type UVij;
    UVij.row(0) = XYZij.row(0).cwiseQuotient(XYZij.row(2));
    UVij.row(1) = XYZij.row(1).cwiseQuotient(XYZij.row(2));
    
    // Residual can now be calculated as difference between observation and estimate
    // We use estimate - observation as jacobians (dResidual_dErrorState) are kept positive
    // As all errors are assumed gaussian (thus symmetric around 0) this makes no difference
    MapMatrix2x4Type residual(tagview_residual_.data());
    residual = UVij - tag_view_->image_coords;
    
    // Report
    //Eigen::Matrix<double,2,8> tag_coords_mat;
    //tag_coords_mat.block<2,4>(0,0) = UVij;
    //tag_coords_mat.block<2,4>(0,4) = tag_view_->image_coords;
    //VLOG(1) << "Projected, viewed tag coordinates = \n" << tag_coords_mat;
    //VLOG(1) << "Residual tag coordinates = \n" << residual;
    
     * Errors are:
     *  dUVij     2x1   derivative of the residual                                  LHS (param#)
     *  dXYZij    3x1                                                             dependent var
     *  CidpfTj   3x1   derivative of position of corner in camera frame          dependent var
     *  dT0pCi    6x1   derivative of pose that includes rotation and position    dependent var
     *  dT0pTj    6x1   derivative of pose of tag in tag map                        RHS   (6)
     *  Tdpf      3x1   derivative of the position of corner on tag                 RHS   (7)
     *  dWpIi     6x1   derivative of pose Ii in world frame                      dependent var
     *  dIpC      6x1   derivative of pose of camera in IMU body frame              RHS   (2)
     *  dWpT0     6x1   derivative of tag map pose in world frame                   RHS
     *   WdrT0     3x1   derivative of tag map rotation in world frame               RHS  (4)
     *   WdpT0     3x1   derivative of tag map position in world frame               RHS  (5)
     *  dWsIt     15x1  derivative of IMU state in world frame at time t            RHS   (1)
     *  Wdg       3x1   derivative of gravity vector in world frame                 RHS   (2)
     *  dK        4x1   derivative of camera intrinsics parameters vector           RHS   (8)
     *
     * Residuals are related with errors using linearized relationships as follows:
     *  dUVij = dUVij_dXYZij * dXYZij
     *  dXYZij = ^CipfTj^ * dK  + K * CidpfTj
     *  CidpfTj = CidpfTj_dT0pCi * dT0pCi + CidpfTj_dT0pTj * dT0pTj + CidpfTj_Tdpf * Tdpf
     *   Tdpf = Tdpf_TjSz * dTjSz
     *  dT0pCi = dT0pCi_dWpIi * dWpIi + dT0pCi_dIpC * dIpC + dT0pCi_dWpT0 * dWpT0
     *   dT0pCi = dT0pCi_dWpIi * dWpIi + dT0pCi_dIpC * dIpC + dT0pCi_WdrT0 * WdrT0 + dT0pCi_WdpT0 * WdpT0
     *  dWpIi = dWpIi_dWsIt * dWsIt + dWpIi_Wdg * Wdg
     *
     *  Here Jacobians have the following dimensions (for a single corner)
     *  dUVij_dXYZij    2x3     ... for all four corners this will be 8x3
     *  CidpfTj_dT0pCi  3x6
     *  CidpfTj_dT0pTj  3x6
     *  CidpfTj_Tdpf    3x3
     *   Tdpf_TjSz       3x1
     *  dT0pCi_dWpIi    6x6
     *  dT0pCi_dIpC     6x6
     *  dT0pCi_dWpT0    6x6
     *   dT0pCi_WdrT0    6x3
     *   dT0pCi_WdpT0    6x3
     *  dWpIi_dWsIt     6x15
     *  dWpIi_Wdg       6x3
     *
     * Jacobians are calculated as follows:
     *
     *  ^CipfTj^ = [ CipfTj_x  0  CipfTj_z  0 ; 0  CipfTj_y  0  CipfTj_z ; 0  0  0  0 ]
     *
     *  dUVij_dXYZij = 1/Z * [ 1 0 -X/Z ; 0 1 -Y/Z ]
     *
     *  CidpfTj_dT0pCi = CirT0 * [  \T0pTj - T0pCi + T0rTj*Tpf x\   -I3 ]
     *
     *  CidpfTj_dT0pTj = CirT0 * [ -\T0rTj*Tpf x\    I3 ]
     *
     *  CidpfTj_Tdpf = CirT0 * T0rTj
     *   Tdpf_TjSz = anantak::AprilTag3dCorners(1.)  ... for kth corner use kth column
     *   CidpfTj_TjSz = CidpfTj_Tdpf * Tdpf_TjSz
     *
     *  dT0pCi_dWpIi = T0rW * [ I3  03 ; -\WrIi*IpC x\  I3 ]
     *
     *  dT0pCi_dIpC = T0rW * [ WrIi  03 ; 03  WrIi ]
     *
     *  dT0pCi_dWpT0 = T0rW * [ -I3  03 ; \WpIi + WrIi*IpC - WpT0 x\ -I3 ]
     *   dT0pCi_WdrT0 = T0rW * [ -I3 ; \WpIi + WrIi*IpC - WpT0 x\ ]
     *   dT0pCi_WdpT0 = T0rW * [ 03 ; -I3 ]
     *
     *   ii = imu_integral
     *  dWpIi_dWsIt = [ I3  03  03  -ii.V  03 ; -\ii.y x\  I3  ii.dt*I3  ii.MM  -ii.P ]
     *  dWpIi_Wdg = [ 03 ; 0.5*ii.dt*ii.dt*I3 ]
     *  
     *
     * Combining above equations, we get:
     *  dUVij =   + dUVij_dXYZij * ^CipfTj^ * dK 
     *            + dUVij_dXYZij * K * CidpfTj_dT0pCi * dT0pCi_dWpIi * dWpIi_dWsIt * dWsIt
     *            + dUVij_dXYZij * K * CidpfTj_dT0pCi * dT0pCi_dWpIi * dWpIi_Wdg * Wdg
     *            + dUVij_dXYZij * K * CidpfTj_dT0pCi * dT0pCi_dIpC * dIpC
     *            + dUVij_dXYZij * K * CidpfTj_dT0pCi * dT0pCi_WdrT0 * WdrT0
     *            + dUVij_dXYZij * K * CidpfTj_dT0pCi * dT0pCi_WdpT0 * WdpT0
     *            + dUVij_dXYZij * K * CidpfTj_dT0pTj * dT0pTj
     *            + dUVij_dXYZij * K * CidpfTj_Tdpf * Tdpf_TjSz * dTjSz
     *
     * Giving us expressions for Jacobians:
     *  dtagview_dK_ =          dUVij_dXYZij * ^CipfTj^
     *  dtagview_dposeI_ =      dUVij_dXYZij * K * CidpfTj_dT0pCi * dT0pCi_dWpIi * dWpIi_dWsIt
     *  dtagview_dgravity_ =    dUVij_dXYZij * K * CidpfTj_dT0pCi * dT0pCi_dWpIi * dWpIi_Wdg
     *  dtagview_dposeItoC_ =   dUVij_dXYZij * K * CidpfTj_dT0pCi * dT0pCi_dIpC
     *  dtagview_drotnWtoT0_ =  dUVij_dXYZij * K * CidpfTj_dT0pCi * dT0pCi_WdrT0
     *  dtagview_dposnWtoT0_ =  dUVij_dXYZij * K * CidpfTj_dT0pCi * dT0pCi_WdpT0
     *  dtagview_dposeT0toTj_ = dUVij_dXYZij * K * CidpfTj_dT0pTj
     *  dtagview_dTj_size_ =    dUVij_dXYZij * K * CidpfTj_TjSz
     *
    
    Eigen::Matrix3d Z3(Eigen::Matrix3d::Zero());
    Eigen::Matrix3d I3(Eigen::Matrix3d::Identity());
    
    Matrix3x4Type Tdpf_TjSz = anantak::AprilTag3dCorners(1.);
    
    Matrix6x6Type dT0pCi_dWpIi;
    dT0pCi_dWpIi.block<3,3>(0,0) =  T0rW;
    dT0pCi_dWpIi.block<3,3>(0,3) =  Z3;
    dT0pCi_dWpIi.block<3,3>(3,0) = -T0rW * anantak::SkewSymmetricMatrix(WIipCi);
    dT0pCi_dWpIi.block<3,3>(3,3) =  T0rW;
    
    Matrix6x6Type dT0pCi_dIpC;
    dT0pCi_dIpC.block<3,3>(0,0) =  T0rW * IirW.transpose();
    dT0pCi_dIpC.block<3,3>(0,3) =  Z3;
    dT0pCi_dIpC.block<3,3>(3,0) =  Z3;
    dT0pCi_dIpC.block<3,3>(3,3) =  T0rW * IirW.transpose();  // could we copy the first block?
    
    Matrix6x3Type dT0pCi_WdrT0;
    dT0pCi_WdrT0.block<3,3>(0,0) = -T0rW;
    dT0pCi_WdrT0.block<3,3>(3,0) =  T0rW * anantak::SkewSymmetricMatrix(WT0pCi);
    
    Matrix6x3Type dT0pCi_WdpT0;
    dT0pCi_WdpT0.block<3,3>(0,0) =  Z3;
    dT0pCi_WdpT0.block<3,3>(3,0) = -T0rW;
    
    Matrix6x15Type dWpIi_dWsIt;
    dWpIi_dWsIt.block<3,3>(0,0)  =  I3;
    dWpIi_dWsIt.block<3,3>(0,3)  =  Z3;
    dWpIi_dWsIt.block<3,3>(0,6)  =  Z3;
    dWpIi_dWsIt.block<3,3>(0,9)  = -imu_integral.V;
    dWpIi_dWsIt.block<3,3>(0,12) =  Z3;
    dWpIi_dWsIt.block<3,3>(3,0)  = -anantak::SkewSymmetricMatrix(imu_integral.y);
    dWpIi_dWsIt.block<3,3>(3,3)  =  I3;
    dWpIi_dWsIt.block<3,3>(3,6)  =  imu_integral.dt * I3;
    dWpIi_dWsIt.block<3,3>(3,9)  =  imu_integral.MM;
    dWpIi_dWsIt.block<3,3>(3,12) = -imu_integral.P;
    
    Matrix6x3Type dWpIi_Wdg;
    dWpIi_Wdg.block<3,3>(0,0) =  Z3;
    dWpIi_Wdg.block<3,3>(3,0) =  0.5 * imu_integral.dt * imu_integral.dt * I3;
    
    // Calculate jacobians for each corner one-by-one and assign to jacobian matrices
    for (int i_crnr=0; i_crnr<4; i_crnr++) {
      
      Eigen::Vector3d proj_point = CipfTj.col(i_crnr);
      Matrix3x4Type proj_point_dK_jac;
      proj_point_dK_jac <<
          proj_point[0], 0., proj_point[2], 0.,
          0., proj_point[1], 0., proj_point[2],
          0., 0., 0., 0.;
      
      Eigen::Vector3d proj_point_K = XYZij.col(i_crnr);
      Matrix2x3Type dUVij_dXYZij;
      dUVij_dXYZij <<
          1., 0., -proj_point_K[0]/proj_point_K[2],
          0., 1., -proj_point_K[1]/proj_point_K[2];
      dUVij_dXYZij *= 1./proj_point_K[2];
      
      Matrix3x6Type CidpfTj_dT0pCi;
      CidpfTj_dT0pCi.block<3,3>(0,0) =  CirT0 * anantak::SkewSymmetricMatrix(T0CipfTj.col(i_crnr));
      CidpfTj_dT0pCi.block<3,3>(0,3) = -CirT0;
      
      Matrix3x6Type CidpfTj_dT0pTj;
      CidpfTj_dT0pTj.block<3,3>(0,0) = -CirT0 * anantak::SkewSymmetricMatrix(T0Tjpf.col(i_crnr));
      CidpfTj_dT0pTj.block<3,3>(0,3) =  CirT0;
      
      Eigen::Vector3d CidpfTj_TjSz;
      CidpfTj_TjSz = CirT0 * TjrT0.transpose() * Tdpf_TjSz.col(i_crnr);
      
      // Intermediate matrices to reduce calculations
      Matrix2x3Type dUVij_dXYZij_K = dUVij_dXYZij * K;
      Matrix2x4Type dUVij_dK       = dUVij_dXYZij * proj_point_dK_jac;
      Matrix2x6Type dUVij_dT0pCi   = dUVij_dXYZij_K * CidpfTj_dT0pCi;
      Matrix2x6Type dUVij_dT0pTj   = dUVij_dXYZij_K * CidpfTj_dT0pTj;
      Matrix2x1Type dUVij_TjSz     = dUVij_dXYZij_K * CidpfTj_TjSz;
      
      // Calculate Jacobians
      dtagview_dK_.block<2,4>(2*i_crnr,0)          = dUVij_dK;
      dtagview_dposeI_.block<2,15>(2*i_crnr,0)     = dUVij_dT0pCi * dT0pCi_dWpIi * dWpIi_dWsIt;
      dtagview_dgravity_.block<2,3>(2*i_crnr,0)    = dUVij_dT0pCi * dT0pCi_dWpIi * dWpIi_Wdg;
      dtagview_dposeItoC_.block<2,6>(2*i_crnr,0)   = dUVij_dT0pCi * dT0pCi_dIpC;
      dtagview_drotnWtoT0_.block<2,3>(2*i_crnr,0)  = dUVij_dT0pCi * dT0pCi_WdrT0;
      dtagview_dposnWtoT0_.block<2,3>(2*i_crnr,0)  = dUVij_dT0pCi * dT0pCi_WdpT0;
      dtagview_dposeT0toTj_.block<2,6>(2*i_crnr,0) = dUVij_dT0pTj;
      dtagview_dTj_size_.block<2,1>(2*i_crnr,0)    = dUVij_TjSz;
      
    }
    
     * Noises and uncertainties
     * Residual calculates conditional distributions of errors given the states. So we take the
     * states as fixed and see how measurement and process noise will affect our estimates. Here
     * we fix IMU state at time t, gravity, camera matrix, Imu to camera pose, World to Tag map
     * pose, Tagj pose in tag map and Size of Tag. Under these fixed (given) conditions, what is
     * the distribution of tag coordinates?
     *
     * Process noise in our model comes from IMU's bias drifts.
     * Measurement noise comes from IMU's reading noise and Tag readings noise.
     * Uncertainty in states is addressed by setting priors on the problem.
     * 
     *
    
    // IMU process and measurement noise is calculated using the imu_integral
    Matrix15x1Type poseI_var;      
    imu_integral.StateNoiseVariance(&poseI_var);
    
    Matrix8x8Type dtagview_dposeI_cov =
        dtagview_dposeI_ * poseI_var.asDiagonal() * dtagview_dposeI_.transpose();
    
    // We do not know how to calculate inv sqrt of a general square matrix. We use its diagonal.
    //  We are neglecting off-diagonal covariance terms. This is an approximation, not too bad hopefully
    Matrix8x1RowType dtagview_cov_diag = dtagview_dposeI_cov.diagonal()
        + options_.q_image * options_.q_image * Matrix8x1RowType::Ones();
    // Convert this to inv sqrt variance to use it in scaling the residual and jacobians
    dtagview_cov_diag = dtagview_cov_diag.cwiseSqrt().cwiseInverse();
    
    // Scale the residual and Jacobians
    tagview_residual_     = dtagview_cov_diag.asDiagonal() * tagview_residual_;
    dtagview_dposeI_      = dtagview_cov_diag.asDiagonal() * dtagview_dposeI_;
    dtagview_dgravity_    = dtagview_cov_diag.asDiagonal() * dtagview_dgravity_;
    dtagview_dposeItoC_   = dtagview_cov_diag.asDiagonal() * dtagview_dposeItoC_;
    dtagview_drotnWtoT0_  = dtagview_cov_diag.asDiagonal() * dtagview_drotnWtoT0_;
    dtagview_dposnWtoT0_  = dtagview_cov_diag.asDiagonal() * dtagview_dposnWtoT0_;
    dtagview_dposeT0toTj_ = dtagview_cov_diag.asDiagonal() * dtagview_dposeT0toTj_;
    dtagview_dTj_size_    = dtagview_cov_diag.asDiagonal() * dtagview_dTj_size_;
    dtagview_dK_          = dtagview_cov_diag.asDiagonal() * dtagview_dK_;
    
    return true;
  }
  
  // Check if the residual is ready for optimization
  bool Check() {
    return true;
  }
  
  // Get ready for Optimization
  bool GetReadyToOptimize() {
    if (!Check()) {
      LOG(ERROR) << "Check failed. Can not continue.";
      return false;
    }
    // Set the errors to zero
    //  Imu error state is set to zero elsewhere
    //  ItoC error is set to zero elsewhere
    //  WtoT0 error is set to zero elsewhere
    //  TagTj error is set to zero elsewhere
    return true;
  }
  
  // Evaluate function for ceres solver
  virtual bool Evaluate(
      double const* const* parameters,
      double* residuals,
      double** jacobians) const {
    
    // Assuming Check() has been performed before
    
    // Mapping residuals and parameters to matrices: AprilTagVioResidual<8, 15,3,6,3,3,6,1,4>
    MapVector8dType        tagview_resid(residuals);
    MapConstVector15dType  dposeI(parameters[0]);
    MapConstVector3dType   dgravity(parameters[1]);
    MapConstVector6dType   dposeItoC(parameters[2]);
    MapConstVector3dType   drotnWtoT0(parameters[3]);
    MapConstVector3dType   dposnWtoT0(parameters[4]);
    MapConstVector6dType   dposeT0toTj(parameters[5]);
    MapConstVector1dType   dTj_size(parameters[6]);
    MapConstVector4dType   dK(parameters[7]);
    
     * Calculate residual from the error states. Linear model of the residual wrt error states
     * is used to recalculate the residual. *
    tagview_resid =
        tagview_residual_ 
        + dtagview_dposeI_ * dposeI
        + dtagview_dgravity_ * dgravity
        + dtagview_dposeItoC_ * dposeItoC
        + dtagview_drotnWtoT0_ * drotnWtoT0
        + dtagview_dposnWtoT0_ * dposnWtoT0
        + dtagview_dposeT0toTj_ * dposeT0toTj
        + dtagview_dTj_size_ * dTj_size
        + dtagview_dK_ * dK;
    
     * Jacobians are not recalculated with changing states. We use first estimates Jacobians
     * as this shown to keep the estimator consistent. Also makes calculations faster.*
    if (jacobians != NULL) {
      if (jacobians[0] != NULL) {
        MapMatrix8x15RowType jac(jacobians[0]);
        jac = dtagview_dposeI_;
      }
      if (jacobians[1] != NULL) {
        MapMatrix8x3RowType jac(jacobians[1]);
        jac = dtagview_dgravity_;
      }
      if (jacobians[2] != NULL) {
        MapMatrix8x6RowType jac(jacobians[2]);
        jac = dtagview_dposeItoC_;
      }
      if (jacobians[3] != NULL) {
        MapMatrix8x3RowType jac(jacobians[3]);
        jac = dtagview_drotnWtoT0_;
      }
      if (jacobians[4] != NULL) {
        MapMatrix8x3RowType jac(jacobians[4]);
        jac = dtagview_dposnWtoT0_;
      }
      if (jacobians[5] != NULL) {
        MapMatrix8x6RowType jac(jacobians[5]);
        jac = dtagview_dposeT0toTj_;
      }
      if (jacobians[6] != NULL) {
        MapMatrix8x1RowType jac(jacobians[6]);
        jac = dtagview_dTj_size_;
      }
      if (jacobians[7] != NULL) {
        MapMatrix8x4RowType jac(jacobians[7]);
        jac = dtagview_dK_;
      }
    }
    
    return true;
  }
  
}; // AprilTagVioResidual */


/* April Tag View residual - for static tags
 * This constrains Camera pose, tag pose and camera matrix given a tag view
 * All maths here in in Robotics Notebook #4 ppg 91-92
 */
class AprilTagViewResidual: public ceres::SizedCostFunction<8, 6,6,1,4> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Type declarations
  typedef Eigen::Map<Eigen::Matrix<double,8,1>> MapVector8dType;
  typedef Eigen::Map<const Eigen::Matrix<double,6,1>> MapConstVector6dType;
  typedef Eigen::Map<const Eigen::Matrix<double,1,1>> MapConstVector1dType;
  typedef Eigen::Map<const Eigen::Matrix<double,4,1>> MapConstVector4dType;
  typedef Eigen::Map<Eigen::Matrix<double,8,6,Eigen::RowMajor>> MapMatrix8x6RowType;
  typedef Eigen::Map<Eigen::Matrix<double,8,4,Eigen::RowMajor>> MapMatrix8x4RowType;
  typedef Eigen::Map<Eigen::Matrix<double,8,1>> MapMatrix8x1RowType;
  typedef Eigen::Matrix<double,8,6,Eigen::RowMajor> Matrix8x6RowType;
  typedef Eigen::Matrix<double,8,1> Matrix8x1RowType;
  typedef Eigen::Matrix<double,8,4,Eigen::RowMajor> Matrix8x4RowType;
  typedef Eigen::Matrix<double,6,6,Eigen::RowMajor> Matrix6x6RowType;
  typedef Eigen::Matrix<double,8,8,Eigen::RowMajor> Matrix8x8RowType;
  typedef Eigen::Matrix<double,3,4> Matrix3x4Type;
  typedef Eigen::Matrix<double,2,4,Eigen::RowMajor> Matrix2x4RowType;
  typedef Eigen::Matrix<double,2,2,Eigen::RowMajor> Matrix2x2RowType;
  typedef Eigen::Matrix<double,2,3,Eigen::RowMajor> Matrix2x3RowType;
  typedef Eigen::Matrix<double,3,6,Eigen::RowMajor> Matrix3x6RowType;
  
  // Options object used by this residual
  struct Options {
    bool use_first_estimates;   /**< Should we re-evaluate Jacobians at every estimate? */
    // Noise parameters
    double sigma_image;             /**< Stdev of u,v location of corner in the image */
    Options() {
      use_first_estimates = true;       /**< Generally we use first estimate jacobians */
      sigma_image = 1.0;                /**< 1-sigma corner location in pixels */
    }
  };
  
  // Data members
  AprilTagViewResidual::Options options_;
  
  // Observation
  const anantak::AprilTagReadingType *tag_view_;
  
  // States that this residual constrains
  anantak::Pose3dState *poseC_;             /**< Camera pose tag map frame */
  anantak::StaticAprilTagState *tagTj_;     /**< Tj tag pose in tag map frame and size of tag */
  anantak::CameraIntrinsicsState *camera_;  /**< Camera intrinsics state */
  
  // Starting residual
  Eigen::Matrix<double,8,1> tagview_residual_;   /**< Starting residual */
  
  // Jacobian matrices
  Matrix8x6RowType    dtagview_dposeC_;
  Matrix8x6RowType    dtagview_dposeT0toTj_;
  Matrix8x1RowType    dtagview_dTj_size_;
  Matrix8x4RowType    dtagview_dK_;             // K is the camera matrix
  
  // Timestamp representing the residual - usually used to decide if residual should be used
  int64_t timestamp_;
  
  // Default constructor
  AprilTagViewResidual() :
    options_(),
    tag_view_(NULL),
    poseC_(NULL), tagTj_(NULL), camera_(NULL), timestamp_(0) {
    tagview_residual_.setZero();
    dtagview_dposeC_.setZero();
    dtagview_dposeT0toTj_.setZero();
    dtagview_dTj_size_.setZero();
    dtagview_dK_.setZero();
  }
  
  // Default copy constructor
  AprilTagViewResidual(const AprilTagViewResidual& r) {
    options_=r.options_;
    tag_view_ = r.tag_view_;
    poseC_=r.poseC_; tagTj_=r.tagTj_; camera_ = r.camera_; 
    tagview_residual_ = r.tagview_residual_;
    dtagview_dposeC_ = r.dtagview_dposeC_;
    dtagview_dposeT0toTj_ = r.dtagview_dposeT0toTj_;
    dtagview_dTj_size_ = r.dtagview_dTj_size_;
    dtagview_dK_ = r.dtagview_dK_;
    timestamp_ = r.timestamp_;
  }
  
  // Equal to assignment operator
  AprilTagViewResidual& operator= (const AprilTagViewResidual& r) {
    options_=r.options_;
    tag_view_ = r.tag_view_;
    poseC_=r.poseC_; tagTj_=r.tagTj_; camera_ = r.camera_; 
    tagview_residual_ = r.tagview_residual_;
    dtagview_dposeC_ = r.dtagview_dposeC_;
    dtagview_dposeT0toTj_ = r.dtagview_dposeT0toTj_;
    dtagview_dTj_size_ = r.dtagview_dTj_size_;
    dtagview_dK_ = r.dtagview_dK_;
    timestamp_ = r.timestamp_;
  }
  
  // Destructor
  virtual ~AprilTagViewResidual() {}
  
  // Reset residual
  bool Reset() {
    // options_ are not reset
    tag_view_ = NULL;
    poseC_ = NULL; tagTj_ = NULL; camera_ = NULL;
    tagview_residual_.setZero();
    dtagview_dposeC_.setZero();
    dtagview_dposeT0toTj_.setZero();
    dtagview_dTj_size_.setZero();
    dtagview_dK_.setZero();
    timestamp_=0;
    return true;
  }
  
  // Create residual - this allows to reuse the memory already assigned for this residual
  bool Create(const anantak::AprilTagReadingType *tag_view, anantak::Pose3dState *poseC,
      anantak::StaticAprilTagState *tagTj, anantak::CameraIntrinsicsState *camera,
      bool is_zero_tag = false) {
    if (tag_view->IsZero()) {
      LOG(ERROR) << "Provided tag view is zero";
      return false;
    }
    if (poseC->IsZero()) {
      LOG(WARNING) << "Provided camera pose is zero";
    }
    if (tagTj->IsZero() && !is_zero_tag) {
      LOG(WARNING) << "Provided non-origin tagTj pose is zero";
    }
    if (camera->IsZero()) {
      LOG(ERROR) << "Provided camera is zero. Can not continue";
      return false;
    }
    
    // Reset the residual
    Reset();
    
    // Assign pointers to states
    tag_view_ = tag_view;     // Observation of tag corners
    poseC_ = poseC;           // Camera pose in TagMap state
    tagTj_ = tagTj;           // Tag pose in TagMap state
    camera_ = camera;         // Camera intrinsics state
    timestamp_ = poseC_->timestamp_;
    
    // Calculate starting Residuals and Jacobians
    if (!CalculateStartingResidualandJacobians()) {
      LOG(ERROR) << "Could not calculate Jacobians. Exit.";
      return false;
    }
    return true;
  }
  
  // Create residual - along with options
  bool Create(const anantak::AprilTagReadingType *tag_view, anantak::Pose3dState *poseC,
      anantak::StaticAprilTagState *tagTj, anantak::CameraIntrinsicsState *camera,
      AprilTagViewResidual::Options *options,
      bool is_zero_tag = false) {
    if (options->sigma_image < Epsilon) {
      LOG(WARNING) << "Provided image corner sqrt variance is zero.";
    }
    options_ = *options;  // copy options as these are kept for life of the residual
    return Create(tag_view, poseC, tagTj, camera, is_zero_tag);
  }
  
  // Calculate starting estimates of Residual and Jacobians.
  bool CalculateStartingResidualandJacobians() {
    
    Eigen::Matrix3d I3(Eigen::Matrix3d::Identity());
    
    Eigen::Quaterniond CiqT0(poseC_->Quaternion());
    Eigen::Quaterniond TjqT0(tagTj_->pose_.Quaternion());
    
    Eigen::Vector3d T0pCi(poseC_->Position());
    Eigen::Vector3d T0pTj(tagTj_->pose_.Position());
    
    Eigen::Matrix3d CirT0(CiqT0);
    
    Eigen::Quaterniond TjqCi = TjqT0 * CiqT0.conjugate();
    Eigen::Vector3d T0CipTj = T0pTj - T0pCi;
    Eigen::Vector3d CipTj = CirT0 * T0CipTj;
    
    Matrix6x6RowType CirT0CirT0;
    CirT0CirT0.setZero();
    CirT0CirT0.block<3,3>(0,0) =  CirT0;
    CirT0CirT0.block<3,3>(3,3) =  CirT0;
    
    Matrix6x6RowType dCiposeTj_dT0poseTj;
    dCiposeTj_dT0poseTj.setZero();
    dCiposeTj_dT0poseTj = CirT0CirT0;
    
    Matrix6x6RowType dCiposeTj_dT0poseCi;
    dCiposeTj_dT0poseCi.setZero();
    dCiposeTj_dT0poseCi.block<3,3>(0,0) = -CirT0;
    dCiposeTj_dT0poseCi.block<3,3>(3,0) =  CirT0*anantak::SkewSymmetricMatrix(T0CipTj);
    dCiposeTj_dT0poseCi.block<3,3>(3,3) = -CirT0;
    
    Eigen::Matrix3d CirTj(TjqCi.conjugate());
    Matrix3x4Type Tjpf = anantak::AprilTag3dCorners(tagTj_->size_.Value());
    
    //VLOG(1) << "Tjpf = \n" << Tjpf;
    
    Eigen::Matrix3d K = camera_->CameraMatrix();
    double fx = K(0,0);
    double fy = K(1,1);
    double cx = K(0,2);
    double cy = K(1,2);
    
    // Uncertainty in tag size
    Matrix8x8RowType tag_size_cov_mat;
    tag_size_cov_mat.setZero();
    
    for (int i_crnr=0; i_crnr<4; i_crnr++) {
      
      Eigen::Vector3d Tjpfk = Tjpf.col(i_crnr);
      Eigen::Vector3d CiTjpfk = CirTj * Tjpfk;
      Eigen::Vector3d Cipfk = CipTj + CiTjpfk;
      
      // Make sure z is positive
      if (Cipfk[2] < Epsilon) {
        LOG(WARNING) << "Tag in camera pose depth is negative. Skipping this residual. Cipfk = "
            << Cipfk.transpose();
        return false;
      }
      if (!Cipfk.allFinite()) {
        LOG(WARNING) << "Tag in camera pose is NAN. Skipping this residual. "
            << "Cipfk = " << Cipfk.transpose()
            << "\nCirTj = \n" << CirTj
            << "\nCiqT0 = " << CiqT0.coeffs().transpose()
            << "\nT0pCi = " << T0pCi.transpose();
        return false;
      }
      
      double z_recip = 1./Cipfk[2];
      double x_by_z = Cipfk[0] * z_recip;
      double y_by_z = Cipfk[1] * z_recip;
      Eigen::Vector2d uv;
      uv << fx*x_by_z + cx, fy*y_by_z + cy;
      
      //VLOG(1) << "uv calc, seen corners = \n" << uv.transpose() << " " << tag_view_->image_coords.col(i_crnr).transpose();
      
      Matrix2x4RowType duv_dK;
      duv_dK << x_by_z, 0., 1., 0.,   0., y_by_z, 0., 1.;
      
      Matrix2x2RowType fxfy_by_z;
      fxfy_by_z << fx*z_recip, 0.,   0., fy*z_recip;
      
      Matrix2x3RowType duv_dCiposnf;
      duv_dCiposnf << 1., 0., -x_by_z,   0., 1., -y_by_z;
      duv_dCiposnf = fxfy_by_z * duv_dCiposnf;
      
      Matrix3x6RowType dCiposnf_dCiposeTj;
      dCiposnf_dCiposeTj.setZero();
      dCiposnf_dCiposeTj.block<3,3>(0,0) = -anantak::SkewSymmetricMatrix(CiTjpfk);
      dCiposnf_dCiposeTj.block<3,3>(0,3) =  I3;
      
      Eigen::Vector3d dCiposnf_dsizeTj = Tjpfk / tagTj_->size_.Value();
      dCiposnf_dsizeTj = CirTj * dCiposnf_dsizeTj;
      
      dtagview_dK_.block<2,4>(2*i_crnr,0) =  duv_dK;
      
      dtagview_dposeC_.block<2,6>(2*i_crnr,0) = duv_dCiposnf * dCiposnf_dCiposeTj * dCiposeTj_dT0poseCi;
      
      dtagview_dposeT0toTj_.block<2,6>(2*i_crnr,0) = duv_dCiposnf * dCiposnf_dCiposeTj * dCiposeTj_dT0poseTj;
      
      dtagview_dTj_size_.block<2,1>(2*i_crnr,0) = duv_dCiposnf * dCiposnf_dsizeTj;
      
      // Residual is calculated as estimate - observation as jacobians are kept positive
      tagview_residual_.block<2,1>(2*i_crnr,0) = uv - tag_view_->image_coords.col(i_crnr);
      
      Eigen::Matrix3d tag_size_cov = tagTj_->size_.covariance_ * tagTj_->size_.covariance_ * I3;
      tag_size_cov = CirTj * tag_size_cov * CirTj.transpose();
      tag_size_cov_mat.block<2,2>(2*i_crnr,2*i_crnr) = duv_dCiposnf * tag_size_cov * duv_dCiposnf.transpose();
    }
    
    //VLOG(1) << "tagview_residual_ = \n" << tagview_residual_.transpose();
    
    // Noises
    // We neglect off-diagonal terms as we do not know how to calculate inv sqrt of a matrix.
    Matrix8x1RowType dtagview_cov_diag =
        options_.sigma_image * options_.sigma_image * Matrix8x1RowType::Ones()
        + tag_size_cov_mat.diagonal();
    // Convert this to inv sqrt variance to use it in scaling the residual and jacobians
    dtagview_cov_diag = dtagview_cov_diag.cwiseSqrt().cwiseInverse();
    
    // Scale the residual and Jacobians
    tagview_residual_     = dtagview_cov_diag.asDiagonal() * tagview_residual_;
    dtagview_dposeC_      = dtagview_cov_diag.asDiagonal() * dtagview_dposeC_;
    dtagview_dposeT0toTj_ = dtagview_cov_diag.asDiagonal() * dtagview_dposeT0toTj_;
    dtagview_dTj_size_    = dtagview_cov_diag.asDiagonal() * dtagview_dTj_size_;
    dtagview_dK_          = dtagview_cov_diag.asDiagonal() * dtagview_dK_;
    
    // if anything has Nans/Inf skip this residual
    if (!tagview_residual_.allFinite()) {
      LOG(ERROR) << "tagview_residual_ = " << tagview_residual_.transpose() << "\n"
          << "Image coords = \n" << tag_view_->image_coords << "\n"
          << "dtagview_cov_diag = " << dtagview_cov_diag.transpose() << "\n"
          << "tag size = " << tagTj_->size_.Value() << "\n"
          << "tag size cov = " << tagTj_->size_.covariance_;
      return false;
    }
    if (!dtagview_dposeC_.allFinite()) {
      LOG(ERROR)<<"dtagview_dposeC_ = \n"<<dtagview_dposeC_; return false;}
    if (!dtagview_dposeT0toTj_.allFinite()) {
      LOG(ERROR)<<"dtagview_dposeT0toTj_ = \n"<<dtagview_dposeT0toTj_; return false;}
    if (!dtagview_dTj_size_.allFinite()) {
      LOG(ERROR)<<"dtagview_dTj_size_ = "<<dtagview_dTj_size_.transpose(); return false;}
    if (!dtagview_dK_.allFinite()) {
      LOG(ERROR)<<"dtagview_dK_ = \n"<<dtagview_dK_; return false;}
    
    return true;
  }
  
  // Report calculations
  bool Report() {
    VLOG(1) << "tagview_residual_ = " << tagview_residual_.transpose() << "\n"
        << "Image coords = \n" << tag_view_->image_coords << "\n";
    }
  
  // Check if the residual is ready for optimization
  bool Check() {
    return true;
  }
  
  // Get ready for Optimization
  bool GetReadyToOptimize() {
    if (!Check()) {
      LOG(ERROR) << "Check failed. Can not continue.";
      return false;
    }
    // Set the errors to zero
    //  Imu error state is set to zero elsewhere
    tagTj_->SetErrorZero();
    camera_->SetErrorZero();
    return true;
  }
  
  // Evaluate function for ceres solver
  virtual bool Evaluate(
      double const* const* parameters,
      double* residuals,
      double** jacobians) const {
    
    // Assuming Check() has been performed before
    
    // Mapping residuals and parameters to matrices: AprilTagVioResidual<8, 6,6,1,4>
    MapVector8dType        tagview_resid(residuals);
    MapConstVector6dType   dposeC(parameters[0]);
    MapConstVector6dType   dposeT0toTj(parameters[1]);
    MapConstVector1dType   dTj_size(parameters[2]);
    MapConstVector4dType   dK(parameters[3]);
    
    /* Calculate residual from the error states. Linear model of the residual wrt error states
     * is used to recalculate the residual. */
    tagview_resid =
        tagview_residual_
        + dtagview_dposeC_ * dposeC
        + dtagview_dposeT0toTj_ * dposeT0toTj
        + dtagview_dTj_size_ * dTj_size
        + dtagview_dK_ * dK;
    
    /* Jacobians are not recalculated with changing states. We use first estimates Jacobians
     * as this shown to keep the estimator consistent. Also makes calculations faster.*/
    if (jacobians != NULL) {
      if (jacobians[0] != NULL) {
        MapMatrix8x6RowType jac(jacobians[0]);
        jac = dtagview_dposeC_;
      }
      if (jacobians[1] != NULL) {
        MapMatrix8x6RowType jac(jacobians[1]);
        jac = dtagview_dposeT0toTj_;
      }
      if (jacobians[2] != NULL) {
        MapMatrix8x1RowType jac(jacobians[2]);
        jac = dtagview_dTj_size_;
      }
      if (jacobians[3] != NULL) {
        MapMatrix8x4RowType jac(jacobians[3]);
        jac = dtagview_dK_;
      }
    }
    
    return true;
  }  
};  // AprilTagViewResidual



/* Tag corners on calibration target */
class CameraIntrinsicsCalibrationTarget {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef Eigen::Matrix<double,3,4> Matrix3x4Type;
  typedef Eigen::Matrix<double,2,4> Matrix2x4Type;
  
  std::map<std::string, Matrix3x4Type> tag_map_;
  std::map<std::string, double> tag_size_;
  int32_t num_tags_;
  bool is_initialized_;
  
  // Get the config file in constructor, generate the tag-map
  CameraIntrinsicsCalibrationTarget(const std::string& config_file):
    tag_map_(), tag_size_(), num_tags_(0), is_initialized_(false)
  {
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
        Matrix3x4Type corners = anantak::AprilTag3dCorners(tagmsg.tag().size());
        corners.colwise() += tag_posn;
        tag_map_[tagmsg.tag().tag_id()] = corners;  // making a copy here, is only done once
        tag_size_[tagmsg.tag().tag_id()] = tagmsg.tag().size(); // tag size is stored here
        VLOG(2) << "Target tag " << i << " id: " << tagmsg.tag().tag_id() << " has corners: \n" << corners;
      }
      // 
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
  const Matrix3x4Type* TagCornersPtr(const std::string& tag_id) const {
    if (!TagIsPresent(tag_id)) {
      LOG(ERROR) << "Tag " << tag_id << " is not on calibration target.";
      return nullptr;
    }
    return &tag_map_.at(tag_id);
  }
  
  // Make sure that presence of the tag is checked before using this accessor
  const Matrix3x4Type& TagCorners(const std::string& tag_id) const {
    return tag_map_.at(tag_id);
  }
  
  Matrix3x4Type TagCorners(const std::string& tag_id, const double& size) {
    Matrix3x4Type crnrs = tag_map_.at(tag_id);
    //crnrs += anantak::AprilTag3dCorners(size - tag_size_[tag_id]);
    return crnrs;
  }
  
  // Create a reading from the tag sightings in a single image
  //  Uses epnp algorithm to calculate the location of calibration target from tag views
  bool CalculateTargetPose(
      const anantak::AprilTagMessage& apriltag_msg,
      const anantak::CameraIntrinsicsState& camera,
      anantak::Pose3dState* target_pose_in_camera)
  const {
    if (apriltag_msg.tag_id_size() == 0) {
      LOG(ERROR) << "No tags seen in the message. Skip.";
      return false;
    }
    
    int32_t num_tags = apriltag_msg.tag_id_size();
    target_pose_in_camera->information_ = Epsilon*2;
    
    // Camera intrinsics
    const Eigen::Matrix3d& K = camera.K_;
    const Eigen::Matrix3d& K_inv = camera.K_inv_;
      
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
      const Matrix3x4Type& Tjpf = tag_map_.at(tag_id);
      
      // Tag coordinates in camera
      Matrix2x4Type image_coords;
      image_coords <<
          apriltag_msg.u_1(i_tag), apriltag_msg.u_2(i_tag), apriltag_msg.u_3(i_tag), apriltag_msg.u_4(i_tag),
          apriltag_msg.v_1(i_tag), apriltag_msg.v_2(i_tag), apriltag_msg.v_3(i_tag), apriltag_msg.v_4(i_tag);
          
      Matrix3x4Type corners_2d;
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
      
      if (std::isnan(information) || information<Epsilon ||
          !TjrCi.allFinite() || !TjpCi.allFinite()) {
        LOG(ERROR) << "information is nan or <0. information = " << information;
        LOG(ERROR) << "\nTjrCi is not finite. TjrCi = \n" << TjrCi;
        LOG(ERROR) << "\nTjpCi is not finite. TjpCi = " << TjpCi.transpose();
        for (int i=0; i<pnp_transformations.size(); i++)
          LOG(ERROR) << "\npnp_transformations ("<<i<<") = \n" << pnp_transformations[i];
        LOG(ERROR) << "\ndffs = " << dffs.transpose() << "  min_idx = " << min_idx;
        continue;
      }
      
      // Assign calculations to a pose 3d
      anantak::Pose3dState tag_pose;
      tag_pose.SetZero();
      tag_pose.LqvG_ = TjqCi.coeffs();
      tag_pose.GpL_ = CipTj;
      tag_pose.information_ = information;
      VLOG(3) << "Calculated tag pose = " << tag_pose;
      
      // Add the tag-pose information to target pose
      if (!UpdatePose3dUsingInformation(&tag_pose, target_pose_in_camera)) {
        LOG(ERROR) << "Could not add tag pose information to target pose. Skip.";
        continue;
      }
      
    } // for each tag seen
    
    VLOG(2) << "Calculated target pose = " << *target_pose_in_camera;
    
    return true;
  }
  
  virtual ~CameraIntrinsicsCalibrationTarget() {}
  
};  // CameraIntrinsicsCalibrationTarget


/* April Tag View residual - for dynamic tags
 * This constrains Camera pose, tag pose and camera matrix given a tag view
 * All maths here in in Robotics Notebook #4 ppg 91-92
 */
class DynamicAprilTagViewResidual: public ceres::SizedCostFunction<8, 6,6,1,4> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Type declarations
  typedef Eigen::Map<Eigen::Matrix<double,8,1>> MapVector8dType;
  typedef Eigen::Map<const Eigen::Matrix<double,6,1>> MapConstVector6dType;
  typedef Eigen::Map<const Eigen::Matrix<double,1,1>> MapConstVector1dType;
  typedef Eigen::Map<const Eigen::Matrix<double,4,1>> MapConstVector4dType;
  typedef Eigen::Map<Eigen::Matrix<double,8,6,Eigen::RowMajor>> MapMatrix8x6RowType;
  typedef Eigen::Map<Eigen::Matrix<double,8,4,Eigen::RowMajor>> MapMatrix8x4RowType;
  typedef Eigen::Map<Eigen::Matrix<double,8,1>> MapMatrix8x1RowType;
  typedef Eigen::Matrix<double,8,6,Eigen::RowMajor> Matrix8x6RowType;
  typedef Eigen::Matrix<double,8,1> Matrix8x1RowType;
  typedef Eigen::Matrix<double,8,4,Eigen::RowMajor> Matrix8x4RowType;
  typedef Eigen::Matrix<double,6,6,Eigen::RowMajor> Matrix6x6RowType;
  typedef Eigen::Matrix<double,8,8,Eigen::RowMajor> Matrix8x8RowType;
  typedef Eigen::Matrix<double,3,4> Matrix3x4Type;
  typedef Eigen::Matrix<double,2,4> Matrix2x4Type;
  typedef Eigen::Matrix<double,2,4,Eigen::RowMajor> Matrix2x4RowType;
  typedef Eigen::Matrix<double,2,2,Eigen::RowMajor> Matrix2x2RowType;
  typedef Eigen::Matrix<double,2,3,Eigen::RowMajor> Matrix2x3RowType;
  typedef Eigen::Matrix<double,3,6,Eigen::RowMajor> Matrix3x6RowType;
  
  // Options object used by this residual
  struct Options {
    bool use_first_estimates;   /**< Should we re-evaluate Jacobians at every estimate? */
    // Noise parameters
    double sigma_image;             /**< Stdev of u,v location of corner in the image */
    Options() {
      use_first_estimates = true;       /**< Generally we use first estimate jacobians */
      sigma_image = 1.0;                /**< 1-sigma corner location in pixels */
    }
    Options(const double& s) {
      use_first_estimates = true;
      sigma_image = s;      
    }
    Options(const anantak::AprilTagViewResidualOptionsConfig& config) {
      use_first_estimates = true;
      sigma_image = config.sigma_image();   
    }
  };
  
  // Data members
  DynamicAprilTagViewResidual::Options options_;
  
  // Observation
  Matrix2x4Type       image_coordinates_;   /** coordinates of the tag in image */
  
  // States that this residual constrains
  anantak::Pose3dState *poseC_;             /**< Camera pose tag map frame */
  anantak::Pose3dState *tagTj_pose_;        /**< Tj tag pose in World frame */
  anantak::ScalarState *tagTj_size_;        /**< Tj tag size */
  anantak::CameraIntrinsicsState *camera_;  /**< Camera intrinsics state */
  
  // Starting residual
  Eigen::Matrix<double,8,1> tagview_residual_;   /**< Starting residual */
  
  // Jacobian matrices
  Matrix8x6RowType    dtagview_dposeC_;
  Matrix8x6RowType    dtagview_dposeT0toTj_;
  Matrix8x1RowType    dtagview_dTj_size_;
  Matrix8x4RowType    dtagview_dK_;             // K is the camera matrix
  
  // Timestamp representing the residual - usually used to decide if residual should be used
  int64_t timestamp_;
  
  // Function declaration for getting tag corners
  std::function<Matrix3x4Type(double)> tag_corner_function_;
  
  // Default constructor
  DynamicAprilTagViewResidual() :
    options_(),
    poseC_(NULL), tagTj_pose_(NULL), tagTj_size_(NULL), camera_(NULL), timestamp_(0),
    tag_corner_function_(anantak::AprilTag3dCorners) {
    tagview_residual_.setZero();
    dtagview_dposeC_.setZero();
    dtagview_dposeT0toTj_.setZero();
    dtagview_dTj_size_.setZero();
    dtagview_dK_.setZero();
    image_coordinates_.setZero();
  }
  
  // Default copy constructor
  DynamicAprilTagViewResidual(const DynamicAprilTagViewResidual& r) {
    options_=r.options_;
    poseC_=r.poseC_; tagTj_pose_=r.tagTj_pose_; tagTj_size_=r.tagTj_size_; camera_ = r.camera_; 
    tagview_residual_ = r.tagview_residual_;
    dtagview_dposeC_ = r.dtagview_dposeC_;
    dtagview_dposeT0toTj_ = r.dtagview_dposeT0toTj_;
    dtagview_dTj_size_ = r.dtagview_dTj_size_;
    dtagview_dK_ = r.dtagview_dK_;
    timestamp_ = r.timestamp_;
    tag_corner_function_ = r.tag_corner_function_;
    image_coordinates_ = r.image_coordinates_;
  }
  
  // Equal to assignment operator
  DynamicAprilTagViewResidual& operator= (const DynamicAprilTagViewResidual& r) {
    options_=r.options_;
    poseC_=r.poseC_; tagTj_pose_=r.tagTj_pose_; tagTj_size_=r.tagTj_size_; camera_ = r.camera_; 
    tagview_residual_ = r.tagview_residual_;
    dtagview_dposeC_ = r.dtagview_dposeC_;
    dtagview_dposeT0toTj_ = r.dtagview_dposeT0toTj_;
    dtagview_dTj_size_ = r.dtagview_dTj_size_;
    dtagview_dK_ = r.dtagview_dK_;
    timestamp_ = r.timestamp_;
    tag_corner_function_ = r.tag_corner_function_;
    image_coordinates_ = r.image_coordinates_;
  }
  
  // Destructor
  virtual ~DynamicAprilTagViewResidual() {}
  
  // Reset residual
  bool Reset() {
    // options_ are not reset
    poseC_ = NULL; tagTj_pose_ = NULL; tagTj_size_ = NULL; camera_ = NULL;
    tagview_residual_.setZero();
    dtagview_dposeC_.setZero();
    dtagview_dposeT0toTj_.setZero();
    dtagview_dTj_size_.setZero();
    dtagview_dK_.setZero();
    timestamp_=0;
    image_coordinates_.setZero();
    return true;
  }
  
  // Create residual - this allows to reuse the memory already assigned for this residual
  bool Create(const anantak::AprilTagReadingType *tag_view, anantak::Pose3dState *poseC,
      anantak::Pose3dState *tagTj_pose, anantak::ScalarState *tagTj_size, 
      anantak::CameraIntrinsicsState *camera,
      bool is_zero_cam = false) {
    if (tag_view->IsZero()) {
      LOG(ERROR) << "Provided tag view is zero";
      return false;
    }
    if (poseC->IsZero() && !is_zero_cam) {
      LOG(WARNING) << "Provided camera pose is zero";
    }
    if (tagTj_pose->IsZero()) {
      LOG(WARNING) << "Provided tagTj pose is zero";
    }
    if (tagTj_size->IsZero()) {
      LOG(WARNING) << "Provided tagTj size is zero " << tagTj_size->Value();
    }
    if (camera->IsZero()) {
      LOG(ERROR) << "Provided camera is zero. Can not continue";
      return false;
    }
    
    // Reset the residual
    Reset();
    
    // Assign pointers to states
    image_coordinates_ = tag_view->image_coords;
    poseC_ = poseC;           // Camera pose in TagMap state
    tagTj_pose_ = tagTj_pose; // Tag pose in TagMap state
    tagTj_size_ = tagTj_size; // Tag pose in TagMap state
    camera_ = camera;         // Camera intrinsics state
    timestamp_ = tag_view->Timestamp();  // Set the timestamp from the reading
    
    // Calculate starting Residuals and Jacobians
    if (!CalculateStartingResidualandJacobians()) {
      LOG(ERROR) << "Could not calculate Jacobians. Exit.";
      return false;
    }
    return true;
  }
  
  // Create residual - along with options
  bool Create(const anantak::AprilTagReadingType *tag_view, anantak::Pose3dState *poseC,
      anantak::Pose3dState *tagTj_pose, anantak::ScalarState *tagTj_size, 
      anantak::CameraIntrinsicsState *camera,
      DynamicAprilTagViewResidual::Options *options,
      bool is_zero_cam = false) {
    if (options->sigma_image < Epsilon) {
      LOG(WARNING) << "Provided image corner sqrt variance is zero.";
    }
    options_ = *options;  // copy options as these are kept for life of the residual
    return Create(tag_view, poseC, tagTj_pose, tagTj_size, camera, is_zero_cam);
  }
  
  // Create residual for i_tag'th tag in apriltag_msg located on the calibration target
  bool Create(
      const int64_t& timestamp,
      const AprilTagMessage& apriltag_msg,
      const int32_t& i_tag,
      const CameraIntrinsicsCalibrationTarget& target,
      anantak::Pose3dState *poseC,
      anantak::Pose3dState *tagTj_pose,
      anantak::ScalarState *tagTj_size, 
      anantak::CameraIntrinsicsState *camera,
      DynamicAprilTagViewResidual::Options *options,
      bool is_zero_cam = false)
  {
    if (options->sigma_image < Epsilon) {
      LOG(WARNING) << "Provided image corner sqrt variance is zero.";
    }
    options_ = *options;  // copy options as these are kept for life of the residual
    
    if (i_tag > apriltag_msg.tag_id_size()-1) {
      LOG(ERROR) << "i_tag > apriltag_msg.tag_id_size()-1 ! "
          << i_tag << " " << apriltag_msg.tag_id_size();
      return false;
    }
    
    const std::string tag_id = apriltag_msg.tag_id(i_tag);
    if (!target.TagIsPresent(tag_id)) {
      LOG(ERROR) << "Tag " << tag_id << " was not found on the target";
      return false;
    }
    
    if (timestamp == 0) {
      LOG(WARNING) << "Timestamp is zero";
    }
    if (poseC->IsZero() && !is_zero_cam) {
      LOG(WARNING) << "Provided camera pose is zero";
    }
    if (tagTj_pose->IsZero()) {
      LOG(WARNING) << "Provided tagTj pose is zero";
    }
    if (tagTj_size->IsZero()) {
      LOG(WARNING) << "Provided tagTj size is zero " << tagTj_size->Value();
    }
    if (camera->IsZero()) {
      LOG(ERROR) << "Provided camera is zero. Can not continue";
      return false;
    }
    
    // Reset the residual
    Reset();
    
    // Assign values and pointers
    image_coordinates_ <<
        apriltag_msg.u_1(i_tag), apriltag_msg.u_2(i_tag), apriltag_msg.u_3(i_tag), apriltag_msg.u_4(i_tag),
        apriltag_msg.v_1(i_tag), apriltag_msg.v_2(i_tag), apriltag_msg.v_3(i_tag), apriltag_msg.v_4(i_tag);
    poseC_ = poseC;           // Camera pose in TagMap state
    tagTj_pose_ = tagTj_pose; // Tag pose in TagMap state
    tagTj_size_ = tagTj_size; // Tag pose in TagMap state
    camera_ = camera;         // Camera intrinsics state
    timestamp_ = timestamp;   // Set the timestamp
    
    // Calculate starting Residuals and Jacobians
    if (!CalculateStartingResidualandJacobians()) {
      LOG(ERROR) << "Could not calculate Jacobians. Exit.";
      return false;
    }
    
    return true;
  }
  
  // Set the timestamp
  bool SetTimestamp(const int64_t& ts) {
    timestamp_ = ts;
    return true;
  }
  
  // Calculate starting estimates of Residual and Jacobians.
  bool CalculateStartingResidualandJacobians() {
    
    Eigen::Matrix3d I3(Eigen::Matrix3d::Identity());
    
    Eigen::Quaterniond CiqT0(poseC_->Quaternion());
    Eigen::Quaterniond TjqT0(tagTj_pose_->Quaternion());
    
    Eigen::Vector3d T0pCi(poseC_->Position());
    Eigen::Vector3d T0pTj(tagTj_pose_->Position());
    
    Eigen::Matrix3d CirT0(CiqT0);
    
    Eigen::Quaterniond TjqCi = TjqT0 * CiqT0.conjugate();
    Eigen::Vector3d T0CipTj = T0pTj - T0pCi;
    Eigen::Vector3d CipTj = CirT0 * T0CipTj;
    
    Matrix6x6RowType CirT0CirT0;
    CirT0CirT0.setZero();
    CirT0CirT0.block<3,3>(0,0) =  CirT0;
    CirT0CirT0.block<3,3>(3,3) =  CirT0;
    
    Matrix6x6RowType dCiposeTj_dT0poseTj;
    dCiposeTj_dT0poseTj.setZero();
    dCiposeTj_dT0poseTj = CirT0CirT0;
    
    Matrix6x6RowType dCiposeTj_dT0poseCi;
    dCiposeTj_dT0poseCi.setZero();
    dCiposeTj_dT0poseCi.block<3,3>(0,0) = -CirT0;
    dCiposeTj_dT0poseCi.block<3,3>(3,0) =  CirT0*anantak::SkewSymmetricMatrix(T0CipTj);
    dCiposeTj_dT0poseCi.block<3,3>(3,3) = -CirT0;
    
    Eigen::Matrix3d CirTj(TjqCi.conjugate());
    //Matrix3x4Type Tjpf = anantak::AprilTag3dCorners(tagTj_size_->Value());
    Matrix3x4Type Tjpf = tag_corner_function_(tagTj_size_->Value());
    
    //VLOG(1) << "Tjpf = \n" << Tjpf;
    
    Eigen::Matrix3d K = camera_->CameraMatrix();
    double fx = K(0,0);
    double fy = K(1,1);
    double cx = K(0,2);
    double cy = K(1,2);
    
    // Uncertainty in tag size
    Matrix8x8RowType tag_size_cov_mat;
    tag_size_cov_mat.setZero();
    
    for (int i_crnr=0; i_crnr<4; i_crnr++) {
      
      Eigen::Vector3d Tjpfk = Tjpf.col(i_crnr);
      Eigen::Vector3d CiTjpfk = CirTj * Tjpfk;
      Eigen::Vector3d Cipfk = CipTj + CiTjpfk;
      
      // Make sure z is positive
      if (Cipfk[2] < Epsilon) {
        LOG(WARNING) << "Tag in camera pose depth is negative. Skipping this residual. Cipfk = "
            << Cipfk.transpose();
        return false;
      }
      if (!Cipfk.allFinite()) {
        LOG(WARNING) << "Tag in camera pose is NAN. Skipping this residual. "
            << "Cipfk = " << Cipfk.transpose()
            << "\nCirTj = \n" << CirTj
            << "\nCiqT0 = " << CiqT0.coeffs().transpose()
            << "\nT0pCi = " << T0pCi.transpose();
        return false;
      }
      
      double z_recip = 1./Cipfk[2];
      double x_by_z = Cipfk[0] * z_recip;
      double y_by_z = Cipfk[1] * z_recip;
      Eigen::Vector2d uv;
      uv << fx*x_by_z + cx, fy*y_by_z + cy;
      
      //VLOG(1) << "uv calc, seen corners = \n" << uv.transpose() << " " << image_coordinates_.col(i_crnr).transpose();
      
      Matrix2x4RowType duv_dK;
      duv_dK << x_by_z, 0., 1., 0.,   0., y_by_z, 0., 1.;
      
      Matrix2x2RowType fxfy_by_z;
      fxfy_by_z << fx*z_recip, 0.,   0., fy*z_recip;
      
      Matrix2x3RowType duv_dCiposnf;
      duv_dCiposnf << 1., 0., -x_by_z,   0., 1., -y_by_z;
      duv_dCiposnf = fxfy_by_z * duv_dCiposnf;
      
      Matrix3x6RowType dCiposnf_dCiposeTj;
      dCiposnf_dCiposeTj.setZero();
      dCiposnf_dCiposeTj.block<3,3>(0,0) = -anantak::SkewSymmetricMatrix(CiTjpfk);
      dCiposnf_dCiposeTj.block<3,3>(0,3) =  I3;
      
      Eigen::Vector3d dCiposnf_dsizeTj = Tjpfk / tagTj_size_->Value();
      dCiposnf_dsizeTj = CirTj * dCiposnf_dsizeTj;
      
      dtagview_dK_.block<2,4>(2*i_crnr,0) =  duv_dK;
      
      dtagview_dposeC_.block<2,6>(2*i_crnr,0) = duv_dCiposnf * dCiposnf_dCiposeTj * dCiposeTj_dT0poseCi;
      
      dtagview_dposeT0toTj_.block<2,6>(2*i_crnr,0) = duv_dCiposnf * dCiposnf_dCiposeTj * dCiposeTj_dT0poseTj;
      
      dtagview_dTj_size_.block<2,1>(2*i_crnr,0) = duv_dCiposnf * dCiposnf_dsizeTj;
      
      // Residual is calculated as estimate - observation as jacobians are kept positive
      tagview_residual_.block<2,1>(2*i_crnr,0) = uv - image_coordinates_.col(i_crnr);
      
      Eigen::Matrix3d tag_size_cov = tagTj_size_->covariance_ * tagTj_size_->covariance_ * I3;
      tag_size_cov = CirTj * tag_size_cov * CirTj.transpose();
      tag_size_cov_mat.block<2,2>(2*i_crnr,2*i_crnr) = duv_dCiposnf * tag_size_cov * duv_dCiposnf.transpose();
    }
    
    //VLOG(1) << "tagview_residual_ = \n" << tagview_residual_.transpose();
    
    // Noises
    // We neglect off-diagonal terms as we do not know how to calculate inv sqrt of a matrix.
    Matrix8x1RowType dtagview_cov_diag =
        options_.sigma_image * options_.sigma_image * Matrix8x1RowType::Ones()
        + tag_size_cov_mat.diagonal();
    // Convert this to inv sqrt variance to use it in scaling the residual and jacobians
    dtagview_cov_diag = dtagview_cov_diag.cwiseSqrt().cwiseInverse();
    
    // Scale the residual and Jacobians
    tagview_residual_     = dtagview_cov_diag.asDiagonal() * tagview_residual_;
    dtagview_dposeC_      = dtagview_cov_diag.asDiagonal() * dtagview_dposeC_;
    dtagview_dposeT0toTj_ = dtagview_cov_diag.asDiagonal() * dtagview_dposeT0toTj_;
    dtagview_dTj_size_    = dtagview_cov_diag.asDiagonal() * dtagview_dTj_size_;
    dtagview_dK_          = dtagview_cov_diag.asDiagonal() * dtagview_dK_;
    
    // if anything has Nans/Inf skip this residual
    if (!tagview_residual_.allFinite()) {
      LOG(ERROR) << "tagview_residual_ = " << tagview_residual_.transpose() << "\n"
          << "Image coords = \n" << image_coordinates_ << "\n"
          << "dtagview_cov_diag = " << dtagview_cov_diag.transpose() << "\n"
          << "tag size = " << tagTj_size_->Value() << "\n"
          << "tag size cov = " << tagTj_size_->covariance_;
      return false;
    }
    if (!dtagview_dposeC_.allFinite()) {
      LOG(ERROR)<<"dtagview_dposeC_ = \n"<<dtagview_dposeC_; return false;}
    if (!dtagview_dposeT0toTj_.allFinite()) {
      LOG(ERROR)<<"dtagview_dposeT0toTj_ = \n"<<dtagview_dposeT0toTj_; return false;}
    if (!dtagview_dTj_size_.allFinite()) {
      LOG(ERROR)<<"dtagview_dTj_size_ = "<<dtagview_dTj_size_.transpose(); return false;}
    if (!dtagview_dK_.allFinite()) {
      LOG(ERROR)<<"dtagview_dK_ = \n"<<dtagview_dK_; return false;}
    
    return true;
  }
  
  // Report calculations
  bool Report() {
    VLOG(1) << "tagview_residual_ = " << tagview_residual_.transpose() << "\n"
        << "Image coords = \n" << image_coordinates_ << "\n";
    }
  
  // Check if the residual is ready for optimization
  bool Check() {
    return true;
  }
  
  // Get ready for Optimization
  bool GetReadyToOptimize() {
    if (!Check()) {
      LOG(ERROR) << "Check failed. Can not continue.";
      return false;
    }
    // Set the errors to zero
    //  Imu error state is set to zero elsewhere
    tagTj_pose_->SetErrorZero();
    tagTj_size_->SetErrorZero();
    camera_->SetErrorZero();
    return true;
  }
  
  // Evaluate function for ceres solver
  virtual bool Evaluate(
      double const* const* parameters,
      double* residuals,
      double** jacobians) const {
    
    // Assuming Check() has been performed before
    
    // Mapping residuals and parameters to matrices: AprilTagVioResidual<8, 6,6,1,4>
    MapVector8dType        tagview_resid(residuals);
    MapConstVector6dType   dposeC(parameters[0]);
    MapConstVector6dType   dposeT0toTj(parameters[1]);
    MapConstVector1dType   dTj_size(parameters[2]);
    MapConstVector4dType   dK(parameters[3]);
    
    /* Calculate residual from the error states. Linear model of the residual wrt error states
     * is used to recalculate the residual. */
    tagview_resid =
        tagview_residual_
        + dtagview_dposeC_ * dposeC
        + dtagview_dposeT0toTj_ * dposeT0toTj
        + dtagview_dTj_size_ * dTj_size
        + dtagview_dK_ * dK;
    
    /* Jacobians are not recalculated with changing states. We use first estimates Jacobians
     * as this shown to keep the estimator consistent. Also makes calculations faster.*/
    if (jacobians != NULL) {
      if (jacobians[0] != NULL) {
        MapMatrix8x6RowType jac(jacobians[0]);
        jac = dtagview_dposeC_;
      }
      if (jacobians[1] != NULL) {
        MapMatrix8x6RowType jac(jacobians[1]);
        jac = dtagview_dposeT0toTj_;
      }
      if (jacobians[2] != NULL) {
        MapMatrix8x1RowType jac(jacobians[2]);
        jac = dtagview_dTj_size_;
      }
      if (jacobians[3] != NULL) {
        MapMatrix8x4RowType jac(jacobians[3]);
        jac = dtagview_dK_;
      }
    }
    
    return true;
  }  
};  // DynamicAprilTagViewResidual

/** TargetPointViewResidual
 * A target lies in front of the machine. A point on the target is seen by a camera on the machine.
 *  Camera state has camera's pinhole model, distortion parameters and extrinsics
 *  Target's pose has its pose wrt machine
 *  Point's pose has its pose wrt target
 * In the most general case, we solve for 
class TargetPointViewResidual : public ceres::SizedCostFunction<2, > {
  
};  // PointViewResidual


// Residual for calculating IMU position wrt camera
class FixedCameraPoseResidualFunction : public ceres::SizedCostFunction<3,3> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef Eigen::Map<const Eigen::Vector3d> MapVector3dConstType;
  typedef Eigen::Map<Eigen::Matrix<double,3,1>> MapVector3dType;
  typedef Eigen::Map<Eigen::Matrix<double,3,3,Eigen::RowMajor>> MapMatrix3x3Type;
  
  FixedCameraPoseResidualFunction(
    const Eigen::Vector3d* T0pC0, const Eigen::Quaterniond* T0qC0,
    const Eigen::Vector3d* T0pCi, const Eigen::Quaterniond* T0qCi,
    const Eigen::Vector3d* CpI,   const Eigen::Quaterniond* CqI,
    const double* info
  ): info_(std::sqrt(*info)), I0pIi_(Eigen::Vector3d::Zero()) {
    Eigen::Matrix3d T0rC0(*T0qC0);
    Eigen::Matrix3d T0rCi(*T0qCi);
    Eigen::Matrix3d CrI(*CqI);
    I0pIi_ = CrI.transpose()*T0rC0.transpose()*((*T0pCi) - (*T0pC0) + (T0rCi-T0rC0)*(*CpI));
  }
  
  FixedCameraPoseResidualFunction(
    const Eigen::Vector3d* I0pIi, const double& sigma_position
  ): info_(1./(sigma_position)), I0pIi_(*I0pIi), dr_dI0pIi_(Eigen::Matrix3d::Identity()/(sigma_position)) {}
  
  virtual ~FixedCameraPoseResidualFunction() {}
  
  virtual bool Evaluate(
      double const* const* parameters,
      double* residuals,
      double** jacobians) const {
    
    MapVector3dConstType I0pIi(parameters[0]);
    MapVector3dType resid(residuals);
    
    resid = I0pIi - I0pIi_;
    resid *= info_;
    
    if (jacobians != NULL) {
      if (jacobians[0] != NULL) {
        MapMatrix3x3Type dr_dI0pIi(jacobians[0]);
        dr_dI0pIi = dr_dI0pIi_;
      }
    }
    
    return true;
  } // Evaluate
  
 protected:
  double info_;
  Eigen::Vector3d I0pIi_;
  Eigen::Matrix3d dr_dI0pIi_;
}; // FixedCameraPoseResidualFunction 


/* IMU pose - Gravity - Imu_to_Cam pose joint prior
 * IMU pose, gravity and ItoC pose estimates are highly corelated. This implements a prior
 * that reflects the codependence of these variables. At the end of each iteration, this prior
 * is estimated and saved in queue. For each iteration, we choose the correct prior to apply.
 * IMU state is 15x1, Gravity is 3x1 and Rigid pose is 6x1. That makes this prior 24x1!
 */
class ImuStateGravityRigidPosePrior: public ceres::SizedCostFunction<24, 15,3,6> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef Eigen::Matrix<double,24,24> Matrix24x24;
  typedef Eigen::Matrix<double,15,15> Matrix15x15;
  typedef Eigen::Matrix<double,3,3>   Matrix3x3;
  typedef Eigen::Matrix<double,6,6>   Matrix6x6;
  typedef Eigen::Matrix<double,15,3>  Matrix15x3;
  typedef Eigen::Matrix<double,15,6>  Matrix15x6;
  typedef Eigen::Matrix<double,3,6>   Matrix3x6;
  typedef Eigen::Matrix<double,24,1>  Matrix24x1;
  typedef Eigen::Map<Eigen::Matrix<double,24,1>>        MapVector24dType;
  typedef Eigen::Map<const Eigen::Matrix<double,15,1>>  MapConstVector15dType;
  typedef Eigen::Map<const Eigen::Matrix<double, 3,1>>  MapConstVector3dType;
  typedef Eigen::Map<const Eigen::Matrix<double, 6,1>>  MapConstVector6dType;
  typedef Eigen::Map<Eigen::Matrix<double,24,15,Eigen::RowMajor>> MapMatrix24x15RowType;
  typedef Eigen::Map<Eigen::Matrix<double,24, 3,Eigen::RowMajor>> MapMatrix24x3RowType;
  typedef Eigen::Map<Eigen::Matrix<double,24, 6,Eigen::RowMajor>> MapMatrix24x6RowType;
  
  ImuState* poseI_;             // Imu state pointer
  Vector3dState* gravity_;      // Gravity state pointer
  Pose3dState* ItoC_;           // Imu to cam state pointer
  
  Matrix24x24 covariance_;
  Matrix24x24 inv_sqrt_cov_;
  
  ImuStateGravityRigidPosePrior():
    poseI_(NULL), gravity_(NULL), ItoC_(NULL),
    covariance_(Matrix24x24::Zero()), inv_sqrt_cov_(Matrix24x24::Zero()) {}
  
  ImuStateGravityRigidPosePrior(const ImuStateGravityRigidPosePrior& r):
    poseI_(r.poseI_), gravity_(r.gravity_), ItoC_(r.ItoC_),
    covariance_(r.covariance_), inv_sqrt_cov_(r.inv_sqrt_cov_) {}
  
  ImuStateGravityRigidPosePrior& operator= (const ImuStateGravityRigidPosePrior& r) {
    poseI_=r.poseI_; gravity_=r.gravity_; ItoC_=r.ItoC_;
    covariance_=r.covariance_; inv_sqrt_cov_=r.inv_sqrt_cov_;
  }
  
  bool Reset() {
    poseI_=NULL; gravity_=NULL; ItoC_=NULL;
    covariance_=Matrix24x24::Zero(); inv_sqrt_cov_=Matrix24x24::Zero();
    return true;
  }
  
  bool Create(ceres::Problem* problem, ImuState* poseI, Vector3dState* gravity,
      Pose3dState* ItoC, bool show_details=false) {
    
    // Calculate the covariance matrix from the problem
    if (!problem) {LOG(ERROR) << "Problem is NULL"; return false;}
    if (!poseI) {LOG(ERROR) << "poseI is NULL"; return false;}
    if (!gravity) {LOG(ERROR) << "gravity is NULL"; return false;}
    if (!ItoC) {LOG(ERROR) << "ItoC is NULL"; return false;}
    
    Reset();
    
    poseI_ = poseI;
    gravity_ = gravity;
    ItoC_ = ItoC;
    
    // Calculate covariance
    if (!CalculateCovariance(problem)) {
      LOG(ERROR) << "Could not calculate covariance. Quit.";
      return false;
    }
    
    // Calculate inv sqrt cov 
    if (!CalculateInvSqrtCov(show_details)) {
      LOG(ERROR) << "Could not calculate inv sqrt cov. Quit.";
      return false;
    }
    
    return true;
  }
  
  bool CalculateCovariance(ceres::Problem* problem) {
    
    // Calculate covariances
    ceres::Covariance::Options covariance_options;
    ceres::Covariance covariance(covariance_options);
    std::vector<std::pair<const double*, const double*>> covariance_blocks;
    
    double* stat = poseI_->error_;
    double* grav = gravity_->error_;
    double* itoc = ItoC_->error_;
    
    Matrix15x15 stat_cov;  Matrix15x3  stat_grav_cov;  Matrix15x6  stat_itoc_cov;
    Matrix3x3   grav_cov;  Matrix3x6   grav_itoc_cov;
    Matrix6x6   itoc_cov;
    
    covariance_blocks.push_back(std::make_pair(stat, stat));
    covariance_blocks.push_back(std::make_pair(stat, grav));
    covariance_blocks.push_back(std::make_pair(stat, itoc));
    covariance_blocks.push_back(std::make_pair(grav, grav));
    covariance_blocks.push_back(std::make_pair(grav, itoc));
    covariance_blocks.push_back(std::make_pair(itoc, itoc));
    
    if (!covariance.Compute(covariance_blocks, problem)) {
      LOG(ERROR) << "Got an error in computing covariance matrix. Quit from here.";
      return false;
    }
    
    covariance.GetCovarianceBlock(stat, stat, stat_cov.data());
    covariance.GetCovarianceBlock(stat, grav, stat_grav_cov.data());
    covariance.GetCovarianceBlock(stat, itoc, stat_itoc_cov.data());
    covariance.GetCovarianceBlock(grav, grav, grav_cov.data());
    covariance.GetCovarianceBlock(grav, itoc, grav_itoc_cov.data());
    covariance.GetCovarianceBlock(itoc, itoc, itoc_cov.data());
    
    covariance_.block<15,15>( 0, 0) = stat_cov;
    covariance_.block<15, 3>( 0,15) = stat_grav_cov;
    covariance_.block<15, 6>( 0,18) = stat_itoc_cov;
    covariance_.block< 3,15>(15, 0) = stat_grav_cov.transpose();
    covariance_.block< 3, 3>(15,15) = grav_cov;
    covariance_.block< 3, 6>(15,18) = grav_itoc_cov;
    covariance_.block< 6,15>(18, 0) = stat_itoc_cov.transpose();
    covariance_.block< 6, 3>(18,15) = grav_itoc_cov.transpose();
    covariance_.block< 6, 6>(18,18) = itoc_cov;
    
    return true;
  }
  
  bool CalculateInvSqrtCov(bool show_details=false) {
    Matrix24x1  stdev;
    Matrix24x24 correlation;
    Matrix24x24 check;
    
    if (show_details) {
      if (!CalculateInverseSqrtMatrix(covariance_, &inv_sqrt_cov_, &stdev, &correlation, &check)) {
        LOG(ERROR) << "Could not calculate inverse sqrt of covariance matrix";
        return false;
      }
      // Report
      Eigen::IOFormat CleanFmt(2, 0, ", ", "\n", "", "");
      LOG(INFO) << "Inv sqrt cov = \n" << inv_sqrt_cov_.format(CleanFmt)
          << "\n Sqrt var = " << stdev.transpose().format(CleanFmt)
          << "\n Correl = \n" << correlation.format(CleanFmt)
          << "\n check = \n" << check.format(CleanFmt);
    } else {
      if (!CalculateInverseSqrtMatrix(covariance_, &inv_sqrt_cov_, &stdev)) {
        LOG(ERROR) << "Could not calculate inverse sqrt of covariance matrix";
        return false;
      }
    }
    
    return true;
  }
  
  virtual ~ImuStateGravityRigidPosePrior() {}
  
  // Evaluate function for ceres solver
  virtual bool Evaluate(
      double const* const* parameters,
      double* residuals,
      double** jacobians) const {
    
    // Mapping residuals and parameters to matrices
    MapConstVector15dType   dposeI(parameters[0]);
    MapConstVector3dType    dgrav(parameters[1]);
    MapConstVector6dType    dItoC(parameters[2]);
    MapVector24dType        residual(residuals);
    
    /* Calculate residual from the error states. Linear model of the residual wrt error states
     * is used to recalculate the residual. */
    residual = inv_sqrt_cov_.block<24,15>(0, 0) * dposeI
             + inv_sqrt_cov_.block<24, 3>(0,15) * dgrav
             + inv_sqrt_cov_.block<24, 6>(0,18) * dItoC;
    
    /* Jacobians are not recalculated with changing states. We use first estimates Jacobians
     * as this shown to keep the estimator consistent. */
    if (jacobians != NULL) {
      if (jacobians[0] != NULL) {
        MapMatrix24x15RowType jac(jacobians[0]);
        jac = inv_sqrt_cov_.block<24,15>(0, 0);
      }
      if (jacobians[1] != NULL) {
        MapMatrix24x3RowType jac(jacobians[1]);
        jac = inv_sqrt_cov_.block<24, 3>(0,15);
      }
      if (jacobians[2] != NULL) {
        MapMatrix24x6RowType jac(jacobians[2]);
        jac = inv_sqrt_cov_.block<24, 6>(0,18);
      }
    }
    
    return true;
  }  // Evaluate
  
};  // ImuStateGravityRigidPosePrior



/* IMU motion along an origin plane residual
 * Implements the constrant that motion of imu is along a plane that passes through the origin.
 * Constraint is simply that the velocity is perpendicular to unit normal of the plane.
 * Input errors are: IMU residual and Unit Normal residual
 * Output error is: scalar dot product between IMU velocity and unit normal of plane.
 * Maths here is in Robotics notebook 4 pg 95
 */
class ImuMotionAlongOriginPlaneResidual: public ceres::SizedCostFunction<1, 15,2> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Type declarations
  typedef Eigen::Map<Eigen::Matrix<double,1,1>> MapVector1dType;
  typedef Eigen::Map<const Eigen::Matrix<double,2,1>> MapConstVector2dType;
  typedef Eigen::Map<const Eigen::Matrix<double,3,1>> MapConstVector3dType;
  typedef Eigen::Map<const Eigen::Matrix<double,15,1>> MapConstVector15dType;
  typedef Eigen::Matrix<double,1,1> Vector1d;
  typedef Eigen::Matrix<double,1,2,Eigen::RowMajor> Matrix1x2RowType;
  typedef Eigen::Matrix<double,1,3,Eigen::RowMajor> Matrix1x3RowType;
  typedef Eigen::Matrix<double,1,15,Eigen::RowMajor> Matrix1x15RowType;
  typedef Eigen::Map<Eigen::Matrix<double,1,2,Eigen::RowMajor>> MapMatrix1x2RowType;
  typedef Eigen::Map<Eigen::Matrix<double,1,3,Eigen::RowMajor>> MapMatrix1x3RowType;
  typedef Eigen::Map<Eigen::Matrix<double,1,15,Eigen::RowMajor>> MapMatrix1x15RowType;
  
  struct Options {
    bool use_first_estimates;   /**< Should we reevaluate Jacobians at every estimate? */
    // Noise parameters
    double sigma_velocity;      /**< in m/s stdev of residual velocity in direction of normal */
    Options() {
      use_first_estimates = true;       /**< Generally we use first estimate jacobians */
      sigma_velocity = 0.5;             /**< 1 sigma value */
    }
  };
  
  // Options
  ImuMotionAlongOriginPlaneResidual::Options options_;
  
  // Data holders
  anantak::ImuState *poseI_;                /**< Pose of Imu sensor in W frame */
  anantak::UnitVector3dState *plane_;       /**< Unit vector of the normal to the plane */
  
  // Starting Residuals
  Vector1d veloP_;                   /**< Starting residual velocity perpendicular to the plane */
  
  // Jacobian matrices
  Matrix1x15RowType dveloP_dposeI_;
  Matrix1x2RowType  dveloP_dplane_;
  
  // Default constructor
  ImuMotionAlongOriginPlaneResidual():
    options_(),
    poseI_(NULL), plane_(NULL) {
    dveloP_dposeI_.setZero(); dveloP_dplane_.setZero();
    veloP_.setZero();
  }
  
  // Default copy constructor
  ImuMotionAlongOriginPlaneResidual(const ImuMotionAlongOriginPlaneResidual& r) {
    options_=r.options_;
    poseI_=r.poseI_; plane_=r.plane_;
    veloP_=r.veloP_;
    dveloP_dposeI_=r.dveloP_dposeI_; 
    dveloP_dplane_=r.dveloP_dplane_;
  }
  
  // Destructor
  virtual ~ImuMotionAlongOriginPlaneResidual() {}
  
  // Reset residual
  bool Reset() {
    poseI_=NULL; plane_=NULL;
    veloP_.setZero();
    dveloP_dposeI_.setZero(); dveloP_dplane_.setZero();
    return true;
  }
  
  // Create residual
  bool Create(anantak::ImuState *poseI, anantak::UnitVector3dState *plane,
      ImuMotionAlongOriginPlaneResidual::Options *options) {
    if (poseI->IsZero()) {
      LOG(WARNING) << "Provided imu pose is zero";
    }
    if (plane->IsZero()) {
      LOG(ERROR) << "Provided unit vector is zero. Need this for Jacobians.";
      return false;
    }
    // Should we check if any of the other poses are zero too?
    // Should we return false if so?
    
    // Reset the residual
    Reset();
    
    // Assign new values
    poseI_ = poseI;
    plane_ = plane;
    options_ = *options;  // copy options
    if (!CalculateStartingResidualandJacobians()) {
      LOG(ERROR) << "Could not calculate Jacobians. Exit.";
      return false;
    }
    return true;
  }
  
  // Calculate Jacobians
  bool CalculateStartingResidualandJacobians() {
    
    // Starting residual
    Eigen::Vector3d WvI = poseI_->GvI_; // copy of imu velocity in IMU's world frame
    Eigen::Vector3d Wn = plane_->Gn_;   // copy the unit vector normal to plane
    veloP_ = WvI.transpose() * Wn;
    
    // Starting Jacobians
    dveloP_dposeI_ << 0., 0., 0.,  0., 0., 0.,  Wn[0], Wn[1], Wn[2],  0., 0., 0.,  0., 0., 0.;
    //dveloP_dplane_ = WvI.transpose() * anantak::SkewSymmetricMatrix(Wn);
    dveloP_dplane_ = WvI.transpose() * plane_->Gn_dGn_;
    
    // Noises
    double inv_sqrt_var = 1./options_.sigma_velocity;
    
    // Scale residual and jacobians with noises.
    veloP_ = inv_sqrt_var * veloP_;
    dveloP_dposeI_ = inv_sqrt_var * dveloP_dposeI_;
    dveloP_dplane_ = inv_sqrt_var * dveloP_dplane_;
    
    return true;
  }
  
  // Check if the residual is ready for optimization
  bool Check() {
    return true;
  }
  
  // Get ready for Optimization
  bool GetReadyToOptimize() {
    if (!Check()) {
      LOG(ERROR) << "Check failed. Can not continue.";
      return false;
    }
    // Set the errors to zero
    // Imu error state is set to zero elsewhere
    plane_->SetErrorZero();
    
    return true;
  }
  
  // Evaluate function for ceres solver
  virtual bool Evaluate(
      double const* const* parameters,
      double* residuals,
      double** jacobians) const {
    
    // Assuming Check() has been performed before
    
    // Mapping residuals and parameters to matrices
    MapConstVector15dType  dposeI(parameters[0]);
    MapConstVector2dType   dplane(parameters[1]);
    MapVector1dType        veloP(residuals);
    
    /* Calculate residual from the error states. Linear model of the residual wrt error states
     * is used to recalculate the residual. */
    veloP = veloP_ + dveloP_dposeI_*dposeI + dveloP_dplane_*dplane;
    
    /* Jacobians are not recalculated with changing states. We use first estimates Jacobians
     * as this shown to keep the estimator consistent. Also make calculations fast.*/
    if (jacobians != NULL) {
      if (jacobians[0] != NULL) {
        MapMatrix1x15RowType jac(jacobians[0]);
        jac = dveloP_dposeI_;
      }
      if (jacobians[1] != NULL) {
        MapMatrix1x2RowType jac(jacobians[1]);
        jac = dveloP_dplane_;
      }
    }
    
    return true;
  }
}; // ImuMotionAlongOriginPlaneResidual

/* IMU motion along plane residual
 * Implements the constrant that motion of imu is along a plane.
 * Constraints are that
 *  position lies on the plane, and
 *  velocity is perpendicular to unit normal of the plane.
 * Input errors are: IMU residual and Plane 3d residual
 * Maths here is in Robotics notebook 4 pg 95
 */
class ImuMotionAlongPlaneResidual: public ceres::SizedCostFunction<2, 15,2,1> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Type declarations
  typedef Eigen::Map<Eigen::Matrix<double,1,1>> MapVector1dType;
  typedef Eigen::Map<Eigen::Matrix<double,2,1>> MapVector2dType;
  typedef Eigen::Map<const Eigen::Matrix<double,1,1>> MapConstVector1dType;
  typedef Eigen::Map<const Eigen::Matrix<double,2,1>> MapConstVector2dType;
  typedef Eigen::Map<const Eigen::Matrix<double,3,1>> MapConstVector3dType;
  typedef Eigen::Map<const Eigen::Matrix<double,15,1>> MapConstVector15dType;
  typedef Eigen::Matrix<double,1,1> Vector1dType;
  typedef Eigen::Matrix<double,2,1> Vector2dType;
  typedef Eigen::Matrix<double,1,2,Eigen::RowMajor> Matrix1x2RowType;
  typedef Eigen::Matrix<double,2,2,Eigen::RowMajor> Matrix2x2RowType;
  typedef Eigen::Matrix<double,2,1> Matrix2x1RowType;
  typedef Eigen::Matrix<double,2,3,Eigen::RowMajor> Matrix2x3RowType;
  typedef Eigen::Matrix<double,2,15,Eigen::RowMajor> Matrix2x15RowType;
  typedef Eigen::Map<Eigen::Matrix<double,1,2,Eigen::RowMajor>> MapMatrix1x2RowType;
  typedef Eigen::Map<Eigen::Matrix<double,1,3,Eigen::RowMajor>> MapMatrix1x3RowType;
  typedef Eigen::Map<Eigen::Matrix<double,1,15,Eigen::RowMajor>> MapMatrix1x15RowType;
  typedef Eigen::Map<Eigen::Matrix<double,2,15,Eigen::RowMajor>> MapMatrix2x15RowType;
  typedef Eigen::Map<Eigen::Matrix<double,2,2,Eigen::RowMajor>> MapMatrix2x2RowType;
  typedef Eigen::Map<Eigen::Matrix<double,2,1>> MapMatrix2x1RowType;
  
  struct Options {
    bool use_first_estimates;   /**< Should we reevaluate Jacobians at every estimate? */
    // Noise parameters
    double sigma_distance;
    double sigma_velocity;      /**< in m/s stdev of residual velocity in direction of normal */
    Options() {
      use_first_estimates = true;       /**< Generally we use first estimate jacobians */
      sigma_distance = 0.10;            /**< 1 sigma value of misfit distance on the plane */
      sigma_velocity = 0.5;             /**< 1 sigma value of misfit velocity on the plane */
    }
  };
  
  // Options
  ImuMotionAlongPlaneResidual::Options options_;
  
  // Data holders
  anantak::ImuState *poseI_;                /**< Pose of Imu sensor in W frame */
  anantak::Plane3dState *plane_;            /**< Plane 3d state in W frame */
  
  // Starting Residuals
  Vector2dType distveloP_;       /**< Starting residual distance and velocity perpendicular to the plane */
  
  // Jacobian matrices
  Matrix2x15RowType ddistveloP_dposeI_;
  Matrix2x2RowType  ddistveloP_dplane_;   // wrt normal of the place
  Matrix2x1RowType  ddistveloP_dpdist_;   // wrt plane distance
  
  // Default constructor
  ImuMotionAlongPlaneResidual():
    options_(),
    poseI_(NULL), plane_(NULL) {
    ddistveloP_dposeI_.setZero(); ddistveloP_dplane_.setZero(); ddistveloP_dpdist_.setZero();
    distveloP_.setZero();
  }
  
  // Default copy constructor
  ImuMotionAlongPlaneResidual(const ImuMotionAlongPlaneResidual& r) {
    options_=r.options_;
    poseI_=r.poseI_; plane_=r.plane_;
    distveloP_=r.distveloP_;
    ddistveloP_dposeI_=r.ddistveloP_dposeI_; 
    ddistveloP_dplane_=r.ddistveloP_dplane_;
    ddistveloP_dpdist_=r.ddistveloP_dpdist_;
  }
  
  // Destructor
  virtual ~ImuMotionAlongPlaneResidual() {}
  
  // Reset residual
  bool Reset() {
    poseI_=NULL; plane_=NULL;
    distveloP_.setZero();
    ddistveloP_dplane_.setZero(); ddistveloP_dplane_.setZero(); ddistveloP_dpdist_.setZero();
    return true;
  }
  
  // Create residual
  bool Create(anantak::ImuState *poseI, anantak::Plane3dState *plane,
      ImuMotionAlongPlaneResidual::Options *options) {
    if (poseI->IsZero()) {
      LOG(WARNING) << "Provided imu pose is zero";
    }
    if (plane->IsZero()) {
      LOG(ERROR) << "Provided plane is zero. Need this for Jacobians.";
      return false;
    }
    // Should we check if any of the other poses are zero too?
    // Should we return false if so?
    
    // Reset the residual
    Reset();
    
    // Assign new values
    poseI_ = poseI;
    plane_ = plane;
    options_ = *options;  // copy options
    if (!CalculateStartingResidualandJacobians()) {
      LOG(ERROR) << "Could not calculate Jacobians. Exit.";
      return false;
    }
    return true;
  }
  
  // Calculate Jacobians
  bool CalculateStartingResidualandJacobians() {
    
    // Get values from states
    Eigen::Vector3d WpI = poseI_->GpI_; // copy of imu position in IMU's world frame
    Eigen::Vector3d WvI = poseI_->GvI_; // copy of imu velocity in IMU's world frame
    Eigen::Vector3d Wn = plane_->normal_.Gn_;   // copy the unit vector normal to plane in IMU's W frame
    Vector1dType Ws; Ws << plane_->distance_.Value();
    
    // Starting residual
    distveloP_.block<1,1>(0,0) = WpI.transpose() * Wn  -  Ws;
    distveloP_.block<1,1>(1,0) = WvI.transpose() * Wn;
    
    // Starting Jacobians
    ddistveloP_dposeI_.setZero();
    ddistveloP_dposeI_.block<1,3>(0,3) = Wn.transpose();
    ddistveloP_dposeI_.block<1,3>(1,6) = Wn.transpose();
    
    ddistveloP_dplane_.setZero();
    ddistveloP_dplane_.block<1,2>(0,0) = WpI.transpose()*plane_->normal_.Gn_dGn_;
    ddistveloP_dplane_.block<1,2>(1,0) = WvI.transpose()*plane_->normal_.Gn_dGn_;
    
    ddistveloP_dpdist_(0,0) = -1.;
    ddistveloP_dpdist_(1,0) =  0.;
    
    
    // Noises
    Vector2dType inv_sqrt_var; 
    inv_sqrt_var << 1./options_.sigma_distance, 1./options_.sigma_velocity;
    
    // Scale residual and jacobians with noises.
    distveloP_         = inv_sqrt_var.asDiagonal() * distveloP_;
    ddistveloP_dposeI_ = inv_sqrt_var.asDiagonal() * ddistveloP_dposeI_;
    ddistveloP_dplane_ = inv_sqrt_var.asDiagonal() * ddistveloP_dplane_;
    ddistveloP_dpdist_ = inv_sqrt_var.asDiagonal() * ddistveloP_dpdist_;
    
    return true;
  }
  
  // Check if the residual is ready for optimization
  bool Check() {
    return true;
  }
  
  // Get ready for Optimization
  bool GetReadyToOptimize() {
    if (!Check()) {
      LOG(ERROR) << "Check failed. Can not continue.";
      return false;
    }
    // Set the errors to zero
    // Imu error state is set to zero elsewhere
    plane_->SetErrorZero();
    
    return true;
  }
  
  // Evaluate function for ceres solver
  virtual bool Evaluate(
      double const* const* parameters,
      double* residuals,
      double** jacobians) const {
    
    // Assuming Check() has been performed before
    
    // Mapping residuals and parameters to matrices
    MapConstVector15dType  dposeI(parameters[0]);
    MapConstVector2dType   dplane(parameters[1]);
    MapConstVector1dType   dpdist(parameters[2]);
    MapVector2dType        distveloP(residuals);
    
    /* Calculate residual from the error states. Linear model of the residual wrt error states
     * is used to recalculate the residual. */
    distveloP = distveloP_
                + ddistveloP_dposeI_*dposeI
                + ddistveloP_dplane_*dplane
                + ddistveloP_dpdist_*dpdist;
    
    /* Jacobians are not recalculated with changing states. We use first estimates Jacobians
     * as this shown to keep the estimator consistent. Also make calculations fast.*/
    if (jacobians != NULL) {
      if (jacobians[0] != NULL) {
        MapMatrix2x15RowType jac(jacobians[0]);
        jac = ddistveloP_dposeI_;
      }
      if (jacobians[1] != NULL) {
        MapMatrix2x2RowType jac(jacobians[1]);
        jac = ddistveloP_dplane_;
      }
      if (jacobians[2] != NULL) {
        MapMatrix2x1RowType jac(jacobians[2]);
        jac = ddistveloP_dpdist_;
      }
    }
    
    return true;
  }
}; // ImuMotionAlongPlaneResidual


/* Unit vector prior residual
 * Implements a prior for a unit vector. Error is the angle between them.
 * Input error is: error of the unit vector
 * Output error is: angle beween vectors.
 * Maths here is in Robotics notebook 4 pg 96
 */
class UnitVectorPriorResidual: public ceres::SizedCostFunction<3, 3> {
 public:
  
};  // UnitVectorPriorResidual

/* Unit vector change residual
 * Implements rotational change constraint between two unit normals (origin planes)
 * Input errors are: Two unit normals
 * Output error is: rotation error between unit normals
 * Maths here is in Robotics notebook 4 pg 96
 */
class UnitVectorChangeResidual: public ceres::SizedCostFunction<3, 3,3> {
  
}; // UnitVectorChangeResidual


/* Perpendicular unit vector residual
 * Implements the constraint that the unit normal is perpendicular to a given vector.
 * Constraint implements a cross product between two normals.
 * Input errors are: Unit normal and the vector3d
 * Output error is: Cross product between the two
 * Maths here is in Robotics notebook 4 pg 97
 */
class PerpendicularUnitVectorResidual: public ceres::SizedCostFunction<3, 3,3> {
  
};  // PerpendicularUnitVectorResidual


/** Closed form integral of rotations and accelerations assuming constant angular velocity */
template<typename Vec3dType, typename Mat3dType>
bool IntegrateConstantAngularVelocity(
    const double& dt,                             /**< Time interval for integration */
    const double& angle,                          /**< Angle travelled along axis during dt */
    const Eigen::MatrixBase<Vec3dType>& axis,     /**< Angular velocity axis */
    const Eigen::MatrixBase<Vec3dType>& accel,    /**< Linear acceleration measurement */      
    const Eigen::MatrixBase<Vec3dType>& d_accel,  /**< Linear acceleration change */      
    Eigen::MatrixBase<Mat3dType>* V_full,         /**< Integral(rotation, dt)  */
    Eigen::MatrixBase<Vec3dType>* s_full,         /**< Integral(rotation*acceleration, dt) */
    const double small_omega_threshold = SmallAngularVelocityThreshold
    ) {
  // Check if inputs make sense
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Vec3dType, 3)
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Mat3dType, 3, 3)
  if (dt < Eigen::NumTraits<double>::epsilon()) {LOG(ERROR) << "dt <= 0!" << dt; return false; }
  
  // Should we use small omega approximation?
  double omega = angle/dt;
  
  Eigen::Matrix3d axis_skew = SkewSymmetricMatrix(axis);
  Eigen::Matrix3d axis_skew2 = axis_skew*axis_skew;
  Eigen::Matrix3d I3_dt = Eigen::Matrix3d::Identity() * dt;
  double dt2_by_2 = 0.5*dt*dt;
  
  d_accel = d_accel/dt;  // modify change in acceleration to rate of change of acceleration
  Eigen::Vector3d axis_skew_dr_am = axis_skew * d_accel;
  Eigen::Vector3d axis_skew2_dr_am = axis_skew2 * d_accel;
  
  if (std::abs(omega) > small_omega_threshold) {
    
    double omega_recip = dt/angle;
    double omega_recip2 = omega_recip*omega_recip;
    
    double cos_angle = std::cos(angle);
    double sin_angle = std::sin(angle);
    double one_m_cos_angle = 1.0 - cos_angle;
    
    *V_full = I3_dt;
    *V_full += one_m_cos_angle * omega_recip * axis_skew;
    *V_full +=  (angle - sin_angle) * omega_recip * axis_skew2;
    
    *s_full = *V_full * accel;
    *s_full += dt2_by_2 * d_accel;
    *s_full += (sin_angle - angle*cos_angle) * omega_recip2 * axis_skew_dr_am;
    *s_full += (dt2_by_2 + omega_recip2*(one_m_cos_angle - angle*sin_angle)) * axis_skew2_dr_am;
    
  } else {
    // Use small omega approximations
    
    double t3_by_6 = dt2_by_2*dt/3.0;
    double t4_by_8 = dt2_by_2*dt2_by_2*0.5;
    
    *V_full = I3_dt;
    *V_full += dt2_by_2 * omega * axis_skew;
    *V_full += t3_by_6 * axis_skew2;
    
    *s_full = *V_full * accel;
    *s_full += dt2_by_2 * d_accel;
    *s_full += 2.0*t3_by_6 * omega * axis_skew_dr_am;
    *s_full += t4_by_8 * omega * omega * axis_skew2_dr_am;
    
  }
  
  return true;
}

/** Integral of rotations and accelerations assuming constant angular velocity */
template<typename Vec3dType, typename Mat3dType>
bool IntegrateConstantAngularVelocity(
    const double& dt,                             /**< Time interval for integration */
    const double& angle,                          /**< Angle travelled along axis during dt */
    const Eigen::MatrixBase<Vec3dType>& axis,     /**< Angular velocity axis */
    const Eigen::MatrixBase<Vec3dType>& accel,    /**< Linear acceleration measurement */      
    const Eigen::MatrixBase<Vec3dType>& d_accel,  /**< Linear acceleration change */      
    Eigen::MatrixBase<Mat3dType>* V_half,         /**< Integral(rotation, dt/2) */
    Eigen::MatrixBase<Mat3dType>* V_full,         /**< Integral(rotation, dt)  */
    Eigen::MatrixBase<Vec3dType>* s_half,         /**< Integral(rotation*acceleration, dt/2)  */
    Eigen::MatrixBase<Vec3dType>* s_full,         /**< Integral(rotation*acceleration, dt) */
    const double small_omega_threshold = SmallAngularVelocityThreshold
    ) {
  // Check if inputs make sense
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Vec3dType, 3)
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Mat3dType, 3, 3)
  if (dt < Eigen::NumTraits<double>::epsilon()) {LOG(ERROR) << "dt <= 0!" << dt; return false; }
  
  // Should we use small omega approximation?
  double omega = angle/dt;
  
  Eigen::Matrix3d axis_skew = SkewSymmetricMatrix(axis);
  Eigen::Matrix3d axis_skew2 = axis_skew*axis_skew;
  Eigen::Matrix3d I3_dt = Eigen::Matrix3d::Identity() * dt;
  double dt2_by_2 = 0.5*dt*dt;
  
  d_accel = d_accel/dt;  // modify change in acceleration to rate of change of acceleration
  Eigen::Vector3d axis_skew_dr_am = axis_skew * d_accel;
  Eigen::Vector3d axis_skew2_dr_am = axis_skew2 * d_accel;
  
  double dt_half = 0.5*dt;
  double dt2_by_2_half = 0.25*dt2_by_2;
  
  if (std::abs(omega) > small_omega_threshold) {
    
    double omega_recip = dt/angle;
    double omega_recip2 = omega_recip*omega_recip;
    
    double cos_angle = std::cos(angle);
    double sin_angle = std::sin(angle);
    double one_m_cos_angle = 1.0 - cos_angle;
    
    *V_full = I3_dt;
    *V_full += one_m_cos_angle * omega_recip * axis_skew;
    *V_full +=  (angle - sin_angle) * omega_recip * axis_skew2;
    
    *s_full = *V_full * accel;
    *s_full += dt2_by_2 * d_accel;
    *s_full += (sin_angle - angle*cos_angle) * omega_recip2 * axis_skew_dr_am;
    *s_full += (dt2_by_2 + omega_recip2*(one_m_cos_angle - angle*sin_angle)) * axis_skew2_dr_am;
    
    // Midpoint is needed for integration with Simpson's rule
    double angle_half = 0.5*angle;
    double cos_angle_half = std::cos(angle_half);
    double sin_angle_half = std::sin(angle_half);
    double one_m_cos_angle_half = 1.0 - cos_angle_half;
    
    *V_half = 0.5 * I3_dt;
    *V_half += one_m_cos_angle_half * omega_recip * axis_skew;
    *V_half +=  (angle_half - sin_angle_half) * omega_recip * axis_skew2;
    
    *s_half = *V_half * accel;
    *s_half += dt2_by_2_half * d_accel;
    *s_half += (sin_angle_half - angle_half*cos_angle_half) * omega_recip2 * axis_skew_dr_am;
    *s_half += (dt2_by_2_half + omega_recip2*(one_m_cos_angle_half - angle_half*sin_angle_half)) *
                axis_skew2_dr_am;
    
  } else {
    // Use small omega approximations
    
    double t3_by_6 = dt2_by_2*dt/3.0;
    double t4_by_8 = dt2_by_2*dt2_by_2*0.5;
    double t3_by_6_half = dt2_by_2_half*dt_half/3.0;
    double t4_by_8_half = dt2_by_2_half*dt2_by_2_half*0.5;
    
    *V_full = I3_dt;
    *V_full += dt2_by_2 * omega * axis_skew;
    *V_full += t3_by_6 * axis_skew2;
    
    *s_full = *V_full * accel;
    *s_full += dt2_by_2 * d_accel;
    *s_full += 2.0*t3_by_6 * omega * axis_skew_dr_am;
    *s_full += t4_by_8 * omega * omega * axis_skew2_dr_am;
    
    *V_half = 0.5 * I3_dt;
    *V_half += dt2_by_2_half * omega * axis_skew;
    *V_half += t3_by_6_half * axis_skew2;
    
    *s_half = *V_half * accel;
    *s_half += dt2_by_2_half * d_accel;
    *s_half += 2.0*t3_by_6_half * omega * axis_skew_dr_am;
    *s_half += t4_by_8_half * omega * omega * axis_skew2_dr_am;
  }
  
  return true;
}

/** Sliding Window Filter utilities */

// Iteration record
//  Keeps a record of queue indexes, times and timings.
/*class IterationRecord {
 public:
  
  // Data holders
  int64_t iter_begin_ts_, iter_end_ts_;
  int64_t data_begin_ts_, data_end_ts_;
  
  // Timings
  int64_t states_creation_time_, residuals_creation_time_, solving_time_, prior_calculation_time_;
  
  // Circular queue segments created in the iteration
  //  Priors queues - priors are created every so often at end of iterations.
  //                  Past priors help initiate the iterations.
  //  States queues - these are kept over the sliding window
  //  Residuals queues - these are reused in overlapping iterations
  std::vector<anantak::FixedPointCQ> queue_data_segments_;
  
  // Default constructor - creates an empty IterationRecord
  //  This should usually not be needed. Number of queues used by an algorithm should be known at compile time.
  IterationRecord():
    iter_begin_ts_(0), iter_end_ts_(0), data_begin_ts_(0), data_end_ts_(0),
    states_creation_time_(0), residuals_creation_time_(0), solving_time_(0), prior_calculation_time_(0),
    queue_data_segments_()
  {}
  
  // Constructor taking the default size of number of queues.
  //  This is to be used generally as the number of queues used in an algorithm should be known at compile time.
  //  At least the starting number of queues should be known.
  IterationRecord(int32_t num_queues):
    iter_begin_ts_(0), iter_end_ts_(0), data_begin_ts_(0), data_end_ts_(0),
    states_creation_time_(0), residuals_creation_time_(0), solving_time_(0), prior_calculation_time_(0),
    queue_data_segments_(num_queues, anantak::FixedPointCQ())
  {}
  
  // Default construction from another
  IterationRecord(const IterationRecord& r):
    iter_begin_ts_(r.iter_begin_ts_), iter_end_ts_(r.iter_end_ts_),
    data_begin_ts_(r.data_begin_ts_), data_end_ts_(r.data_end_ts_),
    states_creation_time_(r.states_creation_time_), residuals_creation_time_(r.residuals_creation_time_),
    solving_time_(r.solving_time_), prior_calculation_time_(r.prior_calculation_time_),
    queue_data_segments_(r.queue_data_segments_)
  {}
  
  // Equal assign operation
  IterationRecord& operator= (const IterationRecord& r) {
    iter_begin_ts_=r.iter_begin_ts_; iter_end_ts_=r.iter_end_ts_;
    data_begin_ts_=r.data_begin_ts_; data_end_ts_=r.data_end_ts_;
    states_creation_time_=r.states_creation_time_; residuals_creation_time_=r.residuals_creation_time_;
    solving_time_=r.solving_time_; prior_calculation_time_=r.prior_calculation_time_;
    queue_data_segments_=r.queue_data_segments_;
  }
  
  
};  // IterationRecord */

/** Message utilities **/

/** Check if the message timestamps are in increasing order */
bool IsIncreasing(const std::vector<anantak::SensorMsg>& msg_vec) {
  bool is_increasing = true;
  if (msg_vec.size()==0) {return true;}
  int64_t last_timestamp = msg_vec[0].header().timestamp();
  int i=1;
  while (i<msg_vec.size() && is_increasing) {
    if (last_timestamp >= msg_vec[i].header().timestamp()) {
      is_increasing = false;
      LOG(WARNING) << "msg_vec is not increasing at index " << i;
    }
    last_timestamp = msg_vec[i].header().timestamp();
    i++;
  }
  return is_increasing;
}

/** Extract a vector of timestamps from a vector of messages */
bool ExtractTimestamps(const std::vector<anantak::SensorMsg>& msgs, TimestampVecType* ts) {
  int32_t num_msgs = msgs.size();
  (*ts).resize(num_msgs);
  for (int i=0; i<msgs.size(); i++) {
    (*ts)(i) = int64_t(msgs[i].header().timestamp());
  }
  return true;
}

bool ExtractTimestamps(const std::vector<anantak::SensorMsg>& msgs, std::vector<int64_t>* ts) {
  int32_t num_msgs = msgs.size();
  (*ts).clear();
  (*ts).resize(num_msgs);
  for (int i=0; i<msgs.size(); i++) {
    (*ts)[i] = int64_t(msgs[i].header().timestamp());
  }
  return true;
}

/** Extract ImuReadingType's from imu messages */
bool ExtractImuReadingsFromImuMessages(const SensorMsgVectorType& msgs,
    std::vector<ImuReadingType>* imu_readings) {
  imu_readings->clear();
  for (int i=0; i<msgs.size(); i++) {
    // make sure that the msg has imu_msg
    if (msgs[i].has_imu_msg()) {
      int64_t ts = int64_t(msgs[i].header().timestamp());
      QuaternionType q(double(msgs[i].imu_msg().quaternion(3)), double(msgs[i].imu_msg().quaternion(0)),
          double(msgs[i].imu_msg().quaternion(1)), double(msgs[i].imu_msg().quaternion(2)));
      q.normalize();
      Eigen::Vector3d v; v << double(msgs[i].imu_msg().linear(0)), double(msgs[i].imu_msg().linear(1)),
          double(msgs[i].imu_msg().linear(2));
      ImuReadingType ir; ir.timestamp = ts; ir.quaternion = q; ir.acceleration = v;
      imu_readings->push_back(ir);
    } else {
      LOG(WARNING) << "MessagesVector does not have an imu_msg at index " << i;
      int64_t ts = 0;
      QuaternionType q; q.normalize();
      Eigen::Vector3d v = Eigen::Vector3d::Zero();
      ImuReadingType ir; ir.timestamp = ts; ir.quaternion = q; ir.acceleration = v;
      imu_readings->push_back(ir);
    }
  }
  return true;
}

/** Statistics utils **/

template<typename T>
T Mean(const std::vector<T>& vec) {
  T _sum = T(0);
  for (int i=0; i<vec.size(); i++) _sum += vec[i];
  return _sum/T(vec.size());
}

// templated class to solve for robust average
template<typename Y> 
class RobustMeanError {
  public:
  Y val;
  RobustMeanError ( Y _val ) { val = _val; }
  
  template <typename T>
  bool operator()(
    const T* const avg,    // 1-vector of robust average
    T* residuals) const    // 1-vector of residual wrt the avg
  {
    residuals[0] = avg[0] - T(val);
    return true;
  }
};

template<typename T> 
T RobustMean(const std::vector<T>& _vec, const int method=1, bool ignore_nans=true ) {
  //check
  if (_vec.size()==0) {return T(NAN);}
  std::vector<T>  vec;
  if (ignore_nans) {
    for (size_t  i=0; i<_vec.size(); ++i) {
      if (!std::isnan(_vec[i])) { vec.push_back ( _vec[i] ); }
    }
  } else {
    vec = _vec;
  }

  // initialize with a simple average
  T init = anantak::Mean(vec);
  // ceres problem
  ceres::Problem problem;
  // build problem
  for (int i=0; i<vec.size(); ++i) {
    ceres::CostFunction* cost_function =
      new ceres::AutoDiffCostFunction < RobustMeanError<T>, 1, 1 > (
        new RobustMeanError<T> ( vec[i] )
      );
    ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);
    problem.AddResidualBlock (
      cost_function,
      loss_function,
      &init
    );
  }
  // solve problem
  ceres::Solver::Options options;
  //options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = false; //true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  //std::cout << summary.FullReport() << endl;
  
  //return
  return init;
}

// templated class to solve for robust average
template<typename Y> 
class RobustMeanAngleError {
  public:
  Y val;
  RobustMeanAngleError ( Y _val ) { 
    val = _val; 
    const Y  pi2 = Y( 2.0*3.14159265358979323846);
    const Y _pi2 = Y(-2.0*3.14159265358979323846);
    while ((val < _pi2) || (val > pi2)) {
      if (val < _pi2) val+=pi2;
      if (val >  pi2) val-=pi2;
    }
  }
  
  template <typename T>
  bool operator()(
    const T* const avg,    // 1-vector of robust average
    T* residuals) const    // 1-vector of residual wrt the avg
  {
    const T  pi  = T( 3.14159265358979323846);
    const T _pi  = T(-3.14159265358979323846);
    const T  pi2 = T( 2.0*3.14159265358979323846);
    T  diff = avg[0] - T(val);
    if ( diff < _pi ) { diff += pi2; }
    if ( diff >  pi ) { diff -= pi2; }
    residuals[0] = diff;
    return true;
  }
};

template<typename T> 
T RobustMeanAngle(const std::vector<T>& _vec, const int method=1, bool ignore_nans=true) {
  //check
  if (_vec.size()==0) {return T(NAN);}
  std::vector<T>  vec;
  if (ignore_nans) {
    for (size_t  i=0; i<_vec.size(); ++i) {
      if (!std::isnan(_vec[i])) { vec.push_back ( _vec[i] ); }
    }
  } else {
    vec = _vec;
  }

  // initialize with a simple average
  T init = anantak::Mean(vec);
  // ceres problem
  ceres::Problem problem;
  // build problem
  for (int i=0; i<vec.size(); ++i) {
    ceres::CostFunction* cost_function =
      new ceres::AutoDiffCostFunction < RobustMeanAngleError<T>, 1, 1 > (
        new RobustMeanAngleError<T> ( vec[i] )
      );
    ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);
    problem.AddResidualBlock (
      cost_function,
      loss_function,
      &init
    );
  }
  // solve problem
  ceres::Solver::Options options;
  //options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = false; //true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  //std::cout << summary.FullReport() << endl;
  
  //return
  return init;
}

/* File Messages Keeper
 * Loads messages data
 * Set Time based on clock time or data will be returned using time interval
 * Returns new messages in an interval or clock time passed
 */
class FileMessagesKeeper {
 public:
  
  FileMessagesKeeper(const std::vector<std::string>& msgs_filenames, const bool run_in_reatime_mode) {
    msgs_filenames_ = msgs_filenames;
    run_in_realtime_mode_ = run_in_reatime_mode;
  }
  
  virtual ~FileMessagesKeeper() {}
  
  bool LoadAllMsgsFromFiles() {
    // Load msgs into sensor_msgs_
    num_files_ = msgs_filenames_.size();
    VLOG(1) << "Number of message files = " << num_files_;
    sensor_msgs_.resize(num_files_);  // all elements are nullptr's
    for (int i=0; i<num_files_; i++) {
      std::unique_ptr<std::vector<anantak::SensorMsg>> ptr(new std::vector<anantak::SensorMsg>);
      sensor_msgs_[i] = std::move(ptr);
      if (!anantak::LoadMsgsFromFile(msgs_filenames_[i], sensor_msgs_[i].get())) {
        LOG(ERROR) << "Could not load messages from " << msgs_filenames_[i];
      }
    }
    // Set file_time_curr_time_offset_
    int64_t min_files_ts = 0;
    for (int i=0; i<num_files_; i++) {
      int64_t file_min_ts = 0;
      const anantak::SensorMsg& msg = sensor_msgs_[i]->front();
      if (msg.has_header()) {
        file_min_ts = msg.header().timestamp();
      }
      VLOG(1) << "  Starting file " << i << " timestamp = "
          << anantak::microsec_to_time_str(file_min_ts);
      if (min_files_ts==0 && file_min_ts!=0) {
        min_files_ts = file_min_ts;
      } else {
        min_files_ts = std::min(file_min_ts, min_files_ts);
      }
    }
    if (min_files_ts==0) {
      LOG(ERROR) << "Could not calculate minimum files timestamp";
      return false;
    } else {
      VLOG(1) << "Starting files timestamp = " << anantak::microsec_to_time_str(min_files_ts);
    }
    curr_time_ = get_wall_time_microsec();
    last_fetch_time_ = 0;
    data_starting_ts_ = min_files_ts;
    file_time_curr_time_offset_ = min_files_ts - curr_time_;
    VLOG(1) << "file_time_curr_time_offset_ = "
        << anantak::microsec_to_time_str(-file_time_curr_time_offset_);
    // Set msgs_indexes_ to beginning
    msgs_indexes_.resize(num_files_, int32_t(0));
    return true;
  }
  
  // Any more data left?
  bool MoreDataLeft() {
    bool data_left = false;
    for (int i_file=0; i_file<num_files_; i_file++)
        data_left |= (msgs_indexes_[i_file] < sensor_msgs_[i_file]->size());
    return data_left;
  }
  
  // Utility to allocate memory for data fetch. Assumes all pointers in array are NULL
  bool AllocateMemoryForNewMessages(const int32_t& num_msgs_per_file,
      std::vector<std::unique_ptr<std::vector<anantak::SensorMsg>>>* new_msgs
  ) {
    new_msgs->resize(num_files_);
    for (int i=0; i<num_files_; i++) {
      std::unique_ptr<std::vector<anantak::SensorMsg>> ptr(new std::vector<anantak::SensorMsg>);
      (*new_msgs)[i] = std::move(ptr);
      (*new_msgs)[i]->reserve(num_msgs_per_file);
    }
  }
  
  // Fetch messages between given historical timestamps
  bool FetchMessagesBetweenTimestamps(const int64_t& ts0, const int64_t& ts1,
      std::vector<std::unique_ptr<std::vector<anantak::SensorMsg>>>* new_msgs) {
    // Read forward from msgs_indexes_ from each file, fetching messages in the time interval
    // new_msgs should have correct size. If not, return false
    if (new_msgs->size()!=num_files_) {
      LOG(ERROR) << "new_msgs->size()!=num_files_ " << new_msgs->size() << " " << num_files_;
      return false;
    }
    // Clear messages in copy buffer
    for (int i_file=0; i_file<num_files_; i_file++) {
      new_msgs->at(i_file)->clear();
    }
    // For each file move forward from current message, check timestamp. Copy message.
    for (int i_file=0; i_file<num_files_; i_file++) {
      bool exceeded_ts1 = false;
      while (!exceeded_ts1 && msgs_indexes_[i_file] < sensor_msgs_[i_file]->size()) {
        const anantak::SensorMsg& msg = sensor_msgs_[i_file]->at(msgs_indexes_[i_file]);
        exceeded_ts1 = (msg.header().timestamp()>ts1);
        if (msg.header().timestamp()>ts0 && msg.header().timestamp()<=ts1) {
          (*new_msgs)[i_file]->push_back(msg);  // copy message
        }
        if (!exceeded_ts1) msgs_indexes_[i_file]++;
      } // while !done
    } // for each file
    
    return true;
  }

  // Fetch last messages
  bool FetchLastMessages(std::vector<std::unique_ptr<std::vector<anantak::SensorMsg>>>* new_msgs) {
    for (int i_file=0; i_file<num_files_; i_file++) {
      new_msgs->at(i_file)->clear();
      const anantak::SensorMsg& msg = sensor_msgs_[i_file]->back();
      (*new_msgs)[i_file]->push_back(msg);  // copy message
    }
    return true;
  }
  
  // Fetch new messages since last time data was fetched
  //  realtime mode - messages are returned since min(last_fetch_time_, curr_time_-interval)
  //  batch mode - message are returned in curr_time_+interval. curr_time_ is updated.
  bool FetchNewMessages(const int64_t& interval,
      std::vector<std::unique_ptr<std::vector<anantak::SensorMsg>>>* new_msgs) {
    //int32_t num_msgs = 0;
    int64_t ts0, ts1;
    if (run_in_realtime_mode_) {
      curr_time_ = get_wall_time_microsec();
      int64_t fetch_interval = std::min(interval, curr_time_ - last_fetch_time_);
      last_fetch_time_ = curr_time_;
      ts1 = curr_time_;
      ts0 = curr_time_ - fetch_interval;
    } else {
      curr_time_ = curr_time_ + interval;
      ts1 = curr_time_;
      ts0 = last_fetch_time_;
      last_fetch_time_ = curr_time_;
    }
    // Convert current timestamps to historical file timestamps
    ts0 += file_time_curr_time_offset_;
    ts1 += file_time_curr_time_offset_;
    return FetchMessagesBetweenTimestamps(ts0, ts1, new_msgs);
  }
  
  inline int64_t get_wall_time_microsec() {
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) (tv.tv_sec * 1000000 + tv.tv_usec);  
  }
  
  int64_t CurrentTime() {return curr_time_;}
  int64_t CurrentDataTime() {return curr_time_+file_time_curr_time_offset_;}
  int64_t DataStartTime() {return data_starting_ts_;}
  
  // Data variables
  std::vector<std::string> msgs_filenames_;
  std::vector<std::unique_ptr<std::vector<anantak::SensorMsg>>> sensor_msgs_;
  int32_t num_files_;
  bool run_in_realtime_mode_;
  int64_t file_time_curr_time_offset_; //
  int64_t data_starting_ts_;  // timestamp of starting of data
  int64_t curr_time_; // Current time
  int64_t last_fetch_time_; // last timestamp when data was fetched
  std::vector<int32_t> msgs_indexes_; // current indexes of each messages vector
};  // FileMessagesKeeper


/* Sliding Window Iterations
 * Implements functions to help run the sliding window filter
 */
class SlidingWindowFilterIterations {
 public:
  
  struct Options {
    uint64_t longest_problem_interval;    // Problem will be built to this longest length
    uint64_t shortest_problem_interval;   // Problem will be built to this length at a minimum
    uint64_t sliding_window_interval;     // This is the length of time in which states will be solved
    uint64_t solving_problem_interval;    // Problem will be solved after every this interval
    
    Options() :
      longest_problem_interval(10000000),
      shortest_problem_interval(3000000),
      sliding_window_interval(2000000),
      solving_problem_interval(1000000)
    {Check();}
    
    Options(uint64_t lpi, uint64_t spi, uint64_t swi, uint64_t vpi) :
      longest_problem_interval(lpi),
      shortest_problem_interval(spi),
      sliding_window_interval(swi),
      solving_problem_interval(vpi)
    {Check();}
    
    Options(const SlidingWindowOptionsConfig& config):
      longest_problem_interval(config.longest_problem_interval()),
      shortest_problem_interval(config.shortest_problem_interval()),
      sliding_window_interval(config.sliding_window_interval()),
      solving_problem_interval(config.solving_problem_interval())
    {Check();}    
    
    bool Check() {
      if (longest_problem_interval < shortest_problem_interval)
        LOG(ERROR) << "longest_problem_interval < shortest_problem_interval! "
          << longest_problem_interval << " " << shortest_problem_interval;
      if (shortest_problem_interval < sliding_window_interval)
        LOG(ERROR) << "shortest_problem_interval < sliding_window_interval! "
          << shortest_problem_interval << " " <<  sliding_window_interval;
    }  // Check
    
    std::string ToString() {
      return "Longest: "+std::to_string(longest_problem_interval)+" Shortest: "+std::to_string(shortest_problem_interval)+
          " Sliding: "+std::to_string(sliding_window_interval)+" Solving: "+std::to_string(solving_problem_interval);
    }
  }; // Options
  
  SlidingWindowFilterIterations::Options options_;
  
  int64_t start_ts_;            // Algorithm begins at this ts
  int64_t data_begin_ts_;       // Data in the problem begins at this ts
  int64_t solve_begin_ts_;      // Solving of the 
  int64_t data_end_ts_;
  int64_t sliding_window_ts_;   // ts where we begin solving data
  
  bool reset_problem_;
  bool solve_problem_;
  
  SlidingWindowFilterIterations():
    options_(), start_ts_(0),
    data_begin_ts_(0), solve_begin_ts_(0), data_end_ts_(0), sliding_window_ts_(0),
    reset_problem_(false), solve_problem_(false)
    {Check();}
  
  SlidingWindowFilterIterations(const SlidingWindowFilterIterations::Options& op):
    options_(op), start_ts_(0),
    data_begin_ts_(0), solve_begin_ts_(0), data_end_ts_(0), sliding_window_ts_(0),
    reset_problem_(false), solve_problem_(false)
    {Check();}
  
  bool Check() {
    if (options_.solving_problem_interval >= options_.sliding_window_interval) {
      LOG(WARNING) << "solving_problem_interval >= sliding_window_interval. "
          << options_.solving_problem_interval << " " << options_.sliding_window_interval;
      LOG(WARNING) << "Usually we expect solving_problem_interval < sliding_window_interval";
      return false;
    }
    return true;
  }
  
  // Starting of the filter timestamp
  bool StartFiltering(const int64_t& start_ts) {
    start_ts_ = start_ts;
    data_begin_ts_ = start_ts;
    solve_begin_ts_ = start_ts;
    data_end_ts_ = start_ts;
    sliding_window_ts_ = start_ts;
    reset_problem_ = true;
  }
  
  // Regular updates to the data, with ending data timestamp provided
  bool AddData(const int64_t& data_ts) {
    
    // Check if new data end ts makes sense
    if (data_ts < data_end_ts_) {
      LOG(ERROR) << "Recieved data end ts < last end ts. Not expected. "
          << data_ts << " " << data_end_ts_;
      return false;
    }
    data_end_ts_ = data_ts;
    
    // Sliding window to solve begins before data end ts
    sliding_window_ts_ = data_end_ts_ - options_.sliding_window_interval;
    if (sliding_window_ts_ < start_ts_) sliding_window_ts_ = start_ts_;
    
    // Is it time to reset the problem?
    reset_problem_ = (data_end_ts_ >= data_begin_ts_ + options_.longest_problem_interval);
    
    if (reset_problem_) {
      // Move forward
      data_begin_ts_ = data_end_ts_ - options_.shortest_problem_interval;
      if (data_begin_ts_ < start_ts_) data_begin_ts_ = start_ts_;
      // Check solve begin ts
      if (solve_begin_ts_ < data_begin_ts_) {
        LOG(WARNING) << "Data solve ts fell before data begin. Has the problem not been solved for a while?"
            << " solve_begin_ts_ was " << data_begin_ts_ - solve_begin_ts_ << " musecs before data_begin_ts_";
        solve_begin_ts_ = data_begin_ts_;
      }
    } else {
      // Data begin ts and solve begin ts both remain at the same place
    }
    
    // Is it time to solve the problem?
    solve_problem_ = (data_end_ts_ >= solve_begin_ts_ + options_.solving_problem_interval);
    
    return true;
  }
  
  const int64_t& DataBeginTimestamp() const {return data_begin_ts_;}
  const int64_t& SlidingWindowTimestamp() const {return sliding_window_ts_;}
  bool IsItTimeToResetProblem() const {return reset_problem_;}
  bool IsItTimeToSolveProblem() const {return solve_problem_;}
  
  // Solve problem update to data. Gets the ts of the last data point when problem is solved
  bool SolveProblem() {
    // Problem was solved, so update solve ts
    solve_begin_ts_ = data_end_ts_;
    return true;
  }
  
};  // SlidingWindowFilterIterations

/* IterationRecord - keeps track of iterations */
struct IterationRecord {
  uint64_t iteration_number;  // Iterations counter
  int64_t begin_ts;           // Beginning timestamp
  int64_t end_ts;             // Ending timestamp
  
  // Counters of the iteration
  std::map<std::string, uint32_t> iteration_counters;
  std::map<std::string, uint64_t> algorithm_counters;
  
  IterationRecord():
    iteration_number(0),
    begin_ts(0), end_ts(0)
  {}
  
  IterationRecord(const int64_t& ts):
    iteration_number(0),
    begin_ts(ts), end_ts(ts)
  {}
  
  bool Reset(const int64_t& ts) {
    iteration_number = 0;
    begin_ts = ts;
    end_ts = ts;
    return true;
  }
  
  bool Increment(const int64_t& new_ts) {
    if (new_ts <= end_ts) {
      LOG(ERROR) << "Can not increment iteration as new_ts <= end_ts "
          << new_ts << " <= " << end_ts;
      return false;
    }
    iteration_number++;
    begin_ts = end_ts;
    end_ts = new_ts;
    return true;
  }
  
  bool ResetIterationCounters() {
    for (auto &pair : iteration_counters) {
      pair.second = 0;
    }
    return true;
  }
  
  std::string IterationCountersToString() const {
    std::string str = std::to_string(iteration_number) + ", ";
    for (const auto &pair : iteration_counters) {
      str += (pair.first + " " + std::to_string(pair.second) + ", ");
    }
    return str;
  }
  
  std::string AlgorithmCountersToString() const {
    std::string str = std::to_string(iteration_number) + ", ";
    for (const auto &pair : algorithm_counters) {
      str += (pair.first + " " + std::to_string(pair.second) + ", ");
    }
    return str;
  }
  
}; // IterationRecord


class Model {
 public:
  /** Constructor - gets a config file */
  Model() {}
  
  /** Destructor - This should never be called - derived class destructor should be called */
  virtual ~Model() {}
  
  // Iteration interval
  virtual const int64_t& IterationInterval() = 0;
  
  // Run iteration with observations
  virtual bool RunIteration(
      const int64_t& iteration_end_ts,
      const anantak::ObservationsVectorStoreMap& observations_map) {}
  
  // Run iteration without any observations
  virtual bool RunIteration(
      const int64_t& iteration_end_ts) {}
  
  // Start a new iteration
  //virtual bool StartIteration(const int64_t& iteration_end_ts) {}
  
  // Create new states for an iteration
  //virtual bool CreateIterationStates(const int64_t& iteration_end_ts) {}
  
  // Process the observations
  //virtual bool CreateIterationResiduals(const anantak::ObservationsVectorStoreMap& observations_map) {}
  
  // Run filtering for the iteration
  //virtual bool RunFiltering(const int64_t& iteration_end_ts) = 0;
  
  // Are the results ready to be published? e.g. did the results change? 
  virtual bool AreResultsReady() const {}
  
  // Get results in form of a message
  virtual inline const anantak::MessageType& GetResultsMessage() {}
  
  // Show results
  virtual bool Show() {}
  
  // Hide results
  virtual bool Hide() {}
  
};

typedef std::function<bool(const anantak::SensorMsg&)> SensorMessageFilterType;


} // namespace anantak


