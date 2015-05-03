/**
 *  Loads a monocular camera calibration, set of raw image-names and their timings.
 *  Undistorts each image, tracks features across images using libviso2.
 *  Packs each sparse feature set in a protocol buffer and saves the file.
 *  Potentially we can also add AprilTag detection to this too.
 *
 */

/** std includes */
#include <vector>
#include <string>
#include <fstream>
#include <algorithm>
#include <memory>

/** Google libraries */
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <google/protobuf/message.h>

/** Anantak includes */
#include "common_config.h"
#include "Filter/performance_tracker.h"
#include "Filter/circular_queue.h"
#include "DataQueue/message_file_writer.h"
#include "DataQueue/message_file_reader.h"
#include "DataQueue/message_file_stats.h"
#include <Utilities/common_functions.h>

/** Protocol buffers */
#include "sensor_messages.pb.h"

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
  
  Eigen::Vector4d QuaternionVector(const QuaternionType& q) {
    Eigen::Vector4d v = q.coeffs();
    return v;
  }
  
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
  
  // April Tag reading
  struct AprilTagReadingType {
    int64_t timestamp;    // timestamp of the observation - serves as image index
    int32_t camera_num;   // camera from which this was seen
    std::string tag_id;
    Eigen::Matrix<double,2,4> image_coords;  // tag corner coordinates in image
    
    AprilTagReadingType() : timestamp(0), camera_num(0), tag_id(16, ' ') {
      image_coords.setZero();
    }
    
    bool Reset() {
      timestamp = 0;
      camera_num = 0;
      tag_id = std::string(16, ' ');
      image_coords.setZero();
      return true;
    }
    
    bool IsZero() const {
      return (timestamp==0);
    }
    
    // Set from AprilTagMessage
    bool SetFromAprilTagMessage(const int64_t& ts, const int32_t& cam_num, const int32_t& i_tag,
        const anantak::AprilTagMessage& apriltag_msg) {
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
  };
  

  Eigen::Matrix<double,4,3> AprilTagCorners3d(const double& tag_size = 1.0) {
    Eigen::Matrix<double,4,3> corners;
    corners << -1.0, -1.0,  0.0,
                1.0, -1.0,  0.0,
                1.0,  1.0,  0.0,
               -1.0,  1.0,  0.0;
    corners *= tag_size*0.5;
    return corners;
  }

  struct AprilTagSighting {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::string tag_id;
    double tag_size;
    int32_t image_idx;
    Eigen::Matrix<double,2,4> image_coords;  // tag corner coordinates in image
    double reproj_error;  /**< indicates how well estimated rotn/trans fits the seen corners */
    Eigen::Matrix3d cam_K; // Camera matrix
    Eigen::Matrix3d TrC;   // 3x3
    QuaternionType  TqC;   // 
    Eigen::Vector3d TpC;   // 3x1
    Eigen::Matrix3d TrW;   // 3x3
    QuatColVecType  TqW;   // 4x1 .coeffs()
    Eigen::Vector3d WpT;   // 3x1
    Eigen::Matrix<double,3,4> Cpf;    // Tag corner 3d coordinates in camera frame
    
    // Check if sighting is valid
    bool IsValid() {
      return (TrC.allFinite() && TpC.allFinite() && TrW.allFinite() && WpT.allFinite()
              && image_coords.allFinite() && Cpf.allFinite());
    }
  };
  typedef std::vector<AprilTagSighting, Eigen::aligned_allocator<AprilTagSighting>>
      AprilTagSightingsVectorType;

  struct AprilTagPose {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::string tag_id;
    Eigen::Matrix3d WrT;
    Eigen::Vector3d WpT;
    Eigen::Quaterniond WqT;
    double tag_size;
  };
  std::ostream& operator<<(std::ostream& os, const AprilTagPose& pose) {
    return os << pose.tag_id << ": " << pose.WqT.coeffs().transpose() << ", "
        << pose.WpT.transpose();
  }
  
  struct AprilTagMap {
    std::vector<std::string> tag_ids;     /**< Collection of April tag ids */
    std::vector<AprilTagPose> tag_poses;  /**< Collection of April tags psoes in one ref frame */
    
    // Access size
    int32_t num_tags() {return tag_ids.size();}
    
    // Add a tag to the map
    bool AddTag(const std::string& tag_id, double tag_size=0.) {
      auto i = std::find(tag_ids.begin(), tag_ids.end(), tag_id);
      if (i==tag_ids.end()) {
        tag_ids.push_back(tag_id);
        AprilTagPose tp;  // empty tag pose
        tp.tag_size = tag_size;
        tag_poses.push_back(tp);
      }
      return true;
    }  // AddTag
    
    // Add new tags from another map
    bool AddNewTags(const AprilTagMap& map) {
      for (int i_tag=0; i_tag<map.tag_ids.size(); i_tag++) {
        auto itr = std::find(tag_ids.begin(), tag_ids.end(), map.tag_ids[i_tag]);
        if (itr==tag_ids.end()) {
          tag_ids.push_back(map.tag_ids[i_tag]);  // copy tag id
          AprilTagPose tp = map.tag_poses[i_tag]; // copy tag pose
          tag_poses.push_back(tp);
        } // if
      } // for all tags in a given map
      return true;
    }  // AddNewTags
    
    // Check if another map overlaps with this one
    bool MapOverlaps(const AprilTagMap& map, const int32_t num_common_tags = 1) {
      int32_t n_common = 0;
      int32_t i_tag = 0;
      while (n_common<num_common_tags && i_tag<map.tag_ids.size()) {
        auto itr = std::find(tag_ids.begin(), tag_ids.end(), map.tag_ids[i_tag]);
        if (itr==tag_ids.end()) {
          // tag not found
        } else {
          // tag found
          n_common++;
        }
        i_tag++;
      }
      return (n_common >= num_common_tags);
    }  // MapOverlaps
    
    
  };  // struct AprilTagMap
  
  /* Apriltag sighting convex cost function - all four corners are addressed
   * Cpf = CrI * WrI' * ( WpT + WrT * Tpf - WpI ) + CpI
   * [x,y,z]' = K * Cpf
   * residuals = [u*z-x, v*z-y]/(sigma_im * z_hat)
   *  Cpf: position of feature in camera frame - intermediate variable
   *  CrI: rotation of imu in camera frame - fixed as an input
   *  WrI: rotation of imu in world frame - fixed as an input
   *  WpT: position of tag in world frame - to be estimated - Vector3d
   *  WrT: rotation of tag in world frame - fixed as an input
   *  Tpf: position of feature in tag frame - fixed as an input, feature is a corner on the tag
   *  WpI: position of imu in world frame - to be estimated - Vector3d
   *  CpI: position of imu in camera frame - to be estimated - Vector3d
   *  K: Camera projection matrix - fixed as input
   *  sigma_im: stddev of image corner localization - input assumption
   *  z_hat: estimate of feature depth in camera frame calculated from before - fixed as input
   * Using z_hat in the denominator helps keep the formulation convex.
   * Another convex formulation could be:
   * residuals = [u*z-x, v*z-y]/(sqrt(z) * sigma_im * sqrt(z_hat))
   * Here cost of sqrt calc is incurred, jacobians are tough, wonder if you gain accuracy?
   * Parameters are:
   *  WpI, WpT and CpI
   * Residuals are:
   *  u,v for each of four corners
   */
  class AprilTagCamImuConvexCostFunction : public ceres::SizedCostFunction<8,3,3,3> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef Eigen::Map<const Eigen::Vector3d> MapVector3dConstType;
    typedef Eigen::Map<Eigen::Matrix<double,8,1>> MapVector8dType;
    typedef Eigen::Map<Eigen::Matrix<double,2,4,Eigen::ColMajor>> MapMatrix2x4Type;
    typedef Eigen::Map<Eigen::Matrix<double,8,3,Eigen::RowMajor>> MapMatrix8x3Type;
    
    AprilTagCamImuConvexCostFunction(
        const Eigen::Matrix3d *CrI, const Eigen::Matrix3d *WrI, const Eigen::Matrix3d *WrT,
        const Eigen::Matrix3d *K,
        const Eigen::Matrix<double,2,4> *corners_im,
        const Eigen::Matrix<double,3,4> *Cpf,
        const double* sigma_im,
        const double* tag_size ) :
      CrI_(CrI), WrI_(WrI), WrT_(WrT), K_(K), corners_im_(corners_im), Cpf_(Cpf), sigma_im_(sigma_im),
      tag_size_(tag_size), I3_(Eigen::Matrix3d::Identity()) {
      // Precompute matrices
      K_CrW_ = (*K_) * (*CrI_) * (*WrI_).transpose();
      K_CrT_ = K_CrW_ * (*WrT_);
      sigma_z_hat_ = (*sigma_im_) * (*Cpf).row(2);  // sigma_im * depth of each corner
      diag_inv_sigma_z_hat_ = sigma_z_hat_.cwiseInverse().asDiagonal();
      Tpf_ = AprilTagCorners3d(*tag_size_).transpose();
      K_CrT_Tpf_ = K_CrT_ * Tpf_;
      // Precompute jacobians - they are constants in this convex formulation
      MultMatToJacobian(&K_CrW_, corners_im, &sigma_z_hat_, &duv_by_dWpT_);
      duv_by_dWpI_ = -duv_by_dWpT_;
      MultMatToJacobian(K_, corners_im, &sigma_z_hat_, &duv_by_dCpI_);
    }
    
    virtual ~AprilTagCamImuConvexCostFunction() {}
    
    bool MultMatToJacobian(const Eigen::Matrix3d *mat, const Eigen::Matrix<double,2,4> *uv,
        const Eigen::Matrix<double,1,4> *sigma, Eigen::Matrix<double,8,3> *jac) {
      for (int i=0; i<4; i++) {
        (*jac).block<2,3>(2*i,0) = (*uv).col(i) * (*mat).row(2) - (*mat).block<2,3>(0,0);
        (*jac).block<2,3>(2*i,0) /= (*sigma)[i];
      }
    }
    
    virtual bool Evaluate(
        double const* const* parameters,
        double* residuals,
        double** jacobians) const {
      
      MapVector3dConstType WpI(parameters[0]);
      MapVector3dConstType WpT(parameters[1]);
      MapVector3dConstType CpI(parameters[2]);
      MapMatrix2x4Type resid(residuals);
      
      Eigen::Vector3d K_CpT = K_CrW_*(WpT - WpI) + (*K_)*CpI;
      Eigen::Matrix<double,3,4> K_Cpf = K_CrT_Tpf_.colwise() + K_CpT;
      resid =  (*corners_im_) * K_Cpf.row(2).asDiagonal() - K_Cpf.block<2,4>(0,0);
      resid *= diag_inv_sigma_z_hat_;
      
      if (jacobians != NULL) {
        if (jacobians[0] != NULL) {
          MapMatrix8x3Type jac0(jacobians[0]);
          jac0 = duv_by_dWpI_;
        }
        if (jacobians[1] != NULL) {
          MapMatrix8x3Type jac1(jacobians[1]);
          jac1 = duv_by_dWpT_;
        }
        if (jacobians[2] != NULL) {
          MapMatrix8x3Type jac2(jacobians[2]);
          jac2 = duv_by_dCpI_;
        }
      }
      
      return true;
    } // Evaluate
    
   protected:
    const Eigen::Matrix3d *CrI_;
    const Eigen::Matrix3d *WrI_;
    const Eigen::Matrix3d *WrT_;
    const Eigen::Matrix3d *K_;
    const Eigen::Matrix<double,2,4> *corners_im_;
    const Eigen::Matrix<double,3,4> *Cpf_;
    const double* sigma_im_;
    const double* tag_size_;
    const Eigen::Matrix3d I3_;
    Eigen::Matrix3d K_CrW_, K_CrT_;
    Eigen::Matrix<double,1,4> sigma_z_hat_;
    Eigen::Matrix<double,4,4> diag_inv_sigma_z_hat_;
    Eigen::Matrix<double,3,4> Tpf_, K_CrT_Tpf_;   // Tag corners in tag frame
    Eigen::Matrix<double,8,3> duv_by_dWpT_, duv_by_dWpI_, duv_by_dCpI_;
  }; // AprilTagCamImuConvexCostFunction 

  class AprilTagFixedPoseConvexCostFunction : public ceres::SizedCostFunction<8,3,3> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef Eigen::Map<const Eigen::Vector3d> MapVector3dConstType;
    typedef Eigen::Map<Eigen::Matrix<double,8,1>> MapVector8dType;
    typedef Eigen::Map<Eigen::Matrix<double,2,4,Eigen::ColMajor>> MapMatrix2x4Type;
    typedef Eigen::Map<Eigen::Matrix<double,8,3,Eigen::RowMajor>> MapMatrix8x3Type;
    
    AprilTagFixedPoseConvexCostFunction(
        const Eigen::Matrix3d *CrI, const Eigen::Matrix3d *WrI, const Eigen::Matrix3d *WrT,
        const Eigen::Matrix3d *K,
        const Eigen::Matrix<double,2,4> *corners_im,
        const Eigen::Matrix<double,3,4> *Cpf,
        const Eigen::Vector3d *WpI,
        const double* sigma_im,
        const double* tag_size ) :
      CrI_(CrI), WrI_(WrI), WrT_(WrT), K_(K), corners_im_(corners_im), Cpf_(Cpf),
      WpI_(WpI),
      sigma_im_(sigma_im), tag_size_(tag_size), I3_(Eigen::Matrix3d::Identity()) {
      // Precompute matrices
      K_CrW_ = (*K_) * (*CrI_) * (*WrI_).transpose();
      K_CrT_ = K_CrW_ * (*WrT_);
      sigma_z_hat_ = (*sigma_im_) * (*Cpf).row(2);  // sigma_im * depth of each corner
      diag_inv_sigma_z_hat_ = sigma_z_hat_.cwiseInverse().asDiagonal();
      Tpf_ = AprilTagCorners3d(*tag_size_).transpose();
      K_CrT_Tpf_ = K_CrT_ * Tpf_;
      // Precompute jacobians - they are constants in this convex formulation
      MultMatToJacobian(&K_CrW_, corners_im, &sigma_z_hat_, &duv_by_dWpT_);
      duv_by_dWpI_ = -duv_by_dWpT_;
      MultMatToJacobian(K_, corners_im, &sigma_z_hat_, &duv_by_dCpI_);
    }
    
    virtual ~AprilTagFixedPoseConvexCostFunction() {}
    
    bool MultMatToJacobian(const Eigen::Matrix3d *mat, const Eigen::Matrix<double,2,4> *uv,
        const Eigen::Matrix<double,1,4> *sigma, Eigen::Matrix<double,8,3> *jac) {
      for (int i=0; i<4; i++) {
        (*jac).block<2,3>(2*i,0) = (*uv).col(i) * (*mat).row(2) - (*mat).block<2,3>(0,0);
        (*jac).block<2,3>(2*i,0) /= (*sigma)[i];
      }
    }
    
    virtual bool Evaluate(
        double const* const* parameters,
        double* residuals,
        double** jacobians) const {
      
      MapVector3dConstType WpT(parameters[0]);
      MapVector3dConstType CpI(parameters[1]);
      MapMatrix2x4Type resid(residuals);
      
      Eigen::Vector3d K_CpT = K_CrW_*(WpT - (*WpI_)) + (*K_)*CpI;
      Eigen::Matrix<double,3,4> K_Cpf = K_CrT_Tpf_.colwise() + K_CpT;
      resid =  (*corners_im_) * K_Cpf.row(2).asDiagonal() - K_Cpf.block<2,4>(0,0);
      resid *= diag_inv_sigma_z_hat_;
      
      if (jacobians != NULL) {
        if (jacobians[0] != NULL) {
          MapMatrix8x3Type jac1(jacobians[0]);
          jac1 = duv_by_dWpT_;
        }
        if (jacobians[1] != NULL) {
          MapMatrix8x3Type jac2(jacobians[1]);
          jac2 = duv_by_dCpI_;
        }
      }
      
      return true;
    } // Evaluate
    
   protected:
    const Eigen::Matrix3d *CrI_;
    const Eigen::Matrix3d *WrI_;
    const Eigen::Matrix3d *WrT_;
    const Eigen::Matrix3d *K_;
    const Eigen::Matrix<double,2,4> *corners_im_;
    const Eigen::Matrix<double,3,4> *Cpf_;
    const Eigen::Vector3d *WpI_;
    const double* sigma_im_;
    const double* tag_size_;
    const Eigen::Matrix3d I3_;
    Eigen::Matrix3d K_CrW_, K_CrT_;
    Eigen::Matrix<double,1,4> sigma_z_hat_;
    Eigen::Matrix<double,4,4> diag_inv_sigma_z_hat_;
    Eigen::Matrix<double,3,4> Tpf_, K_CrT_Tpf_;   // Tag corners in tag frame
    Eigen::Matrix<double,8,3> duv_by_dWpT_, duv_by_dWpI_, duv_by_dCpI_;
  }; // AprilTagFixedPoseConvexCostFunction 

  class AprilTagCamImuConvexCostFunction2 : public ceres::SizedCostFunction<8,3,3> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef Eigen::Map<const Eigen::Vector3d> MapVector3dConstType;
    typedef Eigen::Map<Eigen::Matrix<double,8,1>> MapVector8dType;
    typedef Eigen::Map<Eigen::Matrix<double,2,4,Eigen::ColMajor>> MapMatrix2x4Type;
    typedef Eigen::Map<Eigen::Matrix<double,8,3,Eigen::RowMajor>> MapMatrix8x3Type;
    
    AprilTagCamImuConvexCostFunction2(
        const Eigen::Matrix3d *CrI, const Eigen::Matrix3d *WrI, const Eigen::Matrix3d *WrT,
        const Eigen::Matrix3d *K,
        const Eigen::Matrix<double,2,4> *corners_im,
        const Eigen::Matrix<double,3,4> *Cpf,
        const double* sigma_im,
        const double* tag_size ) :
      CrI_(CrI), WrI_(WrI), WrT_(WrT), K_(K), corners_im_(corners_im), Cpf_(Cpf), sigma_im_(sigma_im),
      tag_size_(tag_size), I3_(Eigen::Matrix3d::Identity()) {
      // Precompute matrices
      K_CrW_ = (*K_) * (*CrI_) * (*WrI_).transpose();
      K_CrT_ = K_CrW_ * (*WrT_);
      sigma_z_hat_ = (*sigma_im_) * (*Cpf).row(2);  // sigma_im * depth of each corner
      diag_inv_sigma_z_hat_ = sigma_z_hat_.cwiseInverse().asDiagonal();
      Tpf_ = AprilTagCorners3d(*tag_size_).transpose();
      K_CrT_Tpf_ = K_CrT_ * Tpf_;
      // Precompute jacobians - they are constants in this convex formulation
      MultMatToJacobian(&K_CrW_, corners_im, &sigma_z_hat_, &duv_by_dWpT_);
      duv_by_dWpI_ = -duv_by_dWpT_;
      MultMatToJacobian(K_, corners_im, &sigma_z_hat_, &duv_by_dCpI_);
      CpI_.setZero();
    }
    
    virtual ~AprilTagCamImuConvexCostFunction2() {}
    
    bool MultMatToJacobian(const Eigen::Matrix3d *mat, const Eigen::Matrix<double,2,4> *uv,
        const Eigen::Matrix<double,1,4> *sigma, Eigen::Matrix<double,8,3> *jac) {
      for (int i=0; i<4; i++) {
        (*jac).block<2,3>(2*i,0) = (*uv).col(i) * (*mat).row(2) - (*mat).block<2,3>(0,0);
        (*jac).block<2,3>(2*i,0) /= (*sigma)[i];
      }
    }
    
    virtual bool Evaluate(
        double const* const* parameters,
        double* residuals,
        double** jacobians) const {
      
      MapVector3dConstType WpI(parameters[0]);
      MapVector3dConstType WpT(parameters[1]);
      MapMatrix2x4Type resid(residuals);
      
      Eigen::Vector3d K_CpT = K_CrW_*(WpT - WpI) + (*K_)*CpI_;
      Eigen::Matrix<double,3,4> K_Cpf = K_CrT_Tpf_.colwise() + K_CpT;
      resid =  (*corners_im_) * K_Cpf.row(2).asDiagonal() - K_Cpf.block<2,4>(0,0);
      resid *= diag_inv_sigma_z_hat_;
      
      if (jacobians != NULL) {
        if (jacobians[0] != NULL) {
          MapMatrix8x3Type jac0(jacobians[0]);
          jac0 = duv_by_dWpI_;
        }
        if (jacobians[1] != NULL) {
          MapMatrix8x3Type jac1(jacobians[1]);
          jac1 = duv_by_dWpT_;
        }
      }
      
      return true;
    } // Evaluate
    
   protected:
    const Eigen::Matrix3d *CrI_;
    const Eigen::Matrix3d *WrI_;
    const Eigen::Matrix3d *WrT_;
    const Eigen::Matrix3d *K_;
    const Eigen::Matrix<double,2,4> *corners_im_;
    const Eigen::Matrix<double,3,4> *Cpf_;
    const double* sigma_im_;
    const double* tag_size_;
    const Eigen::Matrix3d I3_;
    Eigen::Matrix3d K_CrW_, K_CrT_;
    Eigen::Matrix<double,1,4> sigma_z_hat_;
    Eigen::Matrix<double,4,4> diag_inv_sigma_z_hat_;
    Eigen::Matrix<double,3,4> Tpf_, K_CrT_Tpf_;   // Tag corners in tag frame
    Eigen::Matrix<double,8,3> duv_by_dWpT_, duv_by_dWpI_, duv_by_dCpI_;
    Eigen::Vector3d CpI_;
  }; // AprilTagCamImuConvexCostFunction2
  
  class AprilTagFixedPoseConvexCostFunction2 : public ceres::SizedCostFunction<8,3> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef Eigen::Map<const Eigen::Vector3d> MapVector3dConstType;
    typedef Eigen::Map<Eigen::Matrix<double,8,1>> MapVector8dType;
    typedef Eigen::Map<Eigen::Matrix<double,2,4,Eigen::ColMajor>> MapMatrix2x4Type;
    typedef Eigen::Map<Eigen::Matrix<double,8,3,Eigen::RowMajor>> MapMatrix8x3Type;
    
    AprilTagFixedPoseConvexCostFunction2(
        const Eigen::Matrix3d *CrI, const Eigen::Matrix3d *WrI, const Eigen::Matrix3d *WrT,
        const Eigen::Matrix3d *K,
        const Eigen::Matrix<double,2,4> *corners_im,
        const Eigen::Matrix<double,3,4> *Cpf,
        const Eigen::Vector3d *WpI,
        const double* sigma_im,
        const double* tag_size ) :
      CrI_(CrI), WrI_(WrI), WrT_(WrT), K_(K), corners_im_(corners_im), Cpf_(Cpf), sigma_im_(sigma_im),
      WpI_(WpI),
      tag_size_(tag_size), I3_(Eigen::Matrix3d::Identity()) {
      // Precompute matrices
      K_CrW_ = (*K_) * (*CrI_) * (*WrI_).transpose();
      K_CrT_ = K_CrW_ * (*WrT_);
      sigma_z_hat_ = (*sigma_im_) * (*Cpf).row(2);  // sigma_im * depth of each corner
      diag_inv_sigma_z_hat_ = sigma_z_hat_.cwiseInverse().asDiagonal();
      Tpf_ = AprilTagCorners3d(*tag_size_).transpose();
      K_CrT_Tpf_ = K_CrT_ * Tpf_;
      // Precompute jacobians - they are constants in this convex formulation
      MultMatToJacobian(&K_CrW_, corners_im, &sigma_z_hat_, &duv_by_dWpT_);
      duv_by_dWpI_ = -duv_by_dWpT_;
      MultMatToJacobian(K_, corners_im, &sigma_z_hat_, &duv_by_dCpI_);
      CpI_.setZero();
    }
    
    virtual ~AprilTagFixedPoseConvexCostFunction2() {}
    
    bool MultMatToJacobian(const Eigen::Matrix3d *mat, const Eigen::Matrix<double,2,4> *uv,
        const Eigen::Matrix<double,1,4> *sigma, Eigen::Matrix<double,8,3> *jac) {
      for (int i=0; i<4; i++) {
        (*jac).block<2,3>(2*i,0) = (*uv).col(i) * (*mat).row(2) - (*mat).block<2,3>(0,0);
        (*jac).block<2,3>(2*i,0) /= (*sigma)[i];
      }
    }
    
    virtual bool Evaluate(
        double const* const* parameters,
        double* residuals,
        double** jacobians) const {
      
      MapVector3dConstType WpT(parameters[0]);
      MapMatrix2x4Type resid(residuals);
      
      Eigen::Vector3d K_CpT = K_CrW_*(WpT - (*WpI_)) + (*K_)*CpI_;
      Eigen::Matrix<double,3,4> K_Cpf = K_CrT_Tpf_.colwise() + K_CpT;
      resid =  (*corners_im_) * K_Cpf.row(2).asDiagonal() - K_Cpf.block<2,4>(0,0);
      resid *= diag_inv_sigma_z_hat_;
      
      if (jacobians != NULL) {
        if (jacobians[0] != NULL) {
          MapMatrix8x3Type jac1(jacobians[0]);
          jac1 = duv_by_dWpT_;
        }
      }
      
      return true;
    } // Evaluate
    
   protected:
    const Eigen::Matrix3d *CrI_;
    const Eigen::Matrix3d *WrI_;
    const Eigen::Matrix3d *WrT_;
    const Eigen::Matrix3d *K_;
    const Eigen::Matrix<double,2,4> *corners_im_;
    const Eigen::Matrix<double,3,4> *Cpf_;
    const Eigen::Vector3d *WpI_;
    const double* sigma_im_;
    const double* tag_size_;
    const Eigen::Matrix3d I3_;
    Eigen::Matrix3d K_CrW_, K_CrT_;
    Eigen::Matrix<double,1,4> sigma_z_hat_;
    Eigen::Matrix<double,4,4> diag_inv_sigma_z_hat_;
    Eigen::Matrix<double,3,4> Tpf_, K_CrT_Tpf_;   // Tag corners in tag frame
    Eigen::Matrix<double,8,3> duv_by_dWpT_, duv_by_dWpI_, duv_by_dCpI_;
    Eigen::Vector3d CpI_;
  }; // AprilTagFixedPoseConvexCostFunction2
  
  class AprilTagCamImuConvexCostFunction3 : public ceres::SizedCostFunction<8,3,3,3,3> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef Eigen::Map<const Eigen::Vector3d> MapVector3dConstType;
    typedef Eigen::Map<Eigen::Matrix<double,8,1>> MapVector8dType;
    typedef Eigen::Map<Eigen::Matrix<double,2,4,Eigen::ColMajor>> MapMatrix2x4Type;
    typedef Eigen::Map<Eigen::Matrix<double,8,3,Eigen::RowMajor>> MapMatrix8x3Type;
    
    AprilTagCamImuConvexCostFunction3(
        const Eigen::Matrix3d *CrI, const Eigen::Matrix3d *WrI, const Eigen::Matrix3d *WrT,
        const Eigen::Matrix3d *K,
        const Eigen::Matrix<double,2,4> *corners_im,
        const Eigen::Matrix<double,3,4> *Cpf,
        const double* sigma_im,
        const double* tag_size,
        const double frac) :
      CrI_(CrI), WrI_(WrI), WrT_(WrT), K_(K), corners_im_(corners_im), Cpf_(Cpf), sigma_im_(sigma_im),
      tag_size_(tag_size), I3_(Eigen::Matrix3d::Identity()), frac_(frac), frac_1_(1.-frac) {
      // Precompute matrices
      K_CrW_ = (*K_) * (*CrI_) * (*WrI_).transpose();
      K_CrT_ = K_CrW_ * (*WrT_);
      sigma_z_hat_ = (*sigma_im_) * (*Cpf).row(2);  // sigma_im * depth of each corner
      diag_inv_sigma_z_hat_ = sigma_z_hat_.cwiseInverse().asDiagonal();
      Tpf_ = AprilTagCorners3d(*tag_size_).transpose();
      K_CrT_Tpf_ = K_CrT_ * Tpf_;
      // Precompute jacobians - they are constants in this convex formulation
      MultMatToJacobian(&K_CrW_, corners_im, &sigma_z_hat_, &duv_by_dWpT_);
      duv_by_dWpI0_ = -frac_1_*duv_by_dWpT_;
      duv_by_dWpI1_ = -frac_*duv_by_dWpT_;
      MultMatToJacobian(K_, corners_im, &sigma_z_hat_, &duv_by_dCpI_);
    }
    
    virtual ~AprilTagCamImuConvexCostFunction3() {}
    
    bool MultMatToJacobian(const Eigen::Matrix3d *mat, const Eigen::Matrix<double,2,4> *uv,
        const Eigen::Matrix<double,1,4> *sigma, Eigen::Matrix<double,8,3> *jac) {
      for (int i=0; i<4; i++) {
        (*jac).block<2,3>(2*i,0) = (*uv).col(i) * (*mat).row(2) - (*mat).block<2,3>(0,0);
        (*jac).block<2,3>(2*i,0) /= (*sigma)[i];
      }
    }
    
    virtual bool Evaluate(
        double const* const* parameters,
        double* residuals,
        double** jacobians) const {
      
      MapVector3dConstType WpI0(parameters[0]);
      MapVector3dConstType WpI1(parameters[1]);
      MapVector3dConstType WpT(parameters[2]);
      MapVector3dConstType CpI(parameters[3]);
      MapMatrix2x4Type resid(residuals);
      
      Eigen::Vector3d K_CpT = K_CrW_*(WpT - frac_1_*WpI0 - frac_*WpI1) + (*K_)*CpI;
      Eigen::Matrix<double,3,4> K_Cpf = K_CrT_Tpf_.colwise() + K_CpT;
      resid =  (*corners_im_) * K_Cpf.row(2).asDiagonal() - K_Cpf.block<2,4>(0,0);
      resid *= diag_inv_sigma_z_hat_;
      
      if (jacobians != NULL) {
        if (jacobians[0] != NULL) {
          MapMatrix8x3Type jac0(jacobians[0]);
          jac0 = duv_by_dWpI0_;
        }
        if (jacobians[1] != NULL) {
          MapMatrix8x3Type jac1(jacobians[1]);
          jac1 = duv_by_dWpI1_;
        }
        if (jacobians[2] != NULL) {
          MapMatrix8x3Type jac2(jacobians[2]);
          jac2 = duv_by_dWpT_;
        }
        if (jacobians[3] != NULL) {
          MapMatrix8x3Type jac3(jacobians[3]);
          jac3 = duv_by_dCpI_;
        }
      }
      
      return true;
    } // Evaluate
    
   protected:
    const Eigen::Matrix3d *CrI_;
    const Eigen::Matrix3d *WrI_;
    const Eigen::Matrix3d *WrT_;
    const Eigen::Matrix3d *K_;
    const Eigen::Matrix<double,2,4> *corners_im_;
    const Eigen::Matrix<double,3,4> *Cpf_;
    const double* sigma_im_;
    const double* tag_size_;
    const Eigen::Matrix3d I3_;
    Eigen::Matrix3d K_CrW_, K_CrT_;
    Eigen::Matrix<double,1,4> sigma_z_hat_;
    Eigen::Matrix<double,4,4> diag_inv_sigma_z_hat_;
    Eigen::Matrix<double,3,4> Tpf_, K_CrT_Tpf_;   // Tag corners in tag frame
    Eigen::Matrix<double,8,3> duv_by_dWpT_, duv_by_dWpI0_, duv_by_dWpI1_, duv_by_dCpI_;
    double frac_, frac_1_;
  }; // AprilTagCamImuConvexCostFunction3

  class AprilTagFixedPoseConvexCostFunction3 : public ceres::SizedCostFunction<8,3,3,3> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef Eigen::Map<const Eigen::Vector3d> MapVector3dConstType;
    typedef Eigen::Map<Eigen::Matrix<double,8,1>> MapVector8dType;
    typedef Eigen::Map<Eigen::Matrix<double,2,4,Eigen::ColMajor>> MapMatrix2x4Type;
    typedef Eigen::Map<Eigen::Matrix<double,8,3,Eigen::RowMajor>> MapMatrix8x3Type;
    
    AprilTagFixedPoseConvexCostFunction3(
        const Eigen::Matrix3d *CrI, const Eigen::Matrix3d *WrI, const Eigen::Matrix3d *WrT,
        const Eigen::Matrix3d *K,
        const Eigen::Matrix<double,2,4> *corners_im,
        const Eigen::Matrix<double,3,4> *Cpf,
        const Eigen::Vector3d *WpI0,
        const double* sigma_im,
        const double* tag_size,
        const double frac) :
      CrI_(CrI), WrI_(WrI), WrT_(WrT), K_(K), corners_im_(corners_im), Cpf_(Cpf), sigma_im_(sigma_im),
      WpI0_(WpI0),
      tag_size_(tag_size), I3_(Eigen::Matrix3d::Identity()), frac_(frac), frac_1_(1.-frac) {
      // Precompute matrices
      K_CrW_ = (*K_) * (*CrI_) * (*WrI_).transpose();
      K_CrT_ = K_CrW_ * (*WrT_);
      sigma_z_hat_ = (*sigma_im_) * (*Cpf).row(2);  // sigma_im * depth of each corner
      diag_inv_sigma_z_hat_ = sigma_z_hat_.cwiseInverse().asDiagonal();
      Tpf_ = AprilTagCorners3d(*tag_size_).transpose();
      K_CrT_Tpf_ = K_CrT_ * Tpf_;
      // Precompute jacobians - they are constants in this convex formulation
      MultMatToJacobian(&K_CrW_, corners_im, &sigma_z_hat_, &duv_by_dWpT_);
      duv_by_dWpI0_ = -frac_1_*duv_by_dWpT_;
      duv_by_dWpI1_ = -frac_*duv_by_dWpT_;
      MultMatToJacobian(K_, corners_im, &sigma_z_hat_, &duv_by_dCpI_);
      frac_1_WpI0_ = frac_1_ * (*WpI0_);
    }
    
    virtual ~AprilTagFixedPoseConvexCostFunction3() {}
    
    bool MultMatToJacobian(const Eigen::Matrix3d *mat, const Eigen::Matrix<double,2,4> *uv,
        const Eigen::Matrix<double,1,4> *sigma, Eigen::Matrix<double,8,3> *jac) {
      for (int i=0; i<4; i++) {
        (*jac).block<2,3>(2*i,0) = (*uv).col(i) * (*mat).row(2) - (*mat).block<2,3>(0,0);
        (*jac).block<2,3>(2*i,0) /= (*sigma)[i];
      }
    }
    
    virtual bool Evaluate(
        double const* const* parameters,
        double* residuals,
        double** jacobians) const {
      
      MapVector3dConstType WpI1(parameters[0]);
      MapVector3dConstType WpT(parameters[1]);
      MapVector3dConstType CpI(parameters[2]);
      MapMatrix2x4Type resid(residuals);
      
      Eigen::Vector3d K_CpT = K_CrW_*(WpT - frac_1_WpI0_ - frac_*WpI1) + (*K_)*CpI;
      Eigen::Matrix<double,3,4> K_Cpf = K_CrT_Tpf_.colwise() + K_CpT;
      resid =  (*corners_im_) * K_Cpf.row(2).asDiagonal() - K_Cpf.block<2,4>(0,0);
      resid *= diag_inv_sigma_z_hat_;
      
      if (jacobians != NULL) {
        if (jacobians[0] != NULL) {
          MapMatrix8x3Type jac1(jacobians[0]);
          jac1 = duv_by_dWpI1_;
        }
        if (jacobians[1] != NULL) {
          MapMatrix8x3Type jac2(jacobians[1]);
          jac2 = duv_by_dWpT_;
        }
        if (jacobians[2] != NULL) {
          MapMatrix8x3Type jac3(jacobians[2]);
          jac3 = duv_by_dCpI_;
        }
      }
      
      return true;
    } // Evaluate
    
   protected:
    const Eigen::Matrix3d *CrI_;
    const Eigen::Matrix3d *WrI_;
    const Eigen::Matrix3d *WrT_;
    const Eigen::Matrix3d *K_;
    const Eigen::Matrix<double,2,4> *corners_im_;
    const Eigen::Matrix<double,3,4> *Cpf_;
    const Eigen::Vector3d *WpI0_;
    const double* sigma_im_;
    const double* tag_size_;
    const Eigen::Matrix3d I3_;
    Eigen::Matrix3d K_CrW_, K_CrT_;
    Eigen::Matrix<double,1,4> sigma_z_hat_;
    Eigen::Matrix<double,4,4> diag_inv_sigma_z_hat_;
    Eigen::Matrix<double,3,4> Tpf_, K_CrT_Tpf_;   // Tag corners in tag frame
    Eigen::Matrix<double,8,3> duv_by_dWpT_, duv_by_dWpI0_, duv_by_dWpI1_, duv_by_dCpI_;
    Eigen::Vector3d frac_1_WpI0_;
    double frac_, frac_1_;
  }; // AprilTagFixedPoseConvexCostFunction3

  /* Small angle approximations used with errors in rotations */
  
  // Error to quaternion
  template<typename Vec3dType>
  Eigen::Quaterniond ErrorAngleAxisToQuaterion(const Eigen::MatrixBase<Vec3dType>& err,
      const double& small_angle_threshold = 1e-6) {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Vec3dType, 3)
    double err_angle = err.norm();
    Eigen::Quaterniond err_quat;
    if (err_angle < small_angle_threshold) {
      err_quat.coeffs() << 0.5*err[0], 0.5*err[1], 0.5*err[2], 1.;  // x,y,z,w
    } else {
      Eigen::AngleAxisd aa(err_angle, err/err_angle);
      err_quat = aa;
    }
    err_quat.normalize();
    return err_quat;
  }
  
  // Error to matrix
  template<typename Vec3dType>
  Eigen::Matrix3d ErrorAngleAxisToMatrix3d(const Eigen::MatrixBase<Vec3dType>& err,
      const double& small_angle_threshold = 1e-6) {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Vec3dType, 3)
    double err_angle = err.norm();
    Eigen::Matrix3d err_mat;
    if (err_angle < small_angle_threshold) {
      err_mat = Eigen::Matrix3d::Identity() - anantak::SkewSymmetricMatrix(err);
    } else {
      Eigen::AngleAxisd aa(err_angle, err/err_angle);
      err_mat = aa.toRotationMatrix();
    }
    return err_mat;    
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
      return (abs(state_)<Epsilon);
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
    typedef Eigen::Map<Eigen::Matrix<double,6,1>> MapVector6dType;
    typedef Eigen::Matrix<double,6,6> Matrix6dType;
  
    // Constructors
    Pose3dState() : State(), LqvG_(state_), GpL_(state_+4), dGaL_(error_), dGpL_(error_+3),
        timestamp_(0) {
      SetZero();
    }
    
    // Set to zero
    bool SetZero() {
      MapVector7dType s(state_);
      MapVector6dType e(error_);
      s.setZero(); s[3] = 1.;
      e.setZero();
      timestamp_ = 0;
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
    
    // Set timestamp
    bool SetTimestamp(const int64_t& ts) {
      timestamp_ = ts;
      return true;
    }
    
    // Recalculate the state from error state after optimization. Set errors to zero.
    bool Recalculate() {
      Eigen::Quaterniond LqG(LqvG_.data()); // x,y,z,w
      Eigen::Quaterniond dGqL = anantak::ErrorAngleAxisToQuaterion(dGaL_);  // w,x,y,z
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
    
    // Timestamp  -  not required, but could be needed in some cases
    int64_t timestamp_;
    
    // Helper maps
    MapVector4dType LqvG_; // quaternion as a vector x,y,z,w
    MapVector3dType GpL_;
    MapVector3dType dGaL_; // angleaxis as a vector
    MapVector3dType dGpL_;
    
  }; // Pose3dState
  
  
  /* Static April Tag state
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
      return (s.isZero() && e.isZero() && covariance_.isZero());
    }
    
    virtual ~CameraIntrinsicsState() {}
    
    // Create from a camera matrix
    bool Create(const Eigen::Matrix3d *K) {
      Eigen::Matrix3d K_ = *K;
      state_[0] = K_(0,0);
      state_[1] = K_(1,1);
      state_[2] = K_(0,2);
      state_[3] = K_(1,2);
      SetErrorZero();
      covariance_.setZero();
      return true;
    }
    
    // Create from four given values
    bool Create(const double& fx, const double& fy, const double& cx, const double& cy) {
      state_[0] = fx;
      state_[1] = fy;
      state_[2] = cx;
      state_[3] = cy;
      SetErrorZero();
      covariance_.setZero();
      return true;
    }
    
    // Recalculate the state from error state after optimization. Set errors to zero.
    bool Recalculate() {
      MapVector4dType s(state_);
      MapVector4dType e(error_);
      s += e;
      e.setZero();
      return true;
    }
    
    // Return a copy of the camera matrix from this state
    Eigen::Matrix3d CameraMatrix() const {
      Eigen::Matrix3d camera_matrix;
      camera_matrix << state_[0], 0., state_[2], 0., state_[1], state_[3], 0., 0., 1.;
      return camera_matrix;
    }
    
    // State
    double state_[4];   /**< fx, fy, cx, cy */
    // Error
    double error_[4];   /**< dfx, dfy, dcx, dcy */
    // Covariance
    Eigen::Matrix4d covariance_;  /**< Acutally stores inverse of sqrt covariance */
    
    // Helper matrices
    
  }; // CameraIntrinsicsState
  
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
      double grav_range_stdev = 4.0; // range of stdev's between grav ub and lb
      
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
  class ImuResidualFunction : public ceres::SizedCostFunction<16, 15,15,3> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef Eigen::Map<Eigen::Vector3d> MapVector3dType;
    typedef Eigen::Map<Eigen::Vector4d> MapVector4dType;
    typedef Eigen::Matrix<double,16,1> Vector16dType;
    typedef Eigen::Map<Eigen::Matrix<double,16,1>> MapVector16dType;
    typedef Eigen::Map<const Eigen::Matrix<double,15,1>> MapConstVector15dType;
    typedef Eigen::Map<const Eigen::Matrix<double, 3,1>> MapConstVector3dType;
    typedef Eigen::Matrix<double,16,15,Eigen::RowMajor> Matrix16x15RowType;
    typedef Eigen::Matrix<double,16, 3,Eigen::RowMajor> Matrix16x3RowType;
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
    Vector16dType R_;
    
    // First estimates Jacobians
    Matrix16x15RowType dR_dS0_, dR_dS1_;
    Matrix16x3RowType  dR_dG_;
    Vector16dType inv_sqrt_var_;
    
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
    
    // Setup the constraint with two flanking states and a starting reading
    bool Create(ImuState* state0, ImuState* state1, Vector3dState* gravity,
        const ImuReadingType& rdng0) {
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
      if (state0->timestamp_ >= state1->timestamp_) {
        LOG(ERROR) << "state0 timestamp >= state1 timestamp. Not allowed. " << state0->timestamp_
            << ", " << state1->timestamp_;
        return false;
      }
      
      // Reset the residual
      Reset();
      
      // Setup new data
      state0_ = state0; state1_ = state1; gravity_ = gravity;
      last_integrated_reading_ = rdng0; // copy reading
      // Setup integrals
      integral_till_last_reading_.SetFromState(*state0, *gravity);
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
      if (rdng.timestamp <= last_integrated_reading_.timestamp ||
          rdng.timestamp + int64_t(integral_till_last_reading_.td0*1e6) > state1_->timestamp_) {
        LOG(ERROR) << "Can not add a reading before latest one or beyond end state. last_ts = "
            << last_integrated_reading_.timestamp << " reading.ts (delay) = " << rdng.timestamp
            << " (" << int64_t(integral_till_last_reading_.td0*1e6) << ")"
            << " end state ts = " << state1_->timestamp_;
        return false;
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
      const int g_idx    = 15;      
      
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
      double grav_state_mag = gravity_->Gp_.norm();
      R_(g_idx   ,0) = grav_state_mag - options_.gravity_magnitude;
      
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
      dR_dS0_.setZero();  // 16x15
      dR_dS1_.setZero();  // 16x15
      dR_dG_.setZero();   // 16x3
      
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
      dR_dG_.block<1,3>(   g_idx,0) = gravity_->Gp_.transpose()/grav_state_mag;
      
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
      inv_sqrt_var_(g_idx,0) = 1./std::sqrt(options_.qg);
      
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
      MapVector16dType        R(residuals);
      
      /* Calculate residual from the error states. Linear model of the residual wrt error states
       * is used to recalculate the residual. */
      R = R_ + dR_dS0_*E0 + dR_dS1_*E1 + dR_dG_*EG;
      
      /* Jacobians are not recalculated with changing states. We use first estimates Jacobians
       * as this shown to keep the estimator consistent. */
      if (jacobians != NULL) {
        if (jacobians[0] != NULL) {
          MapMatrix16x15RowType dR_dS0(jacobians[0]);
          dR_dS0 = dR_dS0_;
        }
        if (jacobians[1] != NULL) {
          MapMatrix16x15RowType dR_dS1(jacobians[1]);
          dR_dS1 = dR_dS1_;
        }
        if (jacobians[2] != NULL) {
          MapMatrix16x3RowType dR_dG(jacobians[2]);
          dR_dG = dR_dG_;
        }
      }
      
      return true;
    }
    
    // Destructor
    virtual ~ImuResidualFunction() {}
    
  }; // ImuResidualFunction
  
  /* Vector3d Prior
   * Setup a prior on a 3d vector */
  class Vector3dPrior : public ceres::SizedCostFunction<3, 3> {
   public:
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
  
  /* NoisyPosePrior
   * Setup a prior on a noisy pose */
  class NoisyPosePrior : public ceres::SizedCostFunction<6, 6> {
   public:
    // Type declarations
    typedef Eigen::Map<Eigen::Matrix<double,6,1>> MapVector6dType;
    typedef Eigen::Map<const Eigen::Matrix<double,6,1>> MapConstVector6dType;
    typedef Eigen::Matrix<double,6,6,Eigen::RowMajor> Matrix6x6RowType;
    typedef Eigen::Map<Eigen::Matrix<double,6,6,Eigen::RowMajor>> MapMatrix6x6RowType;
    
    struct Options {
      double sigma_theta;
      double sigma_position;
      Options() {
        sigma_theta = 10.*RadiansPerDegree;   // in Radians
        sigma_position = 0.010;               // in meters
      }
    };
    
    // Options
    NoisyPosePrior::Options options_;
    
    // Data holders
    Eigen::Quaterniond expected_rotation_;    /**< IqW value of rotation */
    Eigen::Vector3d expected_position_;       /**< WpI value of position */
    anantak::Pose3dState *measurement_;       /**< One measurement of the pose */
    
    // Starting residual
    Eigen::Matrix<double,6,1> residual_;      /**< Starting residual */
    
    // Jacobian matrices
    Matrix6x6RowType dresidual_dmeasurement_;
    
    // Default constructor
    NoisyPosePrior():
      options_(),
      expected_rotation_(Eigen::Quaterniond::Identity()), expected_position_(Eigen::Vector3d::Zero()),
      measurement_(NULL) {
      residual_.setZero();
      dresidual_dmeasurement_.setZero();
    }
    
    // Default copy constructor
    NoisyPosePrior(const NoisyPosePrior& r) {
      options_=r.options_;
      expected_rotation_=r.expected_rotation_; expected_position_=r.expected_position_;
      measurement_=r.measurement_;
      residual_=r.residual_;
      dresidual_dmeasurement_=r.dresidual_dmeasurement_;
    }
    
    // Destructor
    virtual ~NoisyPosePrior() {}
    
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
        anantak::Pose3dState *measured_pose, NoisyPosePrior::Options *options) {
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
        NoisyPosePrior::Options *options) {
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
      double inv_sqrt_var_theta = 1./options_.sigma_theta;
      double inv_sqrt_var_position = 1./options_.sigma_position;
      Eigen::Matrix<double,6,1> inv_sqrt_var_mat_diag;
      inv_sqrt_var_mat_diag << inv_sqrt_var_theta, inv_sqrt_var_theta, inv_sqrt_var_theta,
          inv_sqrt_var_position, inv_sqrt_var_position, inv_sqrt_var_position;
      
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
    
  };  // NoisyPosePrior 
  
  
  /* Noisy Pose Residual
   * There is a mean pose. But several measurements of the pose are noisy measurements of that pose.
   * Both expectation as well as measurement are variable states.
   */
  class NoisyPoseResidual : public ceres::SizedCostFunction<6, 6,6> {
   public:
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
      CidposeC_ = inv_sqrt_var_vec.asDiagonal() * CidposeC_;
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
  class AprilTagVioResidual: public ceres::SizedCostFunction<8, 15,3,6,3,3,6,1,4> {
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
      bool use_first_estimates;   /**< Should we re-evaluate Jacobians at every estimate? */
      // Noise parameters
      double q_image;             /**< Stdev of u,v location of corner in the image */
      Options() {
        use_first_estimates = true;       /**< Generally we use first estimate jacobians */
        q_image = 0.5;                    /**< 1-sigma conrner location in pixels */
      }
    };
    
    // Data members
    AprilTagVioResidual::Options options_;
    
    // Observation
    const anantak::AprilTagReadingType *tag_view_;
    
    // States that this residual constrains
    anantak::ImuState *poseI_;                /**< Pose of IMU in W frame */
    anantak::Vector3dState* gravity_;         /**< Gravity vector in W frame */
    anantak::Pose3dState *poseItoC_;          /**< Camera pose in IMU body frame */
    anantak::Pose3dState *poseWtoT0_;         /**< Tag map pose in W frame */
    anantak::StaticAprilTagState *tagTj_;     /**< Pose and size of tag Tj in T0 frame */
    anantak::CameraIntrinsicsState *camera_;  /**< Camera intrinsics state */
    
    // IMU residual used to interpolate the IMU integrals
    const anantak::ImuResidualFunction *imu_constraint_;  /**< IMU residual used to interpolate */
    
    // Starting residual
    Eigen::Matrix<double,8,1> tagview_residual_;   /**< Starting residual */
    
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
      
      /* We have IqW,WqI, IqC,CpI, T0qW,WpT0, TjqT0,T0pTj, K from states
       * We have the observation of the corners
       * We have a pointer to the residual that will be used to interpolate imu integrals
       *
       * t time instants are where IMU states are established
       * i time instants are where camera images with tag views are seen
       * j is index of a tag in tag map
       * 
       * IirW,WpIi IMU state at the camera timestamp is calculated by propagation of IMU state at t
       * This is done by linear interpolation (added interpolation noise) using IMU residual.
       */
      
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
      
      /* Estimates are calculated as follows:
       *  UVij = [X/Z, Y/Z]ij
       *  XYZij = K*CipfTj
       *  CipfTj = CirT0 * ( T0pTj - T0pCi + T0rTj*Tpf )  .. for each corner feature
       *  CirT0 = CrI * IirW * WrT0
       *  T0pCi = T0rW * ( WpIi + WrIi*IpC - WpT0 )
       */
      
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
      
      /* Errors are:
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
       */
      
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
      
      /* Noises and uncertainties
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
       */
      
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
      
      /* Calculate residual from the error states. Linear model of the residual wrt error states
       * is used to recalculate the residual. */
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
      
      /* Jacobians are not recalculated with changing states. We use first estimates Jacobians
       * as this shown to keep the estimator consistent. Also makes calculations faster.*/
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
    
  }; // AprilTagVioResidual
  
  
  /* April Tag View residual
   * This constrains Camera pose, tag pose and camera matrix given a tag view */
  class AprilTagViewResidual: public ceres::SizedCostFunction<8, 6,6,1,4> {
   public:
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
    
    // Default constructor
    AprilTagViewResidual() :
      options_(),
      tag_view_(NULL),
      poseC_(NULL), tagTj_(NULL), camera_(NULL) {
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
        double z_recip = Cipfk[2];
        double x_by_z = Cipfk[0] * z_recip;
        double y_by_z = Cipfk[1] * z_recip;
        Eigen::Vector2d uv;
        uv << fx*x_by_z + cx, fy*y_by_z + cy;
        
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
  
  /** For first reading - this is not needed, we use constant parameter block instead */
  class ConvexFirstImuResidualFunction : public ceres::SizedCostFunction<7,3,3,3,3,3> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    typedef Eigen::Map<const Eigen::Vector3d> MapVector3dConstType;
    typedef Eigen::Map<Eigen::Matrix<double,7,1>> MapVector7dType;
    typedef Eigen::Map<Eigen::Matrix<double,7,3,Eigen::RowMajor>> MapMatrix7x3Type;
    
    ConvexFirstImuResidualFunction(
        const Eigen::Matrix3d *P, const Eigen::Matrix3d *V,
        const Eigen::Vector3d *y, const Eigen::Vector3d *s,
        const Eigen::Vector3d *posn0,
        const double& dt, const double& g_mag,
        const double& sigma_a, const double& sigma_g ) :
      dt_(dt), g_mag_(g_mag), dt2_by_2_(0.5*dt*dt), g_mag2_(g_mag*g_mag),
      sigma_a_(sigma_a), sigma_g_(sigma_g),
      I3_(Eigen::Matrix3d::Identity()) {
      P_ = P; V_ = V; y_ = y; s_ = s;
      posn0_ = posn0;
      Initialize();
    }
    ConvexFirstImuResidualFunction(
        const ImuReadingsIntegralType *I,
        const Eigen::Vector3d *posn0,
        const double& g_mag,
        const double& sigma_a, const double& sigma_g ) :
      dt_(I->dt), g_mag_(g_mag), dt2_by_2_(0.5*I->dt*I->dt), g_mag2_(g_mag*g_mag),
      sigma_a_(sigma_a), sigma_g_(sigma_g),
      I3_(Eigen::Matrix3d::Identity()),
      P_(&I->P), V_(&I->V), y_(&I->y), s_(&I->s),
      posn0_(posn0) {
      Initialize();
    }
    
    bool Initialize() {
      dt_I3_ = dt_ * I3_; dt2_by_2_I3_ = dt2_by_2_ * I3_;
      double sqrt_dt, sqrt_dt2by2; sqrt_dt = std::sqrt(dt_); sqrt_dt2by2 = std::sqrt(dt2_by_2_);
      double sigma_p, sigma_v; sigma_v = sigma_a_*sqrt_dt; sigma_p = sigma_a_*sqrt_dt2by2;
      inv_sigma_ << sigma_p, sigma_p, sigma_p, sigma_v, sigma_v, sigma_v, sigma_g_;
      inv_sigma_ = inv_sigma_.cwiseInverse();
      VLOG(1) << "inv_sigma = " << inv_sigma_.transpose();
      
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
      dr_dgrav_.block<3,3>(0,0) = dt2_by_2_I3_ / sigma_p;
      dr_dgrav_.block<3,3>(3,0) = dt_I3_ / sigma_v;
      //dr_dgrav_.block<1,3>(6,0) << 2.*grav.transpose()/sigma_g_; // depends on the grav estimate
      dr_dbias_.setZero();
      dr_dbias_.block<3,3>(0,0) = -(*P_) / sigma_p;
      dr_dbias_.block<3,3>(3,0) = -(*V_) / sigma_v;
      
      return true;
    }
    
    virtual ~ConvexFirstImuResidualFunction() {}
    
    virtual bool Evaluate(
        double const* const* parameters,
        double* residuals,
        double** jacobians) const {
      
      MapVector3dConstType posn1(parameters[0]);
      MapVector3dConstType velo0(parameters[1]);
      MapVector3dConstType velo1(parameters[2]);
      MapVector3dConstType  grav(parameters[3]);
      MapVector3dConstType  bias(parameters[4]);
      MapVector7dType resid(residuals);
      
      resid.block<3,1>(0,0) = (*posn0_) + dt_*velo0 + dt2_by_2_*grav + (*y_) - (*P_)*bias - posn1;
      resid.block<3,1>(3,0) = velo0 + dt_*grav + (*s_) - (*V_)*bias - velo1;
      resid(6) = grav.squaredNorm() - g_mag2_;
      resid = resid.cwiseProduct(inv_sigma_);
      
      if (jacobians != NULL) {
        if (jacobians[0] != NULL) {
          MapMatrix7x3Type dr_dposn1(jacobians[0]);
          dr_dposn1 = dr_dposn1_;
        }
        if (jacobians[1] != NULL) {
          MapMatrix7x3Type dr_dvelo0(jacobians[1]);
          dr_dvelo0 = dr_dvelo0_;
        }
        if (jacobians[2] != NULL) {
          MapMatrix7x3Type dr_dvelo1(jacobians[2]);
          dr_dvelo1 = dr_dvelo1_;
        }
        if (jacobians[3] != NULL) {
          MapMatrix7x3Type dr_dgrav(jacobians[3]);
          dr_dgrav = dr_dgrav_;
          dr_dgrav.block<1,3>(6,0) << 2.*grav.transpose()*inv_sigma_[6];
        }
        if (jacobians[4] != NULL) {
          MapMatrix7x3Type dr_dbias(jacobians[4]);
          dr_dbias = dr_dbias_;
        }
      }
      
      return true;
    } // Evaluate
    
   protected:
    const Eigen::Matrix3d *P_, *V_;   /**< V = Integral(rotation), P = Integral(V) */
    const Eigen::Vector3d *y_, *s_;   /**< s = Integral(rotated_acceleration), y = Integral(s) */
    const Eigen::Vector3d *posn0_;    /**< fixed first position */
    const double dt_, dt2_by_2_;      /**< dt = time of this interval */
    const double g_mag_, g_mag2_;     /**< Gravity magnitude and its square */
    const double sigma_a_, sigma_g_;  /**< Accelerometer noise stdev, gravity_mag^2 noise stdev */
    Eigen::Matrix3d I3_, dt_I3_, dt2_by_2_I3_;
    Eigen::Matrix<double,7,1> inv_sigma_;
    // Precalculated jacobians - in convex formulation most coefficients are constant.
    Eigen::Matrix<double,7,3> dr_dposn0_, dr_dposn1_, dr_dvelo0_, dr_dvelo1_, dr_dgrav_, dr_dbias_;
  };  // ConvexFirstImuResidualFunction

  class ConvexImuKinematicsFunction : public ceres::SizedCostFunction<7,3,3,3,3,3,3,1> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    typedef Eigen::Map<const Eigen::Vector3d> MapVector3dConstType;
    typedef Eigen::Map<Eigen::Matrix<double,7,1>> MapVector7dType;
    typedef Eigen::Map<Eigen::Matrix<double,7,3,Eigen::RowMajor>> MapMatrix7x3Type;
    
    ConvexImuKinematicsFunction(
        const Eigen::Matrix3d *P, const Eigen::Matrix3d *V,
        const Eigen::Vector3d *y, const Eigen::Vector3d *s,
        const double& dt, const double& g_mag,
        const double& sigma_a, const double& sigma_g ) :
      dt_(dt), g_mag_(g_mag), dt2_by_2_(0.5*dt*dt), g_mag2_(g_mag*g_mag),
      sigma_a_(sigma_a), sigma_g_(sigma_g),
      I3_(Eigen::Matrix3d::Identity()) {
      P_ = P; V_ = V; y_ = y; s_ = s;
      Initialize();
    }
    ConvexImuKinematicsFunction(
        const ImuReadingsIntegralType *I, const double& g_mag,
        const double& sigma_a, const double& sigma_g ) :
      dt_(I->dt), g_mag_(g_mag), dt2_by_2_(0.5*I->dt*I->dt), g_mag2_(g_mag*g_mag),
      sigma_a_(sigma_a), sigma_g_(sigma_g),
      I3_(Eigen::Matrix3d::Identity()) {
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
      dr_dgrav_.block<3,3>(0,0) = dt2_by_2_I3_ / sigma_p;
      dr_dgrav_.block<3,3>(3,0) = dt_I3_ / sigma_v;
      //dr_dgrav_.block<1,3>(6,0) << 2.*grav.transpose()/sigma_g_; // depends on the grav estimate
      dr_dbias_.setZero();
      dr_dbias_.block<3,3>(0,0) = -(*P_) / sigma_p;
      dr_dbias_.block<3,3>(3,0) = -(*V_) / sigma_v;
      dr_dmult_.setZero();
      dr_dmult_.block<3,1>(0,0) = (*y_);
      dr_dmult_.block<3,1>(3,0) = (*s_);
      return true;
    }
    
    virtual ~ConvexImuKinematicsFunction() {}
    
    virtual bool Evaluate(
        double const* const* parameters,
        double* residuals,
        double** jacobians) const {
      
      MapVector3dConstType posn0(parameters[0]);
      MapVector3dConstType posn1(parameters[1]);
      MapVector3dConstType velo0(parameters[2]);
      MapVector3dConstType velo1(parameters[3]);
      MapVector3dConstType  grav(parameters[4]);
      MapVector3dConstType  bias(parameters[5]);
      const double *mult = parameters[6];
      MapVector7dType resid(residuals);
      
      resid.block<3,1>(0,0) = posn0 + dt_*velo0 + dt2_by_2_*grav + (*mult)*(*y_) - (*P_)*bias - posn1;
      resid.block<3,1>(3,0) = velo0 + dt_*grav + (*mult)*(*s_) - (*V_)*bias - velo1;
      resid(6) = grav.squaredNorm() - g_mag2_;
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
          MapVector7dType dr_dmult(jacobians[6]);
          dr_dmult = dr_dmult_;
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
    Eigen::Matrix<double,7,1> dr_dmult_;
  };  // ConvexImuKinematicsFunction
  
  class ConvexImuKinFirstFunction : public ceres::SizedCostFunction<7,3,3,3,3,3,1> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    typedef Eigen::Map<const Eigen::Vector3d> MapVector3dConstType;
    typedef Eigen::Map<Eigen::Matrix<double,7,1>> MapVector7dType;
    typedef Eigen::Map<Eigen::Matrix<double,7,3,Eigen::RowMajor>> MapMatrix7x3Type;
    
    ConvexImuKinFirstFunction(
        const Eigen::Matrix3d *P, const Eigen::Matrix3d *V,
        const Eigen::Vector3d *y, const Eigen::Vector3d *s,
        const Eigen::Vector3d *posn0,
        const double& dt, const double& g_mag,
        const double& sigma_a, const double& sigma_g ) :
      dt_(dt), g_mag_(g_mag), dt2_by_2_(0.5*dt*dt), g_mag2_(g_mag*g_mag),
      sigma_a_(sigma_a), sigma_g_(sigma_g),
      I3_(Eigen::Matrix3d::Identity()) {
      P_ = P; V_ = V; y_ = y; s_ = s;
      posn0_ = posn0;
      Initialize();
    }
    ConvexImuKinFirstFunction(
        const ImuReadingsIntegralType *I,
        const Eigen::Vector3d *posn0,
        const double& g_mag,
        const double& sigma_a, const double& sigma_g ) :
      dt_(I->dt), g_mag_(g_mag), dt2_by_2_(0.5*I->dt*I->dt), g_mag2_(g_mag*g_mag),
      sigma_a_(sigma_a), sigma_g_(sigma_g),
      I3_(Eigen::Matrix3d::Identity()) {
      P_ = &I->P; V_ = &I->V; y_ = &I->y; s_ = &I->s;
      posn0_ = posn0;
      Initialize();
    }
    
    bool Initialize() {
      dt_I3_ = dt_ * I3_; dt2_by_2_I3_ = dt2_by_2_ * I3_;
      double sqrt_dt, sqrt_dt2by2; sqrt_dt = std::sqrt(dt_); sqrt_dt2by2 = std::sqrt(dt2_by_2_);
      double sigma_p, sigma_v; sigma_v = sigma_a_*sqrt_dt; sigma_p = sigma_a_*sqrt_dt2by2;
      inv_sigma_ << sigma_p, sigma_p, sigma_p, sigma_v, sigma_v, sigma_v, sigma_g_;
      inv_sigma_ = inv_sigma_.cwiseInverse();
      //VLOG(1) << "inv_sigma = " << inv_sigma_.transpose();
      
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
      dr_dgrav_.block<3,3>(0,0) = dt2_by_2_I3_ / sigma_p;
      dr_dgrav_.block<3,3>(3,0) = dt_I3_ / sigma_v;
      //dr_dgrav_.block<1,3>(6,0) << 2.*grav.transpose()/sigma_g_; // depends on the grav estimate
      dr_dbias_.setZero();
      dr_dbias_.block<3,3>(0,0) = -(*P_) / sigma_p;
      dr_dbias_.block<3,3>(3,0) = -(*V_) / sigma_v;
      dr_dmult_.setZero();
      dr_dmult_.block<3,1>(0,0) = (*y_);
      dr_dmult_.block<3,1>(3,0) = (*s_);
      return true;
    }
    
    virtual ~ConvexImuKinFirstFunction() {}
    
    virtual bool Evaluate(
        double const* const* parameters,
        double* residuals,
        double** jacobians) const {
      
      MapVector3dConstType posn1(parameters[0]);
      MapVector3dConstType velo0(parameters[1]);
      MapVector3dConstType velo1(parameters[2]);
      MapVector3dConstType  grav(parameters[3]);
      MapVector3dConstType  bias(parameters[4]);
      const double *mult = parameters[5];
      MapVector7dType resid(residuals);
      
      resid.block<3,1>(0,0) = (*posn0_) + dt_*velo0 + dt2_by_2_*grav + (*mult)*(*y_) - (*P_)*bias - posn1;
      resid.block<3,1>(3,0) = velo0 + dt_*grav + (*mult)*(*s_) - (*V_)*bias - velo1;
      resid(6) = grav.squaredNorm() - g_mag2_;
      resid = resid.cwiseProduct(inv_sigma_);
      
      if (jacobians != NULL) {
        if (jacobians[0] != NULL) {
          MapMatrix7x3Type dr_dposn1(jacobians[0]);
          dr_dposn1 = dr_dposn1_;
        }
        if (jacobians[1] != NULL) {
          MapMatrix7x3Type dr_dvelo0(jacobians[1]);
          dr_dvelo0 = dr_dvelo0_;
        }
        if (jacobians[2] != NULL) {
          MapMatrix7x3Type dr_dvelo1(jacobians[2]);
          dr_dvelo1 = dr_dvelo1_;
        }
        if (jacobians[3] != NULL) {
          MapMatrix7x3Type dr_dgrav(jacobians[3]);
          dr_dgrav = dr_dgrav_;
          dr_dgrav.block<1,3>(6,0) << 2.*grav.transpose()*inv_sigma_[6];
        }
        if (jacobians[4] != NULL) {
          MapMatrix7x3Type dr_dbias(jacobians[4]);
          dr_dbias = dr_dbias_;
        }
        if (jacobians[5] != NULL) {
          MapVector7dType dr_dmult(jacobians[5]);
          dr_dmult = dr_dmult_;
        }
      }
      
      return true;
    } // Evaluate
    
   protected:
    const Eigen::Matrix3d *P_, *V_;   /**< V = Integral(rotation), P = Integral(V) */
    const Eigen::Vector3d *y_, *s_;   /**< s = Integral(rotated_acceleration), y = Integral(s) */
    const Eigen::Vector3d *posn0_;    /**< fixed first position */
    const double dt_, dt2_by_2_;      /**< dt = time of this interval */
    const double g_mag_, g_mag2_;     /**< Gravity magnitude and its square */
    const double sigma_a_, sigma_g_;  /**< Accelerometer noise stdev, gravity_mag^2 noise stdev */
    Eigen::Matrix3d I3_, dt_I3_, dt2_by_2_I3_;
    Eigen::Matrix<double,7,1> inv_sigma_;
    // Precalculated jacobians - in convex formulation most coefficients are constant.
    Eigen::Matrix<double,7,3> dr_dposn0_, dr_dposn1_, dr_dvelo0_, dr_dvelo1_, dr_dgrav_, dr_dbias_;
    Eigen::Matrix<double,7,1> dr_dmult_;
  };  // ConvexImuKinFirstFunction

  // Residual for calculating gravity and accel biases when IMU is at rest
  /*class ConvexImuKinAtRestFunction : public ceres::SizedCostFunction<7,3,3> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef Eigen::Map<const Eigen::Vector3d> MapVector3dConstType;
    typedef Eigen::Map<Eigen::Matrix<double,7,1>> MapVector7dType;
    typedef Eigen::Map<Eigen::Matrix<double,7,3,Eigen::RowMajor>> MapMatrix7x3Type;
    
    ConvexImuKinAtRestFunction(
        const Eigen::Matrix3d *P, const Eigen::Matrix3d *V,
        const Eigen::Vector3d *y, const Eigen::Vector3d *s,
        const double& dt, const double& g_mag,
        const double& sigma_a, const double& sigma_g ) :
      dt_(dt), g_mag_(g_mag), dt2_by_2_(0.5*dt*dt), g_mag2_(g_mag*g_mag),
      sigma_a_(sigma_a), sigma_g_(sigma_g),
      I3_(Eigen::Matrix3d::Identity()) {
      P_ = P; V_ = V; y_ = y; s_ = s;
      Initialize();
    }
    ConvexImuKinAtRestFunction(
        const ImuReadingsIntegralType *I,
        const double& g_mag,
        const double& sigma_a, const double& sigma_g ) :
      dt_(I->dt), g_mag_(g_mag), dt2_by_2_(0.5*I->dt*I->dt), g_mag2_(g_mag*g_mag),
      sigma_a_(sigma_a), sigma_g_(sigma_g),
      I3_(Eigen::Matrix3d::Identity()) {
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
      
      // Precalculate jacobians
      dr_dgrav_.setZero();
      dr_dgrav_.block<3,3>(0,0) = dt2_by_2_I3_ / sigma_p;
      dr_dgrav_.block<3,3>(3,0) = dt_I3_ / sigma_v;
      //dr_dgrav_.block<1,3>(6,0) << 2.*grav.transpose()/sigma_g_; // depends on the grav estimate
      dr_dbias_.setZero();
      dr_dbias_.block<3,3>(0,0) = -(*P_) / sigma_p;
      dr_dbias_.block<3,3>(3,0) = -(*V_) / sigma_v;
      return true;
    }
    
    virtual ~ConvexImuKinAtRestFunction() {}
    
    virtual bool Evaluate(
        double const* const* parameters,
        double* residuals,
        double** jacobians) const {
      
      MapVector3dConstType  grav(parameters[0]);
      MapVector3dConstType  bias(parameters[1]);
      MapVector7dType resid(residuals);
      
      resid.block<3,1>(0,0) = dt2_by_2_*grav + (*y_) - (*P_)*bias;
      resid.block<3,1>(3,0) = dt_*grav + (*s_) - (*V_)*bias;
      resid(6) = grav.squaredNorm() - g_mag2_;
      resid = resid.cwiseProduct(inv_sigma_);
      
      if (jacobians != NULL) {
        if (jacobians[0] != NULL) {
          MapMatrix7x3Type dr_dgrav(jacobians[0]);
          dr_dgrav = dr_dgrav_;
          dr_dgrav.block<1,3>(6,0) << 2.*grav.transpose()*inv_sigma_[6];
        }
        if (jacobians[1] != NULL) {
          MapMatrix7x3Type dr_dbias(jacobians[1]);
          dr_dbias = dr_dbias_;
        }
      }
      
      return true;
    } // Evaluate
    
   protected:
    const Eigen::Matrix3d *P_, *V_;   // V = Integral(rotation), P = Integral(V) 
    const Eigen::Vector3d *y_, *s_;   // s = Integral(rotated_acceleration), y = Integral(s) 
    const double dt_, dt2_by_2_;      // dt = time of this interval 
    const double g_mag_, g_mag2_;     // Gravity magnitude and its square 
    const double sigma_a_, sigma_g_;  // Accelerometer noise stdev, gravity_mag^2 noise stdev 
    Eigen::Matrix3d I3_, dt_I3_, dt2_by_2_I3_;
    Eigen::Matrix<double,7,1> inv_sigma_;
    // Precalculated jacobians - in convex formulation most coefficients are constant.
    Eigen::Matrix<double,7,3> dr_dgrav_, dr_dbias_;
  }; */

  // Residual for calculating gravity and accel biases when IMU is at rest
  class ConvexImuKinAtRestFunction : public ceres::SizedCostFunction<4,3,3> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef Eigen::Map<const Eigen::Vector3d> MapVector3dConstType;
    typedef Eigen::Map<Eigen::Matrix<double,4,1>> MapVector4dType;
    typedef Eigen::Map<Eigen::Matrix<double,4,3,Eigen::RowMajor>> MapMatrix4x3Type;
    
    ConvexImuKinAtRestFunction(
        const Eigen::Matrix3d *P, const Eigen::Matrix3d *V,
        const Eigen::Vector3d *y, const Eigen::Vector3d *s,
        const double& dt, const double& g_mag,
        const double& sigma_a, const double& sigma_g ) :
      dt_(dt), g_mag_(g_mag), dt2_by_2_(0.5*dt*dt), g_mag2_(g_mag*g_mag),
      sigma_a_(sigma_a), sigma_g_(sigma_g),
      I3_(Eigen::Matrix3d::Identity()) {
      P_ = P; V_ = V; y_ = y; s_ = s;
      Initialize();
    }
    ConvexImuKinAtRestFunction(
        const ImuReadingsIntegralType *I,
        const double& g_mag,
        const double& sigma_a, const double& sigma_g ) :
      dt_(I->dt), g_mag_(g_mag), dt2_by_2_(0.5*I->dt*I->dt), g_mag2_(g_mag*g_mag),
      sigma_a_(sigma_a), sigma_g_(sigma_g),
      I3_(Eigen::Matrix3d::Identity()) {
      P_ = &I->P; V_ = &I->V; y_ = &I->y; s_ = &I->s;
      Initialize();
    }
    
    bool Initialize() {
      dt_I3_ = dt_ * I3_; dt2_by_2_I3_ = dt2_by_2_ * I3_;
      double sqrt_dt, sqrt_dt2by2; sqrt_dt = std::sqrt(dt_); sqrt_dt2by2 = std::sqrt(dt2_by_2_);
      double sigma_p, sigma_v; sigma_v = sigma_a_*sqrt_dt; sigma_p = sigma_a_*sqrt_dt2by2;
      inv_sigma_ << sigma_v, sigma_v, sigma_v, sigma_g_;
      inv_sigma_ = inv_sigma_.cwiseInverse();
      
      // Precalculate jacobians
      dr_dgrav_.setZero();
      dr_dgrav_.block<3,3>(0,0) = dt_I3_ / sigma_v;
      //dr_dgrav_.block<1,3>(3,0) << 2.*grav.transpose()/sigma_g_; // depends on the grav estimate
      dr_dbias_.setZero();
      dr_dbias_.block<3,3>(0,0) = -(*V_) / sigma_v;
      return true;
    }
    
    virtual ~ConvexImuKinAtRestFunction() {}
    
    virtual bool Evaluate(
        double const* const* parameters,
        double* residuals,
        double** jacobians) const {
      
      MapVector3dConstType  grav(parameters[0]);
      MapVector3dConstType  bias(parameters[1]);
      MapVector4dType resid(residuals);
      
      resid.block<3,1>(0,0) = dt_*grav + (*s_) - (*V_)*bias;
      resid(3) = grav.squaredNorm() - g_mag2_;
      resid = resid.cwiseProduct(inv_sigma_);
      
      if (jacobians != NULL) {
        if (jacobians[0] != NULL) {
          MapMatrix4x3Type dr_dgrav(jacobians[0]);
          dr_dgrav = dr_dgrav_;
          dr_dgrav.block<1,3>(3,0) << 2.*grav.transpose()*inv_sigma_[3];
        }
        if (jacobians[1] != NULL) {
          MapMatrix4x3Type dr_dbias(jacobians[1]);
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
    Eigen::Matrix<double,4,1> inv_sigma_;
    // Precalculated jacobians - in convex formulation most coefficients are constant.
    Eigen::Matrix<double,4,3> dr_dgrav_, dr_dbias_;
  }; // ConvexImuKinAtRestFunction
  
  // Residual for calculating IMU position wrt camera
  class FixedCameraPoseResidualFunction : public ceres::SizedCostFunction<3,3> {
   public:
    
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
      const Eigen::Vector3d* I0pIi, const double* info
    ): info_(std::sqrt(*info)), I0pIi_(*I0pIi) {}
    
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
          dr_dI0pIi = Eigen::Matrix3d::Identity();
        }
      }
      
      return true;
    } // Evaluate
    
   protected:
    double info_;
    Eigen::Vector3d I0pIi_;
  }; // ImuCamPoseResidualFunction
  
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

  
  
} // namespace anantak

using namespace anantak;


/** Extract quaternions from imu data */
bool ExtractQuaternionsFromImuMessages(const SensorMsgVectorType& msgs, QuaternionVectorType* qvec) {
  qvec->clear();
  for (int i=0; i<msgs.size(); i++) {
    // make sure that the msg has imu_msg
    if (msgs[i].has_imu_msg()) {
      QuaternionType q(double(msgs[i].imu_msg().quaternion(3)), double(msgs[i].imu_msg().quaternion(0)),
          double(msgs[i].imu_msg().quaternion(1)), double(msgs[i].imu_msg().quaternion(2)));
      q.normalize();
      qvec->push_back(q);
    } else {
      LOG(WARNING) << "MessagesVector does not have an imu_msg at index " << i;
      QuaternionType q; // unit quaternion
      q.normalize();
      qvec->push_back(q);
    }
  }
  return true;
}

/** Extract accelerations from imu data */
bool ExtractAccelerationsFromImuMessages(const SensorMsgVectorType& msgs,
    StdVectorOfVector3dType* accels   /**< Linear acceleration measurements */
    ) {
  accels->clear();
  for (int i=0; i<msgs.size(); i++) {
    // make sure that the msg has imu_msg
    if (msgs[i].has_imu_msg()) {
      Eigen::Vector3d v; v << double(msgs[i].imu_msg().linear(0)), double(msgs[i].imu_msg().linear(1)),
          double(msgs[i].imu_msg().linear(2));
      accels->push_back(v);
    } else {
      LOG(WARNING) << "MessagesVector does not have an imu_msg at index " << i;
      Eigen::Vector3d v = Eigen::Vector3d::Zero();
      accels->push_back(v);
    }
  }
  return true;
}


/** Quaterion to Euler angles */
YawPitchRollType QuaternionToEulerAngles(const QuaternionType& q) {
  Eigen::Matrix3d mat = q.toRotationMatrix();
  return mat.eulerAngles(0,1,2);
}

/** Rotation matrix to yaw pitch roll */
template <typename Derived>
YawPitchRollType RotationMatrixToYawPitchRoll(const Eigen::DenseBase<Derived>& rotmat, int solution=1) {
  struct Euler {
    double yaw;
    double pitch;
    double roll;
  };
  const double kPi = double(3.14159265358979323846);
  Euler euler_out;
  Euler euler_out2; //second solution
  // Check that pitch is not at a singularity
  if ((rotmat(2,0) >= double(1.0)) || (rotmat(2,0) <= double(-1.0))) {
    euler_out.yaw = double(0.0);
    euler_out2.yaw = double(0.0);
    // From difference of angles formula
    double delta = atan2(rotmat(0,0), rotmat(0,2));
    if (rotmat(2,0) > double(0.0)) { //gimbal locked up
      euler_out.pitch = kPi / double(2.0);
      euler_out2.pitch = kPi / double(2.0);
      euler_out.roll = euler_out.pitch + delta;
      euler_out2.roll = euler_out.pitch + delta;
    }
    else { // gimbal locked down
      euler_out.pitch = -kPi / double(2.0);
      euler_out2.pitch = -kPi / double(2.0);
      euler_out.roll = -euler_out.pitch + delta;
      euler_out2.roll = -euler_out.pitch + delta;
    }
  }
  else {
    if (solution == 1) { 
      euler_out.pitch = -asin(rotmat(2,0));
      double  cp = cos(euler_out.pitch);
      euler_out.roll  = atan2(rotmat(2,1)/cp,  rotmat(2,2)/cp);
      euler_out.yaw  = atan2(rotmat(1,0)/cp,  rotmat(0,0)/cp);
    }
    else { 
      euler_out2.pitch = kPi - euler_out.pitch;
      double  cp2 = cos(euler_out2.pitch);
      euler_out2.roll = atan2(rotmat(2,1)/cp2, rotmat(2,2)/cp2);
      euler_out2.yaw = atan2(rotmat(1,0)/cp2, rotmat(0,0)/cp2);
    }
  }
  YawPitchRollType ypr;
  if (solution == 1) { 
    ypr << euler_out.yaw, euler_out.pitch, euler_out.roll;
  }
  else { 
    ypr << euler_out2.yaw, euler_out2.pitch, euler_out2.roll;
  }
  return ypr;
}

/** Quaternion to yaw pitch roll */
YawPitchRollType QuaternionToYawPitchRoll(const QuaternionType& q) {
  Eigen::Matrix3d rotmat = q.toRotationMatrix();
  return RotationMatrixToYawPitchRoll(rotmat, 1);
}

/** Convert a vector of rotation quaternions to yaw-pitch-roll angles */
bool QuaternionVectorToYawPitchRollVector(const QuaternionVectorType& qvec,
      YawPitchRollVectorType* ypr) {
  int32_t num_quats = qvec.size();
  (*ypr).resize(num_quats, 3);
  for (int i=0; i<num_quats; i++) {
    (*ypr).row(i) = QuaternionToYawPitchRoll(qvec[i]);
  }
  return true;
}


/** Creates LinearInterpolation vector for a ts against given ref
 *  Assumes that both vectors and monotonically increasing. */
bool InterpolateTimestamps(const TimestampVecType& ref, const TimestampVecType& ts,
    LinearInterpVectorType* interp) {
  int64_t min_ref = ref.head(1)(0);
  int64_t max_ref = ref.tail(1)(0);
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

/** Interpolate a vector of quaternions using timestamps */
bool InterpolateQuaternions(const QuaternionVectorType& ref, const LinearInterpVectorType& interp,
    QuaternionVectorType* qvec) {
  int32_t num_ref = ref.size();
  QuaternionType unitq;
  qvec->clear();
  for (int i=0; i<interp.size(); i++) {
    int32_t idx = interp[i].index;
    double frac = interp[i].fraction;
    if (idx>-1 && idx<num_ref-1) {
      qvec->push_back(InterpolateQuaternionsLinearly(ref[idx], ref[idx+1], frac));
    } else {
      qvec->push_back(unitq);
    }
  }
  return true;
}

bool FindUniqueTags(const std::vector<anantak::AprilTagSightingsVectorType*>& all_sightings,
    anantak::AprilTagMap* tag_map) {
  tag_map->tag_ids.clear();
  tag_map->tag_poses.clear();
  std::vector<std::string>& unique_tags_seen = tag_map->tag_ids;
  for (int i_cam=0; i_cam<all_sightings.size(); i_cam++) {
    const anantak::AprilTagSightingsVectorType& apriltag_sightings = *(all_sightings[i_cam]);
    for (int i_st=0; i_st<apriltag_sightings.size(); i_st++) {
      auto i = std::find(unique_tags_seen.begin(), unique_tags_seen.end(),
          apriltag_sightings[i_st].tag_id);
      if (i==unique_tags_seen.end()) unique_tags_seen.push_back(apriltag_sightings[i_st].tag_id);
    }
    VLOG(1) << "Cam " << i_cam <<  ": Total number of tag sightings = " << apriltag_sightings.size()
        << " with " << unique_tags_seen.size() << " unique tags seen in all cameras.";
  }
  return true;
}


struct CollectedRotation {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  int32_t index1; int32_t index2;
  QuaternionType quaternion;
  AngleAxisType aa;
  QuaternionType matching_quaternion;
  AngleAxisType matching_aa;
};

struct CollectedTranslation {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  int32_t index1; int32_t index2;
  Eigen::Vector3d translation;
  Eigen::Vector3d matching_translation;
};

struct CollectedMotion {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  int32_t index1; int32_t index2;
  Eigen::Vector3d P0pP1;
  Eigen::Quaterniond P0qP1;
  Eigen::AngleAxisd P0aP1;
  double distance;
  double angle;
};
typedef std::vector<CollectedMotion>
    CollectedMotionsVector;
    
struct CameraPose {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  int64_t timestamp;
  Eigen::Vector3d WpC;
  Eigen::Matrix3d WrC;
  Eigen::Quaterniond WqC;
};

// This is meant to be averaged using multiple readings
struct CompositePose {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double info;   // a measure of information. Usually reciprocal of variance
  Eigen::Vector3d posn;
  Eigen::Matrix3d rotn;
  Eigen::Quaterniond rotn_q;
  Eigen::Matrix4d rotn_q_mat;
  CompositePose() {
    info = 0.;
    posn.setZero(); rotn.setIdentity(); rotn_q.setIdentity();
    rotn_q_mat.setZero(); rotn_q_mat(4,4) = 1.;
  }
  CompositePose(const CompositePose& cp) :
    info(cp.info), posn(cp.posn), rotn(cp.rotn), rotn_q(cp.rotn_q), rotn_q_mat(cp.rotn_q_mat) {}
  bool SetZero() {
    info = 0.;
    posn.setZero(); rotn.setIdentity(); rotn_q.setIdentity();
    rotn_q_mat.setZero(); rotn_q_mat(4,4) = 1.;    
  }
  // Add pose information - expensive operation as it uses the eigen solver
  bool AddInformation(const double* i0, const Eigen::Quaterniond* q0, const Eigen::Vector3d* p0) {
    double i1 = info + (*i0);
    double info_by_i1 = info/i1;
    double i0_by_i1 = (*i0)/i1;
    posn = info_by_i1*posn + i0_by_i1*(*p0);
    Eigen::Matrix4d q_mat0 = (*q0).coeffs() * (*q0).coeffs().transpose();
    rotn_q_mat = info_by_i1*rotn_q_mat + i0_by_i1*q_mat0;
    Eigen::EigenSolver<Eigen::Matrix4d> eigen_solver(rotn_q_mat);
    Eigen::Vector4d eval_real = eigen_solver.eigenvalues().real();
    Eigen::Vector4d::Index max_idx; eval_real.maxCoeff(&max_idx);
    Eigen::Vector4d evec_real = eigen_solver.eigenvectors().col(max_idx).real();
    Eigen::Quaterniond tag_avg_q(evec_real(3), evec_real(0), evec_real(1), evec_real(2));
    rotn_q = tag_avg_q;
    rotn = tag_avg_q.toRotationMatrix();
    info = i1;
    return true;
  }
};

struct CameraMessages {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  int32_t camera_num;
  std::string camera_name;
  
  // Messages
  std::vector<anantak::SensorMsg> mono_calib_msgs;
  std::vector<anantak::SensorMsg> features_msgs;
  std::vector<anantak::SensorMsg> apriltag_msgs;
  
  // Timestamps
  std::vector<int64_t> mono_calib_ts;
  std::vector<int64_t> features_ts;
  std::vector<int64_t> apriltag_ts;
  
  // Camera readings are used in reference with imu readings
  std::vector<LinearInterpolation> imu_interp_features;
  std::vector<LinearInterpolation> imu_interp_apriltag;
  std::vector<anantak::ImuReadingType> imu_interp_features_readings;
  std::vector<anantak::ImuReadingType> imu_interp_apriltag_readings;
  
  // IMU interpolated readings that exceed a threshold
  std::vector<CollectedRotation> CrI_collected_rotations;
  std::vector<CollectedTranslation> CrI_collected_translations;
  int64_t CrI_estimation_end_ts;
  int64_t CrI_estimation_interval;
  int32_t CrI_estimation_images_num;
  
  // Apriltag sightings
  double april_tag_size_;
  double sigma_tag_size_;
  double sigma_im_;
  Eigen::Matrix3d camera_K_;
  Eigen::Matrix<double,4,3> apriltag_3d_corners_;
  int32_t ending_tag_image_idx_;
  int64_t ending_tag_image_ts_;
  anantak::AprilTagMap apriltags_map_;
  std::vector<int64_t> apriltag_sightings_ts_;
  anantak::AprilTagSightingsVectorType apriltag_sightings_;
  std::vector<CompositePose> tag_poses_, camera_poses_;

  std::vector<LinearInterpolation> imu_unbiased_interp_apriltag_;
  std::vector<anantak::ImuReadingType> imu_unbiased_interp_apriltag_readings_;
  std::vector<ImuReadingsIntegralType> imu_unbiased_readings_integrals_;
  
  // Camera-to-imu rotation
  QuaternionType  CqI;   // Quaternion that will rotate a vector in I to C frame
  RotationMatType CrI;   // Matrix3d
  AngleAxisType   CaI;   // Angleaxis

  StdVectorOfMatrix3dType apriltag_WrCs; // Rotations of this camera in world ref frame using IMU readings
  
  // AprilTag sightings based solution
  Eigen::Matrix<double,3,Eigen::Dynamic> WpCs; // Positions of this camera in the world ref frame
  anantak::AprilTagMap initial_tag_map;
  double initial_sigma_im;
  
  // Motions using WpC
  std::vector<LinearInterpolation> cmnds_cam_interp;
  std::vector<CameraPose> cmnds_cam_interp_poses;
  CollectedMotionsVector initial_motions; // Motions collected from initial data
  
  // Acessors
  inline bool SetCameraNumber(int32_t num) {camera_num = num; return true;}
  inline bool SetCameraName(const std::string& name) {camera_name = name; return true;}
  
  // Load data from files
  CameraMessages(int32_t num, const std::string& mono_calib_msgs_file,
      const std::string& features_msgs_file, const std::string& apriltag_msgs_file) {
    camera_num = num;
    if (!anantak::LoadMsgsFromFile(mono_calib_msgs_file, &mono_calib_msgs))
        LOG(ERROR) << "Could not load data from " << mono_calib_msgs_file;
    if (!anantak::LoadMsgsFromFile(features_msgs_file, &features_msgs))
        LOG(ERROR) << "Could not load data from " << features_msgs_file;
    if (!anantak::LoadMsgsFromFile(apriltag_msgs_file, &apriltag_msgs))
        LOG(ERROR) << "Could not load data from " << apriltag_msgs_file;
    // Report
    VLOG(1) << "Loaded " << mono_calib_msgs.size() << " mono_calib msgs, " << features_msgs.size()
        << " features msgs, " << apriltag_msgs.size() << " apriltag msgs";
    
    // Calculate the camera matrix and store
    const anantak::MonocularPinholeCalibrationNoDistortionMsg& calib_msg =
        mono_calib_msgs.back().mono_calib_no_distort_msg();
    camera_K_ << calib_msg.focal_length(), 0.0, calib_msg.cx(),
              0.0, calib_msg.focal_length(), calib_msg.cy(),
              0.0, 0.0, 1.0;
    VLOG(1) << "Cam " << camera_num << ": Exracted camera matrix";
    
    // AprilTag sightings collector
    ending_tag_image_idx_ = -1;
    
  }
  
  // Extract timestamps from messages
  bool ExtractTimestamps() {
    anantak::ExtractTimestamps(mono_calib_msgs, &mono_calib_ts);
    anantak::ExtractTimestamps(features_msgs, &features_ts);
    anantak::ExtractTimestamps(apriltag_msgs, &apriltag_ts);
    // Report
    VLOG(1) << "Extracted " << mono_calib_ts.size() << " mono_calib ts, " << features_ts.size()
        << " features ts, " << apriltag_ts.size() << " apriltag ts";
    return true;
  } // ExtractTimestamps
  
  // Interpolate IMU readings
  bool InterpolateImuReadings(const std::vector<ImuReadingType>& ref) {
    anantak::InterpolateImuReadings(ref, features_ts,
        &imu_interp_features, &imu_interp_features_readings);
    anantak::InterpolateImuReadings(ref, apriltag_ts,
        &imu_interp_apriltag, &imu_interp_apriltag_readings);
    VLOG(1) << "Cam " << camera_num << ": Interpolated " << features_ts.size() << " features " << apriltag_ts.size()
        << " apriltag readings";
    return true;
  } // InterpolateImuReadings
  
  // Collect imu rotations that exceed a threshold using sparse features
  bool CollectImuRotationsUsingSparseFeatures(int64_t initiation_interval,
      double  minimum_rotation_threshold, int32_t minimum_number_of_rotations) {
    CrI_estimation_interval = initiation_interval;
    int64_t time_elapsed = 0;
    int32_t number_of_rotations = 0;
    int32_t last_index = 0;
    int32_t curr_index = 1;
    while ((time_elapsed<initiation_interval || number_of_rotations<minimum_number_of_rotations) &&
        curr_index<imu_interp_features_readings.size()) {
      // Check the angles between curr_index and last_index. If angle>threshold, store the indexes.
      // IMU Reading is WqI. We need I0qIi = WqI0^-1 (x) WqIi.
      QuaternionType dq = imu_interp_features_readings[last_index].quaternion.conjugate() *
          imu_interp_features_readings[curr_index].quaternion; 
      AngleAxisType aa(dq.conjugate());
      if (std::abs(aa.angle())>=minimum_rotation_threshold) {
        CollectedRotation cr;
        cr.index1 = last_index;
        cr.index2 = curr_index;
        cr.quaternion = dq;
        cr.aa = aa;
        CrI_collected_rotations.push_back(cr);
        number_of_rotations++;
        last_index = curr_index;
        curr_index++;
        VLOG(2) << "  " << "Indexes = " << cr.index1 << " " << cr.index2 << ", angle = " <<
            cr.aa.angle()*DegreesPerRadian << ", axis = " << cr.aa.axis().transpose();
      } else {
        curr_index++;
      }
      time_elapsed = imu_interp_features_readings[curr_index].timestamp -
          imu_interp_features_readings[0].timestamp;
      CrI_estimation_end_ts = imu_interp_features_readings[curr_index].timestamp;
    }
    VLOG(1) << "Cam " << camera_num << ": Collected " << CrI_collected_rotations.size()
        << " rotations spanning " << double(time_elapsed)/1e6 << " seconds with min " <<
        minimum_rotation_threshold*DegreesPerRadian << " degrees rotation";
    CrI_estimation_interval = time_elapsed;
    return true;
  } // CollectImuRotationsUsingSparseFeatures

  // Collect imu rotations that exceed a threshold using april tags.
  // An AprilTag reading is admissible only if tags were seen.
  bool CollectImuRotationsUsingAprilTags(int64_t initiation_interval,
      double  minimum_rotation_threshold, int32_t minimum_number_of_rotations) {
    CrI_estimation_interval = initiation_interval;
    int64_t time_elapsed = 0;
    int32_t number_of_rotations = 0;
    int32_t last_index = 0;
    int32_t curr_index = 1;
    // Goto a starting message with at least one tag sighting
    while (apriltag_msgs[last_index].april_msg().tag_id_size() < 1) {
      last_index++;
    }
    curr_index = last_index+1;
    while ((time_elapsed<initiation_interval || number_of_rotations<minimum_number_of_rotations) &&
        curr_index<imu_interp_apriltag_readings.size()) {
      // Check the angles between curr_index and last_index. If angle>threshold, store the indexes.
      // IMU Reading is WqI. We need I0qIi = WqI0^-1 (x) WqIi.
      QuaternionType dq = imu_interp_apriltag_readings[last_index].quaternion.conjugate() *
          imu_interp_apriltag_readings[curr_index].quaternion; 
      AngleAxisType aa(dq.conjugate());
      int32_t num_curr_tags = apriltag_msgs[curr_index].april_msg().tag_id_size();
      if (std::abs(aa.angle())>=minimum_rotation_threshold && num_curr_tags>0) {
        CollectedRotation cr;
        cr.index1 = last_index;
        cr.index2 = curr_index;
        cr.quaternion = dq;
        cr.aa = aa;
        CrI_collected_rotations.push_back(cr);
        number_of_rotations++;
        last_index = curr_index;
        curr_index++;
        VLOG(2) << "  " << "Indexes = " << cr.index1 << " " << cr.index2 << ", angle = " <<
            cr.aa.angle()*DegreesPerRadian << ", axis = " << cr.aa.axis().transpose();
      } else {
        curr_index++;
      }
      time_elapsed = imu_interp_apriltag_readings[curr_index].timestamp -
          imu_interp_apriltag_readings[0].timestamp;
      CrI_estimation_end_ts = imu_interp_apriltag_readings[curr_index].timestamp;
    }
    VLOG(1) << "Cam " << camera_num << ": Collected " << CrI_collected_rotations.size()
        << " rotations spanning " << double(time_elapsed)/1e6 << " seconds with min " <<
        minimum_rotation_threshold*DegreesPerRadian << " degrees rotation";
    CrI_estimation_interval = time_elapsed;
    return true;
  } // CollectImuRotationsUsingAprilTags
  
  // Calculate camera rotations for collected imu rotations
  bool CollectCameraRotationsUsingSparseFeatures() {
    /* Extract the vision-implied rotations using eigensolver from opengv library
     * Rotation is measured using the features points that are tracked frame to frame */
    double sparse_point_wt = 1.0;   // TODO: read from config
    const anantak::MonocularPinholeCalibrationNoDistortionMsg& calib_msg =
        mono_calib_msgs.back().mono_calib_no_distort_msg();
    // Calculate vision-implied rotations corresponding to imu-implied rotations
    for (int i_rotn=0; i_rotn<CrI_collected_rotations.size(); i_rotn++) {
      /* Integrate rotations from index1 to index2 seen by the camera */
      opengv::rotation_t integrated_rotation = opengv::rotation_t::Identity();
      for (int idx=CrI_collected_rotations[i_rotn].index1+1;
          idx<CrI_collected_rotations[i_rotn].index2+1; idx++) {
        if (!features_msgs[idx].has_mono_sparse_points_msg()) {
          LOG(ERROR) << "No sparse points message was found at idx " << idx;
          continue;
        }
        const anantak::MonocularSparsePointsMsg& sparse_points_msg =
            features_msgs[idx].mono_sparse_points_msg();
        VLOG(3) << "Number of sparse points in " << idx << " msg " << sparse_points_msg.u_curr_size();
        opengv::bearingVectors_t bearing_vecs0, bearing_vecs1;
        std::vector<double> wts;
        for (int i=0; i<sparse_points_msg.u_curr_size(); i++) {
          opengv::bearingVector_t bv0, bv1;
          bv0 << sparse_points_msg.u_prev(i) - calib_msg.cx(),
              sparse_points_msg.v_prev(i) - calib_msg.cy(), calib_msg.focal_length();
          bv1 << sparse_points_msg.u_curr(i) - calib_msg.cx(),
              sparse_points_msg.v_curr(i) - calib_msg.cy(), calib_msg.focal_length();
          bv0 /= bv0.norm();
          bv1 /= bv1.norm();
          bearing_vecs0.push_back(bv0);
          bearing_vecs1.push_back(bv1);
          wts.push_back(sparse_point_wt);
        }
        opengv::relative_pose::CentralRelativeWeightingAdapter adapter(
            bearing_vecs0, bearing_vecs1, wts);
        opengv::rotation_t vision_rotation = opengv::relative_pose::eigensolver(adapter);
        // C0rC2 = C0rC1 * C1rC2
        integrated_rotation *= vision_rotation;
      } // for idx
      QuaternionType dq_match(integrated_rotation); 
      AngleAxisType aa_match(dq_match);
      CrI_collected_rotations[i_rotn].matching_quaternion = dq_match;
      CrI_collected_rotations[i_rotn].matching_aa = aa_match;
      const CollectedRotation& cr = CrI_collected_rotations[i_rotn];
      VLOG(2) << "  " << "Indexes = " << cr.index1 << " " << cr.index2 << ", angle = " <<
          cr.matching_aa.angle()*DegreesPerRadian << ", axis = " << cr.matching_aa.axis().transpose();    
    } // for i_rotn
    return true;
  } // CollectCameraRotations
  
  // Estimate camera-to-imu rotation using Sparse Features
  bool EstimateCameraToImuRotationUsingSparseFeatures() {
    /* Solve the AX=XB problem (Hand-Eye calibration) for rotations collected from imu and vision */
    /* We build an over-constrained system of equations using the matrices and solve using SVD */
    int32_t num_blocks = CrI_collected_rotations.size();
    Eigen::Matrix<double, Eigen::Dynamic, 4> svd_mat;
    svd_mat.resize(4*num_blocks, 4);
    for (int i=0; i<num_blocks; i++) {
      Eigen::Matrix4d mat;
      mat = LeftMultiplicationMatrix(CrI_collected_rotations[i].matching_quaternion)  // A matrix
          - RightMultiplicationMatrix(CrI_collected_rotations[i].quaternion);         // B matrix
      svd_mat.block<4,4>(i*4, 0) = mat;
    }
    VLOG(3) << "SVD mat = \n" << svd_mat;
    Eigen::JacobiSVD<Eigen::Matrix<double, Eigen::Dynamic, 4>>
        svd(svd_mat, Eigen::ComputeThinU | Eigen::ComputeThinV);
    VLOG(3) << "Singular values are:\n" << svd.singularValues();
    //VLOG(2) << "Left singular vectors are the columns of U matrix:\n" << svd.matrixU();
    VLOG(3) << "Right singular vectors are the columns of V matrix:\n" << svd.matrixV();
    Eigen::Vector4d ciq_vec = svd.matrixV().col(3);
    //VLOG(1) << "CqI = " << ciq_vec.transpose() << " norm = " << ciq_vec.norm();
    QuaternionType ciq(ciq_vec[3], ciq_vec[0], ciq_vec[1], ciq_vec[2]);
    CqI = ciq;
    //VLOG(1) << "CqI = " << CqI.coeffs().transpose();
    CrI = CqI.toRotationMatrix();
    AngleAxisType ciaa(CqI);
    CaI = ciaa;
    VLOG(1) << "Cam " << camera_num << ": CaI initial estimate  = " << ciaa.axis().transpose()
        << " angle = " << ciaa.angle()*DegreesPerRadian;
    
    // Covariance matrix of the errors in estimate
    /*svd_mat *= RightMultiplicationMatrix(ciq_vec);
    for (int i=0; i<num_blocks; i++) {
      Eigen::JacobiSVD<Eigen::Matrix<double, Eigen::Dynamic, 4>>
          svd2(svd_mat.block<4,4>(i*4,0), Eigen::ComputeThinU | Eigen::ComputeThinV);
      Eigen::Vector4d q_vec = svd2.matrixV().col(3);
      //VLOG(1) << "resid " << i << ": " << q_vec.transpose();
      QuaternionType q_q(q_vec[3], q_vec[0], q_vec[1], q_vec[2]);
      AngleAxisType q_a(q_q);
      VLOG(1) << "resid " << i << ": " << q_vec.transpose() << ", " << q_a.axis().transpose() << ", "
          << q_a.angle()*DegreesPerRadian;
    }
    Eigen::Matrix<double, Eigen::Dynamic, 1> resid_vec;
    resid_vec.resize(4*num_blocks, 1);
    Eigen::Matrix4d mat;
    resid_vec = svd_mat * ciq_vec;
    Eigen::Map<Eigen::Matrix<double,4,Eigen::Dynamic>> resid_mat(resid_vec.data(),4,num_blocks);
    Eigen::Matrix<double,4,4> resid_covar = 1.0/double(num_blocks-1)*(resid_mat * resid_mat.transpose());
    VLOG(1) << "Residuals = \n" << resid_mat.block<4,10>(0,0);
    VLOG(1) << "Covar matrix of CqI residuals = \n" << resid_covar;*/
    // Checking the CqI
    if (false) {  
      double q1_tot_angle = 0.0;
      double q2_tot_angle = 0.0;
      for (int i=0; i<CrI_collected_rotations.size(); i++) {
        // C0qC1 * CqI * I0qI1^-1 * CqI^-1 = C0qC0 = Identity
        QuaternionType q1 = CrI_collected_rotations[i].matching_quaternion
            * CqI
            * CrI_collected_rotations[i].quaternion.conjugate()
            * CqI.conjugate();
        QuaternionType q2 = CrI_collected_rotations[i].matching_quaternion
            * CqI.conjugate()
            * CrI_collected_rotations[i].quaternion.conjugate()
            * CqI;
        
        // C0rC1 * CrI * I1rI0 * IrC
        AngleAxisType a1(q1);
        AngleAxisType a2(q2);
        q1_tot_angle += a1.angle()<Pi ? a1.angle() : Pi_2-a1.angle();
        q2_tot_angle += a2.angle()<Pi ? a2.angle() : Pi_2-a2.angle();
        VLOG(2) << i << " " << a1.angle()*DegreesPerRadian << "   " << a2.angle()*DegreesPerRadian <<
            "  ( " << CrI_collected_rotations[i].matching_aa.angle()*DegreesPerRadian << " " <<
            CrI_collected_rotations[i].aa.angle()*DegreesPerRadian << " " << CaI.angle()*DegreesPerRadian
            << " ) ";
      }
      // We expect q2_tot_angle < q1_tot_angle if not raise warning
      if (q2_tot_angle >= q1_tot_angle) {
        LOG(INFO) << "CrI calculation checks alright";
      } else {
        // I can not explain why what we get here is actually IrC, not CrI. So we invert.
        LOG(INFO) << "CrI calculation actually gives IrC? Transposing. Need to explain this.";
        CqI = CqI.conjugate();
        CrI = CqI.toRotationMatrix();
        AngleAxisType ciaa(CqI);
        CaI = ciaa;
      }
    }
  }
  
  // Calculate WrCs using IMU readings - typically used in convex formulation
  bool CalculateAprilTagWrCsUsingImuReadings() {
    int32_t num_pose_estimates = apriltag_ts.size();
    apriltag_WrCs.resize(num_pose_estimates);
    // WrC = WrI * IrC - WrI was interpolated from IMU readings and IrC was estimated
    for (int i=0; i<num_pose_estimates; i++)
      apriltag_WrCs[i] = imu_interp_apriltag_readings[i].quaternion.toRotationMatrix()
          * CrI.transpose();
    VLOG(1) << "Cam " << camera_num << ": Calculated " << num_pose_estimates << " WrCs";
  }
  
  // Collect AprilTag Sightings from an image, update apriltags_map_ 
  bool CollectAprilTagSightingsFromImage(int32_t i_msg) {
    // Check if AprilTag message exists
    if (!apriltag_msgs[i_msg].has_april_msg()) {
      LOG(ERROR) << "No AprilTag message was found at idx " << i_msg;
      return false;
    }
    // Check if any AprilTags were seen
    const anantak::MonocularPinholeCalibrationNoDistortionMsg& calib_msg =
        mono_calib_msgs.back().mono_calib_no_distort_msg();
    const anantak::AprilTagMessage& apriltag_msg = apriltag_msgs[i_msg].april_msg();
    VLOG(3) << "Number of AprilTags in msg " << i_msg << " = " << apriltag_msg.tag_id_size();
    if (apriltag_msg.tag_id_size()>0) {
      for (int i_tag=0; i_tag<apriltag_msg.tag_id_size(); i_tag++) {
        // Solve for camera pose wrt AprilTag
        opengv::bearingVectors_t bearing_vecs;
        opengv::points_t points_vec;
        { // Create observation vectors from corners
          opengv::bearingVector_t bv1, bv2, bv3, bv4;
          bv1 << apriltag_msg.u_1(i_tag) - calib_msg.cx(), apriltag_msg.v_1(i_tag) - calib_msg.cy(),
              calib_msg.focal_length();
          bv2 << apriltag_msg.u_2(i_tag) - calib_msg.cx(), apriltag_msg.v_2(i_tag) - calib_msg.cy(),
              calib_msg.focal_length();
          bv3 << apriltag_msg.u_3(i_tag) - calib_msg.cx(), apriltag_msg.v_3(i_tag) - calib_msg.cy(),
              calib_msg.focal_length();
          bv4 << apriltag_msg.u_4(i_tag) - calib_msg.cx(), apriltag_msg.v_4(i_tag) - calib_msg.cy(),
              calib_msg.focal_length();
          bv1 /= bv1.norm();
          bv2 /= bv2.norm();
          bv3 /= bv3.norm();
          bv4 /= bv4.norm();
          bearing_vecs.push_back(bv1);
          bearing_vecs.push_back(bv2);
          bearing_vecs.push_back(bv3);
          bearing_vecs.push_back(bv4);
          opengv::point_t pnt1 = apriltag_3d_corners_.row(0);
          opengv::point_t pnt2 = apriltag_3d_corners_.row(1);
          opengv::point_t pnt3 = apriltag_3d_corners_.row(2);
          opengv::point_t pnt4 = apriltag_3d_corners_.row(3);
          points_vec.push_back(pnt1);
          points_vec.push_back(pnt2);
          points_vec.push_back(pnt3);
          points_vec.push_back(pnt4);
        }
        opengv::absolute_pose::CentralAbsoluteAdapter adapter(bearing_vecs, points_vec);
        opengv::transformations_t pnp_transformations = opengv::absolute_pose::p3p_kneip(adapter);
        /*VLOG(1) << "tag = " << apriltag_msg.tag_id(i_tag)
            << " n_tfmtns = " << pnp_transformations.size();
        VLOG(3) << "corners = " << points_vec[0].transpose() << ", " << points_vec[1].transpose()
            << ", " << points_vec[2].transpose() << ", " << points_vec[3].transpose();*/
        Eigen::Matrix<double, 4, 2> corners_in_image;
        corners_in_image << 
            apriltag_msg.u_1(i_tag), apriltag_msg.v_1(i_tag),
            apriltag_msg.u_2(i_tag), apriltag_msg.v_2(i_tag),
            apriltag_msg.u_3(i_tag), apriltag_msg.v_3(i_tag),
            apriltag_msg.u_4(i_tag), apriltag_msg.v_4(i_tag);
        VLOG(3) << "corners = /n" << corners_in_image;
        Eigen::Vector4d dffs;
        Eigen::Matrix<double,3,16> Cpfs; // corner 3d coordinates in camera frame for 4 guesses
        for (int i=0; i<pnp_transformations.size(); i++) {
          //VLOG(1) << "\n" << pnp_transformations[i];
          Eigen::Matrix3d pnp_rotn = pnp_transformations[i].block<3,3>(0,0);
          Eigen::Vector3d pnp_tran = pnp_transformations[i].col(3);
          // Calculate reprojection error
          double total_dff = 0.0;
          for (int j=0; j<4; j++) {
            Eigen::Vector3d c = pnp_rotn.transpose() * (apriltag_3d_corners_.row(j).transpose()
                                - pnp_tran);
            //VLOG(1) << "c = " << c.transpose();
            Eigen::Vector3d Kc = camera_K_*c;
            if (i<4 && j<4) {
              // store Kc (=Cpf) in Cpfs matrix
              Cpfs.block<3,4>(0,4*i).col(j) = Kc;
            }
            //VLOG(1) << "Kc = " << Kc.transpose();
            Eigen::Vector2d Kcn; Kcn << Kc(0)/Kc(2), Kc(1)/Kc(2);
            Eigen::Vector2d dff = Kcn - corners_in_image.row(j).transpose();
            //VLOG(1) << "Kcn = " << Kcn.transpose() << " dff = " << dffs[j];
            total_dff += dff.squaredNorm();
          }
          dffs[i] = total_dff;
        } // for all transformations
        //VLOG(1) << dffs.transpose();
        Eigen::Vector4d::Index min_idx; dffs.minCoeff(&min_idx);
        //VLOG(1) << "Min transformation at " << min_idx;
        AprilTagSighting sighting;
        sighting.tag_id = apriltag_msg.tag_id(i_tag);
        sighting.image_coords = corners_in_image.transpose();
        sighting.reproj_error = dffs[min_idx];
        sighting.TrC = pnp_transformations[min_idx].block<3,3>(0,0);
        sighting.TpC = pnp_transformations[min_idx].col(3);
        QuaternionType q_TqC(sighting.TrC);
        sighting.TqC = q_TqC;
        sighting.Cpf = Cpfs.block<3,4>(0,4*min_idx);
        sighting.cam_K = camera_K_;
        sighting.tag_size = april_tag_size_;
        // IMU readings are WqI.
        RotationMatType WrI(imu_interp_apriltag_readings[i_msg].quaternion);
        // Calculate TrW = TrC * CrI * IrW. TrC, CrI were estimated. IrW = WrI^-1
        sighting.TrW = sighting.TrC * CrI * WrI.transpose();
        QuaternionType q(sighting.TrW);
        sighting.TqW = q.coeffs();
        sighting.image_idx = i_msg;
        apriltag_sightings_.push_back(sighting);
        apriltags_map_.AddTag(sighting.tag_id, april_tag_size_);
        VLOG(2) << "Tag " << sighting.tag_id << " Transform = (" << sighting.reproj_error << ")\n"
            << sighting.TrC << "\n" << sighting.TpC;
      } // for each tag
    } // if n_tags > 0
    return true;
  }
  
  // Set the size of tags in the environment
  bool SetAprilTagsSize(const double& tag_size, const double& sigma_tag_size, const double& sigma_im) {
    april_tag_size_ = tag_size; // meters
    sigma_tag_size_ = sigma_tag_size; // meters
    sigma_im_ = sigma_im;
    apriltag_3d_corners_ = anantak::AprilTagCorners3d(april_tag_size_);
    VLOG(1) << "Cam " << camera_num << ": Setting tag size for all tags = " << april_tag_size_
        << "[" << sigma_tag_size_ << "](m)" << " sigma_im = " << sigma_im_ << "(px)";
  }
  
  // Collect tag sightings in next time interval - call reapeatedly to keep moving forward
  bool CollectAprilTagSightings(const int64_t& interval) {
    /* Collect tag sightings from images in the given interval starting from current image */
    int64_t end_ts;
    if (ending_tag_image_idx_<0) end_ts = apriltag_ts[0] + interval;
        else end_ts = apriltag_ts[ending_tag_image_idx_] + interval;
    int32_t num_msgs = 0;
    while (apriltag_ts[ending_tag_image_idx_] <= end_ts) {
      ending_tag_image_idx_++;
      num_msgs++;
      CollectAprilTagSightingsFromImage(ending_tag_image_idx_);
      apriltag_sightings_ts_.push_back(apriltag_ts[ending_tag_image_idx_]);
    } // while each msg
    ending_tag_image_ts_ = apriltag_ts[ending_tag_image_idx_];
    VLOG(1) << "Cam" << camera_num << ": Processed " << num_msgs << " tag images seen in "
        << interval*1e-6 << "(s) with " << apriltags_map_.tag_ids.size() << " unique tags";
    return true;
  }
  
  // Estimate Camera poses wrt tags using tag sightings only
  bool EstimateMotionUsingTagSightingsOnly(const std::string& save_filename) {
    
    /* Find sets of connected tags.
     * Initiate tag and camera poses in the connected tags.
     * Optimize using reprojection error. */
    
    /* At a time, we have a sets of tag_ids. Each set has tag_ids with no overlaps.
     * For a new set of tag_ids in an image, check where each tag_id belongs. If it is found,
     * mark all images with this set number, add the new tag_ids to the set. If it is not found,
     * add 1 to max set number and mark all tag_ids in image with this. Move to next tag_id. If no
     * more tag_ids are left and number is max number, initiate a new set with images seen here. */
    
    std::vector<std::vector<std::string>> connected_tags;
    for (int i_msg=0; i_msg<ending_tag_image_idx_; i_msg++) {
    //for (int i_msg=0; i_msg<5; i_msg++) {
      bool tags_assigned = false;
      int32_t tag_num = 0;
      const anantak::AprilTagMessage& msg = apriltag_msgs[i_msg].april_msg();
      if (msg.tag_id_size()==0) continue;
      while (!tags_assigned) {
        const std::string& tag_id = msg.tag_id(tag_num);
        //VLOG(1) << "  Looking for " << tag_id << " in " << connected_tags.size() << " sets";
        // find this tag_id in the connected tags
        bool tag_found = false;
        int32_t i_ctags = 0;
        while ((!tag_found) && (i_ctags<connected_tags.size())) {
          auto itrtr = std::find(connected_tags[i_ctags].begin(), connected_tags[i_ctags].end(),
              tag_id);
          tag_found = (itrtr!=connected_tags[i_ctags].end());
          i_ctags++;
        } // while not found
        i_ctags--;
        //VLOG(1) << "    Found tag = " << tag_found << " in set " << i_ctags;
        if (tag_found) {
          // All tags in this image are added to this set
          for (int i_tag=0; i_tag<msg.tag_id_size(); i_tag++) {
            auto itrtr = std::find(connected_tags[i_ctags].begin(), connected_tags[i_ctags].end(),
                msg.tag_id(i_tag));
            if (itrtr==connected_tags[i_ctags].end())
                connected_tags[i_ctags].push_back(msg.tag_id(i_tag));
            //VLOG(1) << "    Added all tags to current set " << i_ctags;
          }          
          tags_assigned = true;
        } else {
          // if this is the last tag_id add a new set
          if (tag_num == msg.tag_id_size()-1) {
            std::vector<std::string> new_set;
            for (int i_tag=0; i_tag<msg.tag_id_size(); i_tag++) {
              new_set.push_back(msg.tag_id(i_tag));
            }
            connected_tags.push_back(new_set);
            tags_assigned = true;
            //VLOG(1) << "    Added all tags to new set " << connected_tags.size();
          }
          // else continue looking
          else {
            tag_num++;
          }
          
        } // tag found
      } // tags assigned
    } // for
    VLOG(1) << "Cam " << camera_num << ": Found " << connected_tags.size() << " connected tag sets";
    
    /* Allocate memory for camera poses and tag poses.
     * Initiate information as 1e14 for tag0, and 0 for all other poses.
     * We focus on the first set. Go through each sighting, check if it is in the first set.
     * Each sighting is a relationship between an image and a tag. Using the max(1,reproj_err)*z_est
     * as the error estimate we add this information to the pose. Pose addition happens as follows:
     *  let w=1/err; posn = (w*posn + w0*posn0)/(w+w0);
     *  rotn_q = eigen_vec_for_max_eigne_val(w*rotn_q*rotn_qT + w0*rotn_q0*rotn_q0T) */
    
    size_t tags_set_num = 0;
    size_t num_tags = connected_tags[tags_set_num].size();
    size_t num_cams = ending_tag_image_idx_+1;
    size_t num_stngs = apriltag_sightings_.size();
    // Check
    if (num_tags == 0) {
      LOG(ERROR) << "Cam " << camera_num << ": No tags were detected. Can not continue";
      return false;
    }
    // Allocate memory
    VLOG(1) << "Cam " << camera_num << ": Allocating memory for " << num_tags << " tags and "
        << num_cams << " camera poses";
    const CompositePose zero_pose;
    tag_poses_.resize(num_tags, zero_pose);
    camera_poses_.resize(num_cams, zero_pose);
    
    // Set the first tag as known position
    const int32_t reference_tag = 0;
    tag_poses_[reference_tag].info = 1e14; // very large amount of information
    
    // A holder for marking if the sighting has been used
    std::vector<bool> sighting_used;
    sighting_used.resize(num_stngs, false);
    const int32_t max_loops = 10;
    bool sightings_left = true;
    int32_t loop_num = 0;
    
    // Mark invalid sightings as used so that they are bypassed in the calculations
    for (int i_stng=0; i_stng<num_stngs; i_stng++) {
      sighting_used[i_stng] = !apriltag_sightings_[i_stng].IsValid();
    }
    
    // Go through each sighting and add its information to the poses
    while (sightings_left && loop_num<max_loops) {
      for (int i_stng=0; i_stng<num_stngs; i_stng++) {
      //for (int i_stng=0; i_stng<500; i_stng++) {
        if (!sighting_used[i_stng]) {
          const anantak::AprilTagSighting& sighting = apriltag_sightings_[i_stng];
          const std::string& tag_id = sighting.tag_id;
          // Make sure that tag id seen belongs to this set
          auto itrtr = std::find(connected_tags[tags_set_num].begin(), connected_tags[tags_set_num].end(),
              tag_id);
          bool tag_found = (itrtr!=connected_tags[tags_set_num].end());
          if (tag_found) {
            int32_t i_cam, i_tag;
            i_cam = sighting.image_idx;
            i_tag = std::distance(connected_tags[tags_set_num].begin(), itrtr);
            //VLOG(1) << "  tag, cam info = " << tag_poses_[i_tag].info << " " << camera_poses_[i_cam].info;
            // Has either the camera or tag for this sighting been initiated?
            // Heuristic based information
            double new_info = 1./(std::max(1.,sighting.reproj_error)*sighting.TpC[2]);
            CompositePose& tag_pose = tag_poses_[i_tag];
            CompositePose& cam_pose = camera_poses_[i_cam];
            // Update camera position and rotation
            if (tag_pose.info>Epsilon) {
            //if (tag_pose.info>Epsilon && cam_pose.info<Epsilon) {
              double info = std::min(new_info, tag_pose.info);
              // Tag's rotation is T0rTi. We have TirCj. We need T0rCj = T0rTi * TirCj
              Eigen::Quaterniond T0qCj = tag_pose.rotn_q * sighting.TqC;
              // T0pCj = T0pTi + T0rTi*TipCj
              Eigen::Vector3d T0pCj = tag_pose.posn + tag_pose.rotn * sighting.TpC;
              // Update with new info
              cam_pose.AddInformation(&info, &T0qCj, &T0pCj);
              //VLOG(1) << "  cam info is now " << cam_pose.info;
              sighting_used[i_stng] = true;
            }
            // Update tag position and rotation 
            if (cam_pose.info>Epsilon && i_tag!=reference_tag) {
            //if (cam_pose.info>Epsilon && tag_pose.info<Epsilon) {
              double info = std::min(new_info, cam_pose.info);
              // Camera's rotation is T0rCj. We saw TirCj. We need T0rTi = T0rCj * TirCj^-1
              Eigen::Quaterniond T0qTi = cam_pose.rotn_q * sighting.TqC.conjugate();
              // T0pTi = T0pCj - T0rCj*  CjrTi * TipCj
              Eigen::Vector3d T0pTi = cam_pose.posn
                  - cam_pose.rotn * sighting.TrC.transpose() * sighting.TpC;
              // Update with new info
              tag_pose.AddInformation(&info, &T0qTi, &T0pTi);
              //VLOG(1) << "  tag info is now " << tag_pose.info << " from " << info;
              sighting_used[i_stng] = true;
            }
          } // tag found
          else {
            sighting_used[i_stng] = true;
          }
        } // if stng is not used yet
      } // for
      loop_num++;
      int32_t num_stngs_left = 0;
      for (int i_stng=0; i_stng<num_stngs; i_stng++) {
        if (!sighting_used[i_stng]) num_stngs_left++;
      }
      VLOG(1) <<  "Cam " << camera_num << ": " << num_stngs_left << " sightings left after pass "
          << loop_num;
      sightings_left = (num_stngs_left>0);
    }  // while more sightings are left
    
    // Report the results after initiation
    { // Number of cameras left from intialization as no tag was seen
      int32_t cams_left = 0;
      for (int i_cam=0; i_cam<num_cams; i_cam++) if (camera_poses_[i_cam].info<Epsilon) cams_left++;
      VLOG(1) << "Cam " << camera_num << ": Number of cameras left as no tag was seen = "
          << cams_left << " of " << num_cams;
    }    
    { // Save tag and camera poses for plotting
      Eigen::Matrix<double,3,Eigen::Dynamic> tps;
      tps.resize(3,num_tags);
      for (int i_tag=0; i_tag<num_tags; i_tag++) {
        tps.col(i_tag) = tag_poses_[i_tag].posn;
        AngleAxisType aa(tag_poses_[i_tag].rotn_q);
        VLOG(1) << "  Tag " << connected_tags[tags_set_num][i_tag] << ": " << aa.axis().transpose()
            << ", " << aa.angle()*DegreesPerRadian;
      }
      anantak::WriteMatrixToCSVFile(save_filename+".tagposns.init", tps.transpose());
      // Save camera poses for plotting
      Eigen::Matrix<double,3,Eigen::Dynamic> cps;
      cps.resize(3,num_cams);
      for (int i=0; i<num_cams; i++) cps.col(i) = camera_poses_[i].posn;
      anantak::WriteMatrixToCSVFile(save_filename+".camposns.init", cps.transpose());
    }
    
    return true;
  } // EstimateMotionUsingTagSightingsOnly
  
  // Interpolate unbiased IMU readings for AprilTag sightings only
  bool InterpolateUnbiasedImuReadings(const std::vector<ImuReadingType>& ref) {
    anantak::InterpolateImuReadings(ref, apriltag_sightings_ts_,
        &imu_unbiased_interp_apriltag_, &imu_unbiased_interp_apriltag_readings_);
    VLOG(1) << "Cam " << camera_num << ": Interpolated " << apriltag_sightings_ts_.size()
        << " apriltag sighting readings";
    return true;
  } // InterpolateUnbiasedImuReadings
  
  // Integrate the unbiased acceleration and rotation imu readings to get imu motion vectors
  bool IntegrateUnbiasedImuReadings(const std::vector<ImuReadingType>& imu_unbiased_readings,
      std::string save_filename = "") {
    
    // Collect rotations and translations
    int32_t num_rotations = CrI_collected_rotations.size();
    CrI_collected_translations.resize(num_rotations);
    imu_unbiased_readings_integrals_.resize(num_rotations);  // allocate memory
    std::vector<bool> usable_block; usable_block.resize(num_rotations, false);
    std::vector<double> block_info; block_info.resize(num_rotations, 0.); double total_info = 0.;
    opengv::bearingVectors_t bearing_vecs, matching_bearing_vecs;
    
    int32_t num_blocks = 0;
    for (int i_rotn=0; i_rotn<num_rotations; i_rotn++) {
      CollectedRotation& coll_rotn = CrI_collected_rotations[i_rotn];
      CollectedTranslation& coll_trans = CrI_collected_translations[i_rotn];
      
      // We can use this rotation only if a camera pose was estimated at both ends
      if (camera_poses_[coll_rotn.index1].info>Epsilon && camera_poses_[coll_rotn.index2].info>Epsilon) {
        
        // Collect rotation
        // We have readings of T0qCj. We need CjqCj+1 = T0qCj^-1 * T0qCj+1
        coll_rotn.matching_quaternion = camera_poses_[coll_rotn.index1].rotn_q.conjugate()
            * camera_poses_[coll_rotn.index2].rotn_q;
        
        // Collect camera translation
        // CjpCj+1 = T0rCj^-1 * (T0pCj+1 - T0pCj)
        coll_trans.matching_translation = camera_poses_[coll_rotn.index1].rotn.transpose()
            * (camera_poses_[coll_rotn.index2].posn - camera_poses_[coll_rotn.index1].posn);
        
        // Collect imu translation
        // Imu integral y = WrI0 * I0pI1. We need I0pI1 = WrI0^-1 * y
        IntegrateImuKinematics(
          imu_unbiased_readings,
          imu_unbiased_interp_apriltag_readings_[coll_rotn.index1],
          imu_unbiased_interp_apriltag_readings_[coll_rotn.index2],
          imu_unbiased_interp_apriltag_[coll_rotn.index1].index+1,
          imu_unbiased_interp_apriltag_[coll_rotn.index2].index,
          1.0,
          &imu_unbiased_readings_integrals_[i_rotn]
        );
        /*coll_trans.translation =
          imu_unbiased_interp_apriltag_readings_[coll_rotn.index1].quaternion.toRotationMatrix().transpose()
          * imu_unbiased_readings_integrals_[i_rotn].y;*/
        coll_trans.translation = imu_unbiased_readings_integrals_[i_rotn].y;
        
        // Housekeeping
        num_blocks++;
        usable_block[i_rotn] = true;
        block_info[i_rotn] = std::min(camera_poses_[coll_rotn.index1].info,
            camera_poses_[coll_rotn.index2].info);
        total_info += block_info[i_rotn];
        
        // Bearing vectors
        opengv::bearingVector_t bv0, m_bv0, bv1, m_bv1;
        AngleAxisType aa0(coll_rotn.quaternion); bv0 = aa0.axis();
        AngleAxisType m_aa0(coll_rotn.matching_quaternion); m_bv0 = m_aa0.axis();
        // Angle is usually small, so we can orient axes by measuring the angle
        if ((aa0.angle()<Pi && m_aa0.angle()>Pi) || (aa0.angle()>Pi && m_aa0.angle()<Pi))
            m_bv0 *= -1.;
        bv1 = coll_trans.translation; bv1 /= bv1.norm();
        m_bv1 = coll_trans.matching_translation; m_bv1 /= m_bv1.norm();
        bearing_vecs.push_back(bv0); matching_bearing_vecs.push_back(m_bv0); 
        //bearing_vecs.push_back(bv1); matching_bearing_vecs.push_back(m_bv1);
        
      } // if both ends of camera poses are available
    }
    VLOG(1) << "Cam " << camera_num << ": " << num_blocks << " collected rotations are useful "
        << "out of " << num_rotations << " for hand-eye calibration";
    
    // We use Kneip's eigensolver to calculate relative rotation between two frames.
    opengv::relative_pose::CentralRelativeAdapter adapter(bearing_vecs, matching_bearing_vecs);
    opengv::rotation_t CrI_rotn = opengv::relative_pose::eigensolver(adapter);
    AngleAxisType aa_rotn(CrI_rotn);
    VLOG(1) << "Cam " << camera_num << ": bv estimate axis = " << aa_rotn.axis().transpose()
        << " angle = " << aa_rotn.angle()*DegreesPerRadian;    
    
    // Using Hand-eye calibration using SVD for computing the rotation quaternion
    Eigen::Matrix<double, Eigen::Dynamic, 4> svd_mat;
    svd_mat.resize(4*num_blocks, 4);
    int32_t i_block = 0;
    for (int i_rotn=0; i_rotn<num_rotations; i_rotn++) {
      if (usable_block[i_rotn]) {
        CollectedRotation& coll_rotn = CrI_collected_rotations[i_rotn];
        CollectedTranslation& coll_trans = CrI_collected_translations[i_rotn];
        Eigen::Matrix4d mat;
        svd_mat.block<4,4>(i_block*4, 0) =
            LeftMultiplicationMatrix(coll_rotn.matching_quaternion)    // A matrix
            - RightMultiplicationMatrix(coll_rotn.quaternion);         // B matrix
        /*Eigen::Vector4d v, m_v;
        v << coll_trans.translation, 0.; m_v << coll_trans.matching_translation, 0.;
        svd_mat.block<4,4>(i_block*4, 0) =
            LeftMultiplicationMatrix(v)       // A matrix
            - RightMultiplicationMatrix(m_v);     // B matrix*/
        i_block++;
      } // if usable block
    } // for i_rotn
    VLOG(3) << "SVD mat = \n" << svd_mat;
    Eigen::JacobiSVD<Eigen::Matrix<double, Eigen::Dynamic, 4>>
        svd(svd_mat, Eigen::ComputeThinU | Eigen::ComputeThinV);
    VLOG(3) << "Singular values are:\n" << svd.singularValues();
    //VLOG(2) << "Left singular vectors are the columns of U matrix:\n" << svd.matrixU();
    VLOG(3) << "Right singular vectors are the columns of V matrix:\n" << svd.matrixV();
    Eigen::Vector4d ciq_vec = svd.matrixV().col(3);
    //VLOG(1) << "CqI = " << ciq_vec.transpose() << " norm = " << ciq_vec.norm();
    
    QuaternionType q(ciq_vec[3], ciq_vec[0], ciq_vec[1], ciq_vec[2]);
    CqI = q;
    CrI = CqI.toRotationMatrix();
    AngleAxisType ciaa(CqI);
    CaI = ciaa;
    VLOG(1) << "Cam " << camera_num << ": CaI initial estimate  = " << ciaa.axis().transpose()
        << " angle = " << ciaa.angle()*DegreesPerRadian;
    
    // Write out the translations for plotting
    Eigen::Matrix<double,3,Eigen::Dynamic> cam_translations, imu_translations;
    cam_translations.resize(3,num_blocks); imu_translations.resize(3,num_blocks);
    i_block = 0;
    for (int i_rotn=0; i_rotn<num_rotations; i_rotn++) {
      if (usable_block[i_rotn]) {
        cam_translations.col(i_block) = CrI_collected_translations[i_rotn].matching_translation;
        cam_translations.col(i_block) /= CrI_collected_translations[i_rotn].matching_translation.norm();
        imu_translations.col(i_block) = CrI_collected_translations[i_rotn].translation;
        imu_translations.col(i_block) /= CrI_collected_translations[i_rotn].translation.norm();
        i_block++;
      }
    }
    anantak::WriteMatrixToCSVFile(save_filename+".cam_movs", cam_translations.transpose());      
    anantak::WriteMatrixToCSVFile(save_filename+".imu_movs", imu_translations.transpose());      
    
    /* Report the camera poses rotated in IMU frame assuming IMU and camera coincide in position.
     * We have T0qC0 and CqI. T0qI0 = T0qC0*CqI. Then I0qCj = T0qI0^-1 * T0qCj.
     * We need I0pCj. I0pCj = I0pT0 + I0rT0*T0pCj = -I0rT0*T0pI0 + IrC*C0rT0*T0pCj
     *  I0pCj = -(T0rI0^-1)*T0pI0 + IrC*(T0rC0^-1)*T0pCj
     * We assume that I and C coincide, meaning IpC = 0. Then T0pI0 = T0pC0 + T0rC0*CpI = T0pC0
     * Finally I0pCj = -(T0rI0^-1)*T0pC0 + IrC*(T0rC0^-1)*T0pCj = IrC*(T0rC0^-1)*(T0pCj - T0pC0)
     * Therefore: I0qCj = IrC*(T0rC0^-1) * T0qCj and I0pCj = IrC*(T0rC0^-1)*(T0pCj - T0pC0) */
    { // Find the first estimated camera pose
      size_t num_cams = ending_tag_image_idx_+1;
      int32_t cam0_idx = 0; while (camera_poses_[cam0_idx].info < Epsilon) cam0_idx++;
      Eigen::Matrix3d I0rT0 = CrI.transpose()*camera_poses_[cam0_idx].rotn.transpose();
      VLOG(1) << "Cam " << camera_num << ": Starting cam index for imu rotn = " << cam0_idx;
      // Calculate poses
      Eigen::Matrix<double,3,Eigen::Dynamic> cps; cps.resize(3,num_cams);
      for (int i=0; i<num_cams; i++) cps.col(i) =
          I0rT0 * (camera_poses_[i].posn - camera_poses_[cam0_idx].posn);
      anantak::WriteMatrixToCSVFile(save_filename+".camposns.imu", cps.transpose());      
    }
    
    return true;
  }
  
  
  // Estimate tag and camera positions - using fixed rotations of cameras and tags
  bool EstimateTagAndCameraPositions(const anantak::AprilTagMap& tag_map, const double& sigma_im,
        std::string save_filename = "") {
    /* Here we build a convex optimization problem by treating rotations of cameras and tags
     * as known. We only estimate the camera and tag positions wrt starting position as origin.*/
    
    // Allocate memory
    int32_t num_pose_estimates = CrI_estimation_images_num;
    WpCs.resize(3, num_pose_estimates); WpCs.setZero();
    initial_tag_map = tag_map; // make a copy of the input tag_map
    
    // Initiate variables
    const std::vector<std::string>& unique_tags_seen = initial_tag_map.tag_ids;
    int32_t num_tags = unique_tags_seen.size();
    Eigen::Vector3d starting_posn; starting_posn.setZero();
    initial_sigma_im = sigma_im;
    Eigen::Matrix3d CrC = Eigen::Matrix3d::Identity();
    //initial_tag_map.tag_poses[i_tag].WrT
    WpCs.col(0) = starting_posn;
    
    // Build problem
    ceres::Problem problem;
    
    // Add constraints using tag sightings
    for (int i_stng=0; i_stng<apriltag_sightings_.size(); i_stng++) {
      
      // Locate the camera pose number and the tag number for this sighting
      int32_t i_cam, i_tag;
      i_cam = apriltag_sightings_[i_stng].image_idx;
      auto itrtr = std::find(unique_tags_seen.begin(), unique_tags_seen.end(),
          apriltag_sightings_[i_stng].tag_id);
      if (itrtr!=unique_tags_seen.end()) {
        i_tag = std::distance(unique_tags_seen.begin(), itrtr); 
      } else {
        LOG(ERROR) << "Could not find " << apriltag_sightings_[i_stng].tag_id << " in unique tags";
        return false;
      }
      
      if (i_cam==0) {
        ceres::CostFunction* pose_constraint =
            new AprilTagFixedPoseConvexCostFunction2(
                &CrC, &apriltag_WrCs[i_cam], &(initial_tag_map.tag_poses[i_tag].WrT),
                &apriltag_sightings_[i_stng].cam_K,
                &apriltag_sightings_[i_stng].image_coords,
                &apriltag_sightings_[i_stng].Cpf,
                &starting_posn,
                &initial_sigma_im,
                &apriltag_sightings_[i_stng].tag_size
            );
        ceres::LossFunction* quadratic_loss = 
            NULL;
        problem.AddResidualBlock(
          pose_constraint,
          quadratic_loss,
          initial_tag_map.tag_poses[i_tag].WpT.data()  // WpT
          //,cam1_CpI.data() // CpI
        );        
      } else {
        ceres::CostFunction* pose_constraint =
            new AprilTagCamImuConvexCostFunction2(
                &CrC, &apriltag_WrCs[i_cam], &(initial_tag_map.tag_poses[i_tag].WrT),
                &apriltag_sightings_[i_stng].cam_K,
                &apriltag_sightings_[i_stng].image_coords,
                &apriltag_sightings_[i_stng].Cpf,
                &initial_sigma_im,
                &apriltag_sightings_[i_stng].tag_size
            );
        ceres::LossFunction* quadratic_loss = 
            NULL;
        problem.AddResidualBlock(
          pose_constraint,
          quadratic_loss,
          WpCs.data() + 3*i_cam, // WpI
          initial_tag_map.tag_poses[i_tag].WpT.data()  // WpT
          //,cam1_CpI.data() // CpI
        );
      }
      
    } // for i_stng
    
    // Solve problem
    ceres::Solver::Options options;
    options.max_num_iterations = 100;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    if (true) std::cout << summary.FullReport() << std::endl;
    
    // Save/report results
    if (save_filename!="") {
      anantak::WriteMatrixToCSVFile(save_filename, WpCs.transpose());
    }
    
    // Calculate Covariance of estimates. 
    
    return true;
  } // EstimateTagAndCameraPositions
  
  // Collect linear and angular motions using camera positions
  bool CollectMotions(const std::vector<int64_t>& cmnds_ts, const double& linear_threshold) {
    /* Interpolate camera WpC at commands timestamps assuming constant linear and angular
     * velocities. Then move along the timeline collecting motions that exceed the threshold */
    // only interpolate command timestamps that lie in the CrI estimation window
    std::vector<int64_t> interp_ts;     // stores the commands timestamps
    std::vector<int32_t> interp_ts_idx; // stores the indexes in cmnds_ts 
    { // Select relevant commands timestamps
      int32_t i1, i2; i1=0; i2=0;
      while (i1<apriltag_ts.size() && i2<cmnds_ts.size() &&
             (apriltag_ts[i1]<=CrI_estimation_end_ts || cmnds_ts[i2]<=CrI_estimation_end_ts)) {
        while (cmnds_ts[i2]<apriltag_ts[i1]) i2++;
        if (cmnds_ts[i2]<=CrI_estimation_end_ts) {
          interp_ts.push_back(cmnds_ts[i2]);
          interp_ts_idx.push_back(i2);
        }
        while (apriltag_ts[i1]<cmnds_ts[i2]) i1++;
      }
      VLOG(1) << "Cam " << camera_num << ": Interpolating " << interp_ts.size() << " command ts";
    }
    cmnds_cam_interp.clear();
    anantak::InterpolateTimestamps(apriltag_ts, interp_ts, &cmnds_cam_interp);
    
    // Interpolate camera poses
    cmnds_cam_interp_poses.clear();
    int32_t num_ref = apriltag_ts.size();
    QuaternionType unitq;
    for (int i=0; i<interp_ts.size(); i++) {
      int32_t idx = cmnds_cam_interp[i].index;
      double frac = cmnds_cam_interp[i].fraction;
      CameraPose rdng;
      rdng.timestamp = interp_ts[i];
      if (idx>-1 && idx<num_ref-1) {
        QuaternionType q0(apriltag_WrCs[idx]); QuaternionType q1(apriltag_WrCs[idx+1]);
        rdng.WqC = InterpolateQuaternionsLinearly(q0, q1, frac);
        rdng.WrC = rdng.WqC.toRotationMatrix();
        rdng.WpC = (1.0-frac)*WpCs.col(idx) + (frac)*WpCs.col(idx+1);
        cmnds_cam_interp_poses.push_back(rdng);
      } else {
        LOG(ERROR) << "idx<-1 || idx>num_ref-1" << idx << " " << num_ref;
      }
    }
    
    // Move through camera poses at command timestamps collecting motions
    int32_t last_i = 0;
    for (int i=1; i<cmnds_cam_interp_poses.size(); i++) {
      Eigen::Vector3d dWpC = cmnds_cam_interp_poses[i].WpC - cmnds_cam_interp_poses[last_i].WpC;
      double distance_moved = dWpC.norm();
      if (distance_moved > linear_threshold) {
        // Save this motion
        CollectedMotion motion;
        motion.index1 = interp_ts_idx[last_i];
        motion.index2 = interp_ts_idx[i];
        motion.P0pP1 = dWpC;
        // P0qP1 = WqP0^-1 * P1qW
        motion.P0qP1 = cmnds_cam_interp_poses[last_i].WqC.conjugate() * cmnds_cam_interp_poses[i].WqC;
        Eigen::AngleAxisd aa(motion.P0qP1);
        motion.P0aP1 = aa;
        motion.distance = distance_moved;
        motion.angle = motion.P0aP1.angle();
        initial_motions.push_back(motion);
        last_i = i;
        //VLOG(1) << "  motion = " << motion.distance << "(m) "
        //    << motion.angle*DegreesPerRadian << "(deg)";
      }
    }
    VLOG(1) << "Cam " << camera_num << ": Collected " << initial_motions.size() << " motions";
    
    return true;
  } // CollectMotions
  
}; // CameraMessages struct/class


/* Estimate tag rotations from tag sightings by multiple cameras */
bool EstimateTagRotationsFromSightings(
    const std::vector<std::unique_ptr<CameraMessages>>& all_cameras,
    anantak::AprilTagMap* tag_map) {
  int32_t num_negative_wt = 0;
  int32_t num_nan_wt = 0;
  /* Calculate an estimate of tag rotations by averaging quaternions over all tag sightings */
  std::vector<std::string>& unique_tags_seen = tag_map->tag_ids;
  for (int i_tag=0; i_tag<unique_tags_seen.size(); i_tag++) {
    //VLOG(1) << "Tag " << unique_tags_seen[i_tag];
    Eigen::Matrix4d wM; wM.setZero();
    double total_wt = 0.0;
    double tag_size;
    for (int i_cam=0; i_cam<all_cameras.size(); i_cam++) {
      const anantak::AprilTagSightingsVectorType& apriltag_sightings = all_cameras[i_cam]->apriltag_sightings_; //*(all_sightings[i_cam]);
      for (int i_st=0; i_st<apriltag_sightings.size(); i_st++) {
        if (apriltag_sightings[i_st].tag_id==unique_tags_seen[i_tag]) {
          double wt = 1./(std::max(apriltag_sightings[i_st].reproj_error,1.0) * apriltag_sightings[i_st].TpC[2]);
          if (!std::isnan(wt) && wt>0) {
            Eigen::Matrix4d qM = apriltag_sightings[i_st].TqW * apriltag_sightings[i_st].TqW.transpose();
            wM += wt * qM;
            total_wt += wt;
            VLOG(3) << "  " << apriltag_sightings[i_st].image_idx << " "
                << apriltag_sightings[i_st].TqW.transpose();
            tag_size = apriltag_sightings[i_st].tag_size;
          } // if wt makes sense
          else {
            if (std::isnan(wt)) num_nan_wt++;
            if (wt<0) num_negative_wt++;
          }
        } // if this tag was seen
      } // i_st
    } // i_cam
    wM /= total_wt;
    //VLOG(1) << "\n" << wM;
    // Solve for eigenvectors
    Eigen::EigenSolver<Eigen::Matrix4d> eigen_solver(wM);
    //VLOG(3) << "eigenvalues :\n" << eigen_solver.eigenvalues();
    //VLOG(3) << "eigenvectors:\n" << eigen_solver.eigenvectors();
    Eigen::Vector4d eval_real = eigen_solver.eigenvalues().real();
    Eigen::Vector4d::Index max_idx; eval_real.maxCoeff(&max_idx);
    Eigen::Vector4d evec_real = eigen_solver.eigenvectors().col(max_idx).real();
    //VLOG(1) << "Avg rotation for " << unique_tags_seen[i_tag] << " = " << evec_real.transpose();
    // Save
    QuaternionType tag_avg_q(evec_real(3), evec_real(0), evec_real(1), evec_real(2));
    tag_map->tag_poses[i_tag].tag_id = unique_tags_seen[i_tag];
    tag_map->tag_poses[i_tag].WqT = tag_avg_q.conjugate();
    tag_map->tag_poses[i_tag].WrT = tag_map->tag_poses[i_tag].WqT.toRotationMatrix();
    tag_map->tag_poses[i_tag].tag_size = tag_size;
  } // for each unique tag
  VLOG(1) << "Found num negative wts = " << num_negative_wt << " nan wts = " << num_nan_wt;
  
  if (true) { // Mutual tag rotations - for checking/reporting
    int anchor_quat_idx = 3;
    // TODO: Chose base tag as the one in first image that is straight ahead of camera.
    for (int i=0; i<tag_map->tag_poses.size(); i++) {
      // We have WqT. We need T0qTi = WqT0^-1 * WqT1.
      QuaternionType T0qT = tag_map->tag_poses[anchor_quat_idx].WqT.conjugate() *
          tag_map->tag_poses[i].WqT;
      RotationMatType T0rT(T0qT);
      AngleAxisType aa(T0rT);
      VLOG(1) << tag_map->tag_poses[anchor_quat_idx].tag_id << " r " << tag_map->tag_poses[i].tag_id
          << " aa = " << aa.axis().transpose() << ", " << aa.angle()*DegreesPerRadian
          << ",  size = " << tag_map->tag_poses[i].tag_size;
    }
  }
  
  return true;
}

/* Estimate tag and camera positions using convex optimization. Tag/Camera rotations are fixed */
bool EstimateTagAndCameraPositionsWithFixedRotations2(
    const std::vector<std::unique_ptr<CameraMessages>>& all_cameras,
    const int32_t ref_cam_num,      // Number of the reference camera
    anantak::AprilTagMap* tag_map,  // Tag map to be filled up with positions
    Eigen::Matrix<double,3,Eigen::Dynamic>* ref_cam_poses,  // return the ref cam poses
    std::string save_filename = "") {  // Poses of ref cam to be returned
  
  int32_t num_cameras = all_cameras.size();
  const std::vector<std::string>& unique_tags_seen = tag_map->tag_ids;
  
  // Checks
  if (ref_cam_num>num_cameras-1 || ref_cam_num<0) {
    LOG(ERROR) << "ref_cam_num>num_cameras-1 " << ref_cam_num << " " << num_cameras;
    return false;
  }
  
  // Assuming sightings are ordered
  int32_t starting_idx = all_cameras[ref_cam_num]->apriltag_sightings_.front().image_idx;
  int32_t num_poses = all_cameras[ref_cam_num]->apriltag_sightings_.back().image_idx+1; 
  
  // Allocate memory and initiate optimization variables
  StdVectorOfMatrix3dType CirC0; CirC0.resize(num_cameras);
  for (int i_cam=0; i_cam<num_cameras; i_cam++) {
    // CirC0 = CirI * C0rI^-1
    //CirC0[i_cam] = all_cameras[i_cam]->CrI * all_cameras[ref_cam_num]->CrI.transpose();
    CirC0[i_cam].setIdentity();
    VLOG(1) << "Cam " << i_cam << ": CirC0 = \n" << CirC0[i_cam];
  }
  Eigen::Vector3d starting_posn; starting_posn.setZero();
  (*ref_cam_poses).resize(3,num_poses); (*ref_cam_poses).setZero();
  (*ref_cam_poses).col(starting_idx) = starting_posn;
  Eigen::Matrix<double,3,Eigen::Dynamic> CipC0; CipC0.resize(3,num_cameras); CipC0.setZero();
  
  // Build problem
  ceres::Problem problem;
  
  //for (int i_cam=0; i_cam<num_cameras; i_cam++) {
  for (int i_cam=0; i_cam<2; i_cam++) {
    int32_t invalid_sightings = 0;
    
    if (i_cam == ref_cam_num) {
      // Use ref_cam timestamps 
      //for (int i_stng=0; i_stng<all_cameras[i_cam]->apriltag_sightings_.size(); i_stng++) {
      for (int i_stng=0; i_stng<0; i_stng++) {
        
        if (all_cameras[i_cam]->apriltag_sightings_[i_stng].IsValid()) {
          
          // Locate the camera pose number and the tag number for this sighting
          int32_t i_idx, i_tag;
          i_idx = all_cameras[i_cam]->apriltag_sightings_[i_stng].image_idx;
          if (i_idx > num_poses-1) {
            LOG(ERROR) << "i_idx > num_poses-1 " << i_idx << " " <<  num_poses;
            return false;
          }
          auto itrtr = std::find(unique_tags_seen.begin(), unique_tags_seen.end(),
              all_cameras[i_cam]->apriltag_sightings_[i_stng].tag_id);
          if (itrtr!=unique_tags_seen.end()) {
            i_tag = std::distance(unique_tags_seen.begin(), itrtr); 
          } else {
            LOG(ERROR) << "Could not find " << all_cameras[i_cam]->apriltag_sightings_[i_stng].tag_id
                << " in unique tags";
            continue;
          }
          
          if (i_idx==starting_idx) {
            ceres::CostFunction* pose_constraint =
                new AprilTagFixedPoseConvexCostFunction2(
                  &(CirC0[i_cam]),
                  &(all_cameras[i_cam]->apriltag_WrCs[i_idx]),
                  &(tag_map->tag_poses[i_tag].WrT),
                  &(all_cameras[i_cam]->apriltag_sightings_[i_stng].cam_K),
                  &(all_cameras[i_cam]->apriltag_sightings_[i_stng].image_coords),
                  &(all_cameras[i_cam]->apriltag_sightings_[i_stng].Cpf),
                  &starting_posn,
                  &(all_cameras[i_cam]->sigma_im_),
                  &(all_cameras[i_cam]->apriltag_sightings_[i_stng].tag_size)
                );
            ceres::LossFunction* quadratic_loss = 
                NULL;
            problem.AddResidualBlock(
              pose_constraint,
              quadratic_loss,
              tag_map->tag_poses[i_tag].WpT.data()  // WpT
              //,cam1_CpI.data() // CpI
            );        
          } else {
            ceres::CostFunction* pose_constraint =
                new AprilTagCamImuConvexCostFunction2(
                  &(CirC0[i_cam]),
                  &(all_cameras[i_cam]->apriltag_WrCs[i_idx]),
                  &(tag_map->tag_poses[i_tag].WrT),
                  &(all_cameras[i_cam]->apriltag_sightings_[i_stng].cam_K),
                  &(all_cameras[i_cam]->apriltag_sightings_[i_stng].image_coords),
                  &(all_cameras[i_cam]->apriltag_sightings_[i_stng].Cpf),
                  &(all_cameras[i_cam]->sigma_im_),
                  &(all_cameras[i_cam]->apriltag_sightings_[i_stng].tag_size)
                );
            ceres::LossFunction* quadratic_loss = 
                NULL;
            problem.AddResidualBlock(
              pose_constraint,
              quadratic_loss,
              (*ref_cam_poses).data() + 3*i_idx, // WpI
              tag_map->tag_poses[i_tag].WpT.data()  // WpT
              //,cam1_CpI.data() // CpI
            );
          } // if staring idx
        }  // is a valid sighting
        else {
          invalid_sightings++;
        }
      } // for all sightings of ref camera
      
    } else {
      // Use non-reference camera timestamps by interpolating them in ref-camera timestamps
      
      // Interpolate timestamps in the reference camera
      std::vector<anantak::LinearInterpolation> ref_interp;
      anantak::InterpolateTimestamps(all_cameras[ref_cam_num]->apriltag_ts,
          all_cameras[i_cam]->apriltag_ts, &ref_interp);
      
      // Use interpolated timestamps
      //for (int i_stng=0; i_stng<all_cameras[i_cam]->apriltag_sightings_.size(); i_stng++) {
      for (int i_stng=0; i_stng<1500; i_stng++) {
        
        if (all_cameras[i_cam]->apriltag_sightings_[i_stng].IsValid()) {
        
          // Locate the camera pose number and the tag number for this sighting
          int32_t i_idx, i_tag, i_ref_idx;
          i_idx = all_cameras[i_cam]->apriltag_sightings_[i_stng].image_idx;
          i_ref_idx = ref_interp[i_idx].index;
          if (i_idx!=i_ref_idx) VLOG(1) << "i_idx!=i_ref_idx " << i_idx << " " << i_ref_idx;
          if (ref_interp[i_idx].fraction>1e-6) VLOG(1) << "ref_interp[i_idx].fraction>1e-6 " << ref_interp[i_idx].fraction;
          if (i_ref_idx+1 > num_poses-1) {
            //LOG(ERROR) << "i_ref_idx+1 > num_poses-1 " << i_ref_idx << " " <<  num_poses;
            continue;
          }
          auto itrtr = std::find(unique_tags_seen.begin(), unique_tags_seen.end(),
              all_cameras[i_cam]->apriltag_sightings_[i_stng].tag_id);
          if (itrtr!=unique_tags_seen.end()) {
            i_tag = std::distance(unique_tags_seen.begin(), itrtr); 
          } else {
            LOG(ERROR) << "Could not find " << all_cameras[i_cam]->apriltag_sightings_[i_stng].tag_id
                << " in unique tags";
            continue;
          }
          
          if (i_ref_idx==starting_idx) {
            ceres::CostFunction* pose_constraint =
                new AprilTagFixedPoseConvexCostFunction(
                  &(CirC0[i_cam]),
                  &(all_cameras[i_cam]->apriltag_WrCs[i_idx]),
                  &(tag_map->tag_poses[i_tag].WrT),
                  &(all_cameras[i_cam]->apriltag_sightings_[i_stng].cam_K),
                  &(all_cameras[i_cam]->apriltag_sightings_[i_stng].image_coords),
                  &(all_cameras[i_cam]->apriltag_sightings_[i_stng].Cpf),
                  &starting_posn,
                  &(all_cameras[i_cam]->sigma_im_),
                  &(all_cameras[i_cam]->apriltag_sightings_[i_stng].tag_size)//,
                  //ref_interp[i_idx].fraction
                );
            ceres::LossFunction* quadratic_loss = 
                NULL;
            problem.AddResidualBlock(
              pose_constraint,
              quadratic_loss,
              //(*ref_cam_poses).data() + 3*(i_ref_idx+1), // WpI1
              tag_map->tag_poses[i_tag].WpT.data(),  // WpT
              CipC0.data() + 3*i_cam // CpI
            );
          } else {
            ceres::CostFunction* pose_constraint =
                new AprilTagCamImuConvexCostFunction(
                  &(CirC0[i_cam]),
                  &(all_cameras[i_cam]->apriltag_WrCs[i_idx]),
                  &(tag_map->tag_poses[i_tag].WrT),
                  &(all_cameras[i_cam]->apriltag_sightings_[i_stng].cam_K),
                  &(all_cameras[i_cam]->apriltag_sightings_[i_stng].image_coords),
                  &(all_cameras[i_cam]->apriltag_sightings_[i_stng].Cpf),
                  &(all_cameras[i_cam]->sigma_im_),
                  &(all_cameras[i_cam]->apriltag_sightings_[i_stng].tag_size)//,
                  //ref_interp[i_idx].fraction                  
                );
            ceres::LossFunction* quadratic_loss = 
                NULL;
            problem.AddResidualBlock(
              pose_constraint,
              quadratic_loss,
              (*ref_cam_poses).data() + 3*i_ref_idx, // WpI0
              //(*ref_cam_poses).data() + 3*(i_ref_idx+1), // WpI1
              tag_map->tag_poses[i_tag].WpT.data(),  // WpT
              CipC0.data() + 3*i_cam // CpI
            );
          } // if staring idx          
        
        }  // is a valid sighting
        else {
          invalid_sightings++;
        }      
      } // for all sightings of camera
      
    } // if this is ref cam
    
    VLOG(1) << "Cam " << i_cam << ": Found " << invalid_sightings << " invalid sightings";
    
  } // for all cameras
  
  // Solve problem
  ceres::Solver::Options options;
  options.max_num_iterations = 100;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  if (true) std::cout << summary.FullReport() << std::endl;
  
  // Save/report results
  VLOG(1) << "Cam to Cam positions = ";
  for (int i_cam=0; i_cam<num_cameras; i_cam++) {
    VLOG(1) << "C" << i_cam << "pC" << ref_cam_num << ": " << CipC0.col(i_cam).transpose() << ", "
        << CipC0.col(i_cam).norm() << "(m)";
  }
  if (save_filename!="") {
    anantak::WriteMatrixToCSVFile(save_filename, (*ref_cam_poses).transpose());
  }
  Eigen::Matrix<double,3,Eigen::Dynamic> tag_poses_mat;
  tag_poses_mat.resize(3,unique_tags_seen.size());
  for (int i_tag=0; i_tag<unique_tags_seen.size(); i_tag++) {
    tag_poses_mat.col(i_tag) = tag_map->tag_poses[i_tag].WpT;
  }
  if (save_filename!="") {
    anantak::WriteMatrixToCSVFile(save_filename+".tags", tag_poses_mat.transpose());
  }
  
  return true;
} 


/* Estimate tag and camera positions using convex optimization. Tag/Camera rotations are fixed */
bool EstimateTagAndCameraPositionsWithFixedRotations(
    const std::vector<std::unique_ptr<CameraMessages>>& all_cameras,
    const int32_t ref_cam_num,      // Number of the reference camera
    anantak::AprilTagMap* tag_map,  // Tag map to be filled up with positions
    Eigen::Matrix<double,3,Eigen::Dynamic>* ref_cam_poses,  // return the ref cam poses
    std::string save_filename = "") {  // Poses of ref cam to be returned
  
  int32_t num_cameras = all_cameras.size();
  const std::vector<std::string>& unique_tags_seen = tag_map->tag_ids;
  
  // Checks
  if (ref_cam_num>num_cameras-1 || ref_cam_num<0) {
    LOG(ERROR) << "ref_cam_num>num_cameras-1 " << ref_cam_num << " " << num_cameras;
    return false;
  }
  
  // Estimate the tag map and all camera positions without linking cameras together
  
  // Allocate memory for camera poses
  std::vector<std::unique_ptr<Eigen::Matrix<double,3,Eigen::Dynamic>>> camera_poses;
  std::vector<int32_t> num_camera_poses;
  camera_poses.resize(num_cameras);
  for (int i_cam=0; i_cam<num_cameras; i_cam++) {
    // Create the matrix on the heap
    std::unique_ptr<Eigen::Matrix<double,3,Eigen::Dynamic>>
        ptr(new Eigen::Matrix<double,3,Eigen::Dynamic>);
    // Assuming sightings are ordered
    int32_t num_cam_poses = all_cameras[i_cam]->apriltag_sightings_.back().image_idx+1;
    num_camera_poses.push_back(num_cam_poses);
    ptr->resize(3,num_cam_poses);
    ptr->setZero();
    // Transfer to vector of pointers
    camera_poses[i_cam] = std::move(ptr);
  }
  
  // Initiate optimization variables
  Eigen::Matrix3d CrC; CrC.setIdentity();
  int32_t tag0_index = 0;
  tag_map->tag_poses[tag0_index].WpT.setZero();
  
  // Build problem
  ceres::Problem problem;
  
  for (int i_cam=0; i_cam<num_cameras; i_cam++) {
    int32_t invalid_sightings = 0;
    
    for (int i_stng=0; i_stng<all_cameras[i_cam]->apriltag_sightings_.size(); i_stng++) {
      
      if (all_cameras[i_cam]->apriltag_sightings_[i_stng].IsValid()) {
        
        // Locate the camera pose number and the tag number for this sighting
        int32_t i_idx, i_tag;
        i_idx = all_cameras[i_cam]->apriltag_sightings_[i_stng].image_idx;
        if (i_idx > num_camera_poses[i_cam]-1) {
          LOG(ERROR) << "i_idx > num_poses-1 " << i_idx << " " <<  num_camera_poses[i_cam];
          return false;
        }
        auto itrtr = std::find(unique_tags_seen.begin(), unique_tags_seen.end(),
            all_cameras[i_cam]->apriltag_sightings_[i_stng].tag_id);
        if (itrtr!=unique_tags_seen.end()) {
          i_tag = std::distance(unique_tags_seen.begin(), itrtr); 
        } else {
          LOG(ERROR) << "Could not find " << all_cameras[i_cam]->apriltag_sightings_[i_stng].tag_id
              << " in unique tags";
          continue;
        }
        
        // Add a constraint
        ceres::CostFunction* pose_constraint =
            new AprilTagCamImuConvexCostFunction2(
              &(CrC),
              &(all_cameras[i_cam]->apriltag_WrCs[i_idx]),
              &(tag_map->tag_poses[i_tag].WrT),
              &(all_cameras[i_cam]->apriltag_sightings_[i_stng].cam_K),
              &(all_cameras[i_cam]->apriltag_sightings_[i_stng].image_coords),
              &(all_cameras[i_cam]->apriltag_sightings_[i_stng].Cpf),
              &(all_cameras[i_cam]->sigma_im_),
              &(all_cameras[i_cam]->apriltag_sightings_[i_stng].tag_size)
            );
        ceres::LossFunction* quadratic_loss = 
            NULL;
        problem.AddResidualBlock(
          pose_constraint,
          quadratic_loss,
          camera_poses[i_cam]->data() + 3*i_idx, // WpI
          tag_map->tag_poses[i_tag].WpT.data()  // WpT
        );
        
      }  // is a valid sighting
      else {
        invalid_sightings++;
      }
    } // i_stng
    
    VLOG(1) << "Cam " << i_cam << ": Found " << invalid_sightings << " invalid sightings";
  } // for all cameras
  
  // Mark tag0 as constant
  problem.SetParameterBlockConstant(tag_map->tag_poses[tag0_index].WpT.data());
  
  // Solve problem
  ceres::Solver::Options options;
  options.max_num_iterations = 100;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  if (true) std::cout << summary.FullReport() << std::endl;
  
  // Save/report results
  if (save_filename!="") {
    for (int i_cam=0; i_cam<num_cameras; i_cam++) {
      anantak::WriteMatrixToCSVFile(save_filename+".cam"+std::to_string(i_cam),
          camera_poses[i_cam]->transpose());
    }
  }
  Eigen::Matrix<double,3,Eigen::Dynamic> tag_poses_mat;
  tag_poses_mat.resize(3,unique_tags_seen.size());
  for (int i_tag=0; i_tag<unique_tags_seen.size(); i_tag++) {
    tag_poses_mat.col(i_tag) = tag_map->tag_poses[i_tag].WpT;
  }
  if (save_filename!="") {
    anantak::WriteMatrixToCSVFile(save_filename+".tags", tag_poses_mat.transpose());
  }
  
  // Use camera poses to estimate camera-to-camera transform
  int32_t n_collect = 500;
  Eigen::Matrix<double,3,Eigen::Dynamic> collect_mat; collect_mat.resize(3,n_collect);
  for (int i_cam=1; i_cam<2; i_cam++) {
    for (int i_pose=0; i_pose<n_collect; i_pose++) {
      collect_mat.col(i_pose) = camera_poses[i_cam]->col(i_pose) - camera_poses[0]->col(i_pose);
      VLOG(1) << collect_mat.col(i_pose).norm();
    }
  }
  
  return true;
} 


struct MachineCommand {
  int64_t timestamp;
  double velocity_command;
  double steering_command;
  double velo_cmd;   // centered
  double strg_cmd;   // centered
};

struct MachineCommands {
  
  // Messages
  std::vector<anantak::SensorMsg> machine_cmnd_msgs_;
  std::vector<MachineCommand> machine_cmnds_;
  std::vector<int64_t> cmnds_ts_;
  
  // Centers of velocity and steering commands
  double center_velocity_;
  double center_steering_;
  
  // Initial period of rest
  double rest_velo_cmnd_threshold_;
  int64_t initial_rest_begin_ts_, initial_rest_end_ts_, initial_rest_interval_;
  
  // Model parameters
  double lambda_velocity_;
  double lambda_steering_;
  
  // Initial estimation
  std::vector<double> command_motion_ratios_;
  
  // Load commands in constructor
  MachineCommands(const std::string& mi_msgs) {
    if (!anantak::LoadMsgsFromFile(mi_msgs, &machine_cmnd_msgs_))
        LOG(ERROR) << "Could not load data from " << mi_msgs;
    for (int i=0; i<machine_cmnd_msgs_.size(); i++) {
      if (!machine_cmnd_msgs_[i].has_mi_msg()) {
        LOG(ERROR) << "machine_cmnd_msgs_ does not have an mi_msg index = " << i;
        continue;
      }
      if (!machine_cmnd_msgs_[i].has_header()) {
        LOG(ERROR) << "machine_cmnd_msgs_ does not have a header index = " << i;
        continue;
      }
      cmnds_ts_.push_back(int64_t(machine_cmnd_msgs_[i].mi_msg().out_time()));
      MachineCommand mc;
      mc.timestamp = int64_t(machine_cmnd_msgs_[i].mi_msg().out_time());
      mc.velocity_command = 0.5*double(machine_cmnd_msgs_[i].mi_msg().left_motor() +
          machine_cmnd_msgs_[i].mi_msg().right_motor());
      mc.steering_command = 0.5*double(machine_cmnd_msgs_[i].mi_msg().left_servo() +
          machine_cmnd_msgs_[i].mi_msg().right_servo());
      machine_cmnds_.push_back(mc);
    }
  }
  
  // Set starting model parameters
  bool SetModelParameters(const double& v0, const double& s0) {
    center_velocity_ = v0;
    center_steering_ = s0;
    for (int i=0; i<machine_cmnds_.size(); i++) {
      MachineCommand& mc = machine_cmnds_[i];
      mc.velo_cmd = mc.velocity_command - center_velocity_;
      mc.strg_cmd = mc.steering_command - center_steering_;
    }
    return true;
  }
  
  // Find an initial period of rest in commands data
  bool FindInitialPeriodOfRest(const double& rest_velo_cmnd_threshold) {
    /* We need to find the time interval in which machine was a rest at the starting of the
     * calibration period. This helps in guessing initial gravity+accel_bias vectors. We do this
     * by finding the period during which velocity command - velocity center was less than
     * a threshold. This is a user-input during calibration. */
    rest_velo_cmnd_threshold_ = rest_velo_cmnd_threshold;
    initial_rest_begin_ts_ = machine_cmnds_.front().timestamp;
    int32_t cntr = 0;
    while (std::abs(machine_cmnds_[cntr].velo_cmd) < rest_velo_cmnd_threshold_) {
      cntr++;
    }
    initial_rest_end_ts_ = machine_cmnds_[cntr].timestamp;
    initial_rest_interval_ = initial_rest_end_ts_ - initial_rest_begin_ts_;
    VLOG(1) << "Initial rest interval using commands = " << double(initial_rest_interval_)*1e-6
        << "(s)";
    return true;
  }
  
  // Estimate lambda_velocity using collected motions from the camera
  bool EstimateLambdaVelocity(const CollectedMotionsVector& collected_motions,
      const double& center_velo, const double& straight_angle_threshold) {
    /* Go through the collected motions */
    center_velocity_ = center_velo;
    for (int i=0; i<collected_motions.size(); i++) {
      /* For each motion that has a turning angle of less than the threshold, integrate the
       * velocity_command (minus the center). */
      if (std::abs(collected_motions[i].angle) < straight_angle_threshold) {
        double command_integral = 0.;
        for (int j=collected_motions[i].index1; j<collected_motions[i].index2; j++) {
          double velo_command = machine_cmnds_[j].velocity_command - center_velocity_;
          double dt = 1e-6*double(machine_cmnds_[j+1].timestamp - machine_cmnds_[j].timestamp);
          double dx = velo_command * dt;
          command_integral += dx;
        }
        double cmnd_distance_ratio = std::abs(command_integral) / collected_motions[i].distance;
        VLOG(1) << "   distance = " << collected_motions[i].distance << " cmnd_integ = "
            << command_integral << " ratio = " << cmnd_distance_ratio;
        command_motion_ratios_.push_back(cmnd_distance_ratio);
      }  // if motion passes threshold angle
    } // for collected_motions
    VLOG(1) << "  Robust mean = " << anantak::RobustMean(command_motion_ratios_);
    return true;
  }
  
};

struct ImuMessages {
  
  // Messages and readings
  std::string imu_msgs_filename_;
  std::vector<anantak::SensorMsg> imu_msgs_;
  std::vector<anantak::ImuReadingType> imu_readings_;

  // Model parameters
  double accel_per_lsb_;  // least significant bits
  double gravity_mag_;    // m/s^2
  double gravity_mag_sigma_;  
  double accel_sigma_;  
  double gyro_sigma_; 
  
  // Estimates of gravity and acceleration using rest period
  Eigen::Vector3d rest_gravity_est_;
  Eigen::Vector3d rest_accelbias_est_;
  Eigen::Matrix3d rest_gravity_est_cov_;
  Eigen::Matrix3d rest_accelbias_est_cov_;
  Eigen::Matrix3d rest_grav_accelbias_est_cov_;
  
  // Integrals for velocity and position using rest period estimates
  Eigen::Matrix<double,3,Eigen::Dynamic> rest_position_integral_;
  Eigen::Matrix<double,3,Eigen::Dynamic> rest_velocity_integral_;
  
  // Load IMU data
  ImuMessages(const std::string& imu_msgs_filename) {
    imu_msgs_filename_ = imu_msgs_filename;
    if (!LoadMsgsFromFile(imu_msgs_filename, &imu_msgs_)) {
      LOG(ERROR) << "Could not load IMU data from " << imu_msgs_filename;
    }
    anantak::ExtractImuReadingsFromImuMessages(imu_msgs_, &imu_readings_);
  } // ImuMessages
  
  // Set model parameters
  bool SetModelParameters(const double& accel_per_lsb, const double& gravity_mag,
      const double& gravity_mag_sigma, const double& accel_sigma, const double& gyro_sigma) {
    accel_per_lsb_ = accel_per_lsb;
    gravity_mag_ = gravity_mag;
    gravity_mag_sigma_ = gravity_mag_sigma;
    accel_sigma_ = accel_sigma;
    gyro_sigma_ = gyro_sigma;
    return true;
  }
  
  // Estimate gravity and accelerometer biases using rest period
  bool EstimateGravityAndAccelBiasUsingRestPeriod(const int64_t& ts0, const int64_t& ts1) {
    
    // Integrate imu readings between ts0 and ts1
    double accel_factor = gravity_mag_ / accel_per_lsb_;
    int32_t idx0, idx1, num_readings;
    idx0=0; while (imu_readings_[idx0].timestamp<ts0 && idx0<imu_readings_.size()-1) idx0++; 
    idx1=0; while (imu_readings_[idx1].timestamp<ts1 && idx1<imu_readings_.size()-1) idx1++; idx1--;
    if (idx0>=idx1) {LOG(ERROR)<<"inconsistent idxs "<<idx0<<" "<<idx1; return false;}
    num_readings = idx1 - idx0;
    VLOG(1) << "IMU num of readings = " << num_readings;
    std::vector<ImuReadingsIntegralType> readings_integrals;
    readings_integrals.resize(num_readings);
    for (int i_rdng=0; i_rdng<num_readings; i_rdng++) {
      IntegrateImuKinematics(
        imu_readings_,
        idx0 + i_rdng,
        idx0 + i_rdng + 1,
        accel_factor,
        &readings_integrals[i_rdng]
      );
      VLOG_EVERY_N(1,300) << "imu integral " << i_rdng << " " << readings_integrals[i_rdng];
    }
    
    // Build a problem and solve for gravity and accel biases at rest
    ceres::Problem problem;
    
    rest_gravity_est_.setZero();
    rest_accelbias_est_.setZero();
    double* grav_ptr = rest_gravity_est_.data();
    double* bias_ptr = rest_accelbias_est_.data();
    
    for (int i_rdng=0; i_rdng<num_readings; i_rdng++) {
      ceres::CostFunction* pose_constraint =
          new ConvexImuKinAtRestFunction(
            &readings_integrals[i_rdng],
            gravity_mag_, accel_sigma_, gravity_mag_sigma_
          );
      ceres::LossFunction* quadratic_loss = 
          NULL;
      problem.AddResidualBlock(
        pose_constraint,
        quadratic_loss,
        grav_ptr,
        bias_ptr
      );
    }
    
    // Solve problem
    ceres::Solver::Options options;
    options.max_num_iterations = 100;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    if (true) std::cout << summary.FullReport() << std::endl;
    
    // Estimate covariance
    ceres::Covariance::Options covariance_options;
    ceres::Covariance covariance(covariance_options);
    std::vector<std::pair<const double*, const double*>> covariance_blocks;
    covariance_blocks.push_back(std::make_pair(grav_ptr, grav_ptr));
    covariance_blocks.push_back(std::make_pair(bias_ptr, grav_ptr));
    covariance_blocks.push_back(std::make_pair(bias_ptr, bias_ptr));
    CHECK(covariance.Compute(covariance_blocks, &problem));
    covariance.GetCovarianceBlock(grav_ptr, grav_ptr, rest_gravity_est_cov_.data());
    covariance.GetCovarianceBlock(bias_ptr, bias_ptr, rest_accelbias_est_cov_.data());
    covariance.GetCovarianceBlock(grav_ptr, bias_ptr, rest_grav_accelbias_est_cov_.data());

    // Report results
    VLOG(1) << "Gravity = " << rest_gravity_est_.transpose() << ", " << rest_gravity_est_.norm()
        << " (m/s^2)";
    VLOG(1) << "Gravity stdev = " << rest_gravity_est_cov_.diagonal().cwiseSqrt().transpose();
    VLOG(1) << "Accel biases = " << rest_accelbias_est_.transpose() << " (m/s^2)";
    VLOG(1) << "Accel biases stdev = " << rest_accelbias_est_cov_.diagonal().cwiseSqrt().transpose();
    VLOG(1) << "Gravity = " << rest_gravity_est_.transpose()/accel_factor << ", "
        << rest_gravity_est_.norm()/accel_factor << " (LSB)";
    VLOG(1) << "Accel biases = " << rest_accelbias_est_.transpose()/accel_factor << " (LSB)";
    
    return true;
  }
  
  // Integrate imu readings using rest period estimates assuming starting from rest
  bool IntegrateImuReadingsUsingRestPeriodEstimates(const int64_t& ts0, const int64_t& ts1,
      std::string save_filename="") {
    
    // Integrate imu readings between ts0 and ts1
    double accel_factor = gravity_mag_ / accel_per_lsb_;
    int32_t idx0, idx1, num_readings;
    idx0=0; while (imu_readings_[idx0].timestamp<ts0 && idx0<imu_readings_.size()-1) idx0++; 
    idx1=0; while (imu_readings_[idx1].timestamp<ts1 && idx1<imu_readings_.size()-1) idx1++; idx1--;
    if (idx0>=idx1) {LOG(ERROR)<<"inconsistent idxs "<<idx0<<" "<<idx1; return false;}
    num_readings = idx1 - idx0;
    VLOG(1) << "IMU num of readings = " << num_readings;
    std::vector<ImuReadingsIntegralType> readings_integrals;
    rest_position_integral_.resize(3,num_readings); rest_position_integral_.setZero();
    rest_velocity_integral_.resize(3,num_readings); rest_velocity_integral_.setZero();
    Eigen::Matrix<double,3,Eigen::Dynamic> accelerations; accelerations.resize(3,num_readings);
    
    for (int i_rdng=0; i_rdng<num_readings-1; i_rdng++) {
      IntegrateImuReading(
        imu_readings_,
        idx0 + i_rdng,
        accel_factor,
        rest_gravity_est_.data(),
        rest_accelbias_est_.data(),
        rest_position_integral_.data() + 3*i_rdng,
        rest_velocity_integral_.data() + 3*i_rdng,
        rest_position_integral_.data() + 3*(i_rdng+1),
        rest_velocity_integral_.data() + 3*(i_rdng+1)
      );
      accelerations.col(i_rdng) = imu_readings_[i_rdng].acceleration;
    }
    accelerations.col(num_readings-1) = imu_readings_[num_readings-1].acceleration;
    
    // Save data to disk
    if (save_filename!="") {
      anantak::WriteMatrixToCSVFile(save_filename+".posn", rest_position_integral_.transpose());
      anantak::WriteMatrixToCSVFile(save_filename+".velo", rest_velocity_integral_.transpose());
      anantak::WriteMatrixToCSVFile(save_filename+".accl", accelerations.transpose());
    }
    
    return true;
  } // IntegrateImuReadingsUsingRestPeriodEstimates
  
};  // ImuMessages


/* Data Keeper
 * Loads messages data
 * Set Time based on clock time or data will be returned using time interval
 * Returns new messages in an interval or clock time passed
 */
class FileMessagesKeeper {
 public:
  
  FileMessagesKeeper(const std::vector<std::string>& msgs_filenames, const bool run_in_reatime_mode) {
    msgs_filenames_ = msgs_filenames;
    run_in_reatime_mode_ = run_in_reatime_mode;
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
      if (min_files_ts==0 && file_min_ts!=0) {
        min_files_ts = file_min_ts;
      } else {
        min_files_ts = std::min(file_min_ts, min_files_ts);
      }
    }
    if (min_files_ts==0) {
      LOG(ERROR) << "Could not calculate minimum files timestamp";
      return false;
    }
    curr_time_ = get_wall_time_microsec();
    last_fetch_time_ = 0;
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
        msgs_indexes_[i_file]++;
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
    int32_t num_msgs = 0;
    int64_t ts0, ts1;
    if (run_in_reatime_mode_) {
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
  
  // Data variables
  std::vector<std::string> msgs_filenames_;
  std::vector<std::unique_ptr<std::vector<anantak::SensorMsg>>> sensor_msgs_;
  int32_t num_files_;
  bool run_in_reatime_mode_;
  int64_t file_time_curr_time_offset_; // 
  int64_t curr_time_; // Current time
  int64_t last_fetch_time_; // last timestamp when data was fetched
  std::vector<int32_t> msgs_indexes_; // current indexes of each messages vector
};  // FileMessagesKeeper

/* Tag Camera options
 * Camera States To Keep - number of past camera states to keep, these will not be marginalized
 * Keep camera intrinsic calibration constant - intrinsics will not be modified (true)
 */

struct AprilTagCameraOptions {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  int32_t max_num_of_views_per_iteration;
  int32_t max_num_of_cam_poses_per_iteration;
  int32_t camera_num;
  double april_tag_size;
  double april_tag_size_sigma;
  double sigma_im;
  int32_t max_cam_poses_per_tag_map;      // Maximum number of camera poses history kept
  double infinite_information;
  int32_t max_views_loops_per_iteration;
  int32_t max_tags_num;
  int32_t max_tag_maps;

  AprilTagCameraOptions() {
    max_num_of_views_per_iteration = 500;
    max_num_of_cam_poses_per_iteration = 100;
    camera_num = 0;
    max_cam_poses_per_tag_map = 500;
    infinite_information = 1e14;
    max_views_loops_per_iteration = 10;
    max_tags_num = 500;
    max_tag_maps = 100;
  }
}; // AprilTagCameraOptions

/* Tag Camera model
 * Init - get starting monocular calibration
 * Add Tag Views - A list of views by this camera are added
 */

class AprilTagCamera {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  struct TagCompositePose {
    std::string tag_id;
    CompositePose pose;
    TagCompositePose(): tag_id(""), pose() {}
    TagCompositePose(const std::string& tid): tag_id(tid), pose() {}
    TagCompositePose(const TagCompositePose& tp): tag_id(tp.tag_id), pose(tp.pose) {}
  };
  
  struct CamCompositePose {
    int64_t timestamp;
    CompositePose pose;
    CamCompositePose(): timestamp(0), pose() {}
    CamCompositePose(const CamCompositePose& cp): timestamp(cp.timestamp), pose(cp.pose) {}
  };
  
  AprilTagCamera(const AprilTagCameraOptions& options, const anantak::SensorMsg& calib_msg) :
      zero_cam_pose_() {
    options_ = options; // copy options, dont own it
    cam_num_ = options_.camera_num;
    curr_views_.resize(options_.max_num_of_views_per_iteration);
    // Calculate the camera matrix and store
    if (!calib_msg.has_mono_calib_no_distort_msg()) {
      LOG(ERROR) << "No calib msg was found";
    } else {
      const anantak::MonocularPinholeCalibrationNoDistortionMsg& msg =
          calib_msg.mono_calib_no_distort_msg();
      camera_K_ << msg.focal_length(), 0.0, msg.cx(),
                0.0, msg.focal_length(), msg.cy(),
                0.0, 0.0, 1.0;
      VLOG(1) << "Cam " << cam_num_ << ": Extracted camera matrix\n" << camera_K_;
      camera_K_inv_ = camera_K_.inverse();
    }
    // Allocate memory for tag_poses_ and connected_tags_
    connected_tags_.resize(options_.max_tag_maps);
    tag_poses_.resize(options_.max_tag_maps);
    num_connected_tags_ = 0;
    connected_tags_sizes_.resize(options_.max_tag_maps, 0);
    const TagCompositePose zero_tag_pose;
    for (int i=0; i<options_.max_tag_maps; i++) {
      std::unique_ptr<std::vector<std::string>> str_vec_ptr(new std::vector<std::string>);
      connected_tags_[i] = std::move(str_vec_ptr);
      connected_tags_[i]->resize(options_.max_tags_num, "");
      std::unique_ptr<std::vector<TagCompositePose>> tp_vec_ptr(new std::vector<TagCompositePose>);
      tag_poses_[i] = std::move(tp_vec_ptr);
      tag_poses_[i]->resize(options_.max_tags_num, zero_tag_pose);
    }
    // Allocate memory for curr_cam_poses_
    curr_cam_poses_.resize(options_.max_num_of_cam_poses_per_iteration, zero_cam_pose_);
  }
  
  virtual ~AprilTagCamera() {}  
  
  // In every iteration, new messages are sent to the model to be processed. 
  bool ExtractAprilTagViews(const std::vector<anantak::SensorMsg>& msgs) {
    curr_num_msgs_ = msgs.size();
    curr_num_views_ = 0;
    // From each message, get tag views
    for (int i_msg=0; i_msg<curr_num_msgs_; i_msg++) {
      // Check if an AprilTagMessage is present
      if (!msgs[i_msg].has_april_msg()) {
        LOG(ERROR) << "No AprilTag message was found in the message";
        continue;
      }
      // Go through all tag views
      const anantak::AprilTagMessage& apriltag_msg = msgs[i_msg].april_msg();
      VLOG(3) << "Number of AprilTags in msg = " << apriltag_msg.tag_id_size();
      // Extract the views one-by-one
      for (int i_tag=0; i_tag<apriltag_msg.tag_id_size(); i_tag++) {
        curr_views_[curr_num_views_].SetFromAprilTagMessage(i_tag, apriltag_msg,
            options_.april_tag_size, i_msg);
        curr_views_[curr_num_views_].CalculateTagPoseInCamera(camera_K_, camera_K_inv_);
        //VLOG(1) << curr_views_[curr_num_views_];
        curr_num_views_++;
      }
    }
    VLOG(2) << "Cam " << cam_num_ << ": num msgs = " << curr_num_msgs_ << ", num tag views = "
        << curr_num_views_;
    return true;
  }
  
  // Group tags in connected sets
  bool GroupTags(const std::vector<anantak::SensorMsg>& msgs) {
    /* At a time, we have a sets of tag_ids. Each set has tag_ids with no overlaps.
     * For a new set of tag_ids in an image, check where each tag_id belongs. If it is found,
     * mark all images with this set number, add the new tag_ids to the set. If it is not found,
     * add 1 to max set number and mark all tag_ids in image with this. Move to next tag_id. If no
     * more tag_ids are left and number is max number, initiate a new set with images seen here. */
    
    for (int i_msg=0; i_msg<msgs.size(); i_msg++) {
      bool tags_assigned = false;
      int32_t tag_num = 0;
      //VLOG(1) << "This message has a AprilTagMsg? " << msgs[i_msg].has_april_msg();
      const anantak::AprilTagMessage& msg = msgs[i_msg].april_msg();
      //VLOG(1) << "This message has n AprilTag msgs = " << msg.tag_id_size();
      if (msg.tag_id_size()==0) continue;
      while (!tags_assigned) {
        const std::string& tag_id = msg.tag_id(tag_num);
        //VLOG(1) << "  Looking for " << tag_id << " in " << num_connected_tags_ << " sets";
        // find this tag_id in the connected tags
        bool tag_found = false; size_t i_ctags = 0;
        for (size_t i=0; i<num_connected_tags_; i++) {
          if (connected_tags_sizes_[i]>0) {
            auto itrtr = std::find(connected_tags_[i]->begin(), connected_tags_[i]->end(),
                tag_id);
            tag_found = (itrtr!=connected_tags_[i]->end());
            if (tag_found) i_ctags=i;
          }
        }
        //VLOG(1) << "    Found tag = " << tag_found << " in set " << i_ctags;
        if (tag_found) {
          // All tags in this image are added to this set
          for (int i_tag=0; i_tag<msg.tag_id_size(); i_tag++) {
            auto itrtr = std::find(connected_tags_[i_ctags]->begin(), connected_tags_[i_ctags]->end(),
                msg.tag_id(i_tag));
            if (itrtr==connected_tags_[i_ctags]->end()) {
              if (connected_tags_sizes_[i_ctags] < options_.max_tags_num-1) {
                connected_tags_[i_ctags]->at(connected_tags_sizes_[i_ctags]) = msg.tag_id(i_tag);
                tag_poses_[i_ctags]->at(connected_tags_sizes_[i_ctags]).tag_id = msg.tag_id(i_tag);
                connected_tags_sizes_[i_ctags]++;
                VLOG(1) << "Cam "<<cam_num_<<": New tag seen " << msg.tag_id(i_tag);
              } // if < options_.max_tags_num
            } // if a new tag
          }
          tags_assigned = true;
        } else {
          // if this is the last tag_id add a new set, allocate memory in tag_poses_
          if (tag_num == msg.tag_id_size()-1) {
            if (num_connected_tags_ < options_.max_tag_maps-1) {
              for (int i_tag=0; i_tag<msg.tag_id_size(); i_tag++) {
                if (connected_tags_sizes_[num_connected_tags_] < options_.max_tags_num-1) {
                  connected_tags_[num_connected_tags_]->at(i_tag) = msg.tag_id(i_tag);
                  tag_poses_[num_connected_tags_]->at(i_tag).tag_id = msg.tag_id(i_tag);
                  connected_tags_sizes_[num_connected_tags_]++;
                }
              }
              // Set the amount of information on the first tag in a set to infinite
              tag_poses_[num_connected_tags_]->at(0).pose.info = options_.infinite_information;
              VLOG(1) << "Cam "<<cam_num_<<": New tag set seen with "
                  << connected_tags_sizes_[num_connected_tags_] << " tags";
              num_connected_tags_++;
              tags_assigned = true;
              
              // Allocate memory in cam_poses_ for the new set
              std::unique_ptr<anantak::CircularQueue<CamCompositePose>> cq_ptr(new
                  anantak::CircularQueue<CamCompositePose>(options_.max_cam_poses_per_tag_map));
              cam_poses_.push_back(std::move(cq_ptr));
            } // < options_.max_tag_maps
          } // if new tag set
          // else keep looking
          else {
            tag_num++;
          }
        } // if tag is found
      } // tags assigned
    } // for
    VLOG(3) << "Cam "<<cam_num_<<": Has "<<num_connected_tags_<<" connected tag sets";
    return true;
  }

  // Add information from new tag views
  bool CalculateTagAndCamPoses(const std::vector<anantak::SensorMsg>& msgs) {
    
    size_t tags_set_num = 0;  // We only solve for the first tag set
    
    for (int i=0; i<curr_num_msgs_; i++) {
      curr_cam_poses_[i] = zero_cam_pose_;
      curr_cam_poses_[i].timestamp = int64_t(msgs[i].header().timestamp());
    }
    
    // A holder for marking if the view has been used
    std::vector<bool> view_used; view_used.resize(curr_num_views_, false);
    bool views_left = true;
    int32_t loop_num = 0;
    
    // Mark invalid views as used so that they are bypassed in the calculations
    //VLOG(1) << "  Marking invalid poses";
    int32_t num_invalid_views = 0;
    for (int i_stng=0; i_stng<curr_num_views_; i_stng++) {
      view_used[i_stng] = !curr_views_[i_stng].IsValid();
      if (view_used[i_stng]) num_invalid_views++;
    }
    //VLOG(1) << "   num_invalid_views = " << num_invalid_views;
    
    // Go through each view and add its information to the poses
    while (views_left && loop_num<options_.max_views_loops_per_iteration) {
      for (int i_stng=0; i_stng<curr_num_views_; i_stng++) {
        if (!view_used[i_stng]) {
          //VLOG(1) << "i_stng = " << i_stng;
          const anantak::AprilTagView& view = curr_views_[i_stng];
          //VLOG(1) << "i_stng 2 = " << i_stng;
          //const std::string& tag_id = view.tag_id;
          const std::string& tag_id = view.tag_id;
          //VLOG(1) << "tag_id = " << tag_id;
          // Make sure that tag id seen belongs to this set
          //VLOG(1) << "connected_tags_[tags_set_num].size() = " << connected_tags_sizes_[tags_set_num];
          //for (int ii=0; ii<connected_tags_sizes_[tags_set_num]; ii++) {
          //  std::cout<<connected_tags_[tags_set_num]->at(ii)<<" ";} std::cout<<"\n"<<std::flush;
          auto itrtr = std::find(connected_tags_[tags_set_num]->begin(), connected_tags_[tags_set_num]->end(), view.tag_id);
          //VLOG(1) << "tag_id 2 = " << tag_id;
          bool tag_found = (itrtr!=connected_tags_[tags_set_num]->end());
          if (tag_found) {  //VLOG(1) << "  tag " << tag_id << "was found";
            int32_t i_cam, i_tag;
            i_cam = view.image_idx;
            i_tag = std::distance(connected_tags_[tags_set_num]->begin(), itrtr);
            //VLOG(1) << "  i_cam, i_tag = " << i_cam << " " << i_tag;
            // Has either the camera or tag for this view been initiated?
            // Heuristic based information
            double new_info = 1./(std::max(1.,view.reproj_error)*view.TpC[2]);
            CompositePose& tag_pose = tag_poses_[tags_set_num]->at(i_tag).pose;
            CompositePose& cam_pose = curr_cam_poses_[i_cam].pose;
            // Update camera position and rotation
            if (tag_pose.info>Epsilon) {
            //if (tag_pose.info>Epsilon && cam_pose.info<Epsilon) {
              double info = std::min(new_info, tag_pose.info);
              // Tag's rotation is T0rTi. We have TirCj. We need T0rCj = T0rTi * TirCj
              Eigen::Quaterniond T0qCj = tag_pose.rotn_q * view.TqC;
              // T0pCj = T0pTi + T0rTi*TipCj
              Eigen::Vector3d T0pCj = tag_pose.posn + tag_pose.rotn * view.TpC;
              // Update with new info
              //VLOG(1) << "  cam info is before " << cam_pose.info;
              cam_pose.AddInformation(&info, &T0qCj, &T0pCj);
              //VLOG(1) << "  cam info is after " << cam_pose.info;
              view_used[i_stng] = true;
            }
            // Update tag position and rotation 
            if (cam_pose.info>Epsilon && i_tag!=0) {
            //if (cam_pose.info>Epsilon && tag_pose.info<Epsilon) {
              double info = std::min(new_info, cam_pose.info);
              // Camera's rotation is T0rCj. We saw TirCj. We need T0rTi = T0rCj * TirCj^-1
              Eigen::Quaterniond T0qTi = cam_pose.rotn_q * view.TqC.conjugate();
              // T0pTi = T0pCj - T0rCj*  CjrTi * TipCj
              Eigen::Vector3d T0pTi = cam_pose.posn
                  - cam_pose.rotn * view.TrC.transpose() * view.TpC;
              // Update with new info
              //VLOG(1) << "  tag info is before " << tag_pose.info << " from " << info;
              tag_pose.AddInformation(&info, &T0qTi, &T0pTi);
              //VLOG(1) << "  tag info is after " << tag_pose.info << " from " << info;
              view_used[i_stng] = true;
            }
          } // tag found
          else {
            view_used[i_stng] = true;
          }
        } // if stng is not used yet
      } // for
      loop_num++;
      int32_t num_views_left = 0;
      for (int i_stng=0; i_stng<curr_num_views_; i_stng++) {
        if (!view_used[i_stng]) num_views_left++;
      }
      VLOG(3)<<"Cam "<<cam_num_<<": "<<num_views_left<<" views left after pass "<<loop_num;
      views_left = (num_views_left>0);
    }  // while more views are left
    
    // Copy the curr_cam_poses_ over to camera's circular queue
    if (cam_poses_.size()>0) {
      //VLOG(1) << "starting temp cam poses copy";
      for (int i=0; i<curr_num_msgs_; i++) {
        cam_poses_[tags_set_num]->add_element(curr_cam_poses_[i]);
      }
      //VLOG(1) << "temp cam poses copied";
    }
    
    return true;
  } // CalculateTagAndCamPoses
  
  // Save poses to file for plotting
  bool SavePosesToFile(const std::string& save_filename, int32_t tags_set_num=0) {
    int32_t num_tags = connected_tags_sizes_[tags_set_num];
    Eigen::Matrix<double,3,Eigen::Dynamic> tps; tps.resize(3,num_tags);
    VLOG(1) << "Cam "<<cam_num_<<": Tags and their poses seen";
    for (int i_tag=0; i_tag<num_tags; i_tag++) {
      tps.col(i_tag) = tag_poses_[tags_set_num]->at(i_tag).pose.posn;
      AngleAxisType aa(tag_poses_[tags_set_num]->at(i_tag).pose.rotn_q);
      VLOG(1) << "  Tag " << connected_tags_[tags_set_num]->at(i_tag) << ": "
          << aa.axis().transpose() << ", " << aa.angle()*DegreesPerRadian;
    }
    anantak::WriteMatrixToCSVFile(save_filename+".tag.posns", tps.transpose());
    // Save camera poses for plotting
    int32_t num_cams = cam_poses_[tags_set_num]->n_msgs();
    Eigen::Matrix<double,3,Eigen::Dynamic> cps; cps.resize(3,num_cams);
    for (int i=0; i<num_cams; i++) cps.col(i) = cam_poses_[tags_set_num]->at(i).pose.posn;
    anantak::WriteMatrixToCSVFile(save_filename+".cam.posns", cps.transpose());
    return true;
  }
  
  bool ProcessAprilTagMessages(const std::vector<anantak::SensorMsg>& msgs) {
    // Extract tag views
    VLOG(3) << "    Starting ExtractAprilTagViews";
    if (!ExtractAprilTagViews(msgs)) {LOG(ERROR)<<"Error in ExtractAprilTagViews";return false;}
    // Group tags in connected sets
    VLOG(3) << "    Starting GroupTags";
    if (!GroupTags(msgs)) {LOG(ERROR)<<"Error in GroupTags"; return false;}
    // Add information to tag poses
    VLOG(3) << "    Starting CalculateTagAndCamPoses";
    if (!CalculateTagAndCamPoses(msgs)) {LOG(ERROR)<<"Error in CalculateTagAndCamPoses"; return false;}
    // Refine tag poses using optimization - not sure if this is needed here, will be done in VIO
    // Marginalize oldest poses - not sure if this is needed here, will be done in VIO
    // Remove oldest poses
    VLOG(3) << "    Done ProcessAprilTagMessages";
    return true;
  }
  
  // Data holders
  AprilTagCameraOptions options_;
  int32_t cam_num_;
  Eigen::Matrix3d camera_K_, camera_K_inv_;
  
  double tag_size_;
  int32_t curr_num_msgs_, curr_num_views_;
  std::vector<anantak::AprilTagView> curr_views_; // Current tag views
  std::vector<CamCompositePose> curr_cam_poses_;  // Current camera poses
  const CamCompositePose zero_cam_pose_;

  std::vector<std::unique_ptr<std::vector<std::string>>> connected_tags_;  // Sets of tags seen together
  std::vector<std::unique_ptr<std::vector<TagCompositePose>>> tag_poses_;
  int32_t num_connected_tags_;
  std::vector<int32_t> connected_tags_sizes_;
  std::vector<std::unique_ptr<anantak::CircularQueue<CamCompositePose>>> cam_poses_;
  
}; // AprilTagCamera


/* IMU Initiation camera coupled with AprilTag camera
 * This collects IMU rotations from its quaternion readings.
 */

struct AprilTagImuInitOptions {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  int32_t max_imu_readings_per_iteration; // Max readings per iterations that will be processed
  int32_t max_readings_history;   // Max num of cam motion readings will be used for imu init
  double min_enough_rotation_angle;
  double min_enough_translation_distance;
  int32_t min_num_enough_rotations;
  int32_t min_num_enough_translations;
  int32_t min_num_reading_history;
  double accel_factor;
  double gravity_magnitude;
  double sigma_gravity;
  double sigma_accel;
  double sigma_gyro;
  
  Eigen::Vector3d CpI_measured;
  Eigen::AngleAxisd CaI_measured;
  Eigen::Quaterniond CqI_measured;
  
  std::string save_filename;
  bool save_data;
  
  AprilTagImuInitOptions() {
    max_imu_readings_per_iteration = 200;
    max_readings_history = 1000;
    
    min_enough_rotation_angle = 5.*RadiansPerDegree;
    min_enough_translation_distance = 0.25;
    min_num_enough_rotations = 20;
    min_num_enough_translations = 20;
    min_num_reading_history = 500;
    
    double grav_mag_lb = 9.7; // m/s^2 - lower bound of gravity mag
    double grav_mag_ub = 9.9; // m/s^2 - upper bound of gravity mag
    double grav_range_stdev = 4.0; // range of stdev's between grav ub and lb
    
    gravity_magnitude = 9.8; // m/s^2  http://www.physicsclassroom.com/class/circles/Lesson-3/The-Value-of-g
    accel_factor = gravity_magnitude/8192.0;
    sigma_gravity = (grav_mag_ub*grav_mag_ub - grav_mag_lb*grav_mag_lb)/grav_range_stdev;
    sigma_accel = 400.0*1e-6*gravity_magnitude; // m/s^s/sqrt(Hz) from the datasheet
    sigma_gyro = 5.*1e-3*RadiansPerDegree; // rad/s/sqrt(Hz) from the datasheet
    
    Eigen::Vector3d CaI_measured_axis;
    double CaI_measured_angle;
    
    CpI_measured << 0.175, 0.100, 0.100; // m
    CaI_measured_axis << 1., 1., -1.; // any measure
    CaI_measured_angle = 120.*RadiansPerDegree;  // radians
    
    CaI_measured = Eigen::AngleAxisd(CaI_measured_angle, CaI_measured_axis.normalized());
    CqI_measured = Eigen::Quaterniond(CaI_measured);
    
    save_filename = "TagCamImuInit";
    save_data = true;
  }
};


/* Desired movement:
 * Move the machine moving it straight forward and backward, plus turning it among AprilTags.
 * Keep AprilTags so that camera views two/more tags simultaneously (connected tags). Try to keep
 * from losing the any tag views for more than one iteration interval.
 *
 * Limitations:
 * (1) Continuity loss of camera poses if no tags are seen for more than iteration interval.
 * (2) Camera poses tracked only for first connected tags set.
 * Limitation 2 is not a big deal as this is intended to only be used for brief periods ranging
 * from 10-60 seconds. Limitation 1 could be limiting as if tags are not seen for more than
 * iteration period, continuity between 'streaks' of motion will be lost. Every 'streak' will have
 * its own starting velocity when if motion was continuous ending velocity of last ending period
 * would be the same as starting velocity of next period.
 *
 * Future Improvement:
 * Limitation 1 can be easily avoided by keeping a sliding window of last IMU and camera poses.
 * Camera interpolation would be done in the sliding window with a size greater than the iteration
 * interval. Thus continuity can be maintained for longer intervals.
 */

class AprilTagImuInitCamera {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  // States of AprilTagCamera ImuInit Model
  enum TagCamImuInitState {kStarting, kCollectingData, kEstimating, kSleeping};
  
  // Data to keep tag poses across iterations
  struct Pose3d {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int64_t timestamp;
    Eigen::Vector3d posn;
    Eigen::Quaterniond quat;
    double info;
    Pose3d(): timestamp(0), posn(Eigen::Vector3d::Zero()), quat(Eigen::Quaterniond::Identity()), info(0.) {}
    Pose3d(const Pose3d& p): timestamp(p.timestamp), posn(p.posn), quat(p.quat), info(p.info) {}
    Pose3d(const CompositePose& cp): timestamp(0), posn(cp.posn), quat(cp.rotn_q), info(cp.info) {}
    Pose3d(const AprilTagCamera::CamCompositePose& cp): timestamp(cp.timestamp), posn(cp.pose.posn),
        quat(cp.pose.rotn_q), info(cp.pose.info) {}
  };
  
  // Data needed to run convex optimization to initiate the IMU
  struct ImuCamInitType {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Pose3d cam0;
    Pose3d cam1;
    anantak::ImuReadingType imu0;
    anantak::ImuReadingType imu1;
    anantak::ImuReadingsIntegralType imu_integral;
    Eigen::Vector3d solved_WpI;
    Eigen::Vector3d solved_WvI;
    Eigen::Vector3d reference_WpI;
    // Constructors
    ImuCamInitType(): cam0(), cam1(), imu0(), imu1(), imu_integral(),
      solved_WpI(Eigen::Vector3d::Zero()), solved_WvI(Eigen::Vector3d::Zero()),
      reference_WpI(Eigen::Vector3d::Zero()) {}
    ImuCamInitType(const ImuCamInitType& ic): cam0(ic.cam0), cam1(ic.cam1), imu0(ic.imu0),
      imu1(ic.imu1), imu_integral(ic.imu_integral),
      solved_WpI(Eigen::Vector3d::Zero()), solved_WvI(Eigen::Vector3d::Zero()),
      reference_WpI(Eigen::Vector3d::Zero()) {}
    // Constructor using TagCamera poses and IMU integrals
    ImuCamInitType(const Pose3d& p0, const Pose3d& p1, const anantak::ImuReadingType& i0,
      const anantak::ImuReadingType& i1, const anantak::ImuReadingsIntegralType& imui):
      cam0(p0), cam1(p1), imu0(i0), imu1(i1), imu_integral(imui),
      solved_WpI(Eigen::Vector3d::Zero()), solved_WvI(Eigen::Vector3d::Zero()),
      reference_WpI(Eigen::Vector3d::Zero()) {}
  };
  
  AprilTagImuInitCamera(
      const AprilTagCameraOptions&  cam_options,
      const anantak::SensorMsg&     calib_msg,
      const AprilTagImuInitOptions&    imu_options) :
    tag_camera_(cam_options, calib_msg), cam_options_(cam_options), imu_options_(imu_options),
    num_curr_imu_readings_(0), num_curr_tag_msgs_(0),
    num_enough_rotations_(0), num_enough_translations_(0),
    last_enough_translation_mark_(Eigen::Vector3d::Zero()),
    last_enough_rotation_mark_(Eigen::Quaterniond::Identity()),
    starting_timestamp_(0), enough_motion_achieved_timestamp_(0), curr_state_(kStarting),
    //,imu_positions_(0, NULL), imu_velocities_(0, NULL)
    imu_gravity_(Eigen::Vector3d::Zero()), imu_accel_bias_(Eigen::Vector3d::Zero()),
    imu_gravity_cov_(Eigen::Matrix3d::Zero()), imu_accel_bias_cov_(Eigen::Matrix3d::Zero()),
    imu_gravity_accel_bias_cov_(Eigen::Matrix3d::Zero()),
    imu_gravity_stdev_(Eigen::Vector3d::Zero()), imu_accel_bias_stdev_(Eigen::Vector3d::Zero())
    {
    // Allocate memory for imu readings and clear
    curr_imu_readings_.resize(imu_options_.max_imu_readings_per_iteration*2);
    curr_imu_readings_.clear();
    last_imu_readings_.resize(imu_options_.max_imu_readings_per_iteration);
    last_imu_readings_.clear();
    // Allocate memory for tag msgs and clear
    curr_tag_msgs_ts_.resize(cam_options_.max_num_of_cam_poses_per_iteration*2, 0);
    curr_tag_msgs_ts_.clear();
    last_tag_msgs_ts_.resize(cam_options_.max_num_of_cam_poses_per_iteration, 0);
    last_tag_msgs_ts_.clear();
    curr_tag_msgs_poses_.resize(cam_options_.max_num_of_cam_poses_per_iteration*2);
    curr_tag_msgs_poses_.clear();
    last_tag_msgs_poses_.resize(cam_options_.max_num_of_cam_poses_per_iteration);
    last_tag_msgs_poses_.clear();
    curr_tag_interp_.resize(cam_options_.max_num_of_cam_poses_per_iteration*2);
    curr_tag_interp_.clear();
    curr_tag_interp_imu_readings_.resize(cam_options_.max_num_of_cam_poses_per_iteration*2);
    curr_tag_interp_imu_readings_.clear();
    curr_imu_readings_integrals_.resize(cam_options_.max_num_of_cam_poses_per_iteration*2);
    curr_imu_readings_integrals_.clear();
    // Allocate memory for readings used for integration
    std::unique_ptr<anantak::CircularQueue<ImuCamInitType>> cq_ptr(new
        anantak::CircularQueue<ImuCamInitType>(imu_options_.max_readings_history));
    collected_readings_ = std::move(cq_ptr);
    // Optimized values
    imu_positions_.resize(0); imu_velocities_.resize(0);
    ref_positions_.resize(0);
    //integ_positions_.resize(0); integ_velocities_.resize(0);
  }

  virtual ~AprilTagImuInitCamera() {}
  
  /*bool CollectImuRotationsUsingAprilTags(int64_t initiation_interval,
      double  minimum_rotation_threshold, int32_t minimum_number_of_rotations) {
    CrI_estimation_interval = initiation_interval;
    int64_t time_elapsed = 0;
    int32_t number_of_rotations = 0;
    int32_t last_index = 0;
    int32_t curr_index = 1;
    // Goto a starting message with at least one tag sighting
    while (apriltag_msgs[last_index].april_msg().tag_id_size() < 1) {
      last_index++;
    }
    curr_index = last_index+1;
    while ((time_elapsed<initiation_interval || number_of_rotations<minimum_number_of_rotations) &&
        curr_index<imu_interp_apriltag_readings.size()) {
      // Check the angles between curr_index and last_index. If angle>threshold, store the indexes.
      // IMU Reading is WqI. We need I0qIi = WqI0^-1 (x) WqIi.
      QuaternionType dq = imu_interp_apriltag_readings[last_index].quaternion.conjugate() *
          imu_interp_apriltag_readings[curr_index].quaternion; 
      AngleAxisType aa(dq.conjugate());
      int32_t num_curr_tags = apriltag_msgs[curr_index].april_msg().tag_id_size();
      if (std::abs(aa.angle())>=minimum_rotation_threshold && num_curr_tags>0) {
        CollectedRotation cr;
        cr.index1 = last_index;
        cr.index2 = curr_index;
        cr.quaternion = dq;
        cr.aa = aa;
        CrI_collected_rotations.push_back(cr);
        number_of_rotations++;
        last_index = curr_index;
        curr_index++;
        VLOG(2) << "  " << "Indexes = " << cr.index1 << " " << cr.index2 << ", angle = " <<
            cr.aa.angle()*DegreesPerRadian << ", axis = " << cr.aa.axis().transpose();
      } else {
        curr_index++;
      }
      time_elapsed = imu_interp_apriltag_readings[curr_index].timestamp -
          imu_interp_apriltag_readings[0].timestamp;
      CrI_estimation_end_ts = imu_interp_apriltag_readings[curr_index].timestamp;
    }
    VLOG(1) << "Cam " << camera_num << ": Collected " << CrI_collected_rotations.size()
        << " rotations spanning " << double(time_elapsed)/1e6 << " seconds with min " <<
        minimum_rotation_threshold*DegreesPerRadian << " degrees rotation";
    CrI_estimation_interval = time_elapsed;
    return true;
  } // CollectImuRotationsUsingAprilTags*/

  // Copy Imu readings from imu messages into readings vector
  bool AppendImuReadingsFromImuMessages(const std::vector<anantak::SensorMsg>& msgs,
      std::vector<ImuReadingType>* imu_readings, int32_t max_num_to_copy) {
    int32_t sz = int32_t(msgs.size());
    int32_t num_to_copy = std::min(max_num_to_copy, sz);
    for (int i=sz-num_to_copy; i<sz; i++) {
      // make sure that the msg has imu_msg, if not, nothing is inserted
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
      }
    }
    VLOG(1) << "Copied " << num_to_copy << " imu readings from msgs";
    return true;
  }  // CopyImuReadingsFromImuMessages
  
  // Copy timestamps from tag msgs to ts where tag pose had enough information
  bool AppendTagPosesWithInformation(const std::vector<anantak::SensorMsg>& msgs,
        std::vector<int64_t>* ts, std::vector<Pose3d>* poses, int32_t max_num_to_copy) {
    int32_t sz = int32_t(msgs.size());
    int32_t num_to_copy = std::min(max_num_to_copy, sz);
    int32_t num_zero_info = 0;
    if (tag_camera_.curr_num_msgs_==sz) {
      for (int i=sz-num_to_copy; i<sz; i++) {
        if (tag_camera_.curr_cam_poses_[i].pose.info > anantak::Epsilon) {
          ts->push_back(int64_t(msgs[i].header().timestamp()));
          poses->emplace_back(tag_camera_.curr_cam_poses_[i]);
        } else {
          num_zero_info++;
        }
      }
    } else {
      LOG(ERROR) << "tag_camera_.curr_num_msgs_!=msgs.size()"
          << tag_camera_.curr_num_msgs_ << " " << sz;
      return false;
    }
    VLOG(1) << "Num tag poses with zero information = " << num_zero_info;
    return true;
  }

  // Integrate IMU readings between Camera readings
  bool IntegrateImuReadingsForTagCamera(const std::vector<anantak::SensorMsg>& imu_msgs,
      const std::vector<anantak::SensorMsg>& tag_msgs) {
    
    // Build curr_imu_readings_ to be interpolated
    num_curr_imu_readings_ = imu_msgs.size();
    curr_imu_readings_.clear();
    // Add last imu readings in the beginning
    for (int i=0; i<last_imu_readings_.size(); i++)
        curr_imu_readings_.push_back(last_imu_readings_[i]);
    // Copy imu readings from imu msgs into the vector
    AppendImuReadingsFromImuMessages(imu_msgs, &curr_imu_readings_,
        imu_options_.max_imu_readings_per_iteration);
    VLOG(1) << "curr_imu_readings_.size = " << curr_imu_readings_.size() << " imu readings";
    
    // Build tag readings to get interpolation timestamps
    num_curr_tag_msgs_ = tag_msgs.size();
    curr_tag_msgs_ts_.clear();
    curr_tag_msgs_poses_.clear();
    // Add last tag readings to this iteration's tag readings to be interpolated
    for (int i=0; i<last_tag_msgs_ts_.size(); i++) {
        curr_tag_msgs_ts_.push_back(last_tag_msgs_ts_[i]);
        curr_tag_msgs_poses_.push_back(last_tag_msgs_poses_[i]);
    }
    // Copy new tag message timestamps that have some information
    AppendTagPosesWithInformation(tag_msgs, &curr_tag_msgs_ts_, &curr_tag_msgs_poses_,
        cam_options_.max_num_of_cam_poses_per_iteration);
    VLOG(1) << "curr_tag_msgs_ts_.size = " << curr_tag_msgs_ts_.size() << " tag readings";
    
    // Interpolate IMU readings for tag msgs
    anantak::InterpolateImuReadings(curr_imu_readings_, curr_tag_msgs_ts_,
        &curr_tag_interp_, &curr_tag_interp_imu_readings_);
    VLOG(1) << "Interpolated " << curr_tag_interp_.size() << " imu readings";
    
    // Integrate imu readings between timestamps
    curr_imu_readings_integrals_.clear();
    if (curr_tag_interp_imu_readings_.size()>1) {
      for (int i=0; i<curr_tag_interp_imu_readings_.size()-1; i++) {
        //VLOG(1) << "    interp indexes = " << curr_tag_interp_[i].index << " " << curr_tag_interp_[i+1].index;
        anantak::ImuReadingsIntegralType iri;
        anantak::IntegrateImuKinematics(
          curr_imu_readings_,
          curr_tag_interp_imu_readings_[i],
          curr_tag_interp_imu_readings_[i+1],
          curr_tag_interp_[i].index+1,
          curr_tag_interp_[i+1].index,
          imu_options_.accel_factor,
          &iri
        );
        curr_imu_readings_integrals_.push_back(iri);
      }
    }
    VLOG(1) << "Integrated " << curr_imu_readings_integrals_.size() << " intervals";
    
    // Setup readings for next iteration
    // Find last interpolated tag index
    int32_t last_interp_tag_idx = curr_tag_msgs_ts_.size()-1;
    while (last_interp_tag_idx>=0 && curr_tag_interp_[last_interp_tag_idx].index<0)
        last_interp_tag_idx--;
    VLOG(1) << "Found last interpolated tag at idx " << last_interp_tag_idx;
    // Add all imu readings after and including last tag index to last imu readings
    last_imu_readings_.clear();
    last_tag_msgs_ts_.clear();
    last_tag_msgs_poses_.clear();    
    if (last_interp_tag_idx>=0) {
      // Add all imu readings after and including the interpolated tag camera
      for (int i=curr_tag_interp_[last_interp_tag_idx].index; i<curr_imu_readings_.size(); i++) {
        last_imu_readings_.push_back(curr_imu_readings_[i]);
      }
      // Add all tag camera msgs ts including and after the interpolated tag camera
      for (int i=last_interp_tag_idx; i<curr_tag_msgs_ts_.size(); i++) {
        last_tag_msgs_ts_.push_back(curr_tag_msgs_ts_[i]);
        last_tag_msgs_poses_.push_back(curr_tag_msgs_poses_[i]);
      }
    } else {
      // No tag reading was interpolated. We just drop everything.
      LOG(WARNING) << "No tag timestamps were interpolated!";
      num_enough_translations_ = std::max(0, num_enough_translations_-1);
      num_enough_rotations_ = std::max(0, num_enough_rotations_-1);
    }    
    VLOG(1) << last_imu_readings_.size() << " imu readings, " << last_tag_msgs_ts_.size()
        << " tag msgs were added for next iteration";
    
    // Save the integrations to the circular queue for imu init
    for (int i=0; i<curr_imu_readings_integrals_.size(); i++) {
      ImuCamInitType ici(curr_tag_msgs_poses_[i], curr_tag_msgs_poses_[i+1],
                         curr_tag_interp_imu_readings_[i], curr_tag_interp_imu_readings_[i+1],
                         curr_imu_readings_integrals_[i]);
      collected_readings_->add_element(ici);  // copy is created in the queue
    }
    VLOG(1) << "Saved " << curr_imu_readings_integrals_.size() << " integrals to queue, len = "
        << collected_readings_->n_msgs();
    
    return true;
  }  // IntegrateImuReadingsForTagCamera
  
  // Use the calculated camera poses to check if the motion is sufficient to init IMU
  bool MotionIsEnoughForImuInit() {
    bool enough_motion = false;
    if (!last_enough_translation_mark_.isZero()) {
      // Track rotations and translations
      for (int i=0; i<tag_camera_.curr_num_msgs_; i++) {
        if (tag_camera_.curr_cam_poses_[i].pose.info > anantak::Epsilon) {
          // TagCam readings are T0pCi, T0qCi. C0qC1 = T0qC0^-1 * T0qC1.
          Eigen::Vector3d d_posn = tag_camera_.curr_cam_poses_[i].pose.posn
              - last_enough_translation_mark_;
          Eigen::Quaterniond d_rotn = last_enough_rotation_mark_.conjugate()
              * tag_camera_.curr_cam_poses_[i].pose.rotn_q;
          double distance_travelled = d_posn.norm();
          //if (std::abs(d_rotn.coeffs().norm()-1.)>anantak::Epsilon) LOG(ERROR) << "d_rotn norm != 1";
          double angle_travelled = 2.*std::acos(std::abs(d_rotn.w()));
          //VLOG(1) << "distance/angle_travelled = " << distance_travelled << " " << angle_travelled*DegreesPerRadian;
          // If distance or angle travelled is more than what is to be tracked, move forward
          if (distance_travelled > imu_options_.min_enough_translation_distance) {
            num_enough_translations_++;
            last_enough_translation_mark_ = tag_camera_.curr_cam_poses_[i].pose.posn;
            VLOG(1) << "num_enough_translations_++ " << num_enough_translations_;
          }
          if (angle_travelled > imu_options_.min_enough_rotation_angle) {
            num_enough_rotations_++;
            last_enough_rotation_mark_ = tag_camera_.curr_cam_poses_[i].pose.rotn_q;
            VLOG(1) << "num_enough_rotations_++ " << num_enough_rotations_;
          }
        }
      }
    } else {
      int32_t i=0;
      while ((tag_camera_.curr_cam_poses_[i].pose.info < anantak::Epsilon)
          && (i < tag_camera_.curr_num_msgs_)) i++;
      if (tag_camera_.curr_cam_poses_[i].pose.info > anantak::Epsilon) {
        // Initiate tracking enough rotations and translations
        last_enough_translation_mark_ = tag_camera_.curr_cam_poses_[i].pose.posn;
        last_enough_rotation_mark_ = tag_camera_.curr_cam_poses_[i].pose.rotn_q;
        VLOG(1) << "last_enough_translation/rotation_mark_ were initiated";
      }
    }
    enough_motion = (num_enough_translations_ >= imu_options_.min_num_enough_translations)
        && (num_enough_rotations_ >= imu_options_.min_num_enough_rotations)
        && (collected_readings_->n_msgs() >= imu_options_.min_num_reading_history);
    return enough_motion;
  }
  
  // Run convex optimization to estimate IMU gravity and accelerometer biases.
  bool EstimateImuParameters() {
    /* Assumptions:
     *  (1) Imu rotations are assumed to be error free. This is to make optimization convex.
     *  (2) Imu-Camera pose is fixed to user-indicated approximate. This code is intended to be
     *      implemented on car-like machines. These undergo very restricted motion usually along
     *      a plane. As Imu accelerometer biases are unknown, we do not know motion direction.
     *      Hence rotations are only known along an axis for imu and cam. Due to this the rotation
     *      estimate is always off by an angle in the motion plane. This necessitates an indication
     *      of imu-cam rotation. As Imu biases are unknown imu's motion can not be estimated and
     *      so imu-cam translation can not be estiamted. So user to enters a tape measured estimate.
     *  (3) This implementation keeps the camera poses constant. Only IMU poses/velocities are
     *      refined. 
     *  Imu-Cam pose (rotation and translation) will be refined continuously in VIO. Here we need
     *  a starting estimate, so that we are close to actual values as VIO optimization is not
     *  convex and might get stuck in a local optimum.
     */
    int32_t num_readings = collected_readings_->n_msgs();
    VLOG(1) << "Beginning to estimate IMU gravity and accel biases using " << num_readings
        << " readings";
    
    // Check for streaks of continuous readings. For each streak starting posn will be 0,0,0 and
    //  Velocities are linked from one reading to next. Between streaks velo's are disconnected.
    int32_t num_streaks = 0;
    std::vector<std::vector<int32_t>> streak_indexes;
    int64_t last_timestamp = 0;
    for (int i_rdng=0; i_rdng<num_readings; i_rdng++) {
      const ImuCamInitType& rdng = collected_readings_->at(i_rdng);
      if (rdng.cam0.timestamp != last_timestamp) {
        std::vector<int32_t> si; si.resize(imu_options_.max_readings_history);
        streak_indexes.push_back(si); streak_indexes.back().clear();
        num_streaks++;
        streak_indexes.back().push_back(i_rdng);        
      } else {
        streak_indexes.back().push_back(i_rdng);        
      }
      last_timestamp = rdng.cam1.timestamp;
    }
    VLOG(1) << "Found " << num_streaks << " streak(s) of connected camera motions.";
    
    // Make a local modifiable copy of the readings' integrals. This step could be avoided just
    // by reusing the memory already allocated in circular queue. But modifying private memory
    // inside a circular queue can lead to unforeseen problems and crashes. Lets make a local copy.
    std::vector<anantak::ImuReadingsIntegralType> collected_readings_integrals;
    collected_readings_integrals.resize(num_readings);
    for (int i_rdng=0; i_rdng<num_readings; i_rdng++) {
      collected_readings_integrals[i_rdng] = collected_readings_->at(i_rdng).imu_integral;
    }
    VLOG(1) << "Made a local copy of the imu integrals";
    
    // Each streak of length n has n+1 positions and velocities. First position is set to 0.
    // Allocate memory for the keeping positions and velocities
    imu_positions_.resize(num_streaks); imu_velocities_.resize(num_streaks);
    ref_positions_.resize(num_streaks);
    //integ_positions_.resize(num_streaks); integ_velocities_.resize(num_streaks);
    for (int i_strk=0; i_strk<num_streaks; i_strk++) {
      std::unique_ptr<Eigen::Matrix<double,3,Eigen::Dynamic>>
          p_ptr(new Eigen::Matrix<double,3,Eigen::Dynamic>);
      imu_positions_[i_strk] = std::move(p_ptr);
      imu_positions_[i_strk]->resize(3,streak_indexes[i_strk].size());
      imu_positions_[i_strk]->setZero();
      std::unique_ptr<Eigen::Matrix<double,3,Eigen::Dynamic>>
          v_ptr(new Eigen::Matrix<double,3,Eigen::Dynamic>);
      imu_velocities_[i_strk] = std::move(v_ptr);
      imu_velocities_[i_strk]->resize(3,streak_indexes[i_strk].size());
      imu_velocities_[i_strk]->setZero();
      std::unique_ptr<Eigen::Matrix<double,3,Eigen::Dynamic>>
          r_ptr(new Eigen::Matrix<double,3,Eigen::Dynamic>);
      ref_positions_[i_strk] = std::move(r_ptr);
      ref_positions_[i_strk]->resize(3,streak_indexes[i_strk].size());
      ref_positions_[i_strk]->setZero();
      /*std::unique_ptr<Eigen::Matrix<double,3,Eigen::Dynamic>>
          ip_ptr(new Eigen::Matrix<double,3,Eigen::Dynamic>);
      integ_positions_[i_strk] = std::move(ip_ptr);
      integ_positions_[i_strk]->resize(3,streak_indexes[i_strk].size());
      integ_positions_[i_strk]->setZero();
      std::unique_ptr<Eigen::Matrix<double,3,Eigen::Dynamic>>
          iv_ptr(new Eigen::Matrix<double,3,Eigen::Dynamic>);
      integ_velocities_[i_strk] = std::move(iv_ptr);
      integ_velocities_[i_strk]->resize(3,streak_indexes[i_strk].size());
      integ_velocities_[i_strk]->setZero();*/
      VLOG(1) << "Allocated memory for imu and ref positions, velocities for streak " << i_strk
          << " of size " << streak_indexes[i_strk].size();
    }
    imu_gravity_.setZero(); imu_accel_bias_.setZero();
    double* grav_ptr = imu_gravity_.data();
    double* bias_ptr = imu_accel_bias_.data();
    
    // Setup the problem
    //  IMU residuals express the posn and velo of IMU wrt starting IMU pose
    //  Each IMU pose is translated to camera pose using user-provided CpI, CrI. Residual is 
    //    measured position of tag camera minus predicted value wrt starting pose.
    //    Use pixel-based variance for residuals.
    ceres::Problem problem;
    
    // Add constraints for IMU position/velocity for each streak
    for (int i_strk=0; i_strk<num_streaks; i_strk++) {
      
      // Positions and velocities are expressed in starting imu pose for the streak
      // Transform all imu integrals of this streak to starting imu pose by pre-mult by WrI^-1
      Eigen::Matrix3d streak_start_WrB = 
          collected_readings_->at(streak_indexes[i_strk].front()).imu0.quaternion.toRotationMatrix();
      //Eigen::Matrix3d streak_start_BrW = Eigen::Matrix3d::Identity(); //streak_start_WrB.transpose(); 
      //for (int i_rdng=streak_indexes[i_strk].front(); i_rdng<streak_indexes[i_strk].back(); i_rdng++) {
      //  collected_readings_integrals[i_rdng].LeftMultiplyRotationMatrix(streak_start_BrW);
      //}
      Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
      // Add IMU position/velocity constraints
      for (int i_rdng=streak_indexes[i_strk].front(); i_rdng<streak_indexes[i_strk].back(); i_rdng++) {
        ceres::CostFunction* pose_constraint =
            new ConvexImuResidualFunction(
              &collected_readings_integrals[i_rdng],
              &I3, //&streak_start_WrB,
              imu_options_.gravity_magnitude,
              imu_options_.sigma_accel,
              imu_options_.sigma_gravity
            );
        ceres::LossFunction* quadratic_loss = 
            NULL;
        int32_t i_locn = i_rdng - streak_indexes[i_strk].front();
        problem.AddResidualBlock(
          pose_constraint,
          quadratic_loss,
          imu_positions_[i_strk]->data() + 3*i_locn,      // posn0 estimated in streak base frame
          imu_positions_[i_strk]->data() + 3*(i_locn+1),  // posn1
          imu_velocities_[i_strk]->data() + 3*i_locn,     // velo0
          imu_velocities_[i_strk]->data() + 3*(i_locn+1), // velo1
          grav_ptr,   // grav is estimated in the IMU inertial frame
          bias_ptr    // bias is estimated in the IMU body frame
        );
      }
      
      // Set the starting position for each streak as constant
      problem.SetParameterBlockConstant(imu_positions_[i_strk]->data());
      
      // Add tag camera pose constraints
      const Pose3d& C0_pose = collected_readings_->at(streak_indexes[i_strk].front()).cam0;
      
      for (int i_rdng=streak_indexes[i_strk].front(); i_rdng<streak_indexes[i_strk].back(); i_rdng++) {
        const Pose3d& Ci_pose = collected_readings_->at(i_rdng).cam0;
        int32_t i_locn = i_rdng - streak_indexes[i_strk].front();
       
        Eigen::Matrix3d T0rC0(C0_pose.quat);
        Eigen::Matrix3d T0rCi(Ci_pose.quat);
        Eigen::Matrix3d CrI(imu_options_.CqI_measured);
        // I0pIi = IrC * C0rT0 * ( T0pCi - T0pC0 + (T0rCi-T0rC0)*CpI )
        // WpIi = WrI0 * (I0pIi - I0pW)  where I0pW = 0;
        Eigen::Vector3d WpIi = streak_start_WrB * CrI.transpose() * T0rC0.transpose() *
            (Ci_pose.posn-C0_pose.posn + (T0rCi-T0rC0)*imu_options_.CpI_measured);
        ref_positions_[i_strk]->col(i_locn) = WpIi;
        
        ceres::CostFunction* posn_constraint = 
            new FixedCameraPoseResidualFunction(
              //&C0_pose.posn, &C0_pose.quat,
              //&Ci_pose.posn, &Ci_pose.quat,
              //&imu_options_.CpI_measured, &imu_options_.CqI_measured,
              &WpIi, &Ci_pose.info
            );
        ceres::LossFunction* huber_loss = NULL;
            //new ceres::HuberLoss(1.0);
        problem.AddResidualBlock(
          posn_constraint,
          huber_loss,
          imu_positions_[i_strk]->data() + 3*i_locn
        );
      }
    } // i_strk
    
    // Solve problem
    ceres::Solver::Options options;
    options.max_num_iterations = 300;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    if (true) std::cout << summary.FullReport() << std::endl;
    
    // Calculate variance of gravity and accel biases.
    ceres::Covariance::Options covariance_options;
    ceres::Covariance covariance(covariance_options);
    std::vector<std::pair<const double*, const double*>> covariance_blocks;
    covariance_blocks.push_back(std::make_pair(grav_ptr, grav_ptr));
    covariance_blocks.push_back(std::make_pair(bias_ptr, grav_ptr));
    covariance_blocks.push_back(std::make_pair(bias_ptr, bias_ptr));
    CHECK(covariance.Compute(covariance_blocks, &problem));
    covariance.GetCovarianceBlock(grav_ptr, grav_ptr, imu_gravity_cov_.data());
    covariance.GetCovarianceBlock(bias_ptr, bias_ptr, imu_accel_bias_cov_.data());
    covariance.GetCovarianceBlock(grav_ptr, bias_ptr, imu_gravity_accel_bias_cov_.data());
    
    imu_gravity_stdev_ = imu_gravity_cov_.diagonal().cwiseSqrt();
    imu_accel_bias_stdev_ = imu_accel_bias_cov_.diagonal().cwiseSqrt();
    
    // Report results
    VLOG(1) << "IMU Init estimates:";
    VLOG(1) << "Gravity = " << imu_gravity_.transpose() << ", " << imu_gravity_.norm() << " (m/s^2)";
    VLOG(1) << "Gravity stdev = " << imu_gravity_stdev_.transpose() << " (m/s^2)";
    VLOG(1) << "Accel biases = " << imu_accel_bias_.transpose() << " (m/s^2)";
    VLOG(1) << "Accel biases stdev = " << imu_accel_bias_stdev_.transpose() << " (m/s^2)";
    VLOG(1) << "Gravity = " << imu_gravity_.transpose()/imu_options_.accel_factor << ", "
        << imu_gravity_.norm()/imu_options_.accel_factor << " (LSB)";
    VLOG(1) << "Gravity stdev = " << imu_gravity_stdev_.transpose()/imu_options_.accel_factor << " (LSB)";
    VLOG(1) << "Accel biases = " << imu_accel_bias_.transpose()/imu_options_.accel_factor << " (LSB)";
    VLOG(1) << "Accel biases stdev = " << imu_accel_bias_stdev_.transpose()/imu_options_.accel_factor << " (LSB)";
    
    // Report gravity in starting base frame of first streak
    Eigen::Vector3d imu_grav_streak0_base =
        collected_readings_->at(streak_indexes.front().front()).imu0.quaternion.toRotationMatrix().transpose() * imu_gravity_;
    VLOG(1) << "Gravity in streak0 starting frame= "
        << imu_grav_streak0_base.transpose()/imu_options_.accel_factor << ", "
        << imu_grav_streak0_base.norm()/imu_options_.accel_factor << " (LSB)";
    
    // In order to use the position and velocity estimates to initiate VIO, save back to readings
    VLOG(1) << "Writing solved positions and velocities to collected readings.";
    for (int i_strk=0; i_strk<num_streaks; i_strk++) {
      //Eigen::Matrix3d streak_start_WrB = Eigen::Matrix3d::Identity();
          //collected_readings_->at(streak_indexes[i_strk].front()).imu0.quaternion.toRotationMatrix();
      for (int i_rdng=streak_indexes[i_strk].front(); i_rdng<streak_indexes[i_strk].back()+1; i_rdng++) {
        int32_t i_locn = i_rdng - streak_indexes[i_strk].front();
        collected_readings_->at(i_rdng).reference_WpI = ref_positions_[i_strk]->col(i_locn);
        collected_readings_->at(i_rdng).solved_WpI = imu_positions_[i_strk]->col(i_locn);
        collected_readings_->at(i_rdng).solved_WvI = imu_velocities_[i_strk]->col(i_locn);
      }
    } // i_strk
    
    // Integrate the position and velocity forward using calculated gravity and accel bias
    //VLOG(1) << "Integrating positions and velocities using solved values.";
    // Can be done. May be when really needed.
    
    return true;
  }
  
  // In every iteration, get the msgs from IMU and from reference camera. Collect motions.
  bool ProcessImuAndAprilTagMessages(const std::vector<anantak::SensorMsg>& imu_msgs,
      const std::vector<anantak::SensorMsg>& tag_msgs) {
    if (curr_state_ != kSleeping) {
      if (curr_state_ == kStarting) curr_state_ = kCollectingData;
      if (starting_timestamp_==0) starting_timestamp_=imu_msgs.front().header().timestamp();
      
      // Process tag camera messages
      tag_camera_.ProcessAprilTagMessages(tag_msgs);
      
      // Interpolate and integrate imu timestamps
      IntegrateImuReadingsForTagCamera(imu_msgs, tag_msgs);
      
      // Check if enough motion has been seen. If so, run convex optimization for IMU parameters
      if (MotionIsEnoughForImuInit()) {
        if (enough_motion_achieved_timestamp_==0)
            enough_motion_achieved_timestamp_ = imu_msgs.front().header().timestamp();
        VLOG(1) << "Enough motion to init IMU achieved in " <<
            double(enough_motion_achieved_timestamp_ - starting_timestamp_)*1e-6 << "(s)";
            
        curr_state_ = kEstimating;
        if (!EstimateImuParameters()) {
          VLOG(1) << "Could not estimate IMU parameters. Trying again.";
          curr_state_ = kStarting;
          num_enough_rotations_ = 0;
          num_enough_translations_ = 0;
          starting_timestamp_ = 0;
          enough_motion_achieved_timestamp_ = 0;
          last_enough_translation_mark_ = Eigen::Vector3d::Zero();
          last_enough_rotation_mark_ = Eigen::Quaterniond::Identity();
          imu_positions_.clear(); imu_velocities_.clear();
        } else {
          VLOG(1) << "Successfully estimated IMU parameters, entering sleeping mode.";
          curr_state_ = kSleeping;
        }
      }
    } else {
      VLOG(1) << "Sleeping, so neglecting data.";
    }
    return true;
  }
  
  bool IsSleeping() {
    return (curr_state_==kSleeping); 
  }

  bool SaveDataToFile(const std::string& save_filename) {
    // Save camera poses to file
    tag_camera_.SavePosesToFile(save_filename+".TagsOnly");
    // Save imu data to file. Only the first streak is written.
    anantak::WriteMatrixToCSVFile(save_filename+".ref.posns", ref_positions_[0]->transpose());
    anantak::WriteMatrixToCSVFile(save_filename+".imu.posns", imu_positions_[0]->transpose());
    anantak::WriteMatrixToCSVFile(save_filename+".imu.velos", imu_velocities_[0]->transpose());
    //anantak::WriteMatrixToCSVFile(save_filename+".imu.integ.posns", integ_positions_[0]->transpose());
    //anantak::WriteMatrixToCSVFile(save_filename+".imu.integ.velos", integ_velocities_[0]->transpose());
    return true;
  }
  
  // Data members
  int64_t starting_timestamp_;
  TagCamImuInitState curr_state_;
  AprilTagCameraOptions cam_options_;
  AprilTagImuInitOptions imu_options_;
  AprilTagCamera tag_camera_;         // AprilTag camera used in imu initiation  
  
  int32_t num_curr_imu_readings_;
  std::vector<anantak::ImuReadingType> last_imu_readings_;
  std::vector<anantak::ImuReadingType> curr_imu_readings_;  
  int32_t num_curr_tag_msgs_;
  std::vector<int64_t> last_tag_msgs_ts_;
  std::vector<int64_t> curr_tag_msgs_ts_;
  std::vector<Pose3d> last_tag_msgs_poses_;
  std::vector<Pose3d> curr_tag_msgs_poses_;

  std::vector<anantak::LinearInterpolation> curr_tag_interp_;
  std::vector<anantak::ImuReadingType> curr_tag_interp_imu_readings_;
  std::vector<anantak::ImuReadingsIntegralType> curr_imu_readings_integrals_;
  
  int32_t num_enough_rotations_, num_enough_translations_;
  int64_t enough_motion_achieved_timestamp_;
  Eigen::Vector3d last_enough_translation_mark_;
  Eigen::Quaterniond last_enough_rotation_mark_;

  std::unique_ptr<anantak::CircularQueue<ImuCamInitType>> collected_readings_;

  // Final estimates are stored here - these are at camera timestamps
  std::vector<std::unique_ptr<Eigen::Matrix<double,3,Eigen::Dynamic>>>
      imu_positions_, imu_velocities_, ref_positions_; //, integ_positions_, integ_velocities_;
  Eigen::Vector3d imu_gravity_, imu_accel_bias_;
  Eigen::Matrix3d imu_gravity_cov_, imu_accel_bias_cov_, imu_gravity_accel_bias_cov_;
  Eigen::Vector3d imu_gravity_stdev_, imu_accel_bias_stdev_;
  
}; // AprilTagImuInitCamera


/* TagVIO11
 * Runs VIO using one IMU and one tag camera.
 * IMU sends readings of timestamp, IMU quaternion (in world frame), acceleration (body frame).
 * Tag camera sends readings with timestamps, all tags seen in an image.
 * Prior for imu state for gravity and acceleration and tag maps are used for initialization
 */

class TagVIO11 {
 public:
  
  struct Options {
    int32_t state_frequency;   /**< IMU states will be created at this frequency using wall time */
    int32_t imu_frequency;     /**< Frequency of IMU readings in Hz */
    int32_t max_states_history_queue_size; /**< Size of queue of states history */
    int32_t max_tags_in_map;  /** Maximum number of recognizable tags in map */
    
    // VIO initiation - how much maximum history of IMU readings and Camera poses to use?
    int32_t max_imu_readings_for_initiation;
    int32_t max_cam_readings_for_initiation;
    int32_t max_april_tag_readings_for_initiation;
    
    // Imu integration options
    ImuIntegrationOptions integration_options;
    
    // Starting state options - these are all not necessarily used
    Eigen::Quaterniond starting_IqG;
    Eigen::Vector3d starting_GpI;
    Eigen::Vector3d starting_GvI;
    Eigen::Vector3d starting_bg;
    Eigen::Vector3d starting_ba;
    
    // April tag and camera options
    double default_april_tag_size;
    double default_april_tag_size_sigma;
    double default_sigma_im;
    double starting_tag_angle_sigma;
    double starting_tag_position_sigma;
    
    // This is where we set the image stdev
    AprilTagViewResidual::Options apriltag_view_residual_options;
    AprilTagVioResidual::Options apriltag_vio_residual_options;  
    
    // Camera to IMU pose residual options
    NoisyPosePrior::Options imu_to_cam_pose_prior_options;
    anantak::NoisyPoseResidual::Options imu_to_cam_pose_noise_options;
    anantak::NoisyPoseResidual::Options imu_to_cam_pose_change_options;
    
    // Camera to IMU pose residual options
    anantak::RigidPoseWithImuResidual::Options rigid_imu_to_cam_pose_options;
    
    // Saving data options
    std::string save_filename;
    
    // Camera number for this VIO11
    int32_t camera_num;
    int32_t imu_num;
    
    // Options that usually never change
    int32_t tags_set_num;        /**< Connect tag set map number used for VIO init. Keep at 0. */
    
    Options(): integration_options(), apriltag_view_residual_options(), apriltag_vio_residual_options(),
        imu_to_cam_pose_noise_options(), imu_to_cam_pose_change_options(),
        rigid_imu_to_cam_pose_options() {
      state_frequency = 25;
      imu_frequency = 100;
      max_states_history_queue_size = 1000;
      max_tags_in_map = 500;
      
      // VIO initiation - using 100Hz for IMU readings and 30Hz for Camera, max history for 5min
      max_imu_readings_for_initiation = 5*60*100;
      max_cam_readings_for_initiation = 5*60*30;
      max_april_tag_readings_for_initiation = 5*60*30*10;   // assuming 10 tag views per image max
      
      // Integration options - any special settings
      // integration_options...
      
      // VIO options
      apriltag_view_residual_options.sigma_image = 1.0;
      apriltag_vio_residual_options.q_image = 0.5;
      
      // Starting state options
      starting_IqG = Eigen::Quaterniond::Identity();
      starting_GpI = Eigen::Vector3d::Zero();
      starting_GvI = Eigen::Vector3d::Zero();
      starting_bg  = Eigen::Vector3d::Zero();
      starting_ba  = Eigen::Vector3d::Zero();
      
      default_april_tag_size = 0.4780; // meters
      default_april_tag_size_sigma = 0.010/3.0; // meters
      default_sigma_im = 0.5; // pixels
      starting_tag_angle_sigma = 20*RadiansPerDegree; // radians
      starting_tag_position_sigma = 1.0; // m
      
      // Camera to imu pose difference from prior 
      imu_to_cam_pose_prior_options.sigma_theta = 20.*RadiansPerDegree;   // in Radians
      imu_to_cam_pose_prior_options.sigma_position = 0.050;               // in meters
      
      // IMU to cam rigid pose options
      rigid_imu_to_cam_pose_options.rate_of_change = false;              // Not a rate of change
      rigid_imu_to_cam_pose_options.sigma_theta = 5.*RadiansPerDegree;  // in Radians
      rigid_imu_to_cam_pose_options.sigma_position = 0.010;              // in meters
      
      // Camera to IMU pose noise options - change of pose from mean pose
      imu_to_cam_pose_noise_options.rate_of_change = false;              // Not a rate of change
      imu_to_cam_pose_noise_options.sigma_theta = 5.*RadiansPerDegree;   // in Radians
      imu_to_cam_pose_noise_options.sigma_position = 0.010;              // in meters
      
      // Camera to IMU pose change options - change of pose from last pose
      imu_to_cam_pose_change_options.rate_of_change = true;               // This is a rate of change
      imu_to_cam_pose_change_options.sigma_theta = 5.*RadiansPerDegree;   // in Radians/sqrt(Hz)
      imu_to_cam_pose_change_options.sigma_position = 0.005;              // in meters/sqrt(Hz)
      
      // Camera number for the VIO-IMU pair
      camera_num = 0;
      imu_num = 0;
      
      save_filename = "";
      
      // Fixed options
      tags_set_num = 0;
    }
    
    bool SetSaveFilename(const std::string& name) {
      save_filename = name;
      return true;
    }
  };

  TagVIO11::Options options_;
  anantak::ImuToCameraPoseResidual::Options imu_cam_residual_options_;

  enum TagCameraVIO11State {
    kInitiateIMU, kInitiateVIO, kRunningVIO
  };

  TagCameraVIO11State state_;

  // Imu init tag camera
  std::unique_ptr<AprilTagImuInitCamera> imu_init_tag_camera_;
  
  // Memory for initiation data
  struct VIOInitCamReadings {
    AprilTagImuInitCamera::Pose3d cam_pose; // Camera pose in tag frame
    anantak::ImuReadingType imu_interp;     // IMU reading at camera timestamp
    // Constructors
    VIOInitCamReadings(): cam_pose(), imu_interp() {}
    VIOInitCamReadings(const AprilTagImuInitCamera::Pose3d& cp, const anantak::ImuReadingType& ir):
      cam_pose(cp), imu_interp(ir) {}
  };
  std::unique_ptr<anantak::CircularQueue<VIOInitCamReadings>> init_cam_readings_;
  std::unique_ptr<anantak::CircularQueue<anantak::ImuReadingType>> init_imu_readings_;
  std::unique_ptr<anantak::CircularQueue<anantak::AprilTagReadingType>> april_tag_readings_;
  int32_t init_cam_data_starting_idx_, init_imu_data_starting_idx_;
  
  // States to be calculated
  std::unique_ptr<anantak::CircularQueue<anantak::ImuState>> imu_states_;
  std::unique_ptr<anantak::CircularQueue<anantak::ImuResidualFunction>> imu_residuals_;
  std::unique_ptr<anantak::CircularQueue<anantak::Pose3dState>> cam_poses_;
  std::unique_ptr<anantak::CircularQueue<anantak::RigidPoseWithImuResidual>> rigid_imu_to_cam_pose_residuals_;
  std::unique_ptr<anantak::CircularQueue<anantak::AprilTagViewResidual>> tag_view_residuals_;
  
  std::unique_ptr<anantak::CircularQueue<anantak::Pose3dState>> cam_to_imu_poses_;
  std::unique_ptr<anantak::CircularQueue<anantak::ImuToCameraPoseResidual>> imu_cam_pose_residuals_;  
  std::unique_ptr<anantak::CircularQueue<anantak::NoisyPoseResidual>> noisy_imu_to_cam_pose_residuals_;
  std::unique_ptr<anantak::CircularQueue<anantak::NoisyPoseResidual>> noisy_imu_to_cam_pose_change_residuals_;
  anantak::Vector3dState gravity_state_;
  std::unique_ptr<anantak::CircularQueue<anantak::StaticAprilTagState>> tag_poses_;
  anantak::Pose3dState cam_to_imu_pose_;
  anantak::Pose3dState tag_map_to_imu_world_pose_;
  anantak::Pose3dState tag_map_pose_;
  anantak::CameraIntrinsicsState camera_intrinsics_;
  std::unique_ptr<anantak::CircularQueue<anantak::AprilTagVioResidual>> tag_vio_residuals_;
  
  // Other variables
  int32_t num_imu_readings_per_residual_;   // Number of readings per IMU residual
  int64_t state_period_;      // Time in micosecs between consecutive states 
  
  // Default constructor
  TagVIO11(
      const AprilTagCameraOptions&  cam_options,
      const anantak::SensorMsg&     calib_msg,
      const AprilTagImuInitOptions& imu_options,
      const TagVIO11::Options&      tagvio11_options):
    options_(tagvio11_options), state_(kInitiateIMU),
    imu_cam_residual_options_(), 
    init_cam_data_starting_idx_(0), init_imu_data_starting_idx_(0) {
    
    // Create the IMU initiator tag camera
    std::unique_ptr<AprilTagImuInitCamera> ptr(new
        AprilTagImuInitCamera(cam_options, calib_msg, imu_options));
    imu_init_tag_camera_ = std::move(ptr);
    
    // Allocate memory for storing initiation data
    std::unique_ptr<anantak::CircularQueue<VIOInitCamReadings>> cq_cam_ptr(new
        anantak::CircularQueue<VIOInitCamReadings>(options_.max_cam_readings_for_initiation));
    init_cam_readings_ = std::move(cq_cam_ptr);
    std::unique_ptr<anantak::CircularQueue<anantak::ImuReadingType>> cq_imu_ptr(new
        anantak::CircularQueue<anantak::ImuReadingType>(options_.max_imu_readings_for_initiation));
    init_imu_readings_ = std::move(cq_imu_ptr);
    // Allocate memory for april tag readings
    std::unique_ptr<anantak::CircularQueue<anantak::AprilTagReadingType>> cq_tag_rdng_ptr(new
        anantak::CircularQueue<anantak::AprilTagReadingType>(options_.max_april_tag_readings_for_initiation));
    april_tag_readings_ = std::move(cq_tag_rdng_ptr);
    
    // Allocate memory for storing imu states
    std::unique_ptr<anantak::CircularQueue<anantak::ImuState>> cq_imu_states_ptr(new
        anantak::CircularQueue<anantak::ImuState>(options_.max_cam_readings_for_initiation));
    imu_states_ = std::move(cq_imu_states_ptr);
    
    // IMU residuals
    num_imu_readings_per_residual_ = options_.imu_frequency / options_.state_frequency + 1;
    anantak::ImuResidualFunction prototype_imu_residual(num_imu_readings_per_residual_);
    VLOG(1) << "Creating imu residuals with history length = " << num_imu_readings_per_residual_;
    std::unique_ptr<anantak::CircularQueue<anantak::ImuResidualFunction>> cq_imu_resids_ptr(new
        anantak::CircularQueue<anantak::ImuResidualFunction>(
            options_.max_cam_readings_for_initiation, prototype_imu_residual));
    imu_residuals_ = std::move(cq_imu_resids_ptr);
    gravity_state_.SetZero();
    
    // Allocate memory for camera pose states and residuals
    std::unique_ptr<anantak::CircularQueue<anantak::Pose3dState>> cq_cam_states_ptr(new
        anantak::CircularQueue<anantak::Pose3dState>(options_.max_cam_readings_for_initiation));
    cam_poses_ = std::move(cq_cam_states_ptr);
    std::unique_ptr<anantak::CircularQueue<anantak::ImuToCameraPoseResidual>> cq_imucam_resids_ptr(new
        anantak::CircularQueue<anantak::ImuToCameraPoseResidual>(options_.max_cam_readings_for_initiation));
    imu_cam_pose_residuals_ = std::move(cq_imucam_resids_ptr);
    cam_to_imu_pose_.SetZero();
    std::unique_ptr<anantak::CircularQueue<anantak::StaticAprilTagState>> cq_tag_poses_ptr(new
        anantak::CircularQueue<anantak::StaticAprilTagState>(options_.max_tags_in_map));
    tag_poses_ = std::move(cq_tag_poses_ptr);
    tag_map_pose_.SetZero();
    camera_intrinsics_.SetZero();
    
    // Allocate memory for tag vio residuals
    std::unique_ptr<anantak::CircularQueue<anantak::AprilTagVioResidual>> cq_tag_vio_residuals(new
        anantak::CircularQueue<anantak::AprilTagVioResidual>(options_.max_cam_readings_for_initiation));
    tag_vio_residuals_ = std::move(cq_tag_vio_residuals);
    
    // Allocate memory for cam-to-imu poses
    std::unique_ptr<anantak::CircularQueue<anantak::Pose3dState>> cq_cam_to_imu_poses_ptr(new
        anantak::CircularQueue<anantak::Pose3dState>(options_.max_cam_readings_for_initiation));
    cam_to_imu_poses_ = std::move(cq_cam_to_imu_poses_ptr);
    
    // Allocate memory for noisy cam-to-imu pose residuals
    std::unique_ptr<anantak::CircularQueue<anantak::NoisyPoseResidual>> cq_noisy_imu_to_cam_pose_residuals_ptr(new
        anantak::CircularQueue<anantak::NoisyPoseResidual>(options_.max_cam_readings_for_initiation));
    noisy_imu_to_cam_pose_residuals_ = std::move(cq_noisy_imu_to_cam_pose_residuals_ptr);
    
    // Allocate memory for imu-to-cam pose change residuals
    std::unique_ptr<anantak::CircularQueue<anantak::NoisyPoseResidual>> cq_noisy_imu_to_cam_pose_change_residuals_ptr(new
        anantak::CircularQueue<anantak::NoisyPoseResidual>(options_.max_cam_readings_for_initiation));
    noisy_imu_to_cam_pose_change_residuals_ = std::move(cq_noisy_imu_to_cam_pose_change_residuals_ptr);
    
    // RigidPoseWithImuResidual cq
    std::unique_ptr<anantak::CircularQueue<anantak::RigidPoseWithImuResidual>> cq_rigid_imu_to_cam_pose_residuals_ptr(new
        anantak::CircularQueue<anantak::RigidPoseWithImuResidual>(options_.max_cam_readings_for_initiation));
    rigid_imu_to_cam_pose_residuals_ = std::move(cq_rigid_imu_to_cam_pose_residuals_ptr);
    
    // Allocate memory for tag view residuals
    std::unique_ptr<anantak::CircularQueue<anantak::AprilTagViewResidual>> cq_tag_view_residuals(new
        anantak::CircularQueue<anantak::AprilTagViewResidual>(options_.max_cam_readings_for_initiation));
    tag_view_residuals_ = std::move(cq_tag_view_residuals);
    
    // Wall-clock states
    state_period_ = 100000 / options_.state_frequency;    // integer division
    
  }
  
  virtual ~TagVIO11() {}
  
  /* Keep a circular queue for each message type - imu and tag pose. A state is initiated each
   * time a message comes. After that the state is estimated. Then error states for each state
   * are created and optimized. Optimization is done by creating a problem to which the state is
   * provided as it implements the Evaluate() method for the error states. VIO will create a
   * problem in every iteration, add the relevant states to the problem, solve it. Then it will
   * re-calculate each state, but the jacobians will be kept constant. VIO then marginalizes the
   * older states, establishes priors. It is then ready for next batch of states. 
   */
  
  /* Allocate
   *  Circular queue for IMU states
   *  Circular queue for tag poses
   *  Camera-to-IMU pose state
   *  Circular queue for IMU reading constraints
   *  Circular queue for TagView constraints
   *  Priors for
   *    Camera-to-IMU pose
   *    Gravity magnitude (?)
   *    Starting poses (?)
   */
  
  /* After every iteration of the imu initiator tag camera, collect data:
   * Begin camera state, ending camera state, state readings and inbetween imu readings
   * This is used at initiation to run a non-convex optimization but using convex estimates.
   */
  bool CollectDataForInitiation(const std::vector<anantak::SensorMsg>& imu_msgs,
      const std::vector<anantak::SensorMsg>& tag_msgs) {
    
    // In IMU init tag camera,
    // number_of_constraints = curr_tag_interp_imu_readings_.size()-1
    // if number_of_constraints>0 then i=0 to number_of_constraints:
    // Begin camera state in tags frame = curr_tag_msgs_poses_[i]
    // Ending camera state in tags frame = curr_tag_msgs_poses_[i+1]
    // Begin camera state interpolated imu reading = curr_tag_interp_imu_readings_[i]
    // Ending camera state interpolated imu reading = curr_tag_interp_imu_readings_[i+1]
    // IMU readings = curr_imu_readings_
    // Intermediate imu readings from/to = curr_tag_interp_[i].index+1, curr_tag_interp_[i+1].index
    
    int32_t num_of_constraints = imu_init_tag_camera_->curr_tag_interp_imu_readings_.size()-1;
    VLOG(1) << "Saw " << num_of_constraints << " constraints in this iteration";
    
    if (num_of_constraints>0) {
      // Save states and constraints for later initialization 
      for (int i_ctr=0; i_ctr<num_of_constraints; i_ctr++) {
        // if this is the first reading, save begin cam reading and imu reading before the cam rdng
        if (init_cam_readings_->n_msgs()==0) {
          VIOInitCamReadings cr0(imu_init_tag_camera_->curr_tag_msgs_poses_[i_ctr],
              imu_init_tag_camera_->curr_tag_interp_imu_readings_[i_ctr]); // first copy
          init_cam_readings_->add_element(cr0); // second copy into queue. Might be a better way?
          // Add the starting imu reading to the queue
          if (imu_init_tag_camera_->curr_tag_interp_[i_ctr].index>=0) {
            init_imu_readings_->add_element(
                imu_init_tag_camera_->curr_imu_readings_[
                imu_init_tag_camera_->curr_tag_interp_[i_ctr].index]);
          }
        }
        // Save ending cam reading
        VIOInitCamReadings cr1(imu_init_tag_camera_->curr_tag_msgs_poses_[i_ctr+1],
            imu_init_tag_camera_->curr_tag_interp_imu_readings_[i_ctr+1]); // first copy
        init_cam_readings_->add_element(cr1); // second copy into queue. Might be a better way?
        // Save all intermediate imu readings
        for (int i_imu_rdng=imu_init_tag_camera_->curr_tag_interp_[i_ctr].index+1;
            i_imu_rdng<imu_init_tag_camera_->curr_tag_interp_[i_ctr+1].index+1; i_imu_rdng++) {
          init_imu_readings_->add_element(imu_init_tag_camera_->curr_imu_readings_[i_imu_rdng]);
        }
      } // for each integral
    } // if any integrals were there
    
    // Save April Tag sightings for initiation
    for (int i_msg=0; i_msg<tag_msgs.size(); i_msg++) {
      if (tag_msgs[i_msg].has_header() && tag_msgs[i_msg].has_april_msg()) {
        const anantak::AprilTagMessage& apriltag_msg = tag_msgs[i_msg].april_msg();
        //VLOG(3) << "Number of AprilTags in msg = " << apriltag_msg.tag_id_size();
        // Extract the views one-by-one
        for (int i_tag=0; i_tag < apriltag_msg.tag_id_size(); i_tag++) {
          anantak::AprilTagReadingType *tag_rdng = april_tag_readings_->next_mutable_element();
          tag_rdng->SetFromAprilTagMessage(tag_msgs[i_msg].header().timestamp(),
              options_.camera_num, i_tag, apriltag_msg);
        }
      } else {
        LOG(ERROR) << "Strange: Apriltag sensor messages has no header or apriltag message!";
      }
    }
    
    // Report
    VLOG(1) << "Num messages in initiation queues of cam, imu, tag messages = "
        << init_cam_readings_->n_msgs() << " " << init_imu_readings_->n_msgs() << " "
        << april_tag_readings_->n_msgs();
    
    return true;
  }
  
  /* Initiate
   * Solve a VIO problem on the past data. Camera poses are kept constant. States are created
   * at each camera reading that are connected via imu readings.
   */
  bool Initiate() {
    
    // Traverse forward till first camera state is found that lies between imu states
    int32_t cam_data_idx = 0;
    int32_t imu_data_idx = 0;
    bool found_starting = false;
    while ((cam_data_idx<init_cam_readings_->n_msgs()-1 || imu_data_idx<init_imu_readings_->n_msgs()-2)
           && !found_starting) {
      found_starting = 
        (init_cam_readings_->at(cam_data_idx).imu_interp.timestamp >= init_imu_readings_->at(imu_data_idx).timestamp)
        && (init_cam_readings_->at(cam_data_idx).imu_interp.timestamp <= init_imu_readings_->at(imu_data_idx+1).timestamp);
      if (!found_starting) {
        bool cam_before = (init_cam_readings_->at(cam_data_idx).imu_interp.timestamp <
            init_imu_readings_->at(imu_data_idx).timestamp);
        bool cam_at_end = (cam_data_idx == init_cam_readings_->n_msgs()-1);
        bool imu_at_end = (imu_data_idx == init_imu_readings_->n_msgs()-2);
        if ((cam_before || imu_at_end) && !cam_at_end) cam_data_idx++;
        if ((!cam_before || cam_at_end) && !imu_at_end) imu_data_idx++;
      }
    }
    if (!found_starting) {
      LOG(ERROR) << "Could not find overlapping camera and imu data, not expected. Exit.";
      return false;
    }
    // Report starting of data overlap
    VLOG(1) << "Found overlap between camera and imu data starting at cam, imu idxs = "
        << cam_data_idx << ", " << imu_data_idx;
    init_cam_data_starting_idx_ = cam_data_idx;
    init_imu_data_starting_idx_ = imu_data_idx;
    
    // Increment imu idx to position after first camera timestamp
    imu_data_idx++;
    
    // Make sure that more than two camera readings have been seen
    if (init_cam_readings_->n_msgs()-1-cam_data_idx < 2) {
      LOG(ERROR) << "Seen less than 2 states?! Exit. num cam msgs = "
          << init_cam_readings_->n_msgs() << " starting cam index = " << cam_data_idx;
      return false;
    }
    
    // Find the starting state in the convex optimization data
    bool starting_state_found = false;
    int32_t starting_state_index = 0;
    int32_t starting_cam_data_index = cam_data_idx;
    while (!starting_state_found && starting_state_index<imu_init_tag_camera_->collected_readings_->n_msgs()) {
      starting_state_found = (init_cam_readings_->at(cam_data_idx).imu_interp.timestamp ==
          imu_init_tag_camera_->collected_readings_->at(starting_state_index).imu0.timestamp);
    }
    
    if (starting_state_found) {
      VLOG(1) << "Starting camera pose state was found at idx = " << starting_state_index
          << " for cam data idx = " << starting_cam_data_index;      
    } else {
      LOG(ERROR) << "Starting state in collected camera data was not found in convex optimization "
          << "results. Can not continue. Increase the size of convex optimization queues.";
      return false;
    }
    
    // Starting calculations
    //  W = IMU world frame
    //  W1 = IMU world frame in convex optimization
    //  T0 = Tag map frame. Tag#0 is the origin
    //  Ci = Camera pose at ith instance
    //  Ii = IMU pose at ith instance
    //  Tj = jth Tag pose
    
    // Create first camera state and set it
    Eigen::Quaterniond C0qT0 = init_cam_readings_->at(cam_data_idx).cam_pose.quat.conjugate();
    Eigen::Matrix3d C0rT0(C0qT0);
    Eigen::Vector3d T0pC0 = init_cam_readings_->at(cam_data_idx).cam_pose.posn;
    anantak::Pose3dState *cam_pose0 = cam_poses_->next_mutable_element();
    cam_pose0->SetZero();
    cam_pose0->Create(&C0qT0, &T0pC0, init_cam_readings_->at(cam_data_idx).cam_pose.timestamp);
    
    // Initiate the cam_to_imu_pose_ using data used in convex optimization
    Eigen::Quaterniond CqI = imu_init_tag_camera_->imu_options_.CqI_measured;
    Eigen::Matrix3d CrI(CqI);
    Eigen::Vector3d IpC = -CrI.transpose() * imu_init_tag_camera_->imu_options_.CpI_measured;
    cam_to_imu_pose_.Create(&CqI, &IpC);
    
    // Starting IMU rotation in IMU world frame
    const AprilTagImuInitCamera::ImuCamInitType& ici =
        imu_init_tag_camera_->collected_readings_->at(starting_state_index);
    Eigen::Quaterniond I0qW(ici.imu0.quaternion.conjugate()); 
    Eigen::Matrix3d I0rW(I0qW);
    
    // IMU world frame in Tag map frame
    //  This serves as reference so is kept constant in this optimization
    //  WqT0 = WqI0 * I0qC0 * C0qT0
    Eigen::Quaterniond WqT0 = I0qW.conjugate() * CqI.conjugate() * C0qT0;
    Eigen::Matrix3d WrT0(WqT0);
    Eigen::Vector3d T0pW = Eigen::Vector3d::Zero();
    tag_map_to_imu_world_pose_.SetZero();
    tag_map_to_imu_world_pose_.Create(&WqT0, &T0pW);
    
    // Create first IMU state and set it
    //  WpI0 = WpT0 + W.T0pC0 + W.C0pI0 = -WrT0*T0pW + WrT0*T0pC0 - WrT0*T0rC0*CrI*IpC
    //       = WrT0*( -T0pW + T0pC0 - T0rC0*CrI*IpC)
    Eigen::Vector3d WpI0 = WrT0*(-T0pW + T0pC0 - C0rT0.transpose()*CrI*IpC);
    anantak::ImuState *imu_state0 = imu_states_->next_mutable_element();
    imu_state0->SetZero();
    imu_state0->SetTimestamp(init_cam_readings_->at(cam_data_idx).imu_interp.timestamp);
    imu_state0->IqvG_ = I0qW.coeffs();
    imu_state0->GpI_ = WpI0;
    imu_state0->GvI_ = ici.solved_WvI;
    imu_state0->bg_ = options_.starting_bg;
    imu_state0->ba_ = imu_init_tag_camera_->imu_accel_bias_;
    
    // Set gravity state from convex optimization results
    Eigen::Vector3d Wg = imu_init_tag_camera_->imu_gravity_;
    gravity_state_.SetFromVector3d(&Wg);
    
    // Create next imu state and the first imu constraint, initiate them
    anantak::ImuState *imu_state1 = imu_states_->next_mutable_element();
    imu_state1->SetZero();
    imu_state1->SetTimestamp(init_cam_readings_->at(cam_data_idx+1).imu_interp.timestamp);
    // Create the first imu constraint
    anantak::ImuResidualFunction *imu_residual0 = imu_residuals_->next_mutable_element();
    imu_residual0->Create(imu_state0, imu_state1, &gravity_state_,
        init_cam_readings_->at(cam_data_idx).imu_interp,
        options_.integration_options);
    
    // Create the first imu-cam pose constraint, set it
    anantak::RigidPoseWithImuResidual *rigid_imu_to_cam_pose0 = rigid_imu_to_cam_pose_residuals_->next_mutable_element();
    rigid_imu_to_cam_pose0->Create(imu_state0, cam_pose0, &cam_to_imu_pose_, &tag_map_to_imu_world_pose_,
        &options_.rigid_imu_to_cam_pose_options);
    
    /*// Create first camera state and constraint
    anantak::Pose3dState *cam_to_imu_pose0 = cam_to_imu_poses_->next_mutable_element();
    anantak::NoisyPoseResidual *noisy_i2c_pose_resid0 = noisy_imu_to_cam_pose_residuals_->next_mutable_element();
    
    // Set starting cam to imu pose to value from convex optimization
    cam_to_imu_pose0->LqvG_ = CqI.coeffs();  // CqI
    cam_to_imu_pose0->GpL_ = IpC; // IpC
    cam_to_imu_pose0->timestamp_ = init_cam_readings_->at(cam_data_idx).imu_interp.timestamp;
    
    // Set constraints
    //imu_cam_pose_resid0->Create(imu_state0, cam_pose0, &cam_to_imu_pose_, &imu_cam_residual_options_);
    imu_cam_pose_resid0->Create(imu_state0, cam_pose0, cam_to_imu_pose0, &imu_cam_residual_options_);
    noisy_i2c_pose_resid0->Create(&cam_to_imu_pose_, cam_to_imu_pose0, &options_.imu_to_cam_pose_noise_options);*/
    
    // Increment cam index as starting camera has been 'consumed'
    cam_data_idx++;
    
    // Report at the beginning of states building
    VLOG(1) << "Starting num of Imu states and constraints = " << imu_states_->n_msgs() << ", "
        << imu_residuals_->n_msgs();
    
    // Traverse forward building the problem in the following way:
    //  if this is a camera reading,
    //    close the existing constraint
    //    create a new imu constraint starting with this camera reading
    //    create a new camera constraint
    //  if this is a tag reading,
    //    if there is an existing constraint (should be) add this reading to it
    
    // Statistics
    int32_t num_cam_readings = 1; // counting the first camera reading 
    int32_t num_imu_readings = 0;
    int32_t num_equal_timestamps = 0;
    
    while (cam_data_idx<init_cam_readings_->n_msgs()-1 || imu_data_idx<init_imu_readings_->n_msgs()) {
      // Decide if this is a camera or imu reading
      bool cam_before = (init_cam_readings_->at(cam_data_idx).imu_interp.timestamp <=
          init_imu_readings_->at(imu_data_idx).timestamp);
      bool cam_at_end = (cam_data_idx == init_cam_readings_->n_msgs()-1);
      bool imu_at_end = (imu_data_idx == init_imu_readings_->n_msgs());
      
      // This is an IMU reading
      if ((!cam_before || cam_at_end) && !imu_at_end) {        
        num_imu_readings++;
        
        // Add this reading to the current residual
        anantak::ImuResidualFunction *imu_resid = imu_residuals_->mutable_element();
        if (imu_resid->IsOpen()) {
          if (!imu_resid->AddReading(init_imu_readings_->at(imu_data_idx))) {
            LOG(ERROR) << "Could not add imu reading. Skipping it.";
          }
        } else {
          LOG(ERROR) << "Expected the current residual to be open. Something is wrong. Exit";
          return false;
        }
        
        // Increment IMU index
        imu_data_idx++;      
      }
      
      // This is a camera reading
      if ((cam_before || imu_at_end) && !cam_at_end) {
        num_cam_readings++;
        
        // Create new camera state (Ci) and set it
        Eigen::Quaterniond CiqT0 = init_cam_readings_->at(cam_data_idx).cam_pose.quat.conjugate();
        Eigen::Matrix3d CirT0(CiqT0);
        Eigen::Vector3d T0pCi = init_cam_readings_->at(cam_data_idx).cam_pose.posn;
        anantak::Pose3dState *curr_cam_pose = cam_poses_->next_mutable_element();
        curr_cam_pose->SetZero();
        curr_cam_pose->Create(&CiqT0, &T0pCi, init_cam_readings_->at(cam_data_idx).cam_pose.timestamp);
        
        // Set the ending IMU state (Ii)
        //  IiqW = IiqCi * CiqT0 * T0qW = IqC * CiqT0 * T0qW
        //  WpIi = WpT0 + W.T0pCi + W.CipIi = -WrT0*T0pW + WrT0*T0pCi - WrT0*T0rCi*CrI*IpC
        //       = WrT0*( -T0pW + T0pCi - T0rCi*CrI*IpC)
        Eigen::Quaterniond IiqW = CqI.conjugate() * CiqT0 * WqT0.conjugate();
        Eigen::Vector3d WpIi = WrT0*(-T0pW + T0pCi - CirT0.transpose()*CrI*IpC);
        anantak::ImuState *curr_imu_state = imu_states_->mutable_element();
        if (curr_imu_state->timestamp_ != init_cam_readings_->at(cam_data_idx).imu_interp.timestamp) {
          LOG(ERROR) << "Expected imu_state timestamp to be equal to init_cam_readings_ ts. Exit. i="
              << cam_data_idx;
          return false;
        }
        //curr_imu_state->SetZero();
        //curr_imu_state->SetTimestamp(init_cam_readings_->at(cam_data_idx).imu_interp.timestamp);
        curr_imu_state->IqvG_ = IiqW.coeffs();
        curr_imu_state->GpI_ = WpIi;
        int32_t state_idx = cam_data_idx - init_cam_data_starting_idx_ + starting_state_index;
        curr_imu_state->GvI_ = imu_init_tag_camera_->collected_readings_->at(state_idx).solved_WvI;
        curr_imu_state->bg_ = options_.starting_bg;
        curr_imu_state->ba_ = imu_init_tag_camera_->imu_accel_bias_;
        
        // Imu to cam rigid pose constraint
        anantak::RigidPoseWithImuResidual *rigid_imu_to_cam_pose = rigid_imu_to_cam_pose_residuals_->next_mutable_element();
        rigid_imu_to_cam_pose->Create(curr_imu_state, curr_cam_pose, &cam_to_imu_pose_, &tag_map_to_imu_world_pose_,
            &options_.rigid_imu_to_cam_pose_options);
        
        // Close the last imu constraint (i-1), do not propagate state
        anantak::ImuResidualFunction *last_imu_resid = imu_residuals_->mutable_element();
        if (last_imu_resid->IsOpen()) {
          if (!last_imu_resid->AddEndStateReading(init_cam_readings_->at(cam_data_idx).imu_interp)) {
            LOG(ERROR) << "Could not close state by adding interp cam reading. Exit. i="
                << cam_data_idx;
            return false;
          }
        } else {
          LOG(ERROR) << "Expected the current residual to be open. Could not close it. Exit";
          return false;
        }
        
        // Create next IMU state and initiate it
        anantak::ImuState *next_imu_state = imu_states_->next_mutable_element();
        next_imu_state->SetZero();
        next_imu_state->SetTimestamp(init_cam_readings_->at(cam_data_idx+1).imu_interp.timestamp);
        
        // Add a new imu constraint between last and next states
        anantak::ImuResidualFunction *curr_imu_resid = imu_residuals_->next_mutable_element();
        curr_imu_resid->Create(curr_imu_state, next_imu_state, &gravity_state_,
            init_cam_readings_->at(cam_data_idx).imu_interp,
            options_.integration_options);        
        
        // If timestamps of camera and imu reading are equal, we skip the imu reading
        if (init_cam_readings_->at(cam_data_idx).imu_interp.timestamp
            == init_imu_readings_->at(imu_data_idx).timestamp) {
          num_equal_timestamps++;
          if (!imu_at_end) imu_data_idx++;
        }
        // Increment camera index
        cam_data_idx++;
      }
      
    }
    // Report
    VLOG(1) << "Number of camera and imu readings seen: " << num_cam_readings << ", "
        << num_imu_readings << " Equal timestamps: " << num_equal_timestamps;
    VLOG(1) << "Number of IMU states created = " << imu_states_->n_msgs();
    VLOG(1) << "Number of IMU constraints added = " << imu_residuals_->n_msgs();
    VLOG(1) << "Number of Camera poses added = " << cam_poses_->n_msgs();
    VLOG(1) << "Number of imu-to-cam rigid residuals added = " << rigid_imu_to_cam_pose_residuals_->n_msgs();
    /*VLOG(1) << "Number of Camera constraints added = " << imu_cam_pose_residuals_->n_msgs();
    VLOG(1) << "Number of imu-to-cam poses added = " << cam_to_imu_poses_->n_msgs();
    VLOG(1) << "Number of imu-to-cam residuals added = " << noisy_imu_to_cam_pose_residuals_->n_msgs();
    VLOG(1) << "Number of imu-to-cam change residuals added = " << noisy_imu_to_cam_pose_change_residuals_->n_msgs();*/
    
    // Create a prior for cam-to-imu pose
    NoisyPosePrior cam_to_imu_pose_prior_;
    cam_to_imu_pose_prior_.Create(&CqI, &IpC, &cam_to_imu_pose_, &options_.imu_to_cam_pose_prior_options);
    
    // Create a prior for gravity state
    Vector3dPrior::Options gravity_vec_prior_options;
    gravity_vec_prior_options.sigma_position = std::sqrt(options_.integration_options.qg);
    Vector3dPrior gravity_vec_prior;
    gravity_vec_prior.Create(&Wg, &gravity_state_, &gravity_vec_prior_options);
    
    VLOG(1) << "Preparing constraints and states for optimization.";
    
    // Prepare all imu constraints for optimization
    //  Any un-closed constraints will not be used for optimization
    int32_t num_open_residuals = 0;
    int32_t num_ready_to_optimize = 0;
    int32_t min_integral_history = 1000; // IMU residual should never have more than 1000 readings.
    int32_t max_integral_history = 0;
    int32_t total_integrals_history = 0;
    float avg_integral_history = 0.;
    int32_t min_readings = 1000; // IMU residual should never have more than 1000 readings.
    int32_t max_readings = 0;
    int32_t total_readings = 0;
    float avg_readings = 0.;
    for (int i=0; i<imu_residuals_->n_msgs(); i++) {
      if (!imu_residuals_->at(i).IsOpen()) {
        if (!imu_residuals_->at(i).GetReadyToOptimize()) {
          LOG(ERROR) << "A closed residual is not ready for optimization. Can not continue.";
          return false;
        } else {
          num_ready_to_optimize++;
          min_integral_history = std::min(min_integral_history, imu_residuals_->at(i).num_integrals_stored_);
          max_integral_history = std::max(min_integral_history, imu_residuals_->at(i).num_integrals_stored_);
          total_integrals_history += imu_residuals_->at(i).num_integrals_stored_;
          min_readings = std::min(min_readings, imu_residuals_->at(i).num_readings_);
          max_readings = std::max(max_readings, imu_residuals_->at(i).num_readings_);
          total_readings += imu_residuals_->at(i).num_readings_;
        }
      } else {
        VLOG(1) << "Found an open constraint at idx = " << i;
        num_open_residuals++;
      }
    }
    avg_integral_history = float(total_integrals_history) / float(num_ready_to_optimize);
    avg_readings = float(total_readings) / float(num_ready_to_optimize);
    VLOG(1) << "Number of imu residuals ready to optimize = " << num_ready_to_optimize
        << ", num open = " << num_open_residuals;
    VLOG(1) << "Residuals can store max history length = " << imu_residuals_->front().max_integrals_history_;
    VLOG(1) << "Integrals history stored min, max, avg per residual= " << min_integral_history << " "
        << max_integral_history << " " << avg_integral_history;
    VLOG(1) << "Number of min, max, avg readings per residual = " << min_readings << " "
        << max_readings << " " << avg_readings;
        
    // Prepare the imu-to-cam pose prior for optimization
    if (!cam_to_imu_pose_prior_.GetReadyToOptimize()) {
      LOG(ERROR) << "Could not get cam_to_imu_pose_prior_ ready for optimization";
      return false;
    }
    
    // Prepare imu-cam-poses for optimization
    for (int i=0; i<rigid_imu_to_cam_pose_residuals_->n_msgs(); i++) {
      if (!rigid_imu_to_cam_pose_residuals_->at(i).GetReadyToOptimize()) {
        LOG(ERROR) << "Rigid pose residual at i = " << i << " could not be readied for optimization";
      }
    }    
    
    // Prepare gravity prior
    if (!gravity_vec_prior.GetReadyToOptimize()) {
      LOG(ERROR) << "Could not get gravity_vec_prior ready for optimization";
      return false;      
    }
    
    VLOG(1) << "Done preparing states and residuals for optimization";
    
    // Save the starting states to a file for plotting and check integrals (for testing)
    SaveStatesToFile(options_.save_filename, "init1");
    SaveResidualsToFile(options_.save_filename, "init1");
    
    // Setup the problem
    //  All camera poses are constant
    //  WqT0, T0pW are constant
    // Priors for accel biases are created
    // Priors for gravity are created
    
    // Build a problem by adding all constraints to it
    ceres::Problem::Options problem_options;
    problem_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    ceres::Problem problem(problem_options);
    
    // Use imu residuals we created before. Do not transfer ownership
    for (int i=0; i<imu_residuals_->n_msgs(); i++) {
      if (!imu_residuals_->at(i).IsOpen()) {
        ceres::CostFunction* i_residual = &imu_residuals_->at(i);
        ceres::LossFunction* quad_loss = NULL;
        problem.AddResidualBlock(
          i_residual,
          quad_loss,
          imu_residuals_->at(i).state0_->error_,
          imu_residuals_->at(i).state1_->error_,
          imu_residuals_->at(i).gravity_->error_
        );
      }
    }
    
    // Add imu-to-cam noisy pose residuals
    for (int i=0; i<rigid_imu_to_cam_pose_residuals_->n_msgs(); i++) {
      ceres::CostFunction* i_residual = &rigid_imu_to_cam_pose_residuals_->at(i);
      ceres::LossFunction* quad_loss = NULL;
      problem.AddResidualBlock(
        i_residual,
        quad_loss,
        rigid_imu_to_cam_pose_residuals_->at(i).poseI_->error_,
        rigid_imu_to_cam_pose_residuals_->at(i).poseC_->error_,
        rigid_imu_to_cam_pose_residuals_->at(i).poseItoC_->error_,
        rigid_imu_to_cam_pose_residuals_->at(i).poseT0toW_->error_
      );
    }
    
    /*// Add imu-to-cam noisy pose change residuals
    for (int i=0; i<noisy_imu_to_cam_pose_change_residuals_->n_msgs(); i++) {
      ceres::CostFunction* i_residual = &noisy_imu_to_cam_pose_change_residuals_->at(i);
      ceres::LossFunction* quad_loss = NULL;
      problem.AddResidualBlock(
        i_residual,
        quad_loss,
        noisy_imu_to_cam_pose_change_residuals_->at(i).expectation_->error_,
        noisy_imu_to_cam_pose_change_residuals_->at(i).measurement_->error_
      );
    }*/
    
    // Add a prior for imu-to-cam pose
    {
      ceres::CostFunction* i_residual = &cam_to_imu_pose_prior_;
      ceres::LossFunction* quad_loss = NULL;
      problem.AddResidualBlock(
        i_residual,
        quad_loss,
        cam_to_imu_pose_prior_.measurement_->error_
      );
    }
    
    // Add a prior for gravity pose
    {
      ceres::CostFunction* i_residual = &gravity_vec_prior;
      ceres::LossFunction* quad_loss = NULL;
      problem.AddResidualBlock(
        i_residual,
        quad_loss,
        gravity_vec_prior.measurement_->error_
      );
    }
    
    // Set camera poses as constants
    for (int i=0; i<cam_poses_->n_msgs(); i++) {
      problem.SetParameterBlockConstant(cam_poses_->at(i).error_);      
    }
    
    // Set IMU World in Tag map pose as constant
    problem.SetParameterBlockConstant(tag_map_to_imu_world_pose_.error_);
    
    // Solve the problem
    ceres::Solver::Options solver_options;
    solver_options.max_num_iterations = 300;
    solver_options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary solver_summary;
    ceres::Solve(solver_options, &problem, &solver_summary);
    if (true) std::cout << solver_summary.FullReport() << std::endl;    
    
    // Calculate covariances
    VLOG(1) << "Calculating covariances";
    Eigen::Matrix3d imu_gravity_cov_; Eigen::Vector3d imu_gravity_stdev_;
    Eigen::Matrix<double,6,6> cam_imu_pose_cov_; Eigen::Matrix<double,6,1> cam_imu_pose_stdev_;
    ceres::Covariance::Options covariance_options;
    ceres::Covariance covariance(covariance_options);
    std::vector<std::pair<const double*, const double*>> covariance_blocks;
    covariance_blocks.push_back(std::make_pair(gravity_state_.error_, gravity_state_.error_));
    covariance_blocks.push_back(std::make_pair(cam_to_imu_pose_.error_, cam_to_imu_pose_.error_));
    CHECK(covariance.Compute(covariance_blocks, &problem));
    covariance.GetCovarianceBlock(gravity_state_.error_, gravity_state_.error_, imu_gravity_cov_.data());
    covariance.GetCovarianceBlock(cam_to_imu_pose_.error_, cam_to_imu_pose_.error_, cam_imu_pose_cov_.data());
    
    imu_gravity_stdev_ = imu_gravity_cov_.diagonal().cwiseSqrt();
    cam_imu_pose_stdev_ = cam_imu_pose_cov_.diagonal().cwiseSqrt();
    
    // Recalculate states, this also sets all error states to zero
    VLOG(1) << "Recalculating states";
    gravity_state_.Recalculate();
    cam_to_imu_pose_.Recalculate();
    tag_map_to_imu_world_pose_.Recalculate();
    for (int i=0; i<imu_states_->n_msgs(); i++) {imu_states_->at(i).Recalculate();}
    for (int i=0; i<cam_poses_->n_msgs(); i++) {cam_poses_->at(i).Recalculate();}
    
    // Report results
    VLOG(1) << "IMU estimates:";
    VLOG(1) << "Gravity = " << gravity_state_.Gp_.transpose() << ", " << gravity_state_.Gp_.norm() << " (m/s^2)";
    VLOG(1) << "Gravity stdev = " << imu_gravity_stdev_.transpose() << " (m/s^2)";
    VLOG(1) << "Gravity = " << gravity_state_.Gp_.transpose()/options_.integration_options.accel_factor << ", "
        << gravity_state_.Gp_.norm()/options_.integration_options.accel_factor << " (LSB)";
    VLOG(1) << "Gravity stdev = " << imu_gravity_stdev_.transpose()/options_.integration_options.accel_factor << " (LSB)";
    
    VLOG(1) << "Cam-Imu pose = " << cam_to_imu_pose_.GpL_.transpose() << " (m)";
    VLOG(1) << "Cam-Imu pose stdev = " << cam_imu_pose_stdev_.block<3,1>(0,0).transpose() << " (m)";
    Eigen::AngleAxisd c2iaa(cam_to_imu_pose_.Quaternion());
    VLOG(1) << "Cam-Imu pose aa = " << c2iaa.axis().transpose() << ", " << c2iaa.angle()*DegreesPerRadian << " (deg)";
    VLOG(1) << "Cam-Imu pose aa stdev = " << cam_imu_pose_stdev_.block<3,1>(3,0).transpose()*DegreesPerRadian << " (deg)";
    
    SaveStatesToFile(options_.save_filename, "init2");
    SaveResidualsToFile(options_.save_filename, "init2");
    
    
    ///////  Now we redo the same problem with Tag Views in place of fixed cam poses  ////////
    
    // Initiate the tag poses in tag map
    int32_t num_tags = imu_init_tag_camera_->tag_camera_.connected_tags_sizes_[options_.tags_set_num];
    for (int i_tag=0; i_tag<num_tags; i_tag++) {
      const AprilTagCamera::TagCompositePose& tag_comp_pose =
          imu_init_tag_camera_->tag_camera_.tag_poses_[options_.tags_set_num]->at(i_tag);
      double size = LookupTagSize(tag_comp_pose.tag_id);
      Eigen::Quaterniond TjqT0 = tag_comp_pose.pose.rotn_q.conjugate();
      Eigen::Vector3d T0pTj = tag_comp_pose.pose.posn;
      // Create a new tag_pose state
      anantak::StaticAprilTagState *tag_pose = tag_poses_->next_mutable_element();
      tag_pose->Create(&tag_comp_pose.tag_id, &TjqT0, &T0pTj, &size);
      // Set the covariances
      Eigen::Matrix<double,6,1> pose_stdev;
      pose_stdev <<  options_.starting_tag_angle_sigma, options_.starting_tag_angle_sigma, options_.starting_tag_angle_sigma,
          options_.starting_tag_position_sigma, options_.starting_tag_position_sigma, options_.starting_tag_position_sigma;
      tag_pose->pose_.covariance_ = pose_stdev.asDiagonal();
      tag_pose->size_.covariance_ = options_.default_april_tag_size_sigma;
    }
    // Report
    VLOG(1) << "Number of tags initiated in map = " << num_tags;
    for (int i=0; i<num_tags; i++) {
      VLOG(2) << "  " << tag_poses_->at(i).tag_id_ << " "
          << tag_poses_->at(i).pose_.Quaternion().coeffs().transpose() << " "
          << tag_poses_->at(i).pose_.Position().transpose() << " "
          << tag_poses_->at(i).size_.Value();
    }
    SaveTagMapToFile(options_.save_filename, "init2");
    
    // Create camera intrinsics state
    Eigen::Matrix3d K_mat = imu_init_tag_camera_->tag_camera_.camera_K_;
    camera_intrinsics_.Create(&K_mat);
    VLOG(1) << "Starting camera matrix = \n" << camera_intrinsics_.CameraMatrix();
    
    int32_t found_cam_idx = 0;
    VLOG(1) << "Num of April tags seen = " << april_tag_readings_->n_msgs();
    
    // Create Tag view states for each camera pose
    for (int i_tag_rdng=0; i_tag_rdng<april_tag_readings_->n_msgs(); i_tag_rdng++) {
    //for (int i_tag_rdng=0; i_tag_rdng<100; i_tag_rdng++) {
      
      anantak::AprilTagReadingType *tag_rdng = april_tag_readings_->at_ptr(i_tag_rdng);
      anantak::Pose3dState *poseC = NULL;
      anantak::StaticAprilTagState *tagTj = NULL;
      
      // Find the corresponding cam state
      bool found_cam = FindTimestampInCamPoses(tag_rdng->timestamp, &poseC, found_cam_idx);
      if (poseC) {
        //
      } else {
        LOG(WARNING) << "Did not find cam pose. Not expected.";
        continue;
      }
      
      // Find this tag in list of tags
      bool found_tag = FindTagInTagMap(tag_rdng->tag_id, &tagTj);
      if (tagTj) {
        //VLOG(3) << "Found tag in map. tag_id = " << tag_rdng->tag_id << " " << tagTj->tag_id_;
      } else {
        LOG(WARNING) << "Did not find tag in tag map. Not expected. tag_id = " << tag_rdng->tag_id;
        continue;
      }
      
      //bool Create(const anantak::AprilTagReadingType *tag_view, anantak::Pose3dState *poseC,
      //    anantak::StaticAprilTagState *tagTj, anantak::CameraIntrinsicsState *camera,
      //    AprilTagVioResidual::Options *options, bool is_zero_tag = false)
      anantak::AprilTagViewResidual *tag_view_resid = tag_view_residuals_->next_mutable_element();
      bool created = tag_view_resid->Create(tag_rdng, poseC, tagTj, &camera_intrinsics_,
          &options_.apriltag_view_residual_options);
      if (!created) {
        LOG(ERROR) << "Could not create tag view residual. Skipping.";
        continue;
      }
      
      // Check the residual after creation if needed
      
    }
    // Report
    VLOG(1) << "Created tag view residuals. num = " << tag_view_residuals_->n_msgs()
        << " of total views = " << april_tag_readings_->n_msgs();
    
    // Find origin tag in tag map
    anantak::StaticAprilTagState *origin_tag = NULL;
    bool found_origin = FindOriginTagInTagMap(&origin_tag);
    if (!origin_tag) {
      LOG(ERROR) << "Could not find the origin tag in tagmap. Quitting.";
      return false;
    }
    
    
    
    return true;
  }
  
  /* Initiate2
   * Solve a VIO problem on the past data. Camera poses are kept constant. States are created
   * at each camera reading that are connected via imu readings.
   */
  bool Initiate2() {
    
    // Traverse forward till first camera state is found that lies between imu states
    int32_t cam_data_idx = 0;
    int32_t imu_data_idx = 0;
    bool found_starting = false;
    while ((cam_data_idx<init_cam_readings_->n_msgs()-1 || imu_data_idx<init_imu_readings_->n_msgs()-2)
           && !found_starting) {
      found_starting = 
        (init_cam_readings_->at(cam_data_idx).imu_interp.timestamp >= init_imu_readings_->at(imu_data_idx).timestamp)
        && (init_cam_readings_->at(cam_data_idx).imu_interp.timestamp <= init_imu_readings_->at(imu_data_idx+1).timestamp);
      if (!found_starting) {
        bool cam_before = (init_cam_readings_->at(cam_data_idx).imu_interp.timestamp <
            init_imu_readings_->at(imu_data_idx).timestamp);
        bool cam_at_end = (cam_data_idx == init_cam_readings_->n_msgs()-1);
        bool imu_at_end = (imu_data_idx == init_imu_readings_->n_msgs()-2);
        if ((cam_before || imu_at_end) && !cam_at_end) cam_data_idx++;
        if ((!cam_before || cam_at_end) && !imu_at_end) imu_data_idx++;
      }
    }
    if (!found_starting) {
      LOG(ERROR) << "Could not find overlapping camera and imu data, not expected. Exit.";
      return false;
    }
    // Report starting of data overlap
    VLOG(1) << "Found overlap between camera and imu data starting at cam, imu idxs = "
        << cam_data_idx << ", " << imu_data_idx;
    init_cam_data_starting_idx_ = cam_data_idx;
    init_imu_data_starting_idx_ = imu_data_idx;
    
    // Increment imu idx to position after first camera timestamp
    imu_data_idx++;
    
    // Make sure that more than two camera readings have been seen
    if (init_cam_readings_->n_msgs()-1-cam_data_idx < 2) {
      LOG(ERROR) << "Seen less than 2 states?! Exit. num cam msgs = "
          << init_cam_readings_->n_msgs() << " starting cam index = " << cam_data_idx;
      return false;
    }
    
    // Find the starting state in the convex optimization data
    bool starting_state_found = false;
    int32_t starting_state_index = 0;
    int32_t starting_cam_data_index = cam_data_idx;
    while (!starting_state_found && starting_state_index<imu_init_tag_camera_->collected_readings_->n_msgs()) {
      starting_state_found = (init_cam_readings_->at(cam_data_idx).imu_interp.timestamp ==
          imu_init_tag_camera_->collected_readings_->at(starting_state_index).imu0.timestamp);
    }
    
    if (starting_state_found) {
      VLOG(1) << "Starting camera pose state was found at idx = " << starting_state_index
          << " for cam data idx = " << starting_cam_data_index;      
    } else {
      LOG(ERROR) << "Starting state in collected camera data was not found in convex optimization "
          << "results. Can not continue. Increase the size of convex optimization queues.";
      return false;
    }
    
    // We will run the calibration by expressing all readings in starting reading frame
    const AprilTagImuInitCamera::ImuCamInitType& ici =
        imu_init_tag_camera_->collected_readings_->at(starting_state_index);
    Eigen::Quaterniond I0qW(ici.imu0.quaternion.conjugate()); // Starting rotation in IMU world frame
    Eigen::Vector3d WpI0(ici.solved_WpI); // Starting position of IMU solved in convex optimization
    Eigen::Matrix3d I0rW(I0qW);
    Eigen::Quaterniond T0qC0 = init_cam_readings_->at(cam_data_idx).cam_pose.quat;
    Eigen::Matrix3d C0rT0(T0qC0.conjugate());
    Eigen::Vector3d T0pC0 = init_cam_readings_->at(cam_data_idx).cam_pose.posn;
    
    // Create the first two imu states and the first imu constraint
    anantak::ImuState *imu_state0 = imu_states_->next_mutable_element();
    anantak::ImuState *imu_state1 = imu_states_->next_mutable_element();
    anantak::ImuResidualFunction *imu_residual0 = imu_residuals_->next_mutable_element();

    // Set gravity state from convex optimization results
    Eigen::Vector3d I0g = I0rW * imu_init_tag_camera_->imu_gravity_;
    gravity_state_.SetFromVector3d(&I0g);

    // Set the starting state using convex optimization results and options
    imu_state0->SetZero();
    imu_state0->SetTimestamp(init_cam_readings_->at(cam_data_idx).imu_interp.timestamp);
    imu_state0->IqvG_ = Eigen::Quaterniond::Identity().coeffs();
    imu_state0->GpI_ = Eigen::Vector3d::Zero();
    imu_state0->GvI_ = I0rW * ici.solved_WvI;
    imu_state0->bg_ = options_.starting_bg;
    imu_state0->ba_ = imu_init_tag_camera_->imu_accel_bias_;
    // Reset the next state, this will be populated when the residual 'closes'
    imu_state1->SetZero();
    imu_state1->SetTimestamp(init_cam_readings_->at(cam_data_idx+1).imu_interp.timestamp);
    // Create the first residual
    imu_residual0->Create(imu_state0, imu_state1, &gravity_state_,
        ImuReadingType(init_cam_readings_->at(cam_data_idx).imu_interp, I0qW),
        options_.integration_options);
    
    // Initiate the cam_to_imu_pose_ using data used in convex optimization
    Eigen::Quaterniond CqI = imu_init_tag_camera_->imu_options_.CqI_measured;
    Eigen::Matrix3d CrI(CqI);
    Eigen::Vector3d IpC = -CrI.transpose() * imu_init_tag_camera_->imu_options_.CpI_measured;
    cam_to_imu_pose_.LqvG_ = CqI.coeffs();  // CqI
    cam_to_imu_pose_.GpL_ = IpC; // IpC
    
    // Create first camera state and constraint
    anantak::Pose3dState *cam_pose0 = cam_poses_->next_mutable_element();
    anantak::ImuToCameraPoseResidual *imu_cam_pose_resid0 = imu_cam_pose_residuals_->next_mutable_element();
    anantak::Pose3dState *cam_to_imu_pose0 = cam_to_imu_poses_->next_mutable_element();
    anantak::NoisyPoseResidual *noisy_i2c_pose_resid0 = noisy_imu_to_cam_pose_residuals_->next_mutable_element();
    
    // Set the first camera state and constraint
    cam_pose0->SetZero();
    cam_pose0->LqvG_ = Eigen::Quaterniond::Identity().coeffs();
    cam_pose0->GpL_ = Eigen::Vector3d::Zero();
    
    // Set starting cam to imu pose to value from convex optimization
    cam_to_imu_pose0->LqvG_ = CqI.coeffs();  // CqI
    cam_to_imu_pose0->GpL_ = IpC; // IpC
    cam_to_imu_pose0->timestamp_ = init_cam_readings_->at(cam_data_idx).imu_interp.timestamp;
    
    // Set constraints
    //imu_cam_pose_resid0->Create(imu_state0, cam_pose0, &cam_to_imu_pose_, &imu_cam_residual_options_);
    imu_cam_pose_resid0->Create(imu_state0, cam_pose0, cam_to_imu_pose0, &imu_cam_residual_options_);
    noisy_i2c_pose_resid0->Create(&cam_to_imu_pose_, cam_to_imu_pose0, &options_.imu_to_cam_pose_noise_options);
    
    // Increment cam index as starting camera has been 'consumed'
    cam_data_idx++;
    
    // Report beginning
    VLOG(1) << "Starting num of Imu states and constraints = " << imu_states_->n_msgs() << ", "
        << imu_residuals_->n_msgs();
    
    // Traverse forward building the problem in the following way:
    //  if this is a camera reading,
    //    close the existing constraint
    //    create a new imu constraint starting with this camera reading
    //    create a new camera constraint
    //  if this is a tag reading,
    //    if there is an existing constraint (should be) add this reading to it
    
    // Statistics
    int32_t num_cam_readings = 1; // counting the first camera reading 
    int32_t num_imu_readings = 0;
    int32_t num_equal_timestamps = 0;
    
    while (cam_data_idx<init_cam_readings_->n_msgs()-1 || imu_data_idx<init_imu_readings_->n_msgs()) {
      // Decide if this is a camera or imu reading
      bool cam_before = (init_cam_readings_->at(cam_data_idx).imu_interp.timestamp <=
          init_imu_readings_->at(imu_data_idx).timestamp);
      bool cam_at_end = (cam_data_idx == init_cam_readings_->n_msgs()-1);
      bool imu_at_end = (imu_data_idx == init_imu_readings_->n_msgs());
      
      // Camera reading
      if ((cam_before || imu_at_end) && !cam_at_end) {
        // This is a camera reading
        num_cam_readings++;
        
        // Close the last imu constraint
        anantak::ImuResidualFunction *imu_resid = imu_residuals_->mutable_element();
        if (imu_resid->IsOpen()) {
          if (!imu_resid->AddEndStateReading(
              ImuReadingType(init_cam_readings_->at(cam_data_idx).imu_interp, I0qW),
              true)) {
            LOG(ERROR) << "Could not close state by adding interp cam reading. Exit.";
            return false;
          }
        } else {
          LOG(ERROR) << "Expected the current residual to be open. Could not close it. Exit";
          return false;
        }
        
        // Add a new imu state at the next camera timestamp
        anantak::ImuState *curr_state = imu_states_->mutable_element();
        anantak::ImuState *next_state = imu_states_->next_mutable_element();
        next_state->SetZero();
        next_state->SetTimestamp(init_cam_readings_->at(cam_data_idx+1).imu_interp.timestamp);
        
        // Add a new imu constraint between last and next states
        anantak::ImuResidualFunction *new_imu_resid = imu_residuals_->next_mutable_element();
        new_imu_resid->Create(curr_state, next_state, &gravity_state_,
            ImuReadingType(init_cam_readings_->at(cam_data_idx).imu_interp, I0qW),
            options_.integration_options);
        
        // Add a camera state for the closed imu state, set it
        anantak::Pose3dState *curr_cam_pose = cam_poses_->next_mutable_element();
        curr_cam_pose->SetZero();
        // Need CiqC0 = CiqT0 * T0qC0
        Eigen::Quaterniond CiqC0 = init_cam_readings_->at(cam_data_idx).cam_pose.quat.conjugate() * T0qC0;
        curr_cam_pose->LqvG_ = CiqC0.coeffs();
        // Need C0pCi = C0rT0 * (T0pCi - T0pC0)
        curr_cam_pose->GpL_ = C0rT0 * (init_cam_readings_->at(cam_data_idx).cam_pose.posn - T0pC0);
        
        // Add a cam-to-imu pose
        anantak::Pose3dState *last_cam_to_imu_pose = cam_to_imu_poses_->mutable_element();
        anantak::Pose3dState *curr_cam_to_imu_pose = cam_to_imu_poses_->next_mutable_element();        
        // Set starting cam to imu pose to value from convex optimization
        curr_cam_to_imu_pose->LqvG_ = CqI.coeffs();  // CqI
        curr_cam_to_imu_pose->GpL_ = IpC; // IpC
        curr_cam_to_imu_pose->timestamp_ = init_cam_readings_->at(cam_data_idx).imu_interp.timestamp;
        
        // Add a camera constraint for the closed imu state and the new camera state
        anantak::ImuToCameraPoseResidual *curr_imu_cam_pose_resid = imu_cam_pose_residuals_->next_mutable_element();
        //curr_imu_cam_pose_resid->Create(curr_state, curr_cam_pose, &cam_to_imu_pose_, &imu_cam_residual_options_);
        curr_imu_cam_pose_resid->Create(curr_state, curr_cam_pose, curr_cam_to_imu_pose, &imu_cam_residual_options_);
        
        // Add a cam-to-imu pose residual
        anantak::NoisyPoseResidual *curr_noisy_i2c_pose_resid = noisy_imu_to_cam_pose_residuals_->next_mutable_element();
        curr_noisy_i2c_pose_resid->Create(&cam_to_imu_pose_, curr_cam_to_imu_pose, &options_.imu_to_cam_pose_noise_options);
        
        // Add a cam-to-imu pose change residual
        anantak::NoisyPoseResidual *curr_noisy_i2c_pose_change_resid = noisy_imu_to_cam_pose_change_residuals_->next_mutable_element();
        curr_noisy_i2c_pose_change_resid->Create(last_cam_to_imu_pose, curr_cam_to_imu_pose, &options_.imu_to_cam_pose_change_options);
        
        // If timestamps of camera and imu reading were equal, we skip the imu reading
        if (init_cam_readings_->at(cam_data_idx).imu_interp.timestamp
            == init_imu_readings_->at(imu_data_idx).timestamp) {
          num_equal_timestamps++;
          if (!imu_at_end) imu_data_idx++;
        }
        // Increment camera index
        cam_data_idx++;
      }
      
      // IMU reading
      if ((!cam_before || cam_at_end) && !imu_at_end) {
        // This is an IMU reading
        num_imu_readings++;
        
        // Add this reading to the current residual
        anantak::ImuResidualFunction *imu_resid = imu_residuals_->mutable_element();
        if (imu_resid->IsOpen()) {
          if (!imu_resid->AddReading(
              ImuReadingType(init_imu_readings_->at(imu_data_idx), I0qW))) {
            LOG(ERROR) << "Could not add imu reading. Skipping it.";
          }
        } else {
          LOG(ERROR) << "Expected the current residual to be open. Something is wrong. Exit";
          return false;
        }
        
        // Increment IMU index
        imu_data_idx++;      
      }
    }
    // Report
    VLOG(1) << "Number of camera and imu readings seen: " << num_cam_readings << ", "
        << num_imu_readings << " Equal timestamps: " << num_equal_timestamps;
    VLOG(1) << "Number of IMU states created = " << imu_states_->n_msgs();
    VLOG(1) << "Number of IMU constraints added = " << imu_residuals_->n_msgs();
    VLOG(1) << "Number of Camera poses added = " << cam_poses_->n_msgs();
    VLOG(1) << "Number of Camera constraints added = " << imu_cam_pose_residuals_->n_msgs();
    VLOG(1) << "Number of imu-to-cam poses added = " << cam_to_imu_poses_->n_msgs();
    VLOG(1) << "Number of imu-to-cam residuals added = " << noisy_imu_to_cam_pose_residuals_->n_msgs();
    VLOG(1) << "Number of imu-to-cam change residuals added = " << noisy_imu_to_cam_pose_change_residuals_->n_msgs();
    
    // Create a prior for cam-to-imu pose
    NoisyPosePrior cam_to_imu_pose_prior_;
    cam_to_imu_pose_prior_.Create(&CqI, &IpC, &cam_to_imu_pose_, &options_.imu_to_cam_pose_prior_options);
    
    // Prepare all imu constraints for optimization
    //  Any un-closed constraints will not be used for optimization
    int32_t num_open_residuals = 0;
    int32_t num_ready_to_optimize = 0;
    int32_t min_integral_history = 1000; // IMU residual should never have more than 1000 readings.
    int32_t max_integral_history = 0;
    int32_t total_integrals_history = 0;
    float avg_integral_history = 0.;
    int32_t min_readings = 1000; // IMU residual should never have more than 1000 readings.
    int32_t max_readings = 0;
    int32_t total_readings = 0;
    float avg_readings = 0.;
    for (int i=0; i<imu_residuals_->n_msgs(); i++) {
      if (!imu_residuals_->at(i).IsOpen()) {
        if (!imu_residuals_->at(i).GetReadyToOptimize()) {
          LOG(ERROR) << "A closed residual is not ready for optimization. Can not continue.";
          return false;
        } else {
          num_ready_to_optimize++;
          min_integral_history = std::min(min_integral_history, imu_residuals_->at(i).num_integrals_stored_);
          max_integral_history = std::max(min_integral_history, imu_residuals_->at(i).num_integrals_stored_);
          total_integrals_history += imu_residuals_->at(i).num_integrals_stored_;
          min_readings = std::min(min_readings, imu_residuals_->at(i).num_readings_);
          max_readings = std::max(max_readings, imu_residuals_->at(i).num_readings_);
          total_readings += imu_residuals_->at(i).num_readings_;
        }
      } else {
        VLOG(1) << "Found an open constraint at idx = " << i;
        num_open_residuals++;
      }
    }
    avg_integral_history = float(total_integrals_history) / float(num_ready_to_optimize);
    avg_readings = float(total_readings) / float(num_ready_to_optimize);
    VLOG(1) << "Number of imu residuals ready to optimize = " << num_ready_to_optimize
        << ", num open = " << num_open_residuals;
    VLOG(1) << "Residuals can store max history length = " << imu_residuals_->front().max_integrals_history_;
    VLOG(1) << "Integrals history stored min, max, avg per residual= " << min_integral_history << " "
        << max_integral_history << " " << avg_integral_history;
    VLOG(1) << "Number of min, max, avg readings per residual = " << min_readings << " "
        << max_readings << " " << avg_readings;
    
    // Set all camera pose constraints to ready for optimization
    for (int i=0; i<imu_cam_pose_residuals_->n_msgs(); i++) {
      if (!imu_cam_pose_residuals_->at(i).GetReadyToOptimize()) {
        LOG(ERROR) << "A created imu-cam-pose residual is not ready for optimization. Not expected.";
        return false;
      }
    }
    
    // Prepare the imu-to-cam pose prior for optimization
    if (!cam_to_imu_pose_prior_.GetReadyToOptimize()) {
      LOG(ERROR) << "Could not get cam_to_imu_pose_prior_ ready for optimization";
      return false;
    }
    
    // Prepare imu-cam-poses for optimization
    for (int i=0; i<noisy_imu_to_cam_pose_residuals_->n_msgs(); i++) {
      if (!noisy_imu_to_cam_pose_residuals_->at(i).GetReadyToOptimize()) {
        LOG(ERROR) << "Noisy pose residual at i = " << i << " could not be readied for optimization";
      }
    }
    
    // Prepare imu-cam-pose changes for optimization
    for (int i=0; i<noisy_imu_to_cam_pose_change_residuals_->n_msgs(); i++) {
      if (!noisy_imu_to_cam_pose_change_residuals_->at(i).GetReadyToOptimize()) {
        LOG(ERROR) << "Noisy pose residual at i = " << i << " could not be readied for optimization";
      }
    }
    
    VLOG(1) << "Done preparing states and residuals for optimization";
    
    // Save the starting states to a file for plotting and check integrals (for testing)
    SaveStatesToFile(options_.save_filename, "init1");
    SaveResidualsToFile(options_.save_filename, "init1");
    //CompareIntegrations();
    //if (starting_state_found) CheckVsConvexIntegrals(starting_state_index);
    
    // Build a problem by adding all constraints to it
    ceres::Problem::Options problem_options;
    problem_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    ceres::Problem problem(problem_options);
    
    // Use imu-cam-pose residuals we created. Do not transfer ownership.
    for (int i=0; i<imu_cam_pose_residuals_->n_msgs(); i++) {
      ceres::CostFunction* i_c_residual = &imu_cam_pose_residuals_->at(i);
      ceres::LossFunction* quad_loss = NULL;
      problem.AddResidualBlock(
        i_c_residual,
        quad_loss,
        imu_cam_pose_residuals_->at(i).poseI_->error_,
        imu_cam_pose_residuals_->at(i).poseItoC_->error_
      );
    }
    
    // Use imu residuals we created before. Do not transfer ownership
    for (int i=0; i<imu_residuals_->n_msgs(); i++) {
      if (!imu_residuals_->at(i).IsOpen()) {
        ceres::CostFunction* i_residual = &imu_residuals_->at(i);
        ceres::LossFunction* quad_loss = NULL;
        problem.AddResidualBlock(
          i_residual,
          quad_loss,
          imu_residuals_->at(i).state0_->error_,
          imu_residuals_->at(i).state1_->error_,
          imu_residuals_->at(i).gravity_->error_
        );
      }
    }
    
    // Add imu-to-cam noisy pose residuals
    for (int i=0; i<noisy_imu_to_cam_pose_residuals_->n_msgs(); i++) {
      ceres::CostFunction* i_residual = &noisy_imu_to_cam_pose_residuals_->at(i);
      ceres::LossFunction* quad_loss = NULL;
      problem.AddResidualBlock(
        i_residual,
        quad_loss,
        noisy_imu_to_cam_pose_residuals_->at(i).expectation_->error_,
        noisy_imu_to_cam_pose_residuals_->at(i).measurement_->error_
      );
    }
    
    // Add imu-to-cam noisy pose change residuals
    for (int i=0; i<noisy_imu_to_cam_pose_change_residuals_->n_msgs(); i++) {
      ceres::CostFunction* i_residual = &noisy_imu_to_cam_pose_change_residuals_->at(i);
      ceres::LossFunction* quad_loss = NULL;
      problem.AddResidualBlock(
        i_residual,
        quad_loss,
        noisy_imu_to_cam_pose_change_residuals_->at(i).expectation_->error_,
        noisy_imu_to_cam_pose_change_residuals_->at(i).measurement_->error_
      );
    }
    
    // Add a prior for imu-to-cam pose
    {
      ceres::CostFunction* i_residual = &cam_to_imu_pose_prior_;
      ceres::LossFunction* quad_loss = NULL;
      problem.AddResidualBlock(
        i_residual,
        quad_loss,
        cam_to_imu_pose_prior_.measurement_->error_
      );
    }
    
    // Should we set the starting imu pose as constant? No.
    // Because Camera starting pose is set to Zero pose. Camera poses are fixed in this problem.
    // When Imu-to-camera pose is modified, starting IMU pose will get modified with it.
    
    // Solve the problem
    ceres::Solver::Options solver_options;
    solver_options.max_num_iterations = 300;
    solver_options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary solver_summary;
    ceres::Solve(solver_options, &problem, &solver_summary);
    if (true) std::cout << solver_summary.FullReport() << std::endl;    
    
    // Calculate covariances
    VLOG(1) << "Calculating covariances";
    Eigen::Matrix3d imu_gravity_cov_; Eigen::Vector3d imu_gravity_stdev_;
    Eigen::Matrix<double,6,6> cam_imu_pose_cov_; Eigen::Matrix<double,6,1> cam_imu_pose_stdev_;
    ceres::Covariance::Options covariance_options;
    ceres::Covariance covariance(covariance_options);
    std::vector<std::pair<const double*, const double*>> covariance_blocks;
    covariance_blocks.push_back(std::make_pair(gravity_state_.error_, gravity_state_.error_));
    covariance_blocks.push_back(std::make_pair(cam_to_imu_pose_.error_, cam_to_imu_pose_.error_));
    CHECK(covariance.Compute(covariance_blocks, &problem));
    covariance.GetCovarianceBlock(gravity_state_.error_, gravity_state_.error_, imu_gravity_cov_.data());
    covariance.GetCovarianceBlock(cam_to_imu_pose_.error_, cam_to_imu_pose_.error_, cam_imu_pose_cov_.data());
    
    imu_gravity_stdev_ = imu_gravity_cov_.diagonal().cwiseSqrt();
    cam_imu_pose_stdev_ = cam_imu_pose_cov_.diagonal().cwiseSqrt();
    
    // Error states
    VLOG(1) << "Cam-Imu pose error = " << cam_to_imu_pose_.dGpL_.transpose()
        << ", " << cam_to_imu_pose_.dGaL_.transpose();
    VLOG(1) << "Gravity state, error = " << gravity_state_.Gp_.transpose()
        << ", " << gravity_state_.dGp_.transpose();
    
    // Recalculate states, this also sets all error states to zero
    VLOG(1) << "Recalculating states";
    gravity_state_.Recalculate();
    cam_to_imu_pose_.Recalculate();
    for (int i=0; i<imu_states_->n_msgs(); i++) {imu_states_->at(i).Recalculate();}
    for (int i=0; i<cam_poses_->n_msgs(); i++) {cam_poses_->at(i).Recalculate();}
    for (int i=0; i<cam_to_imu_poses_->n_msgs(); i++) {cam_to_imu_poses_->at(i).Recalculate();}
    
    // Report results
    VLOG(1) << "IMU estimates:";
    VLOG(1) << "Gravity = " << gravity_state_.Gp_.transpose() << ", " << gravity_state_.Gp_.norm() << " (m/s^2)";
    VLOG(1) << "Gravity stdev = " << imu_gravity_stdev_.transpose() << " (m/s^2)";
    VLOG(1) << "Gravity = " << gravity_state_.Gp_.transpose()/options_.integration_options.accel_factor << ", "
        << gravity_state_.Gp_.norm()/options_.integration_options.accel_factor << " (LSB)";
    VLOG(1) << "Gravity stdev = " << imu_gravity_stdev_.transpose()/options_.integration_options.accel_factor << " (LSB)";
    
    VLOG(1) << "Cam-Imu pose = " << cam_to_imu_pose_.GpL_.transpose() << " (m)";
    VLOG(1) << "Cam-Imu pose stdev = " << cam_imu_pose_stdev_.block<3,1>(0,0).transpose() << " (m)";
    Eigen::AngleAxisd c2iaa(cam_to_imu_pose_.Quaternion());
    VLOG(1) << "Cam-Imu pose aa = " << c2iaa.axis().transpose() << ", " << c2iaa.angle()*DegreesPerRadian << " (deg)";
    VLOG(1) << "Cam-Imu pose aa stdev = " << cam_imu_pose_stdev_.block<3,1>(3,0).transpose()*DegreesPerRadian << " (deg)";
    
    SaveStatesToFile(options_.save_filename, "init2");
    
    return TagVioInit();
    //return true;
  }
  
  /* LookupTagSize
   * Searches the tag size in prior knowledge */
  double LookupTagSize(const std::string& id) const {
    return options_.default_april_tag_size;
  }
  
  // Checks if tag is origin tag
  //  Map could be loaded from someplace. In that case this would use the map to decide.
  bool IsOriginTag(const anantak::StaticAprilTagState *tagTj) const {
    bool is_origin_tag = (tagTj->pose_.Quaternion().isApprox(Eigen::Quaterniond::Identity()) &&
        tagTj->pose_.Position().isZero());
    return is_origin_tag;
  }
  
  // Look for timestamp in imu_residuals starting from the search index. Return the index
  bool FindTimestampInImuResiduals(const int64_t& ts, anantak::ImuResidualFunction **imu_constraint,
      int32_t& search_idx) {
    // Simple linear search starting from the search index.
    bool found_constraint = false;
    *imu_constraint = NULL;
    search_idx = std::min(std::max(0, search_idx), imu_residuals_->n_msgs()-1);
    while (!found_constraint && search_idx>=0 && search_idx<imu_residuals_->n_msgs()) {
      found_constraint = (ts >= imu_residuals_->at(search_idx).state0_->timestamp_ &&
                          ts <  imu_residuals_->at(search_idx).state1_->timestamp_);
      //VLOG(1) << "  idx = " << search_idx << " ts0,1 = "
      //    << imu_residuals_->at(search_idx).state0_->timestamp_ << " "
      //    << imu_residuals_->at(search_idx).state1_->timestamp_ << " " << found_constraint;
      if (found_constraint) {
        *imu_constraint = imu_residuals_->at_ptr(search_idx);
      } else {
        if (ts < imu_residuals_->at(search_idx).state0_->timestamp_) {
          search_idx--;
        } else if (ts >= imu_residuals_->at(search_idx).state1_->timestamp_) {
          search_idx++;
        }
      }
    }
    return found_constraint;
  }
  
  // Find the imu-cam-pose using timestamp
  bool FindTimestampInImuToCamResiduals(const int64_t& ts, anantak::Pose3dState **c2i_pose,
      int32_t& search_idx) {
    // Simple linear search starting from the search index.
    bool found_constraint = false;
    *c2i_pose = NULL;
    search_idx = std::min(std::max(0, search_idx), cam_to_imu_poses_->n_msgs()-1);
    while (!found_constraint && search_idx>=0 && search_idx<cam_to_imu_poses_->n_msgs()) {
      found_constraint = (ts == cam_to_imu_poses_->at(search_idx).timestamp_);
      if (found_constraint) {
        *c2i_pose = cam_to_imu_poses_->at_ptr(search_idx);
      } else {
        if (ts < cam_to_imu_poses_->at(search_idx).timestamp_) {
          search_idx--;
        } else if (ts > cam_to_imu_poses_->at(search_idx).timestamp_) {
          search_idx++;
        }
      }
    }
    return found_constraint;
  }
  
  // Find the camera pose using timestamp
  bool FindTimestampInCamPoses(const int64_t& ts, anantak::Pose3dState **poseC,
      int32_t& search_idx) {
    // Simple linear search starting from the search index.
    bool found_pose = false;
    *poseC = NULL;
    search_idx = std::min(std::max(0, search_idx), cam_poses_->n_msgs()-1);
    while (!found_pose && search_idx>=0 && search_idx<cam_poses_->n_msgs()) {
      found_pose = (ts == cam_poses_->at(search_idx).timestamp_);
      if (found_pose) {
        *poseC = cam_poses_->at_ptr(search_idx);
      } else {
        if (ts < cam_poses_->at(search_idx).timestamp_) {
          search_idx--;
        } else if (ts > cam_poses_->at(search_idx).timestamp_) {
          search_idx++;
        }
      }
    }
    return found_pose;
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
  
  // Look for the origin tag in tag_poses_. Returns a pointer to tag pose state
  bool FindOriginTagInTagMap(anantak::StaticAprilTagState **tagTj) {
    // Simple linear search through the tag map. Inefficient. We can do better
    bool tag_found = false;
    int i=0;
    *tagTj = NULL;
    while (!tag_found && i<tag_poses_->n_msgs()) {
      tag_found = IsOriginTag(tag_poses_->at_ptr(i));
      if (tag_found) {
        *tagTj = tag_poses_->at_ptr(i);
      }
      i++;
    }
    return tag_found;
  }
  
  
  bool TagVioInit() {
    
    // Redo TagVio with tag views. Last time we ran TagVIO assuming fixed camera poses. Now we
    // add Tag view constraints at the camera positions. IMU states are reused from last fitting.
    
    // Check. IMU states in previous optimization were set at camera timestamps.
    // First IMU state is the one at this camera timestamp. If not we can not continue.
    anantak::ImuState *starting_imu_state = imu_states_->at_ptr(0);
    if (starting_imu_state->timestamp_ !=
        init_cam_readings_->at(init_cam_data_starting_idx_).imu_interp.timestamp) {
      LOG(ERROR) << "Expected to see imu states beginning ts == cam readings starting ts. Not so.";
      return false;
    }
    
    // Timestamps of the cam-to-imu pose states must also match those of imu_states. Check.
    anantak::Pose3dState *starting_cam_to_imu_pose = cam_to_imu_poses_->at_ptr(0);
    if (starting_cam_to_imu_pose->timestamp_ != starting_imu_state->timestamp_) {
      LOG(ERROR) << "Expected starting_cam_to_imu_pose->timestamp_ == starting_imu_state->timestamp_"
          << " Not so. Exit.";
      return false;
    }
    
    // Starting calculations
    Eigen::Quaterniond T0qC0 = init_cam_readings_->at(init_cam_data_starting_idx_).cam_pose.quat;
    Eigen::Matrix3d C0rT0(T0qC0.conjugate());
    Eigen::Vector3d T0pC0 = init_cam_readings_->at(init_cam_data_starting_idx_).cam_pose.posn;
    //Eigen::Quaterniond CqI = cam_to_imu_pose_.Quaternion();
    //Eigen::Matrix3d CrI(CqI);
    //Eigen::Vector3d IpC = cam_to_imu_pose_.Position();
    Eigen::Quaterniond C0qI0 = starting_cam_to_imu_pose->Quaternion();
    Eigen::Matrix3d C0rI0(C0qI0);
    Eigen::Vector3d I0pC0 = starting_cam_to_imu_pose->Position();
    
    // Starting IMU reading establishes the World frame
    Eigen::Quaterniond I0qW = init_cam_readings_->at(init_cam_data_starting_idx_).imu_interp.quaternion.conjugate();
    Eigen::Matrix3d I0rW(I0qW);
    // We position World frame origin at the tag map origin. Only rotation relates the two.
    Eigen::Vector3d WpT0 = Eigen::Vector3d::Zero();
    //  T0qW = T0qC0 * CqI * I0qW - this will be kept constant as it serves as reference
    //Eigen::Quaterniond T0qW = T0qC0 * CqI * I0qW;
    Eigen::Quaterniond T0qW = T0qC0 * C0qI0 * I0qW;
    Eigen::Matrix3d T0rW(T0qW);
    // WpI0 = WpT0 + W.T0pC0 + W.CpI = WpT0 + WrT0*T0pC0 + WrI0*IrC*CpI = WpT0 + WrT0*T0pC0 - WrI0*IpC
    //Eigen::Vector3d WpI0 = WpT0 + T0rW.transpose()*T0pC0 - I0rW.transpose()*IpC;
    Eigen::Vector3d WpI0 = WpT0 + T0rW.transpose()*T0pC0 - I0rW.transpose()*I0pC0;
    
    // Report
    //VLOG(1) << "WpT0 = " << WpT0.transpose();
    //VLOG(1) << "WpI0 = " << WpI0.transpose();
    
    // Our current IMU states' poses are in a different reference frame. Call it a Zero frame (Z).
    // All states poses are IiqZ, ZpIi. We need to recalculate these wrt W, ie. IiqW, WpIi.
    // We know [I0qW, WpI0] from above. This fixes the World frame.
    // So using [I0qZ, ZpI0] and [I0qW, WpI0], we need [WqZ, ZpW] to get [IiqW, WpIi] as follows:
    //  IiqW = IiqZ * ZqW
    //  WpIi = WrZ * ( ZpIi - ZpW )
    // where
    //  ZqW = ZqI0 * I0qW
    //  ZpW = ZrI0 * ( I0pW - I0pZ ) = ZrI0 * ( I0rZ*ZpI0 - I0rW*WpI0 ) = ZpI0 - ZrW * WpI0
    Eigen::Quaterniond I0qZ = starting_imu_state->Quaternion();
    Eigen::Vector3d ZpI0 = starting_imu_state->Position();
    Eigen::Quaterniond ZqW = I0qZ.conjugate() * I0qW;
    Eigen::Matrix3d WrZ(ZqW.conjugate());
    Eigen::Vector3d ZpW = ZpI0 - WrZ.transpose()*WpI0;
    // Transform all IMU states
    for (int i=0; i<imu_states_->n_msgs(); i++) {
      Eigen::Quaterniond IiqZ = imu_states_->at(i).Quaternion();
      Eigen::Vector3d ZpIi = imu_states_->at(i).Position();
      Eigen::Quaterniond IiqW = IiqZ * ZqW;
      Eigen::Vector3d WpIi = WrZ * (ZpIi - ZpW);
      Eigen::Vector3d WvIi = WrZ * imu_states_->at(i).GvI_; // WvIi = WrZ * ZvIi
      imu_states_->at(i).IqvG_ = IiqW.coeffs();
      imu_states_->at(i).GpI_  = WpIi;
      imu_states_->at(i).GvI_  = WvIi;
      // biases do not change as they are in IMU body frame
      
      // Report
      if (i==0) VLOG(1) << "  #" << i << " " << WpIi.transpose();
    }
    
    // Transform gravity state - Wg = WrZ * Zg
    Eigen::Vector3d Wg = WrZ * gravity_state_.Position();
    gravity_state_.SetFromVector3d(&Wg);
    
    // Indexes to track IMU state creation
    int32_t imu_data_idx = init_imu_data_starting_idx_;
    int32_t cam_data_idx = init_cam_data_starting_idx_;
    int32_t states_idx = 0;   // at the first camera generated state
    
    // Increment imu idx to position after first camera timestamp
    imu_data_idx++;
    
    // Clear the IMU residuals
    imu_residuals_->Clear();
    VLOG(1) << "Number of imu states, residuals before residuals are built up = "
        << imu_states_->n_msgs() << " " << imu_residuals_->n_msgs();
    
    // Clear IMU-to-cam pose residuals
    noisy_imu_to_cam_pose_residuals_->Clear();
    noisy_imu_to_cam_pose_change_residuals_->Clear();
    VLOG(1) << "Number of imu-to-cam states, residuals before residuals are built up = "
        << cam_to_imu_poses_->n_msgs() << " " << noisy_imu_to_cam_pose_residuals_->n_msgs() << " "
        << noisy_imu_to_cam_pose_change_residuals_->n_msgs();
    
    // Quick check
    if (imu_states_->n_msgs() < 2) {
      LOG(ERROR) << "Less than 2 states exist?! Strange. Nothing to do.";
      return false;
    }
    
    // Create first imu residual
    anantak::ImuState *imu_state0 = &imu_states_->at(states_idx);
    anantak::ImuState *imu_state1 = &imu_states_->at(states_idx+1);
    anantak::ImuResidualFunction *imu_residual0 = imu_residuals_->next_mutable_element();    
    
    imu_residual0->Create(imu_state0, imu_state1, &gravity_state_,
        init_cam_readings_->at(cam_data_idx).imu_interp,
        options_.integration_options);
    VLOG(1) << "Created starting residual. Keeps an integral history of length = " <<
        imu_residual0->integrals_history_.size();
    
    // Create first imu-to-cam residual
    anantak::Pose3dState *cam_to_imu_pose0 = &cam_to_imu_poses_->at(states_idx);
    anantak::NoisyPoseResidual *noisy_i2c_pose_resid0 = noisy_imu_to_cam_pose_residuals_->next_mutable_element();
    
    noisy_i2c_pose_resid0->Create(&cam_to_imu_pose_, cam_to_imu_pose0, &options_.imu_to_cam_pose_noise_options);
    
    // Increment cam index as starting camera has been 'consumed'
    cam_data_idx++;
    
    // Statistics
    int32_t num_state_readings = 1; // counting the first camera reading 
    int32_t num_imu_readings = 0;
    int32_t num_equal_timestamps = 0;
    
    // Create IMU residuals
    while (imu_data_idx<init_imu_readings_->n_msgs() && states_idx<imu_states_->n_msgs()-1 &&
        cam_data_idx < imu_states_->n_msgs()) {
      
      // Does this IMU reading belong to this period?
      bool add_reading_to_curr_residual = (
          (init_imu_readings_->at(imu_data_idx).timestamp > imu_states_->at(states_idx).timestamp_)
          && (init_imu_readings_->at(imu_data_idx).timestamp < imu_states_->at(states_idx+1).timestamp_)
      );
      
      // Add reading to current residual
      if (add_reading_to_curr_residual) {
        num_imu_readings++;
        
        // Add this reading to the current residual
        anantak::ImuResidualFunction *imu_resid = imu_residuals_->mutable_element();
        if (imu_resid->IsOpen()) {
          if (!imu_resid->AddReading(init_imu_readings_->at(imu_data_idx))) {
            LOG(ERROR) << "Could not add imu reading. Skipping it.";
          }
        } else {
          LOG(ERROR) << "Expected the current residual to be open. Something is wrong. Exit";
          return false;
        }
        
        // Increment IMU index
        imu_data_idx++;
      }
      
      // Reading is past or at the end of the current residual
      if (!add_reading_to_curr_residual) {
        num_state_readings++;
        
        // Close the last imu constraint
        anantak::ImuResidualFunction *imu_resid = imu_residuals_->mutable_element();
        if (imu_resid->IsOpen()) {                  // Important - state is not propagated
          if (!imu_resid->AddEndStateReading(init_cam_readings_->at(cam_data_idx).imu_interp)) {
            LOG(ERROR) << "Could not close state by adding interp cam reading. Exit.";
            return false;
          }
        } else {
          LOG(ERROR) << "Expected the current residual to be open. Could not close it. Exit";
          return false;
        }
        
        // Increment states index
        states_idx++;
        anantak::ImuState *curr_state = &imu_states_->at(states_idx);
        anantak::ImuState *next_state = &imu_states_->at(states_idx+1);
        if (init_cam_readings_->at(cam_data_idx).imu_interp.timestamp != curr_state->timestamp_) {
          LOG(ERROR) << "init_cam_readings_->at(cam_data_idx).imu_interp.timestamp != curr_state->timestamp.";
          return false;
        }
        
        // Add a new imu constraint between last and next states
        anantak::ImuResidualFunction *new_imu_resid = imu_residuals_->next_mutable_element();
        new_imu_resid->Create(curr_state, next_state, &gravity_state_,
            init_cam_readings_->at(cam_data_idx).imu_interp, options_.integration_options);
        
        // Find references to imu-to-cam states
        anantak::Pose3dState *last_cam_to_imu_pose = &cam_to_imu_poses_->at(states_idx-1);
        anantak::Pose3dState *curr_cam_to_imu_pose = &cam_to_imu_poses_->at(states_idx);
        // Check that timestamps of imu_state and cam-to-imu pose state are the same
        if (curr_cam_to_imu_pose->timestamp_ != curr_state->timestamp_) {
          LOG(ERROR) << "Expected curr_cam_to_imu_pose->timestamp_ == curr_state->timestamp_"
              << " Not so. Exit.";
          return false;
        }
        
        // Add a cam-to-imu pose residual
        anantak::NoisyPoseResidual *curr_noisy_i2c_pose_resid = noisy_imu_to_cam_pose_residuals_->next_mutable_element();
        curr_noisy_i2c_pose_resid->Create(&cam_to_imu_pose_, curr_cam_to_imu_pose, &options_.imu_to_cam_pose_noise_options);
        
        // Add a cam-to-imu pose change residual
        anantak::NoisyPoseResidual *curr_noisy_i2c_pose_change_resid = noisy_imu_to_cam_pose_change_residuals_->next_mutable_element();
        curr_noisy_i2c_pose_change_resid->Create(last_cam_to_imu_pose, curr_cam_to_imu_pose, &options_.imu_to_cam_pose_change_options);
        
        // If timestamps of state and imu reading are equal, we skip the imu reading
        if (init_imu_readings_->at(imu_data_idx).timestamp == curr_state->timestamp_) {
          num_equal_timestamps++;
          num_imu_readings++;
          imu_data_idx++;
        }
        
        // Increment cam data index as this reading has been used
        cam_data_idx++;
      }
      
    }
    // Report
    VLOG(1) << "Number of states and imu readings seen: " << num_state_readings << ", "
        << num_imu_readings << " Equal timestamps: " << num_equal_timestamps;
    VLOG(1) << "Number of IMU states used = " << imu_states_->n_msgs();
    VLOG(1) << "Number of IMU constraints added = " << imu_residuals_->n_msgs();
    VLOG(1) << "Number of IMU-to-cam pose residuals added = " << noisy_imu_to_cam_pose_residuals_->n_msgs();
    VLOG(1) << "Number of IMU-to-cam pose change residuals added = " << noisy_imu_to_cam_pose_change_residuals_->n_msgs();
    
    // Save starting imu states for plotting
    SaveStatesToFile(options_.save_filename, "init3");
    SaveResidualsToFile(options_.save_filename, "init3");
    
    // Initiate the tag poses in tag map
    int32_t num_tags = imu_init_tag_camera_->tag_camera_.connected_tags_sizes_[options_.tags_set_num];
    for (int i_tag=0; i_tag<num_tags; i_tag++) {
      const AprilTagCamera::TagCompositePose& tag_comp_pose =
          imu_init_tag_camera_->tag_camera_.tag_poses_[options_.tags_set_num]->at(i_tag);
      double size = LookupTagSize(tag_comp_pose.tag_id);
      Eigen::Quaterniond TjqT0 = tag_comp_pose.pose.rotn_q.conjugate();
      Eigen::Vector3d T0pTj = tag_comp_pose.pose.posn;
      // Create a new tag_pose state
      anantak::StaticAprilTagState *tag_pose = tag_poses_->next_mutable_element();
      tag_pose->Create(&tag_comp_pose.tag_id, &TjqT0, &T0pTj, &size);
      // Set the covariances
      Eigen::Matrix<double,6,1> pose_stdev;
      pose_stdev <<  options_.starting_tag_angle_sigma, options_.starting_tag_angle_sigma, options_.starting_tag_angle_sigma,
          options_.starting_tag_position_sigma, options_.starting_tag_position_sigma, options_.starting_tag_position_sigma;
      tag_pose->pose_.covariance_ = pose_stdev.asDiagonal();
      tag_pose->size_.covariance_ = options_.default_april_tag_size_sigma;
    }
    // Report
    VLOG(1) << "Number of tags initiated in map = " << num_tags;
    for (int i=0; i<num_tags; i++) {
      VLOG(2) << "  " << tag_poses_->at(i).tag_id_ << " "
          << tag_poses_->at(i).pose_.Quaternion().coeffs().transpose() << " "
          << tag_poses_->at(i).pose_.Position().transpose() << " "
          << tag_poses_->at(i).size_.Value();
    }
    SaveTagMapToFile(options_.save_filename, "init3");
    
    // Create the tag map pose
    tag_map_pose_.Create(&T0qW, &WpT0);
    
    // Create camera intrinsics state
    Eigen::Matrix3d K_mat = imu_init_tag_camera_->tag_camera_.camera_K_;
    camera_intrinsics_.Create(&K_mat);
    VLOG(1) << "Starting camera matrix = \n" << camera_intrinsics_.CameraMatrix();
    
    int32_t residual_idx = 0;
    VLOG(1) << "Num of April tags seen = " << april_tag_readings_->n_msgs();
    
    // Create Tag view states for each camera pose
    for (int i_tag_rdng=0; i_tag_rdng<april_tag_readings_->n_msgs(); i_tag_rdng++) {
    //for (int i_tag_rdng=0; i_tag_rdng<100; i_tag_rdng++) {
      
      anantak::AprilTagReadingType *tag_rdng = april_tag_readings_->at_ptr(i_tag_rdng);
      anantak::ImuResidualFunction *imu_constraint = NULL;
      anantak::ImuState *poseI = NULL;
      anantak::StaticAprilTagState *tagTj = NULL;
      anantak::Pose3dState *cam_to_imu_pose = NULL;
      
      // Find the tag_rdng timestamp in the list of imu residuals
      bool found_resid = FindTimestampInImuResiduals(tag_rdng->timestamp, &imu_constraint, residual_idx);
      if (imu_constraint) {
        // The residual must be Closed to be used
        if (imu_constraint->IsOpen()) {
          LOG(WARNING) << "Residual is open. Can not use it. Skipping it.";
          continue;
        }
        // Get the corresponding IMU state
        poseI = imu_constraint->state0_;
        if (!poseI) {
          LOG(WARNING) << "Not expected to not have beginning state for an imu residual! Skipping";
          continue;
        }
      } else {
        LOG(WARNING) << "Could not find imu constraint for april tag view at timestamp = " << tag_rdng->timestamp;
        LOG(WARNING) << "Current residual_idx = " << residual_idx;
        LOG(WARNING) << "Tag view ts - First imu state timestamp = " << tag_rdng->timestamp - imu_states_->front().timestamp_;
        LOG(WARNING) << "Tag view ts - Last imu state timestamp = " << tag_rdng->timestamp - imu_states_->back().timestamp_;
        continue;
      }
      
      // Find this tag in list of tags
      bool found_tag = FindTagInTagMap(tag_rdng->tag_id, &tagTj);
      if (tagTj) {
        //VLOG(3) << "Found tag in map. tag_id = " << tag_rdng->tag_id << " " << tagTj->tag_id_;
      } else {
        LOG(WARNING) << "Did not find tag in tag map. Not expected. tag_id = " << tag_rdng->tag_id;
        continue;
      }
      
      // Find the corresponding imu-to-cam state
      bool found_i2c = FindTimestampInImuToCamResiduals(poseI->timestamp_, &cam_to_imu_pose, residual_idx);
      if (cam_to_imu_pose) {
        //
      } else {
        LOG(WARNING) << "Did not find imu-to-cam pose. Not expected.";
        continue;
      }
      
      /*bool Create(const anantak::AprilTagReadingType *tag_view,
          anantak::ImuState *poseI, anantak::Vector3dState* gravity,
          anantak::Pose3dState *poseItoC, anantak::Pose3dState *poseWtoT0,
          anantak::StaticAprilTagState *tagTj, anantak::CameraIntrinsicsState *camera,
          const anantak::ImuResidualFunction *imu_constraint,
          AprilTagVioResidual::Options *options,
          bool is_zero_tag = false)*/
      anantak::AprilTagVioResidual *tag_vio_resid = tag_vio_residuals_->next_mutable_element();
      bool created = tag_vio_resid->Create(tag_rdng, poseI, &gravity_state_, cam_to_imu_pose,
          &tag_map_pose_, tagTj, &camera_intrinsics_, imu_constraint,
          &options_.apriltag_vio_residual_options);
      if (!created) {
        LOG(ERROR) << "Could not create tag view residual. Skipping.";
        continue;
      }
      
      // Look at the created tag view residual
      //VLOG(1) << "  T0pCi = " << init_cam_readings_->at(init_cam_data_starting_idx_+residual_idx).cam_pose.posn.transpose();
      //VLOG(1) << "   WpIi = " << poseI->Position().transpose();
      
    }
    // Report
    VLOG(1) << "Created tag view residuals. num = " << tag_vio_residuals_->n_msgs()
        << " of total views = " << april_tag_readings_->n_msgs();
    
    // Find origin tag in tag map
    anantak::StaticAprilTagState *origin_tag = NULL;
    bool found_origin = FindOriginTagInTagMap(&origin_tag);
    if (!origin_tag) {
      LOG(ERROR) << "Could not find the origin tag in tagmap. Quitting.";
      return false;
    }
    
    // Initiate the cam_to_imu_pose_ using data used in convex optimization
    Eigen::Quaterniond CqI_prior = imu_init_tag_camera_->imu_options_.CqI_measured;
    Eigen::Matrix3d CrI_prior(CqI_prior);
    Eigen::Vector3d IpC_prior = -CrI_prior.transpose() * imu_init_tag_camera_->imu_options_.CpI_measured;
    
    // Create a prior for cam-to-imu pose
    NoisyPosePrior cam_to_imu_pose_prior_;
    cam_to_imu_pose_prior_.Create(&CqI_prior, &IpC_prior, &cam_to_imu_pose_, &options_.imu_to_cam_pose_prior_options);
    
    // Prepare all states for optimization
    
    // Prepare all imu constraints for optimization
    //  Any un-closed constraints will not be used for optimization
    int32_t num_open_residuals = 0;
    int32_t num_ready_to_optimize = 0;
    int32_t min_integral_history = 1000; // IMU residual should never have more than 1000 readings.
    int32_t max_integral_history = 0;
    int32_t total_integrals_history = 0;
    float avg_integral_history = 0.;
    int32_t min_readings = 1000; // IMU residual should never have more than 1000 readings.
    int32_t max_readings = 0;
    int32_t total_readings = 0;
    float avg_readings = 0.;
    for (int i=0; i<imu_residuals_->n_msgs(); i++) {
      if (!imu_residuals_->at(i).IsOpen()) {
        if (!imu_residuals_->at(i).GetReadyToOptimize()) {
          LOG(ERROR) << "A closed residual is not ready for optimization. Can not continue.";
          return false;
        } else {
          num_ready_to_optimize++;
          min_integral_history = std::min(min_integral_history, imu_residuals_->at(i).num_integrals_stored_);
          max_integral_history = std::max(min_integral_history, imu_residuals_->at(i).num_integrals_stored_);
          total_integrals_history += imu_residuals_->at(i).num_integrals_stored_;
          min_readings = std::min(min_readings, imu_residuals_->at(i).num_readings_);
          max_readings = std::max(max_readings, imu_residuals_->at(i).num_readings_);
          total_readings += imu_residuals_->at(i).num_readings_;
        }
      } else {
        VLOG(1) << "Found an open constraint at idx = " << i;
        num_open_residuals++;
      }
    }
    avg_integral_history = float(total_integrals_history) / float(num_ready_to_optimize);
    avg_readings = float(total_readings) / float(num_ready_to_optimize);
    VLOG(1) << "Number of imu residuals ready to optimize = " << num_ready_to_optimize
        << ", num open = " << num_open_residuals;
    VLOG(1) << "Residuals can store max history length = " << imu_residuals_->front().max_integrals_history_;
    VLOG(1) << "Integrals history stored min, max, avg per residual= " << min_integral_history << " "
        << max_integral_history << " " << avg_integral_history;
    VLOG(1) << "Number of min, max, avg readings per residual = " << min_readings << " "
        << max_readings << " " << avg_readings;
    
    // Set all vio constraints to ready for optimization
    for (int i=0; i<tag_vio_residuals_->n_msgs(); i++) {
      if (!tag_vio_residuals_->at(i).GetReadyToOptimize()) {
        LOG(ERROR) << "A created vio residual is not ready for optimization. Not expected.";
        return false;
      }
    }
    
    // Prepare the imu-to-cam pose prior for optimization
    if (!cam_to_imu_pose_prior_.GetReadyToOptimize()) {
      LOG(ERROR) << "Could not get cam_to_imu_pose_prior_ ready for optimization";
      return false;
    }
    
    // Prepare imu-cam-poses for optimization
    for (int i=0; i<noisy_imu_to_cam_pose_residuals_->n_msgs(); i++) {
      if (!noisy_imu_to_cam_pose_residuals_->at(i).GetReadyToOptimize()) {
        LOG(ERROR) << "Noisy pose residual at i = " << i << " could not be readied for optimization";
      }
    }
    
    // Prepare imu-cam-pose changes for optimization
    for (int i=0; i<noisy_imu_to_cam_pose_change_residuals_->n_msgs(); i++) {
      if (!noisy_imu_to_cam_pose_change_residuals_->at(i).GetReadyToOptimize()) {
        LOG(ERROR) << "Noisy pose residual at i = " << i << " could not be readied for optimization";
      }
    }
    
    tag_map_pose_.SetErrorZero();
    
    VLOG(1) << "Done preparing states and residuals for optimization";
    
    // Build a problem by adding all constraints to it. Problem does not own constraints.
    ceres::Problem::Options problem_options;
    problem_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    ceres::Problem problem(problem_options);
    
    // Add imu residuals to the problem. Do not transfer ownership
    for (int i=0; i<imu_residuals_->n_msgs(); i++) {
      if (!imu_residuals_->at(i).IsOpen()) {
        ceres::CostFunction* i_residual = &imu_residuals_->at(i);
        ceres::LossFunction* quad_loss = NULL;
        problem.AddResidualBlock(
          i_residual,
          quad_loss,
          imu_residuals_->at(i).state0_->error_,
          imu_residuals_->at(i).state1_->error_,
          imu_residuals_->at(i).gravity_->error_
        );
      }
    }
    
    // Add tagvio residuals to the problem. Do not transfer ownership.
    for (int i=0; i<tag_vio_residuals_->n_msgs(); i++) {
      ceres::CostFunction* vio_residual = &tag_vio_residuals_->at(i);
      ceres::LossFunction* quad_loss = NULL;
      problem.AddResidualBlock(
        vio_residual,
        quad_loss,
        tag_vio_residuals_->at(i).poseI_->error_,
        tag_vio_residuals_->at(i).gravity_->error_,
        tag_vio_residuals_->at(i).poseItoC_->error_,
        tag_vio_residuals_->at(i).poseWtoT0_->error_,
        tag_vio_residuals_->at(i).poseWtoT0_->error_+3,
        tag_vio_residuals_->at(i).tagTj_->pose_.error_,
        &tag_vio_residuals_->at(i).tagTj_->size_.error_,
        tag_vio_residuals_->at(i).camera_->error_
      );
    }
    
    // Add imu-to-cam noisy pose residuals
    for (int i=0; i<noisy_imu_to_cam_pose_residuals_->n_msgs(); i++) {
      ceres::CostFunction* i_residual = &noisy_imu_to_cam_pose_residuals_->at(i);
      ceres::LossFunction* quad_loss = NULL;
      problem.AddResidualBlock(
        i_residual,
        quad_loss,
        noisy_imu_to_cam_pose_residuals_->at(i).expectation_->error_,
        noisy_imu_to_cam_pose_residuals_->at(i).measurement_->error_
      );
    }
    
    // Add imu-to-cam noisy pose change residuals
    for (int i=0; i<noisy_imu_to_cam_pose_change_residuals_->n_msgs(); i++) {
      ceres::CostFunction* i_residual = &noisy_imu_to_cam_pose_change_residuals_->at(i);
      ceres::LossFunction* quad_loss = NULL;
      problem.AddResidualBlock(
        i_residual,
        quad_loss,
        noisy_imu_to_cam_pose_change_residuals_->at(i).expectation_->error_,
        noisy_imu_to_cam_pose_change_residuals_->at(i).measurement_->error_
      );
    }
    
    // Add a prior for imu-to-cam pose
    {
      ceres::CostFunction* i_residual = &cam_to_imu_pose_prior_;
      ceres::LossFunction* quad_loss = NULL;
      problem.AddResidualBlock(
        i_residual,
        quad_loss,
        cam_to_imu_pose_prior_.measurement_->error_
      );
    }
    
    // Add priors for tag poses
    
    
    // Set Tag0 pose constant
    problem.SetParameterBlockConstant(origin_tag->pose_.error_);
    
    // Set WpT0 as constant
    problem.SetParameterBlockConstant(tag_map_pose_.error_+3);
    
    // Set camera intrinsics as constant
    problem.SetParameterBlockConstant(camera_intrinsics_.error_);
    
    // Set all tag sizes constant
    for (int i=0; i<tag_poses_->n_msgs(); i++) {
      problem.SetParameterBlockConstant(&tag_poses_->at(i).size_.error_);
    }
    
    // Solve the problem
    ceres::Solver::Options solver_options;
    solver_options.max_num_iterations = 300;
    solver_options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary solver_summary;
    ceres::Solve(solver_options, &problem, &solver_summary);
    if (true) std::cout << solver_summary.FullReport() << std::endl;
    
    // Calculate covariances
    VLOG(1) << "Calculating covariances";
    Eigen::Matrix3d imu_gravity_cov_; Eigen::Vector3d imu_gravity_stdev_;
    Eigen::Matrix<double,6,6> cam_imu_pose_cov_; Eigen::Matrix<double,6,1> cam_imu_pose_stdev_;
    ceres::Covariance::Options covariance_options;
    ceres::Covariance covariance(covariance_options);
    std::vector<std::pair<const double*, const double*>> covariance_blocks;
    covariance_blocks.push_back(std::make_pair(gravity_state_.error_, gravity_state_.error_));
    covariance_blocks.push_back(std::make_pair(cam_to_imu_pose_.error_, cam_to_imu_pose_.error_));
    CHECK(covariance.Compute(covariance_blocks, &problem));
    covariance.GetCovarianceBlock(gravity_state_.error_, gravity_state_.error_, imu_gravity_cov_.data());
    covariance.GetCovarianceBlock(cam_to_imu_pose_.error_, cam_to_imu_pose_.error_, cam_imu_pose_cov_.data());
    
    imu_gravity_stdev_ = imu_gravity_cov_.diagonal().cwiseSqrt();
    cam_imu_pose_stdev_ = cam_imu_pose_cov_.diagonal().cwiseSqrt();
    
    // Recalculate states, this also sets all error states to zero
    VLOG(1) << "Recalculating states";
    for (int i=0; i<imu_states_->n_msgs(); i++) {imu_states_->at(i).Recalculate();}
    for (int i=0; i<tag_poses_->n_msgs(); i++) {tag_poses_->at(i).Recalculate();}
    gravity_state_.Recalculate();
    cam_to_imu_pose_.Recalculate();
    tag_map_pose_.Recalculate();
    camera_intrinsics_.Recalculate();
    
    // Report results
    VLOG(1) << "IMU estimates:";
    VLOG(1) << "Gravity = " << gravity_state_.Gp_.transpose() << ", " << gravity_state_.Gp_.norm() << " (m/s^2)";
    VLOG(1) << "Gravity stdev = " << imu_gravity_stdev_.transpose() << " (m/s^2)";
    VLOG(1) << "Gravity = " << gravity_state_.Gp_.transpose()/options_.integration_options.accel_factor << ", "
        << gravity_state_.Gp_.norm()/options_.integration_options.accel_factor << " (LSB)";
    VLOG(1) << "Gravity stdev = " << imu_gravity_stdev_.transpose()/options_.integration_options.accel_factor << " (LSB)";
    
    VLOG(1) << "Cam-Imu pose = " << cam_to_imu_pose_.GpL_.transpose() << " (m)";
    VLOG(1) << "Cam-Imu pose stdev = " << cam_imu_pose_stdev_.block<3,1>(0,0).transpose() << " (m)";
    Eigen::AngleAxisd c2iaa(cam_to_imu_pose_.Quaternion());
    VLOG(1) << "Cam-Imu pose aa = " << c2iaa.axis().transpose() << ", " << c2iaa.angle()*DegreesPerRadian << " (deg)";
    VLOG(1) << "Cam-Imu pose aa stdev = " << cam_imu_pose_stdev_.block<3,1>(3,0).transpose()*DegreesPerRadian << " (deg)";
    
    // Save for plotting
    SaveStatesToFile(*imu_states_, options_.save_filename, "init4");
    SaveResidualsToFile(options_.save_filename, "init4");
    SaveTagMapToFile(options_.save_filename, "init4");
    
    return true;
  }
  
  /* Run full Tag VIO on recorded data
   * Use the estimates of optimization run by keeping camera poses fixed. But now run full Tag VIO.
   * IMU states are created using wall clock time.
   * IMU readings connect these states.
   * Tag map is initiated with tag pose states in tag frame.
   * IMU world frame is positioned at Tag map frame but is rotated, meaning we keep WpT0 = 0
   * Each Tag View has a residual that is function of last IMU state, Tag Map pose and Tag pose.
   * This is starting point of running real-time sliding-window tag vio. 
   */
  bool TagVioInit2() {
    
    // Initiate states
    //  Tag poses in tag map
    //  Tag map pose in world frame. These are at the same point, only rotations are different.
    //  IMU poses in world frame (located at tag map origin)
    //  Gravity in world frame
    //  Cam-to-imu pose
    
    // Starting calculations
    Eigen::Quaterniond T0qC0 = init_cam_readings_->at(init_cam_data_starting_idx_).cam_pose.quat;
    Eigen::Matrix3d C0rT0(T0qC0.conjugate());
    Eigen::Vector3d T0pC0 = init_cam_readings_->at(init_cam_data_starting_idx_).cam_pose.posn;
    Eigen::Quaterniond CqI = cam_to_imu_pose_.Quaternion();
    Eigen::Matrix3d CrI(CqI);
    Eigen::Vector3d IpC = cam_to_imu_pose_.Position();
    
    // IMU states in previous optimization were set at camera timestamps. First IMU state is the
    // one at this camera timestamp. Check this.
    anantak::ImuState *starting_imu_state = imu_states_->at_ptr(0);
    if (starting_imu_state->timestamp_ !=
        init_cam_readings_->at(init_cam_data_starting_idx_).imu_interp.timestamp) {
      LOG(ERROR) << "Expected to see imu states beginning ts == cam readings starting ts. Not so.";
      return false;
    }
    // Check if the starting state rotation and position are zero. They should be.
    if (!starting_imu_state->Quaternion().isApprox(Eigen::Quaterniond::Identity()) ||
        !starting_imu_state->Position().isZero()) {
      LOG(WARNING) << "Expected the starting state quaternion and position to be zero. Not so.";
    } else {
      VLOG(1) << "Starting quaternion, position from previous optimization are zero as expected.";
    }
    
    
    // Starting state quaternion is 0 rotation. We use the IMU reading for World frame rotation.
    Eigen::Quaterniond I0qW = init_cam_readings_->at(init_cam_data_starting_idx_).imu_interp.quaternion.conjugate();
    Eigen::Matrix3d I0rW(I0qW);
    // We position World frame origin at the tag map origin. Only rotation relates the two.
    Eigen::Vector3d WpT0 = Eigen::Vector3d::Zero();
    //  T0qW = T0qC0 * CqI * I0qW - this will be kept constant as it serves as reference
    Eigen::Quaterniond T0qW = T0qC0 * CqI * I0qW;
    Eigen::Matrix3d T0rW(T0qW);
    // WpI0 = WpT0 + W.T0pC0 + W.CpI = WpT0 + WrT0*T0pC0 + WrI0*IrC*CpI = WpT0 + WrT0*T0pC0 - WrI0*IpC
    Eigen::Vector3d WpI0 = WpT0 + T0rW.transpose()*T0pC0 - I0rW.transpose()*IpC;
    
    // Velocity was measured in starting IMU frame. We turn this to World frame. Wv = WrI0 * I0v
    Eigen::Vector3d Wv = I0rW.transpose() * starting_imu_state->GvI_;
    // Biases are measured in IMU body frame
    Eigen::Vector3d I0bg = starting_imu_state->bg_;
    Eigen::Vector3d I0ba = starting_imu_state->ba_;
    
    // Gravity was measured in I0 frame. Transfer to World frame. Wg = WrI0 * I0g
    Eigen::Vector3d Wg = I0rW.transpose() * gravity_state_.Position();
    
    // Timestamps of the states are set by the wall clock, and not by the readings.
    std::vector<int64_t> state_timestamps_;
    VLOG(1) << "Using state_period = " << state_period_;    
    int64_t state_start_ts = init_cam_readings_->at(init_cam_data_starting_idx_).imu_interp.timestamp;
    int64_t state_end_ts = ((init_imu_readings_->back().timestamp / state_period_) + 1) * state_period_;
    int64_t state_ts = ((state_start_ts / state_period_) + 1) * state_period_;    // integer ops
    VLOG(1) << "Camera, States data start/end ts = " << state_start_ts << " " << state_ts << " " << state_end_ts;
    if (state_end_ts <= state_start_ts) {
      LOG(ERROR) << "state_end_ts <= state_start_ts !! Exit.";
      return false;
    }
    while (state_ts <= state_end_ts) {
      state_timestamps_.push_back(state_ts);
      state_ts += state_period_;
    }
    VLOG(1) << "Number of states = " << state_timestamps_.size();
    
    // Interpolate IMU readings for states' timestamps
    std::vector<anantak::LinearInterpolation> states_interp_;
    std::vector<anantak::ImuReadingType> states_interp_imu_readings_;
    anantak::InterpolateImuReadings(*init_imu_readings_, state_timestamps_,
        &states_interp_, &states_interp_imu_readings_);
    VLOG(1) << "Interpolated " << states_interp_imu_readings_.size() << " imu readings for states";
    
    // Indexes to track IMU state creation
    int32_t imu_data_idx = init_imu_data_starting_idx_;
    int32_t states_idx = 0;   // at the first state after camera generated state
    
    // Increment imu idx to position after first camera timestamp
    imu_data_idx++;
    
    // Number of states in the initiation problem are much greater than num of states in realtime.
    //  So we can not use the preallocated storage. Need to reallocate and deallocate, not ideal
    std::unique_ptr<anantak::CircularQueue<anantak::ImuState>> cq_imu_states_(new
        anantak::CircularQueue<anantak::ImuState>(state_timestamps_.size()+10));
    anantak::ImuResidualFunction prototype_imu_residual(num_imu_readings_per_residual_);
    std::unique_ptr<anantak::CircularQueue<anantak::ImuResidualFunction>> cq_imu_residuals_(new
        anantak::CircularQueue<anantak::ImuResidualFunction>(
            state_timestamps_.size()+10, prototype_imu_residual));
    VLOG(1) << "Created queues to hold imu states and residuals with lengths "
        << state_timestamps_.size()+10;
    
    // Clear IMU states
    VLOG(1) << "Number of imu states, residuals at the start = "
        << cq_imu_states_->n_msgs() << " " << cq_imu_residuals_->n_msgs();
    
    // Create the first two imu states and the first imu constraint
    anantak::ImuState *imu_state0 = cq_imu_states_->next_mutable_element();
    anantak::ImuState *imu_state1 = cq_imu_states_->next_mutable_element();
    anantak::ImuResidualFunction *imu_residual0 = cq_imu_residuals_->next_mutable_element();
    
    // Set gravity state from above calculations
    gravity_state_.SetFromVector3d(&Wg);
    
    // Set the starting state using results calculated above
    imu_state0->SetZero();    // Reset state
    imu_state0->SetTimestamp(state_start_ts);
    imu_state0->IqvG_ = I0qW.coeffs();
    imu_state0->GpI_ = WpI0;
    imu_state0->GvI_ = Wv;
    imu_state0->bg_ = I0bg;
    imu_state0->ba_ = I0ba;
    VLOG(1) << "Staring state: q = " << imu_state0->IqvG_.transpose() << " p = " << imu_state0->GpI_.transpose()
        << " v = " << imu_state0->GvI_.transpose() << " bg = " << imu_state0->bg_.transpose()
        << " ba = " << imu_state0->ba_.transpose();
    // Reset the next state, this will be populated when the residual 'closes'
    imu_state1->SetZero();
    imu_state1->SetTimestamp(states_interp_imu_readings_[states_idx].timestamp);
    // Create the first residual
    imu_residual0->Create(imu_state0, imu_state1, &gravity_state_,
        init_cam_readings_->at(init_cam_data_starting_idx_).imu_interp,
        options_.integration_options);
    VLOG(1) << "Created starting residual. Keeps a integral history of length = " <<
        imu_residual0->integrals_history_.size();
    
    // Statistics
    int32_t num_state_readings = 1; // counting the first camera reading 
    int32_t num_imu_readings = 0;
    int32_t num_equal_timestamps = 0;
    
    // Create IMU states
    while (imu_data_idx<init_imu_readings_->n_msgs() && states_idx<states_interp_imu_readings_.size()) {
      
      // Does this IMU reading belong to this period?
      bool add_reading_to_curr_residual = (init_imu_readings_->at(imu_data_idx).timestamp <
          states_interp_imu_readings_[states_idx].timestamp);
      
      // Add reading to current residual
      if (add_reading_to_curr_residual) {
        // This is an IMU reading
        num_imu_readings++;
        
        // Add this reading to the current residual
        anantak::ImuResidualFunction *imu_resid = cq_imu_residuals_->mutable_element();
        if (imu_resid->IsOpen()) {
          if (!imu_resid->AddReading(init_imu_readings_->at(imu_data_idx))) {
            LOG(ERROR) << "Could not add imu reading. Skipping it.";
          }
        } else {
          LOG(ERROR) << "Expected the current residual to be open. Something is wrong. Exit";
          return false;
        }
        
        // Increment IMU index
        imu_data_idx++;
      }
      
      // Reading is past or at the end of the current residual
      if (!add_reading_to_curr_residual) {
        num_state_readings++;
        
        // Close the last imu constraint
        anantak::ImuResidualFunction *imu_resid = cq_imu_residuals_->mutable_element();
        if (imu_resid->IsOpen()) {
          if (!imu_resid->AddEndStateReading(states_interp_imu_readings_[states_idx]), true) {
            LOG(ERROR) << "Could not close state by adding interp cam reading. Exit.";
            return false;
          }
        } else {
          LOG(ERROR) << "Expected the current residual to be open. Could not close it. Exit";
          return false;
        }
        
        // Add a new imu state at the next state timestamp
        anantak::ImuState *curr_state = cq_imu_states_->mutable_element();
        anantak::ImuState *next_state = cq_imu_states_->next_mutable_element();
        next_state->SetZero();
        next_state->SetTimestamp(states_interp_imu_readings_[states_idx+1].timestamp);
        
        // Add a new imu constraint between last and next states
        anantak::ImuResidualFunction *new_imu_resid = cq_imu_residuals_->next_mutable_element();
        new_imu_resid->Create(curr_state, next_state, &gravity_state_,
            states_interp_imu_readings_[states_idx], options_.integration_options);
        
        // If timestamps of state and imu reading are equal, we skip the imu reading
        if (init_imu_readings_->at(imu_data_idx).timestamp ==
          states_interp_imu_readings_[states_idx].timestamp) {
          num_equal_timestamps++;
          imu_data_idx++;
        }
        // Increment states index
        states_idx++;
      }
      
    }
    // Report
    VLOG(1) << "Number of states and imu readings seen: " << num_state_readings << ", "
        << num_imu_readings << " Equal timestamps: " << num_equal_timestamps;
    VLOG(1) << "Number of IMU states created = " << cq_imu_states_->n_msgs();
    VLOG(1) << "Number of IMU constraints added = " << cq_imu_residuals_->n_msgs();
    // Save starting imu states for plotting
    SaveStatesToFile(*cq_imu_states_, options_.save_filename, "init3");
    
    // Initiate the tag poses in tag map
    int32_t num_tags = imu_init_tag_camera_->tag_camera_.connected_tags_sizes_[options_.tags_set_num];
    for (int i_tag=0; i_tag<num_tags; i_tag++) {
      const AprilTagCamera::TagCompositePose& tag_comp_pose =
          imu_init_tag_camera_->tag_camera_.tag_poses_[options_.tags_set_num]->at(i_tag);
      double size = LookupTagSize(tag_comp_pose.tag_id);
      Eigen::Quaterniond TjqT0 = tag_comp_pose.pose.rotn_q.conjugate();
      Eigen::Vector3d T0pTj = tag_comp_pose.pose.posn;
      // Create a new tag_pose state
      anantak::StaticAprilTagState *tag_pose = tag_poses_->next_mutable_element();
      tag_pose->Create(&tag_comp_pose.tag_id, &TjqT0, &T0pTj, &size);
      // Set the covariances
      Eigen::Matrix<double,6,1> pose_stdev;
      pose_stdev <<  options_.starting_tag_angle_sigma, options_.starting_tag_angle_sigma, options_.starting_tag_angle_sigma,
          options_.starting_tag_position_sigma, options_.starting_tag_position_sigma, options_.starting_tag_position_sigma;
      tag_pose->pose_.covariance_ = pose_stdev.asDiagonal();
      tag_pose->size_.covariance_ = options_.default_april_tag_size_sigma;
    }
    
    // Initiate tag map pose in IMU world frame
    
    
    // Add AprilTag views residuals 
    
    // Prepare all imu constraints for optimization
    VLOG(1) << "Preparing all states and residuals for optimization";
    
    //  Any un-closed constraints will not be used for optimization
    int32_t num_open_residuals = 0;
    int32_t num_ready_to_optimize = 0;
    for (int i=0; i<cq_imu_residuals_->n_msgs(); i++) {
      if (!cq_imu_residuals_->at(i).IsOpen()) {
        if (!cq_imu_residuals_->at(i).GetReadyToOptimize()) {
          LOG(ERROR) << "A closed residual is not ready for optimization. Can not continue.";
          return false;
        } else {
          num_ready_to_optimize++;
        }
      } else {
        VLOG(1) << "Found an open constraint at idx = " << i;
        num_open_residuals++;
      }
    }
    VLOG(1) << "Number of residuals ready to optimize = " << num_ready_to_optimize
        << ", num open = " << num_open_residuals;
    
    // Initiate gravity
    gravity_state_.SetErrorZero();
    
    // Initiate cam-to-imu pose
    cam_to_imu_pose_.SetErrorZero();
    
    // Starting pose of IMU wrt tag map
    
    
    // Create IMU states using wall time and integrate IMU readings between them
    
    
    
    // Create TagView constraints connecting IMU states, tag poses and tag map pose in IMU frame
    
    // Save starting states for plotting
    
    // Build problem, solve it
    
    // Report and save results
    
    return true;
  }
  
  bool SaveStatesToFile(const anantak::CircularQueue<anantak::ImuState>& imu_states,
      const std::string& save_filename, const std::string& predicate) {
    // Initial IMU states. Create a matrix, copy all imu states and save
    Eigen::Matrix<double,16,Eigen::Dynamic> states_mat;
    int32_t num_states = imu_states.n_msgs();
    states_mat.resize(16,num_states);
    for (int i=0; i<num_states; i++) {
      Eigen::Map<const Eigen::Matrix<double,16,1>> state_map(imu_states.at(i).state_);
      states_mat.col(i) = state_map;
    }
    anantak::WriteMatrixToCSVFile(save_filename+"."+predicate+".imu_states", states_mat.transpose());
  }
  
  bool SaveStatesToFile(const std::string& save_filename, const std::string& predicate) {
    
    // Initial IMU states. Create a matrix, copy all imu states and save
    Eigen::Matrix<double,16,Eigen::Dynamic> states_mat;
    int32_t num_states = imu_states_->n_msgs();
    states_mat.resize(16,num_states);
    for (int i=0; i<num_states; i++) {
      Eigen::Map<Eigen::Matrix<double,16,1>> state_map(imu_states_->at(i).state_);
      states_mat.col(i) = state_map;
    }
    anantak::WriteMatrixToCSVFile(save_filename+"."+predicate+".imu_states_raw", states_mat.transpose());
    
    // Initial cam poses
    Eigen::Matrix<double,7,Eigen::Dynamic> cam_poses;
    num_states = cam_poses_->n_msgs();
    cam_poses.resize(7,num_states);
    for (int i=0; i<num_states; i++) {
      Eigen::Map<Eigen::Matrix<double,7,1>> cam_poses_map(cam_poses_->at(i).state_);
      cam_poses.col(i) = cam_poses_map;
    }
    anantak::WriteMatrixToCSVFile(save_filename+"."+predicate+".cam_poses", cam_poses.transpose());
    
    // IMU states transformed in Tag map frame.
    // IMU states are IqW, WpI, WvI, bg, ba
    //  IqT0 = IqW * WqT0
    //  T0pI = T0rW * (WpI - WpT0) = T0rW*WpI + T0pW
    //  T0vI = T0rW * WvI
    //  bg and ba do not change
    // Imu to cam pose can now be calculated
    //  CqI = CqT0 * T0qI
    //  IpC = IrT0 * (T0pC - T0pI) 
    Eigen::Quaterniond WqT0 = tag_map_to_imu_world_pose_.Quaternion();
    Eigen::Matrix3d T0rW(WqT0.conjugate());
    Eigen::Vector3d T0pW = tag_map_to_imu_world_pose_.Position();
    
    Eigen::Matrix<double,7,Eigen::Dynamic> cam_to_imu_poses;
    
    num_states = std::min(imu_states_->n_msgs(), cam_poses_->n_msgs());
    states_mat.resize(16,num_states);
    cam_to_imu_poses.resize(7,num_states);
    
    for (int i=0; i<num_states; i++) {
      // IMU states
      Eigen::Quaterniond IqT0 = imu_states_->at(i).Quaternion() * WqT0;
      Eigen::Matrix3d IrT0(IqT0);
      Eigen::Vector3d T0pI = T0rW*imu_states_->at(i).Position() + T0pW;
      states_mat.col(i).block<4,1>(0,0) = IqT0.coeffs();
      states_mat.col(i).block<3,1>(4,0) = T0pI;
      states_mat.col(i).block<3,1>(7,0) = T0rW*imu_states_->at(i).GvI_;
      states_mat.col(i).block<3,1>(10,0) = imu_states_->at(i).bg_;
      states_mat.col(i).block<3,1>(13,0) = imu_states_->at(i).ba_;
      
      // Cam to imu poses
      Eigen::Quaterniond CqI = cam_poses_->at(i).Quaternion() * IqT0.conjugate();
      Eigen::Vector3d IpC = IrT0*(cam_poses_->at(i).Position() - T0pI);
      Eigen::AngleAxisd CaI(CqI);
      cam_to_imu_poses.col(i).block<3,1>(0,0) = CaI.axis();
      cam_to_imu_poses.col(i)[3] = CaI.angle()*DegreesPerRadian;
      cam_to_imu_poses.col(i).block<3,1>(4,0) = IpC;
    }
    anantak::WriteMatrixToCSVFile(save_filename+"."+predicate+".imu_states", states_mat.transpose());
    anantak::WriteMatrixToCSVFile(save_filename+"."+predicate+".camimu_poses", cam_to_imu_poses.transpose());
    
    return true;
  }
  
  bool SaveResidualsToFile(const std::string& save_filename, const std::string& predicate) {
    // Imu Residuals
    Eigen::Matrix<double,16,Eigen::Dynamic> residuals_mat;
    int32_t num_residuals = imu_residuals_->n_msgs();
    residuals_mat.resize(16,num_residuals);
    for (int i=0; i<num_residuals; i++) {
      Eigen::Map<Eigen::Matrix<double,16,1>> resid_map(imu_residuals_->at(i).R_.data());
      residuals_mat.col(i) = resid_map;
    }
    anantak::WriteMatrixToCSVFile(save_filename+"."+predicate+".imu_residuals", residuals_mat.transpose());
  }
  
  bool SaveTagMapToFile(const std::string& save_filename, const std::string& predicate) {
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
  }
  
  bool CheckVsConvexIntegrals(int32_t starting_state_index) {
    // Here we check if integrals from convex method match the integrals done here
    int32_t beg_check_idx = 100;
    int32_t end_check_idx = 120;
    for (int i=beg_check_idx; i<end_check_idx; i++) {
      const anantak::ImuReadingsIntegralType& cvx_integral =
          imu_init_tag_camera_->collected_readings_->at(i+starting_state_index).imu_integral;
      const anantak::ImuEstimatesIntegralType& vio_integral =
          imu_residuals_->at(i).integral_;
      const Eigen::Vector3d& vio_accel_bias =
          imu_residuals_->at(i).integral_.ba0;
      // cvx_integral and vio_integral should match in values for dt, P, V, y, s
      double diff_dt = cvx_integral.dt - vio_integral.dt;
      if (std::abs(diff_dt)<Epsilon) diff_dt=0.;
      VLOG(1) << "   dt: cvx, vio = " << diff_dt << ", " << cvx_integral.dt << " " << vio_integral.dt;
      Eigen::Matrix3d diff_V = cvx_integral.V - vio_integral.V;
      if (diff_V.isZero(Epsilon)) diff_V = Eigen::Matrix3d::Zero();
      VLOG(1) << "    V: cvx, vio = " << Eigen::Map<Eigen::Matrix<double,1,9>>(diff_V.data());
      Eigen::Matrix3d diff_P = cvx_integral.P - vio_integral.P;
      if (diff_P.isZero(Epsilon)) diff_P = Eigen::Matrix3d::Zero();
      VLOG(1) << "    P: cvx, vio = " << Eigen::Map<Eigen::Matrix<double,1,9>>(diff_P.data());
      VLOG(1) << "     accel bias = " << vio_accel_bias.transpose();
      Eigen::Vector3d diff_s = (cvx_integral.s - cvx_integral.V*vio_accel_bias) - vio_integral.s;
      if (diff_s.isZero(Epsilon)) diff_s = Eigen::Vector3d::Zero();
      VLOG(1) << "    s: cvx, vio = " << diff_s.transpose() << ", " << vio_integral.s.transpose();
      Eigen::Vector3d diff_y = (cvx_integral.y - cvx_integral.P*vio_accel_bias) - vio_integral.y;
      if (diff_y.isZero(Epsilon)) diff_y = Eigen::Vector3d::Zero();
      VLOG(1) << "    y: cvx, vio = " << diff_y.transpose() << ", " << vio_integral.y.transpose();
    }
    return true;
  }
  
  bool CompareIntegrations() {
    VLOG(1) << "  Comparing convex integrals versus full-VIO integrals";
    // Create some readings or use existing readings, integrate and compare integrals
    int32_t beg_idx = 0;
    int32_t end_idx = std::min(size_t(20),imu_init_tag_camera_->curr_imu_readings_.size()-1);
    
    ImuIntegrationOptions iop;
    
    ImuReadingsIntegralType iri;
    
    IntegrateImuKinematics(
      imu_init_tag_camera_->curr_imu_readings_,
      beg_idx, end_idx,
      iop.accel_factor,
      &iri
    );
    
    ImuEstimatesIntegralType iei;
    iei.SetZero();
    // Set starting state
    iei.ts0 = imu_init_tag_camera_->curr_imu_readings_[beg_idx].timestamp;
    iei.r0 = imu_init_tag_camera_->curr_imu_readings_[beg_idx].quaternion; // WqI
    // Set ending state
    iei.ts1 = iei.ts0; iei.r1 = iei.r0;
    
    for (int i=beg_idx; i<end_idx; i++) {
      IntegrateImuKinematics(
          imu_init_tag_camera_->curr_imu_readings_[i],
          imu_init_tag_camera_->curr_imu_readings_[i+1],
          iop,
          &iei);      
    }
    
    double diff_dt = iri.dt - iei.dt; if (std::abs(diff_dt)<Epsilon) diff_dt=0.;
    VLOG(1) << "   dt: cvx, vio = " << diff_dt << ", " << iri.dt << " " << iei.dt;
    Eigen::Matrix3d diff_V = iri.V - iei.V;
    if (diff_V.isZero(Epsilon)) diff_V = Eigen::Matrix3d::Zero();
    VLOG(1) << "    V: cvx, vio = " << Eigen::Map<Eigen::Matrix<double,1,9>>(diff_V.data());
    Eigen::Matrix3d diff_P = iri.P - iei.P;
    if (diff_P.isZero(Epsilon)) diff_P = Eigen::Matrix3d::Zero();
    VLOG(1) << "    P: cvx, vio = " << Eigen::Map<Eigen::Matrix<double,1,9>>(diff_P.data());
    VLOG(1) << "     accel bias = " << iei.ba0.transpose();
    Eigen::Vector3d diff_s = (iri.s - iri.V*iei.ba0) - iei.s;
    if (diff_s.isZero(Epsilon)) diff_s = Eigen::Vector3d::Zero();
    VLOG(1) << "    s: cvx, vio = " << diff_s.transpose() << ", " << iei.s.transpose();
    Eigen::Vector3d diff_y = (iri.y - iri.P*iei.ba0) - iei.y;
    if (diff_y.isZero(Epsilon)) diff_y = Eigen::Vector3d::Zero();
    VLOG(1) << "    y: cvx, vio = " << diff_y.transpose() << ", " << iei.y.transpose();
    
    bool test_passed = (diff_dt==0. && diff_V.isZero() && diff_P.isZero() && diff_s.isZero() && diff_y.isZero());
    if (test_passed) VLOG(1) << "  Comparison passed"; else VLOG(1) << "  Comparison FAILED";
    
    return test_passed;
  }
  
  
  /* Process IMU and TagView readings
   * Check if initiated. Initiate or throw an error
   * Start current iteration:
   *  Setup iteration variables using data
   *  Create new IMU states
   *  Create new tag states if new tags were seen
   *  Estimate new states (past states in the queue should have estimates already)
   *  Get all Estimated states ready for optimization
   *  Create a problem
   *    Add priors
   *    Add constraints from imu states
   *    Add constraints from tag views
   *  Solve problem
   *  Re-calculate states from error-states
   *  Marginalize older states, update priors
   */
  // In every iteration, get the msgs from IMU and from reference camera. Collect motions.
  bool ProcessImuAndAprilTagMessages(const std::vector<anantak::SensorMsg>& imu_msgs,
      const std::vector<anantak::SensorMsg>& tag_msgs) {
    
    if (state_==kInitiateIMU) {
      // Send data to IMU init
      if (!imu_init_tag_camera_->IsSleeping()) {
        imu_init_tag_camera_->ProcessImuAndAprilTagMessages(imu_msgs, tag_msgs);
        CollectDataForInitiation(imu_msgs, tag_msgs);
      }
      // Initiate VIO
      if (imu_init_tag_camera_->IsSleeping()) {
        LOG(INFO) << "Initiating VIO";
        state_ = kInitiateVIO;
        if (Initiate()) {
          state_ = kRunningVIO;
        } else {
          LOG(WARNING) << "Could not initiate VIO, can not continue.";
          return false;
        }
      }
    }
    // Run single cam VIO
    else if (state_==kRunningVIO) {
      VLOG(1) << "Running single cam VIO";
    } 
    // Something is wrong
    else {
      LOG(ERROR) << "Multi TagCamera VIO is in an unknown state";
      return false;
    }
    
    return true;
  }  // ProcessImuAndAprilTagMessages


  bool SaveDataToFile(const std::string& save_filename) {
    imu_init_tag_camera_->SaveDataToFile(save_filename+".ImuInitCam");
    return true;
  }
  
  /* Iteration variables
   *  Sliding window start time ( = marginalization end time )
   *  Sliding window end time ( = iteration end time )
   *  Iteration start time ( = last iteration end time )
   *  Marginalization start time ( = optimization is starting here )
   */
  
  
}; 


int main(int argc, char** argv) {  
  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);
  // Parse the commandline flags 
  google::ParseCommandLineFlags(&argc, &argv, true);
  // Verify the version of protobuf library 
  GOOGLE_PROTOBUF_VERIFY_VERSION;
  // Setting the glog output to stderr by default
  FLAGS_logtostderr = 1;

  // Filenames
  std::string project_root_dir = anantak::GetProjectSourceDirectory() + "/";
  std::string plots_dir = project_root_dir + "src/Models/Plots/";

  const int32_t num_cameras = 4;
  std::vector<std::string> camera_mono_calib_msgs_filenames {
    "src/test/cam1_mono_calib_no_distort.pb.data",
    "src/test/cam2_mono_calib_no_distort.pb.data",
    "src/test/cam3_mono_calib_no_distort.pb.data",
    "src/test/cam4_mono_calib_no_distort.pb.data",
  };
  
  // Open calibration files - operate in historical mode
  FileMessagesKeeper calibrations_keeper(camera_mono_calib_msgs_filenames, false);
  if (!calibrations_keeper.LoadAllMsgsFromFiles()) {
    LOG(ERROR) << "Some error in loading calibrations.";
  }
  std::vector<std::unique_ptr<std::vector<anantak::SensorMsg>>> calib_msgs;
  calibrations_keeper.AllocateMemoryForNewMessages(1, &calib_msgs);
  calibrations_keeper.FetchLastMessages(&calib_msgs);

  // April tags specifications - assuming these are known already from the infrastructure
  double april_tag_size = 0.4780; // meters
  double april_tag_size_sigma = 0.010/3.0; // meters
  double sigma_im = 0.5; // pixels

  // Tag Cameras options
  AprilTagCameraOptions tag_cam_options;
  tag_cam_options.april_tag_size = april_tag_size;
  tag_cam_options.april_tag_size_sigma = april_tag_size_sigma;
  tag_cam_options.sigma_im = sigma_im;

  // Tag Cameras
  std::vector<std::unique_ptr<AprilTagCamera>> tag_cameras;
  tag_cameras.resize(num_cameras);
  for (int i_cam=0; i_cam<num_cameras; i_cam++) {
    tag_cam_options.camera_num = i_cam;
    
    std::unique_ptr<AprilTagCamera> ptr(new AprilTagCamera(tag_cam_options, calib_msgs[i_cam]->front()));
    tag_cameras[i_cam] = std::move(ptr);
  }
  
  // Tag IMU-init options
  AprilTagImuInitOptions imu_init_tag_cam_options;
  
  // VIO 11 (1 imu and 1 tag camera) init options
  TagVIO11::Options imu_tag_vio11_options;
  imu_tag_vio11_options.SetSaveFilename(plots_dir+"VIO11");
  
  // VIO with one IMU and one tag camera
  std::unique_ptr<TagVIO11> imu_tag_vio11;
  {
    // Set any tag_cam_options
    int32_t i_cam = 0;
    tag_cam_options.camera_num = i_cam;    
    // Set any imu_init_tag_cam_options
    // Set any imu_tag_vio_options
    std::unique_ptr<TagVIO11> tag_vio11_ptr(new
        TagVIO11(tag_cam_options, calib_msgs[i_cam]->front(), imu_init_tag_cam_options, imu_tag_vio11_options));
    imu_tag_vio11 = std::move(tag_vio11_ptr);
  }
  
  /* Load data into a file data keeper. This gives the facility to get mesages incrementaly */
  std::vector<std::string> msgs_filenames {
    "src/test/imu1_data.pb.data",
    "src/test/cam1_apriltags.pb.data",
    "src/test/cam2_apriltags.pb.data",
    "src/test/cam3_apriltags.pb.data",
    "src/test/cam4_apriltags.pb.data",
  };
  std::vector<int32_t> cam_datasource_map {1,2,3,4}; // index=cam_num_, val=datasource_num_
  std::vector<int32_t> imu_datasource_map {0,1}; // firstval=imu_ds_num, second_val=ref_cam_ds_num
  
  // Open msg files
  FileMessagesKeeper file_msgs_keeper(msgs_filenames, false);
  if (!file_msgs_keeper.LoadAllMsgsFromFiles()) {
    LOG(ERROR) << "Could not load data, exiting.";
    return -1;
  }
  
  int64_t iteration_interval = 500000; // microsec
  
  // Setup memory to get new messages
  std::vector<std::unique_ptr<std::vector<anantak::SensorMsg>>> new_msgs;
  file_msgs_keeper.AllocateMemoryForNewMessages(500, &new_msgs);
  
  /* Keep adding new data to a set of models - Create a tag model, an imu model and an Ackerman
   * model. Keep adding data to the models as time progresses. Models work together.
   *
   * Starting calibration of the IMU is done wrt a reference camera. IMU model will track
   * movement at the beginning. When enough movement is detected, use the convex formulation
   * to solve for the gravity and accel biases along with the reference camera. Then initiate
   * full vio model (using tags).
   *
   * When a camera starts, it continues to create its tag map(s) and marginalize older camera
   * poses. After every iteration, each camera will detect its map overlap with a reference camera.
   * When an overlap is detected, this camera is added to the VIO. 
   *
   * So begin
   */
  
  for (int i_iter=0; i_iter<150; i_iter++) {
    file_msgs_keeper.FetchNewMessages(iteration_interval, &new_msgs);
    // report the number of messages recieved
    std::cout << "iteration " << i_iter << ": Messages ";
    for (int i_ds=0; i_ds<msgs_filenames.size(); i_ds++) std::cout << new_msgs[i_ds]->size() << " ";
    std::cout << "\n";
    
    // Send data to VIO (that contains the IMU initiator)
    imu_tag_vio11->ProcessImuAndAprilTagMessages(
        *new_msgs[imu_datasource_map[0]], *new_msgs[imu_datasource_map[1]]);
    
    // Send data to each tag camera init model
    /*for (int i_cam=0; i_cam<num_cameras; i_cam++) {
      tag_cameras[i_cam]->ProcessAprilTagMessages(*new_msgs[cam_datasource_map[i_cam]]);
    }*/
    
    // Check if VIO with IMU and its reference camera can be initiated
    
    
    // Check if tag camera init models can be combined
    
  }

  /*for (int i_cam=0; i_cam<num_cameras; i_cam++) {
    tag_cameras[i_cam]->SavePosesToFile(plots_dir+"Cam"+std::to_string(i_cam)+".TagsOnly");
  }*/
  
  imu_tag_vio11->SaveDataToFile(plots_dir+"VIO11");
}


int main3(int argc, char** argv) {  
  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);
  // Parse the commandline flags 
  google::ParseCommandLineFlags(&argc, &argv, true);
  // Verify the version of protobuf library 
  GOOGLE_PROTOBUF_VERIFY_VERSION;
  // Setting the glog output to stderr by default
  FLAGS_logtostderr = 1;

  /* Running VIO for multiple cameras. This will be valuable.
   * We start with no prior information, make initial guesses, then refine. */

  // Filenames
  std::string project_root_dir = anantak::GetProjectSourceDirectory() + "/";
  std::string plots_dir = project_root_dir + "src/Models/Plots/";

  std::string imu1_msgs_filename = "src/test/imu1_data.pb.data";

  int32_t num_cameras = 4;
  std::vector<std::string> camera_mono_calib_msgs_filenames {
    "src/test/cam1_mono_calib_no_distort.pb.data",
    "src/test/cam2_mono_calib_no_distort.pb.data",
    "src/test/cam3_mono_calib_no_distort.pb.data",
    "src/test/cam4_mono_calib_no_distort.pb.data",
  };
  std::vector<std::string> camera_features_msgs_filenames {
    "src/test/cam1_sparse_points.pb.data",
    "src/test/cam2_sparse_points.pb.data",
    "src/test/cam3_sparse_points.pb.data",
    "src/test/cam4_sparse_points.pb.data",
  };
  std::vector<std::string> camera_apriltag_msgs_filenames {
    "src/test/cam1_apriltags.pb.data",
    "src/test/cam2_apriltags.pb.data",
    "src/test/cam3_apriltags.pb.data",
    "src/test/cam4_apriltags.pb.data",
  };
  
  std::string mi_msgs_filename = "src/test/machine_interface.pb.data";
  
  /* For each camera we assume that there are three sources of data:
   * Calibration file, Sparse points tracker and an AprilTag detector.
   * Sparse points are used for initial guess of the rotation between camera and imu.
   * AprilTag sightings are used to make an initial guess of IMU bias and gravity vector. */
  
  // Load camera data
  VLOG(1) << "Loading camera data";
  std::vector<std::unique_ptr<CameraMessages>> cam_msgs;
  for (int i_cam=0; i_cam<num_cameras; i_cam++) {
    std::unique_ptr<CameraMessages> ptr(new CameraMessages(i_cam, camera_mono_calib_msgs_filenames[i_cam],
        camera_features_msgs_filenames[i_cam], camera_apriltag_msgs_filenames[i_cam]));
    cam_msgs.push_back(std::move(ptr));
    cam_msgs[i_cam]->ExtractTimestamps();
  }
  
  // Load commands data
  MachineCommands machine_cmnds(mi_msgs_filename);
  
  // Load IMU data
  ImuMessages imu1_msgs(imu1_msgs_filename);
  
  /* Make a starting guess of the parameters of camera and imu sensor models. Only prior knowledge
   * is the number of cameras, their intrinsic calibrations and that there is one main imu.
   * Key assumptions in making starting guesses are:
   *   AprilTags are installed in the calibration environment with known tag size.
   *   Initial machine motion is planar, not necessarily horizontal plane.
   * Cameras are not assumed to be synchronized. */
  
  // Interpolate IMU readings for cameras
  VLOG(1) << "Interpolating IMU readings for camera timestamps";
  for (int i_cam=0; i_cam<num_cameras; i_cam++) {
    cam_msgs[i_cam]->InterpolateImuReadings(imu1_msgs.imu_readings_);
  }
  
  // Specifications to collect camera rotations
  int64_t initiation_interval = 10*1000000;  // These many musec of data will be used // TODO: Read from config
  double  minimum_rotation_threshold = 5.0*RadiansPerDegree; // small angle            // TODO: Read from config
  int32_t minimum_number_of_rotations = 50; // in theory, a minimum of 2 are needed.  // TODO: read from config
  
  VLOG(1) << "Collecting Camera rotations from interpolated IMU data using AprilTags";
  for (int i_cam=0; i_cam<num_cameras; i_cam++) {
    cam_msgs[i_cam]->CollectImuRotationsUsingAprilTags(initiation_interval,
        minimum_rotation_threshold, minimum_number_of_rotations);
  }

  // April tags specification - assuming these are known already from the infrastructure
  double april_tag_size = 0.4780; // meters
  double april_tag_size_sigma = 0.010/3.0; // meters
  double sigma_im = 0.5; // pixels

  VLOG(1) << "Collecting tag sightings for each camera for this period";
  for (int i_cam=0; i_cam<num_cameras; i_cam++) {
    cam_msgs[i_cam]->SetAprilTagsSize(april_tag_size, april_tag_size_sigma, sigma_im);
    cam_msgs[i_cam]->CollectAprilTagSightings(cam_msgs[i_cam]->CrI_estimation_interval);
  }
  
  VLOG(1) << "Solving for camera and tag poses using only tag sightings";
  for (int i_cam=0; i_cam<num_cameras; i_cam++) {
    std::string save_filename = plots_dir+"Cam"+std::to_string(i_cam)+".TagsOnly";
    cam_msgs[i_cam]->EstimateMotionUsingTagSightingsOnly(save_filename);
  }
  
  // Calculate maximum timestamp of CrI estimation interval for all cameras
  int64_t max_CrI_estimation_end_ts = 0;
  for (int i_cam=0; i_cam<num_cameras; i_cam++) {
    max_CrI_estimation_end_ts = std::max(max_CrI_estimation_end_ts,
        cam_msgs[i_cam]->CrI_estimation_end_ts);
  }
  max_CrI_estimation_end_ts += 100000; // add a 100 mili second padding

  double velocity_command_center = 255.;
  double steering_command_center = 1450.;
  double rest_velo_cmnd_threshold = 5.;
  
  machine_cmnds.SetModelParameters(velocity_command_center, steering_command_center);
  machine_cmnds.FindInitialPeriodOfRest(rest_velo_cmnd_threshold);
  
  double g_mag = 9.8;  // m/s^2
  double g_mag_lb = 9.7; // m/s^2 - lower bound of gravity mag
  double g_mag_ub = 9.9; // m/s^2 - upper bound of gravity mag
  double g_range_stdev = 4.0; // range of stdev's between grav ub and lb
  double g_sigma = (g_mag_ub*g_mag_ub - g_mag_lb*g_mag_lb)/g_range_stdev;

  // IMU Datasheet specifications
  double imu1_accel_g_per_lsb = 8192.;
  double imu1_accel_sigma = 400.0*1e-6*g_mag; // m/s^s/sqrt(Hz) from the datasheet
  double imu1_gyro_sigma = 30.*RadiansPerDegree; // rad/s/sqrt(Hz) from the datasheet
  
  imu1_msgs.SetModelParameters(imu1_accel_g_per_lsb, g_mag, g_sigma, imu1_accel_sigma,
      imu1_gyro_sigma);
  
  imu1_msgs.EstimateGravityAndAccelBiasUsingRestPeriod(machine_cmnds.initial_rest_begin_ts_,
      machine_cmnds.initial_rest_end_ts_);
  
  imu1_msgs.IntegrateImuReadingsUsingRestPeriodEstimates(machine_cmnds.initial_rest_begin_ts_,
      max_CrI_estimation_end_ts, plots_dir+"ImuRestIntegral");
  
  /*VLOG(1) << "Interpolating IMU readings for camera timestamps";
  for (int i_cam=0; i_cam<num_cameras; i_cam++) {
    std::string save_filename = plots_dir+"Cam"+std::to_string(i_cam)+".TagsOnly";
    cam_msgs[i_cam]->InterpolateUnbiasedImuReadings(imu1_msgs.imu_readings_minus_gravaccelbias_);
    cam_msgs[i_cam]->IntegrateUnbiasedImuReadings(imu1_msgs.imu_readings_minus_gravaccelbias_,
        save_filename);
  }
  
  double linear_motion_threshold = 0.10; // meters - machine motion is collected when it travels at least this much.
  double straight_angle_threshold = 5.0*RadiansPerDegree; // radians - less than this much is considered straight motion.  
  */
  
  /* Make a guess for the rotation between IMU and camera. This uses the sparse points tracked
   * for each camera. For each camera image calculate the imu rotation. Collect N rotations from
   * imu data where magnitude of rotations is greater than a threshold. Integrate corresponding
   * rotations from sparse points readings. Then use hand-eye calibration for an initial estimate
   * of camera-to-imu rotation. */
  /*

  // Collect IMU readings for initial guess of Camera-IMU rotation
  for (int i_cam=0; i_cam<num_cameras; i_cam++) {
    cam_msgs[i_cam]->CollectImuRotationsUsingSparseFeatures(initiation_interval,
        minimum_rotation_threshold, minimum_number_of_rotations);
  }
  
  // Calculate starting estimate of Camera-to-IMU rotation
  VLOG(1) << "Interpolating IMU readings for camera timestamps and running hand-eye calibration";
  for (int i_cam=0; i_cam<num_cameras; i_cam++) {
    cam_msgs[i_cam]->CollectCameraRotationsUsingSparseFeatures();
    cam_msgs[i_cam]->EstimateCameraToImuRotationUsingSparseFeatures();
    cam_msgs[i_cam]->CalculateAprilTagWrCsUsingImuReadings();
  }*/
  
  /* Get ready for map building and camera-to-camera transpform calculation
   * Set reference map to contain all tags seen in reference camera. Mark reference camera as
   * 'overlapped'. Check if any of the rest of the cameras overlap with the reference map. If so,
   * add this camera's tags to the reference map and mark the camera as overlapped. Check if any
   * camera is still not overlapped. If so move forward and continue. Do this till a limit is
   * reached or all cameras are overlapped. When all cameras are overlapped, mark this timestamp
   * and get ready to run an optimization with all cameras and all tags in the reference map */
  
  /*
  // Tag map building
  anantak::AprilTagMap apriltags_map;
  int32_t reference_cam = 0;
  int64_t move_forward_interval = 5 * 1000000;  // interval to move forward
  int64_t sightings_collection_time_limit = 5*60* 1000000; // Check for overlaps for this interval
  int32_t min_num_tag_for_overlap = 5; // At least so many tags should overlap to mark overlap  
  
  // Collect tags in the environment and all tag sightings
  VLOG(1) << "Collecting apriltag sightings for all cameras and checking for overlaps";
  {
    std::vector<bool> cameras_overlap;
    for (int i=0; i<num_cameras; i++) cameras_overlap.push_back(false);
    cameras_overlap[reference_cam] = true;
    bool all_cameras_overlap = false;
    int64_t total_interval_collected = 0;
    VLOG(1) << "Cam " << reference_cam << " is the reference";
    while (!all_cameras_overlap && total_interval_collected<sightings_collection_time_limit ) {
      // Move forward
      for (int i_cam=0; i_cam<num_cameras; i_cam++) {
        cam_msgs[i_cam]->CollectAprilTagSightings(move_forward_interval);
      }
      total_interval_collected += move_forward_interval;
      // Copy new tags in ref cam to environment map
      apriltags_map.AddNewTags(cam_msgs[reference_cam]->apriltags_map_);
      // Check for overlaps
      for (int i_cam=0; i_cam<num_cameras; i_cam++) {
        if (!cameras_overlap[i_cam]) {
          if (apriltags_map.MapOverlaps(cam_msgs[i_cam]->apriltags_map_, min_num_tag_for_overlap)) {
            cameras_overlap[i_cam] = true;
            apriltags_map.AddNewTags(cam_msgs[i_cam]->apriltags_map_);
            VLOG(1) << "Cam " << i_cam << " overlaps after = " << total_interval_collected*1e-6 << "s";
          }
        }
      } // for all cams
      // Check if all cameras overlap
      all_cameras_overlap = true;
      for (int i_cam=0; i_cam<num_cameras; i_cam++) {all_cameras_overlap &= cameras_overlap[i_cam];}
    } // while not all cameras overlap or sightings collection time limit is exceeded
    VLOG(1) << "Environment map after all cameras overlap has " << apriltags_map.num_tags() << " tags";
    for (int i_cam=0; i_cam<num_cameras; i_cam++) {
      VLOG(1) << "Cam " << i_cam << ": num images = " << cam_msgs[i_cam]->ending_tag_image_idx_
          << " num sightings = " << cam_msgs[i_cam]->apriltag_sightings_.size();
    }
  }*/
  
  /* Estimate the tag rotations in the world frame using the tag sightings. We collect all tag
   * sightings from all cameras, then use those to estimate tag rotations in the world frame.
   * Rotations are averaged using a quaternion averaging algorithm. */
  
  // Estimate rotation of each tag by averaging all sightings by all cameras
  //VLOG(1) << "Estimating tag rotations in IMU frame using all sightings";
  //EstimateTagRotationsFromSightings(cam_msgs, &apriltags_map);

  /* Estimate tag positions, reference camera positions and camera-to-ref-camera positions.
   * We will keep rotations constant - this helps in keeping the optimization problem convex */
  
  /*
  Eigen::Matrix<double,3,Eigen::Dynamic> ref_cam_poses;

  EstimateTagAndCameraPositionsWithFixedRotations(cam_msgs, reference_cam, &apriltags_map,
      &ref_cam_poses, plots_dir+"ref_cam_poses.csv");  
  
  // Estimate tag and camera positions for each camera individually
  VLOG(1) << "Estimating tag and camera positions for each camera";
  for (int i_cam=0; i_cam<num_cameras; i_cam++) {
    std::string save_filename = plots_dir + "Cam" + std::to_string(i_cam) + "_initial_WpC.txt";
    cam_msgs[i_cam]->EstimateTagAndCameraPositions(apriltags_map, sigma_im, save_filename);
  }
  */
  
  /* Estimate parameters of a kinematic motion model of the machine. X axis points forward, Y to
   * left Z points upwards. For initial guess of parameters we use some design parameters and
   * estimated initial motion. Here we only get approximate parameters. Estimates will be refined
   * using a realtime model later. */
  
  
  // Collect linear motions and angular changes of the cameras
  /*VLOG(1) << "Collecting linear motions and angular changes of the cameras";
  for (int i_cam=0; i_cam<num_cameras; i_cam++) {
    cam_msgs[i_cam]->CollectMotions(machine_cmnds.cmnds_ts_, linear_motion_threshold);
    machine_cmnds.EstimateLambdaVelocity(cam_msgs[i_cam]->initial_motions, velocity_command_center,
        straight_angle_threshold);
  }*/
  
  
  
  // Estimate parameters of the machine kinematic model - assuming planar motion
  // This estimates lambda_v, lambda_d, CrM - Camera to machine rotation
  // Estimate of camera to machine rotation will be used to calculate the imu parameters
  
  /* Build a convex optimization problem to estimate tag + camera positions and imu parameters.
   * Imu parameters to be estimated are: gravity vector, accel biases and velocities
   * We want to use N cameras. To do this we estimate the state of the machine at machine-defined
   * timestamps, rather than at one-camera-defined or imu-defined timestamps.
   * We also use the fact that motion was planar. This should help in calculating accel biases. */
  
  
  
} // main

int main2(int argc, char** argv) {  
  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);
  // Parse the commandline flags 
  google::ParseCommandLineFlags(&argc, &argv, true);
  // Verify the version of protobuf library 
  GOOGLE_PROTOBUF_VERIFY_VERSION;
  // Setting the glog output to stderr by default
  FLAGS_logtostderr = 1;

  // Filenames
  std::string project_root_dir = anantak::GetProjectSourceDirectory() + "/";
  std::string mi_msgs_filename = "src/test/machine_interface.pb.data";
  std::string imu1_msgs_filename = "src/test/imu1_data.pb.data";
  std::string cam1_apriltag_msgs_filename = "src/test/cam1_apriltags.pb.data";
  std::string cam1_features_msgs_filename = "src/test/cam1_sparse_points.pb.data";
  std::string cam1_mono_calib_msgs_filename = "src/test/cam1_mono_calib_no_distort.pb.data";
  std::string plots_dir = project_root_dir + "src/Models/Plots/";
  std::string imu1_data_out_file = plots_dir + "imu1_data_out.csv";
  std::string cam1_data_out_file = plots_dir + "cam1_data_out.csv";

  std::vector<anantak::SensorMsg> imu1_msgs, cam1_apriltag_msgs, cam1_features_msgs,
    cam1_mono_calib_msgs;

  int64_t initiation_interval = 10*1000000;  /**< These many musec of data will be used */

  // Load all data from files
  if (!LoadMsgsFromFile(imu1_msgs_filename, &imu1_msgs)) {return -1;}
  if (!LoadMsgsFromFile(cam1_mono_calib_msgs_filename, &cam1_mono_calib_msgs)) {return -1;}
  if (!LoadMsgsFromFile(cam1_apriltag_msgs_filename, &cam1_apriltag_msgs)) {return -1;}
  if (!LoadMsgsFromFile(cam1_features_msgs_filename, &cam1_features_msgs)) {return -1;}
    
  // Check if the timestamps are increasing
  if (!IsIncreasing(imu1_msgs)) {LOG(ERROR) << "imu1_msgs is not monotonously increasing.";}
  if (!IsIncreasing(cam1_mono_calib_msgs)) {LOG(ERROR) << "cam1_mono_calib_msgs is not monotonously increasing.";}
  if (!IsIncreasing(cam1_apriltag_msgs)) {LOG(ERROR) << "cam1_apriltag_msgs is not monotonously increasing.";}
  if (!IsIncreasing(cam1_features_msgs)) {LOG(ERROR) << "cam1_features_msgs is not monotonously increasing.";}
  
  // States are initiated at IMU timestamps. Extract the timestamps.
  TimestampVecType imu1_timestamps, cam1_apriltag_timestamps, cam1_features_timestamps;
  ExtractTimestamps(imu1_msgs, &imu1_timestamps);
  ExtractTimestamps(cam1_apriltag_msgs, &cam1_apriltag_timestamps);  
  ExtractTimestamps(cam1_features_msgs, &cam1_features_timestamps);
  { // Save timestamps to disk for plotting
    WriteMatrixToCSVFile(plots_dir+"imu1_timestamps.csv", imu1_timestamps);
    WriteMatrixToCSVFile(plots_dir+"cam1_timestamps.csv", cam1_features_timestamps);
  }
 
  /* Get interpolated IMU quaternions at Cam1 timestamps. Assuming constant angular velocity */
  QuaternionVectorType imu1_quaternions, cam1_features_quaternions, cam1_apriltag_quaternions;
  StdVectorOfVector3dType imu1_accelerations;
  ExtractQuaternionsFromImuMessages(imu1_msgs, &imu1_quaternions);
  ExtractAccelerationsFromImuMessages(imu1_msgs, &imu1_accelerations);
  LinearInterpVectorType cam1_features_in_imu1, cam1_apriltag_in_imu1;
  InterpolateTimestamps(imu1_timestamps, cam1_features_timestamps, &cam1_features_in_imu1);
  InterpolateTimestamps(imu1_timestamps, cam1_apriltag_timestamps, &cam1_apriltag_in_imu1);
  InterpolateQuaternions(imu1_quaternions, cam1_features_in_imu1, &cam1_features_quaternions);
  InterpolateQuaternions(imu1_quaternions, cam1_apriltag_in_imu1, &cam1_apriltag_quaternions);
  { // Save rotations to disk for plotting
    YawPitchRollVectorType imu1_yprs, cam1_features_yprs, cam1_apriltag_yprs;
    QuaternionVectorToYawPitchRollVector(imu1_quaternions, &imu1_yprs);
    QuaternionVectorToYawPitchRollVector(cam1_features_quaternions, &cam1_features_yprs);
    QuaternionVectorToYawPitchRollVector(cam1_apriltag_quaternions, &cam1_apriltag_yprs);
    WriteMatrixToCSVFile(plots_dir+"imu1_ypr.csv", imu1_yprs);
    WriteMatrixToCSVFile(plots_dir+"cam1_features_ypr.csv", cam1_features_yprs);
    WriteMatrixToCSVFile(plots_dir+"cam1_apriltag_ypr.csv", cam1_apriltag_yprs);
  }

  /* Solving for rotation between IMU1 and Cam1 */

  /* Collect imu rotations where angle of rotation is in access of a threshold */
  double minimum_rotation_threshold = 5.0*RadiansPerDegree; // small angle reduces noise in vision data // TODO: Read from config
  int32_t minimum_number_of_rotations = 10; // in theory, a minimum of 2 are needed.  // TODO: read from config
  struct CollectedRotation {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int32_t index1; int32_t index2;
    QuaternionType quaternion;
    AngleAxisType aa;
    QuaternionType matching_quaternion;
    AngleAxisType matching_aa;
  };
  std::vector<CollectedRotation> collected_rotations;
  int64_t initial_estimation_end_timestamp;
  int64_t CqI_estimation_interval = initiation_interval;
  { // Collect rotations from IMU
    int64_t time_elapsed = 0;
    int32_t number_of_rotations = 0;
    int32_t last_index = 0;
    int32_t curr_index = 1;
    while ((time_elapsed<initiation_interval || number_of_rotations<minimum_number_of_rotations) &&
        curr_index<cam1_features_quaternions.size()) {
      // Check the angles between curr_index and last_index. If angle>threshold, store the indexes.
      // IMU Reading is WqI. We need I0qIi = WqI0^-1 (x) WqIi.
      QuaternionType dq = cam1_features_quaternions[last_index].conjugate() *
          cam1_features_quaternions[curr_index]; 
      AngleAxisType aa(dq.conjugate());
      if (std::abs(aa.angle())>=minimum_rotation_threshold) {
        CollectedRotation cr;
        cr.index1 = last_index;
        cr.index2 = curr_index;
        cr.quaternion = dq;
        cr.aa = aa;
        collected_rotations.push_back(cr);
        number_of_rotations++;
        last_index = curr_index;
        curr_index++;
        VLOG(2) << "  " << "Indexes = " << cr.index1 << " " << cr.index2 << ", angle = " <<
            cr.aa.angle()*DegreesPerRadian << ", axis = " << cr.aa.axis().transpose();
      } else {
        curr_index++;
      }
      time_elapsed = cam1_features_timestamps[curr_index] - cam1_features_timestamps[0];
      initial_estimation_end_timestamp = cam1_features_timestamps[curr_index];
    }
    VLOG(1) << "Collected " << collected_rotations.size() << " rotations spanning " <<
        double(time_elapsed)/1e6 << " seconds with min " <<
        minimum_rotation_threshold*DegreesPerRadian << " degrees rotation";
    CqI_estimation_interval = time_elapsed;
  }
  
  /* Extract the vision-implied rotations using eigensolver from opengv library
   * Rotation is measured using the features points that are tracked frame to frame */
  double sparse_point_wt = 1.0;   // TODO: read from config
  const anantak::MonocularPinholeCalibrationNoDistortionMsg& calib_msg =
      cam1_mono_calib_msgs.back().mono_calib_no_distort_msg();
  // Calculate vision-implied rotations corresponding to imu-implied rotations
  for (int i_rotn=0; i_rotn<collected_rotations.size(); i_rotn++) {
    /* Integrate rotations from index1 to index2 seen by the camera */
    opengv::rotation_t integrated_rotation = opengv::rotation_t::Identity();
    for (int idx=collected_rotations[i_rotn].index1+1; idx<collected_rotations[i_rotn].index2+1;
        idx++) {
      if (!cam1_features_msgs[idx].has_mono_sparse_points_msg()) {
        LOG(ERROR) << "No sparse points message was found at idx " << idx;
        continue;
      }
      const anantak::MonocularSparsePointsMsg& sparse_points_msg =
          cam1_features_msgs[idx].mono_sparse_points_msg();
      VLOG(3) << "Number of sparse points in " << idx << " msg " << sparse_points_msg.u_curr_size();
      opengv::bearingVectors_t bearing_vecs0, bearing_vecs1;
      std::vector<double> wts;
      for (int i=0; i<sparse_points_msg.u_curr_size(); i++) {
        opengv::bearingVector_t bv0, bv1;
        bv0 << sparse_points_msg.u_prev(i) - calib_msg.cx(),
            sparse_points_msg.v_prev(i) - calib_msg.cy(), calib_msg.focal_length();
        bv1 << sparse_points_msg.u_curr(i) - calib_msg.cx(),
            sparse_points_msg.v_curr(i) - calib_msg.cy(), calib_msg.focal_length();
        bv0 /= bv0.norm();
        bv1 /= bv1.norm();
        bearing_vecs0.push_back(bv0);
        bearing_vecs1.push_back(bv1);
        wts.push_back(sparse_point_wt);
      }
      opengv::relative_pose::CentralRelativeWeightingAdapter adapter(
          bearing_vecs0, bearing_vecs1, wts);
      opengv::rotation_t vision_rotation = opengv::relative_pose::eigensolver(adapter);
      // C0rC2 = C0rC1 * C1rC2
      integrated_rotation *= vision_rotation;
    } // for idx
    QuaternionType dq_match(integrated_rotation); 
    AngleAxisType aa_match(dq_match);
    collected_rotations[i_rotn].matching_quaternion = dq_match;
    collected_rotations[i_rotn].matching_aa = aa_match;
    const CollectedRotation& cr = collected_rotations[i_rotn];
    VLOG(2) << "  " << "Indexes = " << cr.index1 << " " << cr.index2 << ", angle = " <<
        cr.matching_aa.angle()*DegreesPerRadian << ", axis = " << cr.matching_aa.axis().transpose();    
  } // for i_rotn
  
  /* Solve the AX=XB problem (Hand-Eye calibration) for rotations collected from imu and vision */
  /* We build an over-constrained system of equations using the matrices and solve using SVD */
  QuaternionType  CqI;   // Quaternion that will rotate a vector in I to C frame
  RotationMatType CrI;   // Matrix3d
  AngleAxisType   CaI;   // Angleaxis
  { // Assemble the block matrix to be decomposed (svd)
    int32_t num_blocks = collected_rotations.size();
    Eigen::Matrix<double, Eigen::Dynamic, 4> svd_mat;
    svd_mat.resize(4*num_blocks, 4);
    for (int i=0; i<num_blocks; i++) {
      Eigen::Matrix4d mat;
      mat = LeftMultiplicationMatrix(collected_rotations[i].matching_quaternion)  // A matrix
          - RightMultiplicationMatrix(collected_rotations[i].quaternion);         // B matrix
      svd_mat.block<4,4>(i*4, 0) = mat;
    }
    VLOG(3) << "SVD mat = \n" << svd_mat;
    Eigen::JacobiSVD<Eigen::Matrix<double, Eigen::Dynamic, 4>>
        svd(svd_mat, Eigen::ComputeThinU | Eigen::ComputeThinV);
    VLOG(3) << "Singular values are:\n" << svd.singularValues();
    //VLOG(2) << "Left singular vectors are the columns of U matrix:\n" << svd.matrixU();
    VLOG(3) << "Right singular vectors are the columns of V matrix:\n" << svd.matrixV();
    Eigen::Vector4d ciq_vec = svd.matrixV().col(3);
    //VLOG(1) << "CqI = " << ciq_vec.transpose() << " norm = " << ciq_vec.norm();
    QuaternionType ciq(ciq_vec[3], ciq_vec[0], ciq_vec[1], ciq_vec[2]);
    CqI = ciq;
    VLOG(1) << "CqI = " << CqI.coeffs().transpose();
    CrI = CqI.toRotationMatrix();
    AngleAxisType ciaa(CqI);
    CaI = ciaa;
    VLOG(1) << "CaI initial estimate  = " << ciaa.axis().transpose()
        << " angle = " << ciaa.angle()*DegreesPerRadian;
    
    // Covariance matrix of the errors in estimate
    /*svd_mat *= RightMultiplicationMatrix(ciq_vec);
    for (int i=0; i<num_blocks; i++) {
      Eigen::JacobiSVD<Eigen::Matrix<double, Eigen::Dynamic, 4>>
          svd2(svd_mat.block<4,4>(i*4,0), Eigen::ComputeThinU | Eigen::ComputeThinV);
      Eigen::Vector4d q_vec = svd2.matrixV().col(3);
      //VLOG(1) << "resid " << i << ": " << q_vec.transpose();
      QuaternionType q_q(q_vec[3], q_vec[0], q_vec[1], q_vec[2]);
      AngleAxisType q_a(q_q);
      VLOG(1) << "resid " << i << ": " << q_vec.transpose() << ", " << q_a.axis().transpose() << ", "
          << q_a.angle()*DegreesPerRadian;
    }
    Eigen::Matrix<double, Eigen::Dynamic, 1> resid_vec;
    resid_vec.resize(4*num_blocks, 1);
    Eigen::Matrix4d mat;
    resid_vec = svd_mat * ciq_vec;
    Eigen::Map<Eigen::Matrix<double,4,Eigen::Dynamic>> resid_mat(resid_vec.data(),4,num_blocks);
    Eigen::Matrix<double,4,4> resid_covar = 1.0/double(num_blocks-1)*(resid_mat * resid_mat.transpose());
    VLOG(1) << "Residuals = \n" << resid_mat.block<4,10>(0,0);
    VLOG(1) << "Covar matrix of CqI residuals = \n" << resid_covar;*/
  }
  if (false) { // Checking the CqI 
    double q1_tot_angle = 0.0;
    double q2_tot_angle = 0.0;
    for (int i=0; i<collected_rotations.size(); i++) {
      // C0qC1 * CqI * I0qI1^-1 * CqI^-1 = C0qC0 = Identity
      QuaternionType q1 = collected_rotations[i].matching_quaternion
          * CqI
          * collected_rotations[i].quaternion.conjugate()
          * CqI.conjugate();
      QuaternionType q2 = collected_rotations[i].matching_quaternion
          * CqI.conjugate()
          * collected_rotations[i].quaternion.conjugate()
          * CqI;
      
      // C0rC1 * CrI * I1rI0 * IrC
      AngleAxisType a1(q1);
      AngleAxisType a2(q2);
      q1_tot_angle += a1.angle()<Pi ? a1.angle() : Pi_2-a1.angle();
      q2_tot_angle += a2.angle()<Pi ? a2.angle() : Pi_2-a2.angle();
      VLOG(2) << i << " " << a1.angle()*DegreesPerRadian << "   " << a2.angle()*DegreesPerRadian <<
          "  ( " << collected_rotations[i].matching_aa.angle()*DegreesPerRadian << " " <<
          collected_rotations[i].aa.angle()*DegreesPerRadian << " " << CaI.angle()*DegreesPerRadian
          << " ) ";
    }
    // We expect q2_tot_angle < q1_tot_angle if not raise warning
    if (q2_tot_angle >= q1_tot_angle) {
      LOG(INFO) << "CrI calculation checks alright";
    } else {
      // I can not explain why what we get here is actually IrC, not CrI. So we invert.
      LOG(INFO) << "CrI calculation actually gives IrC? Transposing. Need to explain this.";
      CqI = CqI.conjugate();
      CrI = CqI.toRotationMatrix();
      AngleAxisType ciaa(CqI);
      CaI = ciaa;
    }
  }
  
  /* Solve for AprilTag rotations in AprilTag messages */
  
  /* Estimate each tag sighting pose using P3P method (opengv) with four 2d-3d corner
   * correspondences to solve for R|t */
  double april_tag_size = 0.4780; // meters
  Eigen::Matrix3d cam1_K;
  cam1_K << calib_msg.focal_length(), 0.0, calib_msg.cx(),
            0.0, calib_msg.focal_length(), calib_msg.cy(),
            0.0, 0.0, 1.0;
  std::vector<AprilTagSighting, Eigen::aligned_allocator<AprilTagSighting>> apriltag_sightings;
  Eigen::Matrix<double, 4, 3> apriltag_3d_corners = AprilTagCorners3d(april_tag_size);
  { // Collect tag sightings
    int32_t i_msg = 0;
    while (cam1_apriltag_timestamps[i_msg]-cam1_apriltag_timestamps[0] <= CqI_estimation_interval) {
      // Check if AprilTag message exists
      if (!cam1_apriltag_msgs[i_msg].has_april_msg()) {
        LOG(ERROR) << "No AprilTag message was found at idx " << i_msg;
        continue;
      }
      // Check if any AprilTags were seen
      const anantak::AprilTagMessage& apriltag_msg = cam1_apriltag_msgs[i_msg].april_msg();
      VLOG(3) << "Number of AprilTags in msg " << i_msg << " = " << apriltag_msg.tag_id_size();
      if (apriltag_msg.tag_id_size()>0) {
        for (int i_tag=0; i_tag<apriltag_msg.tag_id_size(); i_tag++) {
          // Solve for camera pose wrt AprilTag
          opengv::bearingVectors_t bearing_vecs;
          opengv::points_t points_vec;
          { // Create observation vectors from corners
            opengv::bearingVector_t bv1, bv2, bv3, bv4;
            bv1 << apriltag_msg.u_1(i_tag) - calib_msg.cx(), apriltag_msg.v_1(i_tag) - calib_msg.cy(),
                calib_msg.focal_length();
            bv2 << apriltag_msg.u_2(i_tag) - calib_msg.cx(), apriltag_msg.v_2(i_tag) - calib_msg.cy(),
                calib_msg.focal_length();
            bv3 << apriltag_msg.u_3(i_tag) - calib_msg.cx(), apriltag_msg.v_3(i_tag) - calib_msg.cy(),
                calib_msg.focal_length();
            bv4 << apriltag_msg.u_4(i_tag) - calib_msg.cx(), apriltag_msg.v_4(i_tag) - calib_msg.cy(),
                calib_msg.focal_length();
            bv1 /= bv1.norm();
            bv2 /= bv2.norm();
            bv3 /= bv3.norm();
            bv4 /= bv4.norm();
            bearing_vecs.push_back(bv1);
            bearing_vecs.push_back(bv2);
            bearing_vecs.push_back(bv3);
            bearing_vecs.push_back(bv4);
            opengv::point_t pnt1 = apriltag_3d_corners.row(0);
            opengv::point_t pnt2 = apriltag_3d_corners.row(1);
            opengv::point_t pnt3 = apriltag_3d_corners.row(2);
            opengv::point_t pnt4 = apriltag_3d_corners.row(3);
            points_vec.push_back(pnt1);
            points_vec.push_back(pnt2);
            points_vec.push_back(pnt3);
            points_vec.push_back(pnt4);
          }
          opengv::absolute_pose::CentralAbsoluteAdapter adapter(bearing_vecs, points_vec);
          opengv::transformations_t pnp_transformations = opengv::absolute_pose::p3p_kneip(adapter);
          /*VLOG(1) << "tag = " << apriltag_msg.tag_id(i_tag)
              << " n_tfmtns = " << pnp_transformations.size();
          VLOG(3) << "corners = " << points_vec[0].transpose() << ", " << points_vec[1].transpose()
              << ", " << points_vec[2].transpose() << ", " << points_vec[3].transpose();*/
          Eigen::Matrix<double, 4, 2> corners_in_image;
          corners_in_image << 
              apriltag_msg.u_1(i_tag), apriltag_msg.v_1(i_tag),
              apriltag_msg.u_2(i_tag), apriltag_msg.v_2(i_tag),
              apriltag_msg.u_3(i_tag), apriltag_msg.v_3(i_tag),
              apriltag_msg.u_4(i_tag), apriltag_msg.v_4(i_tag);
          VLOG(3) << "corners = /n" << corners_in_image;
          Eigen::Vector4d dffs;
          Eigen::Matrix<double,3,16> Cpfs; // corner 3d coordinates in camera frame for 4 guesses
          for (int i=0; i<pnp_transformations.size(); i++) {
            //VLOG(1) << "\n" << pnp_transformations[i];
            Eigen::Matrix3d pnp_rotn = pnp_transformations[i].block<3,3>(0,0);
            Eigen::Vector3d pnp_tran = pnp_transformations[i].col(3);
            // Calculate reprojection error
            double total_dff = 0.0;
            for (int j=0; j<4; j++) {
              Eigen::Vector3d c = pnp_rotn.transpose() * (apriltag_3d_corners.row(j).transpose() - pnp_tran);
              //VLOG(1) << "c = " << c.transpose();
              Eigen::Vector3d Kc = cam1_K*c;
              if (i<4 && j<4) {
                // store Kc (=Cpf) in Cpfs matrix
                Cpfs.block<3,4>(0,4*i).col(j) = Kc;
              }
              //VLOG(1) << "Kc = " << Kc.transpose();
              Eigen::Vector2d Kcn; Kcn << Kc(0)/Kc(2), Kc(1)/Kc(2);
              Eigen::Vector2d dff = Kcn - corners_in_image.row(j).transpose();
              //VLOG(1) << "Kcn = " << Kcn.transpose() << " dff = " << dffs[j];
              total_dff += dff.squaredNorm();
            }
            dffs[i] = total_dff;
          } // for all transformations
          //VLOG(1) << dffs.transpose();
          Eigen::Vector4d::Index min_idx; dffs.minCoeff(&min_idx);
          //VLOG(1) << "Min transformation at " << min_idx;
          AprilTagSighting sighting;
          sighting.tag_id = apriltag_msg.tag_id(i_tag);
          sighting.image_coords = corners_in_image.transpose();
          sighting.reproj_error = dffs[min_idx];
          sighting.TrC = pnp_transformations[min_idx].block<3,3>(0,0);
          sighting.TpC = pnp_transformations[min_idx].col(3);
          sighting.Cpf = Cpfs.block<3,4>(0,4*min_idx);
          sighting.cam_K = cam1_K;
          // IMU readings are WqI.
          RotationMatType WrI(cam1_apriltag_quaternions[i_msg]);
          // Calculate TrW = TrC * CrI * IrW. TrC, CrI were estimated. IrW = WrI^-1
          sighting.TrW = sighting.TrC * CrI * WrI.transpose();
          QuaternionType q(sighting.TrW);
          sighting.TqW = q.coeffs();
          sighting.image_idx = i_msg;
          apriltag_sightings.push_back(sighting);
          VLOG(2) << "Tag " << sighting.tag_id << " Transform = (" << sighting.reproj_error << ")\n"
              << sighting.TrC << "\n" << sighting.TpC;
        } // for each tag
      } // if n_tags > 0
      i_msg++;
    } // while each msg
    VLOG(1) << "Processed " << i_msg << " tag images seen in " << CqI_estimation_interval*1e-6 << " sec";
  }

  /* Collect unique tags seen */
  std::vector<std::string> unique_tags_seen;
  for (int i_st=0; i_st<apriltag_sightings.size(); i_st++) {
    auto i = std::find(unique_tags_seen.begin(), unique_tags_seen.end(),
        apriltag_sightings[i_st].tag_id);
    if (i==unique_tags_seen.end()) unique_tags_seen.push_back(apriltag_sightings[i_st].tag_id);
  }
  VLOG(1) << "Total number of tag sightings = " << apriltag_sightings.size()
      << " with " << unique_tags_seen.size() << " unique tags.";
  
  /* Calculate an estimate of tag rotations by averaging quaternions over all tag sightings */
  QuaternionVectorType tag_rotations_TqW;
  for (int i_tag=0; i_tag<unique_tags_seen.size(); i_tag++) {
    //VLOG(1) << "Tag " << unique_tags_seen[i_tag];
    Eigen::Matrix4d wM; wM.setZero();
    double total_wt = 0.0;
    for (int i_st=0; i_st<apriltag_sightings.size(); i_st++) {
      if (apriltag_sightings[i_st].tag_id==unique_tags_seen[i_tag]) {
        Eigen::Matrix4d qM = apriltag_sightings[i_st].TqW * apriltag_sightings[i_st].TqW.transpose();
        double wt = 1./apriltag_sightings[i_st].reproj_error;
        wM += wt * qM;
        total_wt += wt;
        VLOG(3) << "  " << apriltag_sightings[i_st].image_idx << " "
            << apriltag_sightings[i_st].TqW.transpose();
      }
    } // for
    wM /= total_wt;
    //VLOG(1) << "\n" << wM;
    // Solve for eigenvectors
    Eigen::EigenSolver<Eigen::Matrix4d> eigen_solver(wM);
    //VLOG(3) << "eigenvalues :\n" << eigen_solver.eigenvalues();
    //VLOG(3) << "eigenvectors:\n" << eigen_solver.eigenvectors();
    Eigen::Vector4d eval_real = eigen_solver.eigenvalues().real();
    Eigen::Vector4d::Index max_idx; eval_real.maxCoeff(&max_idx);
    Eigen::Vector4d evec_real = eigen_solver.eigenvectors().col(max_idx).real();
    VLOG(1) << "Avg rotation for " << unique_tags_seen[i_tag] << " = " << evec_real.transpose();
    QuaternionType tag_avg_q(evec_real(3), evec_real(0), evec_real(1), evec_real(2));
    tag_rotations_TqW.push_back(tag_avg_q);
  } // for each unique tag
  { // Mutual tag rotations - for checking/reporting
    int anchor_quat_idx = 3;
    // TODO: Chose base tag as the one in first image that is straight ahead of camera.
    for (int i=0; i<tag_rotations_TqW.size(); i++) {
      // We have TqW. We need T0qTi = T0qW (x) T1qW^-1.
      QuaternionType T0qT = tag_rotations_TqW[anchor_quat_idx] * tag_rotations_TqW[i].inverse();
      RotationMatType T0rT(T0qT);
      AngleAxisType aa(T0rT);
      VLOG(1) << unique_tags_seen[anchor_quat_idx] << " r " << unique_tags_seen[i] << " aa = "
          << aa.axis().transpose() << ", " << aa.angle()*DegreesPerRadian;
    }
  }
  if (false) { // Check mutual tag rotations in a single image
    int img_idx = 0;
    std::string ref_tag = "Tag36h11_5";
    VLOG(1) << "Base tag = " << ref_tag << " img_idx = " << img_idx;
    AprilTagSighting base_sighting;
    std::vector<AprilTagSighting, Eigen::aligned_allocator<AprilTagSighting>> img_sightings;
    for (int i=0; i<apriltag_sightings.size(); i++) {
      if (apriltag_sightings[i].image_idx==img_idx) {
        img_sightings.push_back(apriltag_sightings[i]);
        if (apriltag_sightings[i].tag_id==ref_tag) base_sighting = apriltag_sightings[i];
      }
    }
    for (int i=0; i<img_sightings.size(); i++) {
      // We have TrC. We need T0rTi = T0rC x T1rC^-1
      Eigen::Matrix3d T0rT = base_sighting.TrC * img_sightings[i].TrC.transpose();
      AngleAxisType aa(T0rT);
      VLOG(1) << base_sighting.tag_id << " r " << img_sightings[i].tag_id << " aa = "
          << aa.axis().transpose() << ", " << aa.angle()*DegreesPerRadian;
      Eigen::Matrix3d test_rotn_mat = std::cos(aa.angle())*Eigen::Matrix3d::Identity()
          + std::sin(aa.angle())*SkewSymmetricMatrix(aa.axis())
          + (1.0 - std::cos(aa.angle()))*(aa.axis()*aa.axis().transpose());
      VLOG(1) << "\n" << test_rotn_mat.transpose() * T0rT; // this should be identity3 matrix
    }
  }

  /* We now have starting estimates of rotations of tags and camera poses. We can now create a
   * convex optimization problem for solving tag positions, camera positions and velocities,
   * accelerometer biases, camera to imu translation and gravity vector. */
  
  /* Integration of IMU rotation and acceleration readings
   *  IMU provides consecutive quaternions of the form WqI0, WqI1, ...
   *  We calculate change in IMU pose assuming angular velocity was constant in this duration
   *  I0qI1 = WqI0.inverse * WqI1, so generally d_imu[i] = imu[i].inverse * imu[i+1]
   *  This is converted to angleaxis using standard Eigen conversion function
   *  This gives theta and k. omega = theta / delta_t
   *  [omega, theta, k] will be used to calculate the rotation integrals used in the IMU formulas
   * Find all IMU readings from image C[i] to C[i+1], these give a set I[0..N], excluding C[i+1]
   * Now calculate s[i], y[i], P[i] and V[i] using all readings [0..N] using subscript i'
   *  Express all omega[i'] and accelerometer readings am[i'] in C[i] frame
   *  Use closed-form first integration to calculate first integral for all end and mid-points.
   *  Use Simpson's rule to integrate the second time.
   * Do this for all cameras images C[i]
   *
   * Residuals for camera positions and velocities
   *  WpC[i+1] = WpC[i] + WvC[i]*dt[i] + Wg*0.5*dt[i]^2 + WrC[i]*(y[i] - P[i]*ba) + p_noise[i]
   *  WvC[i+1] = WvC[i] + Wg*dt[i] + WrC[i]*(s[i] - V[i]*ba) + v_noise[i]
   *  WpC[0] = 0
   *  dt = t[i+1] - t[i]
   *  p_noise[i] and v_noise[i] are IMU process model noises
   *
   * Calculate noises in IMU process model - how?
   * 
   * Residuals for each tag sighting
   *  Cpf = CrW * WrT * ( Tpf - TpC )
   *  CpT = CrW * ( WpT - WpC )
   *  CrW, WrT, TpF are fixed. WpT, WpC are to be estimated.
   *  Cpf is the corner location in tag frame
   * [u,v] = [x/z, y/z] where [x,y,z] = K*Cpf is the location of each corner in image C
   *  each corner's [u,v] will give the residual with respect to observations
   * To make the cost function convex, we will setup the residuals to be [(uz-x)^2, (vz-y)^2]
   *  and the sigma will be sigma_image*z_estimate where z_estimate is the tag position estimate
   *  in camera C
   * 
   * In the convex problem we only solve for position and velocity of camera images
   * So allocate a position and velocity Vector3d for each camera image but for first only velocity
   *    allocate one position for each tag. Tag rotations and tag edges are assumed to be known
   *    allocate a vector3d for gravitation
   *    allocate a vector3d for accelerometer biases
   * Initiate a problem
   *  Apply position and velocity residuals between consecutive C[i]'s
   *  Apply tag sighting constraints
   * Solve
   */

  // Testing imu kinematic integrals
  if (false) { 
    Eigen::Vector3d s,y;
    Eigen::Matrix3d V,P;
    ImuReadingsIntegralType imu_i;
    
    std::vector<ImuReadingType> readings_vector;
    ImuReadingType rdng;
    Vector3dType axis; axis << 0., 0., 1.;
    double angle = Pi*0.25;
    AngleAxisType aa(angle, axis);
    QuaternionType q(aa);
    rdng.timestamp = 0;
    rdng.acceleration << 1., 0., 0.;
    readings_vector.push_back(rdng); // makes a copy
    rdng.timestamp += 1e06;
    rdng.quaternion *= q;
    rdng.acceleration << 1., 0., 0.;  
    readings_vector.push_back(rdng); // makes a copy
    rdng.timestamp += 1e06;
    rdng.quaternion *= q;
    rdng.acceleration << 1., 0., 0.;  
    readings_vector.push_back(rdng); // makes a copy
    rdng.timestamp += 1e06;
    rdng.quaternion *= q;
    rdng.acceleration << 1., 0., 0.;  
    readings_vector.push_back(rdng); // makes a copy
    for (int i=0; i<readings_vector.size(); ++i) {
      VLOG(1) << "Reading " << i << ": " << readings_vector[i];
    }
    //VLOG(1) << "Reading 0 rotmat = \n" << readings_vector[0].quaternion.toRotationMatrix();
    IntegrateImuKinematics(
      readings_vector,
      0, 3,
      1.0,
      &V, &P, &s, &y
    );
    VLOG(1) << "V = \n" << V;
    VLOG(1) << "P = \n" << P;
    VLOG(1) << "s = " << s.transpose();
    VLOG(1) << "y = " << y.transpose();

    IntegrateImuKinematics(
      readings_vector,
      0, 3,
      1.0,
      &imu_i
    );
    VLOG(1) << imu_i;
    
    // Dynamically allocate memory and initiate
    double** parameters = new double*[6];
    for (int i=0; i<6; i++) parameters[i] = new double[3];
    for (int i=0; i<6; i++) for (int j=0; j<3; j++) parameters[i][j] = 0.;
    double* residuals = new double[7];
    double** jacobian = new double*[6];
    for (int i=0; i<6; i++) jacobian[i] = new double[21]; // 7x3 matrices in row major order
    for (int i=0; i<6; i++) for (int j=0; j<21; j++) jacobian[i][j] = 0.;
    
    // Assign some values
    for (int i=0; i<3; i++) parameters[1][i] = 1.;
    for (int i=0; i<3; i++) parameters[3][i] = 1.;
    for (int i=0; i<3; i++) parameters[4][i] = 10.;
    
    // Noise
    double sigma_a = 1.0;
    double grav_lb = 9.9;
    double grav_ub = 10.1;
    double sigma_g = (grav_ub*grav_ub - grav_lb*grav_lb)/6.0;
    
    //ConvexImuResidualFunction cvx_resid (&P, &V, &y, &s, 3.0, 10);
    ConvexImuResidualFunction cvx_resid (&imu_i, 10, sigma_a, sigma_g);
    cvx_resid.Evaluate(parameters, residuals, jacobian);
    
    // Display the values
    std::cout << "Parameters:\n";
    for (int i=0; i<6; i++) {
      std::cout << i << ": ";
      for (int j=0; j<3; j++) {
        std::cout << parameters[i][j] << ", ";
      } std::cout << "\n";
    }
    std::cout << "Residuals:\n    ";
    for (int i=0; i<7; i++) std::cout << residuals[i] << ", "; std::cout << "\n";
    std::cout << "Jacobians:\n";
    for (int i=0; i<6; i++) {
      std::cout << i << ": ";
      for (int j=0; j<7; j++) {
        std::cout << jacobian[i][j*3] << " " << jacobian[i][j*3+1] << " " << jacobian[i][j*3+2] << ", ";
      }
      std::cout << "\n";
    }
    
    // Release dynamically allocated memory
    for (int i=0; i<6; i++) delete[] parameters[i];
    delete[] parameters;
    delete[] residuals;
    for (int i=0; i<6; i++) delete[] jacobian[i];
    delete[] jacobian;
    
  } // Kinematic integrals sanity checks

  // Testing the residuals for AprilTag sighting error
  if (false) {
    VLOG(1) << "Testing the residuals for AprilTag sightings";
    
    int img_idx = 0;
    std::string ref_tag = "Tag36h11_5";
    VLOG(1) << "Base tag = " << ref_tag << " img_idx = " << img_idx;
    AprilTagSighting base_sighting;
    std::vector<AprilTagSighting, Eigen::aligned_allocator<AprilTagSighting>> img_sightings;
    for (int i=0; i<apriltag_sightings.size(); i++) {
      if (apriltag_sightings[i].image_idx==img_idx) {
        img_sightings.push_back(apriltag_sightings[i]);
        if (apriltag_sightings[i].tag_id==ref_tag) base_sighting = apriltag_sightings[i];
      }
    }
    
    double sigma_im = 1.0; // in pixels
    Eigen::Vector3d WpI; WpI << 0., 0., 0.;
    Eigen::Matrix3d WrT = base_sighting.TrW.transpose();
    Eigen::Matrix3d WrI(cam1_apriltag_quaternions[img_idx]);
    Eigen::Vector3d CpI; CpI << .05, 0., 0.;
    Eigen::Vector3d CpT = - base_sighting.TrC.transpose() * base_sighting.TpC;
    Eigen::Vector3d IpC = - CrI.transpose() * CpI;
    Eigen::Vector3d WpT = WpI + WrI * (IpC + CrI.transpose() * CpT);
    VLOG(1) << "\nCamera matrix = \n" << base_sighting.cam_K;
    VLOG(1) << "\nCorners = \n" << base_sighting.image_coords;
    VLOG(1) << "\nCpf = \n" << base_sighting.Cpf;
    
    AprilTagCamImuConvexCostFunction atci(
        &CrI, &WrI, &WrT,
        &base_sighting.cam_K,
        &base_sighting.image_coords,
        &base_sighting.Cpf,
        &sigma_im,
        &april_tag_size      
    );
    
    // Dynamically allocate memory and initiate
    double** parameters = new double*[3];
    for (int i=0; i<3; i++) parameters[i] = new double[3];
    for (int i=0; i<3; i++) for (int j=0; j<3; j++) parameters[i][j] = 0.;
    double* residuals = new double[8];
    double** jacobian = new double*[3];
    for (int i=0; i<3; i++) jacobian[i] = new double[24]; // 7x3 matrices in row major order
    for (int i=0; i<3; i++) for (int j=0; j<24; j++) jacobian[i][j] = 0.;
    
    // Assign some values WpI, WpT, CpI
    for (int i=0; i<3; i++) parameters[0][i] = WpI[i]; // WpI
    for (int i=0; i<3; i++) parameters[1][i] = WpT[i]; // WpT
    for (int i=0; i<3; i++) parameters[2][i] = CpI[i]; // CpI
    
    atci.Evaluate(parameters, residuals, jacobian);
    
    // Display the values
    std::cout << "Parameters:\n";
    for (int i=0; i<3; i++) {
      std::cout << i << ": ";
      for (int j=0; j<3; j++) {
        std::cout << parameters[i][j] << ", ";
      } std::cout << "\n";
    }
    std::cout << "Residuals:\n    ";
    for (int i=0; i<8; i++) std::cout << residuals[i] << ", "; std::cout << "\n";
    std::cout << "Jacobians:\n";
    for (int i=0; i<3; i++) {
      std::cout << i << ": ";
      for (int j=0; j<8; j++) {
        std::cout << jacobian[i][j*3] << " " << jacobian[i][j*3+1] << " " << jacobian[i][j*3+2] << ", ";
      }
      std::cout << "\n";
    }

    // Release dynamically allocated memory
    for (int i=0; i<3; i++) delete[] parameters[i];
    delete[] parameters;
    delete[] residuals;
    for (int i=0; i<3; i++) delete[] jacobian[i];
    delete[] jacobian;

  } // Testing residuals
  
  // Interpolate IMU readings for cameras
  std::vector<ImuReadingType> imu1_readings, cam1_imu1_interp_readings;
  ExtractImuReadingsFromImuMessages(imu1_msgs, &imu1_readings);
  std::vector<LinearInterpolation> cam1_imu1_interp;
  std::vector<int64_t> cam1_initial_calib_ts;
  { // Build a vector of cam1 timestamps to be used for calibration
    int i=0;
    while (cam1_apriltag_timestamps[i] <= initial_estimation_end_timestamp) {
      cam1_initial_calib_ts.push_back(cam1_apriltag_timestamps[i]);
      i++;
    }
    VLOG(1) << "Using " << cam1_initial_calib_ts.size() << " images for calibration.";
  }
  InterpolateImuReadings(imu1_readings, cam1_initial_calib_ts,
      &cam1_imu1_interp, &cam1_imu1_interp_readings);

  /* Build/solve convex optimization problem for VIO
   * State to be estimated are: positions/velocitiesfor all image locations, positions for all tags,
   * gravity vector, bias vector.
   */
  // Holders for estimates
  Eigen::Matrix<double,3,Eigen::Dynamic> cam1_positions, cam1_velocities, tag_positions;
  Eigen::Matrix<double,3,1> imu1_gravity, imu1_accel_bias;
  Eigen::Matrix3d cam1_CrI = CrI;
  Eigen::Vector3d cam1_CpI; cam1_CpI.setZero();
  Eigen::Vector3d starting_posn; starting_posn.setZero();
  double imu_accel_mult = 1.0;
  int32_t num_pose_estimates = cam1_imu1_interp_readings.size();
  int32_t num_tags = unique_tags_seen.size();
  int32_t num_tag_sightings = apriltag_sightings.size();
  if (true) {
    
    // Constants of optimization
    double grav_mag, grav_mag_lb, grav_mag_ub, grav_range_stdev, accel_reading_factor;
    double accel_factor, sigma_a, sigma_g, sigma_im;
    grav_mag = 9.8; // m/s^2 - mean gravity magnitude - in convex formulation this is fixed
    grav_mag_lb = 9.7; // m/s^2 - lower bound of gravity mag
    grav_mag_ub = 9.9; // m/s^2 - upper bound of gravity mag
    grav_range_stdev = 4.0; // range of stdev's between grav ub and lb
    sigma_g = (grav_mag_ub*grav_mag_ub - grav_mag_lb*grav_mag_lb)/grav_range_stdev;
    sigma_a = 400.0*1e-6*grav_mag; // m/s^s/sqrt(Hz) from the datasheet
    accel_reading_factor = 8192.0; // LSB/(m/s^2) from the datasheet
    accel_factor = grav_mag/accel_reading_factor;
    sigma_im = 0.5;
    
    VLOG(1) << "Setting up problem for " << num_pose_estimates << " camera poses and "
        << num_tags << " tags";
    
    // Allocate memory for the estimates
    cam1_positions.resize(3, num_pose_estimates); cam1_positions.setZero();
    cam1_positions.col(0) = starting_posn;
    cam1_velocities.resize(3, num_pose_estimates); cam1_velocities.setZero();
    tag_positions.resize(3, num_tags); tag_positions.setZero();
    
    // Integrate imu readings for the calibration period
    std::vector<ImuReadingsIntegralType> cam1_imu1_readings_integrals;
    cam1_imu1_readings_integrals.resize(num_pose_estimates-1);  // allocate memory
    for (int i_reading=0; i_reading<num_pose_estimates-2; i_reading++) {
      IntegrateImuKinematics(
        imu1_readings,
        cam1_imu1_interp_readings[i_reading],
        cam1_imu1_interp_readings[i_reading+1],
        cam1_imu1_interp[i_reading].index+1,
        cam1_imu1_interp[i_reading+1].index,
        accel_factor,
        &cam1_imu1_readings_integrals[i_reading]
      );
      VLOG_EVERY_N(1, 300) << "Integral at " << i_reading << cam1_imu1_readings_integrals[i_reading];
    }
    
    // Calculations for Apriltag sightings
    StdVectorOfMatrix3dType cam1_WrIs; cam1_WrIs.resize(num_pose_estimates);
    for (int i=0; i<num_pose_estimates; i++)
        cam1_WrIs[i] = cam1_imu1_interp_readings[i].quaternion.toRotationMatrix();
    StdVectorOfMatrix3dType cam1_WrTs; cam1_WrTs.resize(num_tags);
    for (int i=0; i<num_tags; i++)
        cam1_WrTs[i] = tag_rotations_TqW[i].toRotationMatrix().transpose();
    
    // Init problem
    ceres::Problem problem;
    
    // Build problem
    
    // Add constraints for IMU position/velocity
    for (int i_pose=0; i_pose<1; i_pose++) {
      ceres::CostFunction* pose_constraint =
          new ConvexFirstImuResidualFunction( //ConvexImuKinFirstFunction(
            &cam1_imu1_readings_integrals[i_pose],
            &starting_posn,
            grav_mag, sigma_a, sigma_g
          );
      ceres::LossFunction* quadratic_loss = 
          NULL;
      problem.AddResidualBlock(
        pose_constraint,
        quadratic_loss,
        cam1_positions.data() + 3*(i_pose+1), // posn1
        cam1_velocities.data() + 3*i_pose, // velo0
        cam1_velocities.data() + 3*(i_pose+1), // velo1
        imu1_gravity.data(), // grav
        imu1_accel_bias.data() // bias
        //&imu_accel_mult // mult
      );
    }
    for (int i_pose=1; i_pose<num_pose_estimates-2; i_pose++) {
      ceres::CostFunction* pose_constraint =
          new ConvexImuResidualFunction( //ConvexImuKinematicsFunction(
            &cam1_imu1_readings_integrals[i_pose],
            grav_mag, sigma_a, sigma_g
          );
      ceres::LossFunction* quadratic_loss = 
          NULL;
      problem.AddResidualBlock(
        pose_constraint,
        quadratic_loss,
        cam1_positions.data() + 3*i_pose, // posn0
        cam1_positions.data() + 3*(i_pose+1), // posn1
        cam1_velocities.data() + 3*i_pose, // velo0
        cam1_velocities.data() + 3*(i_pose+1), // velo1
        imu1_gravity.data(), // grav
        imu1_accel_bias.data() // bias
        //&imu_accel_mult // mult
      );
    }
    
    // Add constraints using tag sightings
    for (int i_stng=0; i_stng<num_tag_sightings; i_stng++) {
      
      // Locate the camera pose number and the tag number for this sighting
      int32_t i_cam, i_tag;
      i_cam = apriltag_sightings[i_stng].image_idx;
      auto itrtr = std::find(unique_tags_seen.begin(), unique_tags_seen.end(),
          apriltag_sightings[i_stng].tag_id);
      if (itrtr!=unique_tags_seen.end()) {
        i_tag = std::distance(unique_tags_seen.begin(), itrtr); 
      } else {
        LOG(ERROR) << "Could not find tag " << apriltag_sightings[i_stng].tag_id << " in unique tags";
        return false;
      }
      
      if (i_cam==0) {
        ceres::CostFunction* pose_constraint =
            new AprilTagFixedPoseConvexCostFunction2(
                &cam1_CrI, &cam1_WrIs[i_cam], &cam1_WrTs[i_tag],
                &apriltag_sightings[i_stng].cam_K,
                &apriltag_sightings[i_stng].image_coords,
                &apriltag_sightings[i_stng].Cpf,
                &starting_posn,
                &sigma_im,
                &april_tag_size
            );
        ceres::LossFunction* quadratic_loss = 
            NULL;
        problem.AddResidualBlock(
          pose_constraint,
          quadratic_loss,
          tag_positions.data() + 3*i_tag // WpT
          //,cam1_CpI.data() // CpI
        );        
      } else {
        ceres::CostFunction* pose_constraint =
            new AprilTagCamImuConvexCostFunction2(
                &cam1_CrI, &cam1_WrIs[i_cam], &cam1_WrTs[i_tag],
                &apriltag_sightings[i_stng].cam_K,
                &apriltag_sightings[i_stng].image_coords,
                &apriltag_sightings[i_stng].Cpf,
                &sigma_im,
                &april_tag_size
            );
        ceres::LossFunction* quadratic_loss = 
            NULL;
        problem.AddResidualBlock(
          pose_constraint,
          quadratic_loss,
          cam1_positions.data() + 3*i_cam, // WpI
          tag_positions.data() + 3*i_tag // WpT
          //,cam1_CpI.data() // CpI
        );
      }
      
    } // for i_stng
    
    // Bounds on values
    /*problem.SetParameterLowerBound(cam1_CpI.data(), 0, -0.20 );
    problem.SetParameterUpperBound(cam1_CpI.data(), 0,  0.20 );
    problem.SetParameterLowerBound(cam1_CpI.data(), 1, -0.05 );
    problem.SetParameterUpperBound(cam1_CpI.data(), 1,  0.05 );
    problem.SetParameterLowerBound(cam1_CpI.data(), 2, -0.20 );
    problem.SetParameterUpperBound(cam1_CpI.data(), 2,  0.20 );*/
    
    
    // Solve problem
    ceres::Solver::Options options;
    options.max_num_iterations = 300;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    if (true) std::cout << summary.FullReport() << std::endl;
    
    imu1_accel_bias /= accel_factor;
  }
  // Report results
  std::cout << "Gravity = " << imu1_gravity.transpose() << ", " << imu1_gravity.norm() << "\n";
  std::cout << "Accel biases = " << imu1_accel_bias.transpose() << "\n";
  std::cout << "Accel mult = " << imu_accel_mult << "\n";
  std::cout << "cam1_CpI = " << cam1_CpI.transpose() << "\n";
  AngleAxisType cam1_CaI(cam1_CrI);
  std::cout << "cam1_CrI = " << cam1_CaI.axis().transpose() << ", " << cam1_CaI.angle()*DegreesPerRadian << "\n";
  // Tag locations wrt starting pose
  std::cout << "Tags = ";
  for (int i=0; i<unique_tags_seen.size(); i++) {std::cout << unique_tags_seen[i] << " "; }
  std::cout << "\n";
  std::cout << "Tag positions = \n" 
      << cam1_imu1_interp_readings[0].quaternion.toRotationMatrix().transpose() * tag_positions
      << "\n";
  WriteMatrixToCSVFile(plots_dir+"tag_positions.csv", tag_positions.transpose());
  WriteMatrixToCSVFile(plots_dir+"cam1_positions.csv", cam1_positions.transpose());
  
  // testing data pointer for a matrix
  /*Eigen::Matrix<double,3,10> mat;
  for (int i=0; i<3; i++) for (int j=0; j<10; j++) mat(i,j) = i*10+j;
  VLOG(1) << "mat = \n" << mat;
  double* mat_ptr = mat.data();
  for (int i=0; i<30; i++) std::cout << mat_ptr[i] << " "; std::cout << "\n";
  */
  
  // All done
  return 0;
}

