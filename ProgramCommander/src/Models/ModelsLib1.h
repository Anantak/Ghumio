/* Models library */

namespace anantak {

typedef Eigen::Matrix<double,2,1> Vector2d;
typedef Eigen::Matrix<double,3,1> Vector3d;
typedef Eigen::Matrix<double,4,1> Vector4d;
typedef Eigen::Matrix<double,6,1> Vector6d;
typedef Eigen::Matrix<double,7,1> Vector7d;

typedef Eigen::Matrix<double,2,2> Matrix2d;
typedef Eigen::Matrix<double,3,3> Matrix3d;
typedef Eigen::Matrix<double,4,4> Matrix4d;
typedef Eigen::Matrix<double,6,6> Matrix6d;

typedef Eigen::Matrix<double,1,3> Matrix1x3d;
typedef Eigen::Matrix<double,2,3> Matrix2x3d;
typedef Eigen::Matrix<double,2,4> Matrix2x4d;
typedef Eigen::Matrix<double,3,2> Matrix3x2d;
typedef Eigen::Matrix<double,6,18> Matrix6x18d;

/** Add poses
 * Adds two poses */
bool AddPoses(
    const Vector7d* WPA, const Vector7d* APB, Vector7d* WPB,  // Poses
    Matrix6d* WJA, Matrix6d* AJB,                             // Jacobians
    const Matrix6d* WQA, const Matrix6d* AQB, Matrix6d* WQB   // Covariance matrices
) {
  
  if (!WPA || !APB || !WPB) {
    LOG(ERROR) << "WPA or APB or WPB are NULL. Quit.";
    return false;
  }
  
  const Eigen::Quaterniond AqW(WPA->data());
  const Eigen::Quaterniond BqA(APB->data());
  const Eigen::Vector3d WpA(WPA->data()+4);
  const Eigen::Vector3d ApB(APB->data()+4);
  const Eigen::Matrix3d WrA(AqW.conjugate());
  
  Eigen::Quaterniond BqW = BqA * AqW;
  Eigen::Matrix3d BrW(BqW);
  Eigen::Vector3d WApB = WrA*ApB;
  Eigen::Vector3d WpB = WpA + WApB;
  
  Eigen::Map<Eigen::Vector4d> BqW_map(WPB->data());
  Eigen::Map<Eigen::Vector3d> WpB_map(WPB->data()+4);
  BqW_map = BqW.coeffs();
  WpB_map = WpB;
  
  if (!WJA || !AJB) {return true;}
  
  WJA->setZero();
  WJA->block<3,3>(0,0) =  Eigen::Matrix3d::Identity();
  WJA->block<3,3>(3,3) =  Eigen::Matrix3d::Identity();
  WJA->block<3,3>(3,0) = -SkewSymmetricMatrix(WApB);

  AJB->setZero();
  AJB->block<3,3>(0,0) =  WrA;
  AJB->block<3,3>(3,3) =  WrA;
  
  if (!WQA || !AQB || !WQB) {return true;}
  
  *WQB = (*WJA) * (*WQA) * (*WJA).transpose() + (*AJB) * (*AQB) * (*AJB).transpose();
  
  return true;
}

/* Invert pose */
bool InvertPose(
    const Vector7d* APB, Vector7d* BPA,   // Poses
    Matrix6d* AJB,                        // Jacobian
    const Matrix6d* AQB, Matrix6d* BQA    // Covariance matrices
) {
  
  if (!APB || !BPA) {
    LOG(ERROR) << "APB or BPA are NULL. Quit.";
    return false;
  }
  
  const Eigen::Quaterniond BqA(APB->data());
  const Eigen::Vector3d ApB(APB->data()+4);
  const Eigen::Matrix3d BrA(BqA);
  
  Eigen::Quaterniond AqB = BqA.conjugate();
  Eigen::Vector3d BpA = -BrA*ApB;
  
  Eigen::Map<Eigen::Vector4d> AqB_map(BPA->data());
  Eigen::Map<Eigen::Vector3d> BpA_map(BPA->data()+4);
  AqB_map = AqB.coeffs();
  BpA_map = BpA;
  
  if (!AJB) return true;
  
  AJB->setZero();
  AJB->block<3,3>(0,0) = -BrA;
  AJB->block<3,3>(3,3) = -BrA;
  AJB->block<3,3>(3,0) = -BrA*SkewSymmetricMatrix(ApB);
  
  if (!AQB || !BQA) return true;
  
  *BQA = (*AJB) * (*AQB) * (*AJB);
  
  return true;
}

/** Diff poses */
bool DiffPoses(
    const Vector7d* WPA, const Vector7d* WPB, Vector7d* APB,  // Poses
    Matrix6d* WJA, Matrix6d* WJB,                             // Jacobians
    const Matrix6d* WQA, const Matrix6d* WQB, Matrix6d* AQB   // Covariance matrices    
) {
  if (!WPA || !APB || !WPB) {
    LOG(ERROR) << "WPA or APB or WPB are NULL. Quit.";
    return false;
  }
  
  const Eigen::Quaterniond AqW(WPA->data());
  const Eigen::Quaterniond BqW(WPB->data());
  const Eigen::Vector3d WpA(WPA->data()+4);
  const Eigen::Vector3d WpB(WPB->data()+4);
  const Eigen::Matrix3d ArW(AqW);
  
  Eigen::Quaterniond BqA = BqW * AqW.conjugate();
  Eigen::Vector3d WApB = WpB - WpA;
  Eigen::Vector3d ApB = ArW*WApB;

  Eigen::Map<Eigen::Vector4d> BqA_map(APB->data());
  Eigen::Map<Eigen::Vector3d> ApB_map(APB->data()+4);
  BqA_map = BqA.coeffs();
  ApB_map = ApB;
  
  if (!WJA || !WJB) return true;
  
  WJA->setZero();
  WJA->block<3,3>(0,0) = -ArW;
  WJA->block<3,3>(3,3) = -ArW;
  WJA->block<3,3>(3,0) =  ArW*SkewSymmetricMatrix(WApB);
  
  WJB->setZero();
  WJB->block<3,3>(0,0) =  ArW;
  WJB->block<3,3>(3,3) =  ArW;
  
  if (!WQA || !WQB || !AQB) return true;
  
  *AQB = (*WJA) * (*WQA) * (*WJA).transpose() + (*WJB) * (*WQB) * (*WJB).transpose();
  
  return true;
}


/* Point projection on an image */
bool ProjectPoint(
    const Vector3d* xyz,    // Point in camera coordinates
    const Vector4d* K,      // Camera marix components fx, fy, cx, cy 
    const Vector3d* D, const Vector2d* T,  // Radial and tangential distortion coefficients
    Vector2d*   uvim,         // Point in image
    Matrix2x3d* duvim_dxyz,    // Jacobian of projection with point coordinates
    Matrix2x4d* duvim_dK,      // Jacobian of projection with camera matrix
    Matrix2x3d* duvim_dD,      // Jacobian of projection with radial distortion coefficients
    Matrix2d*   duvim_dT       // Jacobian of projection with tangential distortion coefficients
) {
  if (!xyz || !K || !uv_im) {LOG(ERROR) << "!xyz || !K || !uv_im. Quit."; return false;}
  if (!D || !T) {LOG(ERROR) << "!D || !T"; return false;}
  
  if (xyz->(2) <= Epsilon) {LOG(ERROR) << "xyz->(2) <= Epsilon. Quit."; return false;}
  
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
  Matrix2d dudvd_duv = uv*kkkrrr*uvuvuv + d_*Eigen::Vector2d::Identity() + tuv;
  Matrix2x3d duv_dxyz; duv_dxyz << 1., 0., -u,  0., 1., -v;  duv_dxyz /= (*xyz)(2);
  
  *duvim_dxyz = fxfy * dudvd_duv * duv_dxyz;
  *duvim_dK << udvd(0), 0., 1., 0.,   0., udvd(1), 0., 1.;
  *duvim_dD = fxfy * dudvd_dD;
  *duvim_dT = fxfy * dudvd_dT;
  
  return true;
}


/* Kinematic state is pose, velocity and acceleration */

/* Move forward assuming constant accelerations */
bool MoveInTime(
    const Vector7d* P0, const Vector6d* V0, const Vector6d* A0,
    const double dt,
    Vector7d* P1, Vector6d* V1, Vector6d* A1,
    Matrix6x18d* P1J0, Matrix6x18d* V1J0, Matrix6x18d* A1J0,
    Matrix6d* P1Jdt, Matrix6d* V1Jdt, Matrix6d* A1Jdt)
{
  
  return true;
}

/* Move in space at this instant */
bool MoveInSpace(
    const Vector7d* P0, const Vector6d* V0, const Vector6d* A0,
    const Vector7d* P, 
    Vector7d* P1, Vector6d* V1, Vector6d* A1,
    Matrix6x18d* P1J0, Matrix6x18d* V1J0, Matrix6x18d* A1J0,
    Matrix6d* P1Jdt, Matrix6d* V1Jdt, Matrix6d* A1Jdt,
    bool pose_is_absolute = true)     // true: P is absolute pose, false: P is relative
{
  
  return true;
}

/* Relative kinematics */
bool DiffKinematics(
    const Vector7d* P0, const Vector6d* V0, const Vector6d* A0,
    const Vector7d* P1, const Vector6d* V1, const Vector6d* A1,
    Vector7d* P01, Vector6d* V01, Vector6d* A01,
    Matrix6x18d* P01J0, Matrix6x18d* V01J0, Matrix6x18d* A01J0,
    Matrix6x18d* P01J1, Matrix6x18d* V01J1, Matrix6x18d* A01J1)
{
  
  return true;
}

bool AddKinematics(
    const Vector7d* P0, const Vector6d* V0, const Vector6d* A0,
    const Vector7d* P01, const Vector6d* V01, const Vector6d* A01,
    Vector7d* P1, Vector6d* V1, Vector6d* A1,
    Matrix6x18d* P1J0, Matrix6x18d* V1J0, Matrix6x18d* A1J0,
    Matrix6x18d* P1J01, Matrix6x18d* V1J01, Matrix6x18d* A1J01)
{
  
  return true;
}



} // namespace