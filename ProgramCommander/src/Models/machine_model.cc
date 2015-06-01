/**

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

#include "ModelsLib0.h"

namespace anantak {
  
  
  /* Machine state
   * Implements the machine state with the following components making a 16x1 vector
   *  Quaternion in a global frame  Gq
   *  Position in a global frame    Gp
   *  Velocity in body frame        Bv
   *  Acceleration in body frame    Ba
   *  Ang velocity in body frame    Bw
   * Uncertainty is measured by a 3-vector for each element making a 15x1 vector
   *  Gq^ = Gq*Gdq or Gr^ = Gr*(I3-[GdQ x])
   *  Gp^ = Gp + Gdp
   *  Bv^ = Bv + Bdv
   *  Ba^ = Ba + Bda
   *  Bw^ = Bw + Bdw
   */
  class MachineState : public State {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef Eigen::Map<Eigen::Vector3d> MapVector3dType;
    typedef Eigen::Map<Eigen::Vector4d> MapVector4dType;
    typedef Eigen::Map<Eigen::Matrix<double,16,1>> MapVector16dType;
    typedef Eigen::Map<Eigen::Matrix<double,15,1>> MapVector15dType;
    
    // Default constructor
    MachineState(): State(), BqaW_(state_),
      BqW_(state_),  WpB_(state_+4),  Bv_(state_+7),  Ba_(state_+10),  Bw_(state_+13),
      Wdq_(error_),  Wdp_(error_+3), Bdv_(error_+6), Bda_(error_+9),  Bdw_(error_+12) {
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
      return (BqaW_.isZero() && WpB_.isZero() && Bv_.isZero() && Ba_.isZero() && Bw_.isZero());
    }
    
    // Set timestamp
    bool SetTimestamp(const int64_t& ts) {
      timestamp_ = ts;
      return true;
    }
    
    // Recalculate the state from error state after optimization. Set errors to zero.
    bool Recalculate() {
      Eigen::Quaterniond BqW(BqW_.data()); // x,y,z,w
      Eigen::Quaterniond Wdq = anantak::ErrorAngleAxisToQuaterion(Wdq_);
      BqW *= Wdq;  // assuming both quaternions are already normalized
      BqW_ = BqW.coeffs();
      WpB_ += Wdp_;
      Bv_ += Bdv_;
      Ba_ += Bda_;
      Bw_ += Bdw_;
      SetErrorZero();
      return true;
    }
    
    // Helpers that send copies of state values
    Eigen::Quaterniond Quaternion() const {
      return Eigen::Quaterniond(BqW_.data()); // x,y,z,w
    }
    Eigen::Vector3d Position() const {
      return Eigen::Vector3d(WpB_);
    }
    
    // Destructor
    virtual ~MachineState() {}
    
    // Timestamp
    int64_t timestamp_;
    // State
    double state_[16];
    // Error
    double error_[15];
    
    // Helper maps
    MapVector4dType BqW_; // quaternion as a vector x,y,z,w
    MapVector3dType BqaW_; // vector part of quaternion
    MapVector3dType WpB_;
    MapVector3dType Bv_;
    MapVector3dType Ba_;
    MapVector3dType Bw_;
    MapVector3dType Wdq_; // angleaxis as a vector
    MapVector3dType Wdp_;
    MapVector3dType Bdv_;
    MapVector3dType Bda_;
    MapVector3dType Bdw_;
  };  // MachineState
  
  
  /* Interpolated machine state
   * Returns an interpolated Machine state at a timestamp between two machine states */
  
  /* Interpolated Pose3d state
   * Returns an interpolated pose 3d state at a timestamp between two machine states */
  
  /* Machine States Residual
   * Connects two machine states */
  
  /* Interpolated Pose3d states residual
   * Connects a machine state with an interpolated Pose3d state */
  
  /* Machine state prior
   * Implements a prior for the machine state */
  
  
  class SlidingWindowFilterIterations {
   public:
    
    struct Options {
      uint64_t longest_problem_interval;    // Problem will be built to this longest length
      uint64_t shortest_problem_interval;   // Problem will be built to this length at a minimum
      uint64_t solving_problem_interval;    // Problem will be solved after every this interval
      uint64_t sliding_window_interval;     // This is the length of time in which states will be solved
      
      Options() :
        longest_problem_interval(10000000),
        shortest_problem_interval(3000000),
        solving_problem_interval(1000000),
        sliding_window_interval(2000000)
      {}
      
      Options(uint64_t lpi, uint64_t spi, uint64_t vpi, uint64_t swi) :
        longest_problem_interval(lpi),
        shortest_problem_interval(spi),
        solving_problem_interval(vpi),
        sliding_window_interval(swi)
      {}
    }; // Options
    
    SlidingWindowFilterIterations::Options options_;
    
    int64_t start_ts_;
    int64_t data_begin_ts_;
    int64_t solve_begin_ts_;
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
              << " solve_begin_ts_ was " << data_begin_ts_ - solve_begin_ts_ << "musecs before data_begin_ts_";
          solve_begin_ts_ = data_begin_ts_;
        }
      } else {
        // Data begin ts and solve begin ts both remain at the same place
      }
      
      // Is it time to solve the problem?
      solve_problem_ = (data_end_ts_ >= solve_begin_ts_ + options_.solving_problem_interval);
      
      return true;
    }
    
    // Solve problem update to data. Gets the ts of the last data point when problem is solved
    bool SolveProblem() {
      // Problem was solved, so update solve ts
      solve_begin_ts_ = data_end_ts_;
      
      return true;
    }
    
  };  // SlidingWindowFilterIterations
  
  
} // namespace anantak


class TagCamerasCalibrator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  struct Options {
    uint32_t states_frequency;  // Frequency (Hz) at which machine states will be created
    uint64_t states_history_interval;  // amount of states history to keep
    uint64_t states_window_interval;    // Past states are kept for this long interval
    uint64_t starting_sliding_window_interval;    // interval in seconds of the sliding window
    uint16_t starting_num_cameras;   // number of cameras system expects to calibrate
    uint16_t max_camera_frequency;  // Maximum rate of images from cameras (Hz)
    
    uint16_t num_common_tags_to_combine_maps;   // number of tags if common, combine the tag maps
    uint64_t check_maps_to_combine_interval;    // interval to check if tags should be combined
    
    double zero_information;
    double infinite_information;
    
    bool run_filtering;
    
    anantak::SlidingWindowFilterIterations::Options sliding_window_options;
    uint32_t avg_tags_per_image;
    anantak::AprilTagViewResidual::Options apriltag_view_residual_options;
    
    Options():
    states_frequency(10),
    states_history_interval(60000000),
    states_window_interval(5000000),
    starting_sliding_window_interval(2000000),
    starting_num_cameras(4),
    max_camera_frequency(30),
    
    num_common_tags_to_combine_maps(5),
    check_maps_to_combine_interval(2000000),
    
    zero_information(0.),
    infinite_information(1e+20),
    
    run_filtering(true),
    
    sliding_window_options(
      10000000,   // longest problem length
       3000000,   // shortest problem length
       1000000,   // solving problem interval
       2000000    // sliding window length
    ),
    avg_tags_per_image(5),
    apriltag_view_residual_options()
    
    {}
    
  }; // Options
  
  
  struct IterationRecord {
    int64_t begin_ts;   // Beginning timestamp
    int64_t end_ts;     // Ending timestamp
    
    IterationRecord():
    begin_ts(0), end_ts(0)
    {}
  }; // IterationRecord
  
  // Data members
  TagCamerasCalibrator::Options options_;
  TagCamerasCalibrator::IterationRecord last_iter_record_;
  
  /* Circular queues */
  // Reference poses
  // Queues of camera poses
  // Set of tag maps for each camera
  // Set of cam-to-cam transforms
  
  // Initiate reference-to-cam poses using any available priors (how much information?)
  //  When no priors are there, initiate ref-to-cam[0] has zero pose with infinite information and
  //    ref-to-cam[i] (i>0) have zero information
  
  // Process april tag messages
  //  April tag messages flow in from all cameras
  //  Initiate new reference states for the iteration, add them to the queue. Initiate with zero information.
  //  
  
  // For each camera
  //  Use its tag map to calculate camera poses. If camera pose is calculated, add it to its queue.
  
  // Check if a camera can be combined. If so, combine camera.
  
  // For each combined camera 
  //  Interpolate camera poses at the given timestamps, calculate information using end points and time difference
  //  Add ref-to-cam pose to interpolated poses using information.
  //  Add interpolated poses to reference states using information.
  //  Use all camera-interpolated poses and reference poses to update reference-to-cam poses.
  
  // Combine camera
  //  For each overlapping tag in the tag map, calculate difference in pose.
  //  Add all differences to a tag-map-to-tag-map pose using information
  //  Update reference tag map using information from new camera's tag map
  //  Restate all current camera poses in queue wrt reference tag map using information
  //  Interpolate restated camera poses for timestamps in the reference poses
  //  Calculate cam-to-cam pose difference for each calculated pose, add to cam-to-pose using information
  
  const std::vector<uint16_t> cam_ds_;
  const uint16_t num_cameras_;
  bool calibrations_are_set_;
  const uint16_t reference_cam_idx_;
  int64_t last_map_combine_check_ts_;
  const uint64_t inter_state_interval_;
  int64_t data_start_ts_;
  int64_t last_machine_state_ts_;
  
  // Camera intrinsics for each camera
  std::vector<anantak::CameraIntrinsicsState> camera_intrinsics_;
  
  // Tag maps for each camera. When no longer needed, tag maps can be destroyed
  anantak::StaticAprilTagsMap::Options tags_map_options_;
  uint16_t num_tag_maps_;
  std::vector<std::unique_ptr<anantak::StaticAprilTagsMap>> tags_maps_;
  std::vector<uint16_t> tags_maps_idx_for_cam_;
  
  // Camera pose queues for each camera
  uint32_t num_camera_poses_to_keep_;
  uint32_t num_states_to_keep_;
  std::vector<std::unique_ptr<anantak::CircularQueue<anantak::Pose3dState>>> camera_poses_;
  std::vector<std::unique_ptr<anantak::CircularQueue<anantak::Pose3dState>>> camera_states_;
  std::vector<anantak::FixedPointCQ> camera_poses_marker_;
  std::vector<anantak::FixedPointCQ> camera_states_marker_;
  std::vector<anantak::FixedPointCQ> starting_states_marker_;
  std::vector<std::unique_ptr<anantak::Pose3dState>>  camera_to_reference_poses_;
  
  // Data members for optimization
  uint32_t num_tag_views_to_keep_;
  std::vector<std::unique_ptr<anantak::CircularQueue<anantak::AprilTagViewResidual>>> tag_view_residuals_;
  anantak::SlidingWindowFilterIterations swf_iterations_;
  std::unique_ptr<ceres::Problem> problem_;
  ceres::Problem::Options problem_options_;
  ceres::Solver::Options solver_options_;
  ceres::Solver::Summary solver_summary_;
  std::vector<anantak::FixedPointCQ> tvr_begin_marker_, tvr_end_marker_;
  std::vector<anantak::FixedPointCQ> cpc_begin_marker_, cpc_end_marker_;
  std::vector<int64_t> cp_data_begins_after_ts_;   // this is when a camera is added to the tagposes
  
  TagCamerasCalibrator(
    const TagCamerasCalibrator::Options& options,
    const std::vector<uint16_t>& cam_ds
  ):
  options_(options),
  cam_ds_(cam_ds),
  num_cameras_(cam_ds.size()),
  calibrations_are_set_(false),
  tags_map_options_(),
  num_tag_maps_(cam_ds.size()),
  tags_maps_(),
  tags_maps_idx_for_cam_(),
  reference_cam_idx_(0),
  last_map_combine_check_ts_(0),
  inter_state_interval_(1000000/options.states_frequency),
  data_start_ts_(0),
  last_machine_state_ts_(0),
  num_tag_views_to_keep_(0),
  swf_iterations_(options.sliding_window_options),
  problem_(nullptr),
  problem_options_(),
  solver_options_(),
  solver_summary_()
  {
    LOG(INFO) << "Creating TagCamerasCalibrator";
    LOG(INFO) << "  Number of cameras = " << num_cameras_;
    camera_intrinsics_.resize(num_cameras_, anantak::CameraIntrinsicsState());
    
    // Allocate memory for tags maps for each camera
    for (int i=0; i<num_cameras_; i++) {
      std::unique_ptr<anantak::StaticAprilTagsMap> map_ptr_(
          new anantak::StaticAprilTagsMap(tags_map_options_));
      tags_maps_.push_back(std::move(map_ptr_));
      
      tags_maps_idx_for_cam_.push_back(uint16_t(i));
    }
    num_tag_maps_ = num_cameras_;
    
    // Allocate memory for keeping states history
    num_camera_poses_to_keep_ = options_.max_camera_frequency * options_.states_window_interval / 1000000;
    num_states_to_keep_ = options_.states_frequency * options_.states_history_interval / 1000000;
    for (int i=0; i<num_cameras_; i++) {
      // Camera poses
      std::unique_ptr<anantak::CircularQueue<anantak::Pose3dState>> queue_ptr(
          new anantak::CircularQueue<anantak::Pose3dState>(num_camera_poses_to_keep_));
      camera_poses_.push_back(std::move(queue_ptr));
      // Interpolated camera states
      std::unique_ptr<anantak::CircularQueue<anantak::Pose3dState>> states_ptr(
          new anantak::CircularQueue<anantak::Pose3dState>(num_states_to_keep_));
      camera_states_.push_back(std::move(states_ptr));
      // Camera to reference poses
      std::unique_ptr<anantak::Pose3dState> pose_ptr(new anantak::Pose3dState());
      camera_to_reference_poses_.push_back(std::move(pose_ptr));
    }
    // Reference states
    //std::unique_ptr<anantak::CircularQueue<anantak::Pose3dState>> states_ptr(
    //    new anantak::CircularQueue<anantak::Pose3dState>(num_camera_poses_to_keep_));
    //reference_states_ = (std::move(states_ptr));
    camera_poses_marker_.resize(num_cameras_, anantak::FixedPointCQ());
    camera_states_marker_.resize(num_cameras_, anantak::FixedPointCQ());
    starting_states_marker_.resize(num_cameras_, anantak::FixedPointCQ());
    
    // Insert first dummy elements in pose/states queues and set markers
    for (int i_cam=0; i_cam<num_cameras_; i_cam++) {
      anantak::Pose3dState* pose = camera_poses_[i_cam]->next_mutable_element();
      pose->SetZero();
      camera_poses_marker_[i_cam] = camera_poses_[i_cam]->CurrentFixedPoint();
      VLOG(1) << "Camera" << i_cam << " poses queue starting point = "
          << camera_poses_marker_[i_cam].to_string();
      
      anantak::Pose3dState* state = camera_states_[i_cam]->next_mutable_element();
      state->SetZero();
      camera_states_marker_[i_cam] = camera_states_[i_cam]->CurrentFixedPoint();
      VLOG(1) << "Camera" << i_cam << " states queue starting point = "
          << camera_states_marker_[i_cam].to_string();
      
      camera_to_reference_poses_[i_cam]->SetZero();
      camera_to_reference_poses_[i_cam]->information_ = 1e-8;
      VLOG(1) << "Camera" << i_cam << " starting cam to reference pose = "
          << *camera_to_reference_poses_[i_cam];
    } // for i_cam
    
    // Initiate data members for optimization
    if (!InitiateOptimization()) {
      LOG(ERROR) << "Had a problem in initiating optimization data structures.";
    }
    
  }
  
  // Initiate data structures for optimization
  //  TagView residuals, 
  bool InitiateOptimization() {
    
    // Number of tag view history to keep per camera
    num_tag_views_to_keep_ = (options_.sliding_window_options.longest_problem_interval/1000000) *
        options_.max_camera_frequency * options_.avg_tags_per_image;
    LOG(INFO) << "Initiating tag view residuals queues with length = " << num_tag_views_to_keep_;
    
    for (int i=0; i<num_cameras_; i++) {
      // Tag view residuals
      std::unique_ptr<anantak::CircularQueue<anantak::AprilTagViewResidual>> view_queue_ptr(
          new anantak::CircularQueue<anantak::AprilTagViewResidual>(num_tag_views_to_keep_));
      tag_view_residuals_.push_back(std::move(view_queue_ptr));
    }
    
    options_.apriltag_view_residual_options.sigma_image = 0.5;
    
    problem_options_.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    solver_options_.max_num_iterations = 300;
    solver_options_.minimizer_progress_to_stdout = true;
    
    for (int i_cam=0; i_cam<num_cameras_; i_cam++) {
      tvr_begin_marker_.push_back(anantak::FixedPointCQ());
      tvr_end_marker_.push_back(anantak::FixedPointCQ());
      cpc_begin_marker_.push_back(camera_poses_[i_cam]->CurrentFixedPoint());
      cpc_end_marker_.push_back(camera_poses_[i_cam]->CurrentFixedPoint());
      cp_data_begins_after_ts_.push_back(0);
    }
    
    return true;
  }
  
  virtual ~TagCamerasCalibrator() {}
  
  // Process camera calibration messages
  bool ProcessCameraCalibrationMessages(
    const std::vector<std::unique_ptr<std::vector<anantak::SensorMsg>>>& calib_msgs) {
    
    if (calib_msgs.size() != num_cameras_) {
      LOG(ERROR) << "calib_msgs.size() != num_cameras_  "
          << calib_msgs.size() << ", " << num_cameras_;
      return false;
    }
    
    for (int i=0; i<num_cameras_; i++) {
      camera_intrinsics_[i].SetZero();
      Eigen::Matrix3d K;
      
      const anantak::SensorMsg& calib_msg = calib_msgs[i]->front();
      
      // Calculate the camera matrix and store
      if (!calib_msg.has_mono_calib_no_distort_msg()) {
        LOG(ERROR) << "No calib msg was found in calib message for camera " << i;
        return false;
      }
      
      const anantak::MonocularPinholeCalibrationNoDistortionMsg& msg =
          calib_msg.mono_calib_no_distort_msg();
      K << msg.focal_length(), 0.0, msg.cx(),
           0.0, msg.focal_length(), msg.cy(),
           0.0, 0.0, 1.0;
      VLOG(1) << "Cam " << i << ": Extracted camera matrix\n" << K;
      
      if (!camera_intrinsics_[i].Create(&K)) {
        LOG(ERROR) << "Could not create camera intrinsics state for camera " << i;
        return false;
      }
    }
    
    calibrations_are_set_ = true;
    
    return true;
  }
  
  // Create tag view residuals
  bool CreateTagViewResiduals(
      anantak::StaticAprilTagsMap* tags_map,
      anantak::CircularQueue<anantak::AprilTagViewResidual>* residuals_queue,
      anantak::Pose3dState* cam_pose,
      anantak::CameraIntrinsicsState* cam_intrinsics) {
    
    if (!tags_map) return false;
    if (!residuals_queue) return false;
    if (!cam_pose) return false;
    if (!cam_intrinsics) return false;
    int32_t num_resids_created = 0;
    
    // Create a residual for every tag reading
    int32_t num_tag_views = tags_map->april_tag_readings_->n_msgs();
    for (int i_rdng=0; i_rdng < num_tag_views; i_rdng++) {
      anantak::AprilTagReadingType *tag_rdng = tags_map->april_tag_readings_->at_ptr(i_rdng);
      anantak::StaticAprilTagState *tagTj = NULL;
      if (!tags_map->FindTagInTagMap(tag_rdng->tag_id, &tagTj)) {
        LOG(WARNING) << "Did not find tag in tag map. Not expected. tag_id = " << tag_rdng->tag_id;
        continue;
      }
      bool is_origin_tag = tags_map->IsOriginTag(tagTj);
      // Set the residual object
      anantak::AprilTagViewResidual *tag_view_resid = residuals_queue->next_mutable_element();
      if (!tag_view_resid->Create(tag_rdng, cam_pose, tagTj, cam_intrinsics,
          &options_.apriltag_view_residual_options, is_origin_tag)) {
        LOG(ERROR) << "Could not create tag view residual. Skipping.";
        tag_view_resid->Reset();
        residuals_queue->decrement();
        continue;
      } else {
        num_resids_created++;
      }
    }
    //VLOG(1) << "  Created " << num_resids_created << " tag view residuals of " << num_tag_views;
    
    return true;
  }
  
  
  // Add Tag View Residuals to problem
  bool AddTagViewResidualsToProblem(anantak::CircularQueue<anantak::AprilTagViewResidual>& tvr_q,
      const anantak::FixedPointCQ& sp, ceres::Problem& problem, int32_t& n_tvr) {
    
    // Add all tag view residuals from fp onto the end of tvr_q
    n_tvr = 0;
    for (anantak::FixedPointCQ fp=sp;
         tvr_q.FixedPointIsBeforeLastElement(fp);
         tvr_q.IncrementFixedPoint(fp)) {
      
      anantak::AprilTagViewResidual* tvr = tvr_q.MutableElementAfterFixedPoint(fp);
      ceres::CostFunction* tvr_cf = tvr;
      problem.AddResidualBlock(
        tvr_cf, NULL,
        tvr->poseC_->error_,
        tvr->tagTj_->pose_.error_,
        &tvr->tagTj_->size_.error_,
        tvr->camera_->error_
      );
      n_tvr++;
      
      // Set tag size constant
      if (true) {
      problem.SetParameterBlockConstant(&tvr->tagTj_->size_.error_);
      }
      
      // Set camera intrinsics constant
      if (true) {
      problem.SetParameterBlockConstant(tvr->camera_->error_);
      }
      
    }
    
    return true;
  }
  
  
  // Build the problem
  bool BuildProblem() {
    
    // Checks
    if (!problem_) {LOG(ERROR) << "Problem object is null!"; return false;}
    
    // Add tag view residuals to the problem
    for (int i_cam=0; i_cam<num_cameras_; i_cam++) {
      if (tags_maps_idx_for_cam_[i_cam] == reference_cam_idx_) {
        
        // Get to the tvr just before the data is to be added to the problem
        VLOG(1) << "Cam" << i_cam << " tvr marker before inc to begin data = " << tvr_begin_marker_[i_cam].to_string();
        bool before_data_begin_ts = true;
        while (tag_view_residuals_[i_cam]->FixedPointIsBeforeLastElement(tvr_begin_marker_[i_cam])
               && before_data_begin_ts) {
          before_data_begin_ts = (swf_iterations_.data_begin_ts_ >
              tag_view_residuals_[i_cam]->MutableElementAfterFixedPoint(tvr_begin_marker_[i_cam])->timestamp_);
          if (before_data_begin_ts) tag_view_residuals_[i_cam]->IncrementFixedPoint(tvr_begin_marker_[i_cam]);
        }
        VLOG(1) << "Cam" << i_cam << " tvr marker before begin data = " << tvr_begin_marker_[i_cam].to_string();
        
        int32_t n_tvr = 0;
        AddTagViewResidualsToProblem(*tag_view_residuals_[i_cam], tvr_begin_marker_[i_cam],
            *problem_, n_tvr);
        VLOG(1) << "Cam" << i_cam << " Number of tv residuals added to new problem = " << n_tvr;
        
        tvr_end_marker_[i_cam] = tag_view_residuals_[i_cam]->CurrentFixedPoint();
        
      }   // cam is part of reference tags map
    }   // for each camera
    
    return true;
  }
  
  // Add new data to problem
  bool AddNewDataToProblem() {
    // Checks
    if (!problem_) {LOG(ERROR) << "Problem object is null!"; return false;}
    
    // Add tag view residuals to the problem
    for (int i_cam=0; i_cam<num_cameras_; i_cam++) {
      if (tags_maps_idx_for_cam_[i_cam] == reference_cam_idx_) {
        
        VLOG(1) << "Cam" << i_cam << " tvr end marker = " << tvr_end_marker_[i_cam].to_string();
        int32_t n_tvr = 0;
        AddTagViewResidualsToProblem(*tag_view_residuals_[i_cam], tvr_end_marker_[i_cam],
            *problem_, n_tvr);
        VLOG(1) << "Cam" << i_cam << " Number of new tv residuals added = " << n_tvr;
        
        tvr_end_marker_[i_cam] = tag_view_residuals_[i_cam]->CurrentFixedPoint();
        
      }   // cam is part of reference tags map
    }   // for each camera
    
    return true;
  }
  
  // Solve the problem
  bool SolveProblem() {
    // Checks
    if (!problem_) {LOG(ERROR) << "Problem is null!"; return false;}
    
    // Solve the problem
    ceres::Solve(solver_options_, problem_.get(), &solver_summary_);
    if (false) std::cout << solver_summary_.FullReport() << std::endl;
    
    return true;
  }
  
  // Find poses in a poses circular queue
  bool FindPoseAtOrAfterTimestamp(anantak::CircularQueue<anantak::Pose3dState>& pose_q,
      anantak::FixedPointCQ& p, const int64_t& ts) {
    // Checks
    if (pose_q.n_msgs() < 2) {
      LOG(ERROR)<<"pose_q.n_msgs() < 2, can not search. n = " << pose_q.n_msgs();
      return false;
    }
    
    // Start from the back, looking for poses with required timestamp
    anantak::FixedPointCQ fp = pose_q.CurrentFixedPoint();
    bool found_pose = false;
    while (pose_q.FixedPointIsAfterFirstElement(fp) && !found_pose) {
      found_pose = (pose_q.MutableElementAtFixedPoint(fp)->timestamp_ < ts);
      if (!found_pose) pose_q.DecrementFixedPoint(fp);
    }
    pose_q.IncrementFixedPoint(fp);
    
    if (!found_pose) {
      LOG(ERROR) << "Could not find pose AtOrAfterTimestamp " << pose_q.to_string();
      fp = pose_q.CurrentFixedPoint();
      bool found_pose = false;
      VLOG(1) << "   fp = " << fp.to_string() << " "
          << std::to_string(found_pose) << " "
          << std::to_string(pose_q.FixedPointIsAfterFirstElement(fp));
      while (pose_q.FixedPointIsAfterFirstElement(fp) && !found_pose) {
        found_pose = (pose_q.MutableElementAtFixedPoint(fp)->timestamp_ < ts);
        if (!found_pose) pose_q.DecrementFixedPoint(fp);
        VLOG(1) << "   fp = " << fp.to_string() << " "
            << std::to_string(found_pose) << " "
            << std::to_string(pose_q.FixedPointIsAfterFirstElement(fp));
      }
      return false;
    } else {
      p = fp;
    }
    
    return true;
  }
  
  // Set pose constant. Assumes that pose is added to the problem.
  // P0 is included, P1 is excluded
  bool SetPosesAsConstant(anantak::CircularQueue<anantak::Pose3dState>& pose_q,
      const anantak::FixedPointCQ& p0, const anantak::FixedPointCQ& p1,
      ceres::Problem& problem, int32_t& num) {
    
    num = 0;
    for (anantak::FixedPointCQ fp = p0;
        pose_q.FixedPointIsBeforeMarker(p1, fp);
        pose_q.IncrementFixedPoint(fp)) {
      
      // Set pose constant
      anantak::Pose3dState* pose = pose_q.MutableElementAtFixedPoint(fp);
      if (problem.HasParameterBlock(pose->error_)) {
        problem.SetParameterBlockConstant(pose->error_);        
        num++;
      }      
    }
    
    return true;
  }
  
  // Recalculate poses starting from a fixed point p0. P0 is included.
  bool RecalculatePoses(anantak::CircularQueue<anantak::Pose3dState>& pose_q,
      const anantak::FixedPointCQ& p0, int32_t& num) {
    num = 0;
    for (anantak::FixedPointCQ fp = p0; pose_q.NotPastTheEnd(fp); pose_q.Increment(fp)) {
      // Recalculate pose
      anantak::Pose3dState* pose = pose_q.MutableElementAtFixedPoint(fp);
      pose->Recalculate();
      num++;
    }
    return true;
  }
  
  // Mark new states as constant
  //  Assumes that the begin marker is where making constants should being
  bool MarkNewStatesConstant() {
    // Checks
    if (!problem_) {LOG(ERROR) << "Problem is null!"; return false;}
    
    // Set states constant in the problem
    for (int i_cam=0; i_cam<num_cameras_; i_cam++) {
      if (tags_maps_idx_for_cam_[i_cam] == reference_cam_idx_) {
        
        int64_t sliding_window_ts = swf_iterations_.sliding_window_ts_;
        if (sliding_window_ts < cp_data_begins_after_ts_[i_cam])
            sliding_window_ts = cp_data_begins_after_ts_[i_cam];
            
        if (!FindPoseAtOrAfterTimestamp(*camera_poses_[i_cam], cpc_end_marker_[i_cam],
            sliding_window_ts)) {
          LOG(ERROR) << "Cam" << i_cam << " Could not find cam pose for sliding_window_ts_";
          return false;
        }
        int32_t num_const_states = 0;
        if (!SetPosesAsConstant(*camera_poses_[i_cam], cpc_begin_marker_[i_cam],
            cpc_end_marker_[i_cam], *problem_, num_const_states)) {
          LOG(ERROR) << "Cam" << i_cam << " Could not set poses as constant";
          return false;
        } else {
          VLOG(1) << "Cam" << i_cam << " Set poses constant num = " << num_const_states
              << " from " << cpc_begin_marker_[i_cam].to_string()
              << " till before " << cpc_end_marker_[i_cam].to_string();
        }
        
        // Set the begin marker to end marker 
        cpc_begin_marker_[i_cam] = cpc_end_marker_[i_cam];
        
      }   // cam is part of reference tags map
    }   // for each camera
    
    return false;
  }
  
  // Mark constant states
  bool MarkStatesConstant() {
    // Checks
    if (!problem_) {LOG(ERROR) << "Problem is null!"; return false;}
    
    // Set states constant in the problem
    for (int i_cam=0; i_cam<num_cameras_; i_cam++) {
      if (tags_maps_idx_for_cam_[i_cam] == reference_cam_idx_) {
        
        int64_t data_begin_ts = swf_iterations_.data_begin_ts_;
        if (data_begin_ts < cp_data_begins_after_ts_[i_cam])
            data_begin_ts = cp_data_begins_after_ts_[i_cam];
        
        if (!FindPoseAtOrAfterTimestamp(*camera_poses_[i_cam], cpc_begin_marker_[i_cam],
            data_begin_ts)) {
          LOG(ERROR) << "Cam" << i_cam << " Could not find cam pose for data_begin_ts_";
          return false;
        } else {
          VLOG(1) << "Cam" << i_cam << " found data_begin_ts_ at " << cpc_begin_marker_[i_cam].to_string();
        }
        
      }   // cam is part of reference tags map
    }   // for each camera
    
    return MarkNewStatesConstant();
  }
  
  // Recalculate states after the problem has been solved
  bool RecalculateStates() {
    
    // Recalculate only states that have been solved
    for (int i_cam=0; i_cam<num_cameras_; i_cam++) {
      if (tags_maps_idx_for_cam_[i_cam] == reference_cam_idx_) {
        
        int32_t num_recalc = 0;
        if (!RecalculatePoses(*camera_poses_[i_cam], cpc_begin_marker_[i_cam], num_recalc)) {
          LOG(ERROR) << " Could not recalculate poses";
        } else {
          VLOG(1) << "Cam" << i_cam << " Recalculated poses num = " << num_recalc
              << " from " << cpc_begin_marker_[i_cam].to_string()
              << " to end " << camera_poses_[i_cam]->CurrentFixedPoint().to_string();
        }
        
      }   // cam is part of reference tags map
    }   // for each camera
    
    return true;
  }
  
  // Calculate priors from an existing problem
  bool CalculatePriors() {
    // Checks
    if (!problem_) {LOG(ERROR) << "Problem is null!"; return false;}
    
    // Tag pose priors only for reference map that uses filtering
    if (!tags_maps_[reference_cam_idx_]->CalculatePriors(problem_.get())) {
      LOG(ERROR) << "Could not calculate tag pose priors for the reference tag map";
      return false;
    }
    
    return true;
  }
  
  // Add priors to a problem
  bool AddPriors() {
    // Checks
    if (!problem_) {LOG(ERROR) << "Problem is null!"; return false;}
    if (!tags_maps_[reference_cam_idx_]) {LOG(ERROR) << "Reference tag map does not exist!"; return false;}
    
    // Add tag pose priors to the problem
    tags_maps_[reference_cam_idx_]->AddPriors(problem_.get(), true);  // also set origin tag constant
    
    return true;
  }
  
  
  // Process April tag messages
  bool ProcessAprilTagMessages(const int64_t iteration_end_ts,
      const std::vector<std::unique_ptr<std::vector<anantak::SensorMsg>>>& all_msgs) {
    
    // Check if camera instrinsic calibrations are set. Nothing can be done without that.
    if (!calibrations_are_set_) {
      LOG(ERROR) << "Can not process April tag messages till the camera intrinsics are set.";
      return false;
    }
    
    // Process tag views via tagmaps. Add cam poses for each camera.
    for (int i_cam=0; i_cam < num_cameras_; i_cam++) {
      
      // Get the camera messages from all_msgs
      uint16_t msgs_idx = cam_ds_[i_cam];
      if (all_msgs.size() < msgs_idx+1) {
        LOG(ERROR) << "all_msgs.size() < msgs_idx+1 " << all_msgs.size() << ", " << msgs_idx;
        return false;
      }
      const std::vector<anantak::SensorMsg>& tag_msgs = *all_msgs[msgs_idx];
      VLOG(2) << "Using data source idx for cam " << i_cam << " of " << msgs_idx << " has "
          << tag_msgs.size() << " messages";
      
      // Process the tag views 
      for (int i_msg=0; i_msg < tag_msgs.size(); i_msg++) {
        // Get the tags_map for the camera
        uint16_t tags_maps_idx = tags_maps_idx_for_cam_[i_cam];
        
        // Add the tags to the tag map for the camera
        anantak::Pose3dState cam_pose;
        const anantak::SensorMsg& tag_msg = tag_msgs[i_msg];
        
        // Estimate camera pose from the tags map
        //  New tags seen are added to the map and their pose is estimated
        //  If running filtering we do NOT reestimate existing tags here, filtering does estimation.
        //  If not filtering, simple information based method of tag pose estimation is used.
        bool success = false;
        if (tags_maps_idx == reference_cam_idx_) {
          success = tags_maps_[reference_cam_idx_]->ProcessTagMessage(tag_msg,
              camera_intrinsics_[i_cam], &cam_pose, !options_.run_filtering);
        } else {
          success = tags_maps_[tags_maps_idx]->ProcessTagMessage(tag_msg,
              camera_intrinsics_[i_cam], &cam_pose);
        }
        
        // Add camera pose to the camera poses queue
        if (success) {
          // Save the camera pose in the camera poses queue
          cam_pose.timestamp_ = tag_msg.header().timestamp();
          camera_poses_[i_cam]->add_element(cam_pose);
          
          // Set data start ts with first message. Start iterations.
          if (data_start_ts_==0) {
            data_start_ts_=cam_pose.timestamp_;
            swf_iterations_.StartFiltering(data_start_ts_);
          }
          
          // Create tag view residuals if this camera belongs to reference tags map
          //  Both tag poses and cam poses have been initiated
          if (tags_maps_idx == reference_cam_idx_) {
            anantak::Pose3dState* cam_pose_ptr = camera_poses_[i_cam]->mutable_element();
            // Use the current tag view readings to create residuals
            CreateTagViewResiduals(
                tags_maps_[reference_cam_idx_].get(),
                tag_view_residuals_[i_cam].get(),
                cam_pose_ptr,
                &camera_intrinsics_[i_cam]);
          }
          
        } // if successful in calculating camera pose in tag map
        
      }  // for i_msg
      VLOG(1) << "Cam" << i_cam << " Last cam pose point is " << camera_poses_[i_cam]->CurrentFixedPoint().to_string();
    }  // for i_cam
    
    // Set starting timestamps on starting dummy camera poses and states
    if (last_machine_state_ts_==0) {
      // This is the first data segement
      last_machine_state_ts_ = data_start_ts_;
      int64_t starting_ts = (last_machine_state_ts_ / inter_state_interval_) * inter_state_interval_;
      LOG(INFO) << "Starting ts = " << last_machine_state_ts_ << ", states start ts = " << starting_ts;
      for (int i_cam=0; i_cam < num_cameras_; i_cam++) {
        anantak::Pose3dState* cam_pose = camera_poses_[i_cam]->mutable_element();
        cam_pose->SetZero();
        cam_pose->SetTimestamp(starting_ts);
        anantak::Pose3dState* cam_state = camera_states_[i_cam]->mutable_element();
        cam_state->SetZero();
        cam_state->SetTimestamp(starting_ts);
      }
    }
    
    // Insert new camera states
    for (int i_cam=0; i_cam < num_cameras_; i_cam++) {
      const int64_t starting_ts = camera_states_[i_cam]->back().timestamp_;
      int64_t curr_ts = starting_ts + inter_state_interval_;
      while (curr_ts < iteration_end_ts) {
        // Add a pose with this timestamp to the queue
        anantak::Pose3dState* cam_state = camera_states_[i_cam]->next_mutable_element();
        cam_state->SetZero();
        cam_state->SetTimestamp(curr_ts);
        //VLOG(1) << "Cam" << i_cam << " created state for ts = " << curr_ts << " itern end ts = " << iteration_end_ts;
        curr_ts += inter_state_interval_;
      }
      //VLOG(1) << "Cam" << i_cam << " inserted new states to end at point "
      //    << camera_states_[i_cam]->CurrentFixedPoint().to_string();
    }
    
    // Update earliest estimated states and poses markers. If they are invalid, update to first state in the queue
    for (int i_cam=0; i_cam < num_cameras_; i_cam++) {
      if (!camera_states_[i_cam]->IsFixedPointValid(camera_states_marker_[i_cam])) {
        // Update the states marker to first element in the states queue.
        camera_states_marker_[i_cam] = camera_states_[i_cam]->FrontFixedPoint();
        VLOG(1) << "Cam" << i_cam << " updated earliest state estimated marker to "
            << camera_states_marker_[i_cam].to_string();
      }
      if (!camera_poses_[i_cam]->IsFixedPointValid(camera_poses_marker_[i_cam])) {
        // Update the poses marker to first element in the poses queue.
        camera_poses_marker_[i_cam] = camera_poses_[i_cam]->FrontFixedPoint();
        VLOG(1) << "Cam" << i_cam << " updated earliest poses marker to "
            << camera_poses_marker_[i_cam].to_string();
      }
    }
    
    // Estimate states by interpolating camera poses
    for (int i_cam=0; i_cam < num_cameras_; i_cam++) {
      
      // Set starting states markers for use later
      starting_states_marker_[i_cam] = camera_states_marker_[i_cam];
      
      // Only those states need to be interpolated that use the reference tag map
      if (tags_maps_idx_for_cam_[i_cam] == reference_cam_idx_) {
        
        // Cycle through any new camera poses that were estimated
        while (camera_poses_[i_cam]->FixedPointIsBeforeLastElement(camera_poses_marker_[i_cam])) {
          
          anantak::Pose3dState* s0 =
              camera_poses_[i_cam]->MutableElementAtFixedPoint(camera_poses_marker_[i_cam]);
          anantak::Pose3dState* s1 =
              camera_poses_[i_cam]->MutableElementAfterFixedPoint(camera_poses_marker_[i_cam]);
          anantak::Pose3dState* ss =
              camera_states_[i_cam]->MutableElementAtFixedPoint(camera_states_marker_[i_cam]);
          
          // Estimate all states in this interval
          while (ss->timestamp_<=s1->timestamp_ &&
              camera_states_[i_cam]->FixedPointIsBeforeLastElement(camera_states_marker_[i_cam])) {
            
            // If state lies in the interval, interpolate it
            if (s0->timestamp_<ss->timestamp_ && ss->timestamp_<=s1->timestamp_) {
              InterpolatePose3dUsingInformation(s0, s1, ss);
            }
            
            // Increment states marker and set ss to its state
            bool rc = camera_states_[i_cam]->IncrementFixedPoint(camera_states_marker_[i_cam]);
            ss = camera_states_[i_cam]->MutableElementAtFixedPoint(camera_states_marker_[i_cam]);
            
          }
          
          camera_poses_[i_cam]->IncrementFixedPoint(camera_poses_marker_[i_cam]);
          //VLOG(1) << "Cam" << i_cam << " incremented pose to point " << camera_poses_marker_[i_cam].to_string();
          
        } // go till the last interval
      }  // if this cam has reference tag map  
    } // for each cam
    
    // Calculate the cam-to-cam transforms
    if (true) {
    for (int i_cam=0; i_cam < num_cameras_; i_cam++) {
      // Only those states need to be interpolated that use the reference tag map but not the ref cam
      if (tags_maps_idx_for_cam_[i_cam]==reference_cam_idx_ && i_cam!=reference_cam_idx_) {
        //VLOG(1) << "Cam" << i_cam << " starting cam to ref pose = " << *camera_to_reference_poses_[i_cam];
        for (anantak::FixedPointCQ fp=starting_states_marker_[i_cam];
            camera_states_[i_cam]->FixedPointIsBeforeLastElement(fp);
            camera_states_[i_cam]->IncrementFixedPoint(fp)) {
          // Calculate camera to reference pose
          anantak::Pose3dState dpose;
          dpose.SetZero();
          DiffPose3dUsingInformation(
              camera_states_[reference_cam_idx_]->MutableElementAtFixedPoint(fp),
              camera_states_[i_cam]->MutableElementAtFixedPoint(fp),
              &dpose);
          //VLOG(1) << "Cam" << i_cam << "   dpose = " << dpose;
          // Update camera_to_reference_pose with above diff pose
          if (dpose.information_ > anantak::Epsilon) {
            UpdatePose3dUsingInformation(
              &dpose,
              camera_to_reference_poses_[i_cam].get()
            );
          }
        }
        VLOG(1) << "Cam" << i_cam << " updated cam to ref pose = " << *camera_to_reference_poses_[i_cam];
      }  // uses ref tag map but is not the ref cam
    }  // for each cam
    }
    
    // Update the reference cam states
    if (true) {
    for (int i_cam=0; i_cam < num_cameras_; i_cam++) {
      // Only those states need to be interpolated that use the reference tag map but not the ref cam
      if (tags_maps_idx_for_cam_[i_cam]==reference_cam_idx_ && i_cam!=reference_cam_idx_) {
        for (anantak::FixedPointCQ fp=starting_states_marker_[i_cam];
            camera_states_[i_cam]->FixedPointIsBeforeLastElement(fp);
            camera_states_[i_cam]->IncrementFixedPoint(fp)) {
          // Calculate reference camera pose
          anantak::Pose3dState ref_pose;
          ref_pose.SetZero();
          AddPose3dUsingInformation(
              camera_states_[i_cam]->MutableElementAtFixedPoint(fp),
              camera_to_reference_poses_[i_cam].get(),
              &ref_pose);
          //VLOG(1) << "Cam" << i_cam << "   ref_pose = " << ref_pose;
          // Update reference_pose with above pose
          if (ref_pose.information_ > anantak::Epsilon) {
            UpdatePose3dUsingInformation(
              &ref_pose,
              camera_states_[reference_cam_idx_]->MutableElementAtFixedPoint(fp)
            );
          } // info is meaningful
        } // for 
      }  // if uses ref tag map but is not the ref cam
    }  // for each cam
    }
    
    // Run filtering
    if (options_.run_filtering) {
    swf_iterations_.AddData(iteration_end_ts);
    
    // Reset problem
    if (!problem_ || swf_iterations_.reset_problem_) {
      
      // If the problem exists, destroy it first
      if (problem_) {
        
        VLOG(1) << "Calculating priors";
        CalculatePriors();
        
        VLOG(1) << "Resetting the problem.";
        problem_.reset();
      }
      
      // Create a new problem object
      std::unique_ptr<ceres::Problem> problem_ptr(new ceres::Problem(problem_options_));
      problem_ = std::move(problem_ptr);
      VLOG(1) << "Built a new problem object.";
      
      // Add priors
      AddPriors();
      
      // Build the problem
      BuildProblem();
      
      // Mark constant states
      MarkStatesConstant();
    }
    
    // Add this iteration's data to the problem
    VLOG(1) << "Adding new data to the problem.";
    AddNewDataToProblem();
    VLOG(1) << "Added new data to the problem.";
    
    // Solve the problem
    if (swf_iterations_.solve_problem_) {
      VLOG(1) << "Time to solve the problem.";
      SolveProblem();
      VLOG(1) << "Solved the problem.";
      
      // We never recalculate states after solving. Residual is not going to change!
      // Recalculate states after solving - this should not be done!
      //RecalculateStates(); - dont do this
      // Recalculate reference tag map
      //tags_maps_[reference_cam_idx_]->Recalculate(); - dont do this
      
      // Mark constant states
      MarkNewStatesConstant();
      
      // Calculate new solve begin ts
      swf_iterations_.SolveProblem();
    }
    } // run filtering?
    
    
    // Combine tag maps
    //  Check if tag maps can be combined, if so combine them, delete maps that are not needed.
    if (num_tag_maps_>1 &&
        last_map_combine_check_ts_+options_.check_maps_to_combine_interval <= iteration_end_ts) {
      last_map_combine_check_ts_ = iteration_end_ts;
      VLOG(2) << "Checking if tag maps can be combined";
      
      for (int i_cam=0; i_cam<num_cameras_; i_cam++) {
        if (i_cam!=reference_cam_idx_ && tags_maps_[i_cam]) {
          int32_t num_common_tags =
              tags_maps_[reference_cam_idx_]->FindNumberOfCommonTags(*tags_maps_[i_cam]);
          if (num_common_tags >= options_.num_common_tags_to_combine_maps) {
            LOG(INFO) << "Tag map for cam " << i_cam << " can be combined with reference as there are "
                << num_common_tags << " common tags";
            if (!CombineTagMaps(i_cam)) {
              LOG(ERROR) << "Could not combine tag maps for cam " << reference_cam_idx_
                  << " and cam " << i_cam;
              return false;
            }
          } // if number of common tags is greater than the number needed to combine
        } // if tag map is not yet combined
      } // for i_cam
      
    } // if there are still any tag maps left and it is time to check for combination
    
    return true;
  }   // ProcessAprilTagMessages
  
  // Combine tag maps
  bool CombineTagMaps(const int16_t map_idx) {
    
    VLOG(1) << "Combining tag map for camera " << map_idx << " with reference tag map";
    
    anantak::Pose3dState map_to_map_pose;
    tags_maps_[reference_cam_idx_]->CalculateMapToMapPose(*tags_maps_[map_idx], &map_to_map_pose);
    VLOG(1) << "Map to map transform = " << map_to_map_pose;
    
    // Modify tag poses
    if (!tags_maps_[reference_cam_idx_]->CombineMap(*tags_maps_[map_idx], map_to_map_pose)) {
      LOG(ERROR) << "Could not combine maps";
      return false;
    }
    
    // Modify cam poses
    for (int i=0; i<camera_poses_[map_idx]->n_msgs(); i++) {
      // Add the map pose to this one
      anantak::Pose3dState new_pose;
      new_pose.SetZero();
      if (!AddPose3dUsingInformation(&map_to_map_pose, &camera_poses_[map_idx]->at(i), &new_pose)) {
        LOG(ERROR) << "Could not add map pose to camera pose. Will continue forward.";
        // Nullify the camera pose
        camera_poses_[map_idx]->at(i).information_ = 0.;
      } else {
        // Modify the camera pose
        camera_poses_[map_idx]->at(i).LqvG_ = new_pose.LqvG_;
        camera_poses_[map_idx]->at(i).GpL_ = new_pose.GpL_;
      }
    }
    
    // Set the camera pose marker for the camera to the beginning of camera pose queue
    cp_data_begins_after_ts_[map_idx] = camera_poses_[map_idx]->back().timestamp_ + 1; // Added 1 musec
    cpc_begin_marker_[map_idx] = camera_poses_[map_idx]->FrontFixedPoint();
    cpc_end_marker_[map_idx] = camera_poses_[map_idx]->FrontFixedPoint();
    
    // Book keeping and delete older tags map
    tags_maps_idx_for_cam_[map_idx] = reference_cam_idx_;
    tags_maps_[map_idx].reset(nullptr);
    num_tag_maps_--;
    
    return true;
  }
  
  // Recalculate maps, poses, states
  bool RecalculateCameraPoses() {
    for (int i_cam=0; i_cam<num_cameras_; i_cam++) {
      if (camera_poses_[i_cam]) {
        int32_t num_poses = camera_poses_[i_cam]->n_msgs();
        for (int i=0; i<num_poses; i++) {
          camera_poses_[i_cam]->at(i).Recalculate();
        }
      }
    }    
    return true;
  }
  
  bool RecalculateTagMaps() {
    for (int i=0; i<num_cameras_; i++) {
      if (tags_maps_[i]) {
        tags_maps_[i]->Recalculate();
      }
    }    
    return true;
  }
  
  // Save maps, poses, states to file
  bool SaveTagMaps(const std::string& save_filename, const std::string& predicate) {
    for (int i=0; i<num_cameras_; i++) {
      if (tags_maps_[i]) {
        tags_maps_[i]->SaveTagMapToFile(save_filename, "Cam"+std::to_string(i));
      }
    }
    return true;
  }
  
  bool SaveCameraPoses(const std::string& save_filename, const std::string& predicate) {
    for (int i_cam=0; i_cam<num_cameras_; i_cam++) {
      if (camera_poses_[i_cam]) {
        Eigen::Matrix<double,7,Eigen::Dynamic> cam_poses;
        int32_t num_poses = camera_poses_[i_cam]->n_msgs();
        cam_poses.resize(7, num_poses);
        for (int i=0; i<num_poses; i++) {
          Eigen::Map<Eigen::Matrix<double,7,1>> cam_poses_map(camera_poses_[i_cam]->at(i).state_);
          cam_poses.col(i) = cam_poses_map;
        }
        anantak::WriteMatrixToCSVFile(save_filename+".Cam"+std::to_string(i_cam)+".camposes",
            cam_poses.transpose());
      }
    }
    return true;
  }

  bool SaveCameraStates(const std::string& save_filename, const std::string& predicate) {
    for (int i_cam=0; i_cam<num_cameras_; i_cam++) {
      if (camera_states_[i_cam]) {
        Eigen::Matrix<double,7,Eigen::Dynamic> cam_states;
        // Get the number of estimated states. Neglect un-estimated states at the end.
        int32_t num_states = camera_states_[i_cam]->n_msgs()
            - camera_states_[i_cam]->NumElementsTillEnd(camera_states_marker_[i_cam]);
        cam_states.resize(7, num_states);
        for (int i=0; i<num_states; i++) {
          Eigen::Map<Eigen::Matrix<double,7,1>> cam_states_map(camera_states_[i_cam]->at(i).state_);
          cam_states.col(i) = cam_states_map;
        }
        anantak::WriteMatrixToCSVFile(save_filename+".Cam"+std::to_string(i_cam)+".camstates",
            cam_states.transpose());
      }
    }
    return true;
  }
  
  bool SaveCamToCamPoses(const std::string& save_filename, const std::string& predicate) {
    Eigen::Matrix<double,7,Eigen::Dynamic> camcam_poses;
    camcam_poses.resize(7, num_cameras_);
    for (int i_cam=0; i_cam<num_cameras_; i_cam++) {
      anantak::Pose3dState inv_pose;
      inv_pose.SetZero();
      InvertPose3dUsingInformation(camera_to_reference_poses_[i_cam].get(), &inv_pose);
      Eigen::Map<Eigen::Matrix<double,7,1>> camcam_pose_map(inv_pose.state_);
      camcam_poses.col(i_cam) = camcam_pose_map;
    }
    anantak::WriteMatrixToCSVFile(save_filename+".camtocampose",
        camcam_poses.transpose());
    return true;
  }
  
};  // TagCamerasCalibrator




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
  anantak::FileMessagesKeeper calibrations_keeper(camera_mono_calib_msgs_filenames, false);
  if (!calibrations_keeper.LoadAllMsgsFromFiles()) {
    LOG(ERROR) << "Some error in loading calibrations.";
  }
  std::vector<std::unique_ptr<std::vector<anantak::SensorMsg>>> calib_msgs;
  calibrations_keeper.AllocateMemoryForNewMessages(1, &calib_msgs);
  calibrations_keeper.FetchLastMessages(&calib_msgs);

  /* Load data into a file data keeper. This gives the facility to get mesages incrementaly */
  std::vector<std::string> msgs_filenames {
    "src/test/imu1_data.pb.data",
    "src/test/cam1_apriltags.pb.data",
    "src/test/cam2_apriltags.pb.data",
    "src/test/cam3_apriltags.pb.data",
    "src/test/cam4_apriltags.pb.data",
  };
  std::vector<uint16_t> cam_datasource_map {1,2,3,4}; // index=cam_num_, val=datasource_num_
  std::vector<uint16_t> imu_datasource_map {0}; // firstval=imu_ds_num, second_val=ref_cam_ds_num
  
  // Open msg files
  anantak::FileMessagesKeeper file_msgs_keeper(msgs_filenames, false);
  if (!file_msgs_keeper.LoadAllMsgsFromFiles()) {
    LOG(ERROR) << "Could not load data, exiting.";
    return -1;
  }
  
  int64_t iteration_interval = 500000; // microsec
  
  // Setup memory to get new messages
  std::vector<std::unique_ptr<std::vector<anantak::SensorMsg>>> new_msgs;
  file_msgs_keeper.AllocateMemoryForNewMessages(500, &new_msgs);

  
  // Setup TagCamerasCalibrator
  TagCamerasCalibrator::Options tag_cam_calib_options;
  TagCamerasCalibrator tag_cam_calib(tag_cam_calib_options, cam_datasource_map);
  if (!tag_cam_calib.ProcessCameraCalibrationMessages(calib_msgs)) {
    LOG(ERROR) << "Could not process Camera calibration messages.";
  }
  
  for (int i_iter=0; i_iter<600; i_iter++) {    // ~670 max
    file_msgs_keeper.FetchNewMessages(iteration_interval, &new_msgs);
    // report the number of messages recieved
    std::cout << "iteration " << i_iter << ": Messages ";
    for (int i_ds=0; i_ds<msgs_filenames.size(); i_ds++) std::cout << new_msgs[i_ds]->size() << " ";
    std::cout << "\n";
    
    // Send data to tag cameras calibrator
    tag_cam_calib.ProcessAprilTagMessages(file_msgs_keeper.CurrentDataTime(), new_msgs);
    
  }
  
  // Recalculate maps, poses, states
  tag_cam_calib.RecalculateTagMaps();
  tag_cam_calib.RecalculateCameraPoses();
  // Save maps, poses, states to files
  tag_cam_calib.SaveTagMaps(plots_dir+"TagCamCalib", "");
  tag_cam_calib.SaveCameraPoses(plots_dir+"TagCamCalib", "");
  tag_cam_calib.SaveCameraStates(plots_dir+"TagCamCalib", "");
  tag_cam_calib.SaveCamToCamPoses(plots_dir+"TagCamCalib", "");
  
}
