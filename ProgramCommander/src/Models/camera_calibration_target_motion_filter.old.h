/** Calibration target motion filter
 *
 *  Calculates target kinematic motion using april tag observations. As target does not transmit
 *  any acceleration data, accelerations are unobservable and only limited by the assumed variance.
 */

// Anantak includes
#include "Filter/timed_circular_queue.h"
#include "Filter/observation.h"
#include "Models/model.h"
#include "ModelsLib1.h"


namespace anantak {


/** Camera calibration target motion filter
 *
 *  Filtering proceeds as follows:
 *    Filter starts with a given current timestamp or past timestamp.
 *    In every iteration it creates new states, predicts the states, creates inter-state residuals,
 *    creates observation residuals, (it it is time) runs optimization, transmits results, and
 *    decides if the fiter continues to run.
 *  
 */

class CameraCalibrationTargetMotionFilter : public Model {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  struct Options {
    
    // Frequency of running 
    uint64_t iteration_interval;
    
    // Filtering options
    uint64_t states_history_interval;     // Past states are kept for this long interval
    uint64_t longest_problem_interval;    // Problem will be built to this longest length
    uint64_t shortest_problem_interval;   // Problem will be built to this length at a minimum
    uint64_t sliding_window_interval;     // This is the length of time in which states will be solved
    uint64_t solving_problem_interval;    // Problem will be solved after every this interval
    uint32_t states_frequency;            // Frequency (Hz) at which machine states will be created
    anantak::SlidingWindowFilterIterations::Options sliding_window_options;
    
    // Camera specs
    std::vector<uint32_t> tag_camera_ids; // IDs of tag cameras to be processed
    uint16_t max_tag_camera_frequency;    // Maximum rate of images from tag cameras (Hz)
    uint16_t max_tags_per_image;          // Maximum number of images seen in an image
    
    // Calibration target specifications
    std::string calibration_target_config_file;
    
    // Kinematic model fit error specification
    //  Process noise inherent in constant acceleration assumption
    double edw_sigma;
    double edv_sigma;
    
    Options(const std::string& config_filename=""):
      iteration_interval(         1000000),
      
      states_history_interval(   60000000),
      longest_problem_interval(  30000000),
      shortest_problem_interval( 15000000),
      sliding_window_interval(    2000000),
      solving_problem_interval(    100000),
      states_frequency(                10),
      
      sliding_window_options(
        longest_problem_interval,     // longest problem length
        shortest_problem_interval,    // shortest problem length
        sliding_window_interval,      // sliding window length
        solving_problem_interval      // solving problem interval
      ),
      
      tag_camera_ids({1}),
      max_tag_camera_frequency(30),
      max_tags_per_image(35),
      
      calibration_target_config_file("config/camera_intrisics_apriltag_calibration_target.cfg"),
      
      edw_sigma(10.*kRadiansPerDegree),   // rad/s^2 
      edv_sigma(1.0)                      // m/s^2
    {
      LOG(INFO) << "Created CameraCalibrationTargetMotionFilter Options";
    }
    
  };
  
  CameraCalibrationTargetMotionFilter::Options options_;
  
  // Running mode
  enum FilterRunMode {kInitializing, kFiltering};
  FilterRunMode filter_run_mode_;
  
  // Calibration target
  CameraCalibrationTarget target_;
  
  // Camera
  CameraState camera_;
  UndistortedCamera undistorted_camera_;
  
  // Iteration interval. Filter runs at a higher frequency when initializing, then it drops.
  uint64_t iteration_interval_;
  int64_t  filter_start_ts_;
  uint64_t inter_state_interval_;
  double   inter_state_interval_sec_;
  
  // Sliding window filter objects
  anantak::SlidingWindowFilterIterations swf_iterations_;
  anantak::IterationRecord iteration_record_;
  
  // Target poses
  std::unique_ptr<anantak::TimedCircularQueue<anantak::KinematicState>>
      target_poses_;
  
  // Inter-kinematic state residuals
  InterKinematicStateResidual::Options inter_target_pose_residuals_options_;
  std::unique_ptr<anantak::TimedCircularQueue<anantak::InterKinematicStateResidual>>
      inter_target_pose_residuals_;
  
  // Target view residuals
  KinematicCalibrationTargetViewResidual::Options tag_view_residuals_options_;
  std::unique_ptr<anantak::TimedCircularQueue<anantak::KinematicCalibrationTargetViewResidual>>
      tag_view_residuals_;
  
  // Target pose observations interpolator
  anantak::CubicPoseInterpolator target_pose_observations_interpolator_;
  
  // Kinematic state prior
  KinematicStatePrior kinematic_state_prior_;
  
  // Pose prior residuals
  std::unique_ptr<anantak::TimedCircularQueue<anantak::KinematicStatePrior>>
      tag_pose_priors_;
  
  // Linear pose spline of target observations
  std::unique_ptr<anantak::NonUniformLinearPoseSpline> linear_observations_spline_;
  
  // Cubic pose B-spline
  std::unique_ptr<anantak::UniformCubicPoseSpline> cubic_pose_spline_;
  anantak::PoseSpline_Options cubic_pose_spline_options_;
  
  // Spline pose residuals queue
  std::unique_ptr<anantak::TimedCircularQueue<anantak::PoseSpline_PoseResidual>>
      spline_pose_residuals_;
  
  // Spline control poses priors queue
  std::unique_ptr<anantak::TimedCircularQueue<anantak::PoseSpline_ControlPosePrior>>
      spline_pose_priors_;
  
  // Optimization objects
  std::unique_ptr<ceres::Problem> problem_;
  ceres::Problem::Options problem_options_;
  ceres::Solver::Options solver_options_;
  ceres::Solver::Summary solver_summary_;
  
  // Initializing target poses
  std::vector<PoseState> starting_poses_;
  PoseState last_seen_tgt_pose_;
  
  CameraCalibrationTargetMotionFilter(
      const CameraCalibrationTargetMotionFilter::Options& options,
      const int64_t& filter_start_ts):
    options_(options), iteration_interval_(options.iteration_interval),
    filter_start_ts_(filter_start_ts),
    iteration_record_(),
    swf_iterations_(options.sliding_window_options),
    target_(options.calibration_target_config_file),
    camera_(options.tag_camera_ids.at(0)),              // we are initiating at first camera
    undistorted_camera_(options.tag_camera_ids.at(0)),  // we are initiating at first camera 
    inter_target_pose_residuals_options_(options.edw_sigma, options.edv_sigma),
    tag_view_residuals_options_(options.edw_sigma, options.edv_sigma),
    kinematic_state_prior_(),
    problem_(nullptr), problem_options_(), solver_options_(), solver_summary_(),
    target_pose_observations_interpolator_(200, 2.*kRadiansPerDegree, 0.010, 1./15, options.sliding_window_interval),
    cubic_pose_spline_options_(2.*kRadiansPerDegree, 0.05, 100)
  {
    Initialize();
  }
  
  /*bool Initialize00() {
    inter_state_interval_ = 1000000 / options_.states_frequency;
    inter_state_interval_sec_ = double(inter_state_interval_)*1e-6;
    LOG(INFO) << "  Filter staring timestamp = " << filter_start_ts_;
    LOG(INFO) << "  Filter iteration interval = " << float(iteration_interval_)*1e-6 << "sec";
    LOG(INFO) << "  Filter inter-state interval = " << inter_state_interval_ << "usec";
    
    filter_run_mode_ = kInitializing;
    LOG(INFO) << "  Filter is in initializing state";
    
    // Check if the calibration target was initialized
    if (!target_.IsInitialized()) {
      LOG(ERROR) << "Calibration target was not initialized. Quit.";
      return false;
    }
    
    // Load camera from file
    std::string camera_calibration_filename =
        "data/camera"+std::to_string(options_.tag_camera_ids.at(0))+"_state.pb.data";
    if (!camera_.LoadFromFile(camera_calibration_filename)) {
      LOG(FATAL) << "Could not load camera";
      return false;
    }
    LOG(INFO) << "Loaded camera: " << camera_.ToString(1);
    undistorted_camera_.Create(camera_);
    LOG(INFO) << "Created undistorted camera";
    
    // Add counters
    iteration_record_.iteration_counters["TargetPosesCreated"] = 0;   // Number of target poses created
    
    iteration_record_.iteration_counters["InterTargetResidualsCreated"] = 0; // Number of residuals created
    iteration_record_.iteration_counters["InterTargetResidualsAdded"] = 0;   // Number of residuals added to problem
    iteration_record_.iteration_counters["InterTargetResidualsMarked"] = 0;  // Number of residuals marked constant
    //iteration_record_.iteration_counters["TargetStatesUpdated"] = 0;         // Number of states updated
    
    iteration_record_.iteration_counters["TagViewResidualsCreated"] = 0; // Number of tag view residuals created
    iteration_record_.iteration_counters["TagViewResidualsAdded"] = 0;   // Number of tag view residuals added to problem
    iteration_record_.iteration_counters["TagViewResidualsMarked"] = 0;  // Number of tag view residuals marked constant
    iteration_record_.iteration_counters["TagViewResidualsKinReset"] = 0;  // Number of tag view residuals marked constant
    
    iteration_record_.algorithm_counters["ProblemObjects"] = 0;   // Number of problem object created
    
    iteration_record_.iteration_counters["TargetPosePriorsCreated"] = 0;
    iteration_record_.iteration_counters["TargetPosePriorsAdded"] = 0;
    iteration_record_.iteration_counters["TargetPosePriorsMarked"] = 0;
    
    // Set optimization options
    problem_options_.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    problem_options_.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    solver_options_.max_num_iterations = 300;
    solver_options_.minimizer_progress_to_stdout = false;
    solver_options_.logging_type = ceres::SILENT;
    
    // Allocation memory for the queues
    AllocateMemory();
    
    return true;
  }*/

  bool Initialize() {
    inter_state_interval_ = 1000000 / options_.states_frequency;
    inter_state_interval_sec_ = double(inter_state_interval_)*1e-6;
    LOG(INFO) << "  Filter staring timestamp = " << filter_start_ts_;
    LOG(INFO) << "  Filter iteration interval = " << float(iteration_interval_)*1e-6 << "sec";
    LOG(INFO) << "  Filter inter-state interval = " << inter_state_interval_ << "usec";
    
    filter_run_mode_ = kInitializing;
    LOG(INFO) << "  Filter is in initializing state";
    
    // Check if the calibration target was initialized
    if (!target_.IsInitialized()) {
      LOG(ERROR) << "Calibration target was not initialized. Quit.";
      return false;
    }
    
    // Load camera from file
    std::string camera_calibration_filename =
        "data/camera"+std::to_string(options_.tag_camera_ids.at(0))+"_state.pb.data";
    if (!camera_.LoadFromFile(camera_calibration_filename)) {
      LOG(FATAL) << "Could not load camera";
      return false;
    }
    LOG(INFO) << "Loaded camera: " << camera_.ToString(1);
    undistorted_camera_.Create(camera_);
    LOG(INFO) << "Created undistorted camera";
    
    // Set optimization options
    problem_options_.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    problem_options_.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    solver_options_.max_num_iterations = 300;
    solver_options_.minimizer_progress_to_stdout = false;
    solver_options_.logging_type = ceres::SILENT;
    
    // Add counters
    iteration_record_.iteration_counters["TargetObservations"] = 0;   // Number of target observations created
    
    iteration_record_.iteration_counters["SplinePoseResidualsCreated"] = 0; // Number of residuals created
    iteration_record_.iteration_counters["SplinePoseResidualsAdded"] = 0;   // Number of residuals added to problem
    iteration_record_.iteration_counters["SplinePoseResidualsMarked"] = 0;  // Number of residuals marked constant
    
    iteration_record_.iteration_counters["SplinePosePriorsCreated"] = 0;
    iteration_record_.iteration_counters["SplinePosePriorsAdded"] = 0;
    iteration_record_.iteration_counters["SplinePosePriorsMarked"] = 0;
    
    iteration_record_.algorithm_counters["ProblemObjects"] = 0;   // Number of problem object created
    
    // Allocation memory for the queues
    AllocateMemory();
    
    return true;
  }
  
  
  /* Allocate memory for queues */
  /*bool AllocateMemory() {
    LOG(INFO) << "Allocating memory for the camera intrinsics filter queues";
    
    int32_t num_states_to_keep = options_.states_frequency * options_.states_history_interval / 1000000;
    LOG(INFO) << "  num_states_to_keep = " << num_states_to_keep
        << " at states_frequency = " << options_.states_frequency << " Hz";
    
    // Target poses
    std::unique_ptr<anantak::TimedCircularQueue<anantak::KinematicState>> tgt_poses_ptr(
        new anantak::TimedCircularQueue<anantak::KinematicState>(num_states_to_keep));
    target_poses_ = std::move(tgt_poses_ptr);
    
    // Inter-kinematic state residuals
    anantak::InterKinematicStateResidual _prototype(&inter_target_pose_residuals_options_);
    std::unique_ptr<anantak::TimedCircularQueue<anantak::InterKinematicStateResidual>> inter_kin_queue_ptr(
        new anantak::TimedCircularQueue<anantak::InterKinematicStateResidual>(num_states_to_keep, _prototype));
    inter_target_pose_residuals_ = std::move(inter_kin_queue_ptr);
    
    // Tag view residuals
    int32_t num_tag_views_to_keep = options_.max_tag_camera_frequency * options_.states_history_interval / 1000000;
    LOG(INFO) << "Initiating tag view residuals queues with length = " << num_tag_views_to_keep;
    
    anantak::KinematicCalibrationTargetViewResidual _view_prototype(&tag_view_residuals_options_);
    std::unique_ptr<anantak::TimedCircularQueue<anantak::KinematicCalibrationTargetViewResidual>> tag_view_queue_ptr(
        new anantak::TimedCircularQueue<anantak::KinematicCalibrationTargetViewResidual>(num_tag_views_to_keep, _view_prototype));
    tag_view_residuals_ = std::move(tag_view_queue_ptr);
    
    // Tag pose priors - TODO - how many to keep?
    std::unique_ptr<anantak::TimedCircularQueue<anantak::KinematicStatePrior>> tag_pose_priors_ptr(
        new anantak::TimedCircularQueue<anantak::KinematicStatePrior>(num_tag_views_to_keep));
    tag_pose_priors_ = std::move(tag_pose_priors_ptr);
    
    return true;
  }*/

  bool AllocateMemory() {
    LOG(INFO) << "Allocating memory for the camera intrinsics filter queues";
    
    int32_t num_states_to_keep = options_.states_frequency * options_.states_history_interval / 1000000;
    LOG(INFO) << "  num_states_to_keep = " << num_states_to_keep
        << " at states_frequency = " << options_.states_frequency << " Hz";
    
    // Cuboc pose spline object
    std::unique_ptr<anantak::UniformCubicPoseSpline> cubic_pose_spline_ptr(
        new anantak::UniformCubicPoseSpline(num_states_to_keep, inter_state_interval_));
    cubic_pose_spline_ = std::move(cubic_pose_spline_ptr);
    
    // Linear observations spline object
    std::unique_ptr<anantak::NonUniformLinearPoseSpline> linear_pose_spline_ptr(
        new anantak::NonUniformLinearPoseSpline(num_states_to_keep));
    linear_observations_spline_ = std::move(linear_pose_spline_ptr);
    
    // Spline pose residuals
    anantak::PoseSpline_PoseResidual _pose_resid_proto(&cubic_pose_spline_options_);
    std::unique_ptr<anantak::TimedCircularQueue<anantak::PoseSpline_PoseResidual>> spr_ptr(
        new anantak::TimedCircularQueue<anantak::PoseSpline_PoseResidual>(
            num_states_to_keep, _pose_resid_proto));
    spline_pose_residuals_ = std::move(spr_ptr);
    
    // Spline pose priors
    anantak::PoseSpline_ControlPosePrior _pose_prior_proto(&cubic_pose_spline_options_);
    std::unique_ptr<anantak::TimedCircularQueue<anantak::PoseSpline_ControlPosePrior>> spp_ptr(
        new anantak::TimedCircularQueue<anantak::PoseSpline_ControlPosePrior>(
            num_states_to_keep, _pose_prior_proto));
    spline_pose_priors_ = std::move(spp_ptr);
    
    return true;
  }
  
  /** Accessors */
  const uint64_t& IterationInterval() {return iteration_interval_;}
  
  /** Initializing operations */
  
  bool RunInitializingIteration(const int64_t& iteration_end_ts) {
    // Nothing to do! Just wait for the data to come in.
    return true;
  }  
  
  // Run iteration with historical readings read using FileKeeper
  /*bool RunInitializingIteration(const int64_t& iteration_end_ts,
      const std::vector<std::unique_ptr<std::vector<anantak::SensorMsg>>>& observations) {
    
    // Collect all AprilTag observations
    std::vector<anantak::SensorMsg> sensor_msgs;
    for (int i_file=0; i_file<observations.size(); i_file++) {
      for (int i_msg=0; i_msg<observations.at(i_file)->size(); i_msg++) {
        const anantak::SensorMsg& msg = observations.at(i_file)->at(i_msg);
        if (msg.has_header()) {
          if (msg.header().type() == "AprilTags") {
            sensor_msgs.emplace_back(msg);
          }
        }
      }
    }
    
    if (sensor_msgs.size() == 0) {
      // Nothing to do. Wait for observations.
      return true;
    }
    VLOG(1) << "Got " << sensor_msgs.size() << " AprilTag messages.";
    
    // Make a starting guess of the target pose from the first (few) message(s)
    for (int i=0; i<sensor_msgs.size(); i++) {
      PoseState starting_pose;
      const int64_t& apriltag_msg_ts = sensor_msgs.at(i).header().timestamp();
      const AprilTagMessage& apriltag_msg = sensor_msgs.at(i).april_msg();
      anantak::AprilTagMessage undist_apriltag_msg;
      undistorted_camera_.Undistort(apriltag_msg, &undist_apriltag_msg);
      target_.CalculateTargetPose(undist_apriltag_msg, undistorted_camera_.camera_, &starting_pose);
      if (!starting_pose.IsZero()) {
        starting_pose.SetTimestamp(apriltag_msg_ts);
        starting_poses_.emplace_back(starting_pose);
      }
      if (starting_poses_.size()>=2) break;
    }
    
    if (starting_poses_.size()>=2) {
      // If we have the starting pose, create the starting states and start filtering
      //  This would fail if starting pose estimate is poor. May be use an average of few.
      if (SetStartingStates()) {
        filter_run_mode_ = kFiltering;
        LOG(INFO) << "Switched filter from initializing to filtering.";
        return RunIteration(iteration_end_ts, observations);
        //return true;
      } else {
        starting_poses_.clear();
      }
    }
    
    // Make a starting guess of the target pose from the first (few) message(s)
    //PoseState starting_pose;
    //int starting_pose_idx = 0;
    //while (starting_pose.IsZero() && starting_pose_idx<sensor_msgs.size()) {
    //  const int64_t& apriltag_msg_ts = sensor_msgs.at(starting_pose_idx).header().timestamp();
    //  const AprilTagMessage& apriltag_msg = sensor_msgs.at(starting_pose_idx).april_msg();
    //  starting_pose.SetZero();
    //  //target_.CalculateTargetPose(apriltag_msg, camera_, &starting_pose);
    //  anantak::AprilTagMessage undist_apriltag_msg;
    //  undistorted_camera_.Undistort(apriltag_msg, &undist_apriltag_msg);
    //  target_.CalculateTargetPose(undist_apriltag_msg, undistorted_camera_.camera_, &starting_pose);
    //  if (starting_pose.IsZero()) {
    //    starting_pose_idx++;
    //  } else {        
    //    // If we have the starting pose, create the starting states and start filtering
    //    //  This would fail if starting pose estimate is poor. May be an average of few be used.
    //    SetStartingStates(apriltag_msg_ts, starting_pose);
    //    filter_run_mode_ = kFiltering;
    //    LOG(INFO) << "Switched filter from initializing to filtering.";
    //    return RunIteration(iteration_end_ts, observations);
    //  }
    //}
    
    return true;
  }*/
  
  bool RunInitializingIteration(const int64_t& iteration_end_ts,
      const std::vector<std::unique_ptr<std::vector<anantak::SensorMsg>>>& observations) {
    
    // Collect all AprilTag observations
    std::vector<anantak::SensorMsg> sensor_msgs;
    for (int i_file=0; i_file<observations.size(); i_file++) {
      for (int i_msg=0; i_msg<observations.at(i_file)->size(); i_msg++) {
        const anantak::SensorMsg& msg = observations.at(i_file)->at(i_msg);
        if (msg.has_header()) {
          if (msg.header().type() == "AprilTags") {
            sensor_msgs.emplace_back(msg);
          }
        }
      }
    }
    
    if (sensor_msgs.size() == 0) {
      // Nothing to do. Wait for observations.
      return true;
    }
    VLOG(1) << "Got " << sensor_msgs.size() << " AprilTag messages.";
    
    // Make a starting guess of the target pose from the first (few) message(s)
    for (int i=0; i<sensor_msgs.size(); i++) {
      PoseState starting_pose;
      const int64_t& apriltag_msg_ts = sensor_msgs.at(i).header().timestamp();
      const AprilTagMessage& apriltag_msg = sensor_msgs.at(i).april_msg();
      anantak::AprilTagMessage undist_apriltag_msg;
      undistorted_camera_.Undistort(apriltag_msg, &undist_apriltag_msg);
      target_.CalculateTargetPose(undist_apriltag_msg, undistorted_camera_.camera_, &starting_pose);
      if (!starting_pose.IsZero()) {
        starting_pose.SetTimestamp(apriltag_msg_ts);
        starting_poses_.emplace_back(starting_pose);
      }
      if (starting_poses_.size()>=2) break;
    }
    
    if (starting_poses_.size()>=2) {
      // Initiate the spline using first two poses
      if (!cubic_pose_spline_->InitiateSpline(starting_poses_.at(0), starting_poses_.at(1))) {
        LOG(ERROR) << "Could not initiate spline using first two poses. Restating";
        starting_poses_.clear();
        cubic_pose_spline_->Reset();
        
      } else {
        
        if (!SetStartingStatesAndResiduals()) {
          LOG(FATAL) << "Could not set starting states and residuals";
        }
        
        // Switch filter to filtering mode
        filter_run_mode_ = kFiltering;
        LOG(INFO) << "Switched filter from initializing to filtering.";
        return RunIteration(iteration_end_ts, observations);
      }
    }
    
    return true;
  }
  
  bool SetStartingStatesAndResiduals() {
    
    // Calculate the starting state timestamp
    int64_t starting_state_ts = cubic_pose_spline_->StartingTimestamp();
    VLOG(1) << "Using starting state ts = " << starting_state_ts;
    
    // Start filtering at starting_state_ts
    swf_iterations_.StartFiltering(starting_state_ts);
    iteration_record_.Reset(starting_state_ts);
    
    // Set markers on queues
    //target_poses_->SetFixedPoint("TagViewResidual"); // Marks the last tag view residual location
    //target_poses_->SetFixedPoint("ProblemBeginMarker"); // Marks the last tag view residual location
    //VLOG(1) << "Starting target pose at target_poses_ queue idx: "
    //    << target_poses_->FixedPointToString("TagViewResidual");
    
    // Residuals queues have three markers:
    //    PreProblemBegin: at creation of new problem object, residuals are added from here
    //    ProblemEnd: at every iteration, residuals are added from here to the end of queue
    //    PreWindowBegin: at every iteration, states till this residual are marked constant
    
    // Prepare the spline_pose_residuals_ queue for usage
    spline_pose_residuals_->SetStartingTimestamp(starting_state_ts-1);
    spline_pose_residuals_->SetFixedPoint("PreProblemBegin");
    spline_pose_residuals_->SetFixedPoint("ProblemEnd");
    spline_pose_residuals_->SetFixedPoint("PreWindowBegin");
    
    // Prepare the spline_pose_priors_ queue for usage
    spline_pose_priors_->SetStartingTimestamp(starting_state_ts-1);
    spline_pose_priors_->SetFixedPoint("PreProblemBegin");
    spline_pose_priors_->SetFixedPoint("ProblemEnd");
    spline_pose_priors_->SetFixedPoint("PreWindowBegin");
    
    return true;
  }
  
  /* Estimate starting poses */
  /*bool SetStartingStates() {
    if (starting_poses_.size() < 2) {LOG(ERROR) << "starting_poses_.size() < 2"; return false;}
    
    // Calculate the starting state timestamp
    int64_t starting_ts = starting_poses_.at(0).Timestamp();
    if (starting_ts == 0) {LOG(ERROR) << "starting_ts == 0"; return false;}
    int64_t starting_state_ts = (starting_ts / inter_state_interval_) * inter_state_interval_;
    VLOG(1) << "Using starting state ts = " << starting_state_ts;
    
    // Start filtering at starting_state_ts
    swf_iterations_.StartFiltering(starting_state_ts);
    iteration_record_.Reset(starting_state_ts);
    
    // Create a starting kinematic state using starting poses. Add it to target state queues
    //  Assumes that starting acceleration is zero.
    //  Time delay of camera is kept at zero as the camera is the reference itself.
    anantak::KinematicState starting_state;   // State is zero
    starting_state.SetTimestamp(starting_ts);
    starting_state.SetTime(double(starting_ts-starting_state_ts)*1e-6);
    starting_state.SetPose(starting_poses_.at(0));
    starting_state.SetVelocity(starting_poses_.at(1));
    //starting_state.ShiftSmallTime(starting_state_ts - starting_ts);
    anantak::KinematicState pre_starting_state;   // State is zero
    MoveInFixedTime(starting_state, starting_state_ts - starting_ts, &pre_starting_state);
    anantak::Vector7d starting_pose_sigmas;
    starting_pose_sigmas << 200.*kRadiansPerDegree, 10.00,    // rad, m
                            200.*kRadiansPerDegree, 10.00,    // rad/s, m/s
                            200.*kRadiansPerDegree, 10.00,    // rad/s^2, m/s^2
                            0.;                             // s
    pre_starting_state.SetCovariance(starting_pose_sigmas);    
    target_poses_->AddElement(pre_starting_state);
    target_poses_->SetTimestamp(starting_state_ts);
    VLOG(1) << "Starting kinematic state: " << pre_starting_state.ToString(2);
    
    // Set markers on queues
    target_poses_->SetFixedPoint("TagViewResidual"); // Marks the last tag view residual location
    target_poses_->SetFixedPoint("ProblemBeginMarker"); // Marks the last tag view residual location
    VLOG(1) << "Starting target pose at target_poses_ queue idx: "
        << target_poses_->FixedPointToString("TagViewResidual");
    
    return SetStartingResiduals(starting_state_ts-1);
  }*/
  
  /* Set starting states
   * Machine starting state is at origin. Target starting state is measured from observations.
  bool SetStartingStates(const int64_t starting_ts, const PoseState& starting_pose) {
    // Calculate the starting state timestamp
    int64_t starting_state_ts = (starting_ts / inter_state_interval_) * inter_state_interval_;
    VLOG(1) << "Using starting state ts = " << starting_state_ts;
    VLOG(1) << "Using starting pose: " << starting_pose.ToString();
    
    // Start filtering at starting_state_ts
    swf_iterations_.StartFiltering(starting_state_ts);
    iteration_record_.Reset(starting_state_ts);
    
    // Create a starting kinematic state using pose. Add it to target state queues
    //  Assumes that starting velocity and accelerations are zero.
    //  Time delay of camera is kept at zero as the camera is the reference itself.
    anantak::KinematicState starting_state;   // State is zero
    starting_state.SetPose(starting_pose);
    anantak::Vector7d starting_pose_sigmas;
    starting_pose_sigmas << 200.*kRadiansPerDegree, 10.00,    // rad, m
                            200.*kRadiansPerDegree, 10.00,    // rad/s, m/s
                            200.*kRadiansPerDegree, 10.00,    // rad/s^2, m/s^2
                            0.;                             // s
    starting_state.SetCovariance(starting_pose_sigmas);    
    starting_state.SetTimestamp(starting_state_ts);
    target_poses_->AddElement(starting_state);
    target_poses_->SetTimestamp(starting_state_ts);
    
    // Set markers on queues
    target_poses_->SetFixedPoint("TagViewResidual"); // Marks the last tag view residual location
    VLOG(1) << "Starting target pose at target_poses_ queue idx: "
        << target_poses_->FixedPointToString("TagViewResidual");
    
    return SetStartingResiduals(starting_state_ts-1);
  }*/
  
  /*bool SetStartingResiduals(const int64_t& starting_state_ts) {
    
    // Set queues' data starting timestamp
    inter_target_pose_residuals_->SetStartingTimestamp(starting_state_ts);
    tag_view_residuals_->SetStartingTimestamp(starting_state_ts);
    tag_pose_priors_->SetStartingTimestamp(starting_state_ts);
    
    // Residuals queue have three markers:
    //    PreProblemBegin: at creation of new problem object, residuals are added from here
    //    ProblemEnd: at every iteration, residuals are added from here to the end of queue
    //    PreWindowBegin: at every iteration, states till this residual are marked constant
    
    // Markers for inter-target pose residuals
    inter_target_pose_residuals_->SetFixedPoint("PreProblemBegin");
    inter_target_pose_residuals_->SetFixedPoint("ProblemEnd");
    inter_target_pose_residuals_->SetFixedPoint("PreWindowBegin");
    inter_target_pose_residuals_->SetFixedPoint("TagViewResidual"); // Marks the last tag view residual location
    VLOG(1) << "Starting target pose at inter target residual queue idx: "
        << inter_target_pose_residuals_->FixedPointToString("TagViewResidual");
    // Markers for target view residuals
    tag_view_residuals_->SetFixedPoint("PreProblemBegin");
    tag_view_residuals_->SetFixedPoint("ProblemEnd");
    tag_view_residuals_->SetFixedPoint("PreWindowBegin");
    // Markers for tag_pose_priors_
    tag_pose_priors_->SetFixedPoint("PreProblemBegin");
    tag_pose_priors_->SetFixedPoint("ProblemEnd");
    tag_pose_priors_->SetFixedPoint("PreWindowBegin");
    
    return true;
  }*/
  
  /** Filtering operations */
  
  // Run iteration without any observations
  bool RunIteration(const int64_t& iteration_end_ts) {
    return true;
  }
  
  // Run iteration with observations
  bool RunIteration(const int64_t& iteration_end_ts,
                    const anantak::ObservationsVectorStoreMap& observations_map) {
    
    return true;
  }
  
  // Run iteration with historical readings read using FileKeeper
  bool RunIteration(const int64_t& iteration_end_ts,
      const std::vector<std::unique_ptr<std::vector<anantak::SensorMsg>>>& observations) {
    
    if (filter_run_mode_ == kInitializing) {
      return RunInitializingIteration(iteration_end_ts, observations);
    }
    
    if (!StartIteration(iteration_end_ts)) {
      LOG(ERROR) << "Could not start iteration";
      return false;
    }
    
    if (!CreateIterationStatesAndResiduals(iteration_end_ts, observations)) {
      LOG(ERROR) << "Could not create states for the iteration";
      return false;
    }
    
    if (!RunFiltering(iteration_end_ts)) {
      LOG(ERROR) << "Could not run filtering operations";
      return false;
    }
    
    RunReporting();
    
    return true;
  }

  // Run iteration with historical readings read using FileKeeper
  /*bool RunIteration01(const int64_t& iteration_end_ts,
      const std::vector<std::unique_ptr<std::vector<anantak::SensorMsg>>>& observations) {
    
    if (filter_run_mode_ == kInitializing) {
      return RunInitializingIteration(iteration_end_ts, observations);
    }
    
    if (!StartIteration(iteration_end_ts)) {
      LOG(ERROR) << "Could not start iteration";
      return false;
    }
    
    // Create observations residuals
    for (int i_file=0; i_file<observations.size(); i_file++) {
      for (int i_msg=0; i_msg<observations.at(i_file)->size(); i_msg++) {
        if (!CreateObservationResidual(observations.at(i_file)->at(i_msg))) {
          LOG(ERROR) << "Could not create a residual from message. Skipping. file: "
              << i_file << " msg: " << i_msg;
          continue;
        }
      }
    }
    
    if (!RunFiltering(iteration_end_ts)) {
      LOG(ERROR) << "Could not run filtering operations";
      return false;
    }
    
    if (!CreateEndingStatesAndResiduals(iteration_end_ts, observations)) {
      LOG(ERROR) << "Could not create ending states for the iteration";
      return false;
    }
    
    RunReporting();
    
    return true;
  }*/

  // Start iteration: set iteration records, counters and sliding window filter markers
  bool StartIteration(const int64_t& iteration_end_ts) {
    // Is this iteration end ts valid?
    if (!iteration_record_.Increment(iteration_end_ts)) {
      LOG(ERROR) << "Invalid iteration_end_ts " << iteration_end_ts;
      return false;
    }
    iteration_record_.ResetIterationCounters();
    
    // Add this timestamp to swf filtering iterations helper
    swf_iterations_.AddData(iteration_end_ts);
    
    return true;
  }

  // Create new states for the iteration
  /*bool CreateIterationStatesAndResiduals(const int64_t& iteration_end_ts,
      const std::vector<std::unique_ptr<std::vector<anantak::SensorMsg>>>& observations) {
    
    // Make sure that there is an existing last state
    if (target_poses_->n_msgs() == 0) {
      LOG(ERROR) << "Target kin states queue is empty! Can not create new states.";
      return false;
    }
    
    // Calculate iteration ending state timestamp and number of states to be created
    const int64_t iteration_end_state_ts =
        ((iteration_end_ts / inter_state_interval_) +1) * inter_state_interval_;
    const int64_t last_state_ts =
        target_poses_->Back().timestamp_;
    const int32_t num_states_to_create =
        (iteration_end_state_ts - last_state_ts) / inter_state_interval_;
    VLOG(2) << "  States last, iter end, states end, num = " << last_state_ts << " "
        << iteration_end_ts << " " << iteration_end_state_ts << " " << num_states_to_create;
    
    // Calculate target's approximate poses from the observations 
    std::vector<PoseState> approx_tgt_poses;
    for (int i_file=0; i_file<observations.size(); i_file++) {
      for (int i_msg=0; i_msg<observations.at(i_file)->size(); i_msg++) {
        PoseState _pose;
        const anantak::SensorMsg _msg = observations.at(i_file)->at(i_msg);
        if (CalculateApproxTargetPose(_msg, &_pose)) {
          _pose.SetTimestamp(_msg.header().timestamp());
          approx_tgt_poses.emplace_back(_pose);
        }
      }
    }
    
    // Create a spline from the poses
    PiecewisePoseSpline spline;
    // Insert last known target pose
    if (!last_seen_tgt_pose_.IsZero()) {
      if (!spline.AddPose(last_seen_tgt_pose_)) {
        LOG(FATAL)<<"Could not add starting pose to spline";}}
    for (int i=0; i<approx_tgt_poses.size(); i++) {
      if (!spline.AddPose(approx_tgt_poses.at(i))) {
        LOG(FATAL) << "Could not add pose to spline";
        return false;
      }
      // Add pose to observations interpolator
      if (!target_pose_observations_interpolator_.AddPose(approx_tgt_poses.at(i))) {
        LOG(FATAL) << "Could not add pose to target pose observations interpolator";
        return false;
      }
    }
    if (approx_tgt_poses.size()>0) last_seen_tgt_pose_ = approx_tgt_poses.back();
    
    // Re-estimate the current ending state
    if (true) {
      // Re-estimate the current ending state using spline
      //anantak::KinematicState* tgt_state = target_poses_->BackPtr();
      //if (!spline.Interpolate(tgt_state->Timestamp(), tgt_state)) {
      //  LOG(FATAL) << "Can not interpolate kinematic state";
      //  return false;
      //}
      //// Add covariance to state
      //target_pose_observations_interpolator_.SetCovariance(tgt_state->R(), 1./15., 3, &tgt_state->covariance_);
      anantak::KinematicState* tgt_state = target_poses_->BackPtr();
      const anantak::KinematicState* last_tgt_state = target_poses_->NthLastElementPtr(2);
      if (!target_pose_observations_interpolator_.Interpolate(tgt_state->Timestamp(), tgt_state, last_tgt_state)) {
        LOG(FATAL) << "Can not interpolate kinematic state";
        return false;
      }
      // Estimated state
      VLOG(1) << "Target state set from observations: " << tgt_state->ToString(2);      
      
      // Add a pose prior to the current kinematic state
      KinematicStatePrior* kin_state_prior = tag_pose_priors_->NextMutableElement();
      kin_state_prior->Reset();
      if (!kin_state_prior->Create(tgt_state)) {
        LOG(FATAL) << "Could not create the prior";
        return false;
      }
      tag_pose_priors_->SetTimestamp(tgt_state->timestamp_);
      iteration_record_.iteration_counters["TargetPosePriorsCreated"]++;
      //kin_state_prior->AddToProblem(problem_.get());
      //VLOG(1) << "Added a prior to the problem with state at priors queue: "
      //    << target_poses_->FixedPointToString(fp)
      //    << " K0 timestamp: " << _kin->Timestamp();
    }
    
    
    
    // Create and initiate iteration poses
    for (int i=0; i<num_states_to_create; i++) {
    //for (int i=0; i<0; i++) {
      
      // Create a new state
      anantak::KinematicState* last_tgt_state = target_poses_->BackPtr();
      anantak::KinematicState* tgt_state = target_poses_->NextMutableElement();
      tgt_state->SetZero();  // Just to be safe
      
      // Predict the new state
      int64_t tgt_state_ts = last_tgt_state->Timestamp() + inter_state_interval_;
      if (false) {
        // Predict new state using spline
        if (!spline.Interpolate(tgt_state_ts, tgt_state, last_tgt_state)) {
          LOG(FATAL) << "Can not interpolate kinematic state";
          return false;
        }
      } else {
        // Predict the new state using kinematic motion
        if (!MoveInFixedTime(*last_tgt_state, inter_state_interval_, tgt_state)) {
          LOG(FATAL) << "Can not move forward in time from state";
          return false;          
        }
      }
      
      // Assign timestamp on the queues
      if (tgt_state->timestamp_ <= last_tgt_state->timestamp_) {
        LOG(ERROR)<<"tgt_state->timestamp_ <= last_tgt_state->timestamp_ "
            << tgt_state->timestamp_ << " " << last_tgt_state->timestamp_;
        return false;
      }
      target_poses_->SetTimestamp(tgt_state->timestamp_);
      iteration_record_.iteration_counters["TargetPosesCreated"]++;
      
      // Create a new inter-state residual
      anantak::InterKinematicStateResidual* kin_state_resid =
          inter_target_pose_residuals_->NextMutableElement();
      kin_state_resid->Reset(); // Just to be safe
      
      // Create the inter-state residual
      if (!kin_state_resid->Create(last_tgt_state, tgt_state, inter_state_interval_)) {
        LOG(FATAL) << "Could not create inter state residual. Can not continue.";
        return false;
      }
      inter_target_pose_residuals_->SetTimestamp(last_tgt_state->timestamp_);
      iteration_record_.iteration_counters["InterTargetResidualsCreated"]++;
      
      // Report projected state
      VLOG(1) << "Target predicted state: " << tgt_state->ToString(2);
      
      // Check for mistakes
      if (tgt_state->timestamp_ > iteration_end_state_ts) {
        LOG(FATAL) << "tgt_state->timestamp_ > iteration_end_state_ts. Should not happen! Quit. "
            << tgt_state->timestamp_ << " " << iteration_end_state_ts;
        return false;
      }
    }
    
    // Create observations residuals
    for (int i_file=0; i_file<observations.size(); i_file++) {
      for (int i_msg=0; i_msg<observations.at(i_file)->size(); i_msg++) {
        if (!CreateObservationResidual(observations.at(i_file)->at(i_msg))) {
          LOG(ERROR) << "Could not create a residual from message. Skipping. file: "
              << i_file << " msg: " << i_msg;
          continue;
        }
      }
    }
    
    return true;
  }*/
  
  // Create new states for the iteration
  bool CreateIterationStatesAndResiduals(const int64_t& iteration_end_ts,
      const std::vector<std::unique_ptr<std::vector<anantak::SensorMsg>>>& observations) {
    
    // Calculate target's approximate poses from the observations 
    std::vector<PoseState> approx_tgt_poses;
    for (int i_file=0; i_file<observations.size(); i_file++) {
      for (int i_msg=0; i_msg<observations.at(i_file)->size(); i_msg++) {
        PoseState _pose;
        const anantak::SensorMsg _msg = observations.at(i_file)->at(i_msg);
        if (CalculateApproxTargetPose(_msg, &_pose)) {
          _pose.SetTimestamp(_msg.header().timestamp());
          approx_tgt_poses.emplace_back(_pose);
        }
      }
    }
    
    // Create a spline pose residual for every target pose observation
    for (int i=0; i<approx_tgt_poses.size(); i++) {
      const PoseState& tgt_obs = approx_tgt_poses.at(i);
      
      // Get the last observation timestamp
      int64_t last_obs_ts = 0;
      if (linear_observations_spline_->NumControlPoints() > 0) {
        last_obs_ts = linear_observations_spline_->EndingTimestamp();
      }
      
      // Add measured pose to linear spline
      linear_observations_spline_->AddControlPose(tgt_obs);
      iteration_record_.iteration_counters["TargetObservations"]++;
      VLOG(2) << "Added target observation to linear observation spline " << tgt_obs.ToString();
      
      // Add interpolated observations' residuals
      int64_t inter_pose_constraint_interval_ = inter_state_interval_/4;
      if (last_obs_ts > 0) {
        for (int64_t interp_ts = ((last_obs_ts / inter_pose_constraint_interval_)+1) * inter_pose_constraint_interval_;
             interp_ts < tgt_obs.Timestamp();
             interp_ts += inter_pose_constraint_interval_) {
          
          // Interpolate observations
          VLOG(1) << "Creating interpolated target observation residual at ts " << interp_ts;
          PoseState _P;
          linear_observations_spline_->InterpolatePose(interp_ts, &_P);
          
          anantak::PoseSpline_PoseResidual* spline_pose_resid = spline_pose_residuals_->NextMutableElement();
          spline_pose_resid->Reset();
          if (!cubic_pose_spline_->CreatePoseResidual(_P, spline_pose_resid)) {
            LOG(ERROR) << "Could not create a spline pose residual for interpolated target pose. Skip";
            spline_pose_resid->Reset();
            spline_pose_residuals_->decrement();
          } else {
            spline_pose_residuals_->SetTimestamp(_P.Timestamp());
            iteration_record_.iteration_counters["SplinePoseResidualsCreated"]++;
          }
          
        }
      }
      
      // Create the ending residual using the actual residual
      VLOG(1) << "Creating target observation residual at ts " << tgt_obs.Timestamp();
      anantak::PoseSpline_PoseResidual* spline_pose_resid = spline_pose_residuals_->NextMutableElement();
      spline_pose_resid->Reset();
      if (!cubic_pose_spline_->CreatePoseResidual(tgt_obs, spline_pose_resid)) {
        LOG(ERROR) << "Could not create a spline pose residual for target pose. Skip";
        spline_pose_resid->Reset();
        spline_pose_residuals_->decrement();
      } else {
        spline_pose_residuals_->SetTimestamp(tgt_obs.Timestamp());
        iteration_record_.iteration_counters["SplinePoseResidualsCreated"]++;
      }
      
    }
    
    // Create priors for control poses created in this iteration
    int32_t num_created = 0;
    if (!cubic_pose_spline_->CreateControlPosePriors(spline_pose_priors_.get(), &num_created)) {
      LOG(ERROR) << "Could not create spline pose priors. Skip.";
    } else {
      iteration_record_.iteration_counters["SplinePosePriorsCreated"] += num_created;
    }
    
    return true;
  }
  
  
  // Create new states for the iteration
  /*bool CreateEndingStatesAndResiduals(const int64_t& iteration_end_ts,
      const std::vector<std::unique_ptr<std::vector<anantak::SensorMsg>>>& observations) {
    
    // Make sure that there is an existing last state
    if (target_poses_->n_msgs() == 0) {
      LOG(ERROR) << "Target kin states queue is empty! Can not create new states.";
      return false;
    }
    
    // Calculate iteration ending state timestamp and number of states to be created
    const int64_t iteration_end_state_ts = iteration_end_ts;
        //(iteration_end_ts / inter_state_interval_) * inter_state_interval_;
    const int64_t last_state_ts = target_poses_->Back().timestamp_;
    VLOG(2) << "  States last, iter end, states end, num = " << last_state_ts << " "
        << iteration_end_ts << " " << iteration_end_state_ts << " " << 1;
    
    // Create a new state
    anantak::KinematicState* last_tgt_state = target_poses_->BackPtr();
    anantak::KinematicState* tgt_state = target_poses_->NextMutableElement();
    tgt_state->SetZero();  // Just to be safe
    
    // Predict the new state
    int64_t tgt_state_ts = iteration_end_state_ts;
    // Predict the new state using kinematic motion
    if (!MoveInFixedTime(*last_tgt_state, iteration_end_state_ts-last_state_ts, tgt_state)) {
      LOG(FATAL) << "Can not move forward in time from state";
      return false;
    }
    
    // Assign timestamp on the queues
    if (tgt_state->timestamp_ <= last_tgt_state->timestamp_) {
      LOG(ERROR)<<"tgt_state->timestamp_ <= last_tgt_state->timestamp_ "
          << tgt_state->timestamp_ << " " << last_tgt_state->timestamp_;
      return false;
    }
    target_poses_->SetTimestamp(tgt_state->timestamp_);
    iteration_record_.iteration_counters["TargetPosesCreated"]++;
    
    // Create a new inter-state residual
    anantak::InterKinematicStateResidual* kin_state_resid =
        inter_target_pose_residuals_->NextMutableElement();
    kin_state_resid->Reset(); // Just to be safe
    
    // Create the inter-state residual
    if (!kin_state_resid->Create(last_tgt_state, tgt_state, iteration_end_state_ts-last_state_ts)) {
      LOG(FATAL) << "Could not create inter state residual. Can not continue.";
      return false;
    }
    inter_target_pose_residuals_->SetTimestamp(last_tgt_state->timestamp_);
    iteration_record_.iteration_counters["InterTargetResidualsCreated"]++;
    
    // Report projected state
    VLOG(1) << "Target predicted state: " << tgt_state->ToString(2);
    
    // Check for mistakes
    if (tgt_state->timestamp_ > iteration_end_state_ts) {
      LOG(FATAL) << "tgt_state->timestamp_ > iteration_end_state_ts. Should not happen! Quit. "
          << tgt_state->timestamp_ << " " << iteration_end_state_ts;
      return false;
    }
    
    return true;
  }*/
  
  bool CalculateApproxTargetPose(const anantak::SensorMsg& msg, PoseState* pose) {
    if (!IsValidTargetViewObservation(msg)) {
      LOG(ERROR) << "Skipping message, it is invalid or is not processed by this filter.";
      return false;
    }
    
    anantak::AprilTagMessage undist_apriltag_msg;
    undistorted_camera_.Undistort(msg.april_msg(), &undist_apriltag_msg);
    if (!target_.CalculateTargetPose(undist_apriltag_msg, undistorted_camera_.camera_, pose)) {
      LOG(ERROR) << "Could not calculate target pose";
      return false;
    }
    
    return true;
  }
  
  
  /** Create new states for the iteration
  bool CreateIterationStates(const int64_t& iteration_end_ts) {
    
    // Make sure that there is an existing last state
    if (target_poses_->n_msgs() == 0) {
      LOG(ERROR) << "Target kin states queue is empty! Can not create new states.";
      return false;
    }
    
    // Calculate iteration ending state timestamp and number of states to be created
    const int64_t iteration_end_state_ts =
        ((iteration_end_ts / inter_state_interval_) +1) * inter_state_interval_;
    const int64_t last_state_ts =
        target_poses_->Back().timestamp_;
    const int32_t num_states_to_create =
        (iteration_end_state_ts - last_state_ts) / inter_state_interval_;
    VLOG(2) << "  States last, iter end, states end, num = " << last_state_ts << " "
        << iteration_end_ts << " " << iteration_end_state_ts << " " << num_states_to_create;
    
    // Create and predict new target states
    for (int i=0; i<num_states_to_create; i++) {
      
      // Create a new state
      anantak::KinematicState* last_tgt_state = target_poses_->BackPtr();
      anantak::KinematicState* tgt_state = target_poses_->NextMutableElement();
      tgt_state->SetZero();  // Just to be safe
      
      // Create a new inter-state residual
      anantak::InterKinematicStateResidual* kin_state_resid =
          inter_target_pose_residuals_->NextMutableElement();
      kin_state_resid->Reset(); // Just to be safe
      
      // Predict new state and create a inter-state residual
      if (!kin_state_resid->Create(last_tgt_state, tgt_state, inter_state_interval_)) {
        LOG(ERROR) << "Could not predict target kinematic state. Can not continue.";
        return false;
      }
      
      // Assign timestamp on the queues
      if (tgt_state->timestamp_ <= last_tgt_state->timestamp_) {
        LOG(ERROR)<<"tgt_state->timestamp_ <= last_tgt_state->timestamp_ "
            << tgt_state->timestamp_ << " " << last_tgt_state->timestamp_;
        return false;
      }
      target_poses_->SetTimestamp(tgt_state->timestamp_);
      inter_target_pose_residuals_->SetTimestamp(last_tgt_state->timestamp_);
      iteration_record_.iteration_counters["TargetPosesCreated"]++;
      iteration_record_.iteration_counters["InterTargetResidualsCreated"]++;
      
      VLOG(1) << "Target predicted state: " << tgt_state->ToString(2);
      
      // Check for mistakes
      if (tgt_state->timestamp_ > iteration_end_state_ts) {
        LOG(ERROR) << "tgt_state->timestamp_ > iteration_end_state_ts. Should not happen! Quit. "
            << tgt_state->timestamp_ << " " << iteration_end_state_ts;
        return false;
      }
    }
    
    return true;
  }*/
  
  // Check if message has a header, has an AprilMessage, and camera is processed by this filter
  bool IsValidTargetViewObservation(const anantak::SensorMsg& msg) {
    // Message header
    const anantak::HeaderMsg& hdr = msg.header();
    
    // Reject the residual if timestamp of reading is before the sliding window
    if (hdr.timestamp() < swf_iterations_.SlidingWindowTimestamp()) {
      LOG(ERROR) << "Timestamp of reading is before sliding window. Skipping reading.";
      return false;
    }
    
    // April tags message 
    if (hdr.type() == "AprilTags") {
      
      // Make sure there is an Aptiltag message here
      if (!msg.has_april_msg()) {
        LOG(WARNING) << "Apriltag message not found the sensor message! Skipping.";
        return false;
      }
      
      // Extract Apriltag message
      const anantak::AprilTagMessage& apriltag_msg = msg.april_msg();
      
      // Check for camera number
      if (!apriltag_msg.has_camera_num()) {
        LOG(WARNING) << "Apriltag message does not have a camera number! Skipping.";
        return false;
      }
      
      // Get camera number
      const int32_t& cam_id = apriltag_msg.camera_num();
      
      // Check the camera number, this should be in the list for this filter
      auto it = std::find(options_.tag_camera_ids.begin(), options_.tag_camera_ids.end(), cam_id);
      if (it == options_.tag_camera_ids.end()) {
        // This camera is not processed by this filter
        LOG(INFO) << "Camera of this Apriltag message is not processed by this filter";
        return false;
      }
      
    }  // type = "AprilTag"
    
    return true;
  }
  
  // Create a residual from a sensor message
  /*bool CreateObservationResidual(const anantak::SensorMsg& msg) {
    
    if (IsValidTargetViewObservation(msg)) {
      if (!CreateTargetViewResidual(msg)) {
        LOG(ERROR) << "Could not create residual for target view message. Skipping.";
        return false;
      }
    }
    
    return true;
  }*/

  // Run filtering operations
  bool RunFiltering(const int64_t& iteration_end_ts) {
    
    // Reset problem
    if (!problem_ || swf_iterations_.IsItTimeToResetProblem()) {
      
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
      iteration_record_.algorithm_counters["ProblemObjects"]++;
      
      // Add priors
      AddPriors();
      
      // Build the problem
      AddDataToProblem();
      
      // Mark constant states
      MarkStatesConstant();
    }
    
    // Add this iteration's data to the problem
    VLOG(2) << "Adding new data to the problem.";
    AddNewDataToProblem();
    VLOG(2) << "Added new data to the problem.";
    
    // TODO - temporary reporting before solving
    RunReporting("Pre");
    
    // Solve the problem
    if (swf_iterations_.IsItTimeToSolveProblem()) {
      VLOG(2) << "Solving the problem.";
      SolveProblem();
      VLOG(2) << "Solved the problem.";
      
      // If any state is recalculated, its associated residuals need to be updated.
      if (!UpdateStatesAndResiduals(iteration_end_ts)) {
        LOG(ERROR) << "Could not update states and residuals after solving";
      }
      
      // Mark constant states
      MarkNewStatesConstant();
      
      // Calculate new solve begin ts
      swf_iterations_.SolveProblem();
    }
    
    return true;
  }
  
  // Run reporting
  bool RunReporting(const std::string& prefix="") {
    VLOG(1) << "Iteration record " << iteration_record_.IterationCountersToString()
        << "  Algorithm record " << iteration_record_.AlgorithmCountersToString();
    
    std::vector<Vector39d> kin_states_vec;
    
    // Calculate poses at knot points of the spline
    int64_t _ts;
    for (_ts =  cubic_pose_spline_->StartingTimestamp();
         _ts <= cubic_pose_spline_->EndingTimestamp();
         _ts += cubic_pose_spline_->KnotDistance()) {
      KinematicState _K;
      cubic_pose_spline_->InterpolatePose(_ts, &_K, true);  // Include errors is set to true
      VLOG(3) << "Interpolated pose: " << _K.ToString(2);
      kin_states_vec.emplace_back(_K.ToVector());
    }
    
    // Save poses to file
    Eigen::Matrix<double,39,Eigen::Dynamic> kin_states_mat;
    kin_states_mat.resize(39,kin_states_vec.size());
    for (int i=0; i<kin_states_vec.size(); i++) {
      kin_states_mat.col(i) = kin_states_vec[i];
    }
    std::string filename =
      "src/Models/Plots/TargetKinFit." + prefix + std::to_string(iteration_record_.iteration_number%10) + ".data.csv";
    WriteMatrixToCSVFile(filename, kin_states_mat.transpose());
    
    std::vector<Vector8d> tgt_poses_vec;
    
    // Target pose observations
    for (int i=0; i<linear_observations_spline_->control_poses_->n_msgs(); i++) {
      Vector8d vec; 
      linear_observations_spline_->control_poses_->At(i).ToVector(&vec, cubic_pose_spline_->StartingTimestamp());
      tgt_poses_vec.emplace_back(vec);
    }
    
    // Save target poses to file
    Eigen::Matrix<double,8,Eigen::Dynamic> tgt_poses_mat;
    tgt_poses_mat.resize(8,tgt_poses_vec.size());
    for (int i=0; i<tgt_poses_vec.size(); i++) {
      tgt_poses_mat.col(i) = tgt_poses_vec[i];
    }
    filename =
      "src/Models/Plots/TargetPoses." + std::to_string(iteration_record_.iteration_number%10) + ".data.csv";
    WriteMatrixToCSVFile(filename, tgt_poses_mat.transpose());
    
    std::vector<Vector8d> tgt_poses_interp_vec;
    
    // Add interpolations
    for (auto fp = spline_pose_residuals_->GetNextFixedPoint("PreProblemBegin");
         spline_pose_residuals_->IsNotPastFixedPoint("ProblemEnd", fp);
         spline_pose_residuals_->IncrementFixedPoint(fp)) {
      const PoseState& _P = spline_pose_residuals_->ElementAtFixedPoint(fp).Observation();
      Vector8d vec;
      _P.ToVector(&vec, cubic_pose_spline_->StartingTimestamp());
      tgt_poses_interp_vec.emplace_back(vec);
    }
    
    // Save target interpolated poses to file
    Eigen::Matrix<double,8,Eigen::Dynamic> tgt_poses_interp_mat;
    tgt_poses_interp_mat.resize(8,tgt_poses_interp_vec.size());
    for (int i=0; i<tgt_poses_interp_vec.size(); i++) {
      tgt_poses_interp_mat.col(i) = tgt_poses_interp_vec[i];
    }
    filename =
      "src/Models/Plots/TargetPosesInterp." + std::to_string(iteration_record_.iteration_number%10) + ".data.csv";
    WriteMatrixToCSVFile(filename, tgt_poses_interp_mat.transpose());
    
    // Add control points of cubic spline
    std::vector<Vector8d> cubic_spline_state_control_poses_vec;
    std::vector<Vector8d> cubic_spline_control_poses_vec;
    
    for (int i=0; i<cubic_pose_spline_->NumControlPoints(); i++) {
      Vector8d vec;
      PoseState _P(cubic_pose_spline_->control_poses_->At(i)); // make a copy
      _P.ToVector(&vec, cubic_pose_spline_->StartingTimestamp());
      cubic_spline_state_control_poses_vec.emplace_back(vec);
      _P.Recalculate();
      _P.ToVector(&vec, cubic_pose_spline_->StartingTimestamp());
      cubic_spline_control_poses_vec.emplace_back(vec);
    }
    
    // Save target interpolated poses to file
    Eigen::Matrix<double,8,Eigen::Dynamic> cubic_spline_control_poses_mat;
    cubic_spline_control_poses_mat.resize(8,cubic_spline_control_poses_vec.size());
    for (int i=0; i<cubic_spline_control_poses_vec.size(); i++) {
      cubic_spline_control_poses_mat.col(i) = cubic_spline_control_poses_vec[i];
    }
    filename =
      "src/Models/Plots/CubicSplineControlPoses." + prefix + std::to_string(iteration_record_.iteration_number%10) + ".data.csv";
    WriteMatrixToCSVFile(filename, cubic_spline_control_poses_mat.transpose());
    
    Eigen::Matrix<double,8,Eigen::Dynamic> cubic_spline_state_control_poses_mat;
    cubic_spline_state_control_poses_mat.resize(8,cubic_spline_state_control_poses_vec.size());
    for (int i=0; i<cubic_spline_state_control_poses_vec.size(); i++) {
      cubic_spline_state_control_poses_mat.col(i) = cubic_spline_state_control_poses_vec[i];
    }
    filename =
      "src/Models/Plots/CubicSplineStateControlPoses." + std::to_string(iteration_record_.iteration_number%10) + ".data.csv";
    WriteMatrixToCSVFile(filename, cubic_spline_state_control_poses_mat.transpose());
    
    return true;
  }
  
  /*bool RunReporting(const std::string& prefix="") {
    VLOG(1) << "Iteration record " << iteration_record_.IterationCountersToString()
        << "  Algorithm record " << iteration_record_.AlgorithmCountersToString();
    
    std::vector<Vector39d> kin_states_vec;
    
    // Show solved states
    { auto fp = inter_target_pose_residuals_->GetNextFixedPoint("PreWindowBegin");
      KinematicState _K(*inter_target_pose_residuals_->ElementAtFixedPoint(fp).K0_);
      _K.Recalculate();
      VLOG(3) << "Queue timestamp: " << inter_target_pose_residuals_->Timestamp(fp)
          << " State K0: " << _K.ToString(2);
      kin_states_vec.emplace_back(_K.ToVector());
    }
    for (auto fp = inter_target_pose_residuals_->GetNextFixedPoint("PreWindowBegin");
         inter_target_pose_residuals_->IsNotPastTheEnd(fp);
         inter_target_pose_residuals_->IncrementFixedPoint(fp)) {
      KinematicState _K(*inter_target_pose_residuals_->ElementAtFixedPoint(fp).K1_);
      _K.Recalculate();
      VLOG(3) << "Queue timestamp: " << inter_target_pose_residuals_->Timestamp(fp)
          << " State K1: " << _K.ToString(2);
      kin_states_vec.emplace_back(_K.ToVector());
    }
    
    // Save states to file
    Eigen::Matrix<double,39,Eigen::Dynamic> kin_states_mat;
    kin_states_mat.resize(39,kin_states_vec.size());
    for (int i=0; i<kin_states_vec.size(); i++) {
      kin_states_mat.col(i) = kin_states_vec[i];
    }
    std::string filename =
      "src/Models/Plots/TargetKinFit." + prefix + std::to_string(iteration_record_.iteration_number%10) + ".data.csv";
    WriteMatrixToCSVFile(filename, kin_states_mat.transpose());
    
    std::vector<Vector8d> tgt_poses_vec;
    
    // Get target poses from target view residuals
    for (auto fp = tag_view_residuals_->GetNextFixedPoint("PreWindowBegin");
         tag_view_residuals_->IsNotPastTheEnd(fp);
         tag_view_residuals_->IncrementFixedPoint(fp)) {
      KinematicCalibrationTargetViewResidual* tag_view_resid =
          tag_view_residuals_->MutableElementAtFixedPoint(fp);
      Vector8d vec; 
      tag_view_resid->TargetPoseToVector(&vec);
      //tag_view_resid->Dummy(&vec);
      tgt_poses_vec.emplace_back(vec);
    }
    
    // Save target poses to file
    Eigen::Matrix<double,8,Eigen::Dynamic> tgt_poses_mat;
    tgt_poses_mat.resize(8,tgt_poses_vec.size());
    for (int i=0; i<tgt_poses_vec.size(); i++) {
      tgt_poses_mat.col(i) = tgt_poses_vec[i];
    }
    filename =
      "src/Models/Plots/TargetPoses." + std::to_string(iteration_record_.iteration_number%10) + ".data.csv";
    WriteMatrixToCSVFile(filename, tgt_poses_mat.transpose());    
    
    return true;
  }*/
  
  // Test message to kinematic state fitting
  //    Calculate the pose of the target from the message,
  //    Create a kinematic state from the calculated pose, add a prior covariance
  //    Create a Kinematic state prior for this state
  //    Create a Target view residual for the message
  //    Add both residuals to a problem, solve
  //    Recalculate kinematic state, then calculate the target view residual
  //    Compare the results of residuals 
  bool TestKinematicStateToTargetMaths(const anantak::SensorMsg& msg) {
    VLOG(1) << "****** Testing target view residual **********";
    
    const anantak::AprilTagMessage& apriltag_msg = msg.april_msg();
    
    // Display window
    std::string window_name = "TestKinematicStateToTargetMaths";
    cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE ); // Create a window for display
    cv::moveWindow(window_name, 0, 0); // move window to a new positon
    
    // Create a display image
    cv::Mat image(480, 640, CV_8UC3);
    image = 0.;
    
    // Create undistorted camera
    anantak::AprilTagMessage undist_apriltag_msg;
    undistorted_camera_.Undistort(apriltag_msg, &undist_apriltag_msg);
    PlotAprilTagMessage(apriltag_msg, &image, CV_WHITE);
    PlotAprilTagMessage(undist_apriltag_msg, &image, CV_GREEN);
    
    // Calculate approximate target pose using distorted camera
    PoseState dist_target_pose;
    target_.CalculateTargetPose(apriltag_msg, camera_, &dist_target_pose);
    VLOG(1) << "Calculated target pose using distorted camera: " << dist_target_pose.ToString();
    
    // Calculate approximate target pose after adjusting for distortions
    PoseState target_pose;
    target_.CalculateTargetPose(undist_apriltag_msg, undistorted_camera_.camera_, &target_pose);
    VLOG(1) << "Calculated target pose: " << target_pose.ToString();
    
    // Difference in poses above
    PoseState d_tgt_pose;
    target_pose.DiffPose(dist_target_pose, &d_tgt_pose, nullptr, nullptr);
    VLOG(1) << "difference in target poses: \n   " << d_tgt_pose.NormString();
    
    // Create a Kinematic state from target pose
    KinematicState kin;
    kin.SetPose(target_pose);
    anantak::Vector7d starting_pose_sigmas;
    starting_pose_sigmas << 200.*kRadiansPerDegree, 10.00,    // rad, m
                            200.*kRadiansPerDegree, 10.00,    // rad/s, m/s
                            200.*kRadiansPerDegree, 10.00,    // rad/s^2, m/s^2
                            0.;                             // s
    kin.SetCovariance(starting_pose_sigmas);
    kin.SetTimestamp(msg.header().timestamp());
    VLOG(1) << "Staring kinematic state: " << kin.ToString(2);
    
    // Create a prior
    KinematicStatePrior prior;
    prior.Create(&kin);
    
    // Create a target view residual
    KinematicCalibrationTargetViewResidual tgt_view_residual(&tag_view_residuals_options_);
    if (!tgt_view_residual.Create(&target_, &kin, &camera_, msg)) {
      LOG(ERROR) << "Target view residual could not be created.";
      return false;
    }
    
    // Create a problem 
    ceres::Problem problem(problem_options_);
    
    // build problem
    prior.AddToProblem(&problem);
    tgt_view_residual.AddToProblem(&problem);
    //ceres::CauchyLoss loss_func(1.0);
    //tgt_view_residual.AddToProblem(&problem, &loss_func);
    
    // Solve problem
    ceres::Solve(solver_options_, &problem, &solver_summary_);
    VLOG(1) << solver_summary_.BriefReport();
    VLOG(1) << "Num residual blocks reduced: " << solver_summary_.num_residual_blocks_reduced
        << " parameter blocks reduced: " << solver_summary_.num_parameter_blocks_reduced;
    
    // Update kinematic state
    kin.Recalculate();
    kin.SetCovariance(&problem);
    VLOG(1) << "Recalculated kinematic state: " << kin.ToString(2);
    
    // Recalculate the residual
    //tgt_view_residual.CalculateResiduals2(msg, nullptr, &image);
    
    // Show image
    cv::imshow(window_name, image);
    cv::waitKey(0);
    
    VLOG(1) << "********** Done testing maths **********";
    return true;
  }
  
  // Create residuals for the target sighting
  /*bool CreateTargetViewResidual(const anantak::SensorMsg& msg) {
    
    const anantak::AprilTagMessage& apriltag_msg = msg.april_msg();
    
    // Check residual maths
    //TestKinematicStateToTargetMaths(msg);
    
    // Check for number of tag sightings
    const int32_t num_tags = apriltag_msg.tag_id_size();
    if (num_tags == 0) {
      LOG(ERROR) << "Apriltag message has no tags!?";
      return false;
    }
    
    // Calculate observation timestamp using camera delay
    int64_t camera_delay(camera_.TimeDelay().Value() * 1e6);
    int64_t observation_ts = msg.header().timestamp() + camera_delay;
    
    // Find the target pose previous to this reading
    if (!target_poses_->PositionFixedPointBeforeTimestamp("TagViewResidual", observation_ts, true)) {
      LOG(ERROR) << "Did not find target message timestamp in target poses queue. Skipping.";
      LOG(ERROR) << "Diff between target message and last target pose timestamps = "
          << observation_ts - target_poses_->LastTimestamp();
      return false;
    }
    anantak::KinematicState* target_pose =
        target_poses_->MutableElementAtFixedPoint("TagViewResidual");
    VLOG(2) << "Target TagViewResidual posn = " << target_poses_->FixedPointToString("TagViewResidual")
        << " pose ts: " << target_pose->timestamp_ << " obs ts: " << observation_ts
        << " obs - pose: " << observation_ts - target_pose->timestamp_;
    
    // Find the inter state residual previous to this reading
    //if (!inter_target_pose_residuals_->PositionFixedPointBeforeTimestamp("TagViewResidual", observation_ts,
    //                                                                     true)) { // ok_to_position_at_end
    //  LOG(ERROR) << "Did not find target message timestamp in inter target pose residuals queue. Skipping. "
    //      << "Obs ts: " << observation_ts << " Queue last ts: " << inter_target_pose_residuals_->LastTimestamp();
    //  LOG(ERROR) << "Diff between target message and last inter target pose residuals timestamps = "
    //      << observation_ts - inter_target_pose_residuals_->LastTimestamp();
    //  return false;
    ///}
    //anantak::InterKinematicStateResidual* inter_target_pose_resid =
    //    inter_target_pose_residuals_->MutableElementAtFixedPoint("TagViewResidual");
    //VLOG(2) << "inter_target_pose_residuals posn = " << inter_target_pose_residuals_->FixedPointToString("TagViewResidual")
    //    << " pose ts: " << target_pose->timestamp_;
    
    // Create tag view residuals
    KinematicCalibrationTargetViewResidual* tag_view_resid = tag_view_residuals_->NextMutableElement();
    tag_view_resid->Reset();  // Just to be safe
    if (!tag_view_resid->Create(&target_, target_pose, &camera_, msg, nullptr, //inter_target_pose_resid,
                                &undistorted_camera_, &target_pose_observations_interpolator_)) {
      LOG(ERROR) << "Could not create tag view residual. Skipping.";
      tag_view_resid->Reset();
      tag_view_residuals_->decrement();
      return false;
    }
    tag_view_residuals_->SetTimestamp(observation_ts);
    iteration_record_.iteration_counters["TagViewResidualsCreated"]++;
    
    //{PoseState starting_pose;
    //starting_pose.SetZero();
    //target_.CalculateTargetPose(apriltag_msg, camera_, &starting_pose);
    // Starting estimate of target pose wrt camera
    //if (!target_.CalculateTargetPose(apriltag_msg, camera_intrinsics_, target_pose)) {
    //  LOG(ERROR) << "Could not calculate starting estimate of the target pose. Skip";
    //  return false;
    ///}
    //// Report the calculation
    //VLOG(2) << " Calculated target pose in camera = " << *target_pose;
    //
    //// Create tag view residuals
    //for (int i_tag=0; i_tag<num_tags; i_tag++) {
    //  anantak::DynamicAprilTagViewResidual *tag_view_resid = tag_view_residuals_->NextMutableElement();
    //  if (!tag_view_resid->Create(
    //      timestamp, apriltag_msg, i_tag, target_,
    //      camera_pose, target_pose, &target_tag_size_, &camera_intrinsics_,
    //      &options_.apriltag_view_residual_options, true))
    //  {
    //    LOG(ERROR) << "Could not create tag view residual. Skipping.";
    //    tag_view_resid->Reset();
    //    tag_view_residuals_->decrement();
    //    return false;
    //  }
    //  tag_view_residuals_->SetTimestamp(timestamp);
    //  iteration_record_.iteration_counters["TagViewResidualsCreated"]++;
    //  
    //  // Report the timestamps associated
    //  VLOG(2) << "  Residual ts = " << tag_view_resid->timestamp_
    //      << " target pose ts = " << target_pose->timestamp_
    //      << ", " << target_poses_->ElementAtFixedPoint(target_poses_->GetNextFixedPoint("TagViewResidual")).timestamp_
    //      << "  " << target_poses_->Timestamp("TagViewResidual")
    //      << ", " << target_poses_->Timestamp(target_poses_->GetNextFixedPoint("TagViewResidual"));
    //} // for i_tag
    
    return true;
  } // CreateResidualsForTargetView*/
  
  bool CalculatePriors() {
    return true;
  }
  
  bool AddPriors() {
    return true;
  }
  
  /*bool AddDataToProblem() {
    int64_t problem_data_start_ts = swf_iterations_.DataBeginTimestamp();
    
    // Locate the PreProblemBegin marker for inter-state residuals
    if (!inter_target_pose_residuals_->PositionFixedPointBeforeTimestamp(
          "PreProblemBegin", problem_data_start_ts, true)) {
      LOG(ERROR) << "Could not locate the problem_data_start_ts = " << problem_data_start_ts;
      return false;
    }
    VLOG(2) << "inter_target_pose_residuals_: PreProblemBegin marker: " <<
        inter_target_pose_residuals_->FixedPointToString("PreProblemBegin");
    // Add inter state prior to the problem
    if (true) {
      auto fp = target_poses_->GetFixedPoint("ProblemBeginMarker");
      KinematicState* _kin = target_poses_->MutableElementAtFixedPoint(fp);
      kinematic_state_prior_.Reset();
      if (!kinematic_state_prior_.Create(_kin)) {
        LOG(ERROR) << "Could not create the prior";
        return false;
      }
      kinematic_state_prior_.AddToProblem(problem_.get());
      VLOG(1) << "Added a prior to the problem with state at queue fp: "
          << target_poses_->FixedPointToString(fp)
          << " K0 timestamp: " << _kin->Timestamp();
    }
    // Add inter-state residuals to the problem
    for (auto fp = inter_target_pose_residuals_->GetNextFixedPoint("PreProblemBegin");
         inter_target_pose_residuals_->IsNotPastFixedPoint("ProblemEnd", fp);
         inter_target_pose_residuals_->IncrementFixedPoint(fp)) {
      inter_target_pose_residuals_->MutableElementAtFixedPoint(fp)->AddToProblem(problem_.get());
      iteration_record_.iteration_counters["InterTargetResidualsAdded"]++;
    }
    
    // Locate the PreProblemBegin marker for tag view residuals
    if (!tag_view_residuals_->PositionFixedPointBeforeTimestamp(
          "PreProblemBegin", problem_data_start_ts, true)) {
      LOG(ERROR) << "Could not locate the problem_data_start_ts = " << problem_data_start_ts;
      return false;
    }
    VLOG(2) << "tag_view_residuals_: PreProblemBegin marker: " <<
        tag_view_residuals_->FixedPointToString("PreProblemBegin");
    // Add Tag view residuals to the problem
    for (auto fp = tag_view_residuals_->GetNextFixedPoint("PreProblemBegin");
         tag_view_residuals_->IsNotPastFixedPoint("ProblemEnd", fp);
         tag_view_residuals_->IncrementFixedPoint(fp)) {
      tag_view_residuals_->MutableElementAtFixedPoint(fp)->AddToProblem(problem_.get());
      iteration_record_.iteration_counters["TagViewResidualsAdded"]++;
    }
    
    return true;
  }*/

  bool AddDataToProblem() {
    int64_t problem_data_start_ts = swf_iterations_.DataBeginTimestamp();
    
    // Locate the PreProblemBegin marker for spline_pose_residuals_
    if (!spline_pose_residuals_->PositionFixedPointBeforeTimestamp(
          "PreProblemBegin", problem_data_start_ts, true)) {
      LOG(ERROR) << "Could not locate the problem_data_start_ts = " << problem_data_start_ts;
      return false;
    }
    VLOG(2) << "spline_pose_residuals_: PreProblemBegin marker: " <<
        spline_pose_residuals_->FixedPointToString("PreProblemBegin");
    
    // Add spline_pose_residuals_ to the problem
    for (auto fp = spline_pose_residuals_->GetNextFixedPoint("PreProblemBegin");
         spline_pose_residuals_->IsNotPastFixedPoint("ProblemEnd", fp);
         spline_pose_residuals_->IncrementFixedPoint(fp)) {
      spline_pose_residuals_->MutableElementAtFixedPoint(fp)->AddToProblem(problem_.get());
      iteration_record_.iteration_counters["SplinePoseResidualsAdded"]++;
    }
    
    // Locate the PreProblemBegin marker for spline_pose_priors_
    if (!spline_pose_priors_->PositionFixedPointBeforeTimestamp(
          "PreProblemBegin", problem_data_start_ts, true)) {
      LOG(ERROR) << "Could not locate the problem_data_start_ts = " << problem_data_start_ts;
      return false;
    }
    VLOG(2) << "spline_pose_priors_: PreProblemBegin marker: " <<
        spline_pose_priors_->FixedPointToString("PreProblemBegin");
    
    // Add inter-state residuals to the problem
    for (auto fp = spline_pose_priors_->GetNextFixedPoint("PreProblemBegin");
         spline_pose_priors_->IsNotPastFixedPoint("ProblemEnd", fp);
         spline_pose_priors_->IncrementFixedPoint(fp)) {
      spline_pose_priors_->MutableElementAtFixedPoint(fp)->AddToProblem(problem_.get());
      iteration_record_.iteration_counters["SplinePosePriorsAdded"]++;
    }
    
    return true;
  }
  
  // Mark states in the problem before the solving window as constant
  /*bool MarkStatesConstant() {
    int64_t window_start_ts = swf_iterations_.SlidingWindowTimestamp();
    
    // Locate the PreWindowBegin marker for inter_target_pose_residuals_
    if (!inter_target_pose_residuals_->PositionFixedPointBeforeTimestamp(
          "PreWindowBegin", window_start_ts, true)) {
      LOG(WARNING) << "Could not locate the window_start_ts for marking "
          << "constant states in residuals queue = " << window_start_ts;
      return false;
    }
    VLOG(2) << "Solving window start ts was found. PreWindowBegin marker = " <<
        inter_target_pose_residuals_->FixedPointToString("PreWindowBegin");
    
    // Mark tag view states constant
    for (auto fp = inter_target_pose_residuals_->GetNextFixedPoint("PreProblemBegin");
         inter_target_pose_residuals_->IsNotPastFixedPoint("PreWindowBegin", fp);
         inter_target_pose_residuals_->IncrementFixedPoint(fp)) {
      inter_target_pose_residuals_->MutableElementAtFixedPoint(fp)->MarkK0Constant(problem_.get());
      iteration_record_.iteration_counters["InterTargetResidualsMarked"]++;
    }
    
    // Locate the PreWindowBegin marker for tag_view_residuals_
    if (!tag_view_residuals_->PositionFixedPointBeforeTimestamp(
          "PreWindowBegin", window_start_ts, true)) {
      LOG(WARNING) << "Could not locate the window_start_ts in tag_view_residuals_ "
          << window_start_ts;
      return false;
    }
    VLOG(2) << "Solving window start ts was found. PreWindowBegin marker = " <<
        tag_view_residuals_->FixedPointToString("PreWindowBegin");
    
    return true;
  }*/
  
  // Mark states in the problem before the solving window as constant
  bool MarkStatesConstant() {
    int64_t window_start_ts = swf_iterations_.SlidingWindowTimestamp();
    
    // Locate the PreWindowBegin marker for spline_pose_residuals_
    if (!spline_pose_residuals_->PositionFixedPointBeforeTimestamp(
          "PreWindowBegin", window_start_ts, true)) {
      LOG(WARNING) << "Could not locate the window_start_ts for marking "
          << "constant states in spline_pose_residuals_ queue = " << window_start_ts;
      return false;
    }
    VLOG(2) << "spline_pose_residuals_ solving window start ts was found. PreWindowBegin marker = " <<
        spline_pose_residuals_->FixedPointToString("PreWindowBegin");
    
    // Mark tag view states constant
    for (auto fp = spline_pose_residuals_->GetNextFixedPoint("PreProblemBegin");
         spline_pose_residuals_->IsNotPastFixedPoint("PreWindowBegin", fp);
         spline_pose_residuals_->IncrementFixedPoint(fp)) {
      //spline_pose_residuals_->MutableElementAtFixedPoint(fp)->MarkK0Constant(problem_.get());
      iteration_record_.iteration_counters["SplinePoseResidualsMarked"]++;
    }
    
    // Locate the PreWindowBegin marker for spline_pose_priors_
    if (!spline_pose_priors_->PositionFixedPointBeforeTimestamp(
          "PreWindowBegin", window_start_ts, true)) {
      LOG(WARNING) << "Could not locate the window_start_ts for marking "
          << "constant states in spline_pose_priors_ queue = " << window_start_ts;
      return false;
    }
    VLOG(2) << "spline_pose_priors_ solving window start ts was found. PreWindowBegin marker = " <<
        spline_pose_priors_->FixedPointToString("PreWindowBegin");
    
    // Mark tag view states constant
    for (auto fp = spline_pose_priors_->GetNextFixedPoint("PreProblemBegin");
         spline_pose_priors_->IsNotPastFixedPoint("PreWindowBegin", fp);
         spline_pose_priors_->IncrementFixedPoint(fp)) {
      //spline_pose_priors_->MutableElementAtFixedPoint(fp)->MarkK0Constant(problem_.get());
      iteration_record_.iteration_counters["SplinePosePriorsMarked"]++;
    }
    
    return true;
  }
  
  /*bool AddNewDataToProblem() {
    
    // Inter-state residuals
    for (auto fp = inter_target_pose_residuals_->GetNextFixedPoint("ProblemEnd");
         inter_target_pose_residuals_->IsNotPastTheEnd(fp); 
         //inter_target_pose_residuals_->IsNotAtTheEnd(fp);   // Here we avoid last residual
         inter_target_pose_residuals_->IncrementFixedPoint(fp)) {
      inter_target_pose_residuals_->MutableElementAtFixedPoint(fp)->AddToProblem(problem_.get());
      iteration_record_.iteration_counters["InterTargetResidualsAdded"]++;
    }
    // Reset the ProblemEnd marker
    inter_target_pose_residuals_->SetFixedPoint("ProblemEnd"); // Sets fixed point at the end
    VLOG(2) << "InterStateResiduals problem end = " << inter_target_pose_residuals_->FixedPointToString("ProblemEnd");
    
    // Tag view residuals
    for (auto fp = tag_view_residuals_->GetNextFixedPoint("ProblemEnd");
         tag_view_residuals_->IsNotPastTheEnd(fp);
         tag_view_residuals_->IncrementFixedPoint(fp)) {
      //tag_view_residuals_->MutableElementAtFixedPoint(fp)->AddToProblem(problem_.get());
      //iteration_record_.iteration_counters["TagViewResidualsAdded"]++;
    }
    // Reset the ProblemEnd marker
    tag_view_residuals_->SetFixedPoint("ProblemEnd"); // Sets fixed point at the end
    VLOG(2) << "TagViewResiduals problem end = " << tag_view_residuals_->FixedPointToString("ProblemEnd");
    
    // Tag pose priors
    for (auto fp = tag_pose_priors_->GetNextFixedPoint("ProblemEnd");
         tag_pose_priors_->IsNotPastTheEnd(fp);
         tag_pose_priors_->IncrementFixedPoint(fp)) {
      tag_pose_priors_->MutableElementAtFixedPoint(fp)->AddToProblem(problem_.get());
      iteration_record_.iteration_counters["TargetPosePriorsAdded"]++;
    }
    // Reset the ProblemEnd marker
    tag_pose_priors_->SetFixedPoint("ProblemEnd"); // Sets fixed point at the end
    VLOG(2) << "tag_pose_priors_ problem end = " << tag_pose_priors_->FixedPointToString("ProblemEnd");
    
    return true;
  }*/
  
  bool AddNewDataToProblem() {
    
    // spline_pose_residuals_
    for (auto fp = spline_pose_residuals_->GetNextFixedPoint("ProblemEnd");
         spline_pose_residuals_->IsNotPastTheEnd(fp); 
         //spline_pose_residuals_->IsNotAtTheEnd(fp);   // Here we avoid last residual
         spline_pose_residuals_->IncrementFixedPoint(fp)) {
      spline_pose_residuals_->MutableElementAtFixedPoint(fp)->AddToProblem(problem_.get());
      iteration_record_.iteration_counters["SplinePoseResidualsAdded"]++;
    }
    // Reset the ProblemEnd marker
    spline_pose_residuals_->SetFixedPoint("ProblemEnd"); // Sets fixed point at the end
    VLOG(2) << "spline_pose_residuals problem end = " << spline_pose_residuals_->FixedPointToString("ProblemEnd");
    
    // spline_pose_priors_
    for (auto fp = spline_pose_priors_->GetNextFixedPoint("ProblemEnd");
         spline_pose_priors_->IsNotPastTheEnd(fp); 
         //spline_pose_priors_->IsNotAtTheEnd(fp);   // Here we avoid last residual
         spline_pose_priors_->IncrementFixedPoint(fp)) {
      spline_pose_priors_->MutableElementAtFixedPoint(fp)->AddToProblem(problem_.get());
      iteration_record_.iteration_counters["SplinePosePriorsAdded"]++;
    }
    // Reset the ProblemEnd marker
    spline_pose_priors_->SetFixedPoint("ProblemEnd"); // Sets fixed point at the end
    VLOG(2) << "spline_pose_priors problem end = " << spline_pose_priors_->FixedPointToString("ProblemEnd");
    
    return true;
  }
  
  bool SolveProblem() {
    // Checks
    if (!problem_) {LOG(ERROR) << "Problem is null!"; return false;}
    
    // Solve the problem
    ceres::Solve(solver_options_, problem_.get(), &solver_summary_);
    VLOG(1) << solver_summary_.BriefReport();
    VLOG(1) << "Num residual blocks reduced: " << solver_summary_.num_residual_blocks_reduced
        << " parameter blocks reduced: " << solver_summary_.num_parameter_blocks_reduced;
    //if (false) std::cout << solver_summary_.FullReport() << std::endl;
    
    return true;
  }
  
  /*bool MarkNewStatesConstant() {
    // Make a copy of the PreWindowBegin fixed point
    const auto last_pre_window_begin =
        inter_target_pose_residuals_->GetFixedPoint("PreWindowBegin");
    
    // Locate the PreWindowBegin marker for inter_target_pose_residuals_
    int64_t window_start_ts = swf_iterations_.SlidingWindowTimestamp();
    if (!inter_target_pose_residuals_->PositionFixedPointBeforeTimestamp(
          "PreWindowBegin", window_start_ts, true)) {
      LOG(WARNING) << "Could not locate the window_start_ts = " << window_start_ts;
      return false;
    }
    VLOG(2) << "inter_target_pose_residuals_ window start ts was found. PreWindowBegin marker = " <<
        inter_target_pose_residuals_->FixedPointToString("PreWindowBegin");
    
    // Mark tag view states constant
    for (auto fp = inter_target_pose_residuals_->NextFixedPoint(last_pre_window_begin);
         inter_target_pose_residuals_->IsNotPastFixedPoint("PreWindowBegin", fp);
         inter_target_pose_residuals_->IncrementFixedPoint(fp)) {
      inter_target_pose_residuals_->MutableElementAtFixedPoint(fp)->MarkK0Constant(problem_.get());
      iteration_record_.iteration_counters["InterTargetResidualsMarked"]++;
    }
    
    // Locate the PreWindowBegin marker for tag_view_residuals_
    if (!tag_view_residuals_->PositionFixedPointBeforeTimestamp(
          "PreWindowBegin", window_start_ts, true)) {
      LOG(WARNING) << "Could not locate the window_start_ts in tag_view_residuals_ "
          << window_start_ts;
      return false;
    }
    VLOG(2) << "tag_view_residuals_ window start ts was found. PreWindowBegin marker = " <<
        tag_view_residuals_->FixedPointToString("PreWindowBegin");
    
    // Make a copy of the PreWindowBegin fixed point
    const auto last_priors_pre_window_begin = tag_pose_priors_->GetFixedPoint("PreWindowBegin");
    if (!tag_pose_priors_->PositionFixedPointBeforeTimestamp("PreWindowBegin", window_start_ts, true)) {
      LOG(WARNING) << "Could not locate the window_start_ts = " << window_start_ts;
      return false;
    }
    VLOG(2) << "tag_pose_priors_ window start ts was found. PreWindowBegin marker = " <<
        tag_pose_priors_->FixedPointToString("PreWindowBegin");
    // Mark tag view states constant
    for (auto fp = tag_pose_priors_->NextFixedPoint(last_priors_pre_window_begin);
         tag_pose_priors_->IsNotPastFixedPoint("PreWindowBegin", fp);
         tag_pose_priors_->IncrementFixedPoint(fp)) {
      tag_pose_priors_->MutableElementAtFixedPoint(fp)->MarkK0Constant(problem_.get());
      iteration_record_.iteration_counters["TargetPosePriorsMarked"]++;
    }
    
    return true;
  }*/
  
  bool MarkNewStatesConstant() {
    int64_t window_start_ts = swf_iterations_.SlidingWindowTimestamp();
    
    // Locate the PreWindowBegin marker for spline_pose_residuals_
    if (!spline_pose_residuals_->PositionFixedPointBeforeTimestamp(
          "PreWindowBegin", window_start_ts, true)) {
      LOG(WARNING) << "Could not locate the window_start_ts for spline_pose_residuals_ = " << window_start_ts;
      return false;
    }
    VLOG(2) << "spline_pose_residuals_ window start ts was found. PreWindowBegin marker = " <<
        spline_pose_residuals_->FixedPointToString("PreWindowBegin");
    
    // Make a copy of the PreWindowBegin fixed point
    const auto last_pre_window_begin =
        spline_pose_residuals_->GetFixedPoint("PreWindowBegin");
    
    // Mark tag view states constant
    for (auto fp = spline_pose_residuals_->NextFixedPoint(last_pre_window_begin);
         spline_pose_residuals_->IsNotPastFixedPoint("PreWindowBegin", fp);
         spline_pose_residuals_->IncrementFixedPoint(fp)) {
      //spline_pose_residuals_->MutableElementAtFixedPoint(fp)->MarkK0Constant(problem_.get());
      iteration_record_.iteration_counters["SplinePoseResidualsMarked"]++;
    }
    
    // Locate the PreWindowBegin marker for spline_pose_priors_
    if (!spline_pose_priors_->PositionFixedPointBeforeTimestamp(
          "PreWindowBegin", window_start_ts, true)) {
      LOG(WARNING) << "Could not locate the window_start_ts for spline_pose_priors_ = " << window_start_ts;
      return false;
    }
    VLOG(2) << "spline_pose_priors_ window start ts was found. PreWindowBegin marker = " <<
        spline_pose_priors_->FixedPointToString("PreWindowBegin");
    
    // Make a copy of the PreWindowBegin fixed point
    const auto last_pre_window_begin_priors =
        spline_pose_priors_->GetFixedPoint("PreWindowBegin");
    
    // Mark tag view states constant
    for (auto fp = spline_pose_priors_->NextFixedPoint(last_pre_window_begin_priors);
         spline_pose_priors_->IsNotPastFixedPoint("PreWindowBegin", fp);
         spline_pose_priors_->IncrementFixedPoint(fp)) {
      //spline_pose_priors_->MutableElementAtFixedPoint(fp)->MarkK0Constant(problem_.get());
      iteration_record_.iteration_counters["SplinePosePriorsMarked"]++;
    }
    
    return true;
  }
  
  // After solving, some states and their residuals need to be updated.
  bool UpdateStatesAndResiduals(const int64_t& iteration_end_ts) {
    
    if (!cubic_pose_spline_->ExtendSpline(iteration_end_ts)) {
      LOG(FATAL) << "Could not extend cubic pose spline to ts " << iteration_end_ts;
    }
    
    return true;
  }
  
  // After solving, some states and their residuals need to be updated.
  /*bool UpdateStatesAndResiduals(const int64_t& iteration_end_ts) {
    // Update the ending kinematic state and the corresponding interstate residual
    KinematicState* kin_state = target_poses_->BackPtr();
    InterKinematicStateResidual* inter_state_resid =
        inter_target_pose_residuals_->MutableElementAtFixedPoint("ProblemEnd");
    
    // Check that ending state and ending residual coincide
    if (inter_state_resid->K1_ != kin_state) {
      LOG(ERROR) << "Problem end residual's K1 is not equal to ending state. Should be.";
      return false;
    }
    
    // Update the ending state residual for ending state
    inter_state_resid->ResetK1ErrorToZero();
    
    // Update the ending state
    if (!kin_state->Recalculate()) {
      LOG(ERROR) << "Could not recalculate ending state";
    }
    if (!kin_state->SetCovariance(problem_.get())) {
      LOG(ERROR) << "Could not set covariance of ending state";
    }
    
    return true;
  }*/
  
  // After solving, some states and their residuals need to be updated.
  /*bool UpdateStatesAndResiduals00(const int64_t& iteration_end_ts) {
    
    ////// Update states using inter-state residuals solved
    //VLOG(1) << "Updating states using inter-state residuals after solving.";
    //for (auto fp = inter_target_pose_residuals_->GetNextFixedPoint("PreWindowBegin");
    //     inter_target_pose_residuals_->IsNotPastTheEnd(fp);
    //     inter_target_pose_residuals_->IncrementFixedPoint(fp)) {
    //  inter_target_pose_residuals_->MutableElementAtFixedPoint(fp)->UpdateK1Errors();
    //  iteration_record_.iteration_counters["TargetStatesUpdated"]++;
    //}
    
    //// Create and predict new state, add a new residual
    
    const int64_t iteration_end_state_ts =
        ((iteration_end_ts / inter_state_interval_) +1) * inter_state_interval_;
    const int64_t last_state_ts =
        target_poses_->Back().timestamp_;
    const int32_t num_states_to_create =
        (iteration_end_state_ts - last_state_ts) / inter_state_interval_;
    VLOG(2) << "  States last, iter end, states end, num = " << last_state_ts << " "
        << iteration_end_ts << " " << iteration_end_state_ts << " " << num_states_to_create;
        
    for (int i=0; i<num_states_to_create; i++) {
      
      // Create a new state
      anantak::KinematicState* last_tgt_state = target_poses_->BackPtr();
      anantak::KinematicState* tgt_state = target_poses_->NextMutableElement();
      tgt_state->SetZero();  // Just to be safe
      
      // Predict the new state
      int64_t tgt_state_ts = last_tgt_state->Timestamp() + inter_state_interval_;
      //if (false) {
      //  // Predict new state using spline
      //  if (!spline.Interpolate(tgt_state_ts, tgt_state, last_tgt_state)) {
      //    LOG(FATAL) << "Can not interpolate kinematic state";
      //    return false;
      //  }
      //} else {
        // Predict the new state using kinematic motion
        KinematicState _K0(*last_tgt_state);
        _K0.Recalculate();
        if (!MoveInFixedTime(_K0, inter_state_interval_, tgt_state)) {
          LOG(FATAL) << "Can not move forward in time from state";
          return false;          
        }
      //}
      
      // Assign timestamp on the queues
      if (tgt_state->timestamp_ <= last_tgt_state->timestamp_) {
        LOG(ERROR)<<"tgt_state->timestamp_ <= last_tgt_state->timestamp_ "
            << tgt_state->timestamp_ << " " << last_tgt_state->timestamp_;
        return false;
      }
      target_poses_->SetTimestamp(tgt_state->timestamp_);
      iteration_record_.iteration_counters["TargetPosesCreated"]++;
      
      // Create a new inter-state residual
      anantak::InterKinematicStateResidual* kin_state_resid =
          inter_target_pose_residuals_->NextMutableElement();
      kin_state_resid->Reset(); // Just to be safe
      
      // Create the inter-state residual
      if (!kin_state_resid->Create(last_tgt_state, tgt_state, inter_state_interval_)) {
        LOG(FATAL) << "Could not create inter state residual. Can not continue.";
        return false;
      }
      inter_target_pose_residuals_->SetTimestamp(last_tgt_state->timestamp_);
      iteration_record_.iteration_counters["InterTargetResidualsCreated"]++;
      
      // Report projected state
      VLOG(1) << "Target predicted state: " << tgt_state->ToString(2);
      
      // Check for mistakes
      if (tgt_state->timestamp_ > iteration_end_state_ts) {
        LOG(FATAL) << "tgt_state->timestamp_ > iteration_end_state_ts. Should not happen! Quit. "
            << tgt_state->timestamp_ << " " << iteration_end_state_ts;
        return false;
      }
    }
    
    return true;
  }*/
  
  // Destructor
  virtual ~CameraCalibrationTargetMotionFilter() {}
};  // CameraCalibrationTargetMotionFilter


} // namespace