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
  
  // Options for the CameraCalibrationTargetMotionFilter
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
    //uint16_t max_tag_camera_frequency;    // Maximum rate of images from tag cameras (Hz)
    //uint16_t max_tags_per_image;          // Maximum number of images seen in an image
    
    // Calibration target specifications
    std::string calibration_target_config_file;
    
    // Kinematic model fit error specification
    int32_t num_pose_constraints_per_state;  // Number of pose constraints to create per state
    
    //  Process noise
    double sigma_eq;  // sqrt var of spline's rotation fits to target's trajectory
    double sigma_ep;  // sqrt var of spline's position fits to target's trajectory
    double sigma_prior_mult;  // this * above sigmas serve as sqrt variance of control pose priors
    
    // Maximum number of iterations with no observations before filter resets
    int32_t max_num_iterations_with_no_observations; 
    
    Options(const std::string& config_filename=""):
      iteration_interval(          100000),
      
      states_history_interval(   20000000),
      longest_problem_interval(   5000000),
      shortest_problem_interval(  3000000),
      sliding_window_interval(    1000000),
      solving_problem_interval(    100000),
      states_frequency(                10),
      
      sliding_window_options(
        longest_problem_interval,     // longest problem length
        shortest_problem_interval,    // shortest problem length
        sliding_window_interval,      // sliding window length
        solving_problem_interval      // solving problem interval
      ),
      
      tag_camera_ids({0}),
      //max_tag_camera_frequency(30),
      //max_tags_per_image(35),
      
      calibration_target_config_file("config/camera_intrisics_apriltag_calibration_target.cfg"),
      
      num_pose_constraints_per_state(1),
      
      sigma_eq(2.*kRadiansPerDegree),   // rad 
      sigma_ep(0.02),                    // m
      sigma_prior_mult(1.),
      
      max_num_iterations_with_no_observations(5)
    {
      if (config_filename=="") {
        LOG(INFO) << "Created CameraCalibrationTargetMotionFilter Options using default parameters";        
      }
      else {
        // Read the Programs Setup file
        std::string config_file_path = anantak::GetProjectSourceDirectory() + "/" + config_filename;
        VLOG(1) << "Config file = " << config_file_path;
        
        //std::unique_ptr<anantak::CameraCalibratorConfig> config =
        //    anantak::ReadProtobufFile<anantak::CameraCalibratorConfig>(config_file_path);
        //if (!config) {
        //  LOG(ERROR) << "Could not parse the config file resorting to default";
        ///} else {
        ///}
        
      }  // if config filename is provided
    }
    
    // Accessors
    uint64_t IterationInterval() const {return iteration_interval;}
    
  };
  
  // Display for the filter - owns all display/drawing code
  struct Display {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    // Main canvas
    cv::Size size_;
    cv::Mat display_image_;
    std::string name_;
    
    bool initialized_;
    bool window_showing_;
    
    Display(const std::string& name):
        name_(name), size_(0,0), display_image_(), initialized_(false), window_showing_(false)
    {
      VLOG(1) << "Created CameraCalibrationTargetMotion filter display. Yet to be initialized.";
    }
    
    bool SetSize(const int32_t& width, const int32_t& height) {
      size_ = cv::Size(width, height);
      display_image_.create(size_, CV_8UC3);
      initialized_ = true;
      VLOG(1) << "Initialized filter display with width, height: " << width << " " << height;
      return true;
    }
    
    bool ShowWindow(const int32_t& location_x=0, const int32_t& location_y=0) {
      if (!initialized_) return false;
      window_showing_ = true;
      cv::namedWindow(name_, cv::WINDOW_AUTOSIZE ); // Create a window for display
      cv::moveWindow(name_, location_x, location_y); // move window to a new positon
      return true;
    }
    
    bool HideWindow() {
      if (!initialized_) return false;
      window_showing_ = false;
      cv::destroyWindow(name_); // destroy the display window
      cv::waitKey(10);
      cv::destroyWindow(name_); // destroy the display window
      cv::waitKey(10);
      cv::destroyWindow(name_); // destroy the display window
      cv::waitKey(10);
      cv::destroyWindow(name_); // destroy the display window
      cv::waitKey(10);
      cv::destroyWindow(name_); // destroy the display window
      cv::waitKey(10);                
      return true;
    }
    
    bool DisplayImage() {
      if (!initialized_ || !window_showing_) return false;
      cv::imshow(name_, display_image_);
      cv::waitKey(1);
    }
    
    // Clear image
    bool ClearImage() {
      display_image_ = CV_BLACK;
      return true;
    }
    
    // Draw a message
    bool ShowMessage(const std::string msg) {
      cv::Point2d p0(size_.width*0.1, size_.height*0.1);
      cv::putText(display_image_, msg, p0, CV_FONT_NORMAL, .5, CV_WHITE, 1);      
      return true;
    }
    
    // Draw Apriltags from a AprilTag message
    bool DrawAprilTags(const anantak::AprilTagMessage& apriltag_msg) {
      if (!initialized_) return false;
      if (apriltag_msg.tag_id_size() == 0) {
        LOG(ERROR) << "No tag in april tag message. Not expacted.";
        return false;
      }
      // Go though each tag, draw it
      for (int i_tag=0; i_tag<apriltag_msg.tag_id_size(); i_tag++) {
        const std::string& id = apriltag_msg.tag_id(i_tag);
        cv::Point2d p0(apriltag_msg.u_1(i_tag), apriltag_msg.v_1(i_tag));
        cv::Point2d p1(apriltag_msg.u_2(i_tag), apriltag_msg.v_2(i_tag));
        cv::Point2d p2(apriltag_msg.u_3(i_tag), apriltag_msg.v_3(i_tag));
        cv::Point2d p3(apriltag_msg.u_4(i_tag), apriltag_msg.v_4(i_tag));
        cv::line(display_image_, p0, p1, CV_YELLOW, 1);
        cv::line(display_image_, p1, p2, CV_YELLOW, 1);
        cv::line(display_image_, p2, p3, CV_YELLOW, 1);
        cv::line(display_image_, p3, p0, CV_YELLOW, 1);
        //cv::line(display_image_, p0, p2, CV_YELLOW, 1);
        //cv::line(display_image_, p1, p3, CV_YELLOW, 1);
        //cv::circle(display_image_, p0, 3, CV_GREEN, 1);
        //cv::circle(display_image_, p1, 3, CV_RED, 2);
        //cv::circle(display_image_, p2, 3, CV_BLUE, 2);
        //cv::circle(display_image_, p3, 3, CV_BLACK, 2);
        //cv::putText(display_image_, id, cv::Point(dd.cxy[0],dd.cxy[1]),
        //    CV_FONT_NORMAL, .5, CV_WHITE, 4);
        //cv::putText(display_image_, id, cv::Point(dd.cxy[0],dd.cxy[1]),
        //    CV_FONT_NORMAL, .5, CV_BLUE, 1);
      }
      
      return true;
    }
    
  };  // Display
  
  // Options
  CameraCalibrationTargetMotionFilter::Options options_;
  
  // Display
  CameraCalibrationTargetMotionFilter::Display display_;
  bool show_;   // true: show the display, false: hide the display
  
  // Running mode
  enum FilterRunMode {kInitializing, kFiltering};
  FilterRunMode filter_run_mode_;
  int32_t num_consecutive_iterations_with_zero_observations_;
  
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
  
  // Linear pose spline of target observations
  std::unique_ptr<anantak::NonUniformLinearPoseSpline> linear_observations_spline_;
  
  // Cubic pose B-spline
  std::unique_ptr<anantak::UniformCubicPoseSpline> cubic_pose_spline_;
  anantak::PoseSpline_Options cubic_pose_spline_options_;
  
  // Spline pose residuals queue
  int64_t inter_pose_constraint_interval_;
  std::unique_ptr<anantak::TimedCircularQueue<anantak::PoseSpline_PoseResidual>>
      spline_pose_residuals_;
  
  // Spline control poses priors queue
  std::unique_ptr<anantak::TimedCircularQueue<anantak::PoseSpline_ControlPosePrior>>
      spline_pose_priors_;
  
  // Spline target view residuals queue
  PoseSpline_CalibrationTargetViewResidual::Options
      spline_target_view_residuals_options_;
  std::unique_ptr<anantak::TimedCircularQueue<anantak::PoseSpline_CalibrationTargetViewResidual>>
      spline_target_view_residuals_;
  
  // Kinematics pose forecast for the end of iteration using the data for the iteration
  KinematicState K_forecast_;
  
  // Optimization objects
  std::unique_ptr<ceres::Problem> problem_;
  ceres::Problem::Options problem_options_;
  ceres::Solver::Options solver_options_;
  ceres::Solver::Summary solver_summary_;
  
  // Initializing target poses
  std::vector<PoseState> starting_poses_;
  
  // Iteration messages
  std::vector<anantak::SensorMsg> iteration_msgs_;
  
  // Maximum iteration observations timestamp
  int64_t max_iteration_observations_ts_;
  
  CameraCalibrationTargetMotionFilter(
      const CameraCalibrationTargetMotionFilter::Options& options,
      const int64_t& filter_start_ts):
    options_(options), iteration_interval_(options.iteration_interval),
    filter_start_ts_(filter_start_ts),
    iteration_record_(),
    swf_iterations_(options.sliding_window_options),
    target_(options.calibration_target_config_file),
    camera_(options.tag_camera_ids.at(0)),                  // initiating at first camera
    undistorted_camera_(options.tag_camera_ids.at(0)),      // initiating at first camera 
    problem_(nullptr), problem_options_(), solver_options_(), solver_summary_(),
    cubic_pose_spline_options_(options.sigma_eq, options.sigma_ep, options.sigma_prior_mult),
    spline_target_view_residuals_options_(),
    num_consecutive_iterations_with_zero_observations_(0),
    max_iteration_observations_ts_(0),
    display_("CameraCalibrationTargetMotionFilter")
  {
    Initialize();
  }
  
  // Initialize the filter
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
      LOG(FATAL) << "Calibration target was not initialized. Quit.";
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
    
    // Initiate display
    display_.SetSize(camera_.width(), camera_.height());
    
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
    
    iteration_record_.iteration_counters["SplineControlPosesMarked"] = 0;  // Number of control poses marked constant
    iteration_record_.iteration_counters["SplineControlPosesRecalculated"] = 0;  // Number of control poses recalculated
    
    iteration_record_.iteration_counters["SplinePosePriorsCreated"] = 0;
    iteration_record_.iteration_counters["SplinePosePriorsAdded"] = 0;
    iteration_record_.iteration_counters["SplinePosePriorsMarked"] = 0;
    
    iteration_record_.algorithm_counters["ProblemObjects"] = 0;   // Number of problem object created
    
    // Allocation memory for the queues
    AllocateMemory();
    
    return true;
  }
  
  
  bool AllocateMemory() {
    LOG(INFO) << "Allocating memory for the camera intrinsics filter queues";
    
    int32_t num_states_to_keep = options_.states_frequency * options_.states_history_interval / 1000000;
    LOG(INFO) << "  num_states_to_keep = " << num_states_to_keep
        << " at states_frequency = " << options_.states_frequency << " Hz";
    
    // Cubic pose spline object
    std::unique_ptr<anantak::UniformCubicPoseSpline> cubic_pose_spline_ptr(
        new anantak::UniformCubicPoseSpline(num_states_to_keep, inter_state_interval_));
    cubic_pose_spline_ = std::move(cubic_pose_spline_ptr);
    
    // Linear observations spline object
    std::unique_ptr<anantak::NonUniformLinearPoseSpline> linear_pose_spline_ptr(
        new anantak::NonUniformLinearPoseSpline(num_states_to_keep));
    linear_observations_spline_ = std::move(linear_pose_spline_ptr);
    
    inter_pose_constraint_interval_ = inter_state_interval_ / options_.num_pose_constraints_per_state;
    
    // Spline pose residuals
    anantak::PoseSpline_PoseResidual _pose_resid_proto(&cubic_pose_spline_options_);
    std::unique_ptr<anantak::TimedCircularQueue<anantak::PoseSpline_PoseResidual>> spr_ptr(
        new anantak::TimedCircularQueue<anantak::PoseSpline_PoseResidual>(
            num_states_to_keep*options_.num_pose_constraints_per_state, _pose_resid_proto));
    spline_pose_residuals_ = std::move(spr_ptr);
    
    // Spline pose priors
    anantak::PoseSpline_ControlPosePrior _pose_prior_proto(&cubic_pose_spline_options_);
    std::unique_ptr<anantak::TimedCircularQueue<anantak::PoseSpline_ControlPosePrior>> spp_ptr(
        new anantak::TimedCircularQueue<anantak::PoseSpline_ControlPosePrior>(
            num_states_to_keep, _pose_prior_proto));
    spline_pose_priors_ = std::move(spp_ptr);
    
    // Spline target view residuals queue
    anantak::PoseSpline_CalibrationTargetViewResidual
        _target_view_resid_proto(&cubic_pose_spline_options_,
                                 &spline_target_view_residuals_options_);
    std::unique_ptr<anantak::TimedCircularQueue<anantak::PoseSpline_CalibrationTargetViewResidual>>
        stvr_ptr(new anantak::TimedCircularQueue<anantak::PoseSpline_CalibrationTargetViewResidual>(
            num_states_to_keep*options_.num_pose_constraints_per_state, _target_view_resid_proto));
    spline_target_view_residuals_ = std::move(stvr_ptr);
    
    return true;
  }
  
  /** Accessors */
  const uint64_t& IterationInterval() const {return iteration_interval_;}
  
  /** Initializing operations */
  
  bool RunInitializingIteration(const int64_t& iteration_end_ts) {
    // Nothing to do! Just wait for the data to come in.
    return true;
  }  
  
  bool RunInitializingIteration(const int64_t& iteration_end_ts,
      const std::vector<std::unique_ptr<std::vector<anantak::SensorMsg>>>& observations) {
    
    starting_poses_.clear();
    
    // Calculate target's approximate poses from the observations 
    for (int i_file=0; i_file<observations.size(); i_file++) {
      for (int i_msg=0; i_msg<observations.at(i_file)->size(); i_msg++) {
        PoseState _pose;
        const anantak::SensorMsg _msg = observations.at(i_file)->at(i_msg);
        if (CalculateApproxTargetPose(_msg, &_pose)) {
          _pose.SetTimestamp(_msg.header().timestamp());
          starting_poses_.emplace_back(_pose);
        }
        if (starting_poses_.size()>=2) break;
      }
      if (starting_poses_.size()>=2) break;
    }
    
    /*// Collect all AprilTag observations
    //std::vector<anantak::SensorMsg> sensor_msgs;
    //for (int i_file=0; i_file<observations.size(); i_file++) {
    //  for (int i_msg=0; i_msg<observations.at(i_file)->size(); i_msg++) {
    //    const anantak::SensorMsg& msg = observations.at(i_file)->at(i_msg);
    //    if (msg.has_header()) {
    //      if (msg.header().type() == "AprilTags") {
    //        sensor_msgs.emplace_back(msg);
    //      }
    //    }
    //  }
    ///}
    //
    //if (sensor_msgs.size() == 0) {
    //  // Nothing to do. Wait for observations.
    //  return true;
    ///}
    //VLOG(1) << "Got " << sensor_msgs.size() << " AprilTag messages.";
    //
    //// Make a starting guess of the target pose from the first (few) message(s)
    //for (int i=0; i<sensor_msgs.size(); i++) {
    //  PoseState starting_pose;
    //  const int64_t& apriltag_msg_ts = sensor_msgs.at(i).header().timestamp();
    //  const AprilTagMessage& apriltag_msg = sensor_msgs.at(i).april_msg();
    //  anantak::AprilTagMessage undist_apriltag_msg;
    //  undistorted_camera_.Undistort(apriltag_msg, &undist_apriltag_msg);
    //  target_.CalculateTargetPose(undist_apriltag_msg, undistorted_camera_.camera_, &starting_pose);
    //  if (!starting_pose.IsZero()) {
    //    starting_pose.SetTimestamp(apriltag_msg_ts);
    //    starting_poses_.emplace_back(starting_pose);
    /// }
    //  if (starting_poses_.size()>=2) break;
    ///}*/
    
    bool spline_initiated = false;
    //if (starting_poses_.size()==1) {
    //  // Initiate spline using a single pose
    //  if (!cubic_pose_spline_->InitiateSpline(starting_poses_.at(0))) {
    //    LOG(ERROR) << "Could not initiate spline using first pose. Restating";
    //    starting_poses_.clear();
    //    cubic_pose_spline_->Reset();
    ///  } else {
    //    spline_initiated = true;
    ///  }
    ///} else
    if (starting_poses_.size()>=2) {
      // Initiate the spline using first two poses
      if (!cubic_pose_spline_->InitiateSpline(starting_poses_.at(0), starting_poses_.at(1))) {
        LOG(ERROR) << "Could not initiate spline using first two poses. Restating";
        starting_poses_.clear();
        cubic_pose_spline_->Reset();
      } else {
        spline_initiated = true;
      }
    }
    
    if (spline_initiated) {
      if (!SetStartingStatesAndResiduals()) {
        LOG(FATAL) << "Could not set starting states and residuals";
      }
      
      // Switch filter to filtering mode
      filter_run_mode_ = kFiltering;
      LOG(INFO) << "Switched filter from initializing to filtering.";
      return RunIteration(iteration_end_ts, observations);
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
    
    // Prepare the spline_target_view_residuals_ queue for usage
    spline_target_view_residuals_->SetStartingTimestamp(starting_state_ts-1);
    spline_target_view_residuals_->SetFixedPoint("PreProblemBegin");
    spline_target_view_residuals_->SetFixedPoint("ProblemEnd");
    spline_target_view_residuals_->SetFixedPoint("PreWindowBegin");
    
    return true;
  }
  
  // Reset the filter
  inline bool Reset() {
    LOG(INFO) << "Resetting filter";
    filter_run_mode_ = kInitializing;
    //filter_start_ts_ = ??
    //swf_iterations_.StartFiltering(starting_state_ts);
    //iteration_record_.Reset(starting_state_ts);
    starting_poses_.clear();
    linear_observations_spline_->Reset();
    cubic_pose_spline_->Reset();
    spline_pose_residuals_->Clear();
    spline_pose_priors_->Clear();
    spline_target_view_residuals_->Clear();
    if (problem_) problem_.release();
    K_forecast_.SetZero();
    num_consecutive_iterations_with_zero_observations_ = 0;
    return true;
  }
  
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
      const std::vector<std::unique_ptr<std::vector<anantak::SensorMsg>>>& observations,
      const bool save_data = false, const bool use_ending_knot_time = false) {
    
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
    
    if (!RunFiltering(iteration_end_ts, save_data)) {
      LOG(ERROR) << "Could not run filtering operations";
      return false;
    }
    
    if (!RunReporting(iteration_end_ts, use_ending_knot_time)) {
      LOG(ERROR) << "Could not run reporting";
      return false;      
    }
    
    return true;
  }

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
  bool CreateIterationStatesAndResiduals(const int64_t& iteration_end_ts,
      const std::vector<std::unique_ptr<std::vector<anantak::SensorMsg>>>& observations) {
    
    // Collect valid observations
    iteration_msgs_.clear();
    for (int i_file=0; i_file<observations.size(); i_file++) {
      for (int i_msg=0; i_msg<observations.at(i_file)->size(); i_msg++) {
        const anantak::SensorMsg _msg = observations.at(i_file)->at(i_msg);
        if (IsValidTargetViewObservation(_msg)) {
          iteration_msgs_.emplace_back(_msg);
          iteration_record_.iteration_counters["TargetObservations"]++;
        }
      }
    }
    
    // Check if no observations were seen
    int32_t num_observations = iteration_msgs_.size();
    if (num_observations == 0) {
      num_consecutive_iterations_with_zero_observations_++;
      VLOG(1) << "No observation was received. Consecutive zero obs iterations = "
        << num_consecutive_iterations_with_zero_observations_;
    } else {
      num_consecutive_iterations_with_zero_observations_ = 0;
    }
    if (num_consecutive_iterations_with_zero_observations_ >=
        options_.max_num_iterations_with_no_observations) {
      LOG(WARNING) << "Resetting filter as no observations were seen for a while.";
      Reset();
      return false;
    }
    
    // Maximum iteration observations' ts is used to extend spline at the end of the iteration
    max_iteration_observations_ts_ = 0;
    
    // Create a tag view residual for every observation
    for (int i=0; i<num_observations; i++) {
      const anantak::SensorMsg& msg = iteration_msgs_.at(i);
      const int64_t ts = msg.header().timestamp();
      
      // Update maximum observation ts
      if (max_iteration_observations_ts_ < ts) {max_iteration_observations_ts_ = ts;}
      
      // Create the ending residual using the actual observation
      VLOG(1) << "Creating target view residual at ts " << ts;
      anantak::PoseSpline_CalibrationTargetViewResidual* spline_target_view_resid =
          spline_target_view_residuals_->NextMutableElement();
      spline_target_view_resid->Reset();   // Making sure state is fresh
      if (!spline_target_view_resid->Create(msg,
                                            &target_,
                                            &undistorted_camera_,
                                            cubic_pose_spline_.get(),
                                            &camera_)) {
        LOG(ERROR) << "Could not create a spline target view residual for observation. Skip";
        spline_target_view_resid->Reset();
        spline_target_view_residuals_->decrement();
      } else {
        spline_target_view_residuals_->SetTimestamp(ts);
        iteration_record_.iteration_counters["SplinePoseResidualsCreated"]++;
      }
      
    }   // for each observation
    
    // Create priors for control poses created in this iteration
    VLOG(2) << "Creating control pose priors";
    int32_t num_created = 0;
    if (!cubic_pose_spline_->CreateControlPosePriors(spline_pose_priors_.get(), &num_created)) {
      LOG(ERROR) << "Could not create spline pose priors. Skip.";
    } else {
      iteration_record_.iteration_counters["SplinePosePriorsCreated"] += num_created;
    }
    
    return true;
  }

  // Create new states for the iteration
  bool CreateIterationStatesAndResiduals0(const int64_t& iteration_end_ts,
      const std::vector<std::unique_ptr<std::vector<anantak::SensorMsg>>>& observations) {
    
    int32_t num_observations = 0;
    
    // Calculate target's approximate poses from the observations 
    std::vector<PoseState> approx_tgt_poses;
    for (int i_file=0; i_file<observations.size(); i_file++) {
      for (int i_msg=0; i_msg<observations.at(i_file)->size(); i_msg++) {
        PoseState _pose;
        const anantak::SensorMsg _msg = observations.at(i_file)->at(i_msg);
        if (CalculateApproxTargetPose(_msg, &_pose)) {
          _pose.SetTimestamp(_msg.header().timestamp());
          approx_tgt_poses.emplace_back(_pose);
          num_observations++;
        }
      }
    }
    
    // Check if no observations were seen
    if (num_observations == 0) {
      num_consecutive_iterations_with_zero_observations_++;
      VLOG(1) << "No observation was received. Consecutive zero obs iterations = "
        << num_consecutive_iterations_with_zero_observations_;
    } else {
      num_consecutive_iterations_with_zero_observations_ = 0;
    }
    if (num_consecutive_iterations_with_zero_observations_ >=
        options_.max_num_iterations_with_no_observations) {
      LOG(WARNING) << "Resetting filter as no observations were seen for a while.";
      Reset();
      return false;
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
      /*if (last_obs_ts > 0) {
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
      }*/
      
      // Create the ending residual using the actual observation
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
      
    }  // for each observation
    
    // Create priors for control poses created in this iteration
    int32_t num_created = 0;
    if (!cubic_pose_spline_->CreateControlPosePriors(spline_pose_priors_.get(), &num_created)) {
      LOG(ERROR) << "Could not create spline pose priors. Skip.";
    } else {
      iteration_record_.iteration_counters["SplinePosePriorsCreated"] += num_created;
    }
    
    return true;
  }
  
  bool CalculateApproxTargetPose(const anantak::SensorMsg& msg, PoseState* pose) {
    if (!IsValidTargetViewObservation(msg)) {
      VLOG(2) << "Skipping message, it is invalid or is not processed by this filter.";
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
        VLOG(1) << "Camera of this Apriltag message (" << cam_id << ") is not processed by this filter";
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
  bool RunFiltering(const int64_t& iteration_end_ts, const bool save_data = false) {
    
    // Reset problem
    if (!problem_ || swf_iterations_.IsItTimeToResetProblem()) {
      
      // If the problem exists, destroy it first
      if (problem_) {
        
        VLOG(1) << "Calculating priors";
        CalculatePriors();
        
        VLOG(1) << "Resetting the problem.";
        problem_.release();
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
    
    // Save data to disk for plotting and analysis  ** CHECK ** 
    if (save_data) {SaveData("Pre");}
    
    // Solve the problem
    //if (swf_iterations_.IsItTimeToSolveProblem()) {
    if (true) {                                   // Solve in every iteration
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
    
    // Save data to disk for plotting and analysis  ** CHECK ** 
    if (save_data) {SaveData();}
    
    return true;
  }
  
  // Accessor for the Kinematic state forecast
  const KinematicState& KinematicStateForecast() const {return K_forecast_;}
  
  // Create a message from the Kinematic state forecast
  bool GetResultsMessage(anantak::SensorMsg* msg) const {
    K_forecast_.CopyToMessage(msg);
    return true;
  }
  
  // Run reporting at the end of the iteration
  bool RunReporting(const int64_t& iteration_end_ts, bool use_ending_knot_time = false) {
    
    VLOG(1) << "Iteration record " << iteration_record_.IterationCountersToString();
    VLOG(1) << "Algorithm record " << iteration_record_.AlgorithmCountersToString();
    
    // Timestamp to be used for projection
    int64_t interpolation_ts = iteration_end_ts;
    if (use_ending_knot_time) {interpolation_ts = cubic_pose_spline_->EndingTimestamp();}
    
    // Project the pose at the end of the iteration
    cubic_pose_spline_->InterpolatePose(interpolation_ts, &K_forecast_, true);  // true: include errors
    VLOG(1) << "Iteration end pose forecast: \n  " << K_forecast_.ToString(2);
    
    // Show the image if asked to
    if (show_) {
      display_.ClearImage();
      if (iteration_msgs_.size()>0) {
        display_.DrawAprilTags(iteration_msgs_.back().april_msg());
      }
      //display_.DrawPoseState(target_pose, camera_intrinsics_);
      display_.DisplayImage();
    }
    
    // Pose will be transmitted by the filter
    
    return true;
  }
  
  // Save data to disk for plotting and analysis
  bool SaveData(const std::string& prefix="", bool save_all_data = false, bool dont_save = false) {
    
    if (dont_save) return true;    // Disables data saving
    
    bool save_q_as_aa = true;
    
    // Calculate poses at knot points of the spline
    std::vector<Vector39d> kin_states_vec;
    int64_t _ts;
    for (_ts =  cubic_pose_spline_->CurrentStartingTimestamp();
         _ts <= cubic_pose_spline_->CurrentEndingTimestamp();
         _ts += cubic_pose_spline_->KnotDistance()) {
      KinematicState _K;
      cubic_pose_spline_->InterpolatePose(_ts, &_K, true);  // Include errors is set to true
      VLOG(3) << "Interpolated pose: " << _K.ToString(2);
      kin_states_vec.emplace_back(_K.ToVector(save_q_as_aa));
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
    
    if (prefix=="") {
      // Target pose observations
      std::vector<Vector8d> tgt_poses_vec;          // Approximate guess from observation
      for (auto fp = spline_target_view_residuals_->GetNextFixedPoint("PreProblemBegin");
           spline_target_view_residuals_->IsNotPastFixedPoint("ProblemEnd", fp);
           spline_target_view_residuals_->IncrementFixedPoint(fp)) {
        const PoseState& _P = spline_target_view_residuals_->ElementAtFixedPoint(fp).Observation();
        Vector8d vec;
        _P.ToVector(&vec, cubic_pose_spline_->StartingTimestamp(), save_q_as_aa);
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
      
      // Starting pose calculated from the spline
      std::vector<Vector8d> tgt_poses_interp_vec;   // Starting guess from the spline
      for (auto fp = spline_target_view_residuals_->GetNextFixedPoint("PreProblemBegin");
           spline_target_view_residuals_->IsNotPastFixedPoint("ProblemEnd", fp);
           spline_target_view_residuals_->IncrementFixedPoint(fp)) {
        const PoseState& _P = spline_target_view_residuals_->ElementAtFixedPoint(fp).StartingTargetPose();
        Vector8d vec;
        _P.ToVector(&vec, cubic_pose_spline_->StartingTimestamp(), save_q_as_aa);
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
    }
    
    // Control points of cubic spline
    std::vector<Vector8d> cubic_spline_control_poses_vec;
    for (int i=0; i<cubic_pose_spline_->NumControlPoints(); i++) {
      Vector8d vec;
      const PoseState& _P = cubic_pose_spline_->control_poses_->At(i);
      _P.ToVector(&vec, cubic_pose_spline_->StartingTimestamp(), save_q_as_aa);
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
    
    if (prefix=="") {
      std::vector<Vector8d> cubic_spline_state_control_poses_vec;
      for (int i=0; i<cubic_pose_spline_->NumControlPoints(); i++) {
        Vector8d vec;
        const PoseState& _P = cubic_pose_spline_->control_poses_->At(i);
        _P.FirstEstimateToVector(&vec, cubic_pose_spline_->StartingTimestamp(), save_q_as_aa);
        cubic_spline_state_control_poses_vec.emplace_back(vec);
      }
      Eigen::Matrix<double,8,Eigen::Dynamic> cubic_spline_state_control_poses_mat;
      cubic_spline_state_control_poses_mat.resize(8,cubic_spline_state_control_poses_vec.size());
      for (int i=0; i<cubic_spline_state_control_poses_vec.size(); i++) {
        cubic_spline_state_control_poses_mat.col(i) = cubic_spline_state_control_poses_vec[i];
      }
      filename =
        "src/Models/Plots/CubicSplineStateControlPoses." + std::to_string(iteration_record_.iteration_number%10) + ".data.csv";
      WriteMatrixToCSVFile(filename, cubic_spline_state_control_poses_mat.transpose());
    }
    
    return true;
  }
  
  // Save data to disk for plotting and analysis
  bool SaveData0(const std::string& prefix="", bool save_all_data = false, bool dont_save = false) {
    
    if (dont_save) return true;    // Disables data saving
    
    bool save_q_as_aa = true;
    
    // Calculate poses at knot points of the spline
    std::vector<Vector39d> kin_states_vec;
    int64_t _ts;
    for (_ts =  cubic_pose_spline_->StartingTimestamp();
         _ts <= cubic_pose_spline_->EndingTimestamp();
         _ts += cubic_pose_spline_->KnotDistance()) {
      KinematicState _K;
      cubic_pose_spline_->InterpolatePose(_ts, &_K, true);  // Include errors is set to true
      VLOG(3) << "Interpolated pose: " << _K.ToString(2);
      kin_states_vec.emplace_back(_K.ToVector(save_q_as_aa));
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
    
    if (prefix=="") {
      // Target pose observations
      std::vector<Vector8d> tgt_poses_vec;
      for (int i=0; i<linear_observations_spline_->control_poses_->n_msgs(); i++) {
        Vector8d vec; 
        linear_observations_spline_->control_poses_->At(i).ToVector(&vec, cubic_pose_spline_->StartingTimestamp(), save_q_as_aa);
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
      
      // Add interpolations
      std::vector<Vector8d> tgt_poses_interp_vec;
      for (auto fp = spline_pose_residuals_->GetNextFixedPoint("PreProblemBegin");
           spline_pose_residuals_->IsNotPastFixedPoint("ProblemEnd", fp);
           spline_pose_residuals_->IncrementFixedPoint(fp)) {
        const PoseState& _P = spline_pose_residuals_->ElementAtFixedPoint(fp).Observation();
        Vector8d vec;
        _P.ToVector(&vec, cubic_pose_spline_->StartingTimestamp(), save_q_as_aa);
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
    }
    
    // Control points of cubic spline
    std::vector<Vector8d> cubic_spline_control_poses_vec;
    for (int i=0; i<cubic_pose_spline_->NumControlPoints(); i++) {
      Vector8d vec;
      const PoseState& _P = cubic_pose_spline_->control_poses_->At(i);
      _P.ToVector(&vec, cubic_pose_spline_->StartingTimestamp(), save_q_as_aa);
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
    
    if (prefix=="") {
      std::vector<Vector8d> cubic_spline_state_control_poses_vec;
      for (int i=0; i<cubic_pose_spline_->NumControlPoints(); i++) {
        Vector8d vec;
        const PoseState& _P = cubic_pose_spline_->control_poses_->At(i);
        _P.FirstEstimateToVector(&vec, cubic_pose_spline_->StartingTimestamp(), save_q_as_aa);
        cubic_spline_state_control_poses_vec.emplace_back(vec);
      }
      Eigen::Matrix<double,8,Eigen::Dynamic> cubic_spline_state_control_poses_mat;
      cubic_spline_state_control_poses_mat.resize(8,cubic_spline_state_control_poses_vec.size());
      for (int i=0; i<cubic_spline_state_control_poses_vec.size(); i++) {
        cubic_spline_state_control_poses_mat.col(i) = cubic_spline_state_control_poses_vec[i];
      }
      filename =
        "src/Models/Plots/CubicSplineStateControlPoses." + std::to_string(iteration_record_.iteration_number%10) + ".data.csv";
      WriteMatrixToCSVFile(filename, cubic_spline_state_control_poses_mat.transpose());
    }
    
    return true;
  }
  
  
  /* Test message to kinematic state fitting
  //    Calculate the pose of the target from the message,
  //    Create a kinematic state from the calculated pose, add a prior covariance
  //    Create a Kinematic state prior for this state
  //    Create a Target view residual for the message
  //    Add both residuals to a problem, solve
  //    Recalculate kinematic state, then calculate the target view residual
  //    Compare the results of residuals 
  //bool TestKinematicStateToTargetMaths(const anantak::SensorMsg& msg) {
  //  VLOG(1) << "****** Testing target view residual **********";
  //  
  //  const anantak::AprilTagMessage& apriltag_msg = msg.april_msg();
  //  
  //  // Display window
  //  std::string window_name = "TestKinematicStateToTargetMaths";
  //  cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE ); // Create a window for display
  //  cv::moveWindow(window_name, 0, 0); // move window to a new positon
  //  
  //  // Create a display image
  //  cv::Mat image(480, 640, CV_8UC3);
  //  image = 0.;
  //  
  //  // Create undistorted camera
  //  anantak::AprilTagMessage undist_apriltag_msg;
  //  undistorted_camera_.Undistort(apriltag_msg, &undist_apriltag_msg);
  //  PlotAprilTagMessage(apriltag_msg, &image, CV_WHITE);
  //  PlotAprilTagMessage(undist_apriltag_msg, &image, CV_GREEN);
  //  
  //  // Calculate approximate target pose using distorted camera
  //  PoseState dist_target_pose;
  //  target_.CalculateTargetPose(apriltag_msg, camera_, &dist_target_pose);
  //  VLOG(1) << "Calculated target pose using distorted camera: " << dist_target_pose.ToString();
  //  
  //  // Calculate approximate target pose after adjusting for distortions
  //  PoseState target_pose;
  //  target_.CalculateTargetPose(undist_apriltag_msg, undistorted_camera_.camera_, &target_pose);
  //  VLOG(1) << "Calculated target pose: " << target_pose.ToString();
  //  
  //  // Difference in poses above
  //  PoseState d_tgt_pose;
  //  target_pose.DiffPose(dist_target_pose, &d_tgt_pose, nullptr, nullptr);
  //  VLOG(1) << "difference in target poses: \n   " << d_tgt_pose.NormString();
  //  
  //  // Create a Kinematic state from target pose
  //  KinematicState kin;
  //  kin.SetPose(target_pose);
  //  anantak::Vector7d starting_pose_sigmas;
  //  starting_pose_sigmas << 200.*kRadiansPerDegree, 10.00,    // rad, m
  //                          200.*kRadiansPerDegree, 10.00,    // rad/s, m/s
  //                          200.*kRadiansPerDegree, 10.00,    // rad/s^2, m/s^2
  //                          0.;                             // s
  //  kin.SetCovariance(starting_pose_sigmas);
  //  kin.SetTimestamp(msg.header().timestamp());
  //  VLOG(1) << "Staring kinematic state: " << kin.ToString(2);
  //  
  //  // Create a prior
  //  KinematicStatePrior prior;
  //  prior.Create(&kin);
  //  
  //  // Create a target view residual
  //  KinematicCalibrationTargetViewResidual tgt_view_residual(&tag_view_residuals_options_);
  //  if (!tgt_view_residual.Create(&target_, &kin, &camera_, msg)) {
  //    LOG(ERROR) << "Target view residual could not be created.";
  //    return false;
  //  }
  //  
  //  // Create a problem 
  //  ceres::Problem problem(problem_options_);
  //  
  //  // build problem
  //  prior.AddToProblem(&problem);
  //  tgt_view_residual.AddToProblem(&problem);
  //  //ceres::CauchyLoss loss_func(1.0);
  //  //tgt_view_residual.AddToProblem(&problem, &loss_func);
  //  
  //  // Solve problem
  //  ceres::Solve(solver_options_, &problem, &solver_summary_);
  //  VLOG(1) << solver_summary_.BriefReport();
  //  VLOG(1) << "Num residual blocks reduced: " << solver_summary_.num_residual_blocks_reduced
  //      << " parameter blocks reduced: " << solver_summary_.num_parameter_blocks_reduced;
  //  
  //  // Update kinematic state
  //  kin.Recalculate();
  //  kin.SetCovariance(&problem);
  //  VLOG(1) << "Recalculated kinematic state: " << kin.ToString(2);
  //  
  //  // Recalculate the residual
  //  //tgt_view_residual.CalculateResiduals2(msg, nullptr, &image);
  //  
  //  // Show image
  //  cv::imshow(window_name, image);
  //  cv::waitKey(0);
  //  
  //  VLOG(1) << "********** Done testing maths **********";
  //  return true;
  ///}*/
  
  
  bool CalculatePriors() {
    return true;
  }
  
  
  bool AddPriors() {
    return true;
  }
  
  
  bool AddDataToProblem() {
    int64_t problem_data_start_ts = swf_iterations_.DataBeginTimestamp();
    
    //// Locate the PreProblemBegin marker for spline_pose_residuals_
    //if (!spline_pose_residuals_->PositionFixedPointBeforeTimestamp(
    //      "PreProblemBegin", problem_data_start_ts, true)) {
    //  LOG(ERROR) << "Could not locate the problem_data_start_ts = " << problem_data_start_ts;
    //  return false;
    ///}
    //VLOG(2) << "spline_pose_residuals_: PreProblemBegin marker: " <<
    //    spline_pose_residuals_->FixedPointToString("PreProblemBegin");
    //
    //// Add spline_pose_residuals_ to the problem
    //for (auto fp = spline_pose_residuals_->GetNextFixedPoint("PreProblemBegin");
    //     spline_pose_residuals_->IsNotPastFixedPoint("ProblemEnd", fp);
    //     spline_pose_residuals_->IncrementFixedPoint(fp)) {
    //  spline_pose_residuals_->MutableElementAtFixedPoint(fp)->AddToProblem(problem_.get());
    //  iteration_record_.iteration_counters["SplinePoseResidualsAdded"]++;
    ///}
    
    // Locate the PreProblemBegin marker for spline_target_view_residuals_
    if (!spline_target_view_residuals_->PositionFixedPointBeforeTimestamp(
          "PreProblemBegin", problem_data_start_ts, true)) {
      LOG(ERROR) << "Could not locate the problem_data_start_ts = " << problem_data_start_ts;
      return false;
    }
    VLOG(2) << "spline_target_view_residuals_: PreProblemBegin marker: " <<
        spline_target_view_residuals_->FixedPointToString("PreProblemBegin");
    
    // Add spline_target_view_residuals_ to the problem
    for (auto fp = spline_target_view_residuals_->GetNextFixedPoint("PreProblemBegin");
         spline_target_view_residuals_->IsNotPastFixedPoint("ProblemEnd", fp);
         spline_target_view_residuals_->IncrementFixedPoint(fp)) {
      VLOG(2) << "spline_target_view_residuals_ add data to problem fp of resid " << fp.ToString();
          //<< " Address: " << spline_target_view_residuals_->MutableElementAtFixedPoint(fp);
      spline_target_view_residuals_->MutableElementAtFixedPoint(fp)->AddToProblem(problem_.get());
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
  bool MarkStatesConstant() {
    int64_t window_start_ts = swf_iterations_.SlidingWindowTimestamp();
    
    // Locate the PreWindowBegin marker for spline_target_view_residuals_
    if (!spline_target_view_residuals_->PositionFixedPointBeforeTimestamp(
          "PreWindowBegin", window_start_ts, true)) {
      LOG(WARNING) << "Could not locate the window_start_ts for marking "
          << "constant states in spline_target_view_residuals_ queue = " << window_start_ts;
      return false;
    }
    VLOG(2) << "spline_target_view_residuals_ solving window start ts was found. PreWindowBegin marker = " <<
        spline_target_view_residuals_->FixedPointToString("PreWindowBegin");
    
    // Get timestamp for the earliest state that is part of this problem
    auto starting_spline_target_view_residuals_fp = spline_target_view_residuals_->GetNextFixedPoint("PreProblemBegin");
    auto ending_spline_target_view_residuals_fp = spline_target_view_residuals_->GetNextFixedPoint("PreWindowBegin");
    int64_t constant_control_pose_begin_ts =     // constant states include this ts
        spline_target_view_residuals_->ElementAtFixedPoint(starting_spline_target_view_residuals_fp).EarliestControlPoseTimestamp();
    int64_t variable_control_pose_begin_ts =     // variable states begin here, constant states exclude this ts
        spline_target_view_residuals_->ElementAtFixedPoint(ending_spline_target_view_residuals_fp).EarliestControlPoseTimestamp();
    VLOG(1) << "Marking all control poses in range constant [" << constant_control_pose_begin_ts
        << " " << variable_control_pose_begin_ts << ")";
    
    // Mark control poses constant from(including) constant_control_pose_begin_ts to(exculding) variable_control_pose_begin_ts
    int32_t num_marked_constant = 0;
    if (!cubic_pose_spline_->MarkControlPosesConstant(constant_control_pose_begin_ts,
                                                      variable_control_pose_begin_ts,
                                                      problem_.get(), &num_marked_constant)) {
      LOG(ERROR) << "Could not mark control poses constant between ["
          << constant_control_pose_begin_ts << " "
          << variable_control_pose_begin_ts << ")";
      return false;
    }
    iteration_record_.iteration_counters["SplineControlPosesMarked"] += num_marked_constant;
    
    return true;
  }
  
  // Mark states in the problem before the solving window as constant
  bool MarkStatesConstant0() {
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
    
    // Get timestamp for the earliest state that is part of this problem
    auto starting_spline_pose_residuals_fp = spline_pose_residuals_->GetNextFixedPoint("PreProblemBegin");
    auto ending_spline_pose_residuals_fp = spline_pose_residuals_->GetNextFixedPoint("PreWindowBegin");
    int64_t constant_control_pose_begin_ts =     // constant states include this ts
        spline_pose_residuals_->ElementAtFixedPoint(starting_spline_pose_residuals_fp).EarliestControlPoseTimestamp();
    int64_t variable_control_pose_begin_ts =     // variable states begin here, constant states exclude this ts
        spline_pose_residuals_->ElementAtFixedPoint(ending_spline_pose_residuals_fp).EarliestControlPoseTimestamp();
    VLOG(1) << "Marking all control poses in range constant [" << constant_control_pose_begin_ts
        << " " << variable_control_pose_begin_ts << ")";
    
    // Mark control poses constant from(including) constant_control_pose_begin_ts to(exculding) variable_control_pose_begin_ts
    int32_t num_marked_constant = 0;
    if (!cubic_pose_spline_->MarkControlPosesConstant(constant_control_pose_begin_ts,
                                                      variable_control_pose_begin_ts,
                                                      problem_.get(), &num_marked_constant)) {
      LOG(ERROR) << "Could not mark control poses constant between [" << constant_control_pose_begin_ts
        << " " << variable_control_pose_begin_ts << ")";
      return false;
    }
    iteration_record_.iteration_counters["SplineControlPosesMarked"] += num_marked_constant;
    
    return true;
  }
  
  
  bool AddNewDataToProblem() {
    
    //// spline_pose_residuals_
    //for (auto fp = spline_pose_residuals_->GetNextFixedPoint("ProblemEnd");
    //     spline_pose_residuals_->IsNotPastTheEnd(fp); 
    //     //spline_pose_residuals_->IsNotAtTheEnd(fp);   // Here we avoid last residual
    //     spline_pose_residuals_->IncrementFixedPoint(fp)) {
    //  spline_pose_residuals_->MutableElementAtFixedPoint(fp)->AddToProblem(problem_.get());
    //  iteration_record_.iteration_counters["SplinePoseResidualsAdded"]++;
    ///}
    //// Reset the ProblemEnd marker
    //spline_pose_residuals_->SetFixedPoint("ProblemEnd"); // Sets fixed point at the end
    //VLOG(2) << "spline_pose_residuals problem end = " << spline_pose_residuals_->FixedPointToString("ProblemEnd");
    
    // spline_target_view_residuals_
    for (auto fp = spline_target_view_residuals_->GetNextFixedPoint("ProblemEnd");
         spline_target_view_residuals_->IsNotPastTheEnd(fp); 
         //spline_target_view_residuals_->IsNotAtTheEnd(fp);   // Here we avoid last residual
         spline_target_view_residuals_->IncrementFixedPoint(fp)) {
      VLOG(1) << "spline_target_view_residuals_ fp for residual to be added: " << fp.ToString()
          << " Address: " << spline_target_view_residuals_->MutableElementAtFixedPoint(fp);
      spline_target_view_residuals_->MutableElementAtFixedPoint(fp)->AddToProblem(problem_.get());
      iteration_record_.iteration_counters["SplinePoseResidualsAdded"]++;
    }
    // Reset the ProblemEnd marker
    spline_target_view_residuals_->SetFixedPoint("ProblemEnd"); // Sets fixed point at the end
    VLOG(2) << "spline_target_view_residuals problem end = " << spline_target_view_residuals_->FixedPointToString("ProblemEnd");
    
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
  
  
  // After solving, some states and their residuals need to be updated.
  bool UpdateStatesAndResiduals(const int64_t& iteration_end_ts) {
    
    // Recalculate control poses that were estimated in this iteration
    int32_t num_recalculated = 0;
    cubic_pose_spline_->RecalculateControlPoses(&num_recalculated);
    iteration_record_.iteration_counters["SplineControlPosesRecalculated"] += num_recalculated;
    
    // Extend the cubic spline till end of iteration ts
    //if (!cubic_pose_spline_->ExtendSpline(iteration_end_ts)) {
    //  LOG(FATAL) << "Could not extend cubic pose spline to ts " << iteration_end_ts;
    ///}
    
    // Extend the cubic spline
    int64_t extend_to_ts = max_iteration_observations_ts_;
    if (extend_to_ts == 0) {
      extend_to_ts = iteration_end_ts;
      LOG(INFO) << "max_iteration_observations_ts_ == 0. So extending spline to iteration_end_ts";
    }
    if (!cubic_pose_spline_->ExtendSpline(extend_to_ts)) {
      LOG(FATAL) << "Could not extend cubic pose spline to ts " << extend_to_ts;
    }
    
    return true;
  }
  
  
  // Mark new states constant - after the previous solving window
  bool MarkNewStatesConstant() {
    int64_t window_start_ts = swf_iterations_.SlidingWindowTimestamp();
    
    // Locate the PreWindowBegin marker for spline_target_view_residuals_
    if (!spline_target_view_residuals_->PositionFixedPointBeforeTimestamp(
          "PreWindowBegin", window_start_ts, true)) {
      LOG(WARNING) << "Could not locate the window_start_ts for marking "
          << "constant states in spline_target_view_residuals_ queue = " << window_start_ts;
      return false;
    }
    VLOG(2) << "spline_target_view_residuals_ solving window start ts was found. PreWindowBegin marker = " <<
        spline_target_view_residuals_->FixedPointToString("PreWindowBegin");
    
    // Get timestamp for the earliest state that is part of this problem
    auto ending_spline_target_view_residuals_fp = spline_target_view_residuals_->GetNextFixedPoint("PreWindowBegin");
    int64_t variable_control_pose_begin_ts =     // variable states begin here, constant states exclude this ts
        spline_target_view_residuals_->ElementAtFixedPoint(ending_spline_target_view_residuals_fp).EarliestControlPoseTimestamp();
    VLOG(1) << "Marking all control poses constant till " << variable_control_pose_begin_ts;        
    
    // Mark control poses constant till(exculding) variable_control_pose_begin_ts
    int32_t num_marked_constant = 0;
    if (!cubic_pose_spline_->MarkAdditionalControlPosesConstant(variable_control_pose_begin_ts,
                                                      problem_.get(), &num_marked_constant)) {
      LOG(ERROR) << "Could not mark control poses constant till " << variable_control_pose_begin_ts;
      return false;
    }
    iteration_record_.iteration_counters["SplineControlPosesMarked"] += num_marked_constant;
    
    return true;
  }
  
  // Mark new states constant - after the previous solving window
  bool MarkNewStatesConstant0() {
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
    
    // Get timestamp for the earliest state that is part of this problem
    auto ending_spline_pose_residuals_fp = spline_pose_residuals_->GetNextFixedPoint("PreWindowBegin");
    int64_t variable_control_pose_begin_ts =     // variable states begin here, constant states exclude this ts
        spline_pose_residuals_->ElementAtFixedPoint(ending_spline_pose_residuals_fp).EarliestControlPoseTimestamp();
    VLOG(1) << "Marking all control poses constant till " << variable_control_pose_begin_ts;        
    
    // Mark control poses constant till(exculding) variable_control_pose_begin_ts
    int32_t num_marked_constant = 0;
    if (!cubic_pose_spline_->MarkAdditionalControlPosesConstant(variable_control_pose_begin_ts,
                                                      problem_.get(), &num_marked_constant)) {
      LOG(ERROR) << "Could not mark control poses constant till " << variable_control_pose_begin_ts;
      return false;
    }
    iteration_record_.iteration_counters["SplineControlPosesMarked"] += num_marked_constant;
    
    return true;
  }
  
  bool Show() {
    show_ = true;
    display_.ShowWindow();
    return true;
  }
  
  bool Hide() {
    show_ = false;
    display_.HideWindow();
    return true;
  }
  
  // Destructor
  virtual ~CameraCalibrationTargetMotionFilter() {}
};  // CameraCalibrationTargetMotionFilter


} // namespace