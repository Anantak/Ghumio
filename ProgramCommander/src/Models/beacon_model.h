/**
 * Beacon model
 * Models the beacon motion
 */

/* Sliding window filter design notes:
 * New states of beacon and machine pose are created with wall time. As readings come in, residuals
 * are created that connect the states. All states and residuals are kept in circular queues.
 * Typically when a residual is created its jacobians are calculated and are not recalculated later.
 * Optimizer problem object is created and used for a certain time. After this time, object is
 * recycled. Priors and a certain number of existing residuals are added to the new problem object.
 * 
 *
 * Sliding window filter parameters:
 * States frequency - the frequency of creating machine and beacon states. 10Hz should be enough.
 * Iteration interval - period of new data added to the problem
 * Longest problem interval - problem object is recycled when this time length is achieved
 * Shortest problem interval - problem object is built to minimum of this time length
 * Solving problem interval - problem is solved after every this interval
 * Sliding window interval - this much window is solved. States before this interval are kept constant.
 *
 */

/* Kinematics design notes:
 * Machine and beacon are modelled with pose in world frame with velocity and accelerations in body
 * frames. (Yet to decide if velocities and accelerations will be spatial or conventional).
 * Acceleration is assumed to be constant during the interval between consecutive states, but
 * drifts with some process noise allowing it to change from state to state.
 *
 * Creation of states
 * When states are created, they are predicted from previous states. Prediction can only go upto
 * a certain interval, more than which prediction might be too far for the convergence basin of
 * the optimization algorithm. Predicted state is a function of previous state and time delay.
 * So we can calculate a jacobian of predicted state with previous state and time delay.
 *
 * Interpolation between states
 * As sensors are not synchronized, sensor measurement can be at any point between the states. To
 * save time we can linearly interpolate the state and jacobian from two flanking states. This will
 * add an interpolation noise. We can set the process noise large enough to incorporate this.
 *
 * Process noises
 * - Restricting motion to second order (acceleration) adds noise to the process
 * - Integration of motion using Simpson's rule adds noise
 * - Interpolating linearly between states (both state and jacobian) adds noise
 *
 * Kinematic residuals
 * Each consecutive state is linked with kinematic residuals that link error in previous pose with
 * error in the next pose.
 *
 * Kinematics parameters:
 * Maximum rate of change of acceleration - defines the drift of the machine between states.
 */

/* Beacon IMU notes:
 * Beacon sends its IMU readings via a bursty wireless link. So the timing will be off by two
 * parameters - a drift and a scale. IMU readings are timestamped when they are recieved and a
 * recieved sequence number is assigned to them. The sequence number determines the actual delay
 * between consecutive readings, thus giving them a synthetic timestamp.
 *
 * This still needs to be thought through
 *
 */

/* Beacon tag sightings
 * Beacon's April tag message is recieved by the filter. Filter then estimates the timings of the
 * tags in a sequence.
 */

/* Infra red marker messages
 * Pixy cameras send the marker sightings to the filter. These should be the most timely sightings
 * of the beacon and could form the time-delay = 0 anchor.
 *
 * Residuals for pixy sightings
 * Pixy camera sigthings form a reprojection residual. Each sighting has a timestamp. A residual
 * for pixy camera beacon sighting connects the previous beacon state, machine state, pixy camera
 * location on the machine, pixy camera intrinsics and pixy camera time delay (set to zero).
 */


/* How to implement?
 *
 * Kinematic state
 *
 * Predict kinematic state function
 * - takes in a state and a time interval
 * - calculates prediction of the state and jacobian of predicted state wrt given state and interval
 *
 * Interpolate kinematic states function
 * - takes two kinematic states and a double fraction
 * - calculates interpolated state and interpolated jacobian
 *
 * Filter
 * - starts at a certain wall time
 * - Initiation
 *    initiates a starting machine state, beacon state
 *    initiates camera intrinsics states for all cameras
 *    initiates camera-to-machine poses for all cameras
 *    
 * - Iteration   
 *    builds new machine and beacon states, predicts them
 *    tag readings are added as tag view residuals
 *    if it is time to rebuild problem, do it
 *    add new residuals to the problem
 *    if it is time to solve problem, solve it
 *    
 */

// std includes
#include <array>

// Anantak includes
#include "ModelsLib0.h"
#include "Filter/timed_circular_queue.h"
#include "Filter/observation.h"

// Protocol buffers - specific to beacon model
#include "state_messages.pb.h"

namespace anantak {

using namespace anantak;


/* Kinematic state
 * Pose, velocity and acceleration of a body in 3d
 * Pose is quaternion and position in world frame
 * Velocity and acceleration is in body frame. These could be spatial or conventional.
 *  Quaternion in world frame       Wq
 *  Position in world frame         Wp
 *  Ang velocity in body frame      Bw
 *  Velocity in body frame          Bv
 *  Ang acceleration in body frame  Bdw
 *  Acceleration in body frame      Bdv
 * Error is measured by a 3-vector for each element making a 18x1 vector
 *  Wq^ = Wq*dWq => Wr^ = Wr*(I3-[dWQ x]) with dWq = [2*dWQ, 1] and Q is theta, [. x] is skewsymm
 *  Wp^ = Wp + dWp
 *  Bw^ = Bw + dBw
 *  Bv^ = Bv + dBv
 *  Bdw^ = Bdw + dBdw
 *  Bdv^ = Bdv + dBdv
 */
class KinematicState : public State {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef Eigen::Map<Eigen::Vector3d> MapVector3dType;
  typedef Eigen::Map<Eigen::Vector4d> MapVector4dType;
  typedef Eigen::Map<Eigen::Matrix<double,19,1>> MapVector19dType;
  typedef Eigen::Map<Eigen::Matrix<double,18,1>> MapVector18dType;  
  typedef Eigen::Map<const Eigen::Matrix<double,19,1>> MapConstVector19dType;
  typedef Eigen::Map<const Eigen::Matrix<double,18,1>> MapConstVector18dType;  
  
  // Timestamp
  int64_t timestamp_;
  // State
  double state_[19];
  // Error
  double error_[18];
  
  // Helper const pointers
  // Values
  double * const pose_;
  double * const velocity_;
  double * const acceleration_;
  // Errors
  double * const dpose_;
  double * const dvelocity_;
  double * const dacceleration_;
  
  // Helper eigen maps
  // Values
  MapVector4dType BqW_;  // quaternion as a vector x,y,z,w
  MapVector3dType BqaW_; // vector part of quaternion
  MapVector3dType WpB_;
  MapVector3dType Bw_;
  MapVector3dType Bv_;
  MapVector3dType Bdw_;
  MapVector3dType Bdv_;
  // Errors
  MapVector3dType dWQ_; // angleaxis as a vector
  MapVector3dType dWp_;
  MapVector3dType dBw_;
  MapVector3dType dBv_;
  MapVector3dType dBdw_;
  MapVector3dType dBdv_;
  
  // Default constructor
  KinematicState(): State(), BqaW_(state_),
    pose_(state_),  velocity_(state_+7),  acceleration_(state_+13),
    dpose_(error_), dvelocity_(error_+6), dacceleration_(error_+12),
    BqW_(state_),  WpB_(state_+4),  Bw_(state_+7), Bv_(state_+10),  Bdw_(state_+13),  Bdv_(state_+16),
    dWQ_(error_),  dWp_(error_+3), dBw_(error_+6), dBv_(error_+9), dBdw_(error_+12), dBdv_(error_+15)
  {
    SetZero();
  }
  
  // Set to zero
  bool SetZero() {
    timestamp_ = 0;
    MapVector19dType s(state_);
    MapVector18dType e(error_);
    s.setZero(); s[3] = 1.;
    e.setZero();
    return true;
  }
  
  // Set Error state to zero
  bool SetErrorZero() {
    MapVector18dType e(error_);
    e.setZero();
    return true;
  }
  
  // Set timestamp
  bool SetTimestamp(const int64_t& ts) {
    timestamp_ = ts;
    return true;
  }
  
  // Recalculate the state from error state after optimization. Set errors to zero.
  bool Recalculate() {
    Eigen::Quaterniond BqW(BqW_.data()); // x,y,z,w
    Eigen::Quaterniond dWq = anantak::ErrorAngleAxisToQuaterion(dWQ_);
    BqW *= dWq;  // assuming both quaternions are already normalized
    BqW_ = BqW.coeffs();
    WpB_ += dWp_;
    Bw_ += dBw_;
    Bv_ += dBv_;
    Bdw_ += dBdw_;
    Bdv_ += dBdv_;
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
  virtual ~KinematicState() {}
  
  // Copy operator
  KinematicState& operator= (const KinematicState& p) {
    timestamp_ = p.timestamp_;
    MapVector19dType s(state_);
    MapVector18dType e(error_);
    MapConstVector19dType ps(p.state_);
    MapConstVector18dType pe(p.error_);
    s = ps;
    e = pe;
  }
  
};  // KinematicState


/* Predict kinematic state
 * Use the last state to predict the next one
 */
bool PredictKinematicState(const KinematicState& state0, const int64_t& interval,
    KinematicState* state1) {
  
  // Set the target timestamp
  state1->timestamp_ = state0.timestamp_ + interval;
  
  return true;
}

//
bool InterpolateKinematicStates(const KinematicState& state0, const KinematicState& state1,
    const double& frac) {
  
  return true;
}


/** Beacon filter
 * Run beacon filtering
 */
class BeaconFilter : public anantak::Model {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  struct Options {
    // Filtering options
    uint64_t states_history_interval;     // Past states are kept for this long interval
    uint64_t longest_problem_interval;    // Problem will be built to this longest length
    uint64_t shortest_problem_interval;   // Problem will be built to this length at a minimum
    uint64_t sliding_window_interval;     // This is the length of time in which states will be solved
    uint64_t solving_problem_interval;    // Problem will be solved after every this interval
    uint32_t states_frequency;            // Frequency (Hz) at which machine states will be created
    anantak::SlidingWindowFilterIterations::Options sliding_window_options;
    
    // Options relating to data to be processed
    uint16_t max_tag_camera_frequency;    // Maximum rate of images from tag cameras (Hz)
    std::vector<uint32_t> tag_camera_ids; // IDs of tag cameras to be processed
    
    // Beacon specifications
    anantak::BeaconConfig beacon_config;
    std::string beacon_tag_id;
    double beacon_tag_size;
    double beacon_tag_size_stdev;         // uncertainty in knowledge of beacon tag size
    
    // Camera intrinsics specifications
    bool camera_intrinsics_message_is_available;
    anantak::CameraIntrinsicsStateMessage camera_intrinsics_message;
    anantak::CameraIntrinsicsInitMessage camera_intrinsics_init;
    double camera_angle_of_view;
    std::array<double,2> camera_image_size;
    double camera_angle_of_view_stdev;
    double camera_center_stdev;
    
    // Tag view residuals
    anantak::DynamicAprilTagViewResidual::Options apriltag_view_residual_options;    
    
    Options(const std::string& config_filename = ""):
      states_history_interval(  600000000),
      longest_problem_interval(  30000000),
      shortest_problem_interval( 15000000),
      sliding_window_interval(    2000000),
      solving_problem_interval(   1000000),
      states_frequency(                20),
      
      sliding_window_options(
        longest_problem_interval,     // longest problem length
        shortest_problem_interval,    // shortest problem length
        sliding_window_interval,      // sliding window length
        solving_problem_interval      // solving problem interval
      ),
      
      max_tag_camera_frequency(30),
      tag_camera_ids({0}),
      
      beacon_tag_id("Tag16h5_10"),
      beacon_tag_size(0.1615),        // Beacon tag size in meters
      beacon_tag_size_stdev(0.001),
      
      camera_intrinsics_message_is_available(false),
      camera_angle_of_view(100*RadiansPerDegree),
      camera_image_size{{640, 480}},  // array uses aggregates initialization, so double braces(!?)
      camera_angle_of_view_stdev(30*RadiansPerDegree),  // Large starting uncertainty in angle of view
      camera_center_stdev(100),  // Large uncertainty in center location
      
      apriltag_view_residual_options(1.0)    
    {
      // Read the Programs Setup file
      std::string config_file_path = anantak::GetProjectSourceDirectory() + "/" + config_filename;
      VLOG(1) << "Config file = " << config_file_path;
      std::unique_ptr<anantak::BeaconCameraCalibratorConfig> config =
          anantak::ReadProtobufFile<anantak::BeaconCameraCalibratorConfig>(config_file_path);
      if (!config) {
        LOG(ERROR) << "Could not parse the config file resorting to default";
      } else {
        // Use the config file to set all values
        states_history_interval = config->states_history_interval();
        states_frequency = config->states_frequency();
        sliding_window_options = anantak::SlidingWindowFilterIterations::Options(config->sliding_window());
        tag_camera_ids.clear();
        tag_camera_ids.push_back(config->camera_num());
        max_tag_camera_frequency = config->max_camera_frequency();
        beacon_config = config->beacon();
        camera_intrinsics_init = config->camera_init();
        apriltag_view_residual_options =
            anantak::DynamicAprilTagViewResidual::Options(config->apriltag_view_resid_options());
      }
    }
    
    std::string ToString() {
      return "History: "+std::to_string(states_history_interval)+" Frequency: "+std::to_string(states_frequency)+"\n"+
        sliding_window_options.ToString()+
        "\nCamera id: "+std::to_string(tag_camera_ids[0])+" Camera max freq: "+std::to_string(max_tag_camera_frequency);
    }
    
    bool SetStartingCameraIntrinsics(const anantak::CameraIntrinsicsStateMessage& msg) {
      camera_intrinsics_message_is_available = true;
      camera_intrinsics_message = msg;    // copy
      VLOG(1) << "Setting starting camera intrinsics in options = "
        << camera_intrinsics_message.pinhole(0) << " " << camera_intrinsics_message.pinhole(1) << " "
        << camera_intrinsics_message.pinhole(2) << " " << camera_intrinsics_message.pinhole(3);
      return true;
    }
  }; // Options
  
  // Options object
  BeaconFilter::Options options_;
  
  // Sliding window filter objects
  anantak::SlidingWindowFilterIterations swf_iterations_;
  anantak::IterationRecord iteration_record_;
  
  // Optimization objects
  std::unique_ptr<ceres::Problem> problem_;
  ceres::Problem::Options problem_options_;
  ceres::Solver::Options solver_options_;
  ceres::Solver::Summary solver_summary_;
  
  // Camera and beacon poses
  std::unique_ptr<anantak::TimedCircularQueue<anantak::Pose3dState>> camera_poses_;
  std::unique_ptr<anantak::TimedCircularQueue<anantak::Pose3dState>> beacon_poses_;
  // Beacon tag
  anantak::StaticAprilTagState beacon_tag_;
  // Camera intrinsics 
  anantak::CameraIntrinsicsState camera_intrinsics_;
  anantak::CameraIntrinsicsNormalPrior camera_intrinsics_prior_;
  // Tag view Residuals
  std::unique_ptr<anantak::TimedCircularQueue<anantak::DynamicAprilTagViewResidual>> tag_view_residuals_;
  typedef anantak::TimedCircularQueue<anantak::DynamicAprilTagViewResidual>::FixedPoint TagViewQueueFixedPointType;
  
  // Results
  anantak::SensorMsg camera_intrinsics_message_;
  
  // Helpers
  uint64_t inter_state_interval_;
  
  // Default constructor
  BeaconFilter() {
    InitiateFilter();
  }
  
  bool SetOptions(anantak::BeaconFilter::Options options) {
    options_ = options; // using compiler defined = operator
    swf_iterations_ = options.sliding_window_options;
  }
  
  // Constructor with options defined
  BeaconFilter(anantak::BeaconFilter::Options options,
               const int64_t& filter_start_ts = 0):
    options_(options),
    swf_iterations_(options.sliding_window_options), iteration_record_(filter_start_ts),
    problem_(nullptr), problem_options_(), solver_options_(), solver_summary_()    
  {
    InitiateFilter();
  }
  
  bool InitiateFilter() {
    LOG(INFO) << "Creating Beacon filter";
    
    // Memory allocation
    AllocateMemory();
    
    // Helpers
    inter_state_interval_ = 1000000 / options_.states_frequency;
    
    // Set optimization options
    problem_options_.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    solver_options_.max_num_iterations = 300;
    solver_options_.minimizer_progress_to_stdout = false;
    solver_options_.logging_type = ceres::SILENT;
    
    // If no filter starting timestamp is set, use wall time
    if (iteration_record_.begin_ts == 0) {
      int64_t data_start_ts = GetWallTime();
      iteration_record_.Reset(data_start_ts);
      swf_iterations_.StartFiltering(data_start_ts);
    } else {
      swf_iterations_.StartFiltering(iteration_record_.begin_ts);
    }
    
    // Initiate beacon tag
    if (options_.beacon_config.tags_size()<1) {
      LOG(ERROR) << "Provided beacon config has no tags! Using default values";
      Eigen::Quaterniond TqB = Eigen::Quaterniond::Identity();
      Eigen::Vector3d BpT = Eigen::Vector3d::Zero();
      beacon_tag_.SetZero();
      beacon_tag_.Create(&options_.beacon_tag_id, &TqB, &BpT, &options_.beacon_tag_size);
      // Set covariances
      beacon_tag_.pose_.covariance_.setZero();
      beacon_tag_.size_.covariance_ = options_.beacon_tag_size_stdev * options_.beacon_tag_size_stdev;
    } else {
      LOG(INFO) << "Using config file provided beacon tag";
      beacon_tag_.Create(options_.beacon_config.tags(0));
    }
    // Report
    VLOG(1) << "Beacon tag id, size = " << beacon_tag_.tag_id_ << " " << beacon_tag_.size_.state_;
    VLOG(1) << "Beacon tag pose = " << beacon_tag_.pose_.LqvG_.transpose()
        << ", " << beacon_tag_.pose_.GpL_.transpose();
    
    // Initiate camera instrinsics
    //  Best case is camera_intrinsics_message is available
    //  If not, use camera_intrinsics_init from a config file
    //  If not, use the hard coded values
    if (options_.camera_intrinsics_message_is_available) {
      LOG(INFO) << "Using camera intrinsics message from a data queue";
      camera_intrinsics_.Create(options_.camera_intrinsics_message);
    } else if (options_.camera_intrinsics_init.image_size_size() == 2) { 
      LOG(INFO) << "Using config file provided camera intrinsics init";
      camera_intrinsics_.Create(options_.camera_intrinsics_init);
    } else {
      LOG(ERROR) << "Using default values to set starting camera intrinsics";
      camera_intrinsics_.Create(options_.camera_angle_of_view,
          options_.camera_image_size[0], options_.camera_image_size[1],
          options_.camera_angle_of_view_stdev, options_.camera_center_stdev);
    }
    // Report what was used
    VLOG(1) << "Starting camera intrinsics = " << camera_intrinsics_.ToString();
    
    // Initiate camera intrinsics prior
    camera_intrinsics_prior_.Create(&camera_intrinsics_, true);
    
    // Initialize states
    SetStartingStates();
    return true;
  }
  
  /* Allocate memory for queues */
  bool AllocateMemory() {
    LOG(INFO) << "Allocating memory for the beacon filter queues";
    
    int32_t num_states_to_keep = options_.states_frequency * options_.states_history_interval / 1000000;
    LOG(INFO) << "  num_states_to_keep = " << num_states_to_keep
        << " at states_frequency = " << options_.states_frequency << " Hz";
    
    // Camera poses
    std::unique_ptr<anantak::TimedCircularQueue<anantak::Pose3dState>> cam_states_ptr(
        new anantak::TimedCircularQueue<anantak::Pose3dState>(num_states_to_keep));
    camera_poses_ = std::move(cam_states_ptr);
    
    // Beacon poses
    std::unique_ptr<anantak::TimedCircularQueue<anantak::Pose3dState>> bc_poses_ptr(
        new anantak::TimedCircularQueue<anantak::Pose3dState>(num_states_to_keep));
    beacon_poses_ = std::move(bc_poses_ptr);
    
    // Tag residuals
    int32_t num_tag_views_to_keep = options_.states_history_interval / 1000000
        * options_.max_tag_camera_frequency;
    LOG(INFO) << "Initiating tag view residuals queues with length = " << num_tag_views_to_keep;
    
    std::unique_ptr<anantak::TimedCircularQueue<anantak::DynamicAprilTagViewResidual>> view_queue_ptr(
        new anantak::TimedCircularQueue<anantak::DynamicAprilTagViewResidual>(num_tag_views_to_keep));
    tag_view_residuals_ = std::move(view_queue_ptr);
    
    return true;
  }
  
  /* Set starting states
   * Machine starting state is at origin. Beacon starting state is provided via options.
   */
  bool SetStartingStates() {
    // Calculate the starting state timestamp
    int64_t starting_state_ts = (iteration_record_.begin_ts / inter_state_interval_) * inter_state_interval_;
    VLOG(1) << "Using starting state ts = " << starting_state_ts;
    
    // Create a zero pose state and add to camera/beacon state queues
    anantak::Pose3dState zero_pose;
    zero_pose.SetTimestamp(starting_state_ts);
    camera_poses_->AddElement(zero_pose);
    camera_poses_->SetTimestamp(starting_state_ts);
    beacon_poses_->AddElement(zero_pose);
    beacon_poses_->SetTimestamp(starting_state_ts);
    
    // Create a zero tag view residual, add it to the beacon tag residuals 
    anantak::DynamicAprilTagViewResidual zero_tag_view_resid;
    zero_tag_view_resid.SetTimestamp(starting_state_ts);
    tag_view_residuals_->AddElement(zero_tag_view_resid);
    tag_view_residuals_->SetTimestamp(starting_state_ts);
    
    // Set markers on queues
    camera_poses_->SetFixedPoint("TagViewResidual"); // Marks the last tag view residual location
    beacon_poses_->SetFixedPoint("TagViewResidual"); // Marks the last tag view residual location
    tag_view_residuals_->SetFixedPoint("PreProblemBegin"); // Residuals were added to problem after here
    tag_view_residuals_->SetFixedPoint("ProblemEnd"); // Residuals were added to problem till here
    tag_view_residuals_->SetFixedPoint("PreWindowBegin"); // States related with residuals beyond this ts will be solved
    
    // Add counters
    iteration_record_.iteration_counters["CameraPosesCreated"] = 0;   // Number of camera poses created
    iteration_record_.iteration_counters["BeaconPosesCreated"] = 0;   // Number of beacon poses created
    iteration_record_.iteration_counters["TagViewResidualsCreated"] = 0; // Number of tag view residuals created
    iteration_record_.iteration_counters["TagViewResidualsAdded"] = 0;   // Number of tag view residuals added to problem
    iteration_record_.iteration_counters["TagViewResidualsMarked"] = 0;   // Number of tag view residuals marked constant
    iteration_record_.algorithm_counters["ProblemObjects"] = 0;   // Number of tag view residuals marked constant
    
    // Report
    VLOG(1) << "Zero Camera pose at " << camera_poses_->FixedPointToString("TagViewResidual");
    VLOG(1) << "Zero Beacon pose at " << beacon_poses_->FixedPointToString("TagViewResidual");
    VLOG(1) << "Zero TVR PreProblemBegin at " << tag_view_residuals_->FixedPointToString("PreProblemBegin");
    VLOG(1) << "Zero TVR PreWindowBegin at " << tag_view_residuals_->FixedPointToString("PreWindowBegin");
    
    return true;
  }
  
  // Start iteration
  bool StartIteration(const int64_t& iteration_end_ts) {
    // Is this iteration end ts valid?
    if (!iteration_record_.Increment(iteration_end_ts)) {
      LOG(ERROR) << "Invalid iteration_end_ts " << iteration_end_ts;
      return false;
    }
    iteration_record_.ResetIterationCounters();
    
    // Add this timestamp to swf filtering iterations helper
    swf_iterations_.AddData(iteration_end_ts);
    
  }
  
  // Create new states for the iteration
  bool CreateIterationStates(const int64_t& iteration_end_ts) {
    // Create new states
    if (!CreateNewStates(iteration_end_ts)) {
      LOG(ERROR) << "Could not create new states for timestamp " << iteration_end_ts;
      return false;
    }
    
    return true;
  }
  
  // Create new residuals from the observations - sensor-wise data input
  bool CreateIterationResiduals(
      const std::vector<std::unique_ptr<std::vector<anantak::SensorMsg>>>& msgs) {
    
    // Extract sensor messages that this filter can process and create residuals
    for (int i_sensor=0; i_sensor < msgs.size(); i_sensor++) {
      const std::vector<anantak::SensorMsg>& sensor_msgs = *msgs[i_sensor];
      for (int i_msg=0; i_msg < sensor_msgs.size(); i_msg++) {
        const anantak::SensorMsg& msg = sensor_msgs[i_msg];
        
        if (!CreateResidual(msg)) {
          LOG(ERROR) << "Could not create a residual from message. Skipping.";
          continue;
        }
        
      }
    }
    
    VLOG(2) << "Iteration record = " << iteration_record_.IterationCountersToString();
    
    return true;
  }
  
  bool CreateIterationResiduals(
      const anantak::ObservationsVectorStoreMap& observations_map) {
    
    // Extract messages and create residuals
    for (auto i_map=observations_map.begin();
         i_map!=observations_map.end(); i_map++) {
      int32_t num_msgs = i_map->second.n_observations;
      VLOG(1) << "Extracting messages of type " << i_map->first << " num " << num_msgs;
      for (int i_obs=0; i_obs<num_msgs; i_obs++) {
        anantak::MessageType* msg_ptr = i_map->second.observations->at(i_obs).get();
        if (!msg_ptr) {
          LOG(ERROR) << "Message pointer is null. Skipping.";
          continue;
        }
        // Cast the pointer to Sensor message. This 
        if (anantak::SensorMsg* sensor_msg_ptr = dynamic_cast<anantak::SensorMsg*>(msg_ptr)) {
          if (!CreateResidual(*sensor_msg_ptr)) {
            LOG(ERROR) << "Could not create a residual from message. Skipping.";
            continue;
          }
        } else {
          LOG(ERROR) << "Could not cast protocol buffer message to SensorMsg. Skipping.";
          continue;
        }
        
      } // for each message in this type
    } // for each type
    
    return true;
  }
  
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
    
    // Solve the problem
    if (swf_iterations_.IsItTimeToSolveProblem()) {
      VLOG(2) << "Solving the problem.";
      SolveProblem();
      VLOG(2) << "Solved the problem.";
      
      // We never recalculate states after solving. Residual is not going to change!
      ////RecalculateStates(); - Do NOT do this!
      
      // Mark constant states
      MarkNewStatesConstant();
      
      // Calculate new solve begin ts
      swf_iterations_.SolveProblem();
    }
    
    // Post solving operations
    ReportResults();
    
    VLOG(1) << "Iteration record " << iteration_record_.IterationCountersToString()
        << "  Algorithm record " << iteration_record_.AlgorithmCountersToString();
    
    return true;
  }
  
  // Process messages
  bool ProcessMessages(const int64_t& iteration_end_ts,
      const std::vector<std::unique_ptr<std::vector<anantak::SensorMsg>>>& msgs) {
    
    if (!StartIteration(iteration_end_ts)) {
      LOG(ERROR) << "Could not start iteration";
      return false;
    }
    
    if (!CreateIterationStates(iteration_end_ts)) {
      LOG(ERROR) << "Could not create states for the iteration";
      return false;
    }
    
    if (!CreateIterationResiduals(msgs)) {
      LOG(ERROR) << "Could not create residuals for the iteration";
      return false;
    }
    
    if (!RunFiltering(iteration_end_ts)) {
      LOG(ERROR) << "Could not run filtering operations";
      return false;
    }
    
    return true;
  }
  
  /* Create new states
   * At the beginning of every iteration, new states linked to wall clock are created.
   * Each state is created and initiated. Initiation follows some prediction method.
   */
  bool CreateNewStates(const int64_t& iteration_end_ts) {
    
    // Make sure that there is an existing last state
    if (camera_poses_->n_msgs() == 0) {
      LOG(ERROR) << "Machine kin states queue is empty! Can not create new states.";
      return false;
    }
    if (beacon_poses_->n_msgs() == 0) {
      LOG(ERROR) << "Beacon kin states queue is empty! Can not create new states.";
      return false;
    }
    
    // Calculate iteration ending state timestamp and number of states to be created
    const int64_t iteration_end_state_ts =
        ((iteration_end_ts / inter_state_interval_) +1) * inter_state_interval_;
    const int64_t last_state_ts = camera_poses_->Back().timestamp_;
    const int32_t num_states_to_create =
        (iteration_end_state_ts - last_state_ts) / inter_state_interval_;
    VLOG(2) << "  States ts last, end, num = " << last_state_ts << " " << iteration_end_state_ts
        << " " << num_states_to_create;
    
    // Create and predict new machine states
    for (int i=0; i<num_states_to_create; i++) {
      const anantak::Pose3dState& last_cam_state = camera_poses_->Back();
      anantak::Pose3dState* mc_state = camera_poses_->NextMutableElement();
      if (!PredictPose3dState(last_cam_state, inter_state_interval_, mc_state)) {
        LOG(ERROR) << "Could not predict machine kinematic state. Can not continue.";
        return false;
      }
      // Assign timestamp on the queue
      camera_poses_->SetTimestamp(mc_state->timestamp_);
      iteration_record_.iteration_counters["CameraPosesCreated"]++;
      // Check for mistakes
      if (mc_state->timestamp_ > iteration_end_state_ts) {
        LOG(ERROR) << "mc_state->timestamp_ > iteration_end_state_ts. Should not happen! Quit. "
            << mc_state->timestamp_ << " " << iteration_end_state_ts;
        return false;
      }
    }
    
    // Create and predict new beacon states
    for (int i=0; i<num_states_to_create; i++) {
      const anantak::Pose3dState& last_bc_state = beacon_poses_->Back();
      anantak::Pose3dState* bc_state = beacon_poses_->NextMutableElement();
      if (!PredictPose3dState(last_bc_state, inter_state_interval_, bc_state)) {
        LOG(ERROR) << "Could not predict beacon kinematic state. Can not coninue.";
        return false;
      }
      // Assign timestamp on the queue
      beacon_poses_->SetTimestamp(bc_state->timestamp_);
      iteration_record_.iteration_counters["BeaconPosesCreated"]++;
      // Check for mistakes
      if (bc_state->timestamp_ > iteration_end_state_ts) {
        LOG(ERROR) << "bc_state->timestamp_ > iteration_end_state_ts. Should not happen! Quit. "
            << bc_state->timestamp_ << " " << iteration_end_state_ts;
        return false;
      }
    }
    
    return true;
  }
  
  // Create a residual from a sensor message
  bool CreateResidual(const anantak::SensorMsg& msg) {
    const anantak::HeaderMsg& hdr = msg.header();
    
    // Reject the residual if timestamp of reading is before the sliding window
    if (hdr.timestamp() < swf_iterations_.SlidingWindowTimestamp()) {
      LOG(ERROR) << "Timestamp of reading is before sliding window. Skipping reading.";
      return false;
    }
    
    // Based on the type of message, process it
    
    // April tags message 
    if (hdr.type() == "AprilTags") {
      
      // Make sure there is an Aptiltag message here
      if (!msg.has_april_msg()) {
        LOG(WARNING) << "Apriltag message not found the sensor message! Skipping.";
        return false;
      }
      
      // Extract Apriltag message and make sure that it is the beacon tag
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
        return true;
      }
      
      for (uint16_t i_tag=0; i_tag < apriltag_msg.tag_id_size(); i_tag++) {
        const std::string& id = apriltag_msg.tag_id(i_tag);
        //VLOG(1) << "  Saw a tag id = " << id;
        
        // Make sure that this is the beacon tag
        if (id != beacon_tag_.tag_id_) {
          // This is not the tag we are looking for
          continue;
        }
        
        // Process the beacon tag
        //VLOG(1) << "Saw beacon from tag camera " << *it;
        if (!CreateResidualForBeaconTagMessage(hdr.timestamp(), apriltag_msg, i_tag)) {
          LOG(ERROR) << "Could not create residual for Apriltag message. Skipping.";
          return false;
        }
      }
      
    }  // type = "AprilTag"
    
    // Infra red camera
    /*else if (hdr.type() == "Pixy") {
      
      // Check for presence of pixy camera message
      if (!msg.has_pixy_cam_msg()) {
        LOG(WARNING) << "Pixy message not found the sensor message! Skipping.";
        return false;
      }
      
      // Extract the pixy message
      const anantak::PixyCameraMessage& pixy_msg = msg.pixy_cam_msg();
      
      // Check for camera number
      if (!pixy_msg.has_camera_num()) {
        LOG(WARNING) << "Pixy message does not have a camera number! Skipping.";
        return false;
      }
      
      // Get camera number
      const int32_t& cam_id = pixy_msg.camera_num();
      
      // Check the camera number, this should be in the list for this filter
      auto it = std::find(options_.ir_camera_ids.begin(), options_.ir_camera_ids.end(), cam_id);
      if (it == options_.ir_camera_ids.end()) {
        return false;
      }
      
      // Process the ir message
      //VLOG(1) << "Saw beacon from tag camera " << *it;
      
    }*/
    
    return true;
  }
  
  /* Create residuals for Apriltag views
   */
  bool CreateResidualForBeaconTagMessage(const int64_t& timestamp,
      const anantak::AprilTagMessage& apriltag_msg, const uint16_t& i_tag) {
    
    // Find the beacon pose previous to this reading
    if (!beacon_poses_->PositionFixedPointBeforeTimestamp("TagViewResidual", timestamp)) {
      LOG(ERROR) << "Did not find apriltag message timestamp in beacon poses queue. Skipping.";
      LOG(ERROR) << "Diff between apriltag message and last beacon pose timestamps = "
          << timestamp - beacon_poses_->LastTimestamp();
      return false;
    }
    anantak::Pose3dState* beacon_pose = beacon_poses_->MutableElementAtFixedPoint("TagViewResidual");
    VLOG(2) << "Beacon TagViewResidual posn = " << beacon_poses_->FixedPointToString("TagViewResidual");
    
    // Find the camera pose previous to this reading
    if (!camera_poses_->PositionFixedPointBeforeTimestamp("TagViewResidual", timestamp)) {
      LOG(ERROR) << "Did not find apriltag message timestamp in camera poses queue. Skipping.";
      LOG(ERROR) << "Diff between apriltag message and last camera pose timestamps = "
          << timestamp - camera_poses_->LastTimestamp();
      return false;
    }
    anantak::Pose3dState* camera_pose = camera_poses_->MutableElementAtFixedPoint("TagViewResidual");
    VLOG(2) << "Camera TagViewResidual posn = " << camera_poses_->FixedPointToString("TagViewResidual");
    
    // Create a tag view reading from tag message
    anantak::AprilTagReadingType tag_reading;
    if (!tag_reading.SetFromAprilTagMessage(timestamp, apriltag_msg.camera_num(), i_tag, apriltag_msg)) {
      LOG(ERROR) << "Could not set tag reading from tag message Skipping message.";
      return false;
    }
    if (!tag_reading.EstimateCameraToTagPose(&camera_intrinsics_, beacon_tag_.size_.state_)) {
      LOG(ERROR) << "Could not estimate camera to tag pose. Skipping message";
      return false;
    }
    VLOG(2) << "Beacon tag reading = " << timestamp << " " << tag_reading;
    
    // Initiate the beacon pose
    if (!tag_reading.CalculateTagPoseFromCameraPose(camera_pose, beacon_pose)) {
      LOG(ERROR) << "Could not estimate beacon pose from camera pose. Skipping.";
      return false;
    }
    
    // Create a tag view residual from the reading and camera intrinsics
    anantak::DynamicAprilTagViewResidual *tag_view_resid = tag_view_residuals_->NextMutableElement();
    if (!tag_view_resid->Create(&tag_reading, camera_pose, beacon_pose, &beacon_tag_.size_, &camera_intrinsics_,
        &options_.apriltag_view_residual_options, true)) {
      LOG(ERROR) << "Could not create tag view residual. Skipping.";
      tag_view_resid->Reset();
      tag_view_residuals_->decrement();
      return false;
    }
    tag_view_residuals_->SetTimestamp(timestamp);
    iteration_record_.iteration_counters["TagViewResidualsCreated"]++;
    
    // Report the timestamps associated
    VLOG(2) << "  Residual ts = " << tag_view_resid->timestamp_
        << " beacon pose ts = " << beacon_pose->timestamp_
        << ", " << beacon_poses_->ElementAtFixedPoint(beacon_poses_->GetNextFixedPoint("TagViewResidual")).timestamp_;
        //<< "  " << beacon_poses_->Timestamp("TagViewResidual")
        //<< ", " << beacon_poses_->Timestamp(beacon_poses_->GetNextFixedPoint("TagViewResidual"));
    
    return true;
  }
  
  /* Calculate priors for the problem */
  bool CalculatePriors() {
    
    // This is a calibration state - camera intrinsics.
    //  We can try to recalculate the state and calculate a new prior
    //  We then do not add any past residuals to the problem as prior represents all the information
    //  contained in the past states.
    camera_intrinsics_.Recalculate();
    
    // Calculate camera intrinsics covariance matrix
    camera_intrinsics_.CalculateCovariance(problem_.get());
    camera_intrinsics_prior_.Create(&camera_intrinsics_, true);
    // Report covariance
    VLOG(1) << "Camera intrinsics at recycle of problem = " << camera_intrinsics_.ToString();
    VLOG(1) << "Camera intrinsics prior = \n" << camera_intrinsics_prior_;
    
    return true;
  }

  // Add priors to the problem
  bool AddPriors() {
    
    // Add camera intrinsics prior to problem
    ceres::CostFunction* camera_intrinsics_prior_cf = &camera_intrinsics_prior_;
    problem_->AddResidualBlock(
      camera_intrinsics_prior_cf, NULL,
      camera_intrinsics_prior_.cam_->error_
    );
    VLOG(1) << "Added camera intrinsics prior to problem";
    
    return true;
  }
  
  // Add a tag view residual to the problem
  bool AddTagViewResidualToProblem(const TagViewQueueFixedPointType& fixed_point) {
    
    anantak::DynamicAprilTagViewResidual* tvr =
        tag_view_residuals_->MutableElementAtFixedPoint(fixed_point);
    ceres::CostFunction* tvr_cf = tvr;
    problem_->AddResidualBlock(
      tvr_cf, NULL,
      tvr->poseC_->error_,
      tvr->tagTj_pose_->error_,
      &tvr->tagTj_size_->error_,
      tvr->camera_->error_
    );
    
    // Set states constant - depends on the problem being solved
    // Set camera pose constant
    problem_->SetParameterBlockConstant(tvr->poseC_->error_);
    // Set tag pose constant
    //problem_->SetParameterBlockConstant(tvr->tagTj_pose_->error_);
    // Set tag size constant
    problem_->SetParameterBlockConstant(&tvr->tagTj_size_->error_);
    // Set camera intrinsics constant
    //problem_->SetParameterBlockConstant(tvr->camera_->error_);
    
    return true;
  }
  
  // Mark states related with a tag view residual as constants in the problem
  bool MarkTagViewResidualStatesConstant(const TagViewQueueFixedPointType& fixed_point) {
    
    anantak::DynamicAprilTagViewResidual* tvr =
        tag_view_residuals_->MutableElementAtFixedPoint(fixed_point);
    
    // Set camera pose constant
    if (problem_->HasParameterBlock(tvr->poseC_->error_))
      problem_->SetParameterBlockConstant(tvr->poseC_->error_);
    // Set tag pose constant
    //if (problem_->HasParameterBlock(tvr->tagTj_pose_->error_))
    //  problem_->SetParameterBlockConstant(tvr->tagTj_pose_->error_);
    // Set tag size constant
    if (problem_->HasParameterBlock(&tvr->tagTj_size_->error_))
      problem_->SetParameterBlockConstant(&tvr->tagTj_size_->error_);
    // Set camera intrinsics constant
    //if (problem_->HasParameterBlock(tvr->camera_->error_))
    //  problem_->SetParameterBlockConstant(tvr->camera_->error_);
    
    return true;
  }
  
  // Build the problem - at the creation of the problem object
  bool AddDataToProblem() {
    
    int64_t problem_data_start_ts = swf_iterations_.DataBeginTimestamp();
    
    // Locate the PreProblemBegin marker for tag view residuals
    if (!tag_view_residuals_->PositionFixedPointBeforeTimestamp("PreProblemBegin",
                                                                problem_data_start_ts)) {
      LOG(ERROR) << "Could not locate the problem_data_start_ts = " << problem_data_start_ts;
      return false;
    }
    VLOG(2) << "Problem data start ts was found. PreProblemBegin marker = " <<
        tag_view_residuals_->FixedPointToString("PreProblemBegin");
    
    // Add Tag view residuals to the problem
    for (TagViewQueueFixedPointType fp = tag_view_residuals_->GetNextFixedPoint("PreProblemBegin");
         tag_view_residuals_->IsNotPastFixedPoint("ProblemEnd", fp);
         tag_view_residuals_->IncrementFixedPoint(fp))
    {
      // This is a calibration state (camera instrinsics) we are calculating a marginal,
      //  so we do not add any past residuals.
      //AddTagViewResidualToProblem(fp);
      //iteration_record_.iteration_counters["TagViewResidualsAdded"]++;
    }
    return true;
  }
  
  // Add new residuals to the problem
  bool AddNewDataToProblem() {
    
    for (TagViewQueueFixedPointType fp = tag_view_residuals_->GetNextFixedPoint("ProblemEnd");
         tag_view_residuals_->IsNotPastTheEnd(fp);
         tag_view_residuals_->IncrementFixedPoint(fp))
    {
      AddTagViewResidualToProblem(fp);
      iteration_record_.iteration_counters["TagViewResidualsAdded"]++;
    }
    // Reset the ProblemEnd marker
    tag_view_residuals_->SetFixedPoint("ProblemEnd"); // Sets fixed point at the end
    VLOG(2) << "TagViewResiduals problem end = " << tag_view_residuals_->FixedPointToString("ProblemEnd");
    
    return true;
  }
  
  /* Mark constant states - at the creation of problem object
   * All states before the sliding window are marked constant */
  bool MarkStatesConstant() {
    
    // Locate the PreWindowBegin marker for tag view residuals
    int64_t window_start_ts = swf_iterations_.SlidingWindowTimestamp();
    if (!tag_view_residuals_->PositionFixedPointBeforeTimestamp("PreWindowBegin", window_start_ts)) {
      LOG(WARNING) << "Could not locate the window_start_ts = " << window_start_ts;
      return false;
    }
    VLOG(2) << "Solving window start ts was found. PreWindowBegin marker = " <<
        tag_view_residuals_->FixedPointToString("PreWindowBegin");
    
    // Mark tag view states constant
    for (TagViewQueueFixedPointType fp = tag_view_residuals_->GetNextFixedPoint("PreProblemBegin");
         tag_view_residuals_->IsNotPastFixedPoint("PreWindowBegin", fp);
         tag_view_residuals_->IncrementFixedPoint(fp))
    {
      // This is a calibration state (camera instrinsics) we are calculating a marginal,
      //  so we do not add any past residuals.
      //MarkTagViewResidualStatesConstant(fp);
      //iteration_record_.iteration_counters["TagViewResidualsMarked"]++;
    }
    return true;
  }
  
  /* Mark new constant states
   * All states before the sliding window are marked constant */
  bool MarkNewStatesConstant() {
    
    // Make a copy of the PreWindowBegin fixed point
    const TagViewQueueFixedPointType last_pre_window_begin =
        tag_view_residuals_->GetFixedPoint("PreWindowBegin");
    
    // Locate the PreWindowBegin marker for tag view residuals
    int64_t window_start_ts = swf_iterations_.SlidingWindowTimestamp();
    if (!tag_view_residuals_->PositionFixedPointBeforeTimestamp("PreWindowBegin", window_start_ts)) {
      LOG(WARNING) << "Could not locate the window_start_ts = " << window_start_ts;
      return false;
    }
    VLOG(2) << "Solving window start ts was found. PreWindowBegin marker = " <<
        tag_view_residuals_->FixedPointToString("PreWindowBegin");
    
    // Mark tag view states constant
    for (TagViewQueueFixedPointType fp = tag_view_residuals_->NextFixedPoint(last_pre_window_begin);
         tag_view_residuals_->IsNotPastFixedPoint("PreWindowBegin", fp);
         tag_view_residuals_->IncrementFixedPoint(fp))
    {
      MarkTagViewResidualStatesConstant(fp);
      iteration_record_.iteration_counters["TagViewResidualsMarked"]++;
    }
    return true;
  }
  
  // Solve the problem
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
  
  bool ResultsChanged() const {
    int32_t num_iterations = solver_summary_.iterations.size() - 1;
    //VLOG(1) << "  number of iterations = " << num_iterations;
    return (num_iterations > 0);
  }
  
  // Report the results
  bool ReportResults() {
    VLOG(1) << "Camera intrinsics = " << camera_intrinsics_.ToString(false);
  }
  
  // Build a state message from the results
  bool BuildCameraIntrinsicsStateMessage() {
    // Build message
    camera_intrinsics_message_.Clear();
    anantak::HeaderMsg* hdr_msg =
        camera_intrinsics_message_.mutable_header();
    anantak::CameraIntrinsicsStateMessage* cam_msg =
        camera_intrinsics_message_.mutable_camera_intrinsics_state_msg();
    // Set header
    hdr_msg->set_timestamp(iteration_record_.end_ts);
    hdr_msg->set_type("CameraIntrinsics");
    hdr_msg->set_recieve_timestamp(iteration_record_.end_ts);
    hdr_msg->set_send_timestamp(iteration_record_.end_ts);
    // Set message
    cam_msg->set_camera_num(options_.tag_camera_ids[0]);
    camera_intrinsics_.SetCameraIntrinsics(cam_msg);
    camera_intrinsics_.SetCovariance(cam_msg);
    
    // Check the message
    //VLOG(1) << "msg ts = " << camera_intrinsics_message_.header().timestamp()
    //    << " should be = " << iteration_record_.end_ts;
    
    return true;
  }
  
  // Accessor for the state message
  inline const anantak::MessageType& GetResultsMessage() {
    BuildCameraIntrinsicsStateMessage();
    return camera_intrinsics_message_;
  }
  
  // Wall time utility 
  inline int64_t GetWallTime() {
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) (tv.tv_sec * 1000000 + tv.tv_usec);  
  }
  
  // Destructor
  virtual ~BeaconFilter() {
    LOG(INFO) << "Destructing Beacon filter. All objects should self-destruct.";
  }
  
};  // BeaconFilter


} // namespace
