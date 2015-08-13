/**
 * Camera calibration filter
 */
// std includes
#include <array>
#include <sstream>
#include <iomanip>

// Anantak includes
#include "Filter/timed_circular_queue.h"
#include "Filter/observation.h"
#include "Models/model.h"
#include "ModelsLib1.h"

// OpenCV includes
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

// Protocol buffers - specific to beacon model
#include "state_messages.pb.h"

namespace anantak {

/* Camera intrinsics filter
 * Runs a sliding window filter for instrinsics filtering using a AprilTag calibration target
 */
class CameraIntrinsicsFilter : public anantak::Model {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  typedef Eigen::Matrix<double,3,4> Matrix3x4Type;
  typedef Eigen::Matrix<double,2,4> Matrix2x4Type;
  
  // Options for intrinsics filter
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
    uint16_t max_tags_per_image;          // Maximum number of images seen in an image
    std::vector<uint32_t> tag_camera_ids; // IDs of tag cameras to be processed
    
    // Calibration target specifications
    std::string calibration_target_config_file;
    
    // Camera intrinsics specifications
    bool camera_intrinsics_message_is_available;        // camera_intrinsics_message_is_available
    anantak::SensorMsg camera_state_message;            // camera_intrinsics_message
    anantak::CameraIntrinsicsInitMessage camera_intrinsics_init;
    double camera_angle_of_view;
    std::array<double,2> camera_image_size;
    double camera_angle_of_view_stdev;
    double camera_center_stdev;
    
    // Tag view residuals
    //anantak::DynamicAprilTagViewResidual::Options apriltag_view_residual_options;    
    
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
      max_tags_per_image(35),
      tag_camera_ids({0}),
      
      calibration_target_config_file("config/camera_intrisics_apriltag_calibration_target.cfg"),
      
      camera_intrinsics_message_is_available(false),
      camera_angle_of_view(100*kRadiansPerDegree),
      camera_image_size{{640, 480}},  // array uses aggregates initialization, so double braces(!?)
      camera_angle_of_view_stdev(30*kRadiansPerDegree),  // Large starting uncertainty in angle of view
      camera_center_stdev(100)  // Large uncertainty in center location
      
      //apriltag_view_residual_options(1.0)
    {
      // Read the Programs Setup file
      std::string config_file_path = anantak::GetProjectSourceDirectory() + "/" + config_filename;
      VLOG(1) << "Config file = " << config_file_path;
      std::unique_ptr<anantak::CameraCalibratorConfig> config =
          anantak::ReadProtobufFile<anantak::CameraCalibratorConfig>(config_file_path);
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
        max_tags_per_image = config->max_tags_per_image();
        camera_intrinsics_init = config->camera_init();
        calibration_target_config_file = config->calibration_target_config();
        //apriltag_view_residual_options =
        //    anantak::DynamicAprilTagViewResidual::Options(config->apriltag_view_resid_options());
      }
    }
    
    std::string ToString() {
      return "History: "+std::to_string(states_history_interval)+" Frequency: "+std::to_string(states_frequency)+"\n"+
        sliding_window_options.ToString()+
        "\nCamera id: "+std::to_string(tag_camera_ids[0])+" Camera max freq: "+std::to_string(max_tag_camera_frequency)+
        " Camera max tags per image: "+std::to_string(max_tags_per_image);
    }
    
    bool SetStartingCameraIntrinsics(const anantak::SensorMsg& msg) {
      camera_intrinsics_message_is_available = true;
      camera_state_message = msg;    // copy
      //VLOG(1) << "Setting starting camera intrinsics in options = "
      //  << camera_state_message.Intrinsics().fx() << " " << camera_state_message.Intrinsics().fy() << " "
      //  << camera_state_message.Intrinsics().cx() << " " << camera_state_message.Intrinsics().cy();
      return true;
    }
    
  }; // Options
  
  // Grid to help collect target observations
  struct Grid {
    
    struct Cell {
      double x1,y1,x2,y2,a1,a2;
      std::string str;
      bool is_empty_;
      anantak::AprilTagMessage apriltag_msg_;    // Observation
      
      Cell(double x_range, double y_range, double a_range, double x, double y, double a):
        is_empty_(true)
      {
        x1 = x - x_range*0.5;
        x2 = x + x_range*0.5;
        y1 = y - y_range*0.5;
        y2 = y + y_range*0.5;
        a1 = std::max(0., a - a_range*0.5);
        a2 = std::max(0., a + a_range*0.5);
        std::ostringstream ss; ss << std::fixed
            << std::setprecision(0) << a1 << "-" << std::setprecision(0) << a2;
        str = ss.str();
      }
      
      bool FallsInCell(const double& x, const double& y, const double& a) const {
        return (x>=x1 && x<=x2 && y>=y1 && y<=y2 && a>=a1 && a<=a2);
      }
      
      bool AddObservation(const Eigen::Vector2d& position, const double& angle,
          const anantak::AprilTagMessage& msg) {
        if (!is_empty_) return false; 
        if (FallsInCell(position[0], position[1], angle)) {
          apriltag_msg_ = msg; // copy observation
          is_empty_ = false;
          return true;
        }
        return false;
      }
      
      bool IsEmpty() const {return is_empty_;}
      
    };
    
    // Cells in the grid
    std::vector<Cell> cells_;
    bool initialized_;
    
    Grid():
      initialized_(false)
    {}
    
    bool IsFull() const {
      bool is_full = true;
      for (auto it=cells_.begin(); is_full && it!=cells_.end(); it++) {
        is_full &= !it->IsEmpty();
      }
      //if (is_full) VLOG(1) << "Grid is full"; else VLOG(1) << "Grid is not full";
      return is_full;
    }
    
    // Construct the grid using width, height, number of cells and shrinkage factor
    bool SetSize(int32_t w, int32_t h, int32_t nx, int32_t ny, double a_max, double a_range, float shrinkage = 0.5) {
      double cell_w = double(w/nx);
      double cell_h = double(h/ny);
      double x_range = cell_w * shrinkage;
      double y_range = cell_h * shrinkage;
      int32_t nx_half = nx/2;
      int32_t ny_half = ny/2;
      double cell_a = a_max/double(std::max(nx_half, ny_half));
      for (int i=0; i<nx; i++) {
        for (int j=0; j<ny; j++) {
          double x = (double(i)+0.5)*cell_w;
          double y = (double(j)+0.5)*cell_h;
          double a = cell_a*std::sqrt(double((i-nx_half)*(i-nx_half)+(j-ny_half)*(j-ny_half)));
          Cell cell(x_range, y_range, a_range, x,y,a);
          cells_.push_back(cell);
        }
      }
      VLOG(1) << "Constructed the grid with nx,ny: " << nx << " " << ny;
      initialized_ = true;
      return true;
    }
    
    // Add an obervation to the grid
    bool AddObservation(const Eigen::Vector2d& position, const double& angle,
        const anantak::AprilTagMessage& msg) {
      bool was_added = false;
      for (auto it=cells_.begin(); !was_added && it!=cells_.end(); it++) {
        was_added = it->AddObservation(position, angle, msg);
      }
      return was_added;
    }
    
    bool Reset() {
      for (auto it=cells_.begin(); it!=cells_.end(); it++) {
        it->is_empty_ = true;
      }
      return true;
    }
    
    bool SaveObservationsToFile(const std::string& filename) {
      // Use the filemessagewriter to save messages. We will have to construct the SensorMsg.
      anantak::MessageFileWriter file_writer;
      if (!file_writer.Open(filename)) {
        LOG(ERROR) << "Can not write to this file. " << filename;
        return false;
      }
      
      int64_t tm = anantak::GetWallTime();
      
      for (auto it=cells_.begin(); it!=cells_.end(); it++) {
        if (!it->IsEmpty()) {
          // Construct a SensorMsg
          anantak::SensorMsg msg;
          anantak::HeaderMsg* hdr_msg = msg.mutable_header();
          anantak::AprilTagMessage* april_msg = msg.mutable_april_msg();
          // Build header message
          hdr_msg->set_timestamp(tm);
          hdr_msg->set_type("AprilTags");
          hdr_msg->set_recieve_timestamp(tm);
          hdr_msg->set_send_timestamp(tm);
          // Build Apriltag message
          *april_msg = it->apriltag_msg_;
          
          // Write to file
          if (!file_writer.WriteMessage(msg)) {
            LOG(ERROR) << "Could not write message to file";
          }
        }
      }
      
      file_writer.Close();
    } // Save observations
    
    bool GetObservations(std::vector<anantak::SensorMsg>* msgs_vec) const {
      int64_t tm = anantak::GetWallTime();
      for (auto it=cells_.begin(); it!=cells_.end(); it++) {
        if (!it->IsEmpty()) {
          // Construct a SensorMsg
          anantak::SensorMsg msg;
          anantak::HeaderMsg* hdr_msg = msg.mutable_header();
          anantak::AprilTagMessage* april_msg = msg.mutable_april_msg();
          // Build header message
          hdr_msg->set_timestamp(tm);
          hdr_msg->set_type("AprilTags");
          hdr_msg->set_recieve_timestamp(tm);
          hdr_msg->set_send_timestamp(tm);
          // Build Apriltag message
          *april_msg = it->apriltag_msg_;
          // Save the message to given vector
          msgs_vec->emplace_back(msg); 
        }
      }
      return true;
    } // GetObservations
    
  };  // Grid
  
  // Display for the filter - owns all display/drawing code
  struct FilterDisplay {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    // Main canvas
    cv::Size size_;
    cv::Mat display_image_;
    std::string name_;
    
    bool initialized_;
    bool window_showing_;
    
    double rectangle_factor_;
    Eigen::Matrix<double,4,6> rotn_rectangle_;
    Eigen::Matrix3d cam_intrinsics_;
    Eigen::Matrix<double,3,4> rotn_transform_;
    
    cv::Scalar CV_RED, CV_BLUE, CV_BLACK, CV_WHITE, CV_GREEN, CV_YELLOW;
    
    FilterDisplay(const std::string& name):
        name_(name), size_(0,0), display_image_(), initialized_(false), window_showing_(false),
        rectangle_factor_(0.20)
    {
      VLOG(1) << "Created the filter display. Yet to be initialized.";
      CV_RED = cv::Scalar(0,0,255);
      CV_BLUE = cv::Scalar(255,0,0);
      CV_GREEN = cv::Scalar(0,255,0);
      CV_BLACK = cv::Scalar(0,0,0);
      CV_WHITE = cv::Scalar(255,255,255);
      CV_YELLOW = cv::Scalar(0,255,255);
    }
    
    bool SetSize(const int32_t& width, const int32_t& height) {
      size_ = cv::Size(width, height);
      display_image_.create(size_, CV_8UC3);
      initialized_ = true;
      VLOG(1) << "Initialized filter display with width, height: " << width << " " << height;
      
      double w = width * rectangle_factor_ * 0.5;
      double h = height * rectangle_factor_ * 0.5;
      
      rotn_rectangle_ << -w,  w,  w, -w,  0,  0,
                         -h, -h,  h,  h,  0,  0,
                          0,  0,  0,  0,  0,  h,
                          1,  1,  1,  1,  1,  1;
      double f = 0.5*std::max(width, height);
      cam_intrinsics_ << f, 0, 0.5*width,
                         0, f, 0.5*height,
                         0, 0, 1.0;
      rotn_transform_.setZero();
      rotn_transform_(2,3) = f;
      
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
    
    // Draw rotation
    bool DrawRotation(const Eigen::Matrix3d& rotn, const double& angle) {
      
      rotn_transform_.block<3,3>(0,0) = rotn;
      // Calculate corners of rectangle
      Eigen::Matrix<double,3,6> xyz = rotn_transform_ * rotn_rectangle_;
      Eigen::Matrix<double,3,6> uvz = cam_intrinsics_ * xyz;
      Eigen::Matrix<double,2,6> uv;
      uv.row(0) = uvz.row(0).cwiseQuotient(uvz.row(2));
      uv.row(1) = uvz.row(1).cwiseQuotient(uvz.row(2));
      
      // Draw rotation
      cv::Point2d p0(uv(0,0), uv(1,0));
      cv::Point2d p1(uv(0,1), uv(1,1));
      cv::Point2d p2(uv(0,2), uv(1,2));
      cv::Point2d p3(uv(0,3), uv(1,3));
      cv::Point2d p4(uv(0,4), uv(1,4));
      cv::Point2d p5(uv(0,5), uv(1,5));
      cv::line(display_image_, p0, p1, CV_RED, 2);
      cv::line(display_image_, p1, p2, CV_RED, 2);
      cv::line(display_image_, p2, p3, CV_RED, 2);
      cv::line(display_image_, p3, p0, CV_RED, 2);
      //cv::line(display_image_, p4, p5, CV_RED, 2);
      cv::circle(display_image_, p4, 1, CV_RED, 2);
      
      std::ostringstream ss; ss << std::fixed << std::setprecision(0) << angle;
      cv::putText(display_image_, ss.str(), p4, CV_FONT_NORMAL, .5, CV_WHITE, 1);
      
      return true;
    }

    bool DrawPosition(const Eigen::Vector2d& posn) {
      cv::Point2d pc(posn[0], posn[1]);
      cv::circle(display_image_, pc, 3, CV_RED, 2);
      return true;
    }
    
    // Draw a calibration target
    bool DrawPoseState(const anantak::PoseState& pose,
        const anantak::CameraState& camera) {
      
      Eigen::Vector3d uvzc = camera.Intrinsics().K() * pose.P();
      // Draw center point 
      cv::Point2d pc(uvzc[0]/uvzc[2], uvzc[1]/uvzc[2]);
      cv::circle(display_image_, pc, 3, CV_RED, 2);
      
      // Setup rotation transform
      rotn_transform_.block<3,3>(0,0) = Eigen::Matrix3d(pose.Q().conjugate());
      // Calculate corners of rectangle
      Eigen::Matrix<double,3,6> xyz = rotn_transform_ * rotn_rectangle_;
      Eigen::Matrix<double,3,6> uvz = cam_intrinsics_ * xyz;
      Eigen::Matrix<double,2,6> uv;
      uv.row(0) = uvz.row(0).cwiseQuotient(uvz.row(2));
      uv.row(1) = uvz.row(1).cwiseQuotient(uvz.row(2));
      
      // Draw rotation
      cv::Point2d p0(uv(0,0), uv(1,0));
      cv::Point2d p1(uv(0,1), uv(1,1));
      cv::Point2d p2(uv(0,2), uv(1,2));
      cv::Point2d p3(uv(0,3), uv(1,3));
      cv::Point2d p4(uv(0,4), uv(1,4));
      cv::Point2d p5(uv(0,5), uv(1,5));
      cv::line(display_image_, p0, p1, CV_RED, 2);
      cv::line(display_image_, p1, p2, CV_RED, 2);
      cv::line(display_image_, p2, p3, CV_RED, 2);
      cv::line(display_image_, p3, p0, CV_RED, 2);
      //cv::line(display_image_, p4, p5, CV_RED, 2);
      cv::circle(display_image_, p4, 1, CV_RED, 2);
      
      // Angle between tag normal and image plane
      double angle = std::acos(-rotn_transform_(2,2)) * kDegreesPerRadian;
      std::ostringstream ss; ss << std::fixed << std::setprecision(0) << angle;
      cv::putText(display_image_, ss.str(), p4, CV_FONT_NORMAL, .5, CV_WHITE, 1);
      
      return true;
    }
    
    // Draw the grid
    bool DrawGrid(const anantak::CameraIntrinsicsFilter::Grid& grid) {
      if (!grid.initialized_) return false;
      for (int i=0; i<grid.cells_.size(); i++) {
        cv::Point2d p1(grid.cells_[i].x1, grid.cells_[i].y1);
        cv::Point2d p2(grid.cells_[i].x2, grid.cells_[i].y2);
        cv::Scalar draw_color = grid.cells_[i].IsEmpty() ? CV_BLUE : CV_GREEN;
        cv::rectangle(display_image_, p1, p2, draw_color, 1);
        cv::putText(display_image_, grid.cells_[i].str, p1, CV_FONT_NORMAL, .5, draw_color, 1);
      }
      
      return true;
    }
    
    // Draw a message
    bool ShowMessage(const std::string msg) {
      cv::Point2d p0(size_.width*0.1, size_.height*0.1);
      cv::putText(display_image_, msg, p0, CV_FONT_NORMAL, .5, CV_WHITE, 1);      
      return true;
    }
    
  };  // FilterDisplay
  
  // Options object
  CameraIntrinsicsFilter::Options options_;
  
  // Running mode
  enum FilterRunModeType {kInitializing, kFiltering};
  FilterRunModeType filter_run_mode_;
  
  // Filter display objects
  bool show_;
  CameraIntrinsicsFilter::FilterDisplay filter_display_;
  CameraIntrinsicsFilter::Grid grid_;
  
  // Iteration interval. Filter runs at a higher frequency when initializing, then it drops.
  uint64_t iteration_interval_;
  
  // Sliding window filter objects
  anantak::SlidingWindowFilterIterations swf_iterations_;
  anantak::IterationRecord iteration_record_;
  
  // Optimization objects
  std::unique_ptr<ceres::Problem> problem_;
  ceres::Problem::Options problem_options_;
  ceres::Solver::Options solver_options_;
  ceres::Solver::Summary solver_summary_;
  
  // Calibration target
  CameraCalibrationTarget target_;
  
  // Camera
  CameraState camera_;
  
  // Results
  anantak::SensorMsg camera_message_;
  
  // Helpers
  uint64_t inter_state_interval_;
  
  // Constructor with options defined
  CameraIntrinsicsFilter(
      const anantak::CameraIntrinsicsFilter::Options& options,
      const int64_t& filter_start_ts = 0):
    options_(options),
    swf_iterations_(options.sliding_window_options), iteration_record_(filter_start_ts),
    problem_(nullptr), problem_options_(), solver_options_(), solver_summary_(),
    target_(options.calibration_target_config_file),
    camera_(options.tag_camera_ids.at(0)),                  // initiating at first camera
    filter_display_("IntrinsicsFilter")
  {
    InitiateFilter();
  }
  
  const uint64_t& IterationInterval() const {return iteration_interval_;}
  bool Finished() const {return (filter_run_mode_ == kFiltering);}
  
  bool InitiateFilter() {
    LOG(INFO) << "Creating camera intrinsics filter";
    
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
    
    //// Initiate beacon tag
    //{if (options_.beacon_config.tags_size()<1) {
    //  LOG(ERROR) << "Provided beacon config has no tags! Using default values";
    //  Eigen::Quaterniond TqB = Eigen::Quaterniond::Identity();
    //  Eigen::Vector3d BpT = Eigen::Vector3d::Zero();
    //  beacon_tag_.SetZero();
    //  beacon_tag_.Create(&options_.beacon_tag_id, &TqB, &BpT, &options_.beacon_tag_size);
    //  // Set covariances
    //  beacon_tag_.pose_.covariance_.setZero();
    //  beacon_tag_.size_.covariance_ = options_.beacon_tag_size_stdev * options_.beacon_tag_size_stdev;
    ///} else {
    //  LOG(INFO) << "Using config file provided beacon tag";
    //  beacon_tag_.Create(options_.beacon_config.tags(0));
    ///}
    //// Report
    //VLOG(1) << "Beacon tag id, size = " << beacon_tag_.tag_id_ << " " << beacon_tag_.size_.state_;
    //VLOG(1) << "Beacon tag pose = " << beacon_tag_.pose_.LqvG_.transpose()
    //}    << ", " << beacon_tag_.pose_.GpL_.transpose();
    
    // Check if the calibration target was initialized
    if (!target_.IsInitialized()) {
      LOG(FATAL) << "Calibration target was not initialized. Quit.";
      return false;
    }
    
    //if (target_.Size() == 0) {
    //  LOG(ERROR) << "Calibration target does not have any tags!? " << target_.Size();
    //  return false;
    ///}
    //target_tag_size_.SetZero();
    //target_tag_size_.state_ = target_.TagSize(0);
    //target_tag_size_.covariance_ = 0.;
    //}VLOG(1) << "Target tag size = " << target_tag_size_;
    
    // Initiate camera instrinsics
    //  Best case is camera_state_message is available
    //  If not, use camera_intrinsics_init from a config file
    //  If not, use the hard coded values
    if (options_.camera_intrinsics_message_is_available) {
      LOG(INFO) << "Using camera intrinsics message from a data queue";
      if (!camera_.SetFromMessage(options_.camera_state_message)) {
        LOG(FATAL) << "Could not set camera from given message";
        return false;
      }
      filter_run_mode_ = kFiltering;
      filter_display_.SetSize(options_.camera_intrinsics_init.image_size(0),
                              options_.camera_intrinsics_init.image_size(1));
    } else if (options_.camera_intrinsics_init.image_size_size() == 2) { 
      LOG(INFO) << "Using config file provided camera intrinsics init";
      camera_.SetIntrinsicsFromInitMessage(options_.camera_intrinsics_init);
      filter_run_mode_ = kInitializing;
      filter_display_.SetSize(options_.camera_intrinsics_init.image_size(0),
                              options_.camera_intrinsics_init.image_size(1));
    } else {
      LOG(ERROR) << "Using default values to set starting camera intrinsics";
      camera_.SetIntrinsicsFromHeuristics(options_.camera_angle_of_view,
          options_.camera_image_size[0], options_.camera_image_size[1],
          options_.camera_angle_of_view_stdev, options_.camera_center_stdev);
      camera_.SetImageSize(options_.camera_image_size[0], options_.camera_image_size[1]);
      filter_run_mode_ = kInitializing;
      filter_display_.SetSize(options_.camera_image_size[0], options_.camera_image_size[1]);
    }
    // Report what was used
    //VLOG(1) << "Starting camera intrinsics = " << camera_intrinsics_.ToString();
    
    // Setup the grid
    grid_.SetSize(filter_display_.size_.width, filter_display_.size_.height, 3, 3, 40, 30, 0.4);
      
    // Initiate camera intrinsics prior
    //camera_intrinsics_prior_.Create(&camera_intrinsics_, true);
    
    // Initialize the iteration interval depending on the mode
    if (filter_run_mode_ == kInitializing) {
      iteration_interval_ =  100000;
      Show();
    } else {
      iteration_interval_ = 1000000;
      Hide();
    }
    VLOG(1) << "Iteration interval: " << iteration_interval_;
    
    // Initialize states
    SetStartingStates();
    return true;
  }
  
  // Allocate memory for queues 
  bool AllocateMemory() {
    LOG(INFO) << "Allocating memory for the camera intrinsics filter queues";
    
    int32_t num_states_to_keep = options_.states_frequency * options_.states_history_interval / 1000000;
    LOG(INFO) << "  num_states_to_keep = " << num_states_to_keep
        << " at states_frequency = " << options_.states_frequency << " Hz";
    
    //// Camera poses
    //std::unique_ptr<anantak::TimedCircularQueue<anantak::Pose3dState>> cam_states_ptr(
    //    new anantak::TimedCircularQueue<anantak::Pose3dState>(num_states_to_keep));
    //camera_poses_ = std::move(cam_states_ptr);
    //
    //// Target poses
    //std::unique_ptr<anantak::TimedCircularQueue<anantak::Pose3dState>> bc_poses_ptr(
    //    new anantak::TimedCircularQueue<anantak::Pose3dState>(num_states_to_keep));
    //target_poses_ = std::move(bc_poses_ptr);
    
    //// Tag residuals
    //int32_t num_tag_views_to_keep = options_.states_history_interval / 1000000
    //    * options_.max_tag_camera_frequency * options_.max_tags_per_image;
    //LOG(INFO) << " tag view residuals queues length = " << num_tag_views_to_keep;
    
    //std::unique_ptr<anantak::TimedCircularQueue<anantak::DynamicAprilTagViewResidual>> view_queue_ptr(
    //    new anantak::TimedCircularQueue<anantak::DynamicAprilTagViewResidual>(num_tag_views_to_keep));
    //tag_view_residuals_ = std::move(view_queue_ptr);
    
    return true;
  }
  
  /* Set starting states
   * Machine starting state is at origin. Target starting state is provided via options.
   */
  bool SetStartingStates() {
    // Calculate the starting state timestamp
    int64_t starting_state_ts = (iteration_record_.begin_ts / inter_state_interval_) * inter_state_interval_;
    VLOG(1) << "Using starting state ts = " << starting_state_ts;
    
    return true;
  }
  
  bool PassAllMessages(const anantak::SensorMsg& msg) const {
    return true;
  }
  
  bool PassAprilTagCameraMessages(const anantak::SensorMsg& msg) const {
    if (!msg.has_april_msg()) {
      // There is no apriltag message
      VLOG(2) << "There is no apriltag message";
      return false;
    }
    const int32_t& cam_id = msg.april_msg().camera_num();
    // Check the camera number, this should be in the list for this filter
    auto it = std::find(options_.tag_camera_ids.begin(), options_.tag_camera_ids.end(), cam_id);
    if (it == options_.tag_camera_ids.end()) {
      VLOG(2) << "Camera id found is not processed by this filter " << cam_id;
      // This camera is not processed by this filter
      return false;
    }
    return true;
  }
  
  bool ExtractSensorMessagesFromObservations(
      const anantak::ObservationsVectorStoreMap& observations_map,
      const std::string msg_type,
      anantak::SensorMessageFilterType filter_function,
      std::vector<anantak::SensorMsg>* sensor_msgs)
  {
    // Clear return container
    sensor_msgs->clear();
    
    // Go through messages
    for (auto i_map=observations_map.begin(); i_map!=observations_map.end(); i_map++) {
      if (i_map->first == msg_type) {
        int32_t num_msgs = i_map->second.n_observations;
        VLOG(1) << "Extracting messages of type " << i_map->first << " num " << num_msgs;
        for (int i_obs=0; i_obs<num_msgs; i_obs++) {
          anantak::MessageType* msg_ptr = i_map->second.observations->at(i_obs).get();
          if (!msg_ptr) {
            LOG(ERROR) << "Message pointer is null. Skipping.";
            continue;
          }
          // Cast the pointer to Sensor message. 
          if (anantak::SensorMsg* sensor_msg_ptr = dynamic_cast<anantak::SensorMsg*>(msg_ptr)) {
            if (filter_function(*sensor_msg_ptr)) {
              sensor_msgs->push_back(*sensor_msg_ptr);
            }
          } else {
            LOG(ERROR) << "Could not cast protocol buffer message to SensorMsg. Skipping.";
            continue;
          }
          
        } // for each message in this type
      }
    } // for each type
    
    return true;
  }

  bool ExtractSensorMessagesFromObservations(
      const std::vector<std::unique_ptr<std::vector<anantak::SensorMsg>>>& observations_vec,
      const std::string msg_type,
      anantak::SensorMessageFilterType filter_function,
      std::vector<anantak::SensorMsg>* sensor_msgs)
  {
    // Clear return container
    sensor_msgs->clear();
    
    // Go through messages
    for (int i=0; i<observations_vec.size(); i++) {
      for (int j=0; j<observations_vec[i]->size(); j++) {
        const SensorMsg& msg = observations_vec[i]->at(j);
        if (filter_function(msg)) {
          sensor_msgs->emplace_back(msg);
        }
      }
    }
    
    return true;
  }
  
  // Run initializing iteration without any observations
  bool RunInitializingIteration(const int64_t& iteration_end_ts) {
    // nothing to do without any observations
    return true;
  }

  // Run initializing iteration - use observations from filter
  bool RunInitializingIteration(const int64_t& iteration_end_ts,
      const anantak::ObservationsVectorStoreMap& observations_map) {
    
    // Storage
    std::vector<anantak::SensorMsg> sensor_msgs;
    
    // Extract sensor messages
    anantak::SensorMessageFilterType filter_func =
        std::bind(&CameraIntrinsicsFilter::PassAprilTagCameraMessages, this, std::placeholders::_1);
    if (!ExtractSensorMessagesFromObservations(observations_map, "AprilTags", filter_func, &sensor_msgs)) {
      LOG(ERROR) << "Could not extract messages from observations, Skip.";
      return false;
    }
    
    return RunInitializingIteration(iteration_end_ts, sensor_msgs);
  }
  
  // Run initializing iteration - use observations from filter
  bool RunInitializingIteration(const int64_t& iteration_end_ts,
      const std::vector<std::unique_ptr<std::vector<anantak::SensorMsg>>>& observations_vec) {
    
    // Storage
    std::vector<anantak::SensorMsg> sensor_msgs;
    
    // Extract sensor messages
    anantak::SensorMessageFilterType filter_func =
        std::bind(&CameraIntrinsicsFilter::PassAprilTagCameraMessages, this, std::placeholders::_1);
    if (!ExtractSensorMessagesFromObservations(observations_vec, "AprilTags", filter_func, &sensor_msgs)) {
      LOG(ERROR) << "Could not extract messages from observations, Skip.";
      return false;
    }
    
    return RunInitializingIteration(iteration_end_ts, sensor_msgs);
  }
  
  // Run initializing iteration
  bool RunInitializingIteration(const int64_t& iteration_end_ts,
      const std::vector<anantak::SensorMsg>& sensor_msgs) {
    
    // Was there anything?
    if (sensor_msgs.size() == 0) {
      VLOG(2) << "No AprilTag message was found. Skip.";
      return true;
    }
    
    for (int i_msg=0; i_msg<sensor_msgs.size(); i_msg++) {
      
      const anantak::SensorMsg& msg = sensor_msgs.at(i_msg);
      
      // Quick check
      if (!msg.has_april_msg()) {
        LOG(ERROR) << "Last sensor message of AprilTag type has no apriltag message! Skipping.";
        return false;
      }
      
      // Just operate on the last message
      const int64_t& timestamp = msg.header().timestamp();
      const anantak::AprilTagMessage& apriltag_msg = msg.april_msg();
      
      // Get the calibration target pose using the message
      anantak::PoseState target_pose;   // Set to zero here
      target_.CalculateTargetPose(apriltag_msg, camera_, &target_pose);
      
      // Calculate target's image position and angle
      Eigen::Vector3d uvzc = camera_.Intrinsics().K() * target_pose.P();
      Eigen::Vector2d target_img_posn; target_img_posn << uvzc[0]/uvzc[2], uvzc[1]/uvzc[2];
      Eigen::Matrix3d target_rotn(target_pose.Q().conjugate());
      double target_img_angl = std::acos(-target_rotn(2,2)) * kDegreesPerRadian;
      
      // Check if this message should be included in the camera intrinsics estimation
      if (grid_.AddObservation(target_img_posn, target_img_angl, apriltag_msg)) {
        VLOG(1) << "Observation was added to the grid";
      }
      
      bool grid_is_full = grid_.IsFull();
      
      // Store observations for later processing
      
      // Show the image
      if (show_) {
        filter_display_.ClearImage();
        filter_display_.DrawGrid(grid_);
        filter_display_.DrawAprilTags(apriltag_msg);
        //filter_display_.DrawPoseState(target_pose, camera_intrinsics_);
        filter_display_.DrawPosition(target_img_posn);
        filter_display_.DrawRotation(target_rotn, target_img_angl);
        if (grid_is_full) filter_display_.ShowMessage("Grid is full. Calibrating...");
        filter_display_.DisplayImage();
      }
      
      if (grid_is_full) {
        grid_.SaveObservationsToFile("/data/camera_calibration_target_observations.pb.data");
        if (InitiateCalibration()) {
          
          // Prepare for filtering ** CHECK **
          //if (!SetStartingStatesAndResiduals()) {
          //  LOG(FATAL) << "Could not set starting states and residuals";
          ///}
          
          filter_run_mode_ = kFiltering;
          LOG(INFO) << "Switched filter from initializing to filtering.";
          return true;
          
        } else {
          // Initialization failed
          LOG(ERROR) << "Could not initiate calibration! Start again?";
          grid_.Reset();
          return false;        
        }
      }
      
    } // for each message
    
    return true;
  }
  
  bool InitiateCalibration() {
    
    LOG(INFO) << "Starting camera calibration";
    
    std::vector<anantak::SensorMsg> collected_msgs;
    grid_.GetObservations(&collected_msgs);
    VLOG(1) << "Collected " << collected_msgs.size() << " messages.";
    
    CalibrationTargetCameraCalibrator target_camera_calibrator;
    if (!target_camera_calibrator.InitiateCameraCalibration(
            target_, collected_msgs, camera_.ImageSize(), &camera_)) {
      LOG(ERROR) << "Could not initialize the camera calibration from data collected.";
      return false;
    }
    
    std::string savefile = "data/camera"+std::to_string(camera_.CameraNumber())+"_state.pb.data";
    camera_.SaveToFile(savefile);
    
    return true;
  }
  
  // Use the observations stored in the grid to guess calibrations
  bool InitiateCalibrationOld() {
    
    VLOG(1) << "Starting calibration";
    
    // Create vector of vector of correspondeneces for each message stored in the grid
    std::vector<std::vector<cv::Point3f>> all_target_points;
    std::vector<std::vector<cv::Point2f>> all_image_points;
    
    for (int i_image=0; i_image < grid_.cells_.size(); i_image++) {
      const anantak::AprilTagMessage& apriltag_msg = grid_.cells_[i_image].apriltag_msg_;
      std::vector<cv::Point3f> target_points;
      std::vector<cv::Point2f> image_points;
      // Go through each tag sighting in message
      for (int i_tag=0; i_tag<apriltag_msg.tag_id_size(); i_tag++) {
        const std::string& id = apriltag_msg.tag_id(i_tag);
        // Make sure that tag is present in the target
        if (!target_.TagIsPresent(id)) continue; 
        // Extract image points
        cv::Point2f ip0(apriltag_msg.u_1(i_tag), apriltag_msg.v_1(i_tag));
        cv::Point2f ip1(apriltag_msg.u_2(i_tag), apriltag_msg.v_2(i_tag));
        cv::Point2f ip2(apriltag_msg.u_3(i_tag), apriltag_msg.v_3(i_tag));
        cv::Point2f ip3(apriltag_msg.u_4(i_tag), apriltag_msg.v_4(i_tag));
        // Extract target points
        const Matrix3x4Type& tag_corners = target_.TagCorners(id);
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
    VLOG(1) << "Added " << all_target_points.size() << "/" << all_image_points.size() << " images.";
    
    // Run calibration using OpenCV functions
    VLOG(1) << "Running opencv calibration..";
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    cv::Size image_size();
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;
    cv::calibrateCamera(all_target_points, all_image_points, filter_display_.size_,
                        camera_matrix, dist_coeffs, rvecs, tvecs);    
    VLOG(1) << "Finished opencv calibration.";
    
    // Report results
    LOG(INFO) << "Camera matrix = " << camera_matrix;
    LOG(INFO) << "Distortion coeffs = " << dist_coeffs;
    
    
    
    // Store results in states
    
    // Now run non linear optimization using anantak maths
    //  This is necessary as we would like to know the covariance matrix of the fits
    
    return true;
  }
  
  /** Optimize camera intrinsics
   * Given AprilTag messages, initiated camera instrinsics state and calibration target,
   * run full optimization on the data */
  bool OptimizeCameraIntrinsics() {
    // Grid has all the AprilTag messages
    // Camera intrinsics have been initiated
    // Calibration target is known
    
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
    
    return true;
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
  
  // Create new residuals from the observations - coming from sliding window filter
  bool CreateIterationResiduals(
      const anantak::ObservationsVectorStoreMap& observations_map) {
    
    // Extract messages and create residuals
    for (auto i_map=observations_map.begin(); i_map!=observations_map.end(); i_map++) {
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
  
  // Create new residuals from the observations - coming from sliding window filter
  bool CreateIterationResiduals(
      const std::vector<std::unique_ptr<std::vector<anantak::SensorMsg>>>& observations_vec) {
    
    // Go through messages and create residuals
    for (int i=0; i<observations_vec.size(); i++) {
      for (int j=0; j<observations_vec[i]->size(); j++) {
        const SensorMsg& msg = observations_vec[i]->at(j);
        if (!CreateResidual(msg)) {
          LOG(ERROR) << "Could not create a residual from message. Skipping.";
          continue;
        }
      }
    }
    
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
  
  // Run iteration
  bool RunIteration(const int64_t& iteration_end_ts,
      const anantak::ObservationsVectorStoreMap& observations_map) {
    
    if (filter_run_mode_ == kInitializing) {
      return RunInitializingIteration(iteration_end_ts, observations_map);
    }
    
    if (!StartIteration(iteration_end_ts)) {
      LOG(ERROR) << "Could not start iteration";
      return false;
    }
    
    if (!CreateIterationStates(iteration_end_ts)) {
      LOG(ERROR) << "Could not create states for the iteration";
      return false;
    }
    
    if (!CreateIterationResiduals(observations_map)) {
      LOG(ERROR) << "Could not create residuals for the iteration";
      return false;
    }
    
    if (!RunFiltering(iteration_end_ts)) {
      LOG(ERROR) << "Could not run filtering operations";
      return false;
    }
    
    return true;
  }
  
  // Run iteration
  bool RunIteration(const int64_t& iteration_end_ts,
      const std::vector<std::unique_ptr<std::vector<anantak::SensorMsg>>>& observations_vec,
      const bool save_data = false) {
    
    if (filter_run_mode_ == kInitializing) {
      return RunInitializingIteration(iteration_end_ts, observations_vec);
    }
    
    if (!StartIteration(iteration_end_ts)) {
      LOG(ERROR) << "Could not start iteration";
      return false;
    }
    
    if (!CreateIterationStates(iteration_end_ts)) {
      LOG(ERROR) << "Could not create states for the iteration";
      return false;
    }
    
    if (!CreateIterationResiduals(observations_vec)) {
      LOG(ERROR) << "Could not create residuals for the iteration";
      return false;
    }
    
    if (!RunFiltering(iteration_end_ts)) {
      LOG(ERROR) << "Could not run filtering operations";
      return false;
    }
    
    return true;
  }
  
  // Run iteration with no observations provided
  bool RunIteration(const int64_t& iteration_end_ts) {
    
    if (filter_run_mode_ == kInitializing) {
      return RunInitializingIteration(iteration_end_ts);
    }
    
    if (!StartIteration(iteration_end_ts)) {
      LOG(ERROR) << "Could not start iteration";
      return false;
    }
    
    if (!CreateIterationStates(iteration_end_ts)) {
      LOG(ERROR) << "Could not create states for the iteration";
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
        return true;
      }
      
      if (!CreateResidualsForTargetView(hdr.timestamp(), apriltag_msg)) {
          LOG(ERROR) << "Could not create residuals for target view message. Skipping.";
          return false;
      }
      
    }  // type = "AprilTag"
    
    return true;
  }
  
  // Create residuals for the target sighting
  bool CreateResidualsForTargetView(const int64_t& timestamp, const anantak::AprilTagMessage& apriltag_msg) {
    
    return true;
  } // CreateResidualsForTargetView
  
  /* Calculate priors for the problem */
  bool CalculatePriors() {
    
    return true;
  }

  // Add priors to the problem
  bool AddPriors() {
    
    return true;
  }
  
  // Build the problem - at the creation of the problem object
  bool AddDataToProblem() {
    
    return true;
  }
  
  // Add new residuals to the problem
  bool AddNewDataToProblem() {
    
    return true;
  }
  
  /* Mark constant states - at the creation of problem object
   * All states before the sliding window are marked constant */
  bool MarkStatesConstant() {
    
    return true;
  }
  
  /* Mark new constant states
   * All states before the sliding window are marked constant */
  bool MarkNewStatesConstant() {
    
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
  
  bool AreResultsReady() const {
    int32_t num_iterations = solver_summary_.iterations.size() - 1;
    //VLOG(1) << "  number of iterations = " << num_iterations;
    return (num_iterations > 0);
  }
  
  // Report the results
  bool ReportResults() {
    VLOG(1) << "Camera intrinsics = " << camera_.ToString(1);
  }
  
  // Accessor for the state message
  inline const anantak::MessageType& GetResultsMessage() {
    camera_.CopyToMessage(&camera_message_, iteration_record_.end_ts);  // Build the message
    return camera_message_;
  }
  
  bool Show() {
    show_ = true;
    filter_display_.ShowWindow();
    return true;
  }
  
  bool Hide() {
    show_ = false;
    filter_display_.HideWindow();
    return true;
  }
  
  // Wall time utility 
  inline int64_t GetWallTime() {
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) (tv.tv_sec * 1000000 + tv.tv_usec);  
  }
  
  // Destructor
  virtual ~CameraIntrinsicsFilter() {
    LOG(INFO) << "Destructing camera instrinsics filter. All objects should self-destruct.";
  }
  
};  // CameraIntrinsicsFilter


} // namespace
