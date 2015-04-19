/**
 *  Loads a monocular camera calibration, set of raw image-names and their timings.
 *  Undistorts each image, tracks features across images using libviso2.
 *  Packs each sparse feature set in a protocol buffer and saves the file.
 *  Potentially we can also add AprilTag detection to this too.
 *
 */

/** Google libraries */
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <google/protobuf/message.h>

/** Old Anantak includes */
#include "../../../vision/include/anantak/parse_messages.hpp"
#include "../../../vision/include/anantak/dir_utils.hpp"
#include "../../../vision/include/anantak/tag_info.hpp"
#include "../../../vision/include/anantak/cvlibs_config.hpp"

/** Anantak includes */
#include "Filter/performance_tracker.h"
#include "DataQueue/message_file_writer.h"
#include "DataQueue/message_file_reader.h"
#include "DataQueue/message_file_stats.h"

/** Protocol buffers */
#include "sensor_messages.pb.h"

/** opencv */
#include <opencv2/highgui/highgui.hpp>

/** libviso2 includes */
#include "libviso2/matcher.h"
#include "libviso2/viso.h"    // not needed if bucketing parameters are created here

/** Apriltag includes */
#define TAG_FAMILY "4"
#define TAG_FAMILY_SIZE 600
#define TAG_DEBUG_PERFORMANCE 0
#define TAG_DEBUG_DRAW 0
#include "cv2cg/OpenCVHelper.h"
#include "cv2cg/apriltag/apriltag.hpp"
#include "cv2cg/apriltag/TagFamilyFactory.hpp"
#include "cv2cg/config.hpp"

/** Command line flags */
DEFINE_int32(camera, 1, "Camera number [1..4]");

/** Feature Tracker
 *  This object will keep the tracks of the features. It will pre-allocate memory in the heap
 *  to keep tracks of the features. It takes in the expected number of tracks and maximum length
 *  of each track to be kept in memory.
 *  Active and completed tracks are indicated in a boolean vector. 
 */
class FeatureTracker {
 public:
  
  struct Point2d {float u; float v; int32_t idx;};  // image coordinates and index
  typedef std::vector<Point2d> Point2dVec;
  typedef std::unique_ptr<Point2dVec> Point2dVecPtr;
  typedef std::vector<Point2dVecPtr> Point2dVecPtrVec;
  
  bool ResetStatistics() {
    total_tracks_ = 0;
    total_track_length_ = 0;
    average_track_length_ = 0;
    max_seen_length_ = 0;
    max_curr_length_ = 0;
    num_active_tracks_ = 0;
    num_completed_tracks_ = 0;
    return true;
  }
  
  // Constructor
  FeatureTracker(int32_t max_tracks, int32_t max_track_length) {
    max_tracks_ = max_tracks;
    max_track_length_ = max_track_length;
    // preallocate memory
    feature_tracks_.resize(max_tracks_);
    feature_tracks_.shrink_to_fit();
    for (int i=0; i<max_tracks_; i++) {
      Point2dVecPtr point_vec_ptr(new Point2dVec);
      point_vec_ptr->resize(max_track_length_);
      feature_tracks_[i] = std::move(point_vec_ptr);
    }
    track_lengths_.resize(max_tracks_);
    active_tracks_.resize(max_tracks_);
    completed_tracks_.resize(max_tracks_);
    for (int i=0; i<max_tracks_; i++) {
      track_lengths_[i] = 0;
      active_tracks_[i] = false;
      completed_tracks_[i] = false;
    }
    ResetStatistics();
  }
  
  // Destructor - all should self destruct as smart pointers are used
  ~FeatureTracker() {};
  
  /** Update the tracks with new correspondences
   *  Classify tracks as active or completed
   *  All current matches will be active
   *  All lost tracks will be marked completed
   */
  bool UpdateTracks(const std::vector<Matcher::p_match>& matches) {
    
    // Reset the tracks that were completed the last time
    for (int i=0; i<max_tracks_; i++) {
      if (completed_tracks_[i]) {
        track_lengths_[i] = 0;
      }
    }
    // Reset completed tracks to active tracks
    for (int i=0; i<max_tracks_; i++) {
      completed_tracks_[i] = active_tracks_[i];
      active_tracks_[i] = false;
    }
    
    // For each match
    //  Go through all active tracks, see if the feature is found
    //    If found, add the new feature to the track, increment index
    //    Mark this track as false in completed_tracks
    //  If not found, feature is new
    //    Find first track that is not active and not complete. if not found raise warning
    //    Add this feature to the track, mark as active
    num_continued_tracks_ = 0;
    num_new_tracks_ = 0;
    
    for (int i_match=0; i_match<matches.size(); i_match++) {
      
      // check if this feature exists in active tracks
      bool found_in_active = false;
      for (int i=0; i<max_tracks_; i++) {
        if (active_tracks_[i] && !found_in_active) {
          if (matches[i_match].i1p == feature_tracks_[i]->at(track_lengths_[i]).idx) {
            // found a match
            found_in_active = true;
            num_continued_tracks_++;
            active_tracks_[i] = true;
            completed_tracks_[i] = false;  // mark this track as not completed
            // copy the feature into the track
            if (track_lengths_[i] < max_track_length_-1) {
              track_lengths_[i]++;
              feature_tracks_[i]->at(track_lengths_[i]).idx = matches[i_match].i1c;
              feature_tracks_[i]->at(track_lengths_[i]).u = matches[i_match].u1c;
              feature_tracks_[i]->at(track_lengths_[i]).v = matches[i_match].v1c;
            } else {
              LOG(WARNING) << "Could not add feature as not enough storage " <<
                  track_lengths_[i] << " " << max_track_length_;
            }
          }
        }
      } // for i<max_tracks
      
      bool added_as_new = false;
      if (!found_in_active) {
        // feature is new, find an empty spot and add it
        int i=0;
        while (i<max_tracks_ && !added_as_new) {
          if (!active_tracks_[i] && !completed_tracks_[i]) {
            if (track_lengths_[i] == 0) {
              active_tracks_[i] = true;
              track_lengths_[i]++;
              feature_tracks_[i]->at(track_lengths_[i]).idx = matches[i_match].i1c;
              feature_tracks_[i]->at(track_lengths_[i]).u = matches[i_match].u1c;
              feature_tracks_[i]->at(track_lengths_[i]).v = matches[i_match].v1c;
              added_as_new = true;
              num_new_tracks_++;
            } else {
              LOG(ERROR) << "!active_tracks[i] && !completed_tracks[i] && track_lengths[i]!=0 " <<
                  active_tracks_[i] << " " << completed_tracks_[i] << " " << track_lengths_[i];
            }
          }
          i++;
        } // while
      }
      
      // check
      if (!found_in_active && !added_as_new) {
        LOG(WARNING) << "!found_in_active && !added_as_new - not enough space allocated?";
      }
      
    } // for i_match
    
    // Update statistics of completed track lengths
    num_active_tracks_ = matches.size();
    num_completed_tracks_ = 0;
    max_curr_length_ = 0;
    for (int i=0; i<max_tracks_; i++) {
      if (completed_tracks_[i]) {
        num_completed_tracks_++;
        if (max_seen_length_ < track_lengths_[i]) max_seen_length_ = track_lengths_[i];
        if (max_curr_length_ < track_lengths_[i]) max_curr_length_ = track_lengths_[i];
        total_track_length_ += double(total_track_length_);
      }
    }
    total_tracks_ += double(num_completed_tracks_);
    if (total_tracks_>0) average_track_length_ = total_track_length_/total_tracks_;
    
    return true;
  }
  
  // accessors - by reference usually. Caller to make sure feature tracker exists before using ref.
  const Point2dVecPtrVec& feature_tracks() {
    return feature_tracks_;
  }
  const std::vector<bool>& active_tracks() {
    return active_tracks_;
  }
  const std::vector<bool>& completed_tracks() {
    return completed_tracks_;
  }
  
  double average_track_length() {return average_track_length_;}
  int32_t num_active_tracks() {return num_active_tracks_;}
  int32_t num_completed_tracks() {return num_completed_tracks_;}
  int32_t max_seen_length() {return max_seen_length_;}
  int32_t max_curr_length() {return max_curr_length_;}
  int32_t num_continued_tracks() {return num_continued_tracks_;}
  int32_t num_new_tracks() {return num_new_tracks_;}
  
 private:
  // Features data
  Point2dVecPtrVec feature_tracks_;
  std::vector<int32_t> track_lengths_;
  std::vector<bool> active_tracks_, completed_tracks_;
  int32_t max_tracks_, max_track_length_;
  // Statistics
  double total_tracks_, total_track_length_, average_track_length_;
  int32_t max_seen_length_, max_curr_length_, num_active_tracks_, num_completed_tracks_;
  int32_t num_continued_tracks_, num_new_tracks_;
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
  // check if camera number is correct
  if (FLAGS_camera<1 || FLAGS_camera>4) {
    LOG(ERROR) << "Only know cameras in range [1..4]";
    return -1;
  }
  int32_t camera_num = int32_t(FLAGS_camera);
  
  // Select image files and calibration based on camera number
  std::string root_folder = "/home/manujnaman/";
  std::string directory = "Videos/Front/";
  std::string file_pattern = "Front_Left_%d.png";
  std::string timing_file = "Videos/Video_Data.txt";
  std::string calib_file = "Dropbox/ros/vision/src/calibration/Front_Stereo_4-Feb-2014/calib_cam_to_cam.txt";
  std::string left_right = "left";
  if (camera_num == 2) {
    directory = "Videos/Front/";
    file_pattern = "Front_Right_%d.png";
    timing_file = "Videos/Video_Data.txt";
    calib_file = "Dropbox/ros/vision/src/calibration/Front_Stereo_4-Feb-2014/calib_cam_to_cam.txt";
    left_right = "right";
  } else if (camera_num == 3) {
    directory = "Videos/Rear/";
    file_pattern = "Rear_Left_%d.png";
    timing_file = "Videos/Video_Data.txt";
    calib_file = "Dropbox/ros/vision/src/calibration/Rear_Stereo_4-Feb-2014/calib_cam_to_cam.txt";
    left_right = "left";
  } else if (camera_num == 4) {
    directory = "Videos/Rear/";
    file_pattern = "Rear_Right_%d.png";
    timing_file = "Videos/Video_Data.txt";
    calib_file = "Dropbox/ros/vision/src/calibration/Rear_Stereo_4-Feb-2014/calib_cam_to_cam.txt";
    left_right = "right";
  }
  
  // Read the images data for one camera
  file_list_reader front_left_images(root_folder + directory, file_pattern);
  // Read the image timing data
  std::string timing_data_filename = root_folder + timing_file;
  VLOG(1) << "Loading image timing data from " << timing_data_filename;
  timing_data_parser instance_times(timing_data_filename);
  // check
  VLOG(1) << "Timing data has " << instance_times.timing_data.size() << " instances."; 
  if (instance_times.timing_data.size() != front_left_images.file_names.size()) { 
    LOG(ERROR) << "Instances in timing data does not match number of images " <<
        instance_times.timing_data.size() << ", " << front_left_images.file_names.size();
    return -1;
  } else {
    VLOG(1) << "Number of files and timestamps = " << instance_times.timing_data.size();
  }
  
  // Load Calibration for camera
  std::string stereo_config_filename = root_folder + calib_file;
  cvlibs_config front_stereo_rect_config ( stereo_config_filename );
  cout << endl;

  cv::Mat left_image = cv::imread(front_left_images.file_names[0]);
  cv::Mat rectified_left_image(left_image.size(), left_image.type());
  const int image_width =  left_image.size().width;
  const int image_height = left_image.size().height;
  cv::Size image_size(image_width, image_height);

  // Image display window
  std::string raw_window = "Raw";
  std::string undistorted_window = "Undistorted";
  //cv::namedWindow( raw_window, cv::WINDOW_AUTOSIZE ); // Create a window for display
  cv::namedWindow( undistorted_window, cv::WINDOW_AUTOSIZE ); // Create a window for display
  //cv::moveWindow( raw_window, 0, 0); // move window to a new positon
  cv::moveWindow( undistorted_window, left_image.size().width, 0); // move window to a new positon
  cv::Scalar CV_RED(0,0,255);
  cv::Scalar CV_GREEN(0,255,0);
  cv::Scalar CV_BLUE(255,0,0);
  cv::Scalar CV_BLACK(0,0,0);
  cv::Scalar CV_WHITE(255,255,255);

  // Performance tracker
  anantak::PerformanceTracker performance_tracker_; /**< Performance tracker with timer objects */
  performance_tracker_.AddTimer("ImageLoad", 100);
  performance_tracker_.AddTimer("ImageRectify", 100);
  performance_tracker_.AddTimer("ConvertToGray", 100);
  performance_tracker_.AddTimer("CopyToBuffer", 100);
  performance_tracker_.AddTimer("MatchFeatures", 100);
  performance_tracker_.AddTimer("BucketFeatures", 100);
  performance_tracker_.AddTimer("CreateTracks", 100);
  performance_tracker_.AddTimer("TrackFeatures", 100);
  performance_tracker_.AddTimer("CreateMessage", 100);
  
  // Features tracker
  cv::Mat rectified_grayscale_left(image_size, CV_8UC1);    // tracker uses gray scale
  Matcher::parameters match_params;
  VisualOdometry::bucketing bucket_params;
  //bucket_params.max_features = 3;
  Matcher* matcher = new Matcher(match_params);
  int32_t image_dims[] = {image_width, image_height, image_width};
  uint8_t* left_img_data  = (uint8_t*)malloc(image_width*image_height*sizeof(uint8_t));
  FeatureTracker feature_tracker(1500, 20);
  
  // Message to be stored
  std::string message_type = "MonoSparsePoints";
  std::string pb_msgs_filename = "src/test/cam"+std::to_string(camera_num)+"_sparse_points.pb.data";
  // Create a message writer
  anantak::MessageFileWriter pb_file_writer;
  if (!pb_file_writer.Open(pb_msgs_filename)) {
    LOG(ERROR) << "File " << pb_msgs_filename << " could not be opened.";
    return -1;
  }
  pb_file_writer.StartTrackingPerformance();
  
  // Calibration specs file
  std::string calib_message_type = "MonoCalibNoDistort";
  std::string calib_msg_filename = "src/test/cam"+std::to_string(camera_num)+"_mono_calib_no_distort.pb.data";
  // Create a message writer
  anantak::MessageFileWriter calib_file_writer;
  if (!calib_file_writer.Open(calib_msg_filename)) {
    LOG(ERROR) << "File " << calib_msg_filename << " could not be opened.";
    return -1;
  }
  vector<double> calib_params;
  if (left_right == "left") {
    calib_params = front_stereo_rect_config.left_cam_pinhole_calib_no_distortion();
  } else {
    calib_params = front_stereo_rect_config.right_cam_pinhole_calib_no_distortion();
  }
  // Create a calib message
  anantak::SensorMsg calib_msg;
  anantak::HeaderMsg* calib_hdr_msg = calib_msg.mutable_header();
  calib_hdr_msg->set_timestamp(int64_t(instance_times.timing_data[0].msg_sent_time));
  calib_hdr_msg->set_type(calib_message_type);
  calib_hdr_msg->set_recieve_timestamp(int64_t(instance_times.timing_data[0].msg_sent_time));
  calib_hdr_msg->set_send_timestamp(int64_t(instance_times.timing_data[0].msg_sent_time));
  anantak::MonocularPinholeCalibrationNoDistortionMsg* calib_params_msg = calib_msg.mutable_mono_calib_no_distort_msg();
  calib_params_msg->set_camera_num(int32_t(camera_num));
  calib_params_msg->set_focal_length(calib_params[0]);
  calib_params_msg->set_cx(calib_params[1]);
  calib_params_msg->set_cy(calib_params[2]);
  // Write message
  if (!calib_file_writer.WriteMessage(calib_msg)) {
    LOG(ERROR) << "Calibration message not written";
  }
  // Close file writer
  calib_file_writer.Close();
  VLOG(1) << "Closed file " << calib_msg_filename;
  
  
  // AprilTag initialization
  // Create tagFamily
  std::vector<cv::Ptr<april::tag::TagFamily>> gTagFamilies;
  cv::Ptr<april::tag::TagDetector> gDetector;
  std::string tagid(TAG_FAMILY); //default Tag36h11
  april::tag::TagFamilyFactory::create(tagid, gTagFamilies);
  if (gTagFamilies.size() <= 0) {
    LOG(ERROR) << "Create TagFamily failed. Exiting.";
    return -1;
  } else {
    LOG(INFO) << "TagFamily created."; 
  }
  // Create tag detector
  gDetector = new april::tag::TagDetector(gTagFamilies);
  if (gDetector.empty()) {
    LOG(ERROR) << "Create TagDetector failed. Exiting.";
    return -1;
  } else {
    LOG(INFO) << "TagDetector created.";
  }
  performance_tracker_.AddTimer("DetectTags", 100);
  std::string april_message_type = "AprilTags";
  std::string april_pb_msgs_filename = "src/test/cam"+std::to_string(camera_num)+"_apriltags.pb.data";
  // Create a message writer
  anantak::MessageFileWriter april_pb_file_writer;
  if (!april_pb_file_writer.Open(april_pb_msgs_filename)) {
    LOG(ERROR) << "File " << april_pb_msgs_filename << " could not be opened.";
    return -1;
  }
  april_pb_file_writer.StartTrackingPerformance();
  
  // Do the work
  VLOG(1) << "Starting to rectify mono images.";
  for (int i=0; i<front_left_images.file_names.size(); i++) {
  //for (int i=0; i<1000; i++) {
    // Load image
    performance_tracker_("ImageLoad").StartTimer();
    left_image  = cv::imread(front_left_images.file_names[i]);
    performance_tracker_("ImageLoad").StopTimer();
    performance_tracker_("TrackFeatures").StartTimer();
    // Undistort
    performance_tracker_("ImageRectify").StartTimer();
    if (left_right == "left") {
      front_stereo_rect_config.undistort_left_mono(left_image, rectified_left_image);
    } else {
      front_stereo_rect_config.undistort_right_mono(left_image, rectified_left_image);      
    }
    performance_tracker_("ImageRectify").StopTimer();
    // Convert to gray scale
    performance_tracker_("ConvertToGray").StartTimer();
    cv::cvtColor(rectified_left_image, rectified_grayscale_left, CV_BGR2GRAY);
    performance_tracker_("ConvertToGray").StopTimer();
    // Copy data to image buffer
    performance_tracker_("CopyToBuffer").StartTimer();
    int32_t k = 0;
    for (int32_t v = 0; v < image_height; v++) {
      uchar* left_image_row_p  = rectified_grayscale_left.ptr(v);
      for (int32_t u = 0; u < image_width; u++) {
        left_img_data[k]  = (uint8_t) left_image_row_p[u];
        k++;
      }
    }
    performance_tracker_("CopyToBuffer").StopTimer();
    // Match sparse features
    performance_tracker_("MatchFeatures").StartTimer();
    matcher->pushBack(left_img_data, image_dims, false);
    matcher->matchFeatures(0);
    performance_tracker_("MatchFeatures").StopTimer();
    performance_tracker_("BucketFeatures").StartTimer();
    matcher->bucketFeatures(bucket_params.max_features, bucket_params.bucket_width,
                            bucket_params.bucket_height);                          
    performance_tracker_("BucketFeatures").StopTimer();
    // Get features and plot them on the rectified image
    const std::vector<Matcher::p_match>& p_matched = matcher->getMatchesByRef();
    // Create tracks
    performance_tracker_("CreateTracks").StartTimer();
    feature_tracker.UpdateTracks(p_matched);
    performance_tracker_("CreateTracks").StopTimer();
    performance_tracker_("TrackFeatures").StopTimer();
    // report
    cout << "   # " << i << " " << p_matched.size()
        << " new = " << feature_tracker.num_new_tracks()
        << " continued = " << feature_tracker.num_continued_tracks()
        << " completed = " << feature_tracker.num_completed_tracks()
        << " maxlen = " << feature_tracker.max_curr_length()
        << " " << feature_tracker.max_seen_length()
        << "\r" << flush;
    // Draw the points on the image
    for (int j=0; j<p_matched.size(); j++) {
      cv::Point2d curr(p_matched[j].u1c, p_matched[j].v1c);
      cv::Point2d prev(p_matched[j].u1p, p_matched[j].v1p);
      cv::line(rectified_left_image, curr, prev, CV_RED);
      cv::circle(rectified_left_image, curr, 1, CV_RED, 2);
      cv::circle(rectified_left_image, prev, 1, CV_GREEN, 2);
    }
    
    // Create sparse points message
    performance_tracker_("CreateMessage").StartTimer();
    anantak::SensorMsg msg;
    anantak::HeaderMsg* hdr_msg = msg.mutable_header();
    hdr_msg->set_timestamp(int64_t(instance_times.timing_data[i].msg_sent_time));
    hdr_msg->set_type(message_type);
    hdr_msg->set_recieve_timestamp(int64_t(instance_times.timing_data[i].msg_sent_time));
    hdr_msg->set_send_timestamp(int64_t(instance_times.timing_data[i].msg_sent_time));
    anantak::MonocularSparsePointsMsg* msp_msg = msg.mutable_mono_sparse_points_msg();
    msp_msg->set_camera_num(int32_t(camera_num));
    for (int j=0; j<p_matched.size(); j++) {
      msp_msg->mutable_u_curr()->Add(float(p_matched[j].u1c));
      msp_msg->mutable_v_curr()->Add(float(p_matched[j].v1c));
      msp_msg->mutable_i_curr()->Add(int32_t(p_matched[j].i1c));
      msp_msg->mutable_u_prev()->Add(float(p_matched[j].u1p));
      msp_msg->mutable_v_prev()->Add(float(p_matched[j].v1p));
      msp_msg->mutable_i_prev()->Add(int32_t(p_matched[j].i1p));
    }
    performance_tracker_("CreateMessage").StopTimer();    
    
    // Write sparse points message to disk
    if (!pb_file_writer.WriteMessage(msg)) {
      LOG(ERROR) << "Message not written";
    }
    
    //Detect tags
    if (true) {
      
      std::vector<april::tag::TagDetection> detections;
      performance_tracker_("DetectTags").StartTimer();
      gDetector->process(rectified_grayscale_left, detections);
      performance_tracker_("DetectTags").StopTimer();
      int num_tags_detected = (int)detections.size();

      // Create April tags message
      performance_tracker_("CreateAprilTagMessage").StartTimer();
      anantak::SensorMsg msg2;
      anantak::HeaderMsg* hdr_msg2 = msg2.mutable_header();
      hdr_msg2->set_timestamp(int64_t(instance_times.timing_data[i].msg_sent_time));
      hdr_msg2->set_type(april_message_type);
      hdr_msg2->set_recieve_timestamp(int64_t(instance_times.timing_data[i].msg_sent_time));
      hdr_msg2->set_send_timestamp(int64_t(instance_times.timing_data[i].msg_sent_time));
      anantak::AprilTagMessage* april_msg = msg2.mutable_april_msg();
      april_msg->set_camera_num(int32_t(camera_num));
      performance_tracker_("CreateAprilTagMessage").StopTimer();    
      
      // Draw tags on the image
      int nValidDetections=0;
      for (int k=0; k<num_tags_detected; ++k) {
        april::tag::TagDetection &dd = detections[k];
        if (dd.hammingDistance > 0) continue;   // hammingThreshold = 0
        ++nValidDetections;
        
        int tag_id = dd.id;
        //VLOG(INFO) << "id="<<tag_id<<", hdist="<<dd.hammingDistance<<", rotation="<<dd.rotation;
        cv::putText(rectified_left_image, dd.toString(), cv::Point(dd.cxy[0],dd.cxy[1]),
                    CV_FONT_NORMAL, 0.4, CV_BLUE, 1);
        cv::Mat Homo = cv::Mat(3,3,CV_64FC1,dd.homography[0]);
        helper::drawHomography(rectified_left_image, Homo);
        
        float x,y;
        x=dd.p[0][0]; y=dd.p[0][1];
        cv::circle(rectified_left_image, cv::Point2d(x,y), 3, CV_GREEN, 2);
        x=dd.p[1][0]; y=dd.p[1][1];
        cv::circle(rectified_left_image, cv::Point2d(x,y), 3, CV_BLACK, 2);
        x=dd.p[2][0]; y=dd.p[2][1];
        cv::circle(rectified_left_image, cv::Point2d(x,y), 3, CV_BLUE, 2);
        x=dd.p[3][0]; y=dd.p[3][1];
        cv::circle(rectified_left_image, cv::Point2d(x,y), 3, CV_WHITE, 2);
        
        april_msg->add_tag_id(dd.toString());
        april_msg->add_u_1(float(dd.p[0][0]));
        april_msg->add_v_1(float(dd.p[0][1]));
        april_msg->add_u_2(float(dd.p[1][0]));
        april_msg->add_v_2(float(dd.p[1][1]));
        april_msg->add_u_3(float(dd.p[2][0]));
        april_msg->add_v_3(float(dd.p[2][1]));
        april_msg->add_u_4(float(dd.p[3][0]));
        april_msg->add_v_4(float(dd.p[3][1]));
        
      }
      
      // Write april tags message to disk
      if (!april_pb_file_writer.WriteMessage(msg2)) {
        LOG(ERROR) << "Message not written";
      }      
      
    } // detect tags?

    //cv::imshow(raw_window, left_image);
    cv::imshow(undistorted_window, rectified_left_image);
    //cout << "   # " << i << " " << p_matched.size() << "\r" << flush;
    cv::waitKey(1);

    // Report timings
    VLOG_EVERY_N(1,100) << "Avg image load time = " << performance_tracker_("ImageLoad").average_timer_time();
    VLOG_EVERY_N(1,100) << "Avg image rectification time = " << performance_tracker_("ImageRectify").average_timer_time();  
    VLOG_EVERY_N(1,100) << "Avg convert to gray time = " << performance_tracker_("ConvertToGray").average_timer_time();
    VLOG_EVERY_N(1,100) << "Avg copy to buffer time = " << performance_tracker_("CopyToBuffer").average_timer_time();  
    VLOG_EVERY_N(1,100) << "Avg feature matching time = " << performance_tracker_("MatchFeatures").average_timer_time();  
    VLOG_EVERY_N(1,100) << "Avg feature bucketing time = " << performance_tracker_("BucketFeatures").average_timer_time();  
    VLOG_EVERY_N(1,100) << "Avg track create time = " << performance_tracker_("CreateTracks").average_timer_time();  
    VLOG_EVERY_N(1,100) << "Avg feature tracking time = " << performance_tracker_("TrackFeatures").average_timer_time();  
    VLOG_EVERY_N(1,100) << "Avg message creation time = " << performance_tracker_("CreateMessage").average_timer_time();  
    VLOG_EVERY_N(1,100) << "Avg message write time = " << pb_file_writer.MessageWriteTime();
    VLOG_EVERY_N(1,100) << "Avg tag detection time = " << performance_tracker_("DetectTags").average_timer_time();
    VLOG_EVERY_N(1,100) << "";
    
  } // for each image

  // undistortion takes about 3ms!! This is a lot! So it is best done once.  
  
  // Cleanup
  delete matcher;
  pb_file_writer.Close();
  VLOG(1) << "Closed file " << pb_msgs_filename;
  april_pb_file_writer.Close();
  VLOG(1) << "Closed file " << april_pb_msgs_filename;
  
  return 0;
}
