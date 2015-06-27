/**
 *  Show and save images being transmitted from cameras
 *
 *  Shows how to write image processing algorithms. Connects to ZMQ queue,
 *  extracts one or many images, processes, displays them. Images can also
 *  be saved to disk. Uses opencv for image display.
 */

#pragma once

/** std includes */
#include <string>
#include <map>
#include <sys/time.h>
#include <cstdint>

/** ZMQ includes */
#include <zmq.hpp>

/** Anantak includes */
#include "ComponentCommander/component_status_keeper.h"
#include "Filter/performance_tracker.h"

/** Protocol buffers */
#include "configurations.pb.h"
#include "sensor_messages.pb.h"

// OpenCV includes
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

// Apriltag detection includes for cv2cg library
#include "OpenCVHelper.h"
#define TAG_DEBUG_PERFORMANCE 0
#define TAG_DEBUG_DRAW 0
#include "apriltag/apriltag.hpp"
#include "apriltag/TagFamilyFactory.hpp"
#include "config.hpp"

// Apriltag detection includes for AprilTags C library from April lab
#include "apriltag.h"
#include "tag16h5.h"
#include "tag36h11.h"
#include "common/image_u8.h"
#include "common/zarray.h"

namespace anantak {

class ShowSaveImages {
 public:
  /** Constructor - gets the setupfile name and the component name */
  ShowSaveImages(std::string programs_setup_filename, std::string component_name);
  
  /** Destructor - generally everything must destruct on its own */
  virtual ~ShowSaveImages();
  
  /** StartLooping - starts the filter. Needs filter to be initialized beforehand */
  bool StartLooping();
  bool StartProcessingImages();
  
  /** Accessors */
  inline bool is_initiated() {return is_initiated_;}    /**< To check if filter started OK */
  
 private:
  /** Component settings */
  std::string programs_setup_config_filename_;    /**< Name of the ProgramsSetup config file */
  std::string component_name_;                    /**< Name of the component */
  std::string config_filename_;                   /**< Name of the configuration file */
  std::string publisher_name_;                    /**< Name of publisher */
  std::string command_subscriber_name_;           /**< Name of the commander subscriber */
  std::string status_subscriber_name_;            /**< Name of the status subscriber */
  
  /** Component Typedefs */
  typedef std::unique_ptr<std::string> StringPtrType;
  typedef std::unique_ptr<anantak::ComponentStatusKeeper> StatusKeeperPtrType;
  typedef std::unique_ptr<zmq::context_t> PubSubTransportPtrType;
  typedef std::unique_ptr<zmq::socket_t> PubSubPtrType;
  typedef std::map<std::string, PubSubPtrType> PubSubMapType;
  typedef std::map<std::string, PubSubPtrType>::iterator PubSubMapIteratorType;
  typedef std::map<std::string, std::string> StringsMapType;
  typedef std::map<std::string, std::string>::iterator StringsMapIteratorType;
  typedef std::map<std::string, int> IntsMapType;
  typedef std::map<std::string, int>::iterator IntsMapIteratorType;
  enum SubscriberType {kCommand, kStatus, kData};
  typedef std::map<std::string, SubscriberType> SubscriberTypeMapType;
  typedef std::vector<std::string> StringVectorType;
  typedef std::vector<std::string>::iterator StringVectorIteratorType;

  StatusKeeperPtrType status_keeper_;             /**< StatusKeeper */
  PubSubTransportPtrType zmq_transport_;          /**< Zmq transport **/
  PubSubMapType subscriptions_map_;               /**< Zmq Subscribers map */
  PubSubMapType publishers_map_;                  /**< Zmq Publishers map */
  SubscriberTypeMapType subscriber_type_;         /**< Type of subscriber */
  StringsMapType subscriber_subject_;             /**< Subject of subscriber */
  IntsMapType subscriber_subject_length_;         /**< Subject length of subscriber */
  
  bool is_initiated_;                             /**< Indicates that filter is initiated */
  std::string exit_command_str_;                  /**< Commands string for exit component */
  float loop_frequency_;                          /**< Looping frequency of the main loop */
  bool exit_loop_;                                /**< Indicator to signal exit the main loop */
  float max_loop_frequency_;                      /**< Maximum achievable looping frequency */  
 
  // Operating functions
  bool Initiate();                                /**< Initiate the filter */
  bool InitiateComponent();                       /**< Initiate component objects of the filter */
  bool HandleCommand(StringPtrType cmd);          /**< Handles commands from commander */
  bool HandleStatusQuery(StringPtrType query);    /**< Handle status query */
  StringPtrType AssembleStatusString();           /**< Assemble Status String */
  
  /** Inlined utility to get wall time in microseconds */
  inline int64_t get_wall_time_microsec() {
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) (tv.tv_sec * 1000000 + tv.tv_usec);  
  }
  
  /****** Show Save Images ******/
  
  std::string images_subscriber_name_;            /**< Name of the images subscriber */
  bool InitiateShowSaveImages();
  std::unique_ptr<anantak::ImagesProcessorConfig> config_;
  int32_t num_cameras_;
  std::string start_command_str_;
  std::string resume_command_str_;
  std::string pause_command_str_;
  std::string log_command_str_;
  std::string show_command_str_;
  std::string led_command_str_;
  std::string tag_command_str_;
  
  // structure of the Ghumio image message
  static const int Ghumio_Images_Message_Num_Images         = 4;
  static const int Ghumio_Images_Message_Header_Size        = 44;
  static const int Ghumio_Images_Message_Image_Header_Size  = 38;
  static const int Ghumio_Images_Message_Image_Size         = 921600;
  static const int Ghumio_Images_Message_Image_Stride       = 1920;
  static const int Ghumio_Images_Message_Footer_Size        = 27;
  
  static const int camera_num_front_left  = 0;
  static const int camera_num_front_right = 1;
  static const int camera_num_rear_left   = 2;
  static const int camera_num_rear_right  = 3;
  
  struct Ghumio_Images_Message_t {
      char message_header[Ghumio_Images_Message_Header_Size];
      char image_header[Ghumio_Images_Message_Num_Images][Ghumio_Images_Message_Image_Header_Size];
      unsigned char image_data[Ghumio_Images_Message_Num_Images][Ghumio_Images_Message_Image_Size];
      char message_footer[Ghumio_Images_Message_Footer_Size];
  };

  // utility function to provide current system time
  inline long tic() {
    struct timeval t;
    gettimeofday(&t, NULL);
    return (t.tv_sec*1000000 + t.tv_usec);
  }
  
  std::vector<bool> process_camera_;
  int image_width;
  int image_height;
  cv::Size image_size;
  cv::Mat captured_image_front_left, captured_image_front_right;
  cv::Mat captured_image_rear_left, captured_image_rear_right;
  cv::Mat image_front_left_gray, image_front_right_gray;
  cv::Mat image_rear_left_gray,  image_rear_right_gray;
  
  long msg_arrival_delay;     // time it took for message to arrive
  long msg_compiling_delay;   // time it took to get the images from the message
  long msg_processing_delay;  // time it took to process the message
  long image_display_delay;     // time it took to display the images
  unsigned long processed_frames;      // number of frames processed
  unsigned long dropped_frames;        // number of frames dropped
  
  // LED detection code
  bool detect_leds_;
  bool DetectLeds(cv::Mat &color_image, const cv::Mat &gray_image);
  cv::Scalar CV_RED, CV_BLUE, CV_BLACK, CV_WHITE, CV_GREEN, CV_YELLOW;
  
  // April tag detection 
  bool detect_tags_;
  
  // Use cv2cg library
  bool use_cv2cg_;
  //static const int TAG_FAMILY = 0;    //4
  //static const int TAG_FAMILY_SIZE = 30;  //600
  static const int TAG_FAMILY = 4;    //4
  static const int TAG_FAMILY_SIZE = 600;  //600
  std::vector<cv::Ptr<april::tag::TagFamily>> gTagFamilies;
  cv::Ptr<april::tag::TagDetector> gDetector;
  bool DetectTags(cv::Mat &color_image, cv::Mat &gray_image,
      const int64_t& message_sent_time, const int32_t& cam_num);
  
  // April tag detection using AprilTag C library from Aprillab
  bool use_aprillab_;
  apriltag_family_t *tag_family_;
  apriltag_detector_t *tag_detector_;
  image_u8_t *image_u8_;
  bool CopyGrayCvMatToImageU8(cv::Mat &gray_image, image_u8_t *im);
  
  // Beacon detector
  bool run_beacon_detector_;
  zmq::socket_t* apriltag_publisher_;
  std::string apriltag_subject_;
  zmq::socket_t* blob_publisher_;
  std::string blob_subject_;
  static const std::string APRIL_MESSAGE_TYPE;
  bool BuildAndSendAprilTagMessage(const std::vector<april::tag::TagDetection>& detections,
      const int64_t& message_sent_time, const int32_t& cam_num);
  anantak::SensorMsg april_tag_message_;
  
  int log_level;
  bool show_images;
  bool SendSensorMessage(const anantak::SensorMsg& msg,
      const std::string& subject, zmq::socket_t* publisher);
};

} // namespace anantak
