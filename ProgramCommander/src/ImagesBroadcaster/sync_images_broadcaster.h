/**
 *  Sync Images Broadcaster
 *
 *  Synchronous images broadcaster for point gray cameras. Runs as a component.
 *  Connects to cameras at startup, extracts images and transmits them on ZMQ bus.
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

/** Point Gray libraries **/
#include "C/FlyCapture2_C.h"

namespace anantak {

class SyncImagesBroadcaster {
 public:
  /** Constructor - gets the setupfile name and the component name */
  SyncImagesBroadcaster(std::string programs_setup_filename, std::string component_name);
  
  /** Destructor - generally everything must destruct on its own */
  virtual ~SyncImagesBroadcaster();
  
  /** StartLooping - starts the filter. Needs filter to be initialized beforehand */
  bool StartLooping();
  bool GrabAndPublishImages();
  
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
  enum SubscriberType {kCommand, kStatus, kDataQueue};
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
  
  /****** Sync Images Broadcaster ******/
  std::unique_ptr<anantak::ImagesBroadcasterConfig> ib_config_;
  std::vector<fc2Context> pg_contexts_;
  std::vector<fc2PGRGuid> pg_guids_;
  bool InitiateSyncImagesBroadcaster();                       /**< Initiate component objects of the imager */
  bool PrintPGBuildInfo();
  int32_t num_cameras_attached_;
  bool InitiatePGCameras();
  bool PrintCameraInfo(fc2Context* context);
  bool SetTimeStamping(fc2Context* context, BOOL enableTimeStamp);
  long int last_frame_capture_time, frame_capture_time, frame_to_frame_interval,
      end_capture_time, intra_frame_interval, max_to_min_interval;
  long int num_out_of_sync_events_;
  std::string start_command_str_;
  std::string resume_command_str_;
  std::string pause_command_str_;
  std::string log_command_str_;
  
};

} // namespace anantak
