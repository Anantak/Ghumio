/**
 *  Message Files Publisher
 *
 *  This assumes that messages from disk files can be read fairly quickly, it seems each message
 *  can be read within ~1us. Our sensors are at most publishing around 100Hz or 10000us. So we can
 *  directly read from the disk as we publish the data on the wire. There is no need to buffer
 *  data in a message queue before publishing it out.
 *
 *  This class is a simplification of DataLoader. Data Loader loads data from the disk and buffers
 *  it in the MessageQueues. Then the queues are asked for data to be published. The queues could
 *  be in a separate thread.
 *
 *  This class keeps a measure of 'delay' from real time, so if the data loading is too slow to be
 *  realtime, it will indicate that.
 */

#pragma once

/** std includes */
#include <string>
#include <map>
#include <sys/time.h>

/** ZMQ includes */
#include <zmq.hpp>

/** Anantak includes */
#include "DataQueue/message_files_publisher_status_keeper.h"
#include "DataQueue/message_file_reader.h"

namespace anantak {

class MessageFilesPublisher {
 public:
  /** Constructor - gets the setupfile name and the component name */
  MessageFilesPublisher(std::string programs_setup_filename, std::string component_name);
  
  /** Destructor - all objects are self-destructing */
  virtual ~MessageFilesPublisher();

  /** StartPublishing - call this to setup the object and start publishing */
  bool StartPublishing();
  
 private:
  // Settings
  std::string programs_setup_config_filename_;    /**< Name of the ProgramsSetup config file */
  std::string component_name_;                    /**< Name of the component */
  std::string config_filename_;                   /**< Name of the configuration file */
  std::string publisher_name_;                    /**< Name of publisher */
  std::string command_subscriber_name_;           /**< Name of the commander subscriber */
  std::string status_subscriber_name_;            /**< Name of the status subscriber */
  
  // Typedefs
  typedef std::unique_ptr<std::string> StringPtrType;
  typedef std::unique_ptr<anantak::ComponentStatusKeeper> StatusKeeperPtrType;
  typedef std::unique_ptr<zmq::context_t> PubSubTransportPtrType;
  typedef std::unique_ptr<zmq::socket_t> PubSubPtrType;
  typedef std::map<std::string, PubSubPtrType> PubSubMapType;
  typedef std::map<std::string, PubSubPtrType>::iterator PubSubMapIteratorType;
  typedef std::map<std::string, std::string> StringsMapType;
  typedef std::map<std::string, std::string>::iterator StringsMapIteratorType;
  typedef std::unique_ptr<std::vector<int64_t>> TimeVectorPtrType;
  typedef std::map<std::string, TimeVectorPtrType> TimeVectorMapType;
  typedef std::map<std::string, TimeVectorPtrType>::iterator TimeVectorMapIteratorType;
  typedef std::unique_ptr<anantak::MessageFileReader> FileReaderPtrType;
  typedef std::map<std::string, FileReaderPtrType> FileReaderMapType;
  typedef std::map<std::string, FileReaderPtrType>::iterator FileReaderMapIteratorType;
  typedef std::map<std::string, int32_t> MessageIndexMapType;
  typedef std::map<std::string, int32_t>::iterator MessageIndexMapIteratorType;
  

  // Operating variables
  StatusKeeperPtrType status_keeper_;             /**< StatusKeeper */
  PubSubTransportPtrType zmq_transport_;          /**< Zmq transport **/
  PubSubMapType subscriptions_map_;               /**< Zmq Subscribers map */
  PubSubMapType publishers_map_;                  /**< Zmq Publishers map */
  StringsMapType publish_subjects_map_;           /**< Map of publishing subjects */
  
  StringsMapType data_filenames_;                 /**< Map of data filenames */
  TimeVectorMapType data_timestamps_;             /**< Number of messages to load for each cycle */
  int64_t schedule_start_time_;                   /**< Start timestamp of the schedule */
  int64_t schedule_end_time_;                     /**< End timestamp of the schedule */
  FileReaderMapType data_file_readers_;           /**< Map of file readers for each sensor */
  MessageIndexMapType message_index_;             /**< Map of index of message to be sent next */
  std::string schedule_period_str_;               /**< Schedule period string representation */
  
  std::string exit_command_str_;                  /**< Commands string for exit component */
  std::string play_command_str_;                  /**< Commands string for start playing data */
  std::string stop_command_str_;                  /**< Commands string for stop playing data */
  std::string pause_command_str_;                 /**< Commands string for pause playing data */
  float loop_frequency_;                          /**< Looping frequency of the main loop */
  bool exit_loop_;                                /**< Indicator to signal exit the main loop */
  float max_loop_frequency_;                      /**< Maximum achievable looping frequency */
  
  enum StateType {                                /**< Data publisher state enumeration */
      kStart, kPlaying, kPaused, kEnded };
  StateType publisher_state_;                     /**< Data publisher state keeper */
  
  int64_t curr_play_time_;                        /**< Accummulated data playing time */
  int64_t last_real_time_, curr_real_time_;       /**< Wall time last and current in cycle */
  int64_t last_hist_time_, curr_hist_time_;       /**< Historical time last and current in cycle */
  
  // Operating functions
  
  /** Load Configuration
   *    Names of the sensors and their data files
   *    Looping frequency - this determines the loading block size
   *    Starting time of message publishing (optional)
   *  Create subscribers, publishers, status keeper etc.
   */
  bool Initiate();
  
  /** Create Loading Schedule
   *  This uses the cycle frequency to calculate a schedule of data loading. In each iteration,
   *  messages for the iteration are loaded from the disk and published.
   */
  bool CreateLoadingSchedule();
  
  /** Create File Readers
   *  This opens all data files and creates a map of MessageFileReaders. Each reader will be used
   *  to read its file. Key in the map is the sensor name.
   */
  bool CreateFileReaders();
  /** Reset File Readers
   *  This reopens the files and sets the message indexes to 0
   */
  bool ResetFileReaders();

  /** Start Looping
   *  Starts the main loop at the looping frequency. In each loop:
   *    Load data for each file, publish it out
   *    Reply to status queries if any
   *    Act on commands if any
   *    Check if at the end of the loop, walltime is more than realtime.
   *      If so raise alarm and measure the delay.
   */
  bool StartLooping();

  /** Process Commands
   *    Begin, Stop: Begin/stop publishing data from the files.
   *    Suspend, Resume: Temporarily suspend/resume publishing data.
   *    Start, Exit: Start/Exit component
   */
  bool ProcessCommands(StringPtrType command);
  
  /** Assemble Status String
   *    Status string includes max delay, time of data left, number of sensors
   */
  StringPtrType AssembleStatusString();

  /** Handles commands from commander */
  bool HandleCommand(StringPtrType cmd);

  /** Handle status query */
  bool HandleStatusQuery(StringPtrType status_query);
  
  /** Utility to get wall time in microseconds */
  inline int64_t get_wall_time_microsec() {
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) (tv.tv_sec * 1000000 + tv.tv_usec);  
  }
};

} // namespace anantak
