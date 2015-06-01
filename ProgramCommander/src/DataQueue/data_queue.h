/**
 *  Data Queue
 *
 *  Data queue keeps a sliding window of data to be used by filters. At startup it knows the sensors
 *  and data subscribers. It creates a MessageQueue for each sensor. Then is listens to sensor data
 *  and queries from data subscribers. When a sensor measurement comes in, it is stored in the
 *  corresponding MessageQueue. DataQueue knows the how to deserialize the message to extract
 *  message timestamps from it (MessageQueue does not.) When a data query comes, it has a start and
 *  stop time. DataQueue fetches data from each MessageQueue for the interval seeked and publishes
 *  the data out to the asking data subscriber.
 */

#pragma once

/** std includes */
#include <string>
#include <map>
#include <sys/time.h>

/** ZMQ includes */
#include <zmq.hpp>

/** Anantak includes */
#include "DataQueue/message_queue.h"
#include "ComponentCommander/component_status_keeper.h"

/** Protocol Buffers */
#include "sensor_messages.pb.h"

namespace anantak {
  
class DataQueue {
 public:
  /** Constructor - gets the setupfile name and the component name */
  DataQueue(std::string programs_setup_filename, std::string component_name);
  
  /** Destructor - usually all members are self destructing */
  virtual ~DataQueue();

  /**< Start the looping - hear and route messages */
  bool StartLooping();

  /** Start/stop tracking performance */
  bool StartTrackingPerformance(int32_t track_length = 100);
  bool StopTrackingPerformance();
  
 private:
  // Settings
  std::string programs_setup_config_filename_;    /**< Name of the ProgramsSetup config file */
  std::string component_name_;                    /**< Name of the component */
  std::string config_filename_;                   /**< Name of the configuration file */
  std::string publisher_name_;                    /**< Name of publisher */
  std::string command_subscriber_name_;           /**< Name of the commander subscriber */
  std::string status_subscriber_name_;            /**< Name of the status subscriber */
  
  // Typedefs
  typedef std::map<std::string, std::unique_ptr<zmq::socket_t>>::iterator PubSubMapIterator;

  // Data structures
  std::unique_ptr<zmq::context_t> zmq_transport_; /**< ZMQ transport **/
  std::map<std::string, std::unique_ptr<zmq::socket_t>> subscriptions_map_;  /**< subscribers map */
  std::map<std::string, std::unique_ptr<zmq::socket_t>> publishers_map_;     /**< publishers map */
  std::map<std::string, std::unique_ptr<anantak::MessageQueue>> message_queue_map_; /**< msgq map */
  std::unique_ptr<anantak::ComponentStatusKeeper> status_keeper_;  /** StatusKeeper for DataQueue */

  // Functions
  bool Initiate();      /**< Initiator called by the constructor - constructs above members*/
  /** Generate DataQueue status */
  std::unique_ptr<std::string> AssembleStatusString();
  /** Handles commands from commander */
  bool HandleCommand(std::unique_ptr<std::string> cmd);
  /** Handle status query */
  bool HandleStatusQuery(std::unique_ptr<std::string> status_query);
  /** Handle sensor data */
  bool HandleSensorMeasurement(PubSubMapIterator iter,
      std::unique_ptr<anantak::MessageQueue::QueueDataType> msgq_ptr);
  /** Handle data subscriber requests */
  bool HandleDataRequest(PubSubMapIterator iter, std::unique_ptr<std::string> data);
  
  // Operating variables
  std::string exit_command_str_;  /**< Commands string that represents exit */
  float loop_frequency_;          /**< Looping frequency of the main loop */
  bool exit_loop_;                /**< Indicator to signal exit the main loop of data queue */
  float max_loop_frequency_;      /**< Maximum achievable looping frequency */
  
  enum SubscriberType {
    kCommand, kStatus, kSensor, kDataSub
  };
  std::map<std::string, SubscriberType> subscriber_type_;  /**< Type of subscriber */
  std::map<std::string, std::string> subscriber_subject_;  /**< Subject of subscriber */
  std::map<std::string, int> subscriber_subject_length_;   /**< Subject length of subscriber */
  std::map<std::string, int64_t> n_sensor_msgs_received_;  /**< Count of number of msgs received */
  std::map<std::string, int64_t> n_datasub_msgs_received_; /**< Count of number of msgs received */
  
  anantak::SensorMsg sensor_msg_;               /**< Extracted sensor message */
  anantak::DataRequestMsg data_request_msg_;    /**< Extracted data request message */
  anantak::CompositeMsg data_reply_msg_;        /**< Composite msg with data, sent to data subs */
  uint32_t data_reply_msg_size_;                /**< Keeps the size of data reply message */
  
  // Performance tracking
  /** Struct to convey queue performance measures */
  struct QueuePerformanceType {
    float msg_add_rate;       /**< Rate of addition (/sec) of messages */
    float avg_add_delay;      /**< Average delay in adding messages (microsec/msg) */
    float msg_fetch_rate;     /**< Rate of fetch requests (/sec) */
    float avg_fetch_delay;    /**< Average delay in fetching messages (microsec/msg) */
    QueuePerformanceType():msg_add_rate(0), msg_fetch_rate(0), avg_add_delay(0),
        avg_fetch_delay(0) {}
  };
  bool InitiatePerformanceTracking(int32_t track_length); /**< Initiates performance tracking */
  bool track_performance_;                  /**< Indicator to start tracking performance */
  int32_t tracking_vector_lengths_;         /**< Keep this much history of add/fetch operations */
  std::vector<int64_t> data_add_times_;     /**< Circular queue of data add times */
  std::vector<int64_t> data_add_intervals_; /**< Circular queue of data add time intervals */
  int32_t data_add_times_index_;            /**< Current index of circular queue */
  std::vector<int64_t> data_fetch_times_;   /**< Circular queue of data fetch times */
  std::vector<int64_t> data_fetch_intervals_; /**< Circular queue of data fetch time intervals */
  std::vector<int32_t> data_fetch_n_msgs_;  /**< Circular queue of num of data fetch msgs */
  int32_t data_fetch_times_index_;          /**< Current index of circular queue */
  QueuePerformanceType queue_performance_;  /**< Measures of queue performance */
  bool CalculateQueuePerformance();         /**< Calculate data queue performance */

  /** Utility to get wall time in microseconds */
  inline int64_t get_wall_time_microsec() {
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) (tv.tv_sec * 1000000 + tv.tv_usec);  
  }
  
  std::string save_command_str_;  /**< Commands string that represents exit */
  std::map<std::string, std::string> message_queue_data_file_;  /**< Subject of subscriber */
  bool SaveSensorMessageQueues();
};
  
} // namespace anantak
