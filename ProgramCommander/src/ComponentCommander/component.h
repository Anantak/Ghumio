/**
 *  Sliding Window Filter
 *
 *  SWF has multiple DataQueue connections. At a certain cycle rate, it fetches data from each
 *  data queue and provides it to the Models for: new states creation, constraints creation,
 *  marginalization etc. Once all states and constraints are setup, Filter sets up a non-linear
 *  problem that it solves. Once the solving is done, states are stored (first estimates) for
 *  Jacobian calculations.
 *  SWF also owns a StatusKeeper that is used to convey state e.g. Cycle rate, times taken for
 *  each step.
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
#include "Filter/circular_pointer_queue.h"
#include "Filter/circular_queue.h"
#include "Filter/performance_tracker.h"

/** Protocol buffers */
#include "configurations.pb.h"
#include "sensor_messages.pb.h"

namespace anantak {

class Component {
 public:
  typedef std::map<std::string, anantak::ComponentConfig::ObservationType> ObservationTypeMap;
  typedef std::vector<anantak::SensorMsg> ObservationsVector;
  
  /** Constructor - gets the setupfile name and the component name */
  Component(std::string programs_setup_filename, std::string component_name);
  
  /** Destructor - generally everything must destruct on its own */
  virtual ~Component();
  
  /** CheckForData - process any messages in subscribers **/
  bool ReadMessages();
  const ObservationsVector& Observations() const {return observations_;}
  
  /** Publish the message */
  bool PublishMessage(const anantak::MessageType& sensor_msg, const std::string& subject);

  /** StartLooping - starts the filter. Needs filter to be initialized beforehand */
  //bool StartLooping();
  
  /** RequestNewData - ask all queues for data since last ts to provided one */
  //bool RequestNewData(int64_t end_ts);
  
  /** WaitForData - loop till a data reply comes **/
  //bool WaitForData(int64_t wait_till_ts);
  
  /** Accessors */
  inline bool is_initiated() const {return is_initiated_;}  /**< To check if filter started OK */
  inline float loop_frequency() const {return loop_frequency_;}
  inline float max_loop_frequency() const {return max_loop_frequency_;}
  inline bool exit_loop() const {return exit_loop_;}
  inline int64_t wall_time() const {return get_wall_time_microsec();}
  inline void set_max_loop_frequency(float freq) {max_loop_frequency_ = freq;}
  inline int32_t num_observations_received() const {return num_observations_received_;}
  //inline int64_t max_timestamp_received() const {return max_timestamp_received_;}
  //inline int64_t min_timestamp_received() const {return min_timestamp_received_;}
  inline const anantak::ComponentConfig& component_config() const {return *config_;}
  inline const std::string& command_message() const {return *command_str_;}
  inline bool got_command() const {return got_command_;}
  
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
  enum SubscriberType {kCommand, kStatus, kObservation};
  typedef std::map<std::string, SubscriberType> SubscriberTypeMapType;
  typedef std::vector<std::string> StringVectorType;
  typedef std::vector<std::string>::iterator StringVectorIteratorType;

  std::unique_ptr<anantak::ComponentConfig> config_;  /**< Config file parsed by the component */
  StatusKeeperPtrType status_keeper_;             /**< StatusKeeper */
  PubSubTransportPtrType zmq_transport_;          /**< Zmq transport **/
  PubSubMapType subscriptions_map_;               /**< Zmq Subscribers map */
  PubSubMapType publishers_map_;                  /**< Zmq Publishers map */
  SubscriberTypeMapType subscriber_type_;         /**< Type of subscriber */
  StringsMapType subscriber_subject_;             /**< Subject of subscriber */
  IntsMapType subscriber_subject_length_;         /**< Subject length of subscriber */
  
  bool is_initiated_;                             /**< Indicates that filter is initiated */
  std::string exit_command_str_;                  /**< Commands string for exit component */
  bool got_command_;                              /**< true if a command was received */
  StringPtrType command_str_;                     /**< Pointer to the command string */
  float loop_frequency_;                          /**< Looping frequency of the main loop */
  bool exit_loop_;                                /**< Indicator to signal exit the main loop */
  float max_loop_frequency_;                      /**< Maximum achievable looping frequency */  
 
  // Operating functions
  bool Initiate();                                /**< Initiate the filter */
  bool InitiateComponent();                       /**< Initiate component objects of the filter */
  bool HandleCommand(StringPtrType cmd);          /**< Handles commands from commander */
  bool HandleStatusQuery(StringPtrType query);    /**< Handle status query */
  bool HandleObservation(PubSubMapIteratorType iter, StringPtrType data); /* Handle observation */
  StringPtrType AssembleStatusString();           /**< Assemble Status String */
  
  /** Inlined utility to get wall time in microseconds */
  inline int64_t get_wall_time_microsec() const {
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) (tv.tv_sec * 1000000 + tv.tv_usec);  
  }
  
  // Observations
  ObservationTypeMap observation_types_;      /**< Observation types */
  ObservationsVector observations_;
  anantak::SensorMsg sensor_msg_;
  bool ExtractHeaderMsg(const std::string& serial_str, anantak::HeaderMsg* header_msg); 
  int64_t max_expected_iteration_interval_;   /**< If iteration interval exceeds this, problem */
  float max_observation_frequency_;           /**< Max expected frequency of observations */
  int32_t num_observations_received_;
  //int64_t max_timestamp_received_, min_timestamp_received_;
  
  // Operating functions and variables
  anantak::PerformanceTracker performance_tracker_; /**< Performance tracker with timer objects */

  
};    // Component

} // namespace anantak
