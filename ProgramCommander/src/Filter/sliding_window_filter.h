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
#include "Filter/observation.h"
//#include "Filter/model_factory.h"
//#include "Filter/estimates_tracker.h"

/** Protocol buffers */
#include "configurations.pb.h"
#include "sensor_messages.pb.h"

namespace anantak {

class SlidingWindowFilter {
 public:
  /** Constructor - gets the setupfile name and the component name */
  SlidingWindowFilter(std::string programs_setup_filename, std::string component_name);
  
  /** Destructor - generally everything must destruct on its own */
  virtual ~SlidingWindowFilter();
  
  /** StartLooping - starts the filter. Needs filter to be initialized beforehand */
  bool StartLooping();
  
  /** RequestNewData - ask all queues for data since last ts to provided one */
  bool RequestNewData(int64_t end_ts);
  
  /** WaitForData - loop till a data reply comes **/
  bool WaitForData(int64_t wait_till_ts);
  
  /** Publish the message */
  bool PublishMessage(const anantak::MessageType& sensor_msg, const std::string& subject);

  /** Accessors */
  inline bool is_initiated() const {return is_initiated_;}  /**< To check if filter started OK */
  inline float loop_frequency() const {return loop_frequency_;}
  inline float max_loop_frequency() const {return max_loop_frequency_;}
  inline bool exit_loop() const {return exit_loop_;}
  inline int64_t wall_time() const {return get_wall_time_microsec();}
  inline void set_max_loop_frequency(float freq) {max_loop_frequency_ = freq;}
  inline int64_t data_arrival_delay() const {return data_arrival_delay_;}
  inline const ObservationsVectorStoreMap& GetLatestObservationsMap() const {
    return observations_tracker_.element();
  }
  inline int32_t num_observations_received() const {return num_observations_received_;}
  inline int64_t max_timestamp_received() const {return max_timestamp_received_;}
  inline int64_t min_timestamp_received() const {return min_timestamp_received_;}
  inline const anantak::FilterConfig& filter_config() const {return *config_;}
  inline const std::string& command_message() const {return *command_str_;}
  inline bool got_command() const {return got_command_;}
  
  typedef std::map<std::string, anantak::FilterConfig::Model> ModelConfig;
  inline const ModelConfig& ModelsConfiguration() const {return models_config_;}
  bool GetModelConfiguration(const std::string& model_name, anantak::FilterConfig::Model* config);
  bool GetLatestDataFromDataQueue(const std::string& queue_name,
      const int64_t& timeout_interval, const std::string& sensor_name);
  
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
  StringPtrType AssembleStatusString();           /**< Assemble Status String */
  
  /** Inlined utility to get wall time in microseconds */
  inline int64_t get_wall_time_microsec() const {
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) (tv.tv_sec * 1000000 + tv.tv_usec);  
  }
  
  /****** Sliding Window Filter ******/
    
  /** Filtering process is as follows:
   *  Startup
   *    Create models - populates the models_ member
   *    Create empty model states - this allocates memory for the model states
   *    If starting, wait for models initiation period, this is the starting iteration
   *  Iterate
   *    Fetch data since end of last iteration
   *    Marginalize old states - skip if this is starting iteration
   *      This provides priors for model states in form of constraints
   *    Create observations from the data recieved, each observation is tagged with source sensor
   *    Create states for each model
   *    Create constraints for each model - these are among observation-state and state-state
   *    Filter outliers
   *    Build a non-linear problem with states and constraints
   *      Use first-available states for jacobian calculation
   *    Solve the problem
   *    Save the states - used for later jacobian calculation
   *      Models may also save states in outside data queues
   *    Check for convergence - reinitiate models that have diverged
   *    Communicate results - each step time, loop times, covariances etc.
   *    Go back to Iterate
   *  Shutdown
   *    Save model results on disk
   *    Destruct everything cleanly
   **/
  
  // Operating functions and variables
  std::unique_ptr<anantak::FilterConfig> config_;  /**< Config file parsed by the */
  bool InitiateFiltering();                   /**< Initiate component objects of the filter */
  anantak::PerformanceTracker performance_tracker_; /**< Performance tracker with timer objects */
  int64_t max_expected_iteration_interval_;   /**< If iteration interval exceeds this, problem */
  int64_t max_sliding_window_interval_;       /**< Maximum sliding window interval */
  int32_t iterations_per_window_interval_;    /**< Number of iterations in one sliding window */

  // Fetch observations from DataQueues
  StringVectorType data_queues_;    /**< Holds the names of all data queues */
  StringVectorType observation_data_queues_;    /**< Holds the names of observations data queues */
  int64_t data_fetch_begin_timestamp_, data_fetch_end_timestamp_;   /**< data fetch timestamps */
  int64_t data_arrival_delay_;  /**< time it took for data to come since request was made */
  anantak::DataRequestMsg data_interval_request_msg_;   /**< Holds the data requests to be sent */
  anantak::DataRequestMsg data_latest_request_msg_;   /**< Holds the data requests to be sent */
  anantak::CompositeMsg data_reply_msg_;        /**< Message with data, received from DataQueues */
  uint32_t data_interval_request_msg_size_;
  bool SendDataRequestsToDataQueues();
  bool ProcessDataRepliesFromDataQueues(PubSubMapIteratorType iter, StringPtrType data);
  bool SendLatestDataRequestToDataQueue(const std::string& queue_name,
      const std::string& sensor_name);
  
  // Iterations
  anantak::CircularQueue<int64_t> iterations_tracker_;  /**< circular queue for iteration_ids */
  
  // Observations
  float max_observation_frequency_;           /**< Max expected frequency of observations */
  anantak::ObservationTypeMap observation_types_;   /**< Keeps specifics of the observation types */
  anantak::ObservationsVectorStoreMapCirPtrQueue observations_tracker_;  /**< Owns observations */
  int32_t num_observations_received_;
  int64_t max_timestamp_received_, min_timestamp_received_;
  
  // Models
  typedef ModelConfig::iterator ModelConfigIterator;
  ModelConfig models_config_; /**< Models configuration */
  
  //anantak::ModelFactory model_factory_;
  //anantak::ModelPtrMap models_tracker_;   /**< Owns models */

  // States
  //anantak::StateCirPtrQueuePtrMap states_tracker_;   /**< Owns states */
  
  // Estimates
  //anantak::EstimatesPtrMapCirPtrQueue estimates_tracker_;  /**< Owns estimates */  
  
};

} // namespace anantak
