/**
 *  Data Queue
 *
 *  Implementation of Data Queue.
 */

/** std inclues */
#include <cstdio>

/** Anantak includes */
#include "Utilities/common_functions.h"
#include "DataQueue/data_queue.h"

/** Google logging and flags libraries */
#include <glog/logging.h>

/** Protocol Buffers */
#include "configurations.pb.h"
#include <google/protobuf/io/zero_copy_stream_impl_lite.h>    // for creating data reply msg
#include <google/protobuf/io/coded_stream.h>                  // for creating data reply msg

/** ZMQ includes */
#include <zmq.hpp>

namespace anantak {
  
/** Constructor */
DataQueue::DataQueue(std::string programs_setup_filename, std::string component_name) {
  programs_setup_config_filename_ = programs_setup_filename;
  component_name_ = component_name;
  Initiate();
  //StartLooping();  // we can let the object owning the DataQueue call StartLooping.
}

/** Destructor - all members are self destructing */
DataQueue::~DataQueue() {
  VLOG(1) << "DataQueue shutdown. All objects should be destructed automatically.";
  VLOG(1) << "This includes the ZMQ transport and all pub/subs.";
}

/** Initiator */
bool DataQueue::Initiate() {
  // Read the Programs Setup file
  std::string setup_file_path = anantak::GetProjectSourceDirectory() + "/" +
      programs_setup_config_filename_;
  VLOG(1) << "Using ProgramsSetup file = " << setup_file_path;
  std::unique_ptr<anantak::ProgramsSetup> programs_setup =
      anantak::ReadProtobufFile<anantak::ProgramsSetup>(setup_file_path);

  // Find the program settings for this component
  bool found_component_settings = false;
  int settings_counter = -1;
  while (!found_component_settings) {
    settings_counter++;
    const anantak::ProgramsSetup::ProgramSettings& program_settings =
      programs_setup->program_settings(settings_counter);
    const std::string& program_name = program_settings.name();
    found_component_settings = (program_name == component_name_);
  }
  if (!found_component_settings) {
    LOG(ERROR) << "Could not find settings for " << component_name_ << " in programs setup";
    return false;
  } else {
    VLOG(3) << "Found settings for " << component_name_;
  }
  const anantak::ProgramsSetup::ProgramSettings& program_settings =
    programs_setup->program_settings(settings_counter);
  config_filename_ = program_settings.config_file();  // here we are making a copy

  // Build a map of publishers and subscribers in settings
  std::map<std::string, int> program_setting_pubs_map, program_setting_subs_map;
  for (int i=0; i<program_settings.publication_size(); i++) {
    const anantak::ProgramsSetup::ProgramSettings::ConnectionSettings& publication_settings =
        program_settings.publication(i);
    program_setting_pubs_map[publication_settings.name()] = i;
  }
  for (int i=0; i<program_settings.subscription_size(); i++) {
    const anantak::ProgramsSetup::ProgramSettings::ConnectionSettings& subscription_settings =
        program_settings.subscription(i);
    program_setting_subs_map[subscription_settings.name()] = i;
  }

  // Read the config file
  std::string config_file_path = anantak::GetProjectSourceDirectory() + "/" + config_filename_;
  VLOG(1) << "Using config file = " << config_file_path;
  std::unique_ptr<anantak::DataQueueConfig> config =
      anantak::ReadProtobufFile<anantak::DataQueueConfig>(config_file_path);
  VLOG(1) << "Number of sensors: " << config->sensor_size();
  VLOG(1) << "Number of data subscribers: " << config->data_subscriber_size();
  if (!config->has_commander_name() || !config->has_status_name() || !config->has_publisher_name()) {
    LOG(ERROR) << "Config file does not have one or more of commander_name, status_name, publisher name";
    return false;
  }
  
  // Build StatusKeeper to publish component status
  { /* wish there was some more beautiful way of doing this. 'Right' way is using initializer
     * list of the constructor, but I just dont like 'cramming' code in initializer list, may be
     * need to get over it. Till then, allocating a temp pointer, then moving to member */
    std::unique_ptr<anantak::ComponentStatusKeeper>
        temp_ptr(new anantak::ComponentStatusKeeper(component_name_));
    status_keeper_ = std::move(temp_ptr);   // class member pointer now owns the keeper
    // Setup a callback for the StatusKeeper to generate a DataQueue status string.
    status_keeper_->set_component_status_function( [&](){return AssembleStatusString();} );
    if (!status_keeper_) {
      LOG(ERROR) << "Could not create StatusKeeper.";
      return false;
    }
    VLOG(1) << "Built a status keeper";
  }
  
  // Build ZMQ transport
  { /* same as above */
    std::unique_ptr<zmq::context_t> temp_ptr(new zmq::context_t(1));
    zmq_transport_ = std::move(temp_ptr);     // class member ptr now owns this
    VLOG(1) << "Built ZMQ transport";
  }
  
  // Create the publisher  
  publisher_name_ = config->publisher_name();             // copying to class member
  VLOG(1) << "Publisher name = " << publisher_name_;
  if (program_setting_pubs_map.find(publisher_name_) == program_setting_pubs_map.end()) {
    LOG(ERROR) << "Could not find publisher settings in ProgramsSettings.";
    return false;
  } else {
    const anantak::ProgramsSetup::ProgramSettings::ConnectionSettings& publication_settings =
        program_settings.publication(program_setting_pubs_map[publisher_name_]);
    std::unique_ptr<zmq::socket_t> ptr(new zmq::socket_t(*zmq_transport_, ZMQ_PUB));
    ptr->bind(publication_settings.endpoint().c_str());
    publishers_map_[publisher_name_] = std::move(ptr);
    VLOG(1) << "Bound: " << publication_settings.name() << " " << publication_settings.endpoint();
  }
  
  // Build subscribers 
  command_subscriber_name_ = config->commander_name();    // copying to class member
  status_subscriber_name_ = config->status_name();        // copying to class member
  VLOG(1) << "Command subscriber name = " << command_subscriber_name_;
  VLOG(1) << "Status subscriber name = " << status_subscriber_name_;
  /* For each subscriber in config check if the program settings has specs for it. If so, create
   * the subscriber. Else write out an error and do not create a susbcriber */  
  std::vector<std::string> subscribers_to_be_created;
  subscribers_to_be_created.push_back(command_subscriber_name_);
  subscriber_type_[command_subscriber_name_] = kCommand;
  subscribers_to_be_created.push_back(status_subscriber_name_);
  subscriber_type_[status_subscriber_name_] = kStatus;
  for (int i=0; i<config->sensor_size(); i++) {
    const anantak::DataQueueConfig::Sensor& sensor = config->sensor(i);
    subscribers_to_be_created.push_back(sensor.name());
    n_sensor_msgs_received_[sensor.name()] = 0;
    subscriber_type_[sensor.name()] = kSensor;
  }
  for (int i=0; i<config->data_subscriber_size(); i++) {
    const anantak::DataQueueConfig::DataSubscriber& data_subscriber = config->data_subscriber(i);
    subscribers_to_be_created.push_back(data_subscriber.name());
    n_datasub_msgs_received_[data_subscriber.name()] = 0;
    subscriber_type_[data_subscriber.name()] = kDataSub;
  }
  // Make sure all subscribers have settings in setup file
  for (int i=0; i<subscribers_to_be_created.size(); i++) {
    if (program_setting_subs_map.find(subscribers_to_be_created[i]) == program_setting_subs_map.end()) {
      LOG(ERROR) << "Could not find subscriber settings for " << subscribers_to_be_created[i];
      return false;
    }
  }
  // Create subscribers 
  for (int i=0; i<subscribers_to_be_created.size(); i++) {
    std::string sub_name = subscribers_to_be_created[i];
    const anantak::ProgramsSetup::ProgramSettings::ConnectionSettings& subscriber_settings =
        program_settings.subscription(program_setting_subs_map[sub_name]);
    std::unique_ptr<zmq::socket_t> ptr(new zmq::socket_t(*zmq_transport_, ZMQ_SUB));
    ptr->connect(subscriber_settings.endpoint().c_str());
    ptr->setsockopt(ZMQ_SUBSCRIBE, subscriber_settings.subject().c_str(),
        subscriber_settings.subject().length());
    subscriptions_map_[sub_name] = std::move(ptr);
    subscriber_subject_[sub_name] = subscriber_settings.subject();
    subscriber_subject_length_[sub_name] = subscriber_settings.subject().size();
    VLOG(1) << "Connected: " << subscriber_settings.name() << " " << subscriber_settings.endpoint()
        << " \"" << subscriber_settings.subject() << "\"";
  }
  
  // Build a MessageQueue for each sensor
  for (int i=0; i<config->sensor_size(); i++) {
    const anantak::DataQueueConfig::Sensor& sensor = config->sensor(i);
    std::string queue_name = sensor.name();
    int32_t queue_size = sensor.queue_length();
    std::unique_ptr<anantak::MessageQueue> ptr(new anantak::MessageQueue(queue_name, queue_size));
    message_queue_map_[queue_name] = std::move(ptr);    // ptr is gone now
    if (sensor.has_track() && sensor.has_track_length()) {
      if (sensor.track())
        message_queue_map_[queue_name]->StartTrackingPerformance(sensor.track_length());
    }
  }
  VLOG(1) << "Built a MessageQueue for each Sensor";
  
  /** Set operating variables */
  if (config->has_cycle_frequency()) loop_frequency_ = config->cycle_frequency();
  else loop_frequency_ = 100.0;  // Hz
  exit_loop_ = false;
  exit_command_str_ = "COMMAND " + component_name_ + " exit";
  max_loop_frequency_ = 0.0f;
  
  /** Performance tracking */
  track_performance_ = false;
  InitiatePerformanceTracking(100);      // Setup performance tracking  
  
  return true;
}

bool DataQueue::StartTrackingPerformance(int32_t track_length) {
  track_performance_ = true;
  InitiatePerformanceTracking(track_length);
  VLOG(1) << "Switching Performance Tracking ON with tracking_length = " << track_length;
  return true;
}

bool DataQueue::StopTrackingPerformance() {
  track_performance_ = false;
  VLOG(1) << "Switching Performance Tracking OFF";
  return true;
}

bool DataQueue::InitiatePerformanceTracking(int32_t track_length=100) {
  // Initiate performance tracking
  tracking_vector_lengths_ = track_length;
  data_add_times_.resize(tracking_vector_lengths_, 0);
  data_add_intervals_.resize(tracking_vector_lengths_, 0);
  data_add_times_index_ = 0;
  data_fetch_times_.resize(tracking_vector_lengths_, 0);
  data_fetch_intervals_.resize(tracking_vector_lengths_, 0);
  data_fetch_n_msgs_.resize(tracking_vector_lengths_, 0);
  data_fetch_times_index_ = 0;
  if (track_performance_) VLOG(1) << "Performance tracking is running";
  else VLOG(1) << "Performane tracking is NOT running";
  return true;  
}

/** Calculate Queue performance measures and return a struct */
bool DataQueue::CalculateQueuePerformance() {
  int32_t first_data_add_times_index = (data_add_times_index_+1)%tracking_vector_lengths_;
  int32_t first_data_fetch_times_index = (data_fetch_times_index_+1)%tracking_vector_lengths_;
  float float_tracking_vector_lengths = float(tracking_vector_lengths_);
  if (data_add_times_[first_data_add_times_index] != 0) {
    queue_performance_.msg_add_rate = float(1000000) * float_tracking_vector_lengths / float(
        data_add_times_[data_add_times_index_] - data_add_times_[first_data_add_times_index]);
    queue_performance_.avg_add_delay = float(data_add_intervals_[data_add_times_index_] -
        data_add_intervals_[first_data_add_times_index]) / float_tracking_vector_lengths;
  }
  if (data_fetch_times_[first_data_fetch_times_index] != 0 &&
      data_fetch_n_msgs_[data_fetch_times_index_] !=0) {
    queue_performance_.msg_fetch_rate = float(1000000) * float_tracking_vector_lengths / float(
        data_fetch_times_[data_fetch_times_index_] - data_fetch_times_[first_data_fetch_times_index]);
    queue_performance_.avg_fetch_delay = float(data_fetch_intervals_[data_fetch_times_index_] -
        data_fetch_intervals_[first_data_fetch_times_index]) / float(
        data_fetch_n_msgs_[data_fetch_times_index_] - data_fetch_n_msgs_[first_data_fetch_times_index]);
  }
  return true;
}

/** Assemble a status string - this is provided as callback to the status keeper */
std::unique_ptr<std::string> DataQueue::AssembleStatusString() {
  std::unique_ptr<std::string> status_str(new std::string(""));
  // Number of messages from each sensor
  for (std::map<std::string, int64_t>::iterator i_map = n_sensor_msgs_received_.begin();
      i_map != n_sensor_msgs_received_.end(); i_map++) {    
    *status_str += subscriber_subject_[i_map->first]+":"+std::to_string(i_map->second)+" ";
  }
  // Performance for each sensor
  for (std::map<std::string, std::unique_ptr<anantak::MessageQueue>>::iterator i_map =
      message_queue_map_.begin(); i_map != message_queue_map_.end(); i_map++) {
    std::unique_ptr<anantak::MessageQueue::QueuePerformanceType> q_perf;
    q_perf = i_map->second->GetQueuePerformance();
    char buffer[100];
    snprintf(buffer, 100, "%s:%.0f,%.0f,%.0f,%.0f ", subscriber_subject_[i_map->first].c_str(),
        q_perf->msg_add_rate, q_perf->avg_add_delay, q_perf->msg_fetch_rate,
        q_perf->avg_fetch_delay);
    *status_str += buffer;
  }
  // Number of messages from each data subscriber
  for (std::map<std::string, int64_t>::iterator i_map = n_datasub_msgs_received_.begin();
      i_map != n_datasub_msgs_received_.end(); i_map++) {    
    *status_str += subscriber_subject_[i_map->first]+":"+std::to_string(i_map->second)+" ";
  }
  // For the aggregate data queue
  if (track_performance_) {
    CalculateQueuePerformance();
    char buffer[100];
    snprintf(buffer, 100, "DQ :%.0f,%.0f,%.0f,%.0f ", queue_performance_.msg_add_rate,
        queue_performance_.avg_add_delay, queue_performance_.msg_fetch_rate,
        queue_performance_.avg_fetch_delay);
    *status_str += buffer;
  }
  // Looping frequency
  {
    char buffer[100];
    if (max_loop_frequency_>1000.0f) snprintf(buffer, 100, "%.0f(>1k)", loop_frequency_);
    else snprintf(buffer, 100, "%.0f(%.0f)", loop_frequency_, max_loop_frequency_);
    *status_str += buffer;
  }
  return status_str;
}

/** Handle commands coming from the commander subscriber */
bool DataQueue::HandleCommand(std::unique_ptr<std::string> cmd) {
  // exit command
  if (*cmd == exit_command_str_) exit_loop_ = true;
  return true;
}

/** Handle status queries coming from ProgramCommander */
bool DataQueue::HandleStatusQuery(std::unique_ptr<std::string> status_query) {
  std::unique_ptr<std::string> status_query_reply_str =
      status_keeper_->GenerateStatusQueryReplyMessage(std::move(status_query));
  VLOG_EVERY_N(1,100) << "Status query reply: " << *status_query_reply_str;
  zmq::message_t status_query_reply_msg(status_query_reply_str->size());
  // Copy string buffer to message buffer - this can potentially be avoided using zero-copy
  memcpy(status_query_reply_msg.data(), status_query_reply_str->data(),
         status_query_reply_str->size());                                 // copy
  bool sent_ok =
      publishers_map_[publisher_name_]->send(status_query_reply_msg, ZMQ_DONTWAIT);
  if (!sent_ok) {
    LOG(ERROR) << "Status Query Reply message was not sent";
  }
  return true;
}

/** Handle sensor measurements. From each message, extract the message timestamp. 
 *  Record the time the message was recieved. Create a MessageQueue::QueueDataType object.
 *  Add this data to sensor's MessageQueue.
 */
bool DataQueue::HandleSensorMeasurement(PubSubMapIterator iter,
    std::unique_ptr<anantak::MessageQueue::QueueDataType> msgq_ptr) {
  // Track performance if asked
  int64_t data_add_start_time;
  if (track_performance_) {data_add_start_time = get_wall_time_microsec();} 
  // Record in a counter
  n_sensor_msgs_received_[iter->first]++;
  /** We do not need the msg timestamp in the data queue. So we can skip extracting it.
  // Attempt to parse the message to get msg timestamp
  if (!sensor_msg_.ParseFromString(msgq_ptr->message_str)) {
    LOG(ERROR) << "Message could not be parsed";
    return false;
  };
  // Get a reference to the header
  const anantak::HeaderMsg& hdr = sensor_msg_.header();
  // Modify the message time in he msgq_ptr
  msgq_ptr->message_time = hdr.timestamp();
  **/
  if (!message_queue_map_[iter->first]->AddMessage(std::move(msgq_ptr))) {
    LOG(ERROR) << "Could not add message to the queue";
  }
  // Performance measurements
  if (track_performance_) {
    int64_t data_add_acc_time =
        get_wall_time_microsec() - data_add_start_time + data_add_intervals_[data_add_times_index_];
    data_add_times_index_++;
    data_add_times_index_ %= tracking_vector_lengths_;    // this is a circular queue
    data_add_intervals_[data_add_times_index_] = data_add_acc_time;
    data_add_times_[data_add_times_index_] = data_add_start_time;
  }
  return true;
}

/** Handle data subscriber queries. */
bool DataQueue::HandleDataRequest(PubSubMapIterator iter, std::unique_ptr<std::string> msg_ptr) {
  int64_t data_fetch_start_time = get_wall_time_microsec();
  //if (track_performance_) {data_fetch_start_time = get_wall_time_microsec();} 
  // Record reciept in a counter
  n_datasub_msgs_received_[iter->first]++;
  // Parse the data request message
  if (!data_request_msg_.ParseFromString(*msg_ptr)) {
    LOG(ERROR) << "Message could not be parsed";
    return false;
  };
  // Clear the message container - this will call lots of alloc/dealloc not sure how to avoid it
  data_reply_msg_.Clear();    // Lots of dealloc here I guess as this message has varying shape
  int n_msgs_added = 0;
  // Adding a timestamps of sending data
  data_reply_msg_.set_timestamp(data_fetch_start_time);
  // Deal with the message depending on the kind of request it is
  if (data_request_msg_.type() == anantak::DataRequestMsg::LATEST) {
    // Copy the latest messages from each message queue into the message container
    for (std::map<std::string, std::unique_ptr<anantak::MessageQueue>>::iterator i_map =
        message_queue_map_.begin(); i_map != message_queue_map_.end(); i_map++) {
      n_msgs_added += i_map->second->AddLatestMessageToCompositeMessage(&data_reply_msg_);
    }
  } else if (data_request_msg_.type() == anantak::DataRequestMsg::INTERVAL) {
    // Make sure that we have the begin and end timestamps for the interval
    if (data_request_msg_.has_begin_timestamp() && data_request_msg_.has_end_timestamp()) {
      // Copy the messages in interval from each message queue into the message container
      for (std::map<std::string, std::unique_ptr<anantak::MessageQueue>>::iterator i_map =
          message_queue_map_.begin(); i_map != message_queue_map_.end(); i_map++) {
        n_msgs_added += i_map->second->AddMessagesToCompositeMessage(&data_reply_msg_,
            data_request_msg_.begin_timestamp(), data_request_msg_.end_timestamp());
      }
    } // if has begin,end timestamps
    else {
      LOG(ERROR) << "Data request of INTERVAL types does not have begin/end timestamps.";
      return false;
    }
  }
  VLOG(3) << "Added " << n_msgs_added << " messages to composite message container";
  // Serialize the message and publish
  /** Here we copy the composite message one last time into the transport message container.
   *  Need to find a way to avoid doing this. Using protobuf's coded output stream, it may be
   *  possible to avoid doing this copy. May be each message can directly be copied into a buffer
   *  allocated inside a transport message container. But we need to know the size in advance.
   *  This is possible but will take more work. We can look at it next if we decide that this
   *  extra copying step is taking too much time. */
  std::string address = iter->first + " ";
  uint32_t address_size = address.size();
  uint32_t msg_size = data_reply_msg_.ByteSize();
  // Allocate a transport message of size = address size + message size
  data_reply_msg_size_ = address_size + msg_size;
  //VLOG(1) << "msg_size = " << msg_size << " address_size = " << address_size;
  zmq::message_t transport_msg(data_reply_msg_size_);
  // Copy address to the message
  memcpy(transport_msg.data(), address.data(), address.size());
  // Move to the end of the address
  void* transport_msg_ptr = transport_msg.data();
  transport_msg_ptr = static_cast<char*>(transport_msg_ptr) + address_size;
  // Copy data to the buffer using zero copy stream
  google::protobuf::io::ArrayOutputStream raw_out_stream(transport_msg_ptr, msg_size);
  // Creating a coded stream is supposedly quite fast
  google::protobuf::io::CodedOutputStream coded_out_stream(&raw_out_stream);
  uint8_t* buffer = coded_out_stream.GetDirectBufferForNBytesAndAdvance(msg_size);
  if (buffer != NULL) {
    // The message fits in one buffer, so use the faster direct-to-array serialization path.
    data_reply_msg_.SerializeWithCachedSizesToArray(buffer);    // copy
  } else {
    LOG(ERROR) << "Allocated zmq message does not have required length to fit message";
    return false;
  }
  // Send the message
  bool sent_ok = publishers_map_[publisher_name_]->send(transport_msg, ZMQ_DONTWAIT);
  if (!sent_ok) {
    LOG(ERROR) << "Data reply message was not sent due to a transport problem";
    return false;
  }
  // Performance measurements
  if (track_performance_) {
    int64_t data_fetch_acc_time = get_wall_time_microsec() - data_fetch_start_time +
        data_fetch_intervals_[data_fetch_times_index_];
    int32_t tot_data_fetch_msgs =
        int32_t(n_msgs_added) + data_fetch_n_msgs_[data_fetch_times_index_];
    data_fetch_times_index_++;
    data_fetch_times_index_ %= tracking_vector_lengths_;    // this is a circular queue
    data_fetch_intervals_[data_fetch_times_index_] = data_fetch_acc_time;
    data_fetch_times_[data_fetch_times_index_] = data_fetch_start_time;
    data_fetch_n_msgs_[data_fetch_times_index_] = tot_data_fetch_msgs;
  }
  // ArrayOutputStream will be destructed here
  // CodedOutputStream will be destructed here
  // zmq message will be destructed here
  VLOG(3) << "Sent data reply to " << address << " with size = " << data_reply_msg_size_;
  
  return true;
}

/** Main loop that listens to the subscribers and routes the messages
 *  Sensor messages - extract timestamp, add to sensor's message queue
 *  DataSubscriber messages - extract begin/end timestamps, collect data, publish back
 *  StatusQuery subscriber - send to StatusKeeper
 *  Commands - interpret commands and respond accordingly
 */
bool DataQueue::StartLooping() {
  
  anantak::Looper looper(loop_frequency_);
  
  while (!exit_loop_) {
    
    // Loop through each subscriber, read messages and pass them to status keeper objects
    for (PubSubMapIterator i_map = subscriptions_map_.begin(); i_map != subscriptions_map_.end();
         i_map++) {
      VLOG(3) << "Reading subscriber " << i_map->first;
      
      // Poll this subscriber without waiting
      zmq::message_t message;
      if (i_map->second->recv(&message, ZMQ_DONTWAIT)) {
        
        // Create a string by copying the incoming message data. We want to pass the string over
        //  to message handlers. When the string is passed to a handler, it is responsible for
        //  destructing it. It will be 'gone' from here. So we use unique_ptr to pass ownership.
        //  We also want to save on time it takes to copy the string from here to the keeper object
        //  as we are no longer copying the string bytes from here to keeper's method. Copying is
        //  only done once from the message_t buffer to a string object buffer. 
        size_t msg_size = message.size();
        const char* msg_data_ptr = static_cast<char*>(message.data());  // cast the void* to char*
        
        /** Handle commands */
        if (subscriber_type_[i_map->first] == kCommand) {
          std::unique_ptr<std::string> msg_str_ptr(             // copy the entire message
              new std::string(msg_data_ptr, msg_size));   // copy
          VLOG(1) << "Command: " << *msg_str_ptr;
          HandleCommand(std::move(msg_str_ptr));
        }
        
        /** Handle status queries */
        if (subscriber_type_[i_map->first] == kStatus) {
          std::unique_ptr<std::string> msg_str_ptr(             // copy the entire message
              new std::string(msg_data_ptr, msg_size));   // copy
          VLOG(3) << "Status query: " << *msg_str_ptr;
          HandleStatusQuery(std::move(msg_str_ptr));
        }
        
        /** Handle sensor measurements */
        if (subscriber_type_[i_map->first] == kSensor) {
          int address_length = subscriber_subject_length_[i_map->first];
          int64_t current_time = get_wall_time_microsec();
          std::unique_ptr<anantak::MessageQueue::QueueDataType> msgq_ptr(
              new anantak::MessageQueue::QueueDataType(          // copy all but the address
                current_time, current_time, msg_data_ptr+address_length, msg_size-address_length
              ));
          HandleSensorMeasurement(i_map, std::move(msgq_ptr));
        }
        
        /** Handle data subscriber query */
        if (subscriber_type_[i_map->first] == kDataSub) {
          int address_length = subscriber_subject_length_[i_map->first];
          std::unique_ptr<std::string> msg_str_ptr(             // copy all but the address
              new std::string(msg_data_ptr+address_length, msg_size-address_length));  // copy
          VLOG(4) << "DataSub message: " << *msg_str_ptr;
          HandleDataRequest(i_map, std::move(msg_str_ptr));
        }
        
        // Message_String unique pointer destructs here if it has not been moved
      } // recv message
      // Message is destructed here
    } // for     
    
    if (!exit_loop_) looper.Sleep();
    max_loop_frequency_ = looper.max_frequency();
  } // while
  
  return true;
} // StartLooping


} // namespace anantak
