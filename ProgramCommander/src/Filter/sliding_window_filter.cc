/**
 *  Sliding Window Filter
 *
 *  SWF implmenetation
 */

/** Header file include */
#include "Filter/sliding_window_filter.h"

/** Anantak includes */
#include "common_config.h"
#include "Utilities/common_functions.h"

/** Google logging and flags libraries */
#include <glog/logging.h>

/** Protocol Buffers */
#include <google/protobuf/io/zero_copy_stream_impl_lite.h>    // for creating data request msg
#include <google/protobuf/io/coded_stream.h>                  // for creating data request msg

namespace anantak {

/** Sliding Window Filter constructor */
SlidingWindowFilter::SlidingWindowFilter(std::string programs_setup_filename,
    std::string component_name) {
  programs_setup_config_filename_ = programs_setup_filename;
  component_name_ = component_name;  
  is_initiated_ = false;
  if (!Initiate()) LOG(ERROR) << "Could not initiate filter.";
}

/** Destructor - all members are self destructing */
SlidingWindowFilter::~SlidingWindowFilter() {
  VLOG(1) << "SlidingWindowFilter shutdown. All objects should be destructed automatically.";
  VLOG(1) << "This includes the ZMQ transport and all pub/subs.";    
}

/** Initiator - creates all starting objects */
bool SlidingWindowFilter::Initiate() {
  if (!InitiateComponent()) {
    LOG(ERROR) << "Could not initiate filter component objects";
    return false;
  }
  if (!InitiateFiltering()) {
    LOG(ERROR) << "Could not initiate filtering objects";
    return false;    
  }
  // Initiation succeeded
  is_initiated_ = true;
  return true;
}

/** Component Initiator - creates all starting component objects */
bool SlidingWindowFilter::InitiateComponent() {
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

  // Build a map of publishers and subscribers settings
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
  std::unique_ptr<anantak::FilterConfig> config =
      anantak::ReadProtobufFile<anantak::FilterConfig>(config_file_path);
  VLOG(1) << "Number of data queues: " << config->data_queue_size();
  if (!config->has_commander_name() || !config->has_status_name() || !config->has_publisher_name()) {
    LOG(ERROR) << "Config file does not have either of commander_name, status_name, publisher name";
    LOG(ERROR) << "All three are necessary for operation of the component";
    return false;
  }
  
  // Build StatusKeeper to publish component status
  { /* Wish there was some more beautiful way of initiating this. 'Right' way is using initializer
     * list of the constructor, but I just dont like 'cramming' code in initializer list, may be
     * need to get over it. Till then, allocating a temp pointer, then moving to member */
    StatusKeeperPtrType temp_ptr(new anantak::ComponentStatusKeeper(component_name_));
    status_keeper_ = std::move(temp_ptr);   // class member pointer now owns the keeper
    if (!status_keeper_) {
      LOG(ERROR) << "Could not create StatusKeeper.";
      return false;
    }
    // Setup a callback for the StatusKeeper to generate a DataQueue status string.
    status_keeper_->set_component_status_function( [&](){return AssembleStatusString();} );
    VLOG(1) << "Built a status keeper";
  }
  
  // Build ZMQ transport
  { /* same as above */
    PubSubTransportPtrType temp_ptr(new zmq::context_t(1));
    zmq_transport_ = std::move(temp_ptr);     // class member ptr now owns this
    VLOG(1) << "Built ZMQ transport";
  }
  
  /** Build Publishers */
  publisher_name_ = config->publisher_name();             // copying to class member
  VLOG(1) << "Publisher name = " << publisher_name_;
  std::vector<std::string> publishers_to_be_created;
  publishers_to_be_created.push_back(publisher_name_);
  // Find publisher settings
  for (int i=0; i<publishers_to_be_created.size(); i++) {
    if (program_setting_pubs_map.find(publishers_to_be_created[i]) == program_setting_pubs_map.end()) {
      LOG(ERROR) << "Could not find publisher settings for " << publishers_to_be_created[i];
      return false;
    }
  }
  // Create publishers
  try {
    for (int i=0; i<publishers_to_be_created.size(); i++) {
      std::string pub_name = publishers_to_be_created[i];
      const anantak::ProgramsSetup::ProgramSettings::ConnectionSettings& publisher_settings =
          program_settings.publication(program_setting_pubs_map[pub_name]);
      PubSubPtrType ptr(new zmq::socket_t(*zmq_transport_, ZMQ_PUB));
      ptr->bind(publisher_settings.endpoint().c_str());
      publishers_map_[pub_name] = std::move(ptr);
      VLOG(1) << "Bound: " << publisher_settings.name() << " " << publisher_settings.endpoint();
    }
  }
  catch (const zmq::error_t& e) {
    std::string err_str = e.what();
    LOG(ERROR) << "Error in creating publisher: " << err_str;
    return false;
  }
  
  /** Build Subscribers */
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
  for (int i=0; i<config->data_queue_size(); i++) {
    const anantak::FilterConfig::DataQueue& data_queue = config->data_queue(i);
    subscribers_to_be_created.push_back(data_queue.name());
    subscriber_type_[data_queue.name()] = kDataQueue;
    data_queues_.push_back(data_queue.name());
  }
  data_queues_.shrink_to_fit(); // release any extra space
  // Find subscriber settings
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
    PubSubPtrType ptr(new zmq::socket_t(*zmq_transport_, ZMQ_SUB));
    ptr->connect(subscriber_settings.endpoint().c_str());
    ptr->setsockopt(ZMQ_SUBSCRIBE, subscriber_settings.subject().c_str(),
        subscriber_settings.subject().length());
    subscriptions_map_[sub_name] = std::move(ptr);
    subscriber_subject_[sub_name] = subscriber_settings.subject();
    subscriber_subject_length_[sub_name] = subscriber_settings.subject().size();
    VLOG(1) << "Connected: " << subscriber_settings.name() << " " << subscriber_settings.endpoint()
        << " \"" << subscriber_settings.subject() << "\"";
  }
  
  // Looping variables
  if (config->has_cycle_frequency()) loop_frequency_ = config->cycle_frequency();
      else loop_frequency_ = 10.0;  // Hz
  exit_loop_ = false;
  exit_command_str_ = "COMMAND " + component_name_ + " exit";
  max_loop_frequency_ = 0.0f;
  
  // Map of observation types - create from config file
  if (config->observation_type_size() == 0) {
    LOG(ERROR) << "No observation types have been declared. Filter needs data to operate.";
    return false;
  }
  for (int i=0; i<config->observation_type_size(); i++) {
    const anantak::FilterConfig::ObservationType& obs_type = config->observation_type(i);
    observation_types_[obs_type.name()] = obs_type;   // copy operation
    VLOG(1) << "Added observation type = " << obs_type.name();
  }
  
  // Iteration intervals and cycle frequencies
  if (config->has_max_iteration_interval())
      max_expected_iteration_interval_ = config->max_iteration_interval();
  else
      max_expected_iteration_interval_ = 100000; // default set to 100ms or 10Hz
  
  if (config->has_max_sliding_window_interval())
      max_sliding_window_interval_ = config->max_sliding_window_interval();
  else
      max_sliding_window_interval_ = 5000000;  // default max sliding window interval set to 5sec
  
  if (config->has_max_observation_frequency())
      max_observation_frequency_ = config->max_observation_frequency();
  else
      max_observation_frequency_ = 500;       // deafult max frequency set to 500 Hz

  // Maximum size of the circular queue that keeps all observations
  iterations_per_window_interval_ = int32_t(
      max_sliding_window_interval_ / max_expected_iteration_interval_) + 1;
  LOG(INFO) << "Indicated number of iterations per window = " << iterations_per_window_interval_;
  
  // Models configuration
  for (int i=0; i<config->model_size(); i++) {
    models_config_[config->model(i).name()] = config->model(i); // copy
    LOG(INFO) << "Reading configuration for model " << config->model(i).name();
  }

  return true;
}

/** Filtering Initiator - creates all starting filtering objects */
bool SlidingWindowFilter::InitiateFiltering() {
  
  /** Iterations tracker - init with twice the length of sliding window */
  iterations_tracker_.Initiate(2*iterations_per_window_interval_);
  
  /** Allocate memory for observations
   *  Observations are kept in a observations_tracker_ object that is preallocated with memory.
   *  We prefer allocating more than needed memory rather than incur memory reallocation during
   *  filtering operation. For each observation type, the maximum number of messages expected per
   *  iteration is maximum_expected_message_rate(Hz)*maximum_iteration_interval(musec)/1000000. **/
  for (ObservationTypeMapIterator i=observation_types_.begin(); i!=observation_types_.end(); i++) {
    // If given use the sensor's max frequency. If not, use maximum observation frequency
    int32_t max_msgs = i->second.has_max_frequency() ? 
        int32_t(i->second.max_frequency()*float(max_expected_iteration_interval_)/1000000.0)+1 :
        int32_t(max_observation_frequency_*float(max_expected_iteration_interval_)/1000000.0)+1; 
    i->second.set_max_msgs_per_iteration(max_msgs);
    LOG(INFO) << "Setting maximum messages per iteration for " << i->first << " to " << max_msgs;
  }
  // Allocate memory for keeping all observations in observations_tracker_
  observations_tracker_.Initiate(
      component_name_+".ObservationsTracker", iterations_per_window_interval_);
  LOG(INFO) << "Observations keeper initiated as " << observations_tracker_.name() << " " <<
      observations_tracker_.size();
  for (int i=0; i<observations_tracker_.size(); i++) {
    // Create an empty map
    std::unique_ptr<ObservationsVectorStoreMap> map_ptr(new ObservationsVectorStoreMap());
    VLOG(4) << "Created empty map i = " << i;
    // Allocate for each observation type
    for (ObservationTypeMapIterator j=observation_types_.begin(); j!=observation_types_.end(); j++) {
      // Create an empty vector
      int32_t vec_size = j->second.max_msgs_per_iteration();
      ObservationsPtrVectorPtr vec_ptr(new std::vector<anantak::MessagePtrType>());
      VLOG(4) << "Created empty vector i = " << i << " msg_type = " << j->first;
      vec_ptr->resize(vec_size);
      vec_ptr->shrink_to_fit();
      // Allocate memory for messages in the vector
      for (int k = 0; k < vec_size; k++) {
        anantak::MessagePtrType msg_ptr(new anantak::SensorMsg());
        (*vec_ptr)[k] = std::move(msg_ptr);
      }
      // Create a vector store in the map
      (*map_ptr)[j->first] = ObservationsVectorStore();
      // Fill the vector store
      (*map_ptr)[j->first].n_observations = 0;
      (*map_ptr)[j->first].observations = std::move(vec_ptr);
      VLOG(4) << "Moved allocated vector to map i = " << i << " msg_type = " << j->first;
    }
    // Increment circular queue and move the map to it
    observations_tracker_.increment();
    observations_tracker_.set_element(std::move(map_ptr));
    VLOG(4) << "Moved allocated map to queue i = " << i;
  }
  LOG(INFO) << "Observations keeper memory allocation finished idx = "
      << observations_tracker_.current_index();
  // Creating a performance tracker for observations keeper
  performance_tracker_.AddTimer("Observations", 100);
  
  
  /** Initiate Models */
  for (ModelConfigIterator i=models_config_.begin(); i!=models_config_.end(); i++) {
    VLOG(1) << "Creating model " << i->first;
    anantak::ModelPtr model_ptr = model_factory_.CreateModel(
                                    i->first, i->second.type(), i->second.config_file());
    if (!model_ptr) {
      LOG(ERROR) << "Got an empty model. Can not continue.";
      return false;
    }
    VLOG(1) << "Initiating model " << i->first;
    if (!model_ptr->Initiate(max_sliding_window_interval_, &observations_tracker_, &states_tracker_,
                        &estimates_tracker_)) {
      LOG(ERROR) << "Could not initiate model " << i->first << ". Quit.";
      return false;
    }
    models_tracker_[i->first] = std::move(model_ptr);   // moved to the map
  }
  
  /** Allocate memory for the Models' States */
  for (ModelPtrMapIterator i=models_tracker_.begin(); i!=models_tracker_.end(); i++) {
    if (!i->second->AllocateMemoryForStates()) {
      LOG(ERROR) << "Could not allocated memory for states for model " << i->first << ". Quit";
      return false;
    }
  }
  
  /** Allocate empty maps for the estimates */
  estimates_tracker_.Initiate(component_name_+".EstimatesTracker", iterations_per_window_interval_);
  for (int i=0; i<iterations_per_window_interval_; i++) {
    // Create a new map of estimates and move it to estimates tracker
    anantak::EstimatesPtrMapPtr map_ptr(new EstimatesPtrMap());
    if (!estimates_tracker_.add_element(std::move(map_ptr))) {
      LOG(ERROR) << "Could not move estimates map to estimates tracker";
      return false;
    }
  }
  estimates_tracker_.increment(); // move pointer back to the beginning of circular array
  
  /** Allocate memory for the Models' Estimates */
  for (ModelPtrMapIterator i=models_tracker_.begin(); i!=models_tracker_.end(); i++) {
    if (!i->second->AllocateMemoryForEstimates()) {
      LOG(ERROR) << "Could not allocated memory for estimates for model " << i->first << ". Quit";
      return false;
    }
  }
  
  // Data fetching variables
  data_fetch_begin_timestamp_ = get_wall_time_microsec();
  data_fetch_end_timestamp_ = data_fetch_begin_timestamp_;
  data_interval_request_msg_.set_type(anantak::DataRequestMsg::INTERVAL);
  data_interval_request_msg_.set_begin_timestamp(data_fetch_begin_timestamp_);
  data_interval_request_msg_.set_end_timestamp(data_fetch_end_timestamp_);

  return true;
}

/** Start Looping - run filter operations and listen to messages */
bool SlidingWindowFilter::StartLooping() {
  
  // Get starting data from DataQueues to initiate the models
  
  anantak::Looper looper(loop_frequency_);
  while (!exit_loop_) {
    
    // Request data from each DataQueue
    data_fetch_begin_timestamp_ = data_fetch_end_timestamp_;
    data_fetch_end_timestamp_ = get_wall_time_microsec();
    SendDataRequestsToDataQueues();
    
    // Iteration begin
    iterations_tracker_.add_element(data_fetch_end_timestamp_);
    
    // Loop through each subscriber, read messages and pass them to status keeper objects
    for (PubSubMapIteratorType i_map = subscriptions_map_.begin();
        i_map != subscriptions_map_.end(); i_map++) {
      VLOG(4) << "Reading subscriber " << i_map->first;
      
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
          std::unique_ptr<std::string> msg_str_ptr(       // copy the entire message
              new std::string(msg_data_ptr, msg_size));   // copy
          VLOG(1) << "Command: " << *msg_str_ptr;
          HandleCommand(std::move(msg_str_ptr));
        }
        
        /** Handle status queries */
        if (subscriber_type_[i_map->first] == kStatus) {
          std::unique_ptr<std::string> msg_str_ptr(       // copy the entire message
              new std::string(msg_data_ptr, msg_size));   // copy
          VLOG(3) << "Status query: " << *msg_str_ptr;
          HandleStatusQuery(std::move(msg_str_ptr));
        }
        
        /** Handle data subscriber query */
        if (subscriber_type_[i_map->first] == kDataQueue) {
          int address_length = subscriber_subject_length_[i_map->first];
          std::unique_ptr<std::string> msg_str_ptr(             // copy all but the address
              new std::string(msg_data_ptr+address_length, msg_size-address_length));  // copy
          VLOG(4) << "DataReply message: " << *msg_str_ptr;
          ProcessDataRepliesFromDataQueues(i_map, std::move(msg_str_ptr));
        }
        
        // Message_String unique pointer destructs here if it has not been moved
      } // recv message
      // Message is destructed here
    } // for
    
    if (!exit_loop_) looper.Sleep();
    max_loop_frequency_ = looper.max_frequency();
  }
  
  return true;
}

/** Handle commands coming from the commander subscriber */
bool SlidingWindowFilter::HandleCommand(StringPtrType cmd) {
  // exit command
  if (*cmd == exit_command_str_) exit_loop_ = true;
  return true;
}

/** Handle status queries coming from ProgramCommander */
bool SlidingWindowFilter::HandleStatusQuery(StringPtrType status_query) {
  StringPtrType status_query_reply_str =
      status_keeper_->GenerateStatusQueryReplyMessage(std::move(status_query));
  VLOG_EVERY_N(1, 1000) << "Status query reply: " << *status_query_reply_str;
  zmq::message_t status_query_reply_msg(status_query_reply_str->size());
  // Copy string buffer to message buffer - this can potentially be avoided using zero-copy
  memcpy(status_query_reply_msg.data(), status_query_reply_str->data(),
         status_query_reply_str->size());
  bool sent_ok =
      publishers_map_[publisher_name_]->send(status_query_reply_msg, ZMQ_DONTWAIT);
  if (!sent_ok) {
    LOG(ERROR) << "Status Query Reply message was not sent";
    return false;
  }
  return true;
}

/** Assemble DataQueue status string */
SlidingWindowFilter::StringPtrType SlidingWindowFilter::AssembleStatusString() {
  StringPtrType status_str(new std::string(""));
  // Looping frequency
  {
    char buffer[100];
    if (max_loop_frequency_>1000.0f) snprintf(buffer, 100, "%.0f(>1k)", loop_frequency_);
    else snprintf(buffer, 100, "%.0f(%.0f)", loop_frequency_, max_loop_frequency_);
    *status_str += buffer;
  }
  return status_str;
}

/** SendDataRequestsToDataQueues */
bool SlidingWindowFilter::SendDataRequestsToDataQueues() {
  // Build a DataRequestMsg - calling Clear() is not needed as this message 'shape' does not change
  data_interval_request_msg_.set_begin_timestamp(data_fetch_begin_timestamp_);
  data_interval_request_msg_.set_end_timestamp(data_fetch_end_timestamp_);
  // Go through each DataQueue, send data requests
  for (StringVectorIteratorType i=data_queues_.begin(); i!=data_queues_.end(); i++) {
    VLOG(4) << "Sending data request to " << *i;
    // Build a transport message to be sent to this DataQueue
    std::string address = *i + " ";
    uint32_t address_size = address.size();
    uint32_t msg_size = data_interval_request_msg_.ByteSize();
    // Allocate a transport message of size = address size + message size
    data_interval_request_msg_size_ = address_size + msg_size;
    zmq::message_t transport_msg(data_interval_request_msg_size_);
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
      // The message should fit in one buffer, so use the direct-to-array serialization.
      data_interval_request_msg_.SerializeWithCachedSizesToArray(buffer);    // copy
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
  } // for
  return true;
}

/** Extract HeaderMsg from a SensorMsg serialized as string */
bool ExtractHeaderMsg(const std::string& serial_str, anantak::HeaderMsg* header_msg) {
  // Setup a CodedInputStream on the string
  google::protobuf::io::ArrayInputStream raw_in_stream(serial_str.c_str(), serial_str.size());
  google::protobuf::io::CodedInputStream coded_in_stream(&raw_in_stream);
  /* We read the first tag. Should be 10 - for header coded in binary as: 0000 1010
   * This includes last 3 bits coding the Length-delimited Type = 2 = 010 and
   * first bits of tag# for header = 1 = 0000 0001. */
  if (!coded_in_stream.ExpectTag(10)) {
    return false;   // could not find the header at the beginning
  }
  //uint32_t tag0 = coded_in_stream.ReadTag();    // not needed as ExpectTag advances pointer
  uint32_t size0;
  coded_in_stream.ReadVarint32(&size0);
  char buffer[512]; // header should not be more than this size
  coded_in_stream.ReadRaw(buffer, size0);
  bool success = (*header_msg).ParseFromArray(buffer, size0);
  //if (!success) VLOG(2) << "Header parse not successful";
  //else VLOG(2) << "Header type = " << header_msg->type();
  return success;
}

/** ProcessDataRepliesFromDataQueues */
bool SlidingWindowFilter::ProcessDataRepliesFromDataQueues(
    PubSubMapIteratorType iter, StringPtrType data) {
  // Trigger the performance tracker
  performance_tracker_("Observations").StartTimer();
  int32_t n_obs = 0;
  // Deserialize the message into data_reply_msg_ in the most portable way possible
  if (!data_reply_msg_.ParseFromString(*data)) {
    LOG(ERROR) << "Could not parse data reply from " << iter->first;
    return false;
  }
  int n_msgs = data_reply_msg_.message_data_size();
  VLOG(1) << "Got a data reply from " << iter->first << " with " << n_msgs << " messages";
  // Save the observations
  observations_tracker_.increment(); // Move to next place holder in observations queue
  ObservationsVectorStoreMap* map_ptr = observations_tracker_.mutable_element_ptr();
  // Set the number of messages to 0
  for (ObservationsVectorStoreMapIterator i_map=map_ptr->begin(); i_map!=map_ptr->end(); i_map++) {
    i_map->second.n_observations = 0;
    // we do not need to Clear() the observations as n_msgs encodes how many observations came in
  }
  int32_t dropped_observations = 0;
  // Go through each message, check their type and deserialize accordingly
  for (int i=0; i<n_msgs; i++) {
    // Only extract the header - here we avoid deserializing the entire message
    anantak::HeaderMsg header_msg;
    if (!ExtractHeaderMsg(data_reply_msg_.message_data(i), &header_msg)) {
      LOG(ERROR) << "Could not extract header";
      continue;
    }
    const std::string& sensor_msg_type = header_msg.type();  // No copying, getting by reference
    ObservationsVectorStoreMapIterator i_map = map_ptr->find(sensor_msg_type);
    if (i_map != map_ptr->end()) {  // Make sure this observation type is processed by this filter
      // Make sure that we have space to store this message in the observations storage
      if (i_map->second.n_observations >= i_map->second.observations->size()) {
        LOG(ERROR) << "Got more messages for " << sensor_msg_type << " than allocated memory. "
            << "Dropping message! Increase allocation in Filter configuration";
        continue;
      }
      // Parse the data string into the message provided. Wonder if this looks too convoluted?
      if (!i_map->second.observations->at(i_map->second.n_observations)->ParseFromString(
          data_reply_msg_.message_data(i))) {
        continue;
      }
      // Store this observation in the vector for this observation type
      i_map->second.n_observations++;
      n_obs++;
    } else {
      // This observation type is not processed by this filter, drop it silently
      dropped_observations++;
    }
  }
  // Report what was recieved
  for (ObservationsVectorStoreMapIterator i_map=map_ptr->begin(); i_map!=map_ptr->end(); i_map++) {
    VLOG(2) << "  " << i_map->first << " " << i_map->second.n_observations;
  }
  if (dropped_observations>0) VLOG(2) << "  " << "Dropped " << dropped_observations;
  //VLOG(2) << "  Stored in observations map index " << observations_tracker_.current_index();
  // Capture performance
  performance_tracker_("Observations").StopTimer(n_obs);
  VLOG(2) << "  Avg time = " << performance_tracker_("Observations").average_timer_time();
  VLOG(2) << "  Avg time per msg = " << performance_tracker_("Observations").average_event_time();
  VLOG(2) << "  Call rate = " << performance_tracker_("Observations").call_rate();
  return true;
  // data gets destructed here
}

} // namespace anantak
