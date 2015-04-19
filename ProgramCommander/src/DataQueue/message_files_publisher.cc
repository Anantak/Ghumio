/**
 *  Message Files Publisher implementation
 */

/** std includes */
#include <sys/time.h>

/** Header include */
#include "DataQueue/message_files_publisher.h"

/** Anantak includes */
#include "common_config.h"
#include "Utilities/common_functions.h"
#include "DataQueue/message_file_stats.h"

/** Google logging and flags libraries */
#include <glog/logging.h>

/** Protocol Buffers */
#include "configurations.pb.h"

/** ZMQ includes */
#include <zmq.hpp>

namespace anantak {

/** Constructor */
MessageFilesPublisher::MessageFilesPublisher(
    std::string programs_setup_filename,
    std::string component_name) {
  programs_setup_config_filename_ = programs_setup_filename;
  component_name_ = component_name;
}

/** Destructor - all members are self destructing */
MessageFilesPublisher::~MessageFilesPublisher() {
  // Check if the data files are still open. If so, close them
  for (FileReaderMapIteratorType i_map = data_file_readers_.begin();
       i_map != data_file_readers_.end(); i_map++) {
    VLOG(1) << "Closing data file for " << i_map->first;
    if (i_map->second->file_is_open()) i_map->second->Close();
  }
  VLOG(1) << "MessageFilesPublisher shutdown. All objects should be destructed automatically.";
  VLOG(1) << "This includes the ZMQ transport and all pub/subs.";  
}

/** Setup the objects and start publishing loop */
bool MessageFilesPublisher::StartPublishing() {
  if (!Initiate()) return false;
  if (!CreateLoadingSchedule()) return false;
  if (!CreateFileReaders()) return false;
  if (!StartLooping()) return false;
  return true;
}

/** Initiator - creates all starting objects */
bool MessageFilesPublisher::Initiate() {
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
    if (publication_settings.has_subject())
        publish_subjects_map_[publication_settings.name()] = publication_settings.subject();
        else publish_subjects_map_[publication_settings.name()] = "";
  }
  for (int i=0; i<program_settings.subscription_size(); i++) {
    const anantak::ProgramsSetup::ProgramSettings::ConnectionSettings& subscription_settings =
        program_settings.subscription(i);
    program_setting_subs_map[subscription_settings.name()] = i;
  }

  // Read the config file
  std::string config_file_path = anantak::GetProjectSourceDirectory() + "/" + config_filename_;
  VLOG(1) << "Using config file = " << config_file_path;
  std::unique_ptr<anantak::MessageFilesPublisherConfig> config =
      anantak::ReadProtobufFile<anantak::MessageFilesPublisherConfig>(config_file_path);
  VLOG(1) << "Number of sensors: " << config->sensor_size();
  if (!config->has_commander_name() || !config->has_status_name() || !config->has_publisher_name()) {
    LOG(ERROR) << "Config file does not have either of commander_name, status_name, publisher name";
    LOG(ERROR) << "All three are necessary for operation of the component";
    return false;
  }
  
  // Build StatusKeeper to publish component status
  { /* Wish there was some more beautiful way of initiating this. 'Right' way is using initializer
     * list of the constructor, but I just dont like 'cramming' code in initializer list, may be
     * need to get over it. Till then, allocating a temp pointer, then moving to member */
    StatusKeeperPtrType temp_ptr(new anantak::MessageFilesPublisherStatusKeeper(component_name_));
    status_keeper_ = std::move(temp_ptr);   // class member pointer now owns the keeper
    // Setup a callback for the StatusKeeper to generate a DataQueue status string.
    status_keeper_->set_component_status_function( [&](){return AssembleStatusString();} );
    if (!status_keeper_) {
      LOG(ERROR) << "Could not create StatusKeeper.";
      return false;
    }
    VLOG(1) << "Built a status keeper";
  }
  
  // Data Filenames
  for (int i=0; i<config->sensor_size(); i++) {
    const anantak::MessageFilesPublisherConfig::Sensor& sensor = config->sensor(i);
    if (sensor.has_data_file()) {
      data_filenames_[sensor.name()] = sensor.data_file();
      VLOG(1) << "Sensor \"" << sensor.name() << "\" data file \"" << sensor.data_file() << "\"";
    } else {
      LOG(ERROR) << "Could not find data file for sensor " << sensor.name();
      return false;
    }
  }
  
  // Build ZMQ transport
  { /* same as above */
    PubSubTransportPtrType temp_ptr(new zmq::context_t(1));
    zmq_transport_ = std::move(temp_ptr);     // class member ptr now owns this
    VLOG(1) << "Built ZMQ transport";
  }
  
  // Build Publishers
  publisher_name_ = config->publisher_name();             // copying to class member
  VLOG(1) << "Publisher name = " << publisher_name_;
  /* For each sensor in config check if the program settings has pub-specs for it. If so, create
   * the publisher. Else write out an error and do not create a publisher */  
  std::vector<std::string> publishers_to_be_created;
  publishers_to_be_created.push_back(publisher_name_);
  for (int i=0; i<config->sensor_size(); i++) {
    const anantak::MessageFilesPublisherConfig::Sensor& sensor = config->sensor(i);
    publishers_to_be_created.push_back(sensor.name());
  }
  for (int i=0; i<publishers_to_be_created.size(); i++) {
    if (program_setting_pubs_map.find(publishers_to_be_created[i]) == program_setting_pubs_map.end()) {
      LOG(ERROR) << "Could not find publisher settings for " << publishers_to_be_created[i];
      return false;
    }
  }
  for (int i=0; i<publishers_to_be_created.size(); i++) {
    std::string pub_name = publishers_to_be_created[i];
    const anantak::ProgramsSetup::ProgramSettings::ConnectionSettings& publisher_settings =
        program_settings.publication(program_setting_pubs_map[pub_name]);
    PubSubPtrType ptr(new zmq::socket_t(*zmq_transport_, ZMQ_PUB));
    ptr->bind(publisher_settings.endpoint().c_str());
    publishers_map_[pub_name] = std::move(ptr);
    VLOG(1) << "Bound: " << publisher_settings.name() << " " << publisher_settings.endpoint();
  }
  
  // Build Subscribers
  command_subscriber_name_ = config->commander_name();    // copying to class member
  status_subscriber_name_ = config->status_name();        // copying to class member
  VLOG(1) << "Command subscriber name = " << command_subscriber_name_;
  VLOG(1) << "Status subscriber name = " << status_subscriber_name_;
  /* For each subscriber in config check if the program settings has specs for it. If so, create
   * the subscriber. Else write out an error and do not create a susbcriber */  
  std::vector<std::string> subscribers_to_be_created;
  subscribers_to_be_created.push_back(command_subscriber_name_);
  subscribers_to_be_created.push_back(status_subscriber_name_);
  for (int i=0; i<subscribers_to_be_created.size(); i++) {
    if (program_setting_subs_map.find(subscribers_to_be_created[i]) == program_setting_subs_map.end()) {
      LOG(ERROR) << "Could not find subscriber settings for " << subscribers_to_be_created[i];
      return false;
    }
  }
  for (int i=0; i<subscribers_to_be_created.size(); i++) {
    std::string sub_name = subscribers_to_be_created[i];
    const anantak::ProgramsSetup::ProgramSettings::ConnectionSettings& subscriber_settings =
        program_settings.subscription(program_setting_subs_map[sub_name]);
    PubSubPtrType ptr(new zmq::socket_t(*zmq_transport_, ZMQ_SUB));
    ptr->connect(subscriber_settings.endpoint().c_str());
    ptr->setsockopt(ZMQ_SUBSCRIBE, subscriber_settings.subject().c_str(),
        subscriber_settings.subject().length());
    subscriptions_map_[sub_name] = std::move(ptr);
    VLOG(1) << "Connected: " << subscriber_settings.name() << " " << subscriber_settings.endpoint()
        << " \"" << subscriber_settings.subject() << "\"";
  }
  
  // Looping variables
  if (config->has_cycle_frequency()) loop_frequency_ = config->cycle_frequency();
      else loop_frequency_ = 200.0;  // Hz
  exit_loop_ = false;
  exit_command_str_ = "COMMAND " + component_name_ + " exit";
  play_command_str_ = "COMMAND " + component_name_ + " play";
  stop_command_str_ = "COMMAND " + component_name_ + " stop";
  pause_command_str_ = "COMMAND " + component_name_ + " pause";
  max_loop_frequency_ = 0.0f;

  // State and data playing variables
  publisher_state_ = kStart;
  last_real_time_ = 0;
  curr_real_time_ = 0;
  curr_play_time_ = 0;
  last_hist_time_ = 0;
  curr_hist_time_ = 0;
  
  return true;
}

/** Create Loading Schedule parameters */
bool MessageFilesPublisher::CreateLoadingSchedule() {
  // Read all data files and load timestamps
  for (StringsMapIteratorType i_map = data_filenames_.begin(); i_map != data_filenames_.end();
      i_map++) {
    VLOG(1) << "Reading stats for file for " << i_map->first;
    anantak::MessageFileStats file_stats;
    file_stats.CalculateFileStats(i_map->second);
    /** We can access a reference to the vector of send_timestamps to avoid making a copy of vector.
     *  We could make a copy easily too by assigning to a new vector variable. */
    TimeVectorPtrType ptr(new std::vector<int64_t>(file_stats.timestamps()));  // make a copy
    data_timestamps_[i_map->first] = std::move(ptr);    // send it to the time vector map
    VLOG(1) << "Read " << data_timestamps_[i_map->first]->size() << " timestamps for "
        << i_map->first << " sensor";
    // file_stats destructs here
  }
  
  // We assume that timestamps are monotonically increasing. Here we make sure that that is so.
  schedule_start_time_ = get_wall_time_microsec();
  schedule_end_time_ = 0;
  for (TimeVectorMapIteratorType i_map = data_timestamps_.begin(); i_map != data_timestamps_.end();
      i_map++) {
    if (i_map->second->size() > 0) {
      int64_t sensor_last_time = i_map->second->at(0);
      for (int i=0; i<i_map->second->size(); i++) {
        int64_t sensor_this_time = i_map->second->at(i);
        if (schedule_start_time_ > sensor_this_time) schedule_start_time_ = sensor_this_time;
        if (schedule_end_time_ < sensor_this_time) schedule_end_time_ = sensor_this_time;
        if (sensor_last_time > sensor_this_time) {
          LOG(ERROR) << "Sensor " << i_map->first << " found decreasing timestamp! " <<
              sensor_last_time << " " << sensor_this_time << ". Expected monotonically increasing";
          return false;
        }
        sensor_last_time = sensor_this_time;
      }
    } // if size > 0
  }
  
  // Find the closest second for the schedule start/end timestamps
  schedule_start_time_ = (schedule_start_time_ / 1000000) * 1000000;    // take floor
  schedule_end_time_ = (schedule_end_time_ / 1000000 + 1) * 1000000;    // take ceiling
  int64_t schedule_period = schedule_end_time_ - schedule_start_time_;
  schedule_period_str_ = anantak::microsec_to_time_str(schedule_period);
  VLOG(1) << "Schedule start timestamp = " << schedule_start_time_;
  VLOG(1) << "Schedule end timestamp = " << schedule_end_time_;
  VLOG(1) << "Schedule period = " << schedule_period_str_;
  return true;
}

/** Create file readers to setup the files for reading */
bool MessageFilesPublisher::CreateFileReaders() {
  for (StringsMapIteratorType i_map = data_filenames_.begin(); i_map != data_filenames_.end();
      i_map++) {  
    FileReaderPtrType reader(new anantak::MessageFileReader);   // an empty file reader
    if (!reader->Open(i_map->second)) {                   // try opening the file
      LOG(ERROR) << "File " << i_map->second << " could not be opened.";
      return false;
    } else {
      // pass the file reader to the map
      data_file_readers_[i_map->first] = std::move(reader);
      // add an entry for the reader index
      message_index_[i_map->first] = 0;
    }
  }
  return true;
}

/** Reset file readers and file indexes */
bool MessageFilesPublisher::ResetFileReaders() {
  // Check if the data files are open. If so, close them. Set message indexes to 0
  for (FileReaderMapIteratorType i_map = data_file_readers_.begin();
       i_map != data_file_readers_.end(); i_map++) {
    VLOG(1) << "Reopening data file for " << i_map->first;
    i_map->second->Reopen();
    message_index_[i_map->first] = 0;
  }
  return true;
}

/** Start Looping - read files and listen to messages */
bool MessageFilesPublisher::StartLooping() {
  
  anantak::Looper looper(loop_frequency_);
  
  last_real_time_ = get_wall_time_microsec();
  curr_real_time_ = last_real_time_;
  curr_play_time_ = 0;
  last_hist_time_ = schedule_start_time_;
  curr_hist_time_ = schedule_start_time_;
  
  while (!exit_loop_) {
    
    /** Depending on the state, the play time propagates. */
    curr_real_time_ = get_wall_time_microsec();
    int64_t time_passed = curr_real_time_ - last_real_time_;
    last_real_time_ = curr_real_time_;
    if (publisher_state_ == kStart) {
      curr_play_time_ = 0;
      last_hist_time_ = schedule_start_time_;
      curr_hist_time_ = schedule_start_time_;      
    } else if (publisher_state_ == kPlaying) {
      last_hist_time_ = schedule_start_time_ + curr_play_time_;
      curr_play_time_ += time_passed;
      curr_hist_time_ = schedule_start_time_ + curr_play_time_;
      if (curr_hist_time_ > schedule_end_time_) {
        publisher_state_ = kEnded;
      }
    } else if (publisher_state_ == kPaused) {
      last_hist_time_ = schedule_start_time_ + curr_play_time_;
      curr_hist_time_ = schedule_start_time_ + curr_play_time_;
    } else if (publisher_state_ == kEnded) {
      last_hist_time_ = schedule_start_time_ + curr_play_time_;
      curr_hist_time_ = schedule_start_time_ + curr_play_time_;
    }
    
    // Check for messages in data files in (last_hist_time_, curr_hist_time_]. Transmit them.
    if (last_hist_time_ != curr_hist_time_) {
      for (TimeVectorMapIteratorType i_map = data_timestamps_.begin();
          i_map != data_timestamps_.end(); i_map++) {
        if ((i_map->second->size() > 0) &&
            (message_index_[i_map->first] < i_map->second->size())) {
          
          while (i_map->second->at(message_index_[i_map->first]) < curr_hist_time_) {
            // read and transmit one message from file reader
            if (data_file_readers_[i_map->first]->file_is_open()) {
              std::string address = publish_subjects_map_[i_map->first];
              uint32_t address_size = address.size();
              /** We are not interested in the contents of the message, so we do not parse it. We
               *  only want to copy the message in its serialized format to a message to be sent
               *  as it is. We do this by first creating a ZMQ message with the size of address +
               *  message size, then copy the address and message contents to it. */
              uint32_t msg_size;
              if (data_file_readers_[i_map->first]->ReadNextMessageSize(&msg_size)) {
                // Allocate a message of size = address size + message size
                zmq::message_t msg(address_size + msg_size);
                // Copy address to the message
                memcpy(msg.data(), address.data(), address.size());
                // Copy file data to message
                void* msg_ptr = msg.data();
                msg_ptr = static_cast<char*>(msg_ptr) + address_size;
                data_file_readers_[i_map->first]->ReadNextMessageRawToBuffer(msg_ptr, msg_size);
                // Send the message
                bool sent_ok = publishers_map_[i_map->first]->send(msg, ZMQ_DONTWAIT);
                if (!sent_ok) {
                  LOG(ERROR) << "Message was not sent";
                } 
              } else {
                LOG(ERROR) << "Could not get message size from file reader. Closing the file";
                data_file_readers_[i_map->first]->Close();
              }
            } else {
              LOG(ERROR) << "Expected the file to be open for sensor " << i_map->first;
            }
            // increment the index of the file
            message_index_[i_map->first]++;
            if (message_index_[i_map->first] >= i_map->second->size()) break;
          }
          
        } // if size > 0
      } // for
    }
    
    // Loop through each subscriber, read messages and pass them to status keeper objects
    for (PubSubMapIteratorType i_map = subscriptions_map_.begin(); i_map != subscriptions_map_.end();
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
        const char* msg_data_ptr = static_cast<char*>(message.data());
        std::unique_ptr<std::string> msg_str_ptr(new std::string(msg_data_ptr, msg_size)); // copy
        VLOG(3) << "Message at " << i_map->first << "\n" << *msg_str_ptr;
        
        /** Handle commands */
        if (i_map->first == command_subscriber_name_) {
          VLOG(1) << "Command: " << *msg_str_ptr;
          HandleCommand(std::move(msg_str_ptr));
        }
        
        /** Handle status queries */
        if (i_map->first == status_subscriber_name_) {
          VLOG(2) << "Status query: " << *msg_str_ptr;
          HandleStatusQuery(std::move(msg_str_ptr));
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
bool MessageFilesPublisher::HandleCommand(StringPtrType cmd) {
  // exit command
  if (*cmd == exit_command_str_) exit_loop_ = true;
  if (*cmd == play_command_str_) {
    if (publisher_state_ == kStart) {
      // transition state to playing
      publisher_state_ = kPlaying;
    }
    if (publisher_state_ == kPaused) {
      // transition state to playing
      publisher_state_ = kPlaying;
    }
  }
  if (*cmd == pause_command_str_) {
    if (publisher_state_ == kPlaying) {
      // transition state to paused
      publisher_state_ = kPaused;
    }
  }
  if (*cmd == stop_command_str_) {
    if (publisher_state_ == kPlaying) {
      // transition state to start
      publisher_state_ = kStart;
      ResetFileReaders();
    }
    if (publisher_state_ == kPaused) {
      // transition state to start
      publisher_state_ = kStart;
      ResetFileReaders();
    }
    if (publisher_state_ == kEnded) {
      // transition state to start
      publisher_state_ = kStart;
      ResetFileReaders();
    }
  }
  return true;
}

/** Handle status queries coming from ProgramCommander */
bool MessageFilesPublisher::HandleStatusQuery(StringPtrType status_query) {
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
  }
  return true;
}

/** Assemble DataQueue status string */
MessageFilesPublisher::StringPtrType MessageFilesPublisher::AssembleStatusString() {
  StringPtrType status_str(new std::string(""));
  switch (publisher_state_) {
    case kStart: *status_str += "Start "; break;
    case kPlaying: *status_str += "Playing "; break;
    case kPaused: *status_str += "Paused "; break;
    case kEnded: *status_str += "Ended "; break;
  }
  *status_str += anantak::microsec_to_time_str(curr_play_time_);
  *status_str += "/"+schedule_period_str_+" ";
  for (MessageIndexMapIteratorType i_map = message_index_.begin(); i_map != message_index_.end();
      i_map++) {
    *status_str += std::to_string(message_index_[i_map->first])+"/"
        +std::to_string(data_timestamps_[i_map->first]->size())+" ";
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

} // namespace anantak