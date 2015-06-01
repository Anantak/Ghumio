/** Pixy Camera Component
 * Implementation
 */


/** Header file include */
#include "PixyBroadcaster/pixy_camera_component.h"

/** Anantak includes */
#include "common_config.h"
#include "Utilities/common_functions.h"

/** Google logging and flags libraries */
#include <glog/logging.h>

/** Protocol Buffers */
#include <google/protobuf/io/zero_copy_stream_impl_lite.h>    // for creating data request msg
#include <google/protobuf/io/coded_stream.h>                  // for creating data request msg

// std includes
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <fstream>

namespace anantak {

/** Constructor */
PixyCameraComponent::PixyCameraComponent(std::string programs_setup_filename,
    std::string component_name) {
  programs_setup_config_filename_ = programs_setup_filename;
  component_name_ = component_name;  
  is_initiated_ = false;
  if (!Initiate()) LOG(ERROR) << "Could not initiate PixyCameraComponent.";
}

/** Destructor - all members are self destructing */
PixyCameraComponent::~PixyCameraComponent() {
  VLOG(1) << "PixyCameraComponent shutdown.";
  VLOG(1) << "ZMQ transport and all pub/subs will destruct automatically.";
}

/** Initiator - creates all starting objects */
bool PixyCameraComponent::Initiate() {
  if (!InitiateComponent()) {
    LOG(ERROR) << "Could not initiate PixyCameraComponent component objects";
    return false;
  }
  if (!InitiatePixyCamera()) {
    LOG(ERROR) << "Could not initiate PixyCamera objects";
    return false;    
  }
  // Initiation succeeded
  is_initiated_ = true;
  return true;
}

/** Component Initiator - creates all starting component objects */
bool PixyCameraComponent::InitiateComponent() {
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
  std::unique_ptr<anantak::PixyCameraConfig> config =
      anantak::ReadProtobufFile<anantak::PixyCameraConfig>(config_file_path);
  if (config->has_symlink()) {
    symlink_ = config->symlink();
  } else {
    LOG(ERROR) << "No pixy symlink was found in config file? Can not continue.";
    return false;
  }
  VLOG(1) << "Pixy camera symlink: " << symlink_;
  if (!config->has_commander_name() || !config->has_status_name() || !config->has_publisher_name()) {
    LOG(ERROR) << "Config file does not have either of commander_name, status_name, publisher name";
    LOG(ERROR) << "All three are necessary for operation of the component";
    return false;
  }
  
  // Build StatusKeeper to publish component status
  { /* Wish there was some more beautiful way of initiating this. 'Right' way is using initializer
     * list of the constructor, but I just dont like 'cramming' code in initializer list, may be
     * need to get over it. Till then, allocating a temp pointer, then moving to member */
    if (program_settings.has_component_template()) {
      LOG(INFO) << "Using component template = " << program_settings.component_template();
      StatusKeeperPtrType temp_ptr(new anantak::ComponentStatusKeeper(component_name_,
          program_settings.component_template()));
      status_keeper_ = std::move(temp_ptr);   // class member pointer now owns the keeper
    } else {
      StatusKeeperPtrType temp_ptr(new anantak::ComponentStatusKeeper(component_name_));
      status_keeper_ = std::move(temp_ptr);   // class member pointer now owns the keeper      
    }
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
  start_command_str_ = "COMMAND " + component_name_ + " start";
  log_command_str_ = "COMMAND " + component_name_ + " log";
  show_command_str_ = "COMMAND " + component_name_ + " show";
  max_loop_frequency_ = 0.0f;
  
  // Turn over the config object to class storage
  config_= std::move(config);
  message_publisher_ = publishers_map_[publisher_name_].get();
  
  return true;
}

/** Initiator - creates starting image processing objects */
bool PixyCameraComponent::InitiatePixyCamera() {
  
  pixy_camera_.Initialize(symlink_);
  if (!pixy_camera_.IsInitialized()) {
    LOG(ERROR) << "Pixy camera was not initialized. Exit.";
    return false;
  }
  
  pixy_message_subject_ = component_name_ + " ";
  LOG(INFO) << "Pixy messages will be published on subject = '" << pixy_message_subject_;
  pixy_message_subject_length_ = pixy_message_subject_.length();
  log_level_ = 0;
  show_image_ = false;
  
  CV_RED = cv::Scalar(0,0,255);
  CV_WHITE = cv::Scalar(255,255,255);
  CV_YELLOW = cv::Scalar(0,255,255);
  CV_BLACK = cv::Scalar(0,0,0);
  
  image_width_ = config_->image_width();
  image_height_ = config_->image_height();
  image_size_ = cv::Size(image_width_, image_height_);
  pixy_image_.create(image_size_, CV_8UC3);
  
  return true;
}

/** Handle commands coming from the commander subscriber */
bool PixyCameraComponent::HandleCommand(StringPtrType cmd) {
  // exit command
  if (*cmd == exit_command_str_) {exit_loop_ = true;}
  else if (*cmd == show_command_str_) {
    if (!show_image_) {
      show_image_ = true;
      cv::namedWindow(component_name_, cv::WINDOW_AUTOSIZE ); // Create a window for display
      cv::moveWindow(component_name_, 0, 0); // move window to a new positon
    } else {
      show_image_ = false;
      cv::destroyWindow(component_name_); // destroy the display window
      cv::waitKey(10);
      cv::destroyWindow(component_name_); // destroy the display window
      cv::waitKey(10);
      cv::destroyWindow(component_name_); // destroy the display window
      cv::waitKey(10);
      cv::destroyWindow(component_name_); // destroy the display window
      cv::waitKey(10);
      cv::destroyWindow(component_name_); // destroy the display window
      cv::waitKey(10);                
    }
  }
  else if (*cmd == log_command_str_) {if (log_level_==0) log_level_=1; else log_level_=0;}
  return true;
}

/** Handle status queries coming from ProgramCommander */
bool PixyCameraComponent::HandleStatusQuery(StringPtrType status_query) {
  StringPtrType status_query_reply_str =
      status_keeper_->GenerateStatusQueryReplyMessage(std::move(status_query));
  VLOG_EVERY_N(1, 1000) << "Status query reply: " << *status_query_reply_str << "\n";
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

/** Start Looping - run operations and listen to messages */
bool PixyCameraComponent::StartLooping() {
  
  // Get starting data from DataQueues to initiate the models
  
  anantak::Looper looper(loop_frequency_);
  while (!exit_loop_) {
    
    // Read the pixy camera and publish the message
    ReadPixyAndPublish();
    
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
        
        // Message_String unique pointer destructs here if it has not been moved
      } // recv message
      // Message is destructed here
    } // for
    
    if (!exit_loop_) looper.Sleep();
    max_loop_frequency_ = looper.max_frequency();
  }
  
  return true;
}

/** Assemble status string */
PixyCameraComponent::StringPtrType PixyCameraComponent::AssembleStatusString() {
  StringPtrType status_str(new std::string(""));
  // Looping frequency
  {
    char buffer[100];
    snprintf(buffer, 100, "%.2f Hz", max_loop_frequency_);
    //snprintf(buffer, 100, "Arvl,Extr,Proc,Disp %ld %ld %ld %ld(us) Proc,Drop %ld %ld",
    //    msg_arrival_delay, msg_compiling_delay, msg_processing_delay, image_display_delay,
    //    processed_frames, dropped_frames);
    *status_str += buffer;
  }
  return status_str;
}

/** Read a pixy message and publish it **/
bool PixyCameraComponent::ReadPixyAndPublish() {
  
  pixy_camera_.GetNewPixyMessage();
  if (log_level_>0) std::cout << "  Pixy cam message = " << pixy_camera_.PixyCameraMessageToString() << std::endl;
  
  // Only publish where there are any blocks to send. Otherwise remain silent.
  if (pixy_camera_.blocks_copied > 0) {
    // Get message size
    size_t message_length_ = pixy_camera_.pixy_message_.ByteSize();
    // Allocate a message of size = address size + message size
    zmq::message_t msg(pixy_message_subject_length_ + message_length_);
    // Copy address to the message
    void* msg_ptr = msg.data();
    memcpy(msg_ptr, pixy_message_subject_.data(), pixy_message_subject_length_);
    // Increment pointer to end of subject
    msg_ptr = static_cast<char*>(msg_ptr) + pixy_message_subject_length_;
    // Serialize the message to the stream
    pixy_camera_.pixy_message_.SerializeToArray(msg_ptr, message_length_);
    bool sent_ok = message_publisher_->send(msg, ZMQ_DONTWAIT);
    if (!sent_ok) {
      LOG(ERROR) << "Message was not sent";
    }
  }
  
  if (show_image_) {
    // Clear the image
    pixy_image_.setTo(CV_BLACK);
    // Draw rectangles
    const anantak::PixyCameraMessage& pixy_msg = pixy_camera_.pixy_message_.pixy_cam_msg();
    for (int i=0; i<pixy_msg.blocks_size(); i++) {
      const anantak::PixyCameraMessage::Block& bl = pixy_msg.blocks(i);
      cv::Point pt1(bl.x(), bl.y());
      cv::Point pt2(bl.width(), bl.height());
      cv::rectangle(pixy_image_, pt1, pt1+pt2, CV_WHITE, 1);
    }
    // Show image
    cv::imshow(component_name_, pixy_image_);
    cv::waitKey(1);
    
  }
  
  return true;
}


} // namespace anantak