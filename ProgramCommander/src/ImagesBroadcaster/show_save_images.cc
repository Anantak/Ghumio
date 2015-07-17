/**
 *  Show Save Images
 *
 *  Implementation of Show Save Images
 */

/** Header file include */
#include "ImagesBroadcaster/show_save_images.h"

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
#include <algorithm>


namespace anantak {

// Static constants
const std::string ShowSaveImages::APRIL_MESSAGE_TYPE = "AprilTags";

/** Sliding Window Filter constructor */
ShowSaveImages::ShowSaveImages(std::string programs_setup_filename,
    std::string component_name) {
  programs_setup_config_filename_ = programs_setup_filename;
  component_name_ = component_name;  
  is_initiated_ = false;
  if (!Initiate()) LOG(ERROR) << "Could not initiate SyncImagesBroadcaster.";
}

/** Destructor - all members are self destructing */
ShowSaveImages::~ShowSaveImages() {
  VLOG(1) << "ShowSaveImages shutdown.";
  VLOG(1) << "ZMQ transport and all pub/subs will destruct automatically.";
  
  VLOG(1) << "Destructing AprilTag C library objects";
  tag36h11_destroy(tag_family_);
  apriltag_detector_destroy(tag_detector_);
  image_u8_destroy(image_u8_);
}

/** Initiator - creates all starting objects */
bool ShowSaveImages::Initiate() {
  if (!InitiateComponent()) {
    LOG(ERROR) << "Could not initiate ShowSaveImages component objects";
    return false;
  }
  if (!InitiateShowSaveImages()) {
    LOG(ERROR) << "Could not initiate ShowSaveImages display objects";
    return false;    
  }
  // Initiation succeeded
  is_initiated_ = true;
  return true;
}

/** Component Initiator - creates all starting component objects */
bool ShowSaveImages::InitiateComponent() {
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
  std::unique_ptr<anantak::ImagesProcessorConfig> config =
      anantak::ReadProtobufFile<anantak::ImagesProcessorConfig>(config_file_path);
  if (config->camera_size() > 0) {
    num_cameras_ = config->camera_size();
  } else {
    LOG(ERROR) << "No cameras declared in the config file? num found = " << config->camera_size();
  }
  VLOG(1) << "Number of cameras: " << num_cameras_;
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
  images_subscriber_name_ = config->images_broadcaster_name();  // images broadcaster name
  VLOG(1) << "Command subscriber name = " << command_subscriber_name_;
  VLOG(1) << "Status subscriber name = " << status_subscriber_name_;
  VLOG(1) << "Images subscriber name = " << images_subscriber_name_;
  /* For each subscriber in config check if the program settings has specs for it. If so, create
   * the subscriber. Else write out an error and do not create a susbcriber */  
  std::vector<std::string> subscribers_to_be_created;
  subscribers_to_be_created.push_back(command_subscriber_name_);
  subscriber_type_[command_subscriber_name_] = kCommand;
  subscribers_to_be_created.push_back(status_subscriber_name_);
  subscriber_type_[status_subscriber_name_] = kStatus;
  subscribers_to_be_created.push_back(images_subscriber_name_);
  subscriber_type_[images_subscriber_name_] = kData;
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
  resume_command_str_ = "COMMAND " + component_name_ + " resume";
  pause_command_str_ = "COMMAND " + component_name_ + " pause";
  log_command_str_ = "COMMAND " + component_name_ + " log";
  show_command_str_ = "COMMAND " + component_name_ + " show";
  led_command_str_ = "COMMAND " + component_name_ + " led";
  tag_command_str_ = "COMMAND " + component_name_ + " tag";
  max_loop_frequency_ = 0.0f;
  
  // Turn over the config object to class storage
  config_= std::move(config);
  
  return true;
}

/** Handle commands coming from the commander subscriber */
bool ShowSaveImages::HandleCommand(StringPtrType cmd) {
  // exit command
  if (*cmd == exit_command_str_) exit_loop_ = true;
  return true;
}

/** Handle status queries coming from ProgramCommander */
bool ShowSaveImages::HandleStatusQuery(StringPtrType status_query) {
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
bool ShowSaveImages::StartLooping() {
  
  LOG(ERROR) << "Use StartProcessingImages() in place of StartLooping()";
  return false;
  
  // Get starting data from DataQueues to initiate the models
  
  anantak::Looper looper(loop_frequency_);
  while (!exit_loop_) {
    
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

/** Initiator - creates starting image processing objects */
bool ShowSaveImages::InitiateShowSaveImages() {
  
  // Which cameras are to be processed?
  process_camera_.resize(Ghumio_Images_Message_Num_Images, false);
  for (int i=0; i<config_->camera_size(); i++) {
    if (config_->camera(i).has_num()) {
      int cam_num = config_->camera(i).num();
      if (cam_num >= 0 && cam_num < Ghumio_Images_Message_Num_Images) {
        process_camera_[cam_num] = true;
        LOG(INFO) << "  Process camera# " << cam_num;
      }
    }
  }
  
  // Default settings
  run_beacon_detector_ = false;
  detect_leds_ = false;
  detect_tags_ = false;
  use_cv2cg_ = true;
  use_aprillab_ = false;
  apriltag_publisher_ = nullptr;
  blob_publisher_ = nullptr;
  apriltag_subject_ = "";
  blob_subject_ = "";
  show_images = true;
    
  // Does this image processor work as a Beacon Detector?
  if (config_->has_beacon_detector()) {
    LOG(INFO) << "Working as a beacon detector.";
    run_beacon_detector_ = true;
    
    const anantak::ImagesProcessorConfig::BeaconDetector& beacon_config = config_->beacon_detector();
    
    // Modify the settings to work as a beacon detector
    detect_tags_ = true;
    use_cv2cg_ = true; use_aprillab_ = false;  // By default cv2cg library is used
    
    // Check if tag publisher is present in the list of publishers
    if (publishers_map_.find(beacon_config.tag_publisher_name()) == publishers_map_.end()) {
      LOG(ERROR) << "Could not find publisher for apriltag publisher";
      return false;
    }
    
    apriltag_publisher_ = publishers_map_[beacon_config.tag_publisher_name()].get();
    apriltag_subject_ = beacon_config.tag_publisher_subject();
    
    if (beacon_config.has_blob_publisher_name()) {
      if (publishers_map_.find(beacon_config.blob_publisher_name()) == publishers_map_.end()) {
        LOG(ERROR) << "Could not find publisher for blob publisher";
        return false;
      }
      if (!beacon_config.has_blob_publisher_subject()) {
        LOG(ERROR) << "Could not find blob publisher subject but publisher name was given";
        return false;
      }
      detect_leds_ = true;
      blob_publisher_ = publishers_map_[beacon_config.blob_publisher_name()].get();
      blob_subject_ = beacon_config.blob_publisher_subject();      
    }
    
    // Prepare the AprilTag message
    //april_tag_message_
    
  }  // if this is a beacon detector
  
  CV_RED = cv::Scalar(0,0,255);
  CV_BLUE = cv::Scalar(255,0,0);
  CV_GREEN = cv::Scalar(0,255,0);
  CV_BLACK = cv::Scalar(0,0,0);
  CV_WHITE = cv::Scalar(255,255,255);
  CV_YELLOW = cv::Scalar(0,255,255);
  
  // April tag objects initialization
  // Create tagFamily
  std::string tagid(std::to_string(TAG_FAMILY)); //default Tag36h11
  april::tag::TagFamilyFactory::create(tagid, gTagFamilies);
  if (gTagFamilies.size() <= 0) {
    LOG(ERROR) << "Create April TagFamily failed. Exit.";
    return false;
  } else {
    LOG(INFO) << "April TagFamily created."; 
  }

  // Create tag detector
  gDetector = new april::tag::TagDetector(gTagFamilies);
  if(gDetector.empty()) {
    LOG(ERROR) << "Create April TagDetector failed. Exit.";
    return false;
  } else {
    LOG(INFO) << "April TagDetector created.";
  }
  
  // Initiate AprilTags C library
  tag_family_ = tag16h5_create(); //tag36h11_create();
  tag_family_->black_border = 1;
  tag_detector_ = apriltag_detector_create();
  apriltag_detector_add_family(tag_detector_, tag_family_);
  tag_detector_->quad_decimate = 1.0;
  tag_detector_->quad_sigma = 0.0;
  tag_detector_->nthreads = 4;
  tag_detector_->debug = 0;
  tag_detector_->refine_edges = 1;
  tag_detector_->refine_decode = 0;
  tag_detector_->refine_pose = 0;
  image_u8_  = image_u8_create(config_->image_width(), config_->image_height());
  
  return true;
}

/** Assemble status string */
ShowSaveImages::StringPtrType ShowSaveImages::AssembleStatusString() {
  StringPtrType status_str(new std::string(""));
  // Looping frequency
  {
    char buffer[100];
    snprintf(buffer, 100, "Arvl,Extr,Proc,Disp %ld %ld %ld %ld(us) Proc,Drop %ld %ld",
        msg_arrival_delay, msg_compiling_delay, msg_processing_delay, image_display_delay,
        processed_frames, dropped_frames);
    *status_str += buffer;
  }
  return status_str;
}

/** Start grabbing images from messages - listen to messages */
bool ShowSaveImages::StartProcessingImages() {
  
  long run_time = 0; //seconds
  long msg_processing_threshold = 10; // milli seconds
  bool exit_now = false;
  bool process_images = true;
  bool show_images_original  = false;
  int  show_image_location_x = 0;
  int  show_image_location_y = 0;
  bool publish_odometer_readings = false;
  bool show_motion_map = false;
  log_level = 0; // 0 = no logging, 1 = some logging, 2 = verbose, 3 = debug;
  
  LOG(INFO) << "  Run time  = " << run_time; 
  LOG(INFO) << "  Threshold = " << msg_processing_threshold;
  LOG(INFO) << "  Show images = " << show_images;
  LOG(INFO) << "  Show original images  = " << show_images_original;
  LOG(INFO) << "  Position  = " << show_image_location_x << ", " << show_image_location_y;
  
  // which cameras are to be used?
  int image_msg_part_num_front_left  = camera_num_front_left * 2 + 3;
  int image_msg_part_num_front_right = camera_num_front_right * 2 + 3;
  int image_msg_part_num_rear_left   = camera_num_rear_left * 2 + 3;
  int image_msg_part_num_rear_right  = camera_num_rear_right * 2 + 3;
  
  // Subscribers
  zmq::socket_t& subscriber = *subscriptions_map_[images_subscriber_name_];
  zmq::socket_t& command_subscriber = *subscriptions_map_[command_subscriber_name_];
  zmq::socket_t& status_subscriber = *subscriptions_map_[status_subscriber_name_];
  
  // initialize the poll set
  zmq::pollitem_t items[] = {
    { subscriber, 0, ZMQ_POLLIN, 0 },
    { command_subscriber, 0, ZMQ_POLLIN, 0 },
    { status_subscriber, 0, ZMQ_POLLIN, 0 },
  };
  
  long msg_num = 1;
  int msg_partnum = 0;
  
  // times to be measured
  msg_processing_threshold *= 1000; // msg_processing_threshold is given in msec, converting to usec
  long start_time = tic();
  long end_time = start_time + run_time*1000000;
  long current_time = start_time;
  long msg_build_start_time = 0;  // time the message started building - helps understand the uncertainty in the image times 
  long msg_sent_time = 0;         // time the message was sent taken from the footer
  long msg_arrival_time = 0;      // time message came in
  long msg_compile_time = 0;      // time message was deparsed
  long msg_processing_time = 0;   // time message processing was completed
  long msg_processing_time_last = 0;  // time last message was processed
  msg_arrival_delay = 0;     // time it took for message to arrive
  msg_compiling_delay = 0;   // time it took to get the images from the message
  long msg_age = 0;               // = msg_arrival_delay + msg_compiling_delay
  msg_processing_delay = 0;  // time it took to process the message
  image_display_delay = 0;
  //std::cout << "Start time =" << start_time << ", end time = " << end_time << std::endl;
  
  // message statistics
  processed_frames = 0;
  dropped_frames = 0;
  
  // image matrices
  image_width = config_->image_width();
  image_height = config_->image_height();
  image_size = cv::Size(image_width, image_height);
  captured_image_front_left.create(image_size, CV_8UC3);
  captured_image_front_right.create(image_size, CV_8UC3);
  captured_image_rear_left.create(image_size, CV_8UC3);
  captured_image_rear_right.create(image_size, CV_8UC3);
  image_front_left_gray.create(image_size, CV_8UC1);
  image_front_right_gray.create(image_size, CV_8UC1);
  image_rear_left_gray.create(image_size, CV_8UC1);
  image_rear_right_gray.create(image_size, CV_8UC1);
  
  // small images to display the captured images
  double image_shrink_factor = 0.5;
  int small_width  = (int) (image_shrink_factor * image_width );
  int small_height = (int) (image_shrink_factor * image_height);
  cv::Size small_size(small_width, small_height);
  cv::Mat small_front_left (small_size, CV_8UC3);
  cv::Mat small_front_right(small_size, CV_8UC3);
  cv::Mat small_rear_left  (small_size, CV_8UC3);
  cv::Mat small_rear_right (small_size, CV_8UC3);
  cv::Mat small_front_left_gray (small_size, CV_8UC1);
  cv::Mat small_front_right_gray(small_size, CV_8UC1);
  cv::Mat small_rear_left_gray  (small_size, CV_8UC1);
  cv::Mat small_rear_right_gray (small_size, CV_8UC1);
  
  // image display window
  std::string winname = "Ghumio Cameras";
  cv::Mat display_image(cv::Size(small_width*2, small_height*2), CV_8UC3);  // display image
  cv::Mat display_image_gray(cv::Size(small_width*2, small_height*2), CV_8UC1);  // display image
  if (show_images) {
    cv::namedWindow( winname, cv::WINDOW_AUTOSIZE ); // Create a window for display
    cv::moveWindow( winname, show_image_location_x, show_image_location_y); // move window to a new positon
  }
  
  // variables for recording
  int msg_num_cycle = 1000;
  std::string images_dir = "/home/manujnaman/Videos/"; //"./images/";
  std::string front_left_subdir  = "Front/";
  std::string front_right_subdir = "Front/";
  std::string rear_left_subdir   = "Rear/";
  std::string rear_right_subdir  = "Rear/";
  bool take_a_snap = false;
  bool take_many_snaps = false;
  bool is_first_snap = false;
  bool is_last_snap = false;
  int num_snaps = 0;
  int snap_cntr = 0;
  int many_snaps_start_msg_num = 0;
  int last_snap_msg_num = 0;
  int snaps_file_cntr = 0;
  std::fstream snaps_data_file;
  
  //  Main loop to process image messages
  long msg_num_last = 0;
  while (!exit_now) {
    //std::cout << "time = " << current_time << std::endl;
    
    zmq::message_t message;
    zmq::poll (items, 3, -1);
    
    if (items [0].revents & ZMQ_POLLIN) {
      subscriber.recv(&message);
      Ghumio_Images_Message_t msg;
      
      // process message
      msg_partnum++;
      int msg_size = message.size();
      //std::cout << "Message " << msg_num << " part " << msg_partnum << ", size = " << msg_size << std::endl;
      
      // parse the header
      if (msg_partnum == 1 && msg_size == Ghumio_Images_Message_Header_Size ) {
          msg_arrival_time = tic();
          char buffer[40];
          memcpy(buffer, ((char *) message.data())+24, 20+1);
          msg_build_start_time = strtol (buffer, NULL, 10);
          //std::cout << "msg build time = " << msg_build_start_time << std::endl;
          //std::cout << "msg arrival time = " << msg_arrival_time << std::endl;
          memcpy(msg.message_header, (char *) message.data(), Ghumio_Images_Message_Header_Size);
      }
      // image headers
      if (process_camera_[0] && msg_partnum == image_msg_part_num_front_left-1 && msg_size == Ghumio_Images_Message_Image_Header_Size )
          memcpy(msg.image_header[0], (char *) message.data(), Ghumio_Images_Message_Image_Header_Size);
      if (process_camera_[1] && msg_partnum == image_msg_part_num_front_right-1 && msg_size == Ghumio_Images_Message_Image_Header_Size )
          memcpy(msg.image_header[1], (char *) message.data(), Ghumio_Images_Message_Image_Header_Size);
      if (process_camera_[2] && msg_partnum == image_msg_part_num_rear_left-1 && msg_size == Ghumio_Images_Message_Image_Header_Size )
          memcpy(msg.image_header[2], (char *) message.data(), Ghumio_Images_Message_Image_Header_Size);
      if (process_camera_[3] && msg_partnum == image_msg_part_num_rear_right-1 && msg_size == Ghumio_Images_Message_Image_Header_Size )
          memcpy(msg.image_header[3], (char *) message.data(), Ghumio_Images_Message_Image_Header_Size);
      // images
      if (process_camera_[0] && msg_partnum == image_msg_part_num_front_left && msg_size == Ghumio_Images_Message_Image_Size ) {
          cv::Mat temp_image(image_height, image_width, CV_8UC3, message.data(), Ghumio_Images_Message_Image_Stride);
          temp_image.copyTo(captured_image_front_left);
          memcpy(msg.image_data[0], (unsigned char *) message.data(), Ghumio_Images_Message_Image_Size);
      }
      if (process_camera_[1] && msg_partnum == image_msg_part_num_front_right && msg_size == Ghumio_Images_Message_Image_Size ) {
          cv::Mat temp_image(image_height, image_width, CV_8UC3, message.data(), Ghumio_Images_Message_Image_Stride);
          temp_image.copyTo(captured_image_front_right);
          memcpy(msg.image_data[1], (unsigned char *) message.data(), Ghumio_Images_Message_Image_Size);
      }
      if (process_camera_[2] && msg_partnum == image_msg_part_num_rear_left && msg_size == Ghumio_Images_Message_Image_Size ) {
          cv::Mat temp_image(image_height, image_width, CV_8UC3, message.data(), Ghumio_Images_Message_Image_Stride);
          temp_image.copyTo(captured_image_rear_left);
          memcpy(msg.image_data[2], (unsigned char *) message.data(), Ghumio_Images_Message_Image_Size);
      }
      if (process_camera_[3] && msg_partnum == image_msg_part_num_rear_right && msg_size == Ghumio_Images_Message_Image_Size ) {
          cv::Mat temp_image(image_height, image_width, CV_8UC3, message.data(), Ghumio_Images_Message_Image_Stride);
          temp_image.copyTo(captured_image_rear_right);
          memcpy(msg.image_data[3], (unsigned char *) message.data(), Ghumio_Images_Message_Image_Size);
      }
      // parse the footer 
      if (msg_size == Ghumio_Images_Message_Footer_Size) {
          char buffer[40];
          memcpy(buffer, ((char *) message.data())+7, 20+1);
          msg_sent_time = strtol (buffer, NULL, 10);
          //std::cout << "msg sent time = " << msg_sent_time << std::endl;
          memcpy(msg.message_footer, (char *) message.data(), Ghumio_Images_Message_Footer_Size);
      }
      
      // check if this is the end of the multi-part message
      int64_t more = 0;
      size_t more_size = sizeof(more);
      subscriber.getsockopt(ZMQ_RCVMORE, &more, &more_size);
      if (!more) {
        // end of message. Process the message.
        msg_partnum = 0;    // reset back to 0
        msg_num = (msg_num + 1) % msg_num_cycle;
        msg_compile_time = tic();
        
        msg_arrival_delay = msg_arrival_time - msg_sent_time;
        msg_compiling_delay = msg_compile_time - msg_arrival_time;
        msg_age = msg_compile_time - msg_sent_time;
        
        // process the message only if it was sent less than msg_processing_threshold ago
        if (msg_age < msg_processing_threshold) {
          processed_frames++;
          
          // Image processing begin
          if (process_images) {
              //// image processing code begin ////
              
              // Convert to gray scale from color
              if (detect_leds_ || detect_tags_) {
                for (int i_cam=0; i_cam<Ghumio_Images_Message_Num_Images; i_cam++) {
                  if (process_camera_[i_cam]) {
                    if (i_cam==0) {
                      cv::cvtColor(captured_image_front_left, image_front_left_gray, CV_RGB2GRAY);
                    }
                    else if (i_cam==1) {
                      cv::cvtColor(captured_image_front_right, image_front_right_gray, CV_RGB2GRAY);
                    }
                    else if (i_cam==2) {
                      cv::cvtColor(captured_image_rear_left, image_rear_left_gray, CV_RGB2GRAY);
                    }
                    else if (i_cam==3) {
                      cv::cvtColor(captured_image_rear_right, image_rear_right_gray, CV_RGB2GRAY);
                    }
                  }
                }
              }
              
              // Detect April tags
              if (detect_tags_) {
                for (int i_cam=0; i_cam<Ghumio_Images_Message_Num_Images; i_cam++) {
                  if (process_camera_[i_cam]) {
                    if (i_cam==0) {
                      DetectTags(captured_image_front_left, image_front_left_gray, msg_sent_time, i_cam);
                    }
                    else if (i_cam==1) {
                      DetectTags(captured_image_front_right, image_front_right_gray, msg_sent_time, i_cam);
                    }
                    else if (i_cam==2) {
                      DetectTags(captured_image_rear_left, image_rear_left_gray, msg_sent_time, i_cam);
                    }
                    else if (i_cam==3) {
                      DetectTags(captured_image_rear_right, image_rear_right_gray, msg_sent_time, i_cam);
                    }
                  }
                }
              }
              
              // Detect LEDs
              if (detect_leds_) {
                for (int i_cam=0; i_cam<Ghumio_Images_Message_Num_Images; i_cam++) {
                  if (process_camera_[i_cam]) {
                    if (i_cam==0) {
                      DetectLeds(captured_image_front_left, image_front_left_gray);
                    }
                    else if (i_cam==1) {
                      DetectLeds(captured_image_front_right, image_front_right_gray);
                    }
                    else if (i_cam==2) {
                      DetectLeds(captured_image_rear_left, image_rear_left_gray);
                    }
                    else if (i_cam==3) {
                      DetectLeds(captured_image_rear_right, image_rear_right_gray);
                    }
                  }
                }
              }
              
              // Take a snap
              if (take_a_snap) {
                  
                  std::cout << "  Taking one snap for each camera and saving to directory = " << images_dir << std::endl;
                  // save the four camera images as png files
                  std::stringstream fl_fnss, fr_fnss, rl_fnss, rr_fnss; 
                  fl_fnss << images_dir << "Front_Left_.png";
                  cv::imwrite( fl_fnss.str(), captured_image_front_left );
                  fr_fnss << images_dir << "Front_Right_.png";
                  cv::imwrite( fr_fnss.str(), captured_image_front_right );
                  rl_fnss << images_dir << "Rear_Left_.png";
                  cv::imwrite( rl_fnss.str(), captured_image_rear_left );
                  rr_fnss << images_dir << "Rear_Right_.png";
                  cv::imwrite( rr_fnss.str(), captured_image_rear_right );
                  std::cout << "  Done taking a snap for each camera." << std::endl << std::endl;
                  
                  // reset
                  take_a_snap = false;
              }
              
              // Record video
              if (take_many_snaps) {
                  
                  is_first_snap = (snap_cntr == 0);
                  is_last_snap =  (snap_cntr == num_snaps-1);
                  
                  if (is_first_snap) {
                      
                      many_snaps_start_msg_num = msg_num;
                      std::cout << "  Starting to take snaps for video" << std::endl;
                      
                      std::stringstream data_fnss;
                      data_fnss << images_dir << "Video_Data.txt";
                      std::cout << "    writing data to file = " << data_fnss.str() << std::endl;
                      snaps_data_file.open( data_fnss.str().c_str(), std::ios::out );
                  }
                  
                  // write to the file
                  std::stringstream fl_fnss, fr_fnss, rl_fnss, rr_fnss; 
                  fl_fnss << images_dir << front_left_subdir << "Front_Left_" << snap_cntr << ".png";
                  cv::imwrite( fl_fnss.str(), captured_image_front_left );
                  fr_fnss << images_dir << front_right_subdir << "Front_Right_" << snap_cntr << ".png";
                  cv::imwrite( fr_fnss.str(), captured_image_front_right );
                  rl_fnss << images_dir << rear_left_subdir << "Rear_Left_" << snap_cntr << ".png";
                  cv::imwrite( rl_fnss.str(), captured_image_rear_left );
                  rr_fnss << images_dir << rear_right_subdir << "Rear_Right_" << snap_cntr << ".png";
                  cv::imwrite( rr_fnss.str(), captured_image_rear_right );
                  std::cout << "    snap num = " << snap_cntr << ", msg num = " << msg_num << ", missed = " << msg_num - last_snap_msg_num << std::endl;
                  last_snap_msg_num = msg_num;
                  
                  snaps_data_file << "image_num = " << snap_cntr << ", msg_start_time = " << msg_build_start_time << ", msg_sent_time = " << msg_sent_time << ", msg_num = " << msg_num << std::endl;
                  //std::cout << "  written msg " << snap_cntr << ", Bad = " << snaps_data_file.bad() << std::endl;
                  
                  ++snap_cntr;
                  
                  if (is_last_snap) {
                      
                      snaps_data_file.close();
                      std::cout << "    closed data file" << std::endl;
                      std::cout << "  Done taking snaps for video" << std::endl << std::endl;
                      
                      // reset
                      take_many_snaps = false;
                      is_first_snap = false;
                      is_last_snap = false;
                      num_snaps = 0;
                      snap_cntr = 0;
                      many_snaps_start_msg_num = 0;
                      last_snap_msg_num = 0;
                  }
                  
              }
              
              //// image processing code end ////
          
              msg_processing_time = tic();
              msg_processing_delay = msg_processing_time - msg_compile_time;
              int processing_frame_rate = int(1000000.00 / (msg_processing_time - msg_processing_time_last)); 
              msg_processing_time_last = msg_processing_time;
              if (log_level >= 1) std::cout << "  Msg = " << msg_num << ", Time = " << msg_processing_time << ", Processing frame rate = " << processing_frame_rate << std::endl;
          
              // show images if asked
              if (show_images) {
                
                std::string processing_frame_rate_str = std::to_string(processing_frame_rate);
                processing_frame_rate_str.append(" Hz");
                if (detect_leds_) processing_frame_rate_str.append(" LEDs");
                if (detect_tags_ && use_cv2cg_) processing_frame_rate_str.append(" Tags cv2cg");
                else if (detect_tags_ && use_aprillab_) processing_frame_rate_str.append(" Tags aprillab");
                if (take_many_snaps) processing_frame_rate_str.append(" Recording");
                  
                if (false) {
                  cv::resize(image_front_left_gray,  small_front_left_gray,  small_front_left.size(),  0, 0, CV_INTER_AREA);
                  cv::resize(image_front_right_gray, small_front_right_gray, small_front_right.size(), 0, 0, CV_INTER_AREA);
                  cv::resize(image_rear_left_gray,  small_rear_left_gray,  small_rear_left.size(),  0, 0, CV_INTER_AREA);
                  cv::resize(image_rear_right_gray, small_rear_right_gray, small_rear_right.size(), 0, 0, CV_INTER_AREA);
                  small_front_left_gray.copyTo(  display_image_gray(cv::Rect(0, 0, small_width, small_height)) );
                  small_front_right_gray.copyTo( display_image_gray(cv::Rect(small_width, 0, small_width, small_height)) );
                  small_rear_left_gray.copyTo(  display_image_gray(cv::Rect(0, small_height, small_width, small_height)) );
                  small_rear_right_gray.copyTo( display_image_gray(cv::Rect(small_width, small_height, small_width, small_height)) );
                  // add some information on top
                  cv::putText( display_image_gray, processing_frame_rate_str, cv::Point(0,10), CV_FONT_NORMAL, 0.4, cv::Scalar(255,255,255), 1);
                  cv::imshow( winname, display_image_gray );
                  cv::waitKey(1);
                } else {
                  // add resized original images on the display
                  cv::resize(captured_image_front_left,  small_front_left,  small_front_left.size(),  0, 0, CV_INTER_AREA);
                  cv::resize(captured_image_front_right, small_front_right, small_front_right.size(), 0, 0, CV_INTER_AREA);
                  cv::resize(captured_image_rear_left,  small_rear_left,  small_rear_left.size(),  0, 0, CV_INTER_AREA);
                  cv::resize(captured_image_rear_right, small_rear_right, small_rear_right.size(), 0, 0, CV_INTER_AREA);
                  small_front_left.copyTo(  display_image(cv::Rect(0, 0, small_width, small_height)) );
                  small_front_right.copyTo( display_image(cv::Rect(small_width, 0, small_width, small_height)) );
                  small_rear_left.copyTo(  display_image(cv::Rect(0, small_height, small_width, small_height)) );
                  small_rear_right.copyTo( display_image(cv::Rect(small_width, small_height, small_width, small_height)) );
                  // add some information on top
                  cv::putText( display_image, processing_frame_rate_str, cv::Point(0,10), CV_FONT_NORMAL, 0.4, cv::Scalar(0,0,255), 1);
                  cv::imshow( winname, display_image );
                  cv::waitKey(1);
                }
                
              }
              
              // ending image showing
              
              // report timing information
              image_display_delay = tic() - msg_processing_time;
              if (log_level >= 2) std::cout << "  Delays: arrival, compiling, processing, display = "
                  <<  msg_arrival_delay << ", " << msg_compiling_delay << ", "
                  << msg_processing_delay << ", " << image_display_delay << " usec" << std::endl;
              if (log_level >= 2) std::cout << "  Frames: processed, dropped = " << processed_frames << ", " << dropped_frames << std::endl;
              //std::cout << "  Image take between " << msg_sent_time - msg_build_start_time << " usec" << std::endl;
              if (log_level >= 1) std::cout << "\n";
              msg_num_last = msg_num;
              
          } // process_images
            
        } // msg_age < msg_processing_threshold
        else {
          // this message is skipped. move on to the next message.
          dropped_frames++;
        }
          
      } // if (!more) - meaning this is the end of the multi-part message
      
    }  // if (items [0].revents & ZMQ_POLLIN)
    
    if (items [1].revents & ZMQ_POLLIN) {
        command_subscriber.recv(&message);
        int msg_size = message.size();
        msg_size = std::min(msg_size, 256);
        
        char cmnd_msg[256];
        memset(cmnd_msg, 0, sizeof(cmnd_msg));
        
        strncpy( cmnd_msg, ((char *) message.data()), msg_size );
        std::cout << "  Command = " << cmnd_msg << std::endl;
        
        char argument[256]; //maximum size of a command is 256 characters
        int arglength = 0;
        
        bool done = false;
        
        if (!done) {
        strcpy (argument, exit_command_str_.c_str());
        arglength  = strlen(argument);
        if (strncmp(cmnd_msg, argument, arglength)==0) {
            exit_now = true;
            done = true;
        }
        memset(argument, 0, sizeof(argument));
        }
        
        if (!done) {
        strcpy (argument, pause_command_str_.c_str());
        arglength  = strlen(argument);
        if (strncmp(cmnd_msg, argument, arglength)==0) {
            process_images = !process_images;
            std::cout << "    Set process_images = " << process_images << std::endl;
            done = true;
        }
        memset(argument, 0, sizeof(argument));
        }
        
        if (!done) {
        strcpy (argument, resume_command_str_.c_str());
        arglength  = strlen(argument);
        if (strncmp(cmnd_msg, argument, arglength)==0) {
            process_images = true;
            done = true;
        }
        memset(argument, 0, sizeof(argument));
        }
        
        if (!done) {
        strcpy (argument, led_command_str_.c_str());
        arglength  = strlen(argument);
        if (strncmp(cmnd_msg, argument, arglength)==0) {
            detect_leds_ = !detect_leds_;
            done = true;
        }
        memset(argument, 0, sizeof(argument));
        }
        
        if (!done) {
        strcpy (argument, tag_command_str_.c_str());
        arglength  = strlen(argument);
        if (strncmp(cmnd_msg, argument, arglength)==0) {
            if (detect_tags_ && use_cv2cg_) {detect_tags_=true; use_cv2cg_=false; use_aprillab_=true;}
            else if (detect_tags_ && use_aprillab_) {detect_tags_=false; use_cv2cg_=true; use_aprillab_=false;}
            else if (!detect_tags_) {detect_tags_=true; use_cv2cg_=true; use_aprillab_=false;}
            done = true;
        }
        memset(argument, 0, sizeof(argument));
        }
        
        if (!done) {
        strcpy (argument, log_command_str_.c_str());
        arglength  = strlen(argument);
        if (strncmp(cmnd_msg, argument, arglength)==0) {
            if (log_level == 1) log_level = 2;
            else if (log_level == 2) log_level = 0;
            else if (log_level == 0) log_level = 1;
            std::cout << "    Set log level = " << log_level << std::endl;
            done = true;
        }
        memset(argument, 0, sizeof(argument));
        }
        
        if (!done) {
        strcpy (argument, show_command_str_.c_str());
        arglength  = strlen(argument);
        if (strncmp(cmnd_msg, argument, arglength)==0) {
          if (!show_images) {
            show_images = true;
            cv::namedWindow( winname, cv::WINDOW_AUTOSIZE ); // Create a window for display
            cv::moveWindow( winname, show_image_location_x, show_image_location_y); // move window to a new positon
          } else {
            show_images = false;
            cv::destroyWindow(winname); // destroy the display window
            cv::waitKey(10);
            cv::destroyWindow(winname); // destroy the display window
            cv::waitKey(10);
            cv::destroyWindow(winname); // destroy the display window
            cv::waitKey(10);
            cv::destroyWindow(winname); // destroy the display window
            cv::waitKey(10);
            cv::destroyWindow(winname); // destroy the display window
            cv::waitKey(10);                
          }
          std::cout << "    Set show_images = " << show_images << std::endl;
          done = true;
        }
        memset(argument, 0, sizeof(argument));
        }
        
        if (!done) {
        strcpy (argument, "COMMAND IS show_original");
        arglength  = strlen(argument);
        if (strncmp(cmnd_msg, argument, arglength)==0) {
            show_images_original = true;
            done = true;
        }
        memset(argument, 0, sizeof(argument));
        }
        
        if (!done) {
        strcpy (argument, "COMMAND IS hide_original");
        arglength  = strlen(argument);
        if (strncmp(cmnd_msg, argument, arglength)==0) {
            show_images_original = false;
            done = true;
        }
        memset(argument, 0, sizeof(argument));
        }
            
        if (!done) {
        strcpy (argument, "COMMAND IS snap");
        arglength  = strlen(argument);
        if (strncmp(cmnd_msg, argument, arglength)==0) {
            take_a_snap = true;
            done = true;
        }
        memset(argument, 0, sizeof(argument));
        
        strcpy (argument, "COMMAND IS video30");
        arglength  = strlen(argument);
        if (strncmp(cmnd_msg, argument, arglength)==0) {
            take_many_snaps = true;
            num_snaps = 30;
            snap_cntr = 0;
            done = true;
        }
        memset(argument, 0, sizeof(argument));
        
        strcpy (argument, "COMMAND IS video150");
        arglength  = strlen(argument);
        if (strncmp(cmnd_msg, argument, arglength)==0) {
            take_many_snaps = true;
            num_snaps = 150;
            snap_cntr = 0;
            done = true;
        }
        memset(argument, 0, sizeof(argument));
        
        strcpy (argument, "COMMAND IS video900");
        arglength  = strlen(argument);
        if (strncmp(cmnd_msg, argument, arglength)==0) {
            take_many_snaps = true;
            num_snaps = 900;
            snap_cntr = 0;
            done = true;
        }
        memset(argument, 0, sizeof(argument));
        
        strcpy (argument, "COMMAND IS video4500");
        arglength  = strlen(argument);
        if (strncmp(cmnd_msg, argument, arglength)==0) {
            take_many_snaps = true;
            num_snaps = 4500;
            snap_cntr = 0;
            done = true;
        }
        memset(argument, 0, sizeof(argument));
        }
        
    } // if (items [1].revents & ZMQ_POLLIN) 
    
    if (items [2].revents & ZMQ_POLLIN) {        
      status_subscriber.recv(&message);
      size_t msg_size = message.size();
      const char* msg_data_ptr = static_cast<char*>(message.data());  // cast the void* to char*
      
      /** Handle status queries */
      std::unique_ptr<std::string> msg_str_ptr(       // copy the entire message
          new std::string(msg_data_ptr, msg_size));   // copy
      VLOG(3) << "Status query: " << *msg_str_ptr;
      HandleStatusQuery(std::move(msg_str_ptr));
    }
    
    current_time = tic();
    
    if (!exit_now && run_time > 0) {
        if (current_time > end_time) exit_now = true;
    }
  
  }  // while (!exit_now)
  
  return true;
}

/** Image processing methods **/

/* Detect Infrared LEDs in a open cv matrix
 * This is taken from RPG Lab's monocular pose estimator available here:
 *  https://github.com/uzh-rpg/rpg_monocular_pose_estimator
 * Some modifications are made to fit our needs. So credit should be given to Karl Schwabe at RPG
 */
bool ShowSaveImages::DetectLeds(cv::Mat &color_image, const cv::Mat &gray_image) {
  
  const int threshold_value = 220;
  const double gaussian_sigma = 1.;
  const double min_blob_area = 10.;
  const double max_blob_area = 400.;
  const double max_width_height_distortion = 0.7;
  const double max_circular_distortion = 0.7;
  
  // Threshold the image
  cv::Mat bw_image(gray_image);
  //cv::threshold(image, bwImage, threshold_value, 255, cv::THRESH_BINARY);
  cv::threshold(bw_image.clone(), bw_image, threshold_value, 255, cv::THRESH_TOZERO);
  
  // Gaussian blur the image
  //cv::Mat gaussian_image;
  cv::Size ksize; // Gaussian kernel size. If equal to zero, then the kerenl size is computed from the sigma
  ksize.width = 0;
  ksize.height = 0;
  cv::GaussianBlur(bw_image.clone(), bw_image, ksize, gaussian_sigma, gaussian_sigma, cv::BORDER_DEFAULT);
  
  // Find all contours
  std::vector<std::vector<cv::Point> > contours;
  cv::findContours(bw_image.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

  unsigned int numPoints = 0; // Counter for the number of detected LEDs

  // Vector for containing the detected points that will be undistorted later
  std::vector<cv::Point2f> distorted_points;
  std::vector<double> areas;

  // Identify the blobs in the image
  for (unsigned i = 0; i < contours.size(); i++) {
    double area = cv::contourArea(contours[i]); // Blob area
    cv::Rect rect = cv::boundingRect(contours[i]); // Bounding box
    double radius = (rect.width + rect.height) / 4.; // Average radius
    
    cv::Moments mu;
    mu = cv::moments(contours[i], false);
    cv::Point2f mc;
    mc = cv::Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00); // + cv::Point2f(ROI.x, ROI.y);
    
    // Look for round shaped blobs of the correct size
    if (area >= min_blob_area && area <= max_blob_area
        && std::abs(1 - std::min((double)rect.width / (double)rect.height, (double)rect.height / (double)rect.width))
            <= max_width_height_distortion
        && std::abs(1 - (area / (CV_PI * std::pow(rect.width / 2, 2)))) <= max_circular_distortion
        && std::abs(1 - (area / (CV_PI * std::pow(rect.height / 2, 2)))) <= max_circular_distortion)
    {
      areas.push_back(area);
      distorted_points.push_back(mc);
      numPoints++;
    }
  }
  //VLOG(1) << "Detected LED points = " << numPoints;
  
  int max_points = 10;
  double min_area = 0;
  
  if (numPoints>0) {
    // If more than max_points, calculate the min area
    if (numPoints > max_points) {
      // Sort areas
      std::vector<double> areas_copy = areas;
      std::sort(areas_copy.begin(), areas_copy.end());
      min_area = areas_copy[areas_copy.size() - max_points];
    }
    
    // Copy the gray image to color image
    //cv::cvtColor(bw_image, color_image, CV_GRAY2RGB);
    
    // Draw the detected points on color image
    if (show_images) {
      for (unsigned i = 0; i < numPoints; i++) {
        if (areas[i] >= min_area) {
          cv::circle(color_image, distorted_points[i], 5, CV_RED, 2);
        }
      }
    }
  }
  
  return true;
}

/* Detect Tags
 * This uses cv2cg library from Simba Forrest.
 */
bool ShowSaveImages::DetectTags(cv::Mat &color_image, cv::Mat &gray_image,
    const int64_t& message_sent_time, const int32_t& cam_num) {
  
  // Detect using cv2cg library
  if (use_cv2cg_) {
    std::vector<april::tag::TagDetection> detections;
    gDetector->process(gray_image, detections);
    int num_tags_detected = (int) detections.size();
    if (log_level>0) std::cout << "Num tags detected = " << num_tags_detected << std::endl;
    
    unsigned num_seen = 0;
    for (int i=0; i<num_tags_detected; ++i) {
      april::tag::TagDetection& dd = detections[i];
      // Only best detections are chosen
      if (dd.hammingDistance == 0) {
        num_seen++;
        
        // Draw the detections on the color image
        if (show_images) {
          int tag_id = dd.id;
          cv::Point2d p0(dd.p[0][0], dd.p[0][1]);
          cv::Point2d p1(dd.p[1][0], dd.p[1][1]);
          cv::Point2d p2(dd.p[2][0], dd.p[2][1]);
          cv::Point2d p3(dd.p[3][0], dd.p[3][1]);
          cv::line(color_image, p0, p1, CV_YELLOW, 2);
          cv::line(color_image, p1, p2, CV_YELLOW, 2);
          cv::line(color_image, p2, p3, CV_YELLOW, 2);
          cv::line(color_image, p3, p0, CV_YELLOW, 2);
          cv::line(color_image, p0, p2, CV_YELLOW, 2);
          cv::line(color_image, p1, p3, CV_YELLOW, 2);
          cv::circle(color_image, p0, 3, CV_GREEN, 2);
          cv::circle(color_image, p1, 3, CV_RED, 2);
          cv::circle(color_image, p2, 3, CV_BLUE, 2);
          cv::circle(color_image, p3, 3, CV_BLACK, 2);
          cv::putText(color_image, dd.toString(), cv::Point(dd.cxy[0],dd.cxy[1]),
              CV_FONT_NORMAL, .5, CV_WHITE, 4);
          cv::putText(color_image, dd.toString(), cv::Point(dd.cxy[0],dd.cxy[1]),
              CV_FONT_NORMAL, .5, CV_BLUE, 1);
        } // show_image
        
      } // hamming filter
    }
    
    // Build and transmit a message if any tags were seen, otherwise remain silent.
    if (run_beacon_detector_ && num_seen>0) {
      BuildAndSendAprilTagMessage(detections, message_sent_time, cam_num);
    }
  }
  
  // Detect tags using Apriltag C library from Aprillab
  if (use_aprillab_) {
    
    // Extract the image from cv to image_u8
    CopyGrayCvMatToImageU8(gray_image, image_u8_);
    
    // Detect
    zarray_t *detections = apriltag_detector_detect(tag_detector_, image_u8_);
    
    if (show_images) {
      for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
        
        // Only best quality tags are taken
        if (det->hamming == 0) {
          //printf("detection %3d: id (%2dx%2d)-%-4d, hamming %d, goodness %8.3f, margin %8.3f\n",
          //    i, det->family->d*det->family->d, det->family->h, det->id, det->hamming, det->goodness, det->decision_margin);
          
          // Draw the detections on the color image
          std::string tag_id(std::to_string(det->id));
          cv::Point2d p0(det->p[0][0], det->p[0][1]);
          cv::Point2d p1(det->p[1][0], det->p[1][1]);
          cv::Point2d p2(det->p[2][0], det->p[2][1]);
          cv::Point2d p3(det->p[3][0], det->p[3][1]);
          cv::line(color_image, p0, p1, CV_YELLOW, 2);
          cv::line(color_image, p1, p2, CV_YELLOW, 2);
          cv::line(color_image, p2, p3, CV_YELLOW, 2);
          cv::line(color_image, p3, p0, CV_YELLOW, 2);
          cv::line(color_image, p0, p2, CV_YELLOW, 2);
          cv::line(color_image, p1, p3, CV_YELLOW, 2);
          cv::circle(color_image, p0, 3, CV_GREEN, 2);
          cv::circle(color_image, p1, 3, CV_RED, 2);
          cv::circle(color_image, p2, 3, CV_BLUE, 2);
          cv::circle(color_image, p3, 3, CV_BLACK, 2);
          cv::putText(color_image, tag_id, cv::Point(det->c[0],det->c[1]),
              CV_FONT_NORMAL, .5, CV_WHITE, 4);
          cv::putText(color_image, tag_id, cv::Point(det->c[0],det->c[1]),
              CV_FONT_NORMAL, .5, CV_BLUE, 1);
        }
      }
    }
    apriltag_detections_destroy(detections);
  }
  
  return true;
}

// Copy the gray image data to Imageu8 used by AprilTags C library
bool ShowSaveImages::CopyGrayCvMatToImageU8(cv::Mat &gray_image, image_u8_t *im) {
  
  for (int32_t v = 0; v < im->height; v++) {
    uchar* image_row_p  = gray_image.ptr(v);
    for (int32_t u = 0; u < im->width; u++) {
      im->buf[v*im->stride + u] = (uint8_t) image_row_p[u];
    }
  }

  return true;
}

// Build an AprilTag message and send it on the publisher
bool ShowSaveImages::BuildAndSendAprilTagMessage(const std::vector<april::tag::TagDetection>& detections,
    const int64_t& message_sent_time, const int32_t& cam_num) {
  
  // No point of doing all the work if publisher is not available
  if (!apriltag_publisher_) {
    LOG(ERROR) << "AprilTag publisher is nullptr, can not publish";
    return false;
  }
  
  // Clear up the storage for the message
  april_tag_message_.Clear();
  anantak::HeaderMsg* hdr_msg = april_tag_message_.mutable_header();
  anantak::AprilTagMessage* april_msg = april_tag_message_.mutable_april_msg();
  
  // Build header message
  hdr_msg->set_timestamp(message_sent_time);
  hdr_msg->set_type(APRIL_MESSAGE_TYPE);
  hdr_msg->set_recieve_timestamp(message_sent_time);
  hdr_msg->set_send_timestamp(message_sent_time);
  
  // Build Apriltag message
  april_msg->set_camera_num(cam_num);
  int num_tags_detected = (int) detections.size();
  for (int i=0; i<num_tags_detected; ++i) {
    const april::tag::TagDetection& dd = detections[i];
    if (dd.hammingDistance > 0) continue;
    april_msg->add_tag_id(dd.toString());
    april_msg->add_u_1(float(dd.p[0][0]));
    april_msg->add_v_1(float(dd.p[0][1]));
    april_msg->add_u_2(float(dd.p[1][0]));
    april_msg->add_v_2(float(dd.p[1][1]));
    april_msg->add_u_3(float(dd.p[2][0]));
    april_msg->add_v_3(float(dd.p[2][1]));
    april_msg->add_u_4(float(dd.p[3][0]));
    april_msg->add_v_4(float(dd.p[3][1]));
  }
  
  if (log_level>0) std::cout << "Built an Apriltag message with num tags = " << april_msg->tag_id_size()
      << " cam " << cam_num << " time " << message_sent_time << std::endl;
  
  // Transmit the message over ZMQ
  return SendSensorMessage(april_tag_message_, apriltag_subject_, apriltag_publisher_);
}

// Send a sensor message over the bus. This could be a library level function
bool ShowSaveImages::SendSensorMessage(const anantak::SensorMsg& sensor_msg,
    const std::string& subject, zmq::socket_t* publisher) {
  // Get message size
  size_t message_length = sensor_msg.ByteSize();
  size_t subject_length = subject.length();
  // Allocate a message of size = address size + message size
  zmq::message_t zmsg(subject_length + message_length);
  // Copy address to the message
  void* zmsg_ptr = zmsg.data();
  memcpy(zmsg_ptr, subject.data(), subject_length);
  // Increment pointer to end of subject
  zmsg_ptr = static_cast<char*>(zmsg_ptr) + subject_length;
  // Serialize the message to the stream
  sensor_msg.SerializeToArray(zmsg_ptr, message_length);
  bool sent_ok = publisher->send(zmsg, ZMQ_DONTWAIT);
  if (!sent_ok) {
    LOG(ERROR) << "Message was not sent";
  }
  return true;
}


} // namespace anantak
