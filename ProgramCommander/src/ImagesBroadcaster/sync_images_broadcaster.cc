/**
 *  Sync Images Broadcaster
 *
 *  Implementation of Sync Images Broadcaster
 */

/** Header file include */
#include "ImagesBroadcaster/sync_images_broadcaster.h"

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
SyncImagesBroadcaster::SyncImagesBroadcaster(std::string programs_setup_filename,
    std::string component_name) {
  programs_setup_config_filename_ = programs_setup_filename;
  component_name_ = component_name;  
  is_initiated_ = false;
  if (!Initiate()) LOG(ERROR) << "Could not initiate SyncImagesBroadcaster.";
}

/** Destructor - all members are self destructing */
SyncImagesBroadcaster::~SyncImagesBroadcaster() {
  VLOG(1) << "SyncImagesBroadcaster shutdown.";
  VLOG(1) << "ZMQ transport and all pub/subs will destruct automatically.";
  VLOG(1) << "Shutting down cameras.";
  fc2Error error;
  for (int i=0; i<num_cameras_attached_; i++ ) {
    // stop capture and close camera connections
    printf("Stopping capture for camera %d\n", i);
    error = fc2StopCapture(pg_contexts_[i]);
    if (error != FC2_ERROR_OK) {
      printf("Error in fc2StopCapture: %d\n", error);
    }
    printf("Destroying camera context for camera %d\n", i);
    error = fc2DestroyContext(pg_contexts_[i]);
    if (error != FC2_ERROR_OK) {
      printf("Error in fc2DestroyContext: %d\n", error);
    }
  } // for each camera
}

/** Initiator - creates all starting objects */
bool SyncImagesBroadcaster::Initiate() {
  if (!InitiateComponent()) {
    LOG(ERROR) << "Could not initiate SyncImagesBroadcaster component objects";
    return false;
  }
  if (!InitiateSyncImagesBroadcaster()) {
    LOG(ERROR) << "Could not initiate SyncImagesBroadcaster imaging objects";
    return false;    
  }
  // Initiation succeeded
  is_initiated_ = true;
  return true;
}

/** Component Initiator - creates all starting component objects */
bool SyncImagesBroadcaster::InitiateComponent() {
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
  std::unique_ptr<anantak::ImagesBroadcasterConfig> config =
      anantak::ReadProtobufFile<anantak::ImagesBroadcasterConfig>(config_file_path);
  if (config->pg_camera_size() > 0) {
    num_cameras_attached_ = config->pg_camera_size();
  } else {
    LOG(ERROR) << "No cameras declared in the config file? num found = " << config->pg_camera_size();
  }
  VLOG(1) << "Number of cameras: " << num_cameras_attached_;
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
  //for (int i=0; i<config->data_queue_size(); i++) {
  //  const anantak::FilterConfig::DataQueue& data_queue = config->data_queue(i);
  //  subscribers_to_be_created.push_back(data_queue.name());
  //  subscriber_type_[data_queue.name()] = kDataQueue;
  //  data_queues_.push_back(data_queue.name());
  ///}
  //data_queues_.shrink_to_fit(); // release any extra space
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
  max_loop_frequency_ = 0.0f;
  
  // Turn over the config object to class storage
  ib_config_= std::move(config);
  
  return true;
}

/** Initiator - creates all starting imaging objects */
bool SyncImagesBroadcaster::InitiateSyncImagesBroadcaster() {
  
  // Flycapture library information
  PrintPGBuildInfo();
  
  // Resize contexts
  pg_contexts_.resize(num_cameras_attached_, fc2Context());
  pg_guids_.resize(num_cameras_attached_, fc2PGRGuid());
  
  // Initiate each camera
  if (!InitiatePGCameras()) {
    LOG(ERROR) << "Could not initiate attached cameras. Exiting.";
    return false;
  }
  
  return true;
}

/** Handle commands coming from the commander subscriber */
bool SyncImagesBroadcaster::HandleCommand(StringPtrType cmd) {
  // exit command
  if (*cmd == exit_command_str_) exit_loop_ = true;
  return true;
}

/** Handle status queries coming from ProgramCommander */
bool SyncImagesBroadcaster::HandleStatusQuery(StringPtrType status_query) {
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

/** Assemble DataQueue status string */
SyncImagesBroadcaster::StringPtrType SyncImagesBroadcaster::AssembleStatusString() {
  StringPtrType status_str(new std::string(""));
  // Looping frequency
  {
    char buffer[100];
    //if (max_loop_frequency_>1000.0f) snprintf(buffer, 100, "%.0f(>1k)", loop_frequency_);
    //else snprintf(buffer, 100, "%.0f(%.0f)", loop_frequency_, max_loop_frequency_);
    snprintf(buffer, 100, "%ld %ld %ld(us) %ld", frame_to_frame_interval, intra_frame_interval, max_to_min_interval,
        num_out_of_sync_events_);
    *status_str += buffer;
  }
  return status_str;
}

/** Imaging specific methods **/

// Prints point grey library version information
bool SyncImagesBroadcaster::PrintPGBuildInfo() {
  fc2Version version;
  char versionStr[512];
  char timeStamp[512];

  fc2GetLibraryVersion(&version);
  
  sprintf( 
    versionStr, 
    "FlyCapture2 library version: %d.%d.%d.%d\n", 
    version.major, version.minor, version.type, version.build );
  printf( "%s", versionStr );
  sprintf( timeStamp, "Application build date: %s %s\n\n", __DATE__, __TIME__ );
  printf( "%s", timeStamp );
  
  return true;
}

// Initiate the point grey cameras
bool SyncImagesBroadcaster::InitiatePGCameras() {
  
  fc2Error error;
  unsigned int num_cameras = 0;
  
  for (int i = 0; i < num_cameras_attached_; i++ ) {
    
    error = fc2CreateContext(&pg_contexts_[i]);
    if (error != FC2_ERROR_OK){
      printf("Error in fc2CreateContext: %d\n", error);
      return false;
    }
    
    error = fc2GetNumOfCameras(pg_contexts_[i], &num_cameras);
    if ( error != FC2_ERROR_OK ) {
      printf( "Error in fc2GetNumOfCameras: %d\n", error);
      return false;
    }

    if (num_cameras < num_cameras_attached_) {
        printf("Number of cameras found were less than attached cameras, found = %d, attached = %d\n", num_cameras, num_cameras_attached_);
        return false;
    } else {
      printf( "All attached cameras were found = %d\n", num_cameras);
    }

    // Get the i'th camera
    VLOG(1) << "Getting handle for camera " << i << " with id = " << ib_config_->pg_camera(i).id();
    error = fc2GetCameraFromSerialNumber(pg_contexts_[i], ib_config_->pg_camera(i).id(), &pg_guids_[i] );
    if (error != FC2_ERROR_OK) {
      printf("Error in fc2GetCameraFromIndex: %d\n", error);
      return false;
    }

    error = fc2Connect(pg_contexts_[i], &pg_guids_[i]);
    if (error != FC2_ERROR_OK) {
      printf("Error in fc2Connect: %d\n", error);
      return false;
    }

    if (!PrintCameraInfo(&pg_contexts_[i])) {
      LOG(ERROR) << "Could not print camera info";
      return false;
    }
    
    if (ib_config_->frame_rate() == 60) {
      error = fc2SetVideoModeAndFrameRate(pg_contexts_[i], FC2_VIDEOMODE_640x480Y8, FC2_FRAMERATE_60);
    } else {
      error = fc2SetVideoModeAndFrameRate(pg_contexts_[i], FC2_VIDEOMODE_640x480Y8, FC2_FRAMERATE_30);
    }
    if (error != FC2_ERROR_OK) {
      printf("Error in SetVideoModeAndFrameRate: %d\n", error);
      return false;
    }

    if (!SetTimeStamping(&pg_contexts_[i], TRUE)) {
      LOG(ERROR) << "Could not set timestamping";
      return false;
    }
    
    error = fc2StartCapture(pg_contexts_[i]);
    if (error != FC2_ERROR_OK) {
      printf("Error in fc2StartCapture: %d\n", error);
      return false;
    }
    
  }   // for loop to initiate each camera
  
  return true;
}

// Print camera information
bool SyncImagesBroadcaster::PrintCameraInfo(fc2Context* context) {
  fc2Error error;
  fc2CameraInfo camInfo;
  error = fc2GetCameraInfo(*context, &camInfo);
  if ( error != FC2_ERROR_OK ) {
    // Error
    return false;
  }
  
  printf(
    "\n*** CAMERA INFORMATION ***\n"
    "Serial number - %u\n"
    "Camera model - %s\n"
    "Camera vendor - %s\n"
    "Sensor - %s\n"
    "Resolution - %s\n"
    "Firmware version - %s\n"
    "Firmware build time - %s\n\n",
    camInfo.serialNumber,
    camInfo.modelName,
    camInfo.vendorName,
    camInfo.sensorInfo,
    camInfo.sensorResolution,
    camInfo.firmwareVersion,
    camInfo.firmwareBuildTime );
  return true;
}

// Set timestamping on teh camera
bool SyncImagesBroadcaster::SetTimeStamping(fc2Context* context, BOOL enableTimeStamp) {
  fc2Error error;
  fc2EmbeddedImageInfo embeddedInfo;

  error = fc2GetEmbeddedImageInfo(*context, &embeddedInfo);
  if (error != FC2_ERROR_OK) {
    printf( "Error in fc2GetEmbeddedImageInfo: %d\n", error );
    return false;
  }

  if (embeddedInfo.timestamp.available != 0) {       
    embeddedInfo.timestamp.onOff = enableTimeStamp;
  }    

  error = fc2SetEmbeddedImageInfo(*context, &embeddedInfo);
  if (error != FC2_ERROR_OK) {
    printf( "Error in fc2SetEmbeddedImageInfo: %d\n", error);
    return false;
  }
  return true;
}

/** Start Looping - run operations and listen to messages */
bool SyncImagesBroadcaster::StartLooping() {
  
  LOG(ERROR) << "Use GrabAndPublishImages() in place of StartLooping()";
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

/** Start grabbing images - run listen to messages */
bool SyncImagesBroadcaster::GrabAndPublishImages() {
  
    int numCameras = num_cameras_attached_;
    long int numSecondsToGrab = 0;
    static const int NCAMS = num_cameras_attached_;
    static const int FRAME_RATE = ib_config_->frame_rate();
    static const int IMAGE_SIZE = ib_config_->image_size();
    num_out_of_sync_events_ = 0;

    // Initiate pointers to publisher, commands subscriber and status subscriber
    zmq::socket_t& publisher = *publishers_map_[publisher_name_];
    zmq::socket_t& subscriber = *subscriptions_map_[command_subscriber_name_];
    zmq::socket_t& status_subscriber = *subscriptions_map_[status_subscriber_name_];
    
    fc2Error error;
    fc2Image rawImage;
    fc2Image convertedImage;
    char ImageHeaderBuffer[NCAMS][256];
    unsigned char ImageDataBuffer[NCAMS][IMAGE_SIZE];
    
    unsigned long int * camera_ts;
    unsigned long int * last_camera_ts;
    unsigned long int * camera_cycle_counter;
    unsigned long int * last_cycleSeconds;
    long long * camera_start_time;
    int cntr;
    int i;
    int iCam;
    int rc;
    int msg_nbr=0;
    int allocated_image_header_size;
    
    struct timeval current_time; 
    //long int last_frame_capture_time, frame_capture_time;
    long int * last_image_time;
    long int * image_time;
    long int start_time, end_time, curr_time;
    int isFirst = 1; 
    int capture_num = 0;
    const int still_first_count = 5;
    
    // allocate memory to times
    last_image_time = (long int*) malloc (sizeof( long int )*numCameras);
    image_time = (long int*) malloc (sizeof( long int )*numCameras);
    camera_ts = (unsigned long int *) malloc (sizeof(unsigned long int )*numCameras);
    last_camera_ts = (unsigned long int *) malloc (sizeof(unsigned long int )*numCameras);
    camera_start_time = (long long *) malloc (sizeof(long long )*numCameras);
    camera_cycle_counter = (unsigned long int *) malloc (sizeof(unsigned long int )*numCameras);
    last_cycleSeconds = (unsigned long int *) malloc (sizeof(unsigned long int )*numCameras);
    //printf("malloc complete\n");
    
    // initialize times
    gettimeofday(&current_time, NULL);
    last_frame_capture_time = current_time.tv_sec * 1000000 + current_time.tv_usec;
    start_time = last_frame_capture_time;
    end_time = start_time + numSecondsToGrab*1000000;
    frame_capture_time = last_frame_capture_time;
    for (cntr = 0; cntr < numCameras; cntr++) {
        last_image_time[cntr] = last_frame_capture_time;
        image_time[cntr]      = last_frame_capture_time;
        camera_ts[cntr]       = (unsigned long int) last_frame_capture_time;
        last_camera_ts[cntr]  = (unsigned long int) last_frame_capture_time;
        camera_cycle_counter[cntr] = 0;
        last_cycleSeconds[cntr] = 0;
    }
    long long max_image_time = 0;
    long long min_image_time = 0;
    int most_ahead_camera = 0;
    //long long sync_threshold = ((long long) (1000000 / FRAME_RATE)) - 100;
    long long sync_threshold = 1000;
    int skip_camera[NCAMS];
    long long cam_time_vec[NCAMS];
    for (cntr = 0; cntr < numCameras; cntr++) {
        skip_camera[cntr] = 0;
        cam_time_vec[cntr] = 0;
    }
    int out_of_sync_cntr = 0;         // number of frames out of sync has been seen continuously
    int out_of_sync_threshold = FRAME_RATE;  // if out of sync condition is seen continuously for these many frames, sleep a little
    long out_of_sync_sleep_time = 500000;  // sleep this long in usecs to get over out of sync condition
    long wakeup_time = 0;
    //printf("assignment complete\n");
    
    int grab_images = 1; // setting to not capture by default. To start capture, set to 1.
    int exit_now = 0;
    
    int log_level = 1; // 0 = no logging, 1 = some logging, 2 = verbose;

    i = 0; 
    char cmnd_msg [256];
    // for every set of images, go through each camera
    while ( exit_now != 1 ) {
      
        // non blocking read of the command
        memset(cmnd_msg, 0, sizeof(cmnd_msg));
        
        //while (1) {
            int size = zmq_recv (subscriber, cmnd_msg, 255, ZMQ_DONTWAIT);
            if (size != -1) {
                //  Process task
                printf("  Command = %s\n", cmnd_msg);
                
                char argument[256]; //maximum size of a command is 256 characters
                int arglength = 0;
                
                bool done = false;
                
                if (!done) {
                strcpy (argument, pause_command_str_.c_str());
                arglength  = strlen(argument);
                if (strncmp(cmnd_msg, argument, arglength)==0) {
                  if (grab_images == 1) {
                    grab_images = 0;
                  } else {
                    grab_images = 1;
                    capture_num = 0;
                    for (cntr = 0; cntr < numCameras; cntr++) {
                        camera_cycle_counter[cntr] = 0;
                    }                    
                  }
                  done = true;
                }
                memset(argument, 0, sizeof(argument));
                }
                
                //strcpy (argument, resume_command_str_.c_str());
                //arglength  = strlen(argument);
                //if (strncmp(cmnd_msg, argument, arglength)==0) {
                //    grab_images = 1;
                //    capture_num = 0;
                //    for (cntr = 0; cntr < numCameras; cntr++) {
                //        camera_cycle_counter[cntr] = 0;
                //    }
                //}
                //memset(argument, 0, sizeof(argument));
                
                if (!done) {
                strcpy (argument, exit_command_str_.c_str());
                arglength  = strlen(argument);
                if (strncmp(cmnd_msg, argument, arglength)==0) {
                    exit_now = 1;
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
                  done = true;
                }
                memset(argument, 0, sizeof(argument));
                }
                
            }
        //    else
        //        break;
        ///}
        
        if (exit_now == 0 && numSecondsToGrab > 0) {
            if (curr_time > end_time) exit_now = 1;
        }
        
        if (wakeup_time != 0) {
            if (curr_time > wakeup_time) {
                printf("    waking up and continuing image capture.\n\n");
                
                // reset
                grab_images = 1;
                wakeup_time = 0;
                out_of_sync_cntr = 0;
                max_image_time = 0;
                min_image_time = 0;
                capture_num = 0;
                for (cntr = 0; cntr < numCameras; cntr++) {
                    skip_camera[cntr] = 0;
                    camera_cycle_counter[cntr] = 0;
                }
            }
        }
        
        if (grab_images) 
        {
    
            i = (i + 1) % 60000;
            capture_num = (capture_num + 1) % 60000;
            if (capture_num <= still_first_count) {isFirst = 1;} else {isFirst = 0;}
        
            if (log_level >= 2 || (log_level == 1 && i%FRAME_RATE==0)) printf("Capturing images num = %d\n", i);
            gettimeofday(&current_time, NULL);
            frame_capture_time = current_time.tv_sec * 1000000 + current_time.tv_usec;
            if (isFirst==1) {start_time = frame_capture_time;}
            //printf("  Capture delay(us)=%*ld,ts=%*ld,\n", 10, frame_capture_time - last_frame_capture_time, 20, frame_capture_time - start_time);
            
            msg_nbr += 1;
            
            // message header
            char * msg_header;
            //int allocated_msg_header_size = asprintf( &msg_header, "%s", "GhumioImages");
            frame_to_frame_interval = frame_capture_time - last_frame_capture_time;
            int allocated_msg_header_size = asprintf( &msg_header, "GhumioImages,%*ld,%*ld", 10, frame_to_frame_interval, 20, frame_capture_time);
            last_frame_capture_time = frame_capture_time;
            if (log_level >= 2 || (log_level == 1 && i%FRAME_RATE==0)) printf("  %s[%d]\n", msg_header, allocated_msg_header_size);
            int msg_header_size = sizeof(msg_header) / sizeof(msg_header[0]);
            // create a message from the header
            zmq_msg_t msg1;
            rc = zmq_msg_init_size (&msg1, allocated_msg_header_size);
            assert (rc == 0);
            memcpy (zmq_msg_data (&msg1), msg_header, allocated_msg_header_size);
            free(msg_header);
            // Send the message to the socket
            rc = zmq_msg_send (&msg1, publisher, ZMQ_SNDMORE); 
            assert (rc == allocated_msg_header_size);
            //printf("  Sent msg %d header\n", msg_nbr);

            for (iCam = 0; iCam < numCameras; iCam++)
            {
            
                if (skip_camera[iCam] != 1) 
                {
            
                    error = fc2CreateImage( &rawImage );
                    if ( error != FC2_ERROR_OK )
                    {
                        printf( "Error in fc2CreateImage: %d\n", error );
                    }

                    error = fc2CreateImage( &convertedImage );
                    if ( error != FC2_ERROR_OK )
                    {
                        printf( "Error in fc2CreateImage: %d\n", error );
                    }

                    // If externally allocated memory is to be used for the converted image,
                    // simply assigning the pData member of the fc2Image structure is
                    // insufficient. fc2SetImageData() should be called in order to populate
                    // the fc2Image structure correctly. This can be done at this point,
                    // assuming that the memory has already been allocated.

                    // Retrieve the image
                    error = fc2RetrieveBuffer( pg_contexts_[iCam], &rawImage );
                    
                    gettimeofday(&current_time, NULL);
                    image_time[iCam] = current_time.tv_sec * 1000000 + current_time.tv_usec;
                    //printf("    Camera capture delay(us) = %ld\n", image_time[iCam] - last_image_time[iCam]);
                    last_image_time[iCam] = image_time[iCam];
                    
                    if ( error != FC2_ERROR_OK )
                    {
                        printf( "Error in retrieveBuffer: %d\n", error);
                    }
                    else
                    {
                        // Get and print out the time stamp
                        fc2TimeStamp ts = fc2GetImageTimeStamp( &rawImage);
                        // increment the camera cycle counter when cycle counter hits 128
                        if (ts.cycleSeconds == 0 && last_cycleSeconds[iCam] == 127) camera_cycle_counter[iCam] += 1;
                        unsigned long int offset = (unsigned long int) ((double) ts.cycleOffset * 125.0 / 3072.0);
                        camera_ts[iCam] = (unsigned long int) (ts.cycleSeconds * 1000000 + 125 * ts.cycleCount + offset);
                        last_cycleSeconds[iCam] = (unsigned long int) ts.cycleSeconds;
                        if (isFirst==1) {camera_start_time[iCam] = (long long) frame_capture_time - (long long) camera_ts[iCam];}
                        
                        //printf("    Camera %d, delay(us) = %ld, ts = %ld \n",
                        //    iCam, camera_ts[iCam] - last_camera_ts[iCam], camera_ts[iCam] - camera_start_time[iCam]);
                            
                        //printf("    raw image info = %d x %d s %d, %d, format = 0x%x \n",
                        //    rawImage.cols, rawImage.rows, rawImage.stride, rawImage.dataSize, rawImage.format
                        //);
                        
                        // image header
                        char * image_header;
                        long long cam_time = (long long) camera_ts[iCam] + (long long) (camera_cycle_counter[iCam]*128000000) + (long long) camera_start_time[iCam];
                        allocated_image_header_size = asprintf( &image_header, "Image%d,%*ld,%*lld", iCam, 10, camera_ts[iCam] - last_camera_ts[iCam], 20, cam_time);
                        last_camera_ts[iCam] = camera_ts[iCam];
                        cam_time_vec[iCam] = cam_time;
                        // copy header to image header buffer for later reuse
                        memcpy (ImageHeaderBuffer[iCam], image_header, allocated_image_header_size);
                        if (log_level >= 2) printf("    %s[%d]\n", image_header, allocated_image_header_size);
                        // create a message from the header
                        zmq_msg_t msg1;
                        rc = zmq_msg_init_size (&msg1, allocated_image_header_size);
                        assert (rc == 0);
                        memcpy (zmq_msg_data (&msg1), image_header, allocated_image_header_size);
                        free(image_header);
                        // Send the message to the socket
                        rc = zmq_msg_send (&msg1, publisher, ZMQ_SNDMORE); 
                        assert (rc == allocated_image_header_size);
                        //printf("  Sent image %d header\n", msg_nbr);
                        
                    }
                    
                    // Convert image from raw format
                    if ( error == FC2_ERROR_OK )
                    {
                        // Convert the final image to RGB
                        error = fc2ConvertImageTo(FC2_PIXEL_FORMAT_BGR, &rawImage, &convertedImage);
                        if ( error != FC2_ERROR_OK )
                        {
                            printf( "Error in fc2ConvertImageTo: %d\n", error );
                        } else {
                            //printf(
                            //    "    new image info = %d x %d s %d, %d , format = 0x%x \n",
                            //    convertedImage.cols, convertedImage.rows, convertedImage.stride, convertedImage.dataSize, convertedImage.format
                            //);
                            
                            // create image message
                            int image_size = convertedImage.dataSize;
                            zmq_msg_t msg;
                            rc = zmq_msg_init_size (&msg, image_size);
                            assert (rc == 0);
                            // copy image data to the buffer
                            memcpy (ImageDataBuffer[iCam], convertedImage.pData, IMAGE_SIZE);
                            // Add the image data
                            memcpy (zmq_msg_data (&msg), convertedImage.pData, image_size);
                            // Send the message to the socket
                            rc = zmq_msg_send (&msg, publisher, ZMQ_SNDMORE);
                            assert (rc == image_size);
                            if (log_level >= 2) printf("    Image %d [%d]\n", iCam, image_size);

                            /*
                            // Save it to PNG
                            char* save_filename;
                            int fnmsz = asprintf( &save_filename, "%s%d%s", "fc2TestImage", iCam, ".png");
                            printf("Saving the last image to %s \n", save_filename);
		                        error = fc2SaveImage( &convertedImage, save_filename, FC2_PNG );
		                        if ( error != FC2_ERROR_OK )
		                        {
			                        printf( "Error in fc2SaveImage: %d\n", error );
			                        printf( "Please check write permissions.\n");
		                        }
		                        free( save_filename );*/
                        }
                    }
                    
                    // close 
                    error = fc2DestroyImage( &rawImage );
                    if ( error != FC2_ERROR_OK )
                    {
                        printf( "Error in fc2DestroyImage: %d\n", error );
                    }
                    error = fc2DestroyImage( &convertedImage );
                    if ( error != FC2_ERROR_OK )
                    {
                        printf( "Error in fc2DestroyImage: %d\n", error );
                    }
                    
                } // if (skip_camera[most_ahead_camera] != 1)
                else 
                {
                
                    printf("    Skipped image, reusing older image, camera = %d.\n", iCam);
                    skip_camera[iCam] = 0;

                        // image header
                        char * image_header;
                        image_header = (char*) malloc (sizeof(char)*allocated_image_header_size);
                        last_camera_ts[iCam] = camera_ts[iCam];
                        // copy header to image header buffer for later reuse
                        memcpy (image_header, ImageHeaderBuffer[iCam], allocated_image_header_size);
                        if (log_level >= 2) printf("    %s[%d]\n", image_header, allocated_image_header_size);
                        // create a message from the header
                        zmq_msg_t msg1;
                        rc = zmq_msg_init_size (&msg1, allocated_image_header_size);
                        assert (rc == 0);
                        memcpy (zmq_msg_data (&msg1), image_header, allocated_image_header_size);
                        free(image_header);
                        // Send the message to the socket
                        rc = zmq_msg_send (&msg1, publisher, ZMQ_SNDMORE); 
                        assert (rc == allocated_image_header_size);
                        //printf("  Sent image %d header\n", msg_nbr);

                        // create image message
                        int image_size = IMAGE_SIZE;
                        zmq_msg_t msg;
                        rc = zmq_msg_init_size (&msg, image_size);
                        assert (rc == 0);
                        // Add the image data
                        memcpy (zmq_msg_data (&msg), ImageDataBuffer[iCam], image_size);
                        // Send the message to the socket
                        rc = zmq_msg_send (&msg, publisher, ZMQ_SNDMORE);
                        assert (rc == image_size);
                        if (log_level >= 2) printf("    Image %d [%d]\n", iCam, image_size);
                            
                    printf("    Done sending older image, camera = %d.\n", iCam);
                }
                
                // setup max and min image times for sync check
                if (iCam == 0) {
                    min_image_time = cam_time_vec[iCam];
                    max_image_time = cam_time_vec[iCam];
                    most_ahead_camera = 0;
                } else {
                    if (cam_time_vec[iCam] < min_image_time) {
                        min_image_time = cam_time_vec[iCam];
                    }
                    if (cam_time_vec[iCam] > max_image_time) {
                        max_image_time = cam_time_vec[iCam];
                        most_ahead_camera = iCam;
                    }
                }  // if (iCam == 0)
                        
            } //iCam for loop
            
            // message footer 
            gettimeofday(&current_time, NULL);
            end_capture_time = current_time.tv_sec * 1000000 + current_time.tv_usec;
            char* ending_tag;
            int tag_size = asprintf( &ending_tag, "Ending,%*ld", 20, end_capture_time);
            zmq_msg_t msg0;
            rc = zmq_msg_init_size (&msg0, tag_size);
            assert (rc == 0);
            // Add the message subject
            memcpy (zmq_msg_data (&msg0), ending_tag, tag_size);
            // Send the message to the socket
            rc = zmq_msg_send (&msg0, publisher, 0);
            assert (rc == tag_size);
            if (log_level >= 2) printf("  %s[%d]\n", ending_tag, tag_size);
            free ( ending_tag );
            
            // reporting
            intra_frame_interval = end_capture_time - frame_capture_time;
            max_to_min_interval = max_image_time - min_image_time;
            if (log_level >= 2) printf("    took(us) = %ld\n", intra_frame_interval);
            if (log_level >= 2 || (log_level == 1 && i%FRAME_RATE==0)) printf("    sync diff(us) = %ld[Cam%d]\n", max_to_min_interval, most_ahead_camera);
            
            // Out-of-sync checks
            // setup skipping of camera capture if images are out-of-sync
            // capture_num > 1 condition is to ensure that last image is present in the last image buffer
            if (max_image_time - min_image_time > sync_threshold && capture_num > 1) {
                printf("    cameras not in sync. will skip most ahead camera = %d\n", most_ahead_camera);
                skip_camera[most_ahead_camera] = 1;
                ++out_of_sync_cntr;
                num_out_of_sync_events_++;
                
                // check if out of sync condition has been going on for a while. if so, sleep a little
                if (out_of_sync_cntr > out_of_sync_threshold) {
                    printf("\n    cameras have been out of sync for a while. Will sleep for %ld(us) to remove the race condition.\n", out_of_sync_sleep_time);
                    grab_images = 0;
                    wakeup_time = curr_time + out_of_sync_sleep_time;
                }
            } else {
                out_of_sync_cntr = 0;
            }
            
            if (log_level >= 2 || (log_level == 1 && i%FRAME_RATE==0)) printf("\n");
            
        } //if (grab_images)
        else 
        {
            usleep(200000);
        }
        
        gettimeofday(&current_time, NULL);
        curr_time = current_time.tv_sec * 1000000 + current_time.tv_usec;
        
      // Handle status messages
      zmq::message_t message;
      if (status_subscriber.recv(&message, ZMQ_DONTWAIT)) {
        
        // Create a string by copying the incoming message data. We want to pass the string over
        //  to message handlers. When the string is passed to a handler, it is responsible for
        //  destructing it. It will be 'gone' from here. So we use unique_ptr to pass ownership.
        //  We also want to save on time it takes to copy the string from here to the keeper object
        //  as we are no longer copying the string bytes from here to keeper's method. Copying is
        //  only done once from the message_t buffer to a string object buffer. 
        size_t msg_size = message.size();
        const char* msg_data_ptr = static_cast<char*>(message.data());  // cast the void* to char*
        
        /** Handle status queries */
        std::unique_ptr<std::string> msg_str_ptr(       // copy the entire message
            new std::string(msg_data_ptr, msg_size));   // copy
        VLOG(3) << "Status query: " << *msg_str_ptr;
        HandleStatusQuery(std::move(msg_str_ptr));
        
        // Message_String unique pointer destructs here if it has not been moved
      } // recv message
        
            
    } //numSecondsToGrab
    
    free( last_image_time );
    free( image_time );
    free( camera_ts );
    free( last_camera_ts );
    free( camera_start_time );
    free( camera_cycle_counter );
    free( last_cycleSeconds );
    
    return true;
}


} // namespace anantak
