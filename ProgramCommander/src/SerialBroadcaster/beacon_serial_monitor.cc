/** Beacon serial monitor
 * Implementation
 */


/** Header file include */
#include "SerialBroadcaster/beacon_serial_monitor.h"

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
#include <iomanip>

namespace anantak {

const std::string BeaconSerialMonitor::IMU_TYPE = "ImuQuatAccel";
const int32_t BeaconSerialMonitor::IMU_NUM = 3;

/** Constructor */
BeaconSerialMonitor::BeaconSerialMonitor(std::string programs_setup_filename,
    std::string component_name) {
  programs_setup_config_filename_ = programs_setup_filename;
  component_name_ = component_name;  
  is_initiated_ = false;
  if (!Initiate()) LOG(ERROR) << "Could not initiate BeaconSerialMonitor.";
}

/** Destructor - all members are self destructing */
BeaconSerialMonitor::~BeaconSerialMonitor() {
  VLOG(1) << "BeaconSerialMonitor shutdown.";
  VLOG(1) << "ZMQ transport and all pub/subs will destruct automatically.";
  LOG(INFO) << "Disconnecting from port " << symlink_;
  DisconnectFromPort();
  LOG(INFO) << "Disconnected.";
}

/** Initiator - creates all starting objects */
bool BeaconSerialMonitor::Initiate() {
  if (!InitiateComponent()) {
    LOG(ERROR) << "Could not initiate BeaconSerialMonitor component objects";
    return false;
  }
  if (!InitiateSerialMonitor()) {
    LOG(ERROR) << "Could not initiate SerialMonitor objects";
    return false;    
  }
  // Initiation succeeded
  is_initiated_ = true;
  return true;
}

/** Component Initiator - creates all starting component objects */
bool BeaconSerialMonitor::InitiateComponent() {
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
  std::unique_ptr<anantak::SerialMonitorConfig> config =
      anantak::ReadProtobufFile<anantak::SerialMonitorConfig>(config_file_path);
  if (config->has_symlink()) {
    symlink_ = config->symlink();
  } else {
    LOG(ERROR) << "No serial symlink was found in config file? Can not continue.";
    return false;
  }
  VLOG(1) << "Serial monitor symlink: " << symlink_;
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
  max_loop_frequency_ = 0.0f;
  
  // Turn over the config object to class storage
  config_= std::move(config);
  message_publisher_ = publishers_map_[publisher_name_].get();
  
  return true;
}

/** Initiator - creates starting image processing objects */
bool BeaconSerialMonitor::InitiateSerialMonitor() {
  
  is_connected = false;
  if (!ConnectToPort(symlink_)) {
    LOG(ERROR) << "Could not connect to port " << symlink_ << " .Exit.";
    return false;
  }
  
  start_time = get_wall_time_microsec();
  last_recieve_time = 0;
  recieve_time = 1000;
  recieve_rate = 0.0;
  elapsed_time = 0.0;
  
  imu_message_subject_ = component_name_ + " ";
  LOG(INFO) << "Beacon imu messages will be published on subject = '" << imu_message_subject_;
  imu_message_subject_length_ = imu_message_subject_.length();
  log_level_ = 0;
  
  return true;
}

/** Handle commands coming from the commander subscriber */
bool BeaconSerialMonitor::HandleCommand(StringPtrType cmd) {
  // exit command
  if (*cmd == exit_command_str_) {exit_loop_ = true;}
  else if (*cmd == log_command_str_) {if (log_level_==0) log_level_=1; else log_level_=0;}
  return true;
}

/** Handle status queries coming from ProgramCommander */
bool BeaconSerialMonitor::HandleStatusQuery(StringPtrType status_query) {
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
bool BeaconSerialMonitor::StartLooping() {
  
  anantak::Looper looper(loop_frequency_);
  while (!exit_loop_) {
    
    // Read the pixy camera and publish the message
    ReadDataAndPublish();
    
    // Loop through each subscriber, read messages and pass them to status keeper objects
    for (PubSubMapIteratorType i_map = subscriptions_map_.begin();
        i_map != subscriptions_map_.end(); i_map++) {
      //VLOG(4) << "Reading subscriber " << i_map->first;
      
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
BeaconSerialMonitor::StringPtrType BeaconSerialMonitor::AssembleStatusString() {
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
bool BeaconSerialMonitor::ReadDataAndPublish() {
  
  // Allocate memory for read buffer
  uint8_t temp, rd0, rd1, rd2;
  temp = rd0 = rd1 = rd2 = '\0';
  ssize_t sz;
  uint8_t buf[256];
  memset (&buf, '\0', sizeof buf);
  int n = 0;
  // Read from port 
  do {
    sz = read(USB, &temp, 1);
    if (sz>0) {
      buf[n] = temp; // copy from temp buffer
      n++;
      // save for end of message checking
      rd2 = rd1;
      rd1 = rd0;
      rd0 = temp;
    }
  }
  while (!(rd0=='\n' && rd1=='\r' && rd2==0x00));
  recieve_time = get_wall_time_microsec();

  if (n < 0) {
    std::cout << "Error reading: " << strerror(errno) << std::endl;
  }
  else if (n == 0) {
    std::cout << "Read nothing!" << std::endl;
  }
  else {
    if (log_level_>0) std::cout << recieve_time << " ";
    if (n==20 && buf[0]=='$') {
      // Decode packet
      uint8_t msg_count;
      uint8_t packet_count;
      int16_t q[4];
      int16_t a[3];
      msg_count = int8_t(buf[1]);
      packet_count = int8_t(buf[16]);
      q[0] = int16_t((buf[2] << 8) | buf[3]);
      q[1] = int16_t((buf[4] << 8) | buf[5]);
      q[2] = int16_t((buf[6] << 8) | buf[7]);
      q[3] = int16_t((buf[8] << 8) | buf[9]);
      a[0] = int16_t((buf[10] << 8) | buf[11]);
      a[1] = int16_t((buf[12] << 8) | buf[13]);
      a[2] = int16_t((buf[14] << 8) | buf[15]);
      //printf("buf = %X %X %X %X, %X %X %X \n",
      //       buf[2]<<8 | buf[3] , buf[4]<<8 | buf[5] , buf[6]<<8 | buf[7], buf[8]<<8 | buf[9],
      //       buf[10]<<8 | buf[11] , buf[12]<<8 | buf[13], buf[14]<<8 | buf[15]);
      if (log_level_>0) std::cout << " q=" << q[0] << "," << q[1] << "," << q[2] << "," << q[3]
          << " a=" << a[0] << "," << a[1] << "," << a[2] 
          << " counts = " << int(msg_count) << " " << int(packet_count);
          
      // Assemble message
      imu_message_.Clear();
      anantak::HeaderMsg* hdr_msg = imu_message_.mutable_header();
      anantak::ImuMsg* imu_msg = imu_message_.mutable_imu_msg();
      // Set header
      hdr_msg->set_timestamp(recieve_time);
      hdr_msg->set_type(IMU_TYPE);
      hdr_msg->set_recieve_timestamp(recieve_time);
      hdr_msg->set_send_timestamp(recieve_time);
      // Set message
      imu_msg->set_imu_num(IMU_NUM);
      imu_msg->add_quaternion(int32_t(q[0]));
      imu_msg->add_quaternion(int32_t(q[1]));
      imu_msg->add_quaternion(int32_t(q[2]));
      imu_msg->add_quaternion(int32_t(q[3]));
      imu_msg->add_linear(int32_t(a[0]));
      imu_msg->add_linear(int32_t(a[1]));
      imu_msg->add_linear(int32_t(a[2]));
      imu_msg->add_angular(int32_t(0));
      imu_msg->add_angular(int32_t(0));
      imu_msg->add_angular(int32_t(0));
      
      // Send message
      SendSensorMessage(imu_message_, imu_message_subject_, message_publisher_);
          
    } else {
      //std::string buf_str(buf);
      //std::cout << "  " << buf_str;
    }
    
    // report message rate
    recieve_rate = 1000000.00 / (double) (recieve_time - last_recieve_time);
    //elapsed_time = (double) (recieve_time - start_time) / 60000000.0;
    last_recieve_time = recieve_time;
    if (log_level_>0) {
        //cout << fixed << setprecision(1) << " elapsed(m) =" << setw(6) << elapsed_time << ", ";
        std::cout << std::fixed << std::setprecision(0) << " rate=" << std::setw(5) << recieve_rate << ", ";
    }
    
    if (log_level_>0) std::cout << std::endl;
  }
  
  return true;
}

// Send a sensor message over the bus. This could be a library level function
bool BeaconSerialMonitor::SendSensorMessage(const anantak::SensorMsg& sensor_msg,
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


bool BeaconSerialMonitor::ConnectToPort(const std::string& portname) {
  //// Setup the serial port ////

  // Open File Descriptor
  USB = open(portname.c_str(), O_RDWR|O_NOCTTY);

  // Error Handling
  if (USB < 0) {
    LOG(ERROR) << "Error " << errno << " opening " << portname << ": " << strerror(errno);
    return false;
  }

  // Configure Port
  memset(&tty, 0, sizeof tty);
  
  if (tcgetattr(USB, &tty) != 0 ) {
    LOG(ERROR) << "Error " << errno << " from tcgetattr: " << strerror(errno);
    return false;
  }

  // Save old tty parameters 
  tty_old = tty;

  // Set Baud Rate 
  //cfsetospeed (&tty, (speed_t)B115200);
  //cfsetispeed (&tty, (speed_t)B115200);
  cfsetospeed (&tty, (speed_t)B57600);
  cfsetispeed (&tty, (speed_t)B57600);

  // Setting other Port Stuff 
  tty.c_cflag     &=  ~PARENB;          // Make 8n1
  tty.c_cflag     &=  ~CSTOPB;
  tty.c_cflag     &=  ~CSIZE;
  tty.c_cflag     |=  CS8;

  tty.c_cflag     &=  ~CRTSCTS;         // no flow control
  //tty.c_lflag     &=  ~ICANON;          // Set non-canonical mode
  tty.c_cc[VMIN]      =   1;            // read doesn't block
  tty.c_cc[VTIME]     =   5;            // 0.5 seconds read timeout
  tty.c_cflag     |=  CREAD | CLOCAL;   // turn on READ & ignore ctrl lines

  // Make raw
  cfmakeraw(&tty);

  // Flush Port, then applies attributes
  tcflush(USB, TCIFLUSH);
  if (tcsetattr(USB, TCSANOW, &tty) != 0) {
    LOG(ERROR) << "Error " << errno << " from tcsetattr";
    return false;
  }
  
  // Set non-blocking mode
  //int flags;
  //flags = fcntl(USB, F_GETFL, 0);
  //if (-1 != flags)
  //int rc = fcntl(USB, F_SETFL, flags | O_NONBLOCK);  
  
  LOG(INFO) << "Connecteded to port" << portname;
  is_connected = true;
  
  return true;
}

bool BeaconSerialMonitor::DisconnectFromPort() {
  if (is_connected) {
    close(USB);
    LOG(INFO) << "Disconnected from USB port";
  }
  is_connected = false;;
  return true;
}



} // namespace anantak