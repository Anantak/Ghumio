/**
 *  Test Component 
 *
 *  This is to test the status-query-reply system. This is a dummy component that does some 'work'
 *  by simply sleeping for some time. It periodically processes the incoming status queries. It
 *  also listens to manual commands issued by the web commander. 
 */


/** Google logging and flags libraries */
#include <glog/logging.h>
#include <gflags/gflags.h>

// Anantak includes
#include "Utilities/common_functions.h"
#include "ComponentCommander/component_status_keeper.h"
#include "ComponentCommander/component_status_keeper_factory.h"

// Include protocol buffers
#include "programs_setup.pb.h"
#include "configurations.pb.h"

/** ZMQ includes */
#include <zmq.hpp>

/** Command line flags */
DEFINE_string(name, "", "Name of the component");
DEFINE_string(setup_file, "", "Programs setup file");

int status_counter = 0;

std::unique_ptr<std::string> GenerateHappyStatus() {
  std::unique_ptr<std::string> ptr(new std::string("I am happy "));
  *ptr += std::to_string(status_counter);
  status_counter++;
  return ptr;
}

int main(int argc, char** argv) {
  
  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);
  // Parse the commandline flags 
  google::ParseCommandLineFlags(&argc, &argv, true);
  // Verify the version of protobuf library 
  GOOGLE_PROTOBUF_VERIFY_VERSION;
  // Setting the glog output to stderr by default
  FLAGS_logtostderr = 1;
  
  std::string component_name = FLAGS_name;
  VLOG(1) << "Component name = " << component_name;
  std::string programs_setup_filename = FLAGS_setup_file;
  VLOG(1) << "Setup file = " << programs_setup_filename;
  //std::string programs_setup_filename = "../src/test/test_node.cfg";
  std::string command_subscriber_name = "";
  std::string status_query_subscriber_name = "";
  std::string exit_command_str = "COMMAND "+component_name+" exit";
  
  // Read the ProgramsSetup file and look for your own component settings.
  anantak::ProgramsSetup programs_setup;
  anantak::ReadProgramsSetup(programs_setup_filename, &programs_setup);
  anantak::ProgramsSetup::ProgramSettings component_settings;
  for (int i=0; i<programs_setup.program_settings_size(); i++) {
    const anantak::ProgramsSetup::ProgramSettings& program_settings =
      programs_setup.program_settings(i);
    const std::string& program_name = program_settings.name();
    if (program_name == component_name) {
      VLOG(1) << "Found settings for " << component_name;
      component_settings = program_settings;    // made a copy of object here
    }
  } // for
  
  // Create the StatusKeeper
  anantak::ComponentStatusKeeperFactory component_status_keeper_factory;
  std::unique_ptr<anantak::ComponentStatusKeeper> status_keeper_ptr;
  if (component_settings.has_status_type()) {
    const std::string& program_status_type = component_settings.status_type();
    VLOG(1) << "  " << program_status_type;
    // Allocate memory for status keeper
    status_keeper_ptr = component_status_keeper_factory.CreateComponentStatusKeeper(
        program_status_type, component_name);
    // Setup a callback for the status keeper status generation. This uses a lambda function.
    status_keeper_ptr->set_component_status_function(
        [&](){return GenerateHappyStatus();} );
  } else {
    LOG(ERROR) << "There is no status_type setting for " << component_name << "in programs_setup.";
    return -1;
  }
  
  // Read a config file
  std::string config_filename =
      "/home/manujnaman/Dropbox/ros/Anantak/ProgramCommander/src/DataQueue/data_queue_00.cfg";
  std::unique_ptr<anantak::DataQueueConfig> config =
      anantak::ReadProtobufFile<anantak::DataQueueConfig>(config_filename);
  VLOG(1) << "Number of sensors: " << config->sensor_size();
  VLOG(1) << "Number of data subscribers: " << config->data_subscriber_size();
  
  // Construct the zmq transport
  zmq::context_t zmq_context(1);
  
  // Create the subscribers and publishers using component settings
  const int n_subs = component_settings.subscription_size();
  const int n_pubs = component_settings.publication_size();
  VLOG(1) << "Number of subscriptions, publications to be created = " << n_subs << ", " << n_pubs;
  // Construct a map of subscribers
  VLOG(1) << "Building a map of subscribers for program = \"" << component_name << "\"";
  std::map<std::string, std::unique_ptr<zmq::socket_t>> subscriptions_map;
  for (int i=0; i<n_subs; i++) {
    const anantak::ProgramsSetup::ProgramSettings::ConnectionSettings& subscription_settings =
        component_settings.subscription(i);
    VLOG(1) << "Sub: " << subscription_settings.name() << " " << subscription_settings.endpoint()
        << " \"" << subscription_settings.subject() << "\"";
    if (subscription_settings.has_type()) {
      if (subscription_settings.type() == anantak::ProgramsSetup::ProgramSettings::COMMAND) {
        command_subscriber_name = subscription_settings.name();
        VLOG(1) << "Command subscriber = " << command_subscriber_name;
      }
      if (subscription_settings.type() == anantak::ProgramsSetup::ProgramSettings::STATUS) {
        status_query_subscriber_name = subscription_settings.name();
        VLOG(1) << "Status query subscriber = " << status_query_subscriber_name;
      }
    }
    std::unique_ptr<zmq::socket_t> ptr(new zmq::socket_t(zmq_context, ZMQ_SUB));
    ptr->connect(subscription_settings.endpoint().c_str());
    ptr->setsockopt(ZMQ_SUBSCRIBE, subscription_settings.subject().c_str(),
                    subscription_settings.subject().length());
    subscriptions_map[subscription_settings.name()] = std::move(ptr);
  }
  // Construct a map of publishers
  VLOG(1) << "Building a map of publications for program = \"" << component_name << "\"";
  std::map<std::string, std::unique_ptr<zmq::socket_t>> publications_map;
  for (int i=0; i<n_pubs; i++) {
    const anantak::ProgramsSetup::ProgramSettings::ConnectionSettings& publications_settings =
        component_settings.publication(i);
    VLOG(1) << "Pub: " << publications_settings.name() << " " << publications_settings.endpoint();
    std::unique_ptr<zmq::socket_t> ptr(new zmq::socket_t(zmq_context, ZMQ_PUB));
    ptr->bind(publications_settings.endpoint().c_str());
    publications_map[publications_settings.name()] = std::move(ptr);
  }
  
  // Check variables and warn
  if (status_query_subscriber_name == "")
    LOG(WARNING) << "status_query_subscriber_name was not found";
  if (command_subscriber_name == "")
    LOG(WARNING) << "command_subscriber_name was not found";
    
  // Start the subscription loop.
  //   Send all StatusQueries to StatusKeeper object.
  //   Process commands as needed.
  bool exit_component = false;
  
  // start inner loop
  while (!exit_component) {
    // type
    typedef std::map<std::string, std::unique_ptr<zmq::socket_t>>::iterator subs_map_iter_type;
    
    // Loop through each subscriber, read messages and pass them to relevant processors
    for (subs_map_iter_type i_map = subscriptions_map.begin(); i_map != subscriptions_map.end(); i_map++) {
      VLOG(3) << "Reading subscriber " << i_map->first;
      
      // Poll this subscriber without waiting
      zmq::message_t message;
      if (i_map->second->recv(&message, ZMQ_DONTWAIT)) {
        
        // Create a string by copying the incoming message data. We want to pass the string over
        //  to keeper objects. When the string is passed to keepers, they are responsible to
        //  destruct it. It will be 'gone' from here. So we use unique_ptr to pass ownership.
        //  We also want to save on time it takes to copy the string from here to the keeper object
        //  as we are no longer copying the string bytes from here to keeper's method. Copying is
        //  only done once from the message_t buffer to a string object buffer. 
        size_t msg_size = message.size();
        const char* msg_data_ptr = reinterpret_cast<char*>(message.data());
        std::unique_ptr<std::string> msg_str_ptr(new std::string(msg_data_ptr, msg_size));
        VLOG(3) << "Message at " << i_map->first << "\n" << *msg_str_ptr;
        
        if (i_map->first == command_subscriber_name) {
          // This is a command
          VLOG(3) << "This is a command.";
          if (*msg_str_ptr == exit_command_str) {
            exit_component = true;
            VLOG(1) << "Exiting component " << component_name;
          }
          
        } else if (i_map->first == status_query_subscriber_name) {
          // This is a status query
          VLOG_EVERY_N(1, 10) << "Got a status query.";
          std::unique_ptr<std::string> status_query_reply_str =
              status_keeper_ptr->GenerateStatusQueryReplyMessage(std::move(msg_str_ptr));
          zmq::message_t status_query_reply_msg(status_query_reply_str->size());
          // copy string buffer to message buffer - this can potentially be avoided using zero-copy
          memcpy(status_query_reply_msg.data(), status_query_reply_str->data(),
                 status_query_reply_str->size());
          bool sent_ok = publications_map.begin()->second->send(status_query_reply_msg, ZMQ_DONTWAIT);
          if (!sent_ok) {
            LOG(ERROR) << "Status Query Reply message was not sent";
          }
          // string and message will be destructed here
        } // if
        
        // Message_String unique pointer destructs here if it has not been moved
      } // recv message
      // Message is destructed here
    } // for 
    
    // Sleep a little if no restarts are needed
    if (!exit_component) {
      int msecs = 10;
      struct timespec t;
      t.tv_sec = msecs / 1000;
      t.tv_nsec = (msecs % 1000) * 1000000;
      nanosleep(&t, NULL);      
    }
    
  } // subscription loop
  
  // Close all subscribers, publishers and zmq transport
  //  this is done by the destructor of the map->unique_ptr->object
  VLOG(1) << "Closing ZMQ transport, subscribers and publishers";

  return 0;
}
