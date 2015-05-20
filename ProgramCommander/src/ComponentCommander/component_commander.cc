/**
 *  Component Commander
 *
 *  Starts all components of the system and keeps their status.
 *  
 */

#include "ComponentCommander/component_commander.h"

/** Spawning a child process */
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/time.h>
#include <unistd.h>

/** Google Logging library */
#include <glog/logging.h>

/** Google Flags library */
#include <gflags/gflags.h>

/** Google Protocol Buffers library */
#include <fcntl.h>
#include <google/protobuf/text_format.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>

/** ZMQ includes */
#include <zmq.hpp>

/** Display system includes */
#include <dirent.h>
#include <fstream>

namespace anantak {
  
/**
 * Component Commander
 */

/**
 * Load a programs setup to get the new things working.
 * Create a map of each component name and their StatusKeeperObjects.
 * Create a map of each component name and their next status poll time set to -1.
 * Create a vector for program names to startup, set to an empty list. 
 */
ComponentCommander::ComponentCommander(
    std::string this_program_name,              /**< name of this program */
    std::string programs_setup_config_filename, /**< filename from where to load the config */
    std::string command_subscriber_name,        /**< name of command subscriber */
    std::string exit_command_str,               /**< exit string */
    std::string status_query_subscriber_name    /**< susbcriber name for the status query */
) {
  this_program_name_ = this_program_name;
  programs_setup_config_filename_ = programs_setup_config_filename;
  command_subscriber_name_ = command_subscriber_name;
  exit_command_str_ = exit_command_str;
  status_query_subscriber_name_ = status_query_subscriber_name;
  // Read config file and create the ProgramsSetup object
  ReadProgramsSetup(programs_setup_config_filename_, &programs_setup_);
  panel_save_dir_ = "/src/ComponentCommander/ComponentPanels";
  // Build the components map 
  Initialize();
}

/** Destructs the ComponentCommander */
ComponentCommander::~ComponentCommander() {}

/** Read the setup of the programs */
bool ComponentCommander::ReadProgramsSetup(
  std::string config_filename,            /**< ProgramsSettings protobuf text format filename */
  anantak::ProgramsSetup* programs_setup  /**< programs setup object to be filled up */
) {
  // open the file
  LOG(INFO) << "Reading the file " << config_filename;
  int in_file_descriptor = open(config_filename.c_str(), O_RDONLY);
  if (in_file_descriptor < 0) {
    LOG(ERROR) << " Error opening the file: " << strerror(errno);
    return false;
  }
  // parse the file
  google::protobuf::io::FileInputStream file_input_stream(in_file_descriptor);
  google::protobuf::TextFormat::Parse(&file_input_stream, programs_setup);
  file_input_stream.Close();
  // convert to string to display
  std::string in_programs_setup_str;
  google::protobuf::TextFormat::PrintToString(*programs_setup, &in_programs_setup_str);
  VLOG(2) << "in_programs_setup = \n" << in_programs_setup_str;
  return true;
}

/** Spawn a child process running a new program. PROGRAM is the name
 *  of the program to run; the path will be searched for this program.
 *  ARG_LIST is a NULL-terminated list of character strings to be
 *  passed as the program's argument list. Returns the process id of
 *  the spawned process. All file handles and zmq transports should be
 *  already closed when a child process is spawned.
 */
int ComponentCommander::SpawnChildProcess(char* program, char** arg_list) {
  pid_t child_pid;
  /* Duplicate this process.  */
  child_pid = fork ();
  if (child_pid != 0)
    /* This is the parent process.  */
    return child_pid;
  else {
    /* Now execute PROGRAM, searching for it in the path.  */
    execvp (program, arg_list);
    /* The execvp function returns only if an error occurs.  */
    fprintf (stderr, "an error occurred in execvp\n");
    abort();
  }
}

/** Initialize - Build a map of Status Keeper objects */
bool ComponentCommander::Initialize() {
  // Go through the list of components to be created
  LOG(INFO) << "Building a map of Component Status Keepers";
  for (int i=0; i<programs_setup_.program_settings_size(); i++) {
    const anantak::ProgramsSetup::ProgramSettings& program_settings =
      programs_setup_.program_settings(i);
    const std::string& program_name = program_settings.name();
    VLOG(1) << program_name;
    if (program_settings.has_status_type()) {
      const std::string& program_status_type = program_settings.status_type();
      VLOG(1) << "  " << program_status_type;
      
      // Allocate memory for status keeper
      if (program_settings.has_component_template()) {
        LOG(INFO) << "Using component template = " << program_settings.component_template();
        std::unique_ptr<anantak::ComponentStatusKeeper> temp_ptr(new anantak::ComponentStatusKeeper(program_name,
            program_settings.component_template()));
        status_keepers_map_[program_name] = std::move(temp_ptr);   // class member pointer now owns the keeper
      } else {
        std::unique_ptr<anantak::ComponentStatusKeeper> temp_ptr(new anantak::ComponentStatusKeeper(program_name));
        status_keepers_map_[program_name] = std::move(temp_ptr);   // class member pointer now owns the keeper      
      }
      
      // Allocate memory for status keeper
      //std::unique_ptr<anantak::ComponentStatusKeeper> ptr =
      //  component_status_keeper_factory_.CreateComponentStatusKeeper(
      //      program_status_type, program_name);
      //// Transfer object ptr to map. ptr will be 'gone' after move
      //status_keepers_map_[program_name] = std::move(ptr);
      
      // Initiate the next query poll time to -1
      next_status_query_time_map_[program_name] = -1;   // this is not used
      // Startup set to automatic if startup_direction == "auto start"
      if (program_settings.has_startup_direction()) {
        // Initiate the startup direction map
        const std::string& startup_direction = program_settings.startup_direction();
        auto_startup_map_[program_name] = (startup_direction == "auto start");
        VLOG(3) << program_name << " is auto start = " << auto_startup_map_[program_name];
        status_keepers_map_[program_name]->set_auto_restart_component(auto_startup_map_[program_name]);
      }
      // Stagger the startup times
      status_keepers_map_[program_name]->set_status_query_time_offset(
          i, programs_setup_.program_settings_size());  // offset = query interval * i / n_programs
    } // has_status_type
    if (program_name == this_program_name_) {
      VLOG(1) << "Found settings for " << program_name;
      this_program_settings_ = program_settings;
    }
  } // for
  // Make sure that vector of programs to be restarted are NULL
  components_to_restart_.clear();
  // Display some information about the map
  LOG(INFO) << "Number of entries in the maps = " << status_keepers_map_.size() << ", "
    << next_status_query_time_map_.size() << ", " << components_to_restart_.size();
  // Set the looping flags
  exit_program_commander_ = false;
  start_some_programs_ = false;
  // Other basic things
  GenerateDisplayPanels();
  return true;
}

/** Delete all files in the directory */
bool ComponentCommander::CleanDirectory(std::string dir_path) {
  // These are data types defined in the "dirent" header
  struct dirent *next_file;
  DIR *theFolder;  
  char filepath[256];
  theFolder = opendir(dir_path.c_str());
  int num_files_removed = 0;
  while (next_file = readdir(theFolder)) {
    // we always get . and .., simply ignore these
    if (strcmp(next_file->d_name, ".") && strcmp(next_file->d_name, "..")) {
      // build the full path for each file in the folder
      sprintf(filepath, "%s/%s", dir_path.c_str(), next_file->d_name);
      VLOG(1) << "Removing file " << filepath;
      remove(filepath);
      ++num_files_removed;
    }
  }
  VLOG(1) << "Removed " << num_files_removed << " files";
  return true;
}

/** Create display panels for each component */
bool ComponentCommander::GenerateDisplayPanels() {
  // Go through each keeper and call their GenerateDisplayPanel() function.
  // Save all panels in the given directory after cleaning the directory of current panels.
  std::string panels_dir = anantak::GetProjectSourceDirectory() + panel_save_dir_;
  VLOG(1) << "Removing all current panels from " << panels_dir;
  CleanDirectory(panels_dir);
  VLOG(1) << "Generating display panels for each keeper";
  typedef std::map<std::string, std::unique_ptr<anantak::ComponentStatusKeeper>>::iterator
      keepers_map_iter_type;  
  for (keepers_map_iter_type i_kmap = status_keepers_map_.begin();
       i_kmap != status_keepers_map_.end(); i_kmap++) {
    if (i_kmap->second) {
      // Call GenerateDisplayPanel function
      std::unique_ptr<std::string> display_panel_str;
      display_panel_str = i_kmap->second->GenerateDisplayPanel();
      std::string panel_filename = panels_dir + "/" + i_kmap->first + ".html";
      VLOG(1) << "Saving " << panel_filename;
      std::ofstream out(panel_filename);
      out << *display_panel_str;
      out.close();
    } // if keeper exists
  } // for all keepers 
}


/** Start the outerloop that constructs the ZMQ transport and listens to messages */
bool ComponentCommander::StartListening() {
  // Report the number of subscribers and publishers to be created
  const int n_subs = this_program_settings_.subscription_size();
  const int n_pubs = this_program_settings_.publication_size();
  VLOG(1) << "Number of subscriptions, publications to be created = " << n_subs << ", " << n_pubs;
  // construct the zmq transport
  zmq::context_t zmq_context(1);
  // construct a map of subscribers
  VLOG(3) << "Building a map of subscribers for program = \"" << this_program_name_ << "\"";
  std::map<std::string, std::unique_ptr<zmq::socket_t>> subscriptions_map;
  for (int i=0; i<n_subs; i++) {
    const anantak::ProgramsSetup::ProgramSettings::ConnectionSettings& subscription_settings =
        this_program_settings_.subscription(i);
    VLOG(3) << "Sub: " << subscription_settings.name() << " " << subscription_settings.endpoint()
        << " \"" << subscription_settings.subject() << "\"";
    std::unique_ptr<zmq::socket_t> ptr(new zmq::socket_t(zmq_context, ZMQ_SUB));
    ptr->connect(subscription_settings.endpoint().c_str());
    ptr->setsockopt(ZMQ_SUBSCRIBE, subscription_settings.subject().c_str(),
                    subscription_settings.subject().length());
    subscriptions_map[subscription_settings.name()] = std::move(ptr);
    // find the subscriber in the list of programs
    if (status_keepers_map_.find(subscription_settings.name()) == status_keepers_map_.end()) {
      LOG(INFO) << "Could not find " << subscription_settings.name() << " in programs list";
    } else {
      VLOG(3) << "Found " << subscription_settings.name() << " in programs list";
    }
  }  
  // construct a map of publishers
  VLOG(3) << "Building a map of publications for program = \"" << this_program_name_ << "\"";
  std::map<std::string, std::unique_ptr<zmq::socket_t>> publications_map;
  for (int i=0; i<n_pubs; i++) {
    const anantak::ProgramsSetup::ProgramSettings::ConnectionSettings& publications_settings =
        this_program_settings_.publication(i);
    VLOG(3) << "Pub: " << publications_settings.name() << " " << publications_settings.endpoint()
        << " \"" << publications_settings.subject() << "\"";
    std::unique_ptr<zmq::socket_t> ptr(new zmq::socket_t(zmq_context, ZMQ_PUB));
    ptr->bind(publications_settings.endpoint().c_str());
    publications_map[publications_settings.name()] = std::move(ptr);
  }
  
  // typedefs
  typedef std::map<std::string, std::unique_ptr<zmq::socket_t>>::iterator subs_map_iter_type;
  typedef std::map<std::string, std::unique_ptr<anantak::ComponentStatusKeeper>>::iterator keepers_map_iter_type;
  
  // start inner loop
  while (!exit_program_commander_ && !start_some_programs_) {
    
    // Loop through each subscriber, read messages and pass them to status keeper objects
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
        const char* msg_data_ptr = static_cast<char*>(message.data());
        std::unique_ptr<std::string> msg_str_ptr(new std::string(msg_data_ptr, msg_size));
        VLOG(2) << "Message at " << i_map->first << "\n" << *msg_str_ptr;
        
        // If subscriber is the command subscriber, send the recieved command to all status keepers
        //  otherwise check if the subscriber exists in the list of programs. Send to its keeper.
        if (i_map->first == command_subscriber_name_) {
          // This message is a command
          // check if this is to exit this program
          if (*msg_str_ptr == exit_command_str_) {
            exit_program_commander_ = true;
          }
          VLOG(3) << "This is a command. Shipping to all keepers.";
          // iterate through all keepers
          for (keepers_map_iter_type i_kmap = status_keepers_map_.begin();
               i_kmap != status_keepers_map_.end(); i_kmap++) {
            if (i_kmap->second) {
              // copy the command string and send to call command
              std::unique_ptr<std::string> msg_str_ptr_copy(new std::string(*msg_str_ptr));
              i_kmap->second->ProcessCommand(std::move(msg_str_ptr_copy));
            } // if keeper exists
          } // for all keepers
          
        } else if (i_map->first == status_query_subscriber_name_) {
          // This is a status query
          VLOG(3) << "This is a status query. Shipping to keeper for " << this_program_name_;
          keepers_map_iter_type i_keeper = status_keepers_map_.find(this_program_name_);
          if (i_keeper != status_keepers_map_.end()) {
            if (i_keeper->second) {
              VLOG(3) << "Found keeper for this program. Generating status reply.";
              std::unique_ptr<std::string> status_query_reply_str =
                  i_keeper->second->GenerateStatusQueryReplyMessage(std::move(msg_str_ptr));
              VLOG(3) << "Sending: " << *status_query_reply_str;
              zmq::message_t status_query_reply_msg(status_query_reply_str->size());
              // Copy string buffer to message buffer - this can potentially be avoided using zero-copy
              memcpy(status_query_reply_msg.data(), status_query_reply_str->data(),
                     status_query_reply_str->size());
              bool sent_ok = publications_map.begin()->second->send(status_query_reply_msg, ZMQ_DONTWAIT);
              if (!sent_ok) {
                LOG(ERROR) << "Status Query Reply message was not sent";
              }
              // String and message will be destructed here
            } // if keeper exists
          } // if found keeper
          
        } else {
          // This is a status query reply
          VLOG(3) << "This is a status reply.";
          // Check if this subscriber name exists in the keepers, ship message if found
          keepers_map_iter_type i_keeper = status_keepers_map_.find(i_map->first);
          if (i_keeper != status_keepers_map_.end()) {
            if (i_keeper->second) {
              VLOG(3) << "Found keeper for subscriber. Shipping message.";
              i_keeper->second->ProcessStatusReply(std::move(msg_str_ptr));
            }
          } // if this is a keeper
        } // if else subscriber type
        
        // Message_String unique pointer destructs here if it has not been moved
      } // recv message
      // Message is destructed here
    } // for 
    
    // use this time for decision making for component status times
    int64_t current_time = get_wall_time_microsec();
    
    // ask each keeper if it is time to poll their component,
    //  if so, ask the keeper for a query message, send the message using the publisher
    for (keepers_map_iter_type i_kmap = status_keepers_map_.begin();
         i_kmap != status_keepers_map_.end(); i_kmap++) {
      if (i_kmap->second) {
        bool poll_now = i_kmap->second->SendStatusQueryNow(current_time);
        if (poll_now) {
          std::unique_ptr<std::string> status_query_str = i_kmap->second->GenerateStatusQueryMessage();
          VLOG(3) << "Sending: " << *status_query_str;
          zmq::message_t status_query_msg(status_query_str->size());
          // copy string buffer to message buffer - this can potentially be avoided using zero-copy
          memcpy(status_query_msg.data(), status_query_str->data(), status_query_str->size());
          bool sent_ok = publications_map.begin()->second->send(status_query_msg, ZMQ_DONTWAIT);
          if (!sent_ok) {
            LOG(ERROR) << "Status Query message was not sent";
          }
          // string and message will be destructed here
        }
      } // if keeper exists
    }
    
    // ask each subscriber if the component needs to be started,
    //  if so, add the name of the component to the to-be-started list. set the global flag.
    for (keepers_map_iter_type i_kmap = status_keepers_map_.begin();
         i_kmap != status_keepers_map_.end(); i_kmap++) {
      if (i_kmap->second) {
        bool is_manual_start = false;
        bool start_now = i_kmap->second->StartComponentNow(current_time, &is_manual_start);
        if (start_now) {
          VLOG(1) << "Got a signal to start " << i_kmap->first << " manual_start = " << is_manual_start;
          VLOG(1) << i_kmap->first << " is set to auto_start " << auto_startup_map_[i_kmap->first];
          // Add this program to the list of programs to be restarted. Only when the program is set
          // to manual startup but signal is an auto start, we do not start the program.
          if (!(!auto_startup_map_[i_kmap->first] && !is_manual_start)) {
            VLOG(1) << "Adding " << i_kmap->first << " for restart";
            components_to_restart_.push_back(i_kmap->first);
            start_some_programs_ = true;
          }
        } // if poll_now
      } // if keeper exists
    } // for loop for sending status query messages    
    
    // ask each keeper if it is time to send out a status 
    //  if so, ask the keeper to generate the status and publish it out
    for (keepers_map_iter_type i_kmap = status_keepers_map_.begin();
         i_kmap != status_keepers_map_.end(); i_kmap++) {
      if (i_kmap->second) {
        bool send_status_now = i_kmap->second->SendStatusNow(current_time);
        if (send_status_now) {
          std::unique_ptr<std::string> status_str = i_kmap->second->GenerateStatusMessage();
          VLOG(3) << "Sending: " << *status_str;
          zmq::message_t status_msg(status_str->size());
          // copy string buffer to message buffer - this can potentially be avoided using zero-copy
          memcpy(status_msg.data(), status_str->data(), status_str->size());
          bool sent_ok = publications_map.begin()->second->send(status_msg, ZMQ_DONTWAIT);
          if (!sent_ok) {
            LOG(ERROR) << "Status message was not sent";
          } 
          // string and message will be destructed here
        }
      } // if keeper exists
    }
    
    // Sleep a little if no restarts are needed
    if (!start_some_programs_ && !exit_program_commander_) {
      int msecs = 100;
      struct timespec t;
      t.tv_sec = msecs / 1000;
      t.tv_nsec = (msecs % 1000) * 1000000;
      nanosleep(&t, NULL);      
    }
    
  } // inner loop
  
  // Close all subscribers, publishers and zmq transport
  //  this is done by the destructor of the map->unique_ptr->object
  VLOG(1) << "Closing ZMQ transport, subscribers and publishers";
  
} // StartListening

bool ComponentCommander::StartChildProcesses() {
  // check if each program is to be started
  std::string terminal_command[3] = {"gnome-terminal", "-e", ""};
  for (int i=0; i<programs_setup_.program_settings_size(); i++) {
    const anantak::ProgramsSetup::ProgramSettings& program_settings = programs_setup_.program_settings(i);
    const std::string& program_name = program_settings.name();
    if (program_settings.has_command_line()) {
      bool to_be_started = (std::find(components_to_restart_.begin(), components_to_restart_.end(),
                                      program_name) != components_to_restart_.end());
      if (to_be_started) {
        const std::string& program_command_line = program_settings.command_line();
        VLOG(1) << "Starting " << program_name << " with cmdline = " << program_command_line;
        /* Arguments list to be run with execve */
        char* arg_list[4];
        arg_list[0] = const_cast<char*>(terminal_command[0].c_str()); /* argv[0], the name of the program.  */
        arg_list[1] = const_cast<char*>(terminal_command[1].c_str()); 
        arg_list[2] = const_cast<char*>(program_command_line.c_str()); 
        arg_list[3] = NULL;                                           /* The argument list ends with a NULL.  */
        /* Spawn a child process.  */
        SpawnChildProcess(arg_list[0], arg_list);
      } // to be started
    } // if command line is present
  } // for each program
  return true;
}

bool ComponentCommander::StartLooping() {
  // loop while we do not have to exit
  int counter = 0;
  while (!exit_program_commander_) {
    // Start the inner loop
    StartListening();
    // Start the programs in the list
    StartChildProcesses();
    // Clear the buffers
    components_to_restart_.clear();
    start_some_programs_ = false;
    
    // Temporary for now
    counter++;
    if (counter > 200) exit_program_commander_ = true;
  }
}

int64_t ComponentCommander::get_wall_time_microsec() {
  struct timeval tv;
  gettimeofday (&tv, NULL);
  return (int64_t) (tv.tv_sec * 1000000 + tv.tv_usec);  
}

}  // namespace anantak
