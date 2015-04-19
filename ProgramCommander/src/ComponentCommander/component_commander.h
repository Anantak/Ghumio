/**
 *  Component Commander
 *
 *  Starts all components of the system and keeps their status.
 *  
 */


#pragma once

// std includes
#include <string>
#include <cstdint>
#include <memory>
#include <vector>
#include <map>

// include other headers
#include "ComponentCommander/component_status_keeper.h"
#include "ComponentCommander/component_status_keeper_factory.h"

// Include protocol buffers
#include "programs_setup.pb.h"

namespace anantak {
  
/**
 * Component Commander
 */
class ComponentCommander {
 public:
  /**
   * Load a list of components to keep track of.
   * Create a map of each component name and their StatusKeeperObjects.
   * Create a map of each component name and their next status poll time set to -1.
   * Create a vector for program names to startup, set to an empty list. 
   */
  ComponentCommander(
      std::string this_program_name,              /**< name of this program */
      std::string programs_setup_config_filename, /**< filename from where to load the config */
      std::string command_subscriber_name,        /**< name of command subscriber */
      std::string exit_command_str,               /**< command string that exits this program */
      std::string status_query_subscriber_name    /**< status query subscirber name */
  );
  
  /** Destructs the ComponentCommander */
  virtual ~ComponentCommander();

  /** Read the setup of the programs */
  bool ReadProgramsSetup(
      std::string config_filename,            /**< ProgramsSettings protobuf text format filename */
      anantak::ProgramsSetup* programs_setup  /**< Pointer to programs setup object */
  );

  /** Spawn a child process running a new program. PROGRAM is the name
   *  of the program to run; the path will be searched for this program.
   *  ARG_LIST is a NULL-terminated list of character strings to be
   *  passed as the program's argument list. Returns the process id of
   *  the spawned process. All file handles and zmq transports should be
   *  already closed when a child process is spawned.
   */
  int SpawnChildProcess(char* program, char** arg_list);

  /** Initialize - Build maps of Status Keeper and other objects */
  bool Initialize();
  
  /** Start listening for messages in the inner loop */
  bool StartListening();
  
  /** Start the outer loop */
  bool StartLooping();
  
  /** Start all components asked to be started */
  bool StartChildProcesses();
  
 private:
  
  /** Name of this program - this is used to identify the settings of this program in config file */
  std::string this_program_name_;
  
  /** Programs setup filename */
  std::string programs_setup_config_filename_;
  
  /** Name of the command subscriber used to identify which messages are commands */
  std::string command_subscriber_name_;
  
  /** Name of the subscriber that listens to the StatusQueries for ProgramCommander */
  std::string status_query_subscriber_name_;
  
  /** Exit command string */
  std::string exit_command_str_;
  
  /** Programs setup protobuf object */
  anantak::ProgramsSetup programs_setup_;
  
  /** Program settings for this program from the config file */
  anantak::ProgramsSetup::ProgramSettings this_program_settings_;
  
  /** Status Keeper Factory to create StatusKeepers */
  anantak::ComponentStatusKeeperFactory component_status_keeper_factory_;
  
  /** Status Keepers Map - one for each component to be managed
   *  We keep unique_ptr to status keeper objects as we want to have objects derived from keepers
   *  here too. Keeping pointers in place of objects itself saves us from the problem of 'slicing',
   *  where derived object gets 'sliced' to only the super class' size. */
  std::map<std::string, std::unique_ptr<anantak::ComponentStatusKeeper>> status_keepers_map_;
  
  /** Auto start or Manual start map */
  std::map<std::string, bool> auto_startup_map_;
  
  /** Next Status Query Time Map - one for each component */
  std::map<std::string, int64_t> next_status_query_time_map_;
  
  /** Programs to be restarted */
  std::vector<std::string> components_to_restart_;
  
  /** Flags for indicating exit from the loops */
  bool exit_program_commander_;
  bool start_some_programs_;

  /** Get current wall time in microseconds */
  int64_t get_wall_time_microsec();    /**< Returns current wall time in microseconds */
  
  /** Display panel functions and variables */
  bool GenerateDisplayPanels();     /**< Create display panels for each component */
  std::string panel_save_dir_;      /**< Directory location where panels will be saved */
  bool CleanDirectory(std::string dir_path);  /** Utility function to delete all files in dir */
  
};

  
}  // namespace anantak
