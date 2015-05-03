/**
 *  Test Component Commander
 *
 */


/** Includes for child process signal handling */
#include <signal.h>
#include <string.h>
#include <sys/wait.h>

/** Includes for Google libraries */
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <google/protobuf/message.h>

/** anantak header includes */
#include "ComponentCommander/component_commander.h"

// Config file to use
DEFINE_string(config_filename, "../src/test/test_node.cfg",
    "Configuration file to use for this commander");

/** Global variable that holds the status of the child process when child terminates
 *  This should help clean up zombie processes. See http://advancedlinuxprogramming.com/ Ch3
 */
sig_atomic_t child_exit_status;

/** Handler to clean up child process. */
void CleanUpChildProcess (int signal_number) {
  printf ("SIGCHLD recieved.");
  int status;
  wait (&status);
  /* Store its exit status in a global variable.  */
  sig_atomic_t child_exit_status;
  child_exit_status = status;
  printf (" Finished cleaning up the child process.\n");
}

int main(int argc, char** argv) {
  
  /** Register the call-back that handles SIGCHLD signal.
   *  This calls CleanUpChildProcess.
   */
  struct sigaction sigchld_action;
  memset (&sigchld_action, 0, sizeof (sigchld_action));
  sigchld_action.sa_handler = &CleanUpChildProcess;
  sigaction (SIGCHLD, &sigchld_action, NULL);
  
  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);
  // Parse the commandline flags 
  google::ParseCommandLineFlags(&argc, &argv, true);
  // Verify the version of protobuf library 
  GOOGLE_PROTOBUF_VERIFY_VERSION;
  // Setting the glog output to stderr by default
  FLAGS_logtostderr = 1;

  /** Instantiate the Component Commander */
  std::string program_name = "ProgramCommander";
  std::string programs_setup_filename = FLAGS_config_filename;
  std::string command_subscriber_name = "ManualCommands";
  std::string exit_command = "COMMAND ProgramCommander exit";
  std::string status_query_subscriber_name = "ProgramCommanderStatusQueries";
  anantak::ComponentCommander component_commander(
      program_name, programs_setup_filename, command_subscriber_name, exit_command,
      status_query_subscriber_name);
  
  /** Start the ComponentCommander's main loop */
  component_commander.StartLooping();
  
  return 0;
}
