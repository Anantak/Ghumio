/**
 *  Test Proto Fork
 *
 *  This is to test fork-execve pair on Linux. We want to give an external command, that this
 *  program should run by forking. Next version will recieve the command to be run via a config
 *  file, that will be run by this program. Then final test will be to send a command via ZMQ bus
 *  that will be run externally. It will be crucial to see how ZMQ's context and all file handles
 *  will be closed before a fork is created, then rebuilt after the fork. We want to check if this
 *  process goes smoothly.
 *
 *  Protobuf library is used to supply the configuration. Which commands? etc.
 *
 *  Started code from: Code listing from "Advanced Linux Programming," by CodeSourcery LLC
 *  
 */


/** Includes for spawning a child process */
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <unistd.h>

/** Includes for child process signal handling */
#include <signal.h>
#include <string.h>
#include <sys/wait.h>

/** Includes for Google Logging library */
#include <glog/logging.h>

/** Includes for Google Flags library */
#include <gflags/gflags.h>

/** Includes for Google Protocol Buffers library */
#include <fcntl.h>
#include <google/protobuf/text_format.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>

// Include protocol buffers
#include "program_commander.pb.h"

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

/** Spawn a child process running a new program. PROGRAM is the name
 *  of the program to run; the path will be searched for this program.
 *  ARG_LIST is a NULL-terminated list of character strings to be
 *  passed as the program's argument list. Returns the process id of
 *  the spawned process. */
int SpawnChildProcess (char* program, char** arg_list) {
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

/** CreateDummyProgramsSetup
 * This creates a dummy protobuf file. It is used to undestand the file format. This will then be
 * modified to create a real config file.
 */
bool CreateDummyProgramsSetup(std::string dummy_filename) {

  // Create a ProgramsSetup object
  anantak::ProgramsSetup programs_setup;
  programs_setup.set_name("GhumioPrograms");
  //  add a program
  anantak::ProgramsSetup::ProgramSettings* program_00 = programs_setup.add_program_settings();
  program_00->set_name("program00");
  program_00->set_command_line("bash -c \"ls; sleep 1\"");
  //  add a program
  anantak::ProgramsSetup::ProgramSettings* program_01 = programs_setup.add_program_settings();
  program_01->set_name("program01");
  program_01->set_command_line("bash -c \"ls -la; sleep 1\"");

  // display the text format of the object
  std::string programs_setup_str;
  google::protobuf::TextFormat::PrintToString(programs_setup, &programs_setup_str);
  LOG(INFO) << "programs_setup_str = \n" << programs_setup_str;
  
  // write to disk in text_format
  int file_descriptor = open(dummy_filename.c_str(), O_WRONLY | O_CREAT, S_IRWXU);
  if (file_descriptor < 0) {
    LOG(ERROR) << " Error opening the file: " << strerror(errno);
    return false;
  }
  LOG(INFO) << "Writing the message out to file " << dummy_filename;
  google::protobuf::io::FileOutputStream file_output_stream(file_descriptor);
  google::protobuf::TextFormat::Print(programs_setup, &file_output_stream);
  file_output_stream.Close();
  return true;
}

bool ReadProgramsSetup(
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
  LOG(INFO) << "in_programs_setup = \n" << in_programs_setup_str;
  return true;
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

  /** Create a dummy ProgramsSetup file. It helps check the format of the text formal file.  */
  std::string test_ouput_filename = "../src/test/test.cfg";
  CreateDummyProgramsSetup(test_ouput_filename);

  /** Read the protocol buffers for ProgramsSetup. This protobuf contains all the programs that
   * are to be run on the machine, along with the command lines used to run them.
   */
  std::string programs_setup_filename = "../src/test/test_node.cfg";
  anantak::ProgramsSetup programs_setup;
  ReadProgramsSetup(programs_setup_filename, &programs_setup);

  /** When we spawn a child, we intend to close all external connections. This includes file
   * handles and zmq transports. This makes sure that the child starts with a clean slate and
   * there is no corruption of zmq transports/ file handles that are not thread safe. Main loop
   * keeps going in a loop till an exit command comes in via ZMQ. When a command to start a program
   * comes through, ProgramCommander first checks if the program to be started is already running.
   * If so, it does not issues the start program command. If it decides that target program is not
   * running, it closes all its transports, and exits the zmq message-listening loop. It returns
   * to the main program with a message containing the command to be run. Main loop spawns the
   * program, then returns to the zmq listening loop. 
   */
  
  std::string terminal_command[3] = {"gnome-terminal", "-e", ""};
  
  // Launch programs in a loop
  for (int i=0; i<programs_setup.program_settings_size(); i++) {
    
    const anantak::ProgramsSetup::ProgramSettings& program_settings = programs_setup.program_settings(i);
    
    if (program_settings.has_startup_direction()) {
      const std::string& startup_direction = program_settings.startup_direction();
      if (startup_direction=="run on startup") {
        const std::string& program_command_line = program_settings.command_line();
        
        /* Arguments list to be run with execve */
        char* arg_list[4];
        arg_list[0] = const_cast<char*>(terminal_command[0].c_str()); /* argv[0], the name of the program.  */
        arg_list[1] = const_cast<char*>(terminal_command[1].c_str()); 
        arg_list[2] = const_cast<char*>(program_command_line.c_str()); 
        arg_list[3] = NULL;                                           /* The argument list ends with a NULL.  */
      
        /* Spawn a child process.  */
        SpawnChildProcess(arg_list[0], arg_list);
      }
    }
  }
  
  printf ("Parent is going to sleep\n");
  sleep(1);  // this will not sleep for entire period as a signal comes through
  printf ("Done with the main program\n");

  return 0;
}
