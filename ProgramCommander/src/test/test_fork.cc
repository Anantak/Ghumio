/**
 *  Test Fork
 *
 *  This is to test fork-execve pair on Linux. We want to give an external command, that this
 *  program should run by forking. Next version will recieve the command to be run via a config
 *  file, that will be run by this program. Then final test will be to send a command via ZMQ bus
 *  that will be run externally. It will be crucial to see how ZMQ's context and all file handles
 *  will be closed before a fork is created, then rebuilt after the fork. We want to check if this
 *  process goes smoothly.
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


/** Global variable that holds the status of the child process when child terminates
 *  This should help clean up zombie processes. See http://advancedlinuxprogramming.com/ Ch3
 */
sig_atomic_t child_exit_status;

/** Handler to clean up child process. */
void clean_up_child_process (int signal_number) {
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
int spawn (char* program, char** arg_list) {
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

int main(int argc, char** argv) {
  
  /* Handle SIGCHLD signal by calling clean_up_child_process.  */
  struct sigaction sigchld_action;
  memset (&sigchld_action, 0, sizeof (sigchld_action));
  sigchld_action.sa_handler = &clean_up_child_process;
  sigaction (SIGCHLD, &sigchld_action, NULL);

  /* The argument list to pass to the "ls" command.  */
  char* arg_list[] = {
    "gnome-terminal",     /* argv[0], the name of the program.  */
    "-e", 
    "bash -c \"ls|less\"",
    NULL      /* The argument list must end with a NULL.  */
  };

  /* Spawn a child process.  */
  spawn ("gnome-terminal", arg_list); 

  printf ("Parent is going to sleep\n");
  sleep(1);  // this will not sleep for entire period as a signal comes through
  printf ("Done with the main program\n");

  return 0;
}
