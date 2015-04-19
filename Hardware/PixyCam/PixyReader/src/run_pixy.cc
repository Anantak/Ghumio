/* get data from a pixy using given symlink */

#include <ios>
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <algorithm>
#include <memory>

/** Google libraries */
#include <glog/logging.h>
#include <gflags/gflags.h>

/** Pixy library */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include "pixy.h"


#define BLOCK_BUFFER_SIZE    25

DEFINE_string(symlink, "/dev/PixyCam1", "Symlink to cam generated using a udev rule e.g. /dev/PixyCam1");

// Pixy Block buffer // 
struct Block blocks[BLOCK_BUFFER_SIZE];

static bool run_flag = true;

void handle_SIGINT(int unused) {
  // On CTRL+C - abort! //
  run_flag = false;
}

                 
int main(int argc, char** argv) {
  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);
  // Parse the commandline flags 
  google::ParseCommandLineFlags(&argc, &argv, true);
  // Verify the version of protobuf library 
  //GOOGLE_PROTOBUF_VERIFY_VERSION;
  // Setting the glog output to stderr by default
  FLAGS_logtostderr = 1;
  
  LOG(INFO) << "Provided pixycam symlink = " << FLAGS_symlink;
  
  int      i = 0;
  int      index;
  int      blocks_copied;
  int      pixy_init_status;
  char     buf[128];

  // Catch CTRL+C (SIGINT) signals //
  signal(SIGINT, handle_SIGINT);

  printf("run_pixy :\n libpixyusb Version: %s\n", __LIBPIXY_VERSION__);

  // Connect to Pixy //
  pixy_init_status = pixy_init_symlink(FLAGS_symlink.c_str());

  // Was there an error initializing pixy? //
  if(!pixy_init_status == 0)
  {
    // Error initializing Pixy //
    printf("pixy_init(): ");
    pixy_error(pixy_init_status);

    return pixy_init_status;
  }

  // Request Pixy firmware version //
  {
    uint16_t major;
    uint16_t minor;
    uint16_t build;
    int      return_value;

    return_value = pixy_get_firmware_version(&major, &minor, &build);

    if (return_value) {
      // Error //
      printf("Failed to retrieve Pixy firmware version. ");
      pixy_error(return_value);

      return return_value;
    } else {
      // Success //
      printf(" Pixy Firmware Version: %d.%d.%d\n", major, minor, build);
    }
  }

#if 1
  // Pixy Command Examples //
  {
    int32_t response;
    int     return_value;

    // Execute remote procedure call "cam_setAWB" with one output (host->pixy) parameter (Value = 1)
    //
    //   Parameters:                 Notes:
    //
    //   pixy_command("cam_setAWB",  String identifier for remote procedure
    //                        0x01,  Length (in bytes) of first output parameter
    //                           1,  Value of first output parameter
    //                           0,  Parameter list seperator token (See value of: END_OUT_ARGS)
    //                   &response,  Pointer to memory address for return value from remote procedure call
    //                           0); Parameter list seperator token (See value of: END_IN_ARGS)
    //

    // Enable auto white balance //
    //pixy_command("cam_setAWB", UINT8(0x01), END_OUT_ARGS,  &response, END_IN_ARGS);

    // Execute remote procedure call "cam_getAWB" with no output (host->pixy) parameters
    //
    //   Parameters:                 Notes:
    //
    //   pixy_command("cam_setAWB",  String identifier for remote procedure
    //                           0,  Parameter list seperator token (See value of: END_OUT_ARGS)
    //                   &response,  Pointer to memory address for return value from remote procedure call
    //                           0); Parameter list seperator token (See value of: END_IN_ARGS)
    //

    // Get auto white balance //
    return_value = pixy_command("cam_getAWB", END_OUT_ARGS, &response, END_IN_ARGS);
    printf(" AWB: 0x%x 0x%x\n", response, return_value);
    
    int brightness = pixy_cam_get_brightness();
    printf(" Brightness = %d\n", brightness);
    
    uint8_t gain;
    uint16_t compensation;
    int rc = pixy_cam_get_exposure_compensation(&gain, &compensation);
    printf(" Gain, Compensation = %d %d \n", gain, compensation);
    
    uint32_t white_balance = pixy_cam_get_white_balance_value();
    printf(" White Balance = %d\n", white_balance);    

    // Set auto white balance back to disabled //
    //pixy_command("cam_setAWB", UINT8(0x00), END_OUT_ARGS,  &response, END_IN_ARGS);
  }
#endif

  printf("Detecting blocks...\n");
  //while(false)
  while(run_flag)
  {
    // Wait for new blocks to be available //
    while(!pixy_blocks_are_new() && run_flag); 

    // Get blocks from Pixy //
    blocks_copied = pixy_get_blocks(BLOCK_BUFFER_SIZE, &blocks[0]);

    if(blocks_copied < 0) {
      // Error: pixy_get_blocks //
      printf("pixy_get_blocks(): ");
      pixy_error(blocks_copied);
    }

    // Display received blocks //
    //printf("frame %d:\n", i);
    for (index = 0; index != blocks_copied; ++index) {    
       blocks[index].print(buf);
       printf(" %d: %s\n", index, buf);
    }
    //i++;
  }
  pixy_close();



  return 0;
}


