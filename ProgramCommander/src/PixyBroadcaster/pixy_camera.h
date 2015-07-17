/** Pixy Camera
 *  Connects to a pixy camera, provides ability to create pixy messages
 * 
 * Declares a pixy camera object. This can only handle one camera.
 * Pixy.h declares an interface. Pixy.cpp in libpixyusb declares the PixyInterpreter interpreter as a global object.
 * So only one camera can be used by one PixyCamera. For mulitple cameras, one has to create multiple processes.
 */
#pragma once

// std includes
#include <stdio.h>
#include <string>
#include <iostream>
#include <vector>
#include <sstream>
#include <sys/time.h>

// Google logging
#include <glog/logging.h>

/** Protocol Buffers */
#include "configurations.pb.h"
#include "sensor_messages.pb.h"

// Pixy includes
#include "pixy.h"

namespace anantak {

class PixyCamera {
 public:
  
  // Pixy interpreter object
  //  This runs its own thread
  //PixyInterpreter pixy_interpreter_;
  
  std::string symlink_;
  bool pixy_initialized_;
  std::string firmware_version_;
  
  // Camera settings - current values
  int auto_white_balance_;
  int auto_exposure_compensation_;
  uint8_t gain_;
  uint16_t compensation_;
  int brightness_;
  uint32_t white_balance_;
  uint8_t red_, green_, blue_;
  int led_max_current_;

  // Pixy Block buffer
  const static int BLOCK_BUFFER_SIZE = 25;  // Maximum number of blocks that will be copied
  struct Block blocks_[BLOCK_BUFFER_SIZE];
  int blocks_copied;
  char buf[128];
  int64_t blocks_time_;
  
  // Sensor message
  const std::string PIXY_MESSAGE_TYPE;
  anantak::SensorMsg pixy_message_;
  //anantak::PixyCameraMessage pixy_message_;

  // No information constructor - need to initialize after construction
  PixyCamera():
      symlink_(""),
      pixy_initialized_(false),
      blocks_copied(0),
      PIXY_MESSAGE_TYPE("Pixy") {}
  
  // Constructor expects a symlink for the pixy camera
  PixyCamera(const std::string& symlink):
      symlink_(""),
      pixy_initialized_(false),
      blocks_copied(0),
      PIXY_MESSAGE_TYPE("Pixy") {
    Initialize(symlink);
  }
  
  // Connect to the camera and get its settings
  bool Initialize(const std::string& symlink) {
    int return_value = pixy_init_symlink(symlink.c_str());
    if (return_value == 0) {
      symlink_ = symlink;
      pixy_initialized_ = true;
      GetFirmwareVersion();
      GetCameraSettings();
      //SetCameraSettings();
    } else {
      LOG(ERROR) << "Could not initialize pixy camera at symlink = " << symlink;
    }    
    return true;
  }

  // Destructor 
  virtual ~PixyCamera() {
    LOG(INFO) << "Destructing pixy camera";
    if (!pixy_initialized_) {
      LOG(ERROR) << "Pixy camera is not initialized so not closing";
    } else {
      pixy_close();
      LOG(INFO) << "Pixy camera closed.";
    }
  }
  
  // Is the pixy initialized?
  bool IsInitialized() {return pixy_initialized_;}
  
  // Request Pixy firmware version //
  bool GetFirmwareVersion() {
    
    if (!pixy_initialized_) {
      LOG(ERROR) << "Pixy camera is not initialized yet.";
      return false;
    }
    
    uint16_t major;
    uint16_t minor;
    uint16_t build;
    int      return_value;
    return_value = pixy_get_firmware_version(&major, &minor, &build);
    
    if (return_value) {
      // Error //
      //printf("Failed to retrieve Pixy firmware version. ");
      LOG(ERROR) << "Failed to retrieve Pixy firmware version. ";
      pixy_error(return_value);
      return false;
    } else {
      // Success //
      firmware_version_ = std::to_string(major) + "." + std::to_string(minor) +"."+ std::to_string(build);
      LOG(INFO) << " Pixy Firmware Version: " << firmware_version_;
    }
    return true;
  }
  
  // Get camera settings
  bool GetCameraSettings() {
    
    if (!pixy_initialized_) {
      LOG(ERROR) << "Pixy camera is not initialized yet.";
      return false;
    }
    
    brightness_ = pixy_cam_get_brightness();
    printf(" Brightness = %d\n", brightness_);
    
    int rc = pixy_cam_get_exposure_compensation(&gain_, &compensation_);
    printf(" Gain, Compensation = %d %d \n", gain_, compensation_);
    
    white_balance_ = pixy_cam_get_white_balance_value();
    printf(" White Balance = %d\n", white_balance_);
    uint32_t wb = white_balance_;
    green_ = wb & 0xFF;
    wb >> 8;
    red_ = wb & 0xFF;
    wb >> 8;
    blue_ = wb & 0xFF;
    printf("   green, red, blue = %d %d %d\n", int(green_), int(red_), int(blue_));
    
    auto_exposure_compensation_ = pixy_cam_get_auto_exposure_compensation();
    printf(" Auto exposure compensation = %d\n", auto_exposure_compensation_);
    
    auto_white_balance_ = pixy_cam_get_auto_white_balance();
    printf(" Auto white balance = %d\n", auto_white_balance_);
    
    led_max_current_ = pixy_led_get_max_current();
    printf(" LED max current = %d\n", led_max_current_);    
    
    return true;
  }
  
  // Set camera settings - does not work with the older firmware version of 1.0.2. Needs newer firmware
  bool SetCameraSettings() {
    return false;
    if (!pixy_initialized_) {
      LOG(ERROR) << "Pixy camera is not initialized yet.";
      return false;
    }
    uint8_t gain_tgt = 0x2c;
    uint16_t compensation_tgt = 0x0001;
    printf("Setting gain, compensation values to %d %d\n", int(gain_tgt), int(compensation_tgt));
    int rc = pixy_cam_set_exposure_compensation(gain_tgt, compensation_tgt);
    
    GetCameraSettings();
  }
  
  // Get pixy blocks
  bool GetNewBlocks(bool verbose=false) {
    if (!pixy_initialized_) {
      LOG(ERROR) << "Pixy camera is not initialized yet.";
      return false;
    }
    
    while (!pixy_blocks_are_new());
    
    // Get blocks from Pixy //
    blocks_copied = pixy_get_blocks(BLOCK_BUFFER_SIZE, &blocks_[0]);
    
    // Get current timestamp
    struct timeval tv;
    gettimeofday(&tv, NULL);
    blocks_time_ = int64_t(tv.tv_sec * 1000000 + tv.tv_usec);
    
    if (blocks_copied < 0) {
      // Error: pixy_get_blocks //
      LOG(ERROR) << "pixy_get_blocks(): ";
      pixy_error(blocks_copied);
    }
    
    // Display received blocks //
    if (verbose) {
      for (int index = 0; index != blocks_copied; ++index) {
         blocks_[index].print(buf);
         printf(" %d: %s\n", index, buf);
      }
    }
    
    return true;
  }
  
  // Copy block to protocol buffer
  bool CopyBlockToProtoBuf(const Block& block, anantak::PixyCameraMessage::Block* blockmsg) {
    // Set every field
    blockmsg->set_type(uint32_t(block.type));
    blockmsg->set_signature(uint32_t(block.signature));
    blockmsg->set_x(uint32_t(block.x));
    blockmsg->set_y(uint32_t(block.y));
    blockmsg->set_width(uint32_t(block.width));
    blockmsg->set_height(uint32_t(block.height));
    return true;
  }
  
  // Create a pixy message
  bool CreatePixyMessage() {
    // Clean the message
    pixy_message_.Clear();
    anantak::HeaderMsg* hdr_msg = pixy_message_.mutable_header();
    anantak::PixyCameraMessage* pixy_msg = pixy_message_.mutable_pixy_cam_msg();
    
    // Set header
    hdr_msg->set_timestamp(blocks_time_);
    hdr_msg->set_type(PIXY_MESSAGE_TYPE);
    hdr_msg->set_recieve_timestamp(blocks_time_);
    hdr_msg->set_send_timestamp(blocks_time_);
    
    // Set pixy message
    for (int i=0; i<blocks_copied; i++) {
      anantak::PixyCameraMessage::Block* msg_block = pixy_msg->add_blocks();
      CopyBlockToProtoBuf(blocks_[i], msg_block);
    }
    return true;
  }
  
  bool GetNewPixyMessage(bool verbose=false) {
    GetNewBlocks(verbose);
    CreatePixyMessage();
    return true;
  }
  
  std::string PixyCameraMessageToString() {
    std::stringstream ss;
    const anantak::PixyCameraMessage& pixy_msg = pixy_message_.pixy_cam_msg();
    for (int i=0; i<pixy_msg.blocks_size(); i++) {
      const anantak::PixyCameraMessage::Block& block = pixy_msg.blocks(i);
      ss << "[" << block.x() << " " << block.y() << " " << block.width() << " " << block.height() << "] ";
    }
    return ss.str();
  }
  
};

}
