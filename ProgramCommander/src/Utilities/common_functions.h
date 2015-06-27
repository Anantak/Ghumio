/**
 *  Common functions used by many programs
 *   - Read setup config file
 *   - Read program specific config file
 */

#pragma once

/** std includes */
#include <string>
#include <memory>

/** Google Protocol Buffers library */
#include <fcntl.h>
#include <google/protobuf/message.h>
#include <google/protobuf/text_format.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>

/** Google libraries */
#include <glog/logging.h>

/** Protocol buffers */
#include "programs_setup.pb.h"

namespace anantak {

/** Read Programs Setup
 *  Reads programs setup file - this is used by many programs to read pub/sub zmq setups
 */
bool ReadProgramsSetup(std::string config_filename, /**< Protobuf text format filename */
    anantak::ProgramsSetup* programs_setup);        /**< Programs setup object to be filled up */

/** Converts the microseconds to dd:mm:ss:ms string representation
 *  I dont know if this is the most efficient way of doing this, but might be
 */
inline std::string microsec_to_time_str(int64_t musec) {
  std::lldiv_t us = std::lldiv( musec, 1000 );    // quot=time in millisec, rem=neglect
  std::lldiv_t ms = std::lldiv( us.quot, 1000 );  // quot=time in sec, rem=millisec left
  std::lldiv_t ss = std::lldiv( ms.quot, 60 );    // quot=time in min, rem=seconds left
  std::lldiv_t mm = std::lldiv( ss.quot, 60 );    // quot=time in hrs, rem=minutes left
  std::lldiv_t hh = std::lldiv( mm.quot, 24 );    // quot=time in days, rem=hours left
  std::string str = std::to_string(hh.rem)+":"+std::to_string(mm.rem)+":"+std::to_string(ss.rem)
      +"."+std::to_string(ms.rem);
  if (hh.quot > 0) str = std::to_string(hh.quot)+" "+str;
  return str;
}

/** Read configuration file
 *  Configuration files need to be parsed and stored in a config object.
 *  Templated functions body needs to be put in the header file.
 */
template<typename MsgType>
std::unique_ptr<MsgType> ReadProtobufFile(std::string config_filename) {
  std::unique_ptr<MsgType> config(new MsgType());
  // 'Fillup' the 'empty' config object with data read from config_filename
  LOG(INFO) << "Reading the file " << config_filename;
  int in_file_descriptor = open(config_filename.c_str(), O_RDONLY);
  if (in_file_descriptor < 0) {
    LOG(ERROR) << " Error opening the file: " << strerror(errno);
    return nullptr;
  }
  // Parse the file
  google::protobuf::io::FileInputStream file_input_stream(in_file_descriptor);
  if (!google::protobuf::TextFormat::Parse(&file_input_stream, &(*config))) {
    LOG(ERROR) << "Could not parse config file";
    return nullptr;
  }
  file_input_stream.Close();
  if (VLOG_IS_ON(3)) {
    // convert to string to display
    std::string in_config_str;
    google::protobuf::TextFormat::PrintToString(*config, &in_config_str);
    VLOG(3) << "in_config = \n" << in_config_str;
  }
  // Done
  return config;
}


class Looper {
 public:
  Looper(float freq);
  bool SetInterval(const int64_t& interval, const int64_t& curr_time);
  virtual ~Looper();
  int64_t Sleep();
  inline float max_frequency() {return max_frequency_;}
 private:
  float loop_frequency_;
  float max_frequency_;
  int64_t loop_timeperiod_;   // microsec
  int64_t last_touch_time_;
  int64_t get_wall_time_microsec();
};



}   // namespace anantak

