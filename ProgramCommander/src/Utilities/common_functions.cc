/**
 *  Common functions used by many programs
 *   - Read setup config file
 *   - Read program specific config file
 */

/** Anantak includes */
#include "Utilities/common_functions.h"

/** for wall time */
#include <sys/time.h>

namespace anantak {

/** Read Programs Setup
 *  Reads programs setup file - this is used by many programs to read pub/sub zmq setups
 */
bool ReadProgramsSetup(std::string config_filename, /**< Protobuf text format filename */
    anantak::ProgramsSetup* programs_setup) {       /**< Programs setup object to be filled up */
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
  VLOG(3) << "in_programs_setup = \n" << in_programs_setup_str;
  return true;
}

Looper::Looper(float freq) {
  loop_frequency_ = freq;
  loop_timeperiod_ = int64_t(float(1000000)/freq);
  last_touch_time_ = get_wall_time_microsec();
  max_frequency_ = 0.0;
  VLOG(2) << "Created a looper at " << loop_timeperiod_/1000 << "ms/" << loop_frequency_ << "Hz";
}
bool Looper::SetInterval(const int64_t& interval, const int64_t& curr_time) {
  loop_frequency_ = float(1000000/interval);
  loop_timeperiod_ = interval; //int64_t(float(1000000)/freq);
  last_touch_time_ = curr_time;
  max_frequency_ = 0.0;
  VLOG(2) << "Setting looper frequency " << loop_timeperiod_/1000 << "ms/" << loop_frequency_ << "Hz";
  return true;
}
Looper::~Looper() {}
int64_t Looper::Sleep() {
  int64_t current_time = get_wall_time_microsec();
  int64_t round_trip_time = current_time - last_touch_time_;
  max_frequency_ = float(1000000)/float(round_trip_time);
  int64_t microsecs = loop_timeperiod_ - round_trip_time;
  VLOG(3) << "Sleep time: " << microsecs << "us";
  if (microsecs<0) microsecs = 0;
  struct timespec t;
  t.tv_sec = microsecs / 1000000;
  t.tv_nsec = (microsecs % 1000000) * 1000;
  nanosleep(&t, NULL);    
  last_touch_time_ = get_wall_time_microsec();
  return last_touch_time_;
}
int64_t Looper::get_wall_time_microsec() {
  struct timeval tv;
  gettimeofday (&tv, NULL);
  return (int64_t) (tv.tv_sec * 1000000 + tv.tv_usec);
}

  
}   // namespace anantak

