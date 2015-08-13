/* File Messages Keeper
 * Loads sensor messages data
 * Set Time based on clock time or data will be returned using time interval
 * Returns new messages in an interval or clock time passed
 */

#pragma once

/** std includes */
#include <vector>
#include <string>
#include <fstream>
#include <algorithm>
#include <memory>

/** Google libraries */
#include <glog/logging.h>
#include <google/protobuf/message.h>

/** Anantak includes */
#include "common_config.h"
#include "Filter/circular_queue.h"
#include "Filter/observation.h"
#include "DataQueue/message_file_writer.h"
#include "DataQueue/message_file_reader.h"
#include "DataQueue/message_file_stats.h"
#include "Utilities/common_functions.h"

/** Protocol buffers */
#include "sensor_messages.pb.h"
#include "state_messages.pb.h"
#include "configurations.pb.h"

namespace anantak {

class FileMessagesKeeper {
 public:
  
  FileMessagesKeeper(const std::vector<std::string>& msgs_filenames, const bool run_in_reatime_mode) {
    msgs_filenames_ = msgs_filenames;
    run_in_realtime_mode_ = run_in_reatime_mode;
  }
  
  virtual ~FileMessagesKeeper() {}
  
  bool LoadMsgsFromFile(const std::string& filename, std::vector<anantak::SensorMsg>* storage) {
    anantak::MessageFileReader pb_file_reader;
    if (!pb_file_reader.Open(filename)) {
      LOG(ERROR) << "File " << filename << " could not be opened.";
      return false;
    }
    // Read messages one-by-one
    anantak::SensorMsg msg;
    int n_msgs_in_file = 0;
    // Message is read 'in place'. Message's fields will be written 'over' the existing message. 
    while (pb_file_reader.ReadMessage(&msg)) {
      anantak::SensorMsg msg_copy(msg);
      storage->push_back(msg_copy);
      msg.Clear();
      n_msgs_in_file++;
    }
    //pb_file_reader.Close(); /** no closing is necessary as file is already closed */
    VLOG(1) << "Read " << n_msgs_in_file << " messages from " << filename;  
    return true;
  }
  
  bool LoadAllMsgsFromFiles() {
    // Load msgs into sensor_msgs_
    num_files_ = msgs_filenames_.size();
    VLOG(1) << "Number of message files = " << num_files_;
    sensor_msgs_.resize(num_files_);  // all elements are nullptr's
    for (int i=0; i<num_files_; i++) {
      std::unique_ptr<std::vector<anantak::SensorMsg>> ptr(new std::vector<anantak::SensorMsg>);
      sensor_msgs_[i] = std::move(ptr);
      if (!LoadMsgsFromFile(msgs_filenames_[i], sensor_msgs_[i].get())) {
        LOG(ERROR) << "Could not load messages from " << msgs_filenames_[i];
      }
    }
    // Set file_time_curr_time_offset_
    int64_t min_files_ts = 0;
    for (int i=0; i<num_files_; i++) {
      int64_t file_min_ts = 0;
      const anantak::SensorMsg& msg = sensor_msgs_[i]->front();
      if (msg.has_header()) {
        file_min_ts = msg.header().timestamp();
      } else {
        LOG(WARNING) << "File " << i << " front message has no header!";
      }
      int64_t file_max_ts = 0;
      const anantak::SensorMsg& msg2 = sensor_msgs_[i]->back();
      if (msg2.has_header()) {
        file_max_ts = msg2.header().timestamp();
      } else {
        LOG(WARNING) << "File " << i << " last message has no header!";
      }
      VLOG(1) << "  File " << i << " starting, ending timestamps = "
          << anantak::microsec_to_time_str(file_min_ts) << " "
          << anantak::microsec_to_time_str(file_max_ts);
      if (min_files_ts==0 && file_min_ts!=0) {
        min_files_ts = file_min_ts;
      } else {
        min_files_ts = std::min(file_min_ts, min_files_ts);
      }
    }
    if (min_files_ts==0) {
      LOG(ERROR) << "Could not calculate minimum files timestamp";
      return false;
    } else {
      VLOG(1) << "Starting files timestamp = " << anantak::microsec_to_time_str(min_files_ts);
    }
    curr_time_ = get_wall_time_microsec();
    last_fetch_time_ = 0;
    data_starting_ts_ = min_files_ts;
    file_time_curr_time_offset_ = min_files_ts - curr_time_;
    VLOG(1) << "file_time_curr_time_offset_ = "
        << anantak::microsec_to_time_str(-file_time_curr_time_offset_);
    // Set msgs_indexes_ to beginning
    msgs_indexes_.resize(num_files_, int32_t(0));
    return true;
  }
  
  // Any more data left?
  bool MoreDataLeft() {
    bool data_left = false;
    for (int i_file=0; i_file<num_files_; i_file++)
        data_left |= (msgs_indexes_[i_file] < sensor_msgs_[i_file]->size());
    return data_left;
  }
  
  // Utility to allocate memory for data fetch. Assumes all pointers in array are NULL
  bool AllocateMemoryForNewMessages(const int32_t& num_msgs_per_file,
      std::vector<std::unique_ptr<std::vector<anantak::SensorMsg>>>* new_msgs
  ) {
    new_msgs->resize(num_files_);
    for (int i=0; i<num_files_; i++) {
      std::unique_ptr<std::vector<anantak::SensorMsg>> ptr(new std::vector<anantak::SensorMsg>);
      (*new_msgs)[i] = std::move(ptr);
      (*new_msgs)[i]->reserve(num_msgs_per_file);
    }
  }
  
  // Fetch messages between given historical timestamps
  bool FetchMessagesBetweenTimestamps(const int64_t& ts0, const int64_t& ts1,
      std::vector<std::unique_ptr<std::vector<anantak::SensorMsg>>>* new_msgs) {
    // Read forward from msgs_indexes_ from each file, fetching messages in the time interval
    // new_msgs should have correct size. If not, return false
    if (new_msgs->size()!=num_files_) {
      LOG(ERROR) << "new_msgs->size()!=num_files_ " << new_msgs->size() << " " << num_files_;
      return false;
    }
    // Clear messages in copy buffer
    for (int i_file=0; i_file<num_files_; i_file++) {
      new_msgs->at(i_file)->clear();
    }
    // For each file move forward from current message, check timestamp. Copy message.
    for (int i_file=0; i_file<num_files_; i_file++) {
      bool exceeded_ts1 = false;
      while (!exceeded_ts1 && msgs_indexes_[i_file] < sensor_msgs_[i_file]->size()) {
        const anantak::SensorMsg& msg = sensor_msgs_[i_file]->at(msgs_indexes_[i_file]);
        exceeded_ts1 = (msg.header().timestamp()>ts1);
        if (msg.header().timestamp()>ts0 && msg.header().timestamp()<=ts1) {
          (*new_msgs)[i_file]->push_back(msg);  // copy message
        }
        if (!exceeded_ts1) msgs_indexes_[i_file]++;
      } // while !done
    } // for each file
    
    return true;
  }

  // Fetch last messages
  bool FetchLastMessages(std::vector<std::unique_ptr<std::vector<anantak::SensorMsg>>>* new_msgs) {
    for (int i_file=0; i_file<num_files_; i_file++) {
      new_msgs->at(i_file)->clear();
      const anantak::SensorMsg& msg = sensor_msgs_[i_file]->back();
      (*new_msgs)[i_file]->push_back(msg);  // copy message
    }
    return true;
  }
  
  // Fetch new messages since last time data was fetched
  //  realtime mode - messages are returned since min(last_fetch_time_, curr_time_-interval)
  //  batch mode - message are returned in curr_time_+interval. curr_time_ is updated.
  bool FetchNewMessages(const int64_t& interval,
      std::vector<std::unique_ptr<std::vector<anantak::SensorMsg>>>* new_msgs) {
    //int32_t num_msgs = 0;
    int64_t ts0, ts1;
    if (run_in_realtime_mode_) {
      curr_time_ = get_wall_time_microsec();
      int64_t fetch_interval = std::min(interval, curr_time_ - last_fetch_time_);
      last_fetch_time_ = curr_time_;
      ts1 = curr_time_;
      ts0 = curr_time_ - fetch_interval;
    } else {
      curr_time_ = curr_time_ + interval;
      ts1 = curr_time_;
      ts0 = last_fetch_time_;
      last_fetch_time_ = curr_time_;
    }
    // Convert current timestamps to historical file timestamps
    ts0 += file_time_curr_time_offset_;
    ts1 += file_time_curr_time_offset_;
    return FetchMessagesBetweenTimestamps(ts0, ts1, new_msgs);
  }
  
  inline int64_t get_wall_time_microsec() {
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) (tv.tv_sec * 1000000 + tv.tv_usec);  
  }
  
  int64_t CurrentTime() {return curr_time_;}
  int64_t CurrentDataTime() {return curr_time_+file_time_curr_time_offset_;}
  int64_t DataStartTime() {return data_starting_ts_;}
  
  // Data variables
  std::vector<std::string> msgs_filenames_;
  std::vector<std::unique_ptr<std::vector<anantak::SensorMsg>>> sensor_msgs_;
  int32_t num_files_;
  bool run_in_realtime_mode_;
  int64_t file_time_curr_time_offset_; //
  int64_t data_starting_ts_;  // timestamp of starting of data
  int64_t curr_time_; // Current time
  int64_t last_fetch_time_; // last timestamp when data was fetched
  std::vector<int32_t> msgs_indexes_; // current indexes of each messages vector
};  // FileMessagesKeeper

}   // anantak
