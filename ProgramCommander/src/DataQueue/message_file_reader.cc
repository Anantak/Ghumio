/**
 *  Message File Reader implementation
 */

/** std include */
#include <sys/time.h>  // gettimeofday

/** Header include */
#include "DataQueue/message_file_reader.h"

/** Google libraries includes */
#include <glog/logging.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/io/coded_stream.h>

/** File utilities */
#include <fcntl.h>

/** Anantak includes */
#include "common_config.h"

namespace anantak {
  
/** Message File Writer constructor */
MessageFileReader::MessageFileReader() {
  VLOG(3) << "Created Message File Reader";
  file_is_open_ = false;
  track_performance_ = false;
  n_messages_read_ = 0;
}

/** Destructor */
MessageFileReader::~MessageFileReader() {
  if (file_is_open_) {
    VLOG(1) << "File is still open closing file.";
    Close();
  }
  VLOG(3) << "Destructed the MessageFileReader";
}

/** Open the file */
bool MessageFileReader::Open(std::string filename, bool absolute) {
  if (!absolute) filename_ = anantak::GetProjectSourceDirectory() + "/" + filename;
  else filename_ = filename;
  VLOG(3) << "Opening file " << filename_;
  if ((file_handle_ = open(filename_.c_str(), O_RDONLY)) == -1) {
    LOG(ERROR) << "Could not open file " << strerror(errno);
    return false;
  }
  VLOG(3) << "File opened ";
  in_stream_ = new google::protobuf::io::FileInputStream(file_handle_);
  VLOG(3) << "InStream created";
  file_is_open_ = true;
  return true;
}

/** Close the file */
bool MessageFileReader::Close() {
  delete in_stream_;
  VLOG(3) << "Destructed the input stream";
  close(file_handle_);
  VLOG(3) << "Closed the input file " << filename_;
  file_is_open_ = false;
  return true;
}

/** Reopen the file */
bool MessageFileReader::Reopen() {
  if (file_is_open_) Close();
  bool success = Open(filename_, true);
  return success;
}

/** Read the message from the input file */
bool MessageFileReader::ReadMessage(::google::protobuf::Message* message) {
  if (file_is_open_) {
    int64_t current_time;
    if (track_performance_) {current_time = get_wall_time_microsec();}
    bool read = ReadDelimitedFrom(in_stream_, message);
    if (track_performance_) {
      int32_t last_acc_times_idx_ = acc_times_idx_;
      acc_times_idx_++;
      acc_times_idx_ %= n_acc_times_;
      acc_times_[acc_times_idx_] = get_wall_time_microsec() - current_time
          + acc_times_[last_acc_times_idx_];
    }
    if (!read) {
      VLOG(1) << "Input file stream has reached an end (or an error occurred). Closing stream.";
      Close();
    } else {
      n_messages_read_++;
    }
    return read;
  } else {
    LOG(ERROR) << "Can not read message as file is closed.";
  }
}

/** Directly take from code by Kenton Varda */
bool MessageFileReader::ReadDelimitedFrom(::google::protobuf::io::ZeroCopyInputStream* rawInput,
    ::google::protobuf::MessageLite* message) {
  // We create a new coded stream for each message.  Don't worry, this is fast,
  // and it makes sure the 64MB total size limit is imposed per-message rather
  // than on the whole stream.  (See the CodedInputStream interface for more
  // info on this limit.)
  ::google::protobuf::io::CodedInputStream input(rawInput);
  
  // Read the size.
  uint32_t size;
  if (!input.ReadVarint32(&size)) return false;

  // Tell the stream not to read beyond that size.
  google::protobuf::io::CodedInputStream::Limit limit =
      input.PushLimit(size);

  // Parse the message.
  if (!message->MergeFromCodedStream(&input)) return false;
  if (!input.ConsumedEntireMessage()) return false;

  // Release the limit.
  input.PopLimit(limit);

  return true;
}

/** Read the size of the next message */
bool MessageFileReader::ReadNextMessageSize(uint32_t* size) {
  if (file_is_open_) {
    ::google::protobuf::io::CodedInputStream input(in_stream_);
    if (!input.ReadVarint32(size)) return false;
  } else {
    *size = 0;
    return false;
  }
  return true;
}

/** Read next message raw bytes from file to buffer */
bool MessageFileReader::ReadNextMessageRawToBuffer(void* buffer, uint32_t size) {
  if (file_is_open_) {
    // Create a coded input stream
    ::google::protobuf::io::CodedInputStream input(in_stream_);  
    // Tell the stream not to read beyond that size.
    google::protobuf::io::CodedInputStream::Limit limit = input.PushLimit(size);
    // Copy to the buffer
    if (!input.ReadRaw(buffer, size)) return false;
    // Release the limit.
    input.PopLimit(limit);
  } else {
    return false;
  }
  return true;
}


/** Start tracking performance */
bool MessageFileReader::StartTrackingPerformance(int32_t n_acc_times) {
  track_performance_ = true;
  n_acc_times_ = n_acc_times;
  acc_times_idx_ = 0;
  acc_times_.resize(n_acc_times_, 0);
  return true;
}

/** Stop tracking performance */
bool MessageFileReader::StopTrackingPerformance() {
  track_performance_ = false;
  n_acc_times_ = 0;
  acc_times_idx_ = 0;
  acc_times_.resize(n_acc_times_, 0);
  return true;
}

/** Calculate average message write time in microsecs */
float MessageFileReader::MessageReadTime() {
  int32_t next_acc_times_idx_ = acc_times_idx_ + 1;
  next_acc_times_idx_ %= n_acc_times_;
  if (acc_times_[next_acc_times_idx_] != 0) {
    return float(acc_times_[acc_times_idx_] -
        acc_times_[next_acc_times_idx_]) / float(n_acc_times_);
  } else {
    return float(-1);
  }
}

/** Utility function to read all sensor messages from a file **/
bool MessageFileReader::LoadMessagesFromFile(const std::string& filename,
    std::vector<anantak::SensorMsg>* msgs) {
  int32_t num_msgs_read = 0;
  if (!Open(filename)) {
    LOG(ERROR) << "Could not open file " << filename << ". Quit.";
    return false;
  }
  // Clear the msgs
  msgs->clear();
  anantak::SensorMsg sensor_msg;
  while (file_is_open()) {
    // Read the size of the message
    uint32_t msg_size;
    bool read_size = ReadNextMessageSize(&msg_size);
    if (!read_size) {
      VLOG(2) << "Closing file after " << num_msgs_read << " messages have been read";
      Close();
      continue;
    }
    // Allocate an array of message size
    std::vector<char> msg_str(msg_size, 0x00);
    // Get a pointer to the data buffer
    char* msg_str_ptr = msg_str.data();
    // Copy the message data into the buffer
    bool read_data = ReadNextMessageRawToBuffer(msg_str_ptr, msg_size);
    if (!read_data) {
      VLOG(2) << "Closing file after " << num_msgs_read << " messages have been read";
      Close();
      continue;
    }
    // Try to parse the message to a protocol buffer message
    sensor_msg.Clear();  // Clear the message before a new parse it done.
    if (sensor_msg.ParseFromArray(msg_str_ptr, msg_size)) {
      msgs->push_back(sensor_msg);
      num_msgs_read++;
    } else {
      LOG(ERROR) << "Could not parse message as a sensor message. Skipping.";
      continue;
    }
  }
  VLOG(1) << "Read " << num_msgs_read << " messages from file " << filename;
  return true;
}


/** Utility to get wall time in microseconds */
int64_t MessageFileReader::get_wall_time_microsec() {
  struct timeval tv;
  gettimeofday (&tv, NULL);
  return (int64_t) (tv.tv_sec * 1000000 + tv.tv_usec);
}


} // namespace anantak