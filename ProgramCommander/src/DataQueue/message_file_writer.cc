/**
 *  Message File Writer implementation
 */

/** std include */
#include <sys/time.h>  // gettimeofday

/** Header include */
#include "DataQueue/message_file_writer.h"

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
MessageFileWriter::MessageFileWriter() {
  VLOG(3) << "Created Message File Writer";
  file_is_open_ = false;
  track_performace_ = false;
  n_messages_written_ = 0;
}

/** Destructor */
MessageFileWriter::~MessageFileWriter() {
  if (file_is_open_) {
    VLOG(1) << "File is still open closing file.";
    Close();
  }
  VLOG(3) << "Destructed the MessageFileWriter";
}

/** Open the file */
bool MessageFileWriter::Open(const std::string& filename) {
  filename_ = anantak::GetProjectSourceDirectory() + "/" + filename;
  VLOG(3) << "Opening file " << filename_;
  if ((file_handle_ = open(filename_.c_str(), O_WRONLY|O_CREAT,
                           S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH)) == -1) {
    LOG(ERROR) << "Could not open file " << filename_ << " Error: " << strerror(errno);
    return false;
  }
  VLOG(3) << "File opened ";
  out_stream_ = new google::protobuf::io::FileOutputStream(file_handle_);
  VLOG(3) << "Outstream created";
  file_is_open_ = true;
  return true;
}

/** Close the file */
bool MessageFileWriter::Close() {
  delete out_stream_;
  VLOG(3) << "Destructed the output stream";
  close(file_handle_);
  VLOG(3) << "Closed the output file " << filename_;
  file_is_open_ = false;
  return true;
}

/** Write the message to the output file */
bool MessageFileWriter::WriteMessage(const ::google::protobuf::Message& message) {
  if (file_is_open_) {
    int64_t current_time;
    if (track_performace_) {current_time = get_wall_time_microsec();}
    bool written = WriteDelimitedTo(message, out_stream_);
    if (track_performace_) {
      int32_t last_acc_write_times_idx_ = acc_write_times_idx_;
      acc_write_times_idx_++;
      acc_write_times_idx_ %= n_acc_write_times_;
      acc_write_times_[acc_write_times_idx_] = get_wall_time_microsec() - current_time
          + acc_write_times_[last_acc_write_times_idx_];
    }
    if (written) n_messages_written_++;
    return written;
  } else {
    LOG(ERROR) << "Can not read message as file is closed.";
  }
}

/** Directly take from code by Kenton Varda */
bool MessageFileWriter::WriteDelimitedTo(
    const google::protobuf::MessageLite& message,
    google::protobuf::io::ZeroCopyOutputStream* rawOutput) {
  // We create a new coded stream for each message.  Don't worry, this is fast.
  google::protobuf::io::CodedOutputStream output(rawOutput);

  // Write the size.
  const int size = message.ByteSize();
  output.WriteVarint32(size);

  uint8_t* buffer = output.GetDirectBufferForNBytesAndAdvance(size);
  if (buffer != NULL) {
    // Optimization:  The message fits in one buffer, so use the faster
    // direct-to-array serialization path.
    message.SerializeWithCachedSizesToArray(buffer);
  } else {
    // Slightly-slower path when the message is multiple buffers.
    message.SerializeWithCachedSizes(&output);
    if (output.HadError()) return false;
  }

  return true;
}

/** Start tracking performance */
bool MessageFileWriter::StartTrackingPerformance(int32_t n_acc_write_times) {
  track_performace_ = true;
  n_acc_write_times_ = n_acc_write_times;
  acc_write_times_idx_ = 0;
  acc_write_times_.resize(n_acc_write_times_, 0);
  return true;
}

/** Stop tracking performance */
bool MessageFileWriter::StopTrackingPerformance() {
  track_performace_ = false;
  n_acc_write_times_ = 0;
  acc_write_times_idx_ = 0;
  acc_write_times_.resize(n_acc_write_times_, 0);
  return true;
}

/** Calculate average message write time in microsecs */
float MessageFileWriter::MessageWriteTime() {
  int32_t next_acc_write_times_idx_ = acc_write_times_idx_ + 1;
  next_acc_write_times_idx_ %= n_acc_write_times_;
  if (acc_write_times_[next_acc_write_times_idx_] != 0) {
    return float(acc_write_times_[acc_write_times_idx_] -
        acc_write_times_[next_acc_write_times_idx_]) / float(n_acc_write_times_);
  } else {
    return float(-1);
  }
}

/** Utility to get wall time in microseconds */
int64_t MessageFileWriter::get_wall_time_microsec() {
  struct timeval tv;
  gettimeofday (&tv, NULL);
  return (int64_t) (tv.tv_sec * 1000000 + tv.tv_usec);
}

  
}