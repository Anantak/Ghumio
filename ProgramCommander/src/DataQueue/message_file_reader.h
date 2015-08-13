/**
 *  Message File Reader
 *
 *  Given a message file this opens the file, and provides a way to read the messages one-by-one.
 *  File is closed only at the end.
 */

#pragma once

/** std includes */
#include <string>
#include <memory>

/** Protobuf includes */
#include <google/protobuf/message.h>
#include "sensor_messages.pb.h"

namespace anantak {
  
class MessageFileReader {
 public:
  /** Constructor - creates a blank message file reader*/
  MessageFileReader();
  
  /** Destructor - closes file if open and destroys the open file handles/objects */
  virtual ~MessageFileReader();
  
  /** Open a messages file - returns true on success. Path is assumed to be relative */
  bool Open(std::string filename, bool absolute=false);
  
  /** Reads one message and advances the reading pointer forward ready to read the next message
   *  It allocates memory for the message object and passes a unique pointer indicating transfer
   *  of ownership of the allocated memory to the calling function. Returns a nullptr on error. */
  std::unique_ptr<google::protobuf::Message> ReadMessage();
  
  /** Reads one message and advances the pointer. Here the message memory is allocated by the
   *  calling function. This only 'fills up' the message with message data read from file.
   *  Important that fields will overwrite existing fields. Old fields will still exist. So the
   *  calling function must call Clear() on the message before reading the next message.
   *  It returns true on a successful read */
  bool ReadMessage(::google::protobuf::Message* message);
  
  /** Read the size of the next message + Read next message to buffer 
   *  NOTE: these need to be called in succession. Intention is that the calling function will
   *  first call the size function to get the size, allocate memory, then call Read message to
   *  to copy data into that buffer. This avoids a copy operation to an intermediate string.
   */
  bool ReadNextMessageSize(uint32_t* size);
  bool ReadNextMessageRawToBuffer(void* buffer, uint32_t size);
  
  /** Close the messages file - returns true on success */
  bool Close();
  
  /** Reopen the messages file - returns true on success */
  bool Reopen();

  /** Switch on/off performance tracking */
  bool StartTrackingPerformance(int32_t n_acc_write_times = 100);
  bool StopTrackingPerformance();
  float MessageReadTime();           /**< Average time taken to write one message */
  
  /** Utility function to read all sensor messages from a file **/
  bool LoadMessagesFromFile(const std::string& filename, std::vector<anantak::SensorMsg>* msgs,
                            bool absolute=false);
  
  // Accessors and mutators
  inline bool file_is_open() {return file_is_open_;}    /**< Is the file open? */
  inline std::string filename() {return filename_;}     /**< Get the filename */
  inline int32_t n_messages_read() {return n_messages_read_;} /**< Num of messages read */
  inline bool is_tracking_performance() {return track_performance_;} /**< Is tracking performance? */
  
 private:
  /** Operating variables */
  std::string filename_;
  bool file_is_open_;
  int file_handle_;
  ::google::protobuf::io::ZeroCopyInputStream* in_stream_;
  int32_t n_messages_read_;
  
  /** Read a delimited file from disk
   *  Taken from a post by Kenton Varda on stackoverflow */ 
  bool ReadDelimitedFrom(
      google::protobuf::io::ZeroCopyInputStream* rawInput,
      google::protobuf::MessageLite* message);

  /** Performance measurement tools */
  bool track_performance_;               /**< Indicates if tracking is on */
  int32_t n_acc_times_;                 /**< Number of read times */
  int32_t acc_times_idx_;               /**< Index of the current read time */
  std::vector<int64_t> acc_times_;      /**< Vector of accummulated read times */
  int64_t get_wall_time_microsec();     /**< Get current wall time in microsec*/

};
  
}   // namespace anantak
