/**
 *  Message File Writer
 *
 *  Creates a messages file. It opens the file in write mode. Provides a way to write a delimited
 *  messages file on the disk.
 */

/** std includes */
#include <string>
#include <memory>

/** Protobuf includes */
#include <google/protobuf/message.h>
#include <google/protobuf/io/zero_copy_stream.h>

namespace anantak {
  
class MessageFileWriter {
 public:
  /** Constructor - creates an empty file writer object */
  MessageFileWriter();
  
  /** Destructor - close the file if open, destruct all objects and open file handles */
  virtual ~MessageFileWriter();
  
  /** Open a file for writing */
  bool Open(const std::string& filename);
  
  /** Write a message - message is passed as const reference. Memory is owned by calling function,
   *  it is not deallocated here. Returns true on success. */
  bool WriteMessage(const ::google::protobuf::Message& message);
  
  /** Write a message - message is passed through a unique_ptr indicating transfer of ownership to
   *  the Write function. Return true on success. */
  bool WriteMessage(std::unique_ptr<google::protobuf::Message> message);
  
  /** Close the file */
  bool Close();
  
  /** Switch on/off performance tracking */
  bool StartTrackingPerformance(int32_t n_acc_write_times = 100);
  bool StopTrackingPerformance();
  float MessageWriteTime();           /**< Average time taken to write one message */
  
  // Accessors and mutators
  inline bool file_is_open() {return file_is_open_;}    /**< Is the file open? */
  inline std::string filename() {return filename_;}     /**< Get the filename */
  inline int32_t n_messages_written() {return n_messages_written_;} /**< Num of messages read */
  inline bool is_tracking_performance() {return track_performace_;} /**< Is tracking performance? */
  
 private:
  /** Operating variables */
  std::string filename_;
  bool file_is_open_;
  int file_handle_;
  ::google::protobuf::io::ZeroCopyOutputStream* out_stream_;
  int32_t n_messages_written_;
  
  /** Write a delimited protobuf message to a file stream.
   *  Taken from a post by Kenton Varda on stackoverflow */ 
  bool WriteDelimitedTo(
      const google::protobuf::MessageLite& message,
      google::protobuf::io::ZeroCopyOutputStream* rawOutput);

  /** Performance measurement tools */
  bool track_performace_;                     /**< Indicates if tracking is on */
  int32_t n_acc_write_times_;                 /**< Number of write times */
  int32_t acc_write_times_idx_;               /**< Index of the current write time */
  std::vector<int64_t> acc_write_times_;      /**< Vector of accummulated write times */
  int64_t get_wall_time_microsec();             /**< Get current wall time in microsec*/
  
};  
  
}
