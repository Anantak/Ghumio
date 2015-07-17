/**
 *  Message File Stats
 *
 *  Gets the name of a message file, loads all message headers and calculates statistics.
 */

#pragma once

/** std includes */
#include <string>
#include <vector>

/** message types include */
#include "sensor_messages.pb.h"

namespace anantak {

class MessageFileStats {
 public:
  /** Constructor - just create a blank class */
  MessageFileStats();
  
  /** Destructor - every member will be auto-destructuble */
  virtual ~MessageFileStats();
  
  /** Calculate stats for a given messages file - this will open and close the file */
  bool CalculateFileStats(std::string filename);

  // Accessors and mutators
  inline std::string filename() {return filename_;}   /**< Messages filename */
  inline bool have_stats() {return have_stats_;}      /**< Set to true when stats are available */
  /** return reference to timestamps vector */
  inline const std::vector<int64_t>& timestamps() const {
    return timestamps_;
  }
  /** return reference to timestamps vector */
  inline const std::vector<anantak::HeaderMsg>& headers() const {
    return headers_;
  }
  // various stats
  inline int64_t n_msgs() {return n_msgs_;}
  inline int64_t min_timestamp() {return min_timestamp_;}
  inline int64_t max_timestamp() {return max_timestamp_;}
  inline int64_t period() {return period_;}
  inline float avg_frequency() {return avg_frequency_;}
  inline float avg_msg_read_time() {return avg_msg_read_time_;}
  
 private:
  // Operating variables
  std::string filename_;
  bool have_stats_;
  std::vector<anantak::HeaderMsg> headers_;   /**< Vector of headers from the file */
  std::vector<int64_t> timestamps_;           /**< Message timestamps in the file */
  
  // Worker functions
  bool CollectHeaders();                    /**< Collects all headers from the file */
  bool ProcessHeaders();                    /**< Process the headers to calculate stats */
  
  // File stats
  int32_t n_msgs_;                          /**< Number of messages in file */
  int64_t min_timestamp_;                   /**< Minimum timestamp */
  int64_t max_timestamp_;                   /**< Maximum timestamp */
  int64_t period_;                          /**< Time period of file messages */
  float avg_frequency_;                     /**< Average frequency in Hz of messages in file */
  float avg_msg_read_time_;                 /**< Time (us) it takes to read a message on avg */
};


}