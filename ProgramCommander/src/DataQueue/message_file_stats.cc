/**
 *  Message File Stats
 *
 *  This reads a message file, gets all message headers and generates statistics for it.
 */

/** Header file */
#include "DataQueue/message_file_stats.h"

/** Anantak includes */
#include "DataQueue/message_file_reader.h"
#include "Utilities/common_functions.h"

/** Google libraries includes */
#include <glog/logging.h>


namespace anantak {

/** MessageFileStats */

/** Constructor - build an empty MessageFileStats object */
MessageFileStats::MessageFileStats() {
  have_stats_ = false;
  n_msgs_ = -1;
  headers_.resize(0);
  timestamps_.resize(0);
  min_timestamp_ = -1;
  max_timestamp_ = -1;
  period_ = -1;
  avg_frequency_ = -1.0;
  avg_msg_read_time_ = -1.0;
}

/** Destructor - all members are self destructing */
MessageFileStats::~MessageFileStats() {}

/** CalculateFileStats - the main worker function */
bool MessageFileStats::CalculateFileStats(std::string filename) {
  filename_ = filename;
  if (!CollectHeaders()) return false;
  if (!ProcessHeaders()) return false;
  return true;
}

/** CollectHeaders - Opens the file, collects all headers, closes file */
bool MessageFileStats::CollectHeaders() {
  anantak::MessageFileReader file_reader;
  if (!file_reader.Open(filename_)) {
    LOG(ERROR) << "File " << filename_ << " could not be opened.";
    return false;
  }
  file_reader.StartTrackingPerformance();
  // Read messages one-by-one
  anantak::SensorMsg msg;
  n_msgs_ = 0;
  // Message is read 'in place'. Message's fields will be written 'over' the existing message. 
  while (file_reader.ReadMessage(&msg)) {
    // get the message header by reference
    const anantak::HeaderMsg& hdr = msg.header();
    // copy the header to the vector of headers
    headers_.push_back(hdr);  // copy of header is made here - copied to the container
    VLOG_EVERY_N(3, 1000) << "Average message read time " << file_reader.MessageReadTime();
    // Clear the message before next one is read, increment number of messages
    msg.Clear();
    n_msgs_++;
  }
  headers_.shrink_to_fit();
  avg_msg_read_time_ = file_reader.MessageReadTime();
  /** no file closing is necessary as file is already closed */
  VLOG(1) << "Read " << n_msgs_ << " messages from " << filename_;
  return true;
}

/** ProcessHeaders - Calculates the statistics from the collected headers */
bool MessageFileStats::ProcessHeaders() {
  min_timestamp_ = headers_[0].timestamp();
  max_timestamp_ = headers_[0].timestamp();
  for (int i=0; i<headers_.size(); i++) {
    if (headers_[i].has_send_timestamp()) timestamps_.push_back(headers_[i].send_timestamp());
    else timestamps_.push_back(headers_[i].timestamp());
    if (min_timestamp_ > headers_[i].timestamp()) min_timestamp_ = headers_[i].timestamp();
    if (max_timestamp_ < headers_[i].timestamp()) max_timestamp_ = headers_[i].timestamp();
  }
  timestamps_.shrink_to_fit();
  period_ = max_timestamp_ - min_timestamp_;
  avg_frequency_ = float(1000000) * float(n_msgs_) / float(period_);
  // We now have the stats
  have_stats_ = true;
  VLOG(2) << "Min timestamp = " << min_timestamp_;
  VLOG(2) << "Max timestamp = " << max_timestamp_;
  VLOG(2) << "Period = " << anantak::microsec_to_time_str(period_);
  VLOG(2) << "Avg message frequency = " << avg_frequency_ << " Hz";
  VLOG(2) << "Avg message read time = " << avg_msg_read_time_ << " us";
  return true;
}

}
