/**
 *  Message Queue implementation
 *
 *  Maintains a circular queue of messages. Returns messages as needed.
 */

#include "DataQueue/message_queue.h"
#include "DataQueue/message_file_writer.h"

/** Google Logging library */
#include <glog/logging.h>

/** for wall time */
#include <sys/time.h>

namespace anantak {
  
/** Constructor to create a MessageQueue */
MessageQueue::MessageQueue(std::string queue_name, int32_t queue_size) {
  queue_name_ = queue_name;
  queue_size_ = queue_size;
  max_queue_size_multiple_ = 1.05;  // Initial value of max queue size is 5%+ queue size.
  max_queue_size_ = (int32_t) (max_queue_size_multiple_ * queue_size_);
  Initiate();
}

MessageQueue::MessageQueue(std::string queue_name, int32_t queue_size, int32_t max_queue_size) {
  queue_name_ = queue_name;
  queue_size_ = queue_size;
  max_queue_size_ = max_queue_size;
  max_queue_size_multiple_ = float(max_queue_size_) / float(queue_size_);
  Initiate();
}

/** Destructs the MessageQueue */
MessageQueue::~MessageQueue() {
  // All members can be automatically destructed by the compiler.
  // There are no dynamically created arrays.
}

bool MessageQueue::Initiate() {
  n_msgs_ = 0;              // Starting with no messages in the queue
  current_msg_index_ = -1;          
  oldest_msg_index_ = -1;
  VLOG(2) << "Initiating message queue. " << queue_name_ << " " << queue_size_ << " "
      << max_queue_size_multiple_ << " " << max_queue_size_;
  InitiateDataQueue();      // Create an empty queue
  track_performance_ = false;
  InitiatePerformanceTracking(100);      // Setup performance tracking
  return true;
}

/** Allocates data container memory */
bool MessageQueue::InitiateDataQueue() {
  // Create a vector with max_queue_size_ QueueDataType elements
  data_queue_.reserve(max_queue_size_);   // to avoid dynamic allocation and deallocation 
  for (int i=0; i<max_queue_size_; i++) {
    std::unique_ptr<QueueDataType> data_element(new QueueDataType());
    data_queue_.push_back(std::move(data_element));
  }
  VLOG(2) << "Initiated data_queue_ with " << data_queue_.size() << " elements";
  return true;
}

bool MessageQueue::StartTrackingPerformance(int32_t track_length=100) {
  track_performance_ = true;
  InitiatePerformanceTracking(track_length);
  VLOG(2) << "Switching Performance Tracking ON with tracking_length = " << track_length;
  return true;
}

bool MessageQueue::StopTrackingPerformance() {
  track_performance_ = false;
  VLOG(2) << "Switching Performance Tracking OFF";
  return true;
}

bool MessageQueue::InitiatePerformanceTracking(int32_t track_length=100) {
  // Initiate performance tracking
  tracking_vector_lengths_ = track_length;  // hard coding this, maybe set by value in a constructor
  data_add_times_.resize(tracking_vector_lengths_, 0);
  data_add_intervals_.resize(tracking_vector_lengths_, 0);
  data_add_times_index_ = 0;
  data_fetch_times_.resize(tracking_vector_lengths_, 0);
  data_fetch_intervals_.resize(tracking_vector_lengths_, 0);
  data_fetch_n_msgs_.resize(tracking_vector_lengths_, 0);
  data_fetch_times_index_ = 0;
  if (track_performance_) VLOG(3) << "Performance tracking is running";
  else VLOG(3) << "Performane tracking is NOT running";
  return true;  
}

inline int32_t MessageQueue::next_index() {
  return (current_msg_index_+1)%queue_size_;
}

inline int32_t MessageQueue::prev_index() {
  int32_t idx;
  if (current_msg_index_ == 0) idx = queue_size_-1;
  else idx = (current_msg_index_-1)%queue_size_;
  return idx;
}

/** Adds a message to the MessageQueue */
bool MessageQueue::AddMessage(std::unique_ptr<QueueDataType> msg) {
  int64_t data_add_start_time;
  if (track_performance_) {data_add_start_time = get_wall_time_microsec();} 
  // house keeping
  if (n_msgs_ == 0) {current_msg_index_=0; oldest_msg_index_=0; n_msgs_=1;}
  else if (n_msgs_ < queue_size_) {current_msg_index_=next_index(); n_msgs_++;}
  else if (n_msgs_ == queue_size_) {current_msg_index_=oldest_msg_index_; oldest_msg_index_=next_index();}
  else {LOG(ERROR) << "n_msgs_ > queue_size_!!";}
  // Copy the msg data to the data_queue_. Space is pre-allocated in queue, can not avoid copying.
  // A message comes, we copy from msg buffer to local buffer, then from that to queue. So two copy
  // operations. This is not ideal. At least we should be able to avoid copy from message. This may
  // be available via nanomsg library, but it is in early beta. We will consider it later.
  data_queue_[current_msg_index_]->CopyData(*msg);
  VLOG(3) << "Got message to save: " << msg->message_str << " q_idxs = " << current_msg_index_
      << " " << oldest_msg_index_;
  // Performance measurements
  if (track_performance_) {
    int64_t data_add_acc_time =
        get_wall_time_microsec() - data_add_start_time + data_add_intervals_[data_add_times_index_];
    data_add_times_index_++;
    data_add_times_index_ %= tracking_vector_lengths_;    // this is a circular queue
    data_add_intervals_[data_add_times_index_] = data_add_acc_time;
    data_add_times_[data_add_times_index_] = data_add_start_time;
  }
  // msg is destructed here - expensive but can't be avoided.
  return true;
}

/** Add many messages */
bool MessageQueue::AddMessages(std::unique_ptr<QueueDataVectorType> msgs) {
  // Is there any optimization we could achieve when many messages are to be inserted? Is it just
  // that we will insert multiple messages one-by-one? Cant think of any at this point.
  for (int i=0; i<(*msgs).size(); i++) {
    // We transfer the ownership of the data structure to AddMessage from this vector.
    // This is by design as we do not use this vector after adding the messages. Good thing is that
    // this avoids deallocating strings both here and in AddMessage. 
    AddMessage(std::move((*msgs)[i]));
  }
  // msgs will be destructed here - this will be expensive, not sure right now how to get over it.
  // This is a problem with messaging - you have to allocate and deallocate memory for messages.
  // Other option is to make a giant monolithic application, that will avoid several such issues.
  // But that may not be the best idea.
  return true;
}


/** Add all messages that lie in an interval to a Composite message - a pointer is used to
 *  indicate that this function will only modify the message. Message is owned by the calling
 *  function. Here we copy the message queue string to composite msg. Return number of msgs */
int MessageQueue::AddMessagesToCompositeMessage(anantak::CompositeMsg* composite_msg_ptr,
    int64_t begin_time, int64_t end_time) {
  int64_t data_fetch_start_time;
  if (track_performance_) {data_fetch_start_time = get_wall_time_microsec();} 
  int n = 0;
  for (int i=0; i<n_msgs_; i++) {
    int32_t msg_idx = (oldest_msg_index_+i)%queue_size_;
    if ((data_queue_[msg_idx]->arrival_time > begin_time) &&
        (data_queue_[msg_idx]->arrival_time <= end_time)) {
      // make a copy of the message into composite message container
      composite_msg_ptr->add_message_data(data_queue_[msg_idx]->message_str);    // copy
      n++;
    }
  }
  // Performance measurements
  if (track_performance_) {
    int64_t data_fetch_acc_time = get_wall_time_microsec() - data_fetch_start_time +
        data_fetch_intervals_[data_fetch_times_index_];
    int32_t tot_data_fetch_msgs =
        int32_t(n) + data_fetch_n_msgs_[data_fetch_times_index_];
    data_fetch_times_index_++;
    data_fetch_times_index_ %= tracking_vector_lengths_;    // this is a circular queue
    data_fetch_intervals_[data_fetch_times_index_] = data_fetch_acc_time;
    data_fetch_times_[data_fetch_times_index_] = data_fetch_start_time;
    data_fetch_n_msgs_[data_fetch_times_index_] = tot_data_fetch_msgs;
  }
  VLOG(3) << "Returned " << n << " messages";
  return n;
}

/** Add latest message to a composite message - message is owned by the calling function */
int MessageQueue::AddLatestMessageToCompositeMessage(anantak::CompositeMsg* composite_msg_ptr) {
  int n = 0;
  // Copy the string in data queue to the message
  composite_msg_ptr->add_message_data(data_queue_[current_msg_index_]->message_str);    // copy
  n++;
  return n;
} 

/** Return copies of messages from the queue that lie in a given time interval */
std::unique_ptr<MessageQueue::QueueDataVectorType>
    MessageQueue::FetchMessagesReceivedInTimeInterval(
    int64_t begin_time, int64_t end_time) {
  int64_t data_fetch_start_time;
  if (track_performance_) {data_fetch_start_time = get_wall_time_microsec();} 
  // Here we have to create copies of the messages. This is necessary as these messages are owned
  // by the queue. The requestor is just asking for copies. Ownership is not transferred.
  // We will not explicitly sort the messages, we will put them in a vector in the order in which
  // they arrived.
  // Initialize the container with no elements. This version is only valid till C++11
  std::unique_ptr<QueueDataVectorType> msgs(new QueueDataVectorType(0));
  //(*msgs).reserve(n_msgs_); // at most the entire queue could be transferred
  // Lets do the search for messages that lie in the interval simply
  int n=0;
  for (int i=0; i<n_msgs_; i++) {
    int32_t msg_idx = (oldest_msg_index_+i)%queue_size_;
    if ((data_queue_[msg_idx]->arrival_time > begin_time) &&
        (data_queue_[msg_idx]->arrival_time <= end_time)) {
      // make a copy of the message
      std::unique_ptr<QueueDataType> msg(new QueueDataType(*data_queue_[msg_idx]));  // copy
      msgs->push_back(std::move(msg));
      n++;
    }
  }
  // Performance measurements
  if (track_performance_) {
    int64_t data_fetch_acc_time = get_wall_time_microsec() - data_fetch_start_time +
        data_fetch_intervals_[data_fetch_times_index_];
    int32_t tot_data_fetch_msgs =
        int32_t(n) + data_fetch_n_msgs_[data_fetch_times_index_];
    data_fetch_times_index_++;
    data_fetch_times_index_ %= tracking_vector_lengths_;    // this is a circular queue
    data_fetch_intervals_[data_fetch_times_index_] = data_fetch_acc_time;
    data_fetch_times_[data_fetch_times_index_] = data_fetch_start_time;
    data_fetch_n_msgs_[data_fetch_times_index_] = tot_data_fetch_msgs;
  }
  VLOG(3) << "Returned " << n << " messages";
  return msgs;
}

/** Return a copy of the current message */
std::unique_ptr<MessageQueue::QueueDataType> MessageQueue::FetchLatestMessage() {
  // make a copy of the current message and return a pointer to it
  std::unique_ptr<QueueDataType> msg(new QueueDataType(*data_queue_[current_msg_index_]));  // copy
  return msg;
}


/** Change queue size */
bool MessageQueue::ChangeQueueSize(int32_t new_queue_size) {
  // We will do this in future when we get to dynamic queue sizing
  return false;
}

/** Change max queue size */
bool MessageQueue::ChangeMaxQueueSize(int32_t new_max_queue_size) {
  // We will do this in future when we get to dynamic queue sizing
  return false;
}

/** Save sensor message to file
 * This assumes that all messages are sensor messages. So tries to first parse into a sensor message.
 */
bool MessageQueue::SaveSensorMessagesToFile(const std::string& filename) {
  // Open a sensor message file writer, go through all messages, parse each one, save to file and close file.
  VLOG(2) << "Creating a file writer for " << filename;
  anantak::MessageFileWriter file_writer;
  if (!file_writer.Open(filename)) {
    LOG(ERROR) << "Could not open file. Quitting.";
    return false;
  }
  anantak::SensorMsg sensor_msg;
  int32_t num_msgs_written = 0;
  for (int i=0; i<n_msgs_; i++) {
    // Try to parse the message to a protocol buffer message
    int32_t msg_idx = (oldest_msg_index_ + i)%queue_size_;
    sensor_msg.Clear();  // Clear the message before a new parse it done.
    if (sensor_msg.ParseFromString((*data_queue_.at(msg_idx)).message_str)) {
      file_writer.WriteMessage(sensor_msg);
      num_msgs_written++;
    } else {
      LOG(ERROR) << "Could not parse message as a sensor message. Skipping.";
    }
  }
  LOG(INFO) << "Written " << num_msgs_written << " messages to file " << filename;
  file_writer.Close();
  VLOG(2) << "Closed file " << filename;
  return true;
}
  
/** Calculate Queue performance measures and return a struct */
std::unique_ptr<MessageQueue::QueuePerformanceType> MessageQueue::GetQueuePerformance() {
  std::unique_ptr<QueuePerformanceType> queue_perf(new QueuePerformanceType());
  int32_t first_data_add_times_index = (data_add_times_index_+1)%tracking_vector_lengths_;
  int32_t first_data_fetch_times_index = (data_fetch_times_index_+1)%tracking_vector_lengths_;
  float float_tracking_vector_lengths = float(tracking_vector_lengths_);
  if (data_add_times_[first_data_add_times_index] != 0) {
    queue_perf->msg_add_rate = float(1000000) * float_tracking_vector_lengths / float(
        data_add_times_[data_add_times_index_] - data_add_times_[first_data_add_times_index]);
    queue_perf->avg_add_delay = float(data_add_intervals_[data_add_times_index_] -
        data_add_intervals_[first_data_add_times_index]) / float_tracking_vector_lengths;
  }
  if (data_fetch_times_[first_data_fetch_times_index] != 0 &&
      data_fetch_n_msgs_[data_fetch_times_index_] != 0) {
    queue_perf->msg_fetch_rate = float(1000000) * float_tracking_vector_lengths / float(
        data_fetch_times_[data_fetch_times_index_] - data_fetch_times_[first_data_fetch_times_index]);
    queue_perf->avg_fetch_delay = float(data_fetch_intervals_[data_fetch_times_index_] -
        data_fetch_intervals_[first_data_fetch_times_index]) / float(
        data_fetch_n_msgs_[data_fetch_times_index_] - data_fetch_n_msgs_[first_data_fetch_times_index]);
  }
  return queue_perf;
}


/** Utility to get wall time in microseconds */
int64_t MessageQueue::get_wall_time_microsec() {
  struct timeval tv;
  gettimeofday (&tv, NULL);
  return (int64_t) (tv.tv_sec * 1000000 + tv.tv_usec);
}

} // namespace anantak
