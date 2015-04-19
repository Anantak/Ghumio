/**
 *  Message Queue
 *
 *  Keeps a circular queue of messages. Provides the ability to change the length of the queue,
 *  access the messages in the queue in realtime via its methods.
 */

#pragma once

// std includes
#include <string>
#include <memory>
#include <cstdint>
#include <vector>

/** Common anantak declarations - using the anantak::message_type declaration */
#include <common_config.h>

/** Sensor message declarations. By making message queue aware of the message structure, we save
 *  the need for extra copying. If message structure is not known message queue will only reply
 *  with a string copy, which then needs to be copied into a message structure. Instead message
 *  queue can now directly copy into the message. This saves one string copy operation. */
#include "sensor_messages.pb.h"

namespace anantak {
  
/**
 *  Message Queue
 */
class MessageQueue {
 public:
  
  /** QueueDataType
   *  To add to the queue, caller provides this struct. MessageQueue does not know how to deserialize
   *  the messages. It can not extract timestamps out of the message. Upside is MessageQueue does not
   *  bother about format of the messages. This would make this more general. Messages are kept in
   *  serialized format as they are usually kept to be retransmitted. 
   */
  struct QueueDataType {        /**< Message Queue holds this */
    int64_t arrival_time;       /**< Time when message came in */
    int64_t message_time;       /**< Message timestamp */
    std::string message_str;    /**< Serialized message as a string */
    // Constructors
    QueueDataType():arrival_time(0), message_time(0), message_str("") {}
    QueueDataType(int64_t arr_tm, int64_t msg_tm, std::string msg_str):
        arrival_time(arr_tm), message_time(msg_tm), message_str(msg_str) {}
    QueueDataType(const QueueDataType& msg) {
      arrival_time = msg.arrival_time; message_time = msg.message_time;
      message_str = msg.message_str;
    }
    QueueDataType(int64_t arr_tm, int64_t msg_tm, const char* data_ptr, uint32_t size):
        arrival_time(arr_tm), message_time(msg_tm), message_str(data_ptr, size) {}
    // Functions
    bool SetValues(int64_t arr_tm, int64_t msg_tm, std::string msg_str) {
      arrival_time = arr_tm; message_time = msg_tm; message_str = msg_str; return true;
    }
    bool CopyData(const QueueDataType& msg) {
      arrival_time = msg.arrival_time; message_time = msg.message_time;
      message_str = msg.message_str; return true;
    }
  };
  
  /** QueueDataVectorType
   *  This is the type of a vector of messages to be sent to the message queue. We never want to
   *  copy the message strings, unless really needed so unique_ptrs are kept. 
   */
  typedef std::vector<std::unique_ptr<QueueDataType>> QueueDataVectorType;

  /** Struct to convey queue performance measures */
  struct QueuePerformanceType {
    float msg_add_rate;       /**< Rate of addition (/sec) of messages */
    float avg_add_delay;      /**< Average delay in adding messages (microsec/msg) */
    float msg_fetch_rate;     /**< Rate of fetch requests (/sec) */
    float avg_fetch_delay;    /**< Average delay in fetching messages (microsec/msg) */
    QueuePerformanceType():msg_add_rate(0), msg_fetch_rate(0), avg_add_delay(0),
        avg_fetch_delay(0) {}
  };

  /** Constructor creates a MessageQueue object with default settings */
  MessageQueue(std::string queue_name, int32_t queue_size);
  MessageQueue(std::string queue_name, int32_t queue_size, int32_t max_queue_size);
  
  /** Destructs the MessageQueue
   *  All objects will be automatically destructed. No special destructions are needed */
  virtual ~MessageQueue();

  /** Add a message to the queue. Ownership is transferred to the function */
  bool AddMessage(std::unique_ptr<QueueDataType> msg);
  
  /** Add messages to the queue. Vector of messages are then owned by the MessageQueue */
  bool AddMessages(std::unique_ptr<QueueDataVectorType> msgs);
  
  /** Get messages in a given time interval. This is one place where copies of messages are made,
   *  stored in a vector container as unique_ptr's and then a pointer to the vector is sent out.
   *  Owndership of the vector is to be assumed by the receiving code. */
  std::unique_ptr<QueueDataVectorType> FetchMessagesReceivedInTimeInterval(
      int64_t begin_time, int64_t end_time);
  
  /** Get latest message from the message queue. This could be useful for getting the latest
   *  message timestamp e.g. */
  std::unique_ptr<QueueDataType> FetchLatestMessage();
  
  /** Add all messages that lie in an interval to a Composite message - a pointer is used to
   *  indicate that this function will only modify the message. Message is owned by the calling
   *  function. Here we copy the message queue string to composite msg. Return number of msgs */
  int AddMessagesToCompositeMessage(anantak::CompositeMsg* composite_msg_ptr, int64_t begin_time,
      int64_t end_time);
  
  /** Add latest message to a composite message */
  int AddLatestMessageToCompositeMessage(anantak::CompositeMsg* composite_msg_ptr);  
  
  /** Change queue size - this allows changing the queue size dynamically.
   *  Need arises when the number of messages to be kept in the queue are a function of say, the
   *  uncertainty in measurement. An advancement might be that only a few of the measurements with
   *  high uncertainty can be kept and the rest intermediate ones can be discarded, but this can be
   *  done at a later time.
   *  In order to increase the size, pointers to objects before current circular index going to
   *  physical 0 index will need to moved to additional space. Implementation can simply work as:
   *  After vector size is increased, calculate old and new index of each circular element. If the
   *  index is the same, leave it untouched. If the index is changed, move it to the new location.
   *  In order to decrease the size, some pointers will need to be null-ified.
   */
  bool ChangeQueueSize(int32_t new_queue_size);
  
  /** Change max queue size
   *  Entire purpose of max queue size is the idea that dynamic queue size can be accomodated
   *  without reallocating memory for messages. It is possible that due to dynamic queue size
   *  max queue size needs to be increased, or if too large, decreased.
   */
  bool ChangeMaxQueueSize(int32_t new_max_queue_size);
  
  /** Start/stop tracking performance */
  bool StartTrackingPerformance(int32_t track_length);
  bool StopTrackingPerformance();
  
  /** Get performance for the queue */
  std::unique_ptr<QueuePerformanceType> GetQueuePerformance(); /**< Get current queue performance */
  
  /** Accessors, mutators */  
  inline std::string queue_name() {return queue_name_;}
  inline int32_t queue_size() {return queue_size_;}
  inline int32_t max_queue_size() {return max_queue_size_;}
  inline float max_queue_size_multiple() {return max_queue_size_multiple_;}
  inline bool track_performance() {return track_performance_;}
  
 private:
  // Settings
  std::string queue_name_;          /**< Name of the queue */
  int32_t queue_size_;              /**< Current size of the queue */
  int32_t max_queue_size_;          /**< Maximum queue size that has memeory allocated */
  float max_queue_size_multiple_;   /**< Max queue size = multiple * queue_size_ */

  // Operating variables
  QueueDataVectorType data_queue_;  /**< Data container */
  int32_t n_msgs_;                  /**< Number of messages in the queue */
  int32_t current_msg_index_;       /**< Index of the latest msg in the data_queue_ */
  int32_t oldest_msg_index_;        /**< Index of the oldest msg in the data_queue_ */
  
  // Useful internal functions
  inline int32_t next_index();      /**< Returns the next index for the queue */
  inline int32_t prev_index();      /**< Returns the previous index for the queue */
  bool Initiate();                  /**< Initiate the class */
  bool InitiateDataQueue();         /**< Initiates an empty queue by allocating memory */
  
  // Performance tracking
  bool InitiatePerformanceTracking(int32_t track_length); /**< Initiates performance tracking */
  inline int64_t get_wall_time_microsec();  /**< Returns current wall time in microseconds */
  bool track_performance_;                  /**< Indicator to start tracking performance */
  int32_t tracking_vector_lengths_;         /**< Keep this much history of add/fetch operations */
  std::vector<int64_t> data_add_times_;     /**< Circular queue of data add times */
  std::vector<int64_t> data_add_intervals_; /**< Circular queue of data add time intervals */
  int32_t data_add_times_index_;            /**< Current index of circular queue */
  std::vector<int64_t> data_fetch_times_;   /**< Circular queue of data fetch times */
  std::vector<int64_t> data_fetch_intervals_; /**< Circular queue of data fetch time intervals */
  std::vector<int32_t> data_fetch_n_msgs_;  /**< Circular queue of num of data fetch msgs */
  int32_t data_fetch_times_index_;          /**< Current index of circular queue */
  
};
  
  
  
}