/**
 * Test Message Queue
 *
 * We want to test the MessageQueue for a number of things:
 * (1) Populate msgq with different types of messages and whole thing works seemlessly.
 * (2) Save data from the queue to disk and load back
 * 
 */

/** std includes */
#include <string>

/** Google logging and flags libraries */
#include <glog/logging.h>
#include <gflags/gflags.h>

/** Google Protocol Buffers library */
#include <fcntl.h>
#include <google/protobuf/message.h>
#include <google/protobuf/text_format.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>

/** anantak includes */
#include "DataQueue/message_queue.h"

/** for wall time */
#include <sys/time.h>

class Sleeper {
 public:
  Sleeper(float freq) {
    loop_frequency_ = freq;
    loop_timeperiod_ = int64_t(float(1000000)/freq);
    last_touch_time_ = get_wall_time_microsec();
    max_frequency_ = 0.0;
    VLOG(2) << "Created a looper at " << loop_timeperiod_/1000 << "ms/" << loop_frequency_ << "Hz";
  }
  virtual ~Sleeper() {}
  int64_t Sleep() {
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
  inline float max_frequency() {return max_frequency_;}
 private:
  float loop_frequency_;
  float max_frequency_;
  int64_t loop_timeperiod_;   // microsec
  int64_t last_touch_time_;
  int64_t get_wall_time_microsec() {
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) (tv.tv_sec * 1000000 + tv.tv_usec);
  }
};

int64_t get_wall_time_microsec() {
  struct timeval tv;
  gettimeofday (&tv, NULL);
  return (int64_t) (tv.tv_sec * 1000000 + tv.tv_usec);
}

int main(int argc, char** argv) {
  
  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);
  // Parse the commandline flags 
  google::ParseCommandLineFlags(&argc, &argv, true);
  // Verify the version of protobuf library 
  GOOGLE_PROTOBUF_VERIFY_VERSION;
  // Setting the glog output to stderr by default
  FLAGS_logtostderr = 1;

  std::string queue_name = "machine_interface";
  int32_t queue_size = 200;
  anantak::MessageQueue msg_q(queue_name, queue_size);
  msg_q.StartTrackingPerformance(100);
  
  // run a loop while adding messages to the queue. Get performance intermittently.
  int64_t start_time = get_wall_time_microsec();
  int64_t run_time = int64_t(1000000*10);   // microsecs
  int64_t current_time = get_wall_time_microsec();
  int64_t queue_status_interval = int64_t(1000000);  // microsec
  int64_t queue_status_time = 0;
  int64_t read_data_period = int64_t(10000);
  int64_t read_data_time = 0;
  
  Sleeper looper(200);
  
  while (current_time < start_time + run_time) {
    
    // Create a message, add to MessageQueue
    std::string msg_str = "message: " + std::to_string(current_time);
    std::unique_ptr<anantak::MessageQueue::QueueDataType> msg(
        new anantak::MessageQueue::QueueDataType(
          current_time, current_time, msg_str
        ));
    msg_q.AddMessage(std::move(msg));
    
    // get status if it is time
    if (current_time > queue_status_time + queue_status_interval) {
      std::unique_ptr<anantak::MessageQueue::QueuePerformanceType> q_perf;
      q_perf = msg_q.GetQueuePerformance();
      VLOG(1) << "Performance: " << q_perf->msg_add_rate << "Hz " << q_perf->avg_add_delay << "us "
          << q_perf->msg_fetch_rate << "Hz " << q_perf->avg_fetch_delay << "us "
          << " max_freq " << looper.max_frequency() << "Hz";
      queue_status_time = current_time;
    }
    
    // read data if it is time
    if (current_time > read_data_time + read_data_period) {
      std::unique_ptr<anantak::MessageQueue::QueueDataVectorType> q_data;
      q_data = msg_q.FetchMessagesReceivedInTimeInterval(read_data_time, current_time);
      VLOG(3) << "Read " << q_data->size() << " messages";
      read_data_time = current_time;
    }
    
    // sleep a little and update current time
    current_time = looper.Sleep();
  }
  
  return 0;
}

