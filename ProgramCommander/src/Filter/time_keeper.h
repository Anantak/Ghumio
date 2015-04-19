/**
 *  Time Keeper
 *
 *  Timer keeps a circular queue of accummulated times and number of events.
 **/

#pragma once

/** std includes */
#include <string>
#include <memory>
#include <vector>
#include <cstdint>
#include <sys/time.h>

/** Google libraries */
#include <glog/logging.h>

namespace anantak {

class TimeKeeper {
 public:
  
  struct Snap {
    int64_t acc_time;
    int32_t acc_events;
    int64_t curr_time;
  };

  /** Default constructor */
  TimeKeeper() {}

  /** Main constructor - takes a name and size of queue to be tracked */
  TimeKeeper(std::string name, int32_t size) {
    Initiate(name, size);
  }
  
  /** Initiator - takes a name and size of queue to be tracked */
  bool Initiate(std::string name, int32_t size) {
    name_ = name;
    size_ = size;
    n_snaps_ = 0;
    queue_.resize(size_);
    queue_.shrink_to_fit();
    current_index_ = -1;
    oldest_index_ = -1;
    timer_running_ = false;
    start_time_ = 0;
    increment();
    queue_[current_index_].acc_time = 0;
    queue_[current_index_].acc_events = 0;
    queue_[current_index_].curr_time = get_wall_time_microsec();
    return true;
  }
  
  /** Destructor - all should self destruct */
  virtual ~TimeKeeper() {}
  
  /** Start timer for this iteration */
  bool StartTimer() {
    start_time_ = get_wall_time_microsec();
    timer_running_ = true;
    return true;
  }
  
  /** Stop timer - default number of events is 1, but could be any positive number */
  bool StopTimer(int32_t n_events = 1) {
    if (!timer_running_) {
      LOG(ERROR) << name_ << " timer is not running so can not stop it.";
      return false;
    }
    int64_t current_time = get_wall_time_microsec();
    int64_t time_passed = current_time - start_time_;
    timer_running_ = false;
    int32_t previous_index = current_index_;
    increment();
    queue_[current_index_].acc_time = queue_[previous_index].acc_time + time_passed;
    queue_[current_index_].acc_events = queue_[previous_index].acc_events + n_events;
    queue_[current_index_].curr_time = current_time;
    start_time_ = 0;
    return true;
  }

  /** Increment the queue counter */
  inline bool increment() {
    if (n_snaps_ == 0) {current_index_=0; oldest_index_=0; n_snaps_=1;}
    else if (n_snaps_ <  size_) {current_index_=next_index(); n_snaps_++;}
    else if (n_snaps_ == size_) {current_index_=oldest_index_; oldest_index_=next_index();}
    else {LOG(ERROR) << "n_snaps_ > size_!! in queue " << name_; return false;}
    return true;
  }

  /** Next queue index */
  inline int32_t next_index() {
    return (current_index_+1)%size_;
  }
  
  /** Previous queue index */
  inline int32_t prev_index() {
    return (current_index_ == 0) ? size_ : (current_index_-1)%size_;
  }
  
  /** Average time of each event */
  inline float average_event_time() {
    return ((n_snaps_>0) && (queue_[current_index_].acc_events!=queue_[oldest_index_].acc_events)) ?
        float(queue_[current_index_].acc_time - queue_[oldest_index_].acc_time) /
        float(queue_[current_index_].acc_events - queue_[oldest_index_].acc_events) :
        -1.0f;
  }
  
  /** Average time of each timer on/off */
  inline float average_timer_time() {
    return (n_snaps_>0) ?
        float(queue_[current_index_].acc_time - queue_[oldest_index_].acc_time) /
        float((current_index_ + size_ - oldest_index_)%size_) :
        -1.0f;
  }

  /** Call rate */
  inline float call_rate() {
    return (n_snaps_>0) ?
        float((current_index_ + size_ - oldest_index_)%size_) * 1000000.0f / 
        float(queue_[current_index_].curr_time - queue_[oldest_index_].curr_time) :
        -1.0f;
  }

  /** Accessors */
  inline bool is_running() {return timer_running_;}
  inline const std::string& name() const {return name_;}
  inline int32_t size() {return size_;}
  inline int32_t n_snaps() {return n_snaps_;}
  inline int32_t current_index() {return current_index_;}
  inline int32_t oldest_index() {return oldest_index_;}

 private:
  std::string name_;  /**< Name of the Performance Tracker */
  std::vector<Snap> queue_; /**< Queue of snaps */
  int32_t size_;  /**< Number of snaps to keep track of */
  int32_t n_snaps_; /**< Number of snaps stored in the queue */
  int32_t current_index_; /**< Current element index */
  int32_t oldest_index_; /**< Oldest element index */
  
  bool timer_running_; /**< Indicates if timer has been triggered */
  int64_t start_time_; /**< Starting time */

  /** Inlined utility to get wall time in microseconds */
  inline static int64_t get_wall_time_microsec() {
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) (tv.tv_sec * 1000000 + tv.tv_usec);  
  }
  
};

} // namespace anantak
