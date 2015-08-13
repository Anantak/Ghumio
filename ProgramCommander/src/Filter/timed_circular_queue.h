/**
 *  Timed Circular Queue
 *
 *  Utility to help keep a circular queue with timestamps.
 *  
 **/

#pragma once

/** std includes */
#include <string>
#include <vector>
#include <cstdint>
#include <cmath>
#include <map>

/** Google libraries */
#include <glog/logging.h>

namespace anantak {

/** Timed Circular Queue
 *  Designed to be used where element copy is cheap, such as int64, int32, (may be short strings)
 *  For structs and large classes where data copy is expensive, use CircularPointer queue.
 *  This requires the element to be copy-able.
 *  Not sure if we need a name for this queue, so removing it, add back later if needed.
 **/

/** Timed Circular queue - rolling data storage in a vector. Allows memory resuse */
template<typename ElementType>
class TimedCircularQueue {
 public:
  
  // This element is stored in the queue
  struct TimedQueueElementType {
    int64_t timestamp;    // Timestamp of this element
    ElementType element;  // Actual element to be saved
    
    TimedQueueElementType():
      timestamp(0), element() {}
      
    TimedQueueElementType(const ElementType& value):
      timestamp(0), element(value) {}
      
    TimedQueueElementType(const TimedQueueElementType& r):
      timestamp(r.timestamp), element(r.element) {}
    
    TimedQueueElementType& operator= (const TimedQueueElementType& r) {
      timestamp = r.timestamp;
      element = r.element;
    }
  };
  
  /** Vector of Elements */
  typedef std::vector<TimedQueueElementType> ElementVectorType;
  
  bool is_initiated_; /**< Is this queue initiated yet? */
  ElementVectorType queue_; /**< Holds the data */
  uint32_t size_; /**< Size of the circular queue */
  uint32_t mask_; /**< Mask for mod operations */
  uint32_t n_msgs_; /**< Number of messages stored in the queue */
  uint32_t current_index_; /**< Current element index */
  uint32_t oldest_index_; /**< Oldest element index */
  uint64_t cycle_number_; /**< Cycle number of the circular queue */

  static ElementType empty_element_type_;   // This is declared at the end of this file
  
  /** Accessors */
  inline bool is_initiated() const {return is_initiated_;}
  //inline const std::string& name() const {return name_;}
  inline uint32_t size() const {return size_;}
  inline uint32_t n_msgs() const {return n_msgs_;}
  inline uint32_t current_index() const {return current_index_;}
  inline uint32_t oldest_index() const {return oldest_index_;}
  inline uint64_t cycle_number() const {return cycle_number_;}
  inline const ElementVectorType& queue() const {return queue_;}
  
  inline std::string ToString() {
    return std::to_string(size_)+" "+std::to_string(n_msgs_)+" "+std::to_string(current_index_)+" "
        +std::to_string(oldest_index_)+" "+std::to_string(cycle_number_);
  }
  
  /** Constructor taking the size of the queue - this is to be used generally */
  TimedCircularQueue(uint32_t size) {
    TimedQueueElementType queue_value;
    Initiate(size, queue_value);
  }

  /** Constructor taking the size and a value of element type */
  TimedCircularQueue(uint32_t size, const ElementType& value) {
    TimedQueueElementType queue_value(value);
    Initiate(size, queue_value);
  }

  /** Destructor */
  virtual ~TimedCircularQueue() {
    VLOG(3) << "Destructing TimedCircularQueue ";
    // everything should self destruct
  }
  
  /** Initiate - allocate memory - fill it up with copies of value */
  bool Initiate(uint32_t size, const TimedQueueElementType& value) {
    is_initiated_ = true;
    // Use a next power of 2 for the vector size
    uint32_t num_bits = uint32_t(std::floor(std::log(double(size))/std::log(2.))) + 1;
    size_ = uint32_t(std::pow(2., double(num_bits)));
    mask_ = size_ - 1;
    VLOG(1) << "Created timed circular queue. Provided size = " << size
        << ". size = " << size_ << " mask = " << mask_ << " = " << std::hex << mask_;
    n_msgs_ = 0;
    queue_.resize(size_, value);
    queue_.shrink_to_fit();
    current_index_ = 0;
    oldest_index_ = 0;
    cycle_number_ = 0;
    return true;
  }
  
  /** Clear the queue - this only resets the indexes. Elements are not 'reset' or 'zero-ed' */
  inline bool Clear() {
    n_msgs_ = 0;
    current_index_ = 0;
    oldest_index_ = 0;
    cycle_number_ = 0;
    return true;
  }
  
  inline uint32_t PlusOne(uint32_t index) const {
    index++;
    index &= mask_;
    return index;
  }
  
  inline uint32_t MinusOne(uint32_t index) const {
    index--;
    index &= mask_;
    return index;
  }
  
  inline uint32_t PlusOffset(uint32_t index, int32_t offset) const {
    index += offset;
    index &= mask_;
    return index;
  }
  
  inline uint32_t QueueIndex(uint32_t msg_idx) const {
    msg_idx = std::min(msg_idx, n_msgs_-1);
    return PlusOffset(oldest_index_, msg_idx);
  }
  
  bool CheckTimestamp() {
    if (queue_.at(current_index_).timestamp == 0) {
      LOG(ERROR) << "Increment called in a timed queue when last timestamp is 0";
      return false;
    }
    return true;
  }
  
  /** Increment the queue counter */
  inline bool increment() {
    if (n_msgs_ == 0) {current_index_=0; oldest_index_=0; n_msgs_=1;}
    else if (n_msgs_ <  size_) {CheckTimestamp(); current_index_=next_index(); n_msgs_++;}
    else if (n_msgs_ == size_) {CheckTimestamp(); current_index_=oldest_index_; oldest_index_=next_index();}
    else {LOG(ERROR) << "n_msgs_ > size_!! in queue"; return false;}
    if (current_index_==0) cycle_number_++;
    return true;
  }
  
  inline bool decrement() {
    if (n_msgs_ == 0) {return false;}
    else {current_index_=prev_index(); n_msgs_--;}
    if (current_index_==size_-1) cycle_number_--;
    return true;
  }
  
  /** Next queue index */
  inline uint32_t next_index() const {
    return PlusOne(current_index_);
  }
  
  /** Previous queue index */
  inline uint32_t prev_index() const {
    return MinusOne(current_index_);
  }
  
  /** This provides a const reference to the current element */
  inline const ElementType& CurrentElement() const {
    return is_initiated_ ? queue_.at(current_index_).element : empty_element_type_;
  }
  
  /** This provides a copy of the current element */
  inline ElementType Element() {
    return is_initiated_ ? queue_.at(current_index_).element : empty_element_type_;
  }

  /** Mutable element - this is still owned by the queue. But the calling function can modify it */
  inline ElementType* MutableElement() {
    return is_initiated_ ? &queue_.at(current_index_).element : NULL;    
  }
  
  inline bool SetTimestamp(const int64_t& ts) {
    queue_[current_index_].timestamp = ts;
  }
  
  inline bool SetStartingTimestamp(const int64_t& ts) {
    if (cycle_number_!=0) {
      LOG(ERROR) << "Starting timestamp can be set only at starting";
      return false;
    }
    increment();
    queue_[current_index_].timestamp = ts;
  }
  
  inline int64_t LastTimestamp() const {
    return queue_[current_index_].timestamp;
  }
  
  inline int64_t FirstTimestamp() const {
    return queue_[oldest_index_].timestamp;
  }
  
  /** Set element in the queue by copying the provided one. */
  inline bool SetElement(const ElementType& element) {
    if (!is_initiated_) {
      LOG(ERROR) << "First initialize the queue, cannot store in queue";
      return false;
    }
    queue_[current_index_].element = element;  // copy here, element parameter gets destructed
    return true;
  }
  
  /** Add element in the queue by copying the provided one. This calls increment before copying */
  inline bool AddElement(const ElementType& element) {
    if (!is_initiated_) {
      LOG(ERROR) << "First initialize the queue, cannot store in queue";
      return false;
    }
    //VLOG(1) << "queue = " << ToString() << " incrementing";
    increment();
    //VLOG(1) << "queue = " << ToString() << " now copying";
    queue_[current_index_].element = element;  // copy here, element parameter gets destructed
    //VLOG(1) << "queue = " << ToString() << " copied";
    return true;
  }
  
  /** Provide random access to messages ranging from [0..n_msgs-1] */
  // Const reference for read only
  inline const ElementType& At(uint32_t msg_idx) const {
    uint32_t q_idx = QueueIndex(msg_idx);
    return queue_.at(q_idx).element;
  }
  // Const reference to the first element
  inline const ElementType& Front() const {
    uint32_t q_idx = QueueIndex(0);
    return queue_.at(q_idx).element;
  }
  // Const reference to the last element
  inline const ElementType& Back() const {
    uint32_t q_idx = QueueIndex(n_msgs_-1);
    return queue_.at(q_idx).element;
  }
  // Mutable ptr to last element
  inline ElementType* BackPtr() {
    uint32_t q_idx = QueueIndex(n_msgs_-1);
    return is_initiated_ ? &queue_.at(q_idx).element : NULL;    
  }
  // Mutable reference for modification of the element
  inline ElementType& At(uint32_t msg_idx) {
    uint32_t q_idx = QueueIndex(msg_idx);
    return queue_.at(q_idx).element;
  }
  // Mutable pointer to an element
  inline ElementType* AtPtr(uint32_t msg_idx) {
    uint32_t q_idx = QueueIndex(msg_idx);
    return is_initiated_ ? &queue_.at(q_idx).element : NULL;    
  }
  // Mutable pointer to the next element - elements already exist. Can be modified by caller.
  inline ElementType* NextMutableElement() {
    increment();
    return is_initiated_ ? &queue_.at(current_index_).element : NULL;    
  }
  // Mutable pointer to penultimate element - elements already exist. Can be modified by caller.
  inline ElementType* PenultimateMutableElement() {
    if (n_msgs_<2) return NULL;
    uint32_t q_idx = QueueIndex(n_msgs_-2);
    return is_initiated_ ? &queue_.at(q_idx).element : NULL;    
  }

  // Nth last element. First last = last. Second last = one before last. and so on.
  //inline const ElementType* NthLastElementPtr(uint32_t n) {
  //  if (n_msgs_ < n) return nullptr;
  //  uint32_t q_idx = QueueIndex(n_msgs_-n);
  //  return is_initiated_ ? &queue_.at(q_idx).element : nullptr;    
  //}
  
  // Nth last element. First last = last. Second last = one before last. and so on.
  inline const ElementType& NthLastElement(uint32_t n) const {
    if (n_msgs_ < n) return empty_element_type_;
    uint32_t q_idx = QueueIndex(n_msgs_-n);
    return is_initiated_ ? queue_.at(q_idx).element : empty_element_type_;    
  }
  
  // Nth last element. First last = last. Second last = one before last. and so on.
  inline ElementType* NthLastElementPtr(uint32_t n) {
    if (n_msgs_ < n) return nullptr;
    uint32_t q_idx = QueueIndex(n_msgs_-n);
    return is_initiated_ ? &queue_.at(q_idx).element : nullptr;    
  }
  
  // Nth last timestamp. First last = last. Second last = one before last. and so on.
  inline int64_t NthLastTimestamp(uint32_t n) const {
    if (n_msgs_ < n) return 0;
    uint32_t q_idx = QueueIndex(n_msgs_-n);
    return is_initiated_ ? queue_.at(q_idx).timestamp : 0; 
  }
  
  // This identifies a fixed point in the queue
  struct FixedPoint {
    uint64_t cycle_number;    /**< Cycle number */
    uint32_t element_index;   /**< Current index of the element in the vector */
    // Default constructor
    FixedPoint(): cycle_number(0), element_index(0) {}
    FixedPoint(uint64_t cn, int32_t ci) {
      cycle_number = cn;
      element_index = ci;
    }
    FixedPoint(const FixedPoint& fp):
      cycle_number(fp.cycle_number),
      element_index(fp.element_index) {}
    FixedPoint operator= (const FixedPoint& fp) {
      cycle_number = fp.cycle_number;
      element_index = fp.element_index;
    }
    std::string ToString() const {
      // Get the timestamp for this fixed point
      return std::to_string(cycle_number)+", "+std::to_string(element_index);
    }
  };
  
  // Map of fixed points on this circular queue
  std::map<std::string, FixedPoint> fixed_points_;
  
  // Return a copy of the fixed point of current element in the queue
  inline FixedPoint CurrentFixedPoint() const {
    FixedPoint fixed_point(cycle_number_, current_index_);
    return fixed_point;
  }
  
  inline FixedPoint FirstFixedPoint() const {
    FixedPoint fixed_point;
    if (oldest_index_>current_index_) {
      fixed_point = FixedPoint(cycle_number_-1, oldest_index_);
    } else {
      fixed_point = FixedPoint(cycle_number_, oldest_index_);      
    }
    return fixed_point;
  }
  
  // Check if a fixed point is still valid
  inline bool IsFixedPointValid(const FixedPoint& fixed_point) const {
    return
      ((fixed_point.cycle_number > 0) &&
        ((fixed_point.cycle_number == cycle_number_
            && fixed_point.element_index <= current_index_
            && fixed_point.element_index >= 0) ||
         (fixed_point.cycle_number == cycle_number_-1
            && fixed_point.element_index > current_index_
            && fixed_point.element_index < size_))
      );
  }
  
  // Add a fixed point to the queue
  bool SetFixedPoint(const std::string& fp_name) {
    fixed_points_[fp_name] = CurrentFixedPoint();
  }
  
  inline bool FixedPointExists(const std::string& fp_name) const {
    auto search = fixed_points_.find(fp_name);
    if (search == fixed_points_.end()) {
      return false;
    }
    return true;
  }
  
  // Fixed point to string
  inline std::string FixedPointToString(const std::string& fp_name) const {
    if (!FixedPointExists(fp_name)) {
      return fp_name+" does not exist";
    }
    const FixedPoint& fixed_point = fixed_points_.at(fp_name);
    int64_t ts = queue_.at(fixed_point.element_index).timestamp;
    return fixed_point.ToString()+", "+std::to_string(ts);
  }
  
  inline std::string FixedPointToString(const FixedPoint& fixed_point) const {
    int64_t ts = queue_.at(fixed_point.element_index).timestamp;
    return fixed_point.ToString()+", "+std::to_string(ts);
  }
  
  // Return a copy of fixed point
  inline FixedPoint GetFixedPoint(const std::string& fp_name) const {
    if (!FixedPointExists(fp_name)) {
      LOG(ERROR) << fp_name << " fixed point is not in the queue.";
      FixedPoint fp;
      return fp;
    }
    return fixed_points_.at(fp_name);
  }
  
  inline FixedPoint NextFixedPoint(const FixedPoint& fp) const {
    FixedPoint fixed_point(fp);
    if (fixed_point.cycle_number==0 && fixed_point.element_index==0) {
      fixed_point.cycle_number = 1;
      fixed_point.element_index = 0;
    } else {
      fixed_point.element_index = PlusOne(fixed_point.element_index);
      if (fixed_point.element_index == 0) fixed_point.cycle_number++;
    }
    return fixed_point;
  }
  
  // n = 1 means last element, 2 means second last and so on
  inline ElementType* NthLastElementPtrBefore(const FixedPoint& fp, uint32_t n) {
    int32_t offset = -n+1;
    uint32_t q_idx = PlusOffset(fp.element_index, offset);
    return is_initiated_ ? &queue_.at(q_idx).element : nullptr;    
  }
  
  // Return a copy of fixed point
  inline FixedPoint GetNextFixedPoint(const std::string& fp_name) const {
    if (!FixedPointExists(fp_name)) {
      LOG(ERROR) << fp_name << " fixed point is not in the queue.";
      FixedPoint fp;
      return fp;
    }
    return NextFixedPoint(fixed_points_.at(fp_name));
  }
  
  inline FixedPoint PrevFixedPoint(const FixedPoint& fp) const {
    FixedPoint fixed_point(fp);
    if (fixed_point.cycle_number==0 && fixed_point.element_index==0) {
      fixed_point.cycle_number = 0;
      fixed_point.element_index = 0;
    } else {
      fixed_point.element_index = MinusOne(fixed_point.element_index);
      if (fixed_point.element_index == size_-1) fixed_point.cycle_number--;
    }
    return fixed_point;
  }
  
  inline bool IncrementFixedPoint(FixedPoint& fp) const {
    fp = NextFixedPoint(fp);
    return true;
  }
  
  inline bool IncrementFixedPoint(const std::string& fp_name) {
    if (!FixedPointExists(fp_name)) {
      LOG(ERROR) << fp_name << " fixed point is not in the queue.";
      return false;
    }
    FixedPoint fp = fixed_points_[fp_name];
    fixed_points_[fp_name] = NextFixedPoint(fp);
    return true;
  }
  
  // Are we past a given fixed point?
  inline bool IsNotPastFixedPoint(const std::string& ref_fp_name, const FixedPoint& fp) const {
    if (!FixedPointExists(ref_fp_name)) {
      LOG(ERROR) << ref_fp_name << " fixed point is not in the queue.";
      return false;
    }
    const FixedPoint& ref_fp = fixed_points_.at(ref_fp_name);
    return ((fp.cycle_number <= ref_fp.cycle_number) && (fp.element_index <= ref_fp.element_index));
  }

  // Are we before a given fixed point?
  inline bool IsNotAtFixedPoint(const std::string& ref_fp_name, const FixedPoint& fp) const {
    if (!FixedPointExists(ref_fp_name)) {
      LOG(ERROR) << ref_fp_name << " fixed point is not in the queue.";
      return false;
    }
    const FixedPoint& ref_fp = fixed_points_.at(ref_fp_name);
    return ((fp.cycle_number <= ref_fp.cycle_number) && (fp.element_index < ref_fp.element_index));
  }

  // Are we past the end?
  inline bool IsNotPastTheEnd(const FixedPoint& fp) const {
    return ((fp.cycle_number <= cycle_number_) && (fp.element_index <= current_index_));
  }
  
  // Are we past the end?
  inline bool IsNotAtTheEnd(const FixedPoint& fp) const {
    return ((fp.cycle_number <= cycle_number_) && (fp.element_index < current_index_));
  }
  
  // Get const element reference at fixed point
  inline const ElementType& ElementAtFixedPoint(const FixedPoint& fixed_point,
      bool check_validity=false) const {
    uint32_t q_idx = fixed_point.element_index;
    if (check_validity) {
      if (!IsFixedPointValid(fixed_point)) q_idx = current_index_;
    }
    return queue_.at(q_idx).element;   
  }
  
  // Get timestamp in the queue at fixed point
  inline int64_t Timestamp(const FixedPoint& fixed_point,
      bool check_validity=false) const {
    uint32_t q_idx = fixed_point.element_index;
    if (check_validity) {
      if (!IsFixedPointValid(fixed_point)) q_idx = current_index_;
    }
    return queue_.at(q_idx).timestamp;   
  }
  
  // Get timestamp in the queue at fixed point
  inline int64_t Timestamp(const std::string& fp_name,
      bool check_validity=false) const {
    uint32_t q_idx = current_index_;
    if (!FixedPointExists(fp_name)) {
      LOG(ERROR) << fp_name << " fixed point is not in the queue.";
    } else {
      const FixedPoint& fixed_point = fixed_points_.at(fp_name);
      q_idx = fixed_point.element_index;
      if (check_validity) {
        if (!IsFixedPointValid(fixed_point)) q_idx = current_index_;
      }
    }
    return queue_.at(q_idx).timestamp;   
  }
  
  // Get mutable element pointer at fixed point
  inline ElementType* MutableElementAtFixedPoint(const FixedPoint& fixed_point,
      bool check_validity=false) {
    uint32_t q_idx = fixed_point.element_index;
    if (check_validity) {
      if (!IsFixedPointValid(fixed_point)) q_idx = current_index_;
    }
    return &queue_.at(q_idx).element;
  }
  
  // Get const element reference at fixed point
  inline const ElementType& ElementAtFixedPoint(const std::string& fp_name,
      bool check_validity=false) const {
    uint32_t q_idx = current_index_;
    if (!FixedPointExists(fp_name)) {
      LOG(ERROR) << fp_name << " fixed point is not in the queue.";
    } else {
      FixedPoint& fixed_point = fixed_points_[fp_name];
      q_idx = fixed_point.element_index;
      if (check_validity) {
        if (!IsFixedPointValid(fixed_point)) q_idx = current_index_;
      }
    }
    return queue_.at(q_idx).element;   
  }
  
  // Get mutable element pointer at fixed point
  inline ElementType* MutableElementAtFixedPoint(const std::string& fp_name,
      bool check_validity=false) {
    uint32_t q_idx = current_index_;
    if (!FixedPointExists(fp_name)) {
      LOG(ERROR) << fp_name << " fixed point is not in the queue.";
    } else {
      FixedPoint& fixed_point = fixed_points_[fp_name];
      q_idx = fixed_point.element_index;
      if (check_validity) {
        if (!IsFixedPointValid(fixed_point)) q_idx = current_index_;
      }
    }
    return &queue_.at(q_idx).element;
  }
  
  inline bool PositionFixedPointBeforeTimestamp(const std::string& fp_name, const int64_t& ts,
                                                bool ok_to_position_at_end = false,
                                                bool include_beginning = false) {
    if (!FixedPointExists(fp_name)) {
      LOG(ERROR) << fp_name << " fixed point is not in the queue.";
      return false;
    }
    return PositionFixedPointBeforeTimestamp(fixed_points_.at(fp_name), ts, ok_to_position_at_end,
                                             include_beginning);
  }
  
  // Position fixed point before timestamp
  //  return false if timestamp was not found in the queue. Leaves the fixed point unchanged if so
  inline bool PositionFixedPointBeforeTimestamp(FixedPoint& fp, const int64_t& ts,
                                                bool ok_to_position_at_end = false,
                                                bool include_beginning = false) {
    // Check if ts is greater than the last ts
    if (ok_to_position_at_end) {
      if (include_beginning) {
        if (LastTimestamp() <= ts) {
          fp = CurrentFixedPoint();
          VLOG(3) << "Set the fixed point at the ending.";
          return true;
        }
      } else {
        if (LastTimestamp() < ts) {
          fp = CurrentFixedPoint();
          VLOG(3) << "Set the fixed point at the ending.";
          return true;
        }
      }
      //else if (ts < FirstTimestamp()) {
      //  fixed_points_[fp_name] = FirstFixedPoint();
      //  VLOG(3) << "Set the fixed point at the beginning."
      //  return true;
      //}
    }
    bool interval_found = false;
    
    // Search in forward direction
    FixedPoint fp0 = fp;
    FixedPoint fp1 = NextFixedPoint(fp0);
    while (!interval_found && IsFixedPointValid(fp1) && Timestamp(fp1)>0) {
      if (include_beginning) {
        interval_found = (queue_.at(fp0.element_index).timestamp <= ts &&
                          queue_.at(fp1.element_index).timestamp > ts);
      } else {
        interval_found = (queue_.at(fp0.element_index).timestamp < ts &&
                          queue_.at(fp1.element_index).timestamp >= ts);
      }
      if (!interval_found) {
        fp0 = fp1;
        fp1 = NextFixedPoint(fp0);
      }
    }
    if (interval_found) VLOG(3) << "Found interval in fwd pass at "
        << fp0.ToString() << " " << Timestamp(fp0) << ", "
        << fp1.ToString() << " " << Timestamp(fp1);
    
    // If not found, search in backward direction.
    if (!interval_found) {
      fp1 = fp;
      fp0 = PrevFixedPoint(fp1);
      while (!interval_found && IsFixedPointValid(fp0) && Timestamp(fp0)>0) {
        if (include_beginning) {
          interval_found = (queue_.at(fp0.element_index).timestamp <= ts &&
                            queue_.at(fp1.element_index).timestamp > ts);
        } else {
          interval_found = (queue_.at(fp0.element_index).timestamp < ts &&
                            queue_.at(fp1.element_index).timestamp >= ts);
        }
        if (!interval_found) {
          fp1 = fp0;
          fp0 = PrevFixedPoint(fp1);
        }
        if (interval_found) VLOG(3) << "Found interval in bkwd pass at "
            << fp0.ToString() << " " << Timestamp(fp0) << ", "
            << fp1.ToString() << " " << Timestamp(fp1);
      }
    }
    if (interval_found) {
      fp = fp0;
      //VLOG(3) << "Fixed point set at " << fp.ToString() << " " << Timestamp(fp);
    }
    //if (interval_found) VLOG(3) << "Found interval in back pass"; else VLOG(3) << "Interval not found in back pass";
    return interval_found;
  }
  
};

//Static element for timed circular queue
template <typename ElementType>
ElementType TimedCircularQueue<ElementType>::empty_element_type_ = ElementType();

} // namespace anantak
