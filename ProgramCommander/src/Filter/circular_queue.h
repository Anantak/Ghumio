/**
 *  Circular Queue
 *
 *  Utility to help keep a circular queue. This pattern keeps coming up. This is templated
 *  so we keep all code in the header file.
 *  
 **/

#pragma once

/** std includes */
#include <string>
#include <vector>
#include <cstdint>

/** Google libraries */
#include <glog/logging.h>

namespace anantak {

/** Circular Queue of a primitive type.
 *  Designed to be used where element copy is cheap, such as int64, int32, (may be short strings)
 *  For structs and large classes where data copy is expensive, use CircularPointer queue.
 *  This requires the element to be copy-able.
 *  Not sure if we need a name for this queue, so removing it, add back later if needed.
 **/

/** Fixed point in a circular queue. This is used for random access in the queue **/
struct FixedPointCQ {
  uint64_t cycle_number;    /**< Cycle number */
  int32_t  element_index;   /**< Current index of the element in the vector */
  // Default constructor
  FixedPointCQ(): cycle_number(0), element_index(-1) {}
  FixedPointCQ(uint64_t cn, int32_t ci) {
    cycle_number = cn;
    element_index = ci;
  }
  FixedPointCQ(const FixedPointCQ& fp):
    cycle_number(fp.cycle_number),
    element_index(fp.element_index) {}
  FixedPointCQ operator= (const FixedPointCQ& fp) {
    cycle_number = fp.cycle_number;
    element_index = fp.element_index;
  }
  std::string to_string() {return std::to_string(cycle_number)+", "+std::to_string(element_index);}
};

/** A segment of data in the circular queue */
struct DataSegmentCQ {
  FixedPointCQ begin_point; // starting element begin index
  FixedPointCQ end_point;   // ending element begin index
  int32_t size;   // including both begin and end
  // Default constructor
  DataSegmentCQ(): begin_point(), end_point(), size(0) {}
  DataSegmentCQ(const FixedPointCQ& bp, const FixedPointCQ& ep, int32_t sz):
  begin_point(bp), end_point(ep), size(sz) {}
};

/** Circular queue - rolling data storage in a vector. Allows memory resuse */
template<typename ElementType>
class CircularQueue {
 public:
  /** Vector of Elements */
  typedef std::vector<ElementType> ElementVectorType;
  
  /** Constructor - empty constructor to help use of CircularQueue as a class member */
  CircularQueue() {
    is_initiated_ = false;
    //name_ = "";
    size_ = 0;
    n_msgs_ = 0;
    queue_.reserve(0);  // no space reserved
    queue_.resize(0);   // there are no elements
    current_index_ = -1;
    oldest_index_ = -1;
    cycle_number_ = 0;
  }
  
  /** Constructor taking the size of the queue - this is to be used generally */
  CircularQueue(int32_t size) {
    Initiate(size);
  }

  /** Constructor taking the size and a value of element type */
  CircularQueue(int32_t size, const ElementType& value) {
    Initiate(size, value);
  }

  /** Destructor */
  virtual ~CircularQueue() {
    //VLOG(3) << "Destructing CircularQueue ";
    // everything should self destruct
  }
  
  /** Initiate - allocate memory */
  bool Initiate(int32_t size) {
    is_initiated_ = true;
    //name_ = name;
    size_ = size;
    n_msgs_ = 0;
    queue_.resize(size_);
    queue_.shrink_to_fit();
    current_index_ = -1;
    oldest_index_ = -1;
    cycle_number_ = 0;
    return true;
  }

  /** Initiate - allocate memory - fill it up with copies of value */
  bool Initiate(int32_t size, const ElementType& value) {
    is_initiated_ = true;
    //name_ = name;
    size_ = size;
    n_msgs_ = 0;
    queue_.resize(size_, value);
    queue_.shrink_to_fit();
    current_index_ = -1;
    oldest_index_ = -1;
    cycle_number_ = 0;
    return true;
  }
  
  /** Clear the queue - this only resets the indexes. Elements are not 'reset' or 'zero-ed' */
  bool Clear() {
    n_msgs_ = 0;
    current_index_ = -1;
    oldest_index_ = -1;
    cycle_number_ = 0;
    return true;
  }
  
  /** Resize - resize the queue by copying the values */
  bool Resize(int32_t new_size) {
    /** This is most probably not the most efficient way of doing this, but this operation will be
     *  done rarely. For now we do it this way, if this start causing too much delay, revisit */
    if (!is_initiated_) {
      LOG(ERROR) << "CircularQueue is not yet initiated, can not resize it.";
      return false;
    }
    if (n_msgs_ == 0) {
      size_ = new_size;
      queue_.resize(size_);
      VLOG(3) << "Resized queue to " << new_size;
    } else {
      std::vector<ElementType> temp_queue;
      temp_queue.resize(size_);
      // move all elements from queue_ to temp_queue in order starting from index=0,size_-1,...;
      for (int i=0; i<size_; i++) {
        int32_t idx = (current_index_ + size_ - i) % size_;
        int32_t temp_idx = (size_ - i) % size_;
        temp_queue[temp_idx] = queue_[idx]; // copy operation
      }
      // resize the empty queue_ 
      queue_.resize(new_size);
      queue_.shrink_to_fit();
      // put elements back
      int32_t n_elements_to_put_back = (new_size < size_) ? new_size : size_;
      for (int i=0; i<n_elements_to_put_back; i++) {
        int32_t idx = (size_ - i) % size_;
        int32_t new_idx = (new_size - i) % new_size;
        queue_[new_idx] = temp_queue[idx];  // copy operation
      }
      // reset the indexes and sizes
      size_ = new_size;
      n_msgs_ = n_elements_to_put_back;
      current_index_ = 0;
      oldest_index_ = (new_size - (n_elements_to_put_back-1)) % new_size;
      cycle_number_ = 0;
      VLOG(3) << "Resized queue to " << new_size << " shifted indexes ";
      // temp queue destructs here along with all elements in it.
    }
    return true;
  }
  
  /** This provides a const reference to the current element */
  inline const ElementType& element() const {
    return is_initiated_ ? queue_.at(current_index_) : empty_element_type_;
  }
  
  /** This provides a copy of the current element */
  inline ElementType element() {
    return is_initiated_ ? queue_.at(current_index_) : empty_element_type_;
  }

  /** Mutable element - this is still owned by the queue. But the calling function can modify it */
  inline ElementType* mutable_element() {
    return is_initiated_ ? &queue_.at(current_index_) : NULL;    
  }
  
  /** Set element in the queue by copying the provided one. */
  inline bool set_element(const ElementType& element) {
    if (!is_initiated_) {
      LOG(ERROR) << "First initialize the queue, cannot store in queue";
      return false;
    }
    queue_[current_index_] = element;  // copy here, element parameter gets destructed
    return true;
  }
  
  /** Add element in the queue by copying the provided one. This calls increment before copying */
  inline bool add_element(const ElementType& element) {
    if (!is_initiated_) {
      LOG(ERROR) << "First initialize the queue, cannot store in queue";
      return false;
    }
    increment();
    queue_[current_index_] = element;  // copy here, element parameter gets destructed
    return true;
  }

  /** Increment the queue counter */
  inline bool increment() {
    if (n_msgs_ == 0) {current_index_=0; oldest_index_=0; n_msgs_=1;}
    else if (n_msgs_ <  size_) {current_index_=next_index(); n_msgs_++;}
    else if (n_msgs_ == size_) {current_index_=oldest_index_; oldest_index_=next_index();}
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
  inline int32_t next_index() {
    return (current_index_+1)%size_;
  }
  
  /** Previous queue index */
  inline int32_t prev_index() {
    return (current_index_ == 0) ? size_-1 : (current_index_-1)%size_;
  }
  
  /** Provide random access to messages ranging from [0..n_msgs-1] */
  // Const reference for read only
  inline const ElementType& at(int32_t msg_idx) const {
    msg_idx = std::min(msg_idx, n_msgs_-1);
    int32_t q_idx = (oldest_index_ + msg_idx)%size_;
    //return is_initiated_ ? queue_.at(q_idx) : empty_element_type_;
    return queue_.at(q_idx);
  }
  // Const reference to the first element
  inline const ElementType& front() const {
    int32_t msg_idx = 0;
    int32_t q_idx = (oldest_index_ + msg_idx)%size_;
    //return is_initiated_ ? queue_.at(q_idx) : empty_element_type_;
    return queue_.at(q_idx);
  }
  // Const reference to the last element
  inline const ElementType& back() const {
    int32_t msg_idx = n_msgs_-1;
    int32_t q_idx = (oldest_index_ + msg_idx)%size_;
    //return is_initiated_ ? queue_.at(q_idx) : empty_element_type_;
    return queue_.at(q_idx);
  }
  // Mutable ptr to last element
  inline ElementType* back_ptr() {
    int32_t msg_idx = n_msgs_-1;
    int32_t q_idx = (oldest_index_ + msg_idx)%size_;
    return is_initiated_ ? &queue_.at(q_idx) : NULL;    
  }
  // Mutable reference for modification of the element
  inline ElementType& at(int32_t msg_idx) {
    msg_idx = std::min(msg_idx, n_msgs_-1);
    int32_t q_idx = (oldest_index_ + msg_idx)%size_;
    //return is_initiated_ ? queue_.at(q_idx) : empty_element_type_;
    return queue_.at(q_idx);
  }
  // Mutable pointer to an element
  inline ElementType* at_ptr(int32_t msg_idx) {
    msg_idx = std::min(msg_idx, n_msgs_-1);
    int32_t q_idx = (oldest_index_ + msg_idx)%size_;
    return is_initiated_ ? &queue_.at(q_idx) : NULL;    
  }  
  // Mutable pointer to the next element - elements already exist. Can be modified by caller.
  inline ElementType* next_mutable_element() {
    increment();
    return is_initiated_ ? &queue_.at(current_index_) : NULL;    
  }
  // Mutable pointer to penultimate element - elements already exist. Can be modified by caller.
  inline ElementType* penultimate_mutable_element() {
    if (n_msgs_<2) return NULL;
    int32_t q_idx = (oldest_index_ + n_msgs_-2)%size_;
    return is_initiated_ ? &queue_.at(q_idx) : NULL;    
  }
  
  // Return a copy of the fixed point of current element in the queue
  inline FixedPointCQ CurrentFixedPoint() const {
    FixedPointCQ fixed_point(cycle_number_, current_index_);
    return fixed_point;
  }
  
  // Return a copy of the fixed point of element before current element in the queue
  inline FixedPointCQ PenultimateFixedPoint() const {
    if (n_msgs_ < 2) return FixedPointCQ();
    FixedPointCQ fp(cycle_number_, current_index_-1);
    if (fp.element_index < 0) {fp.cycle_number -= 1; fp.element_index = size_-1;}
    return fp;
  }
  
  // Return a copy of fixed point for the first element in the queue
  inline FixedPointCQ FrontFixedPoint() const {
    if (oldest_index_ <= current_index_) {
      return FixedPointCQ(cycle_number_, oldest_index_);
    }
    return FixedPointCQ(cycle_number_-1, oldest_index_);
  }
  
  // Check if a fixed point is still valid
  inline bool IsFixedPointValid(const FixedPointCQ& fixed_point) const {
    return
      ((fixed_point.cycle_number == cycle_number_
          && fixed_point.element_index <= current_index_
          && fixed_point.element_index >=0) ||
       (fixed_point.cycle_number == cycle_number_-1
          && fixed_point.element_index > current_index_
          && fixed_point.element_index < size_));
  }
  
  // Check if fixed point has reached the end
  inline bool FixedPointIsBeforeLastElement(const FixedPointCQ& fixed_point) const {
    return
      ((fixed_point.cycle_number == cycle_number_
          && fixed_point.element_index < current_index_) ||
       (fixed_point.cycle_number == cycle_number_-1));
  }
  
  inline bool FixedPointIsBeforeOrAtLastElement(const FixedPointCQ& fixed_point) const {
    return
      ((fixed_point.cycle_number == cycle_number_
          && fixed_point.element_index <= current_index_) ||
       (fixed_point.cycle_number == cycle_number_-1));
  }
  
  // To be used in conjunction with Increment() to iterate using for() statement
  inline bool NotPastTheEnd(const FixedPointCQ& fixed_point) const {
    return
      ((fixed_point.cycle_number == cycle_number_
          && fixed_point.element_index <= current_index_) ||
       (fixed_point.cycle_number == cycle_number_-1));
  }
  
  inline bool FixedPointIsBeforeMarker(const FixedPointCQ& fp_marker, const FixedPointCQ& fp) const {
    return
      ((fp.cycle_number == fp_marker.cycle_number
          && fp.element_index < fp_marker.element_index) ||
       (fp.cycle_number == fp_marker.cycle_number-1));
  }
  
  inline bool FixedPointIsBeforeOrAtMarker(const FixedPointCQ& fp_marker, const FixedPointCQ& fp) const {
    return
      ((fp.cycle_number == fp_marker.cycle_number
          && fp.element_index <= fp_marker.element_index) ||
       (fp.cycle_number == fp_marker.cycle_number-1));
  }
  
  // Check if fixed point has reached the beginning
  inline bool FixedPointIsAfterFirstElement(const FixedPointCQ& fixed_point) const {
    return
      ((oldest_index_ < current_index_ && fixed_point.cycle_number == cycle_number_
          && fixed_point.element_index > oldest_index_) ||
       (oldest_index_ > current_index_ && fixed_point.cycle_number == cycle_number_
          && fixed_point.element_index > -1) ||
       (oldest_index_ > current_index_ && fixed_point.cycle_number == cycle_number_-1
          && fixed_point.element_index > oldest_index_));
  }
  
  inline bool IncrementFixedPoint(FixedPointCQ& fixed_point) const {
    if (fixed_point.cycle_number==cycle_number_ && fixed_point.element_index==current_index_) {
      // We are at the end. Can not increment!
      return false;
    }
    if (fixed_point.cycle_number==0 && fixed_point.element_index==-1) {
      fixed_point.cycle_number = 1;
      fixed_point.element_index = 0;
      return true;
    }
    fixed_point.element_index++;
    if (fixed_point.element_index > size_-1) {
      fixed_point.element_index = 0;
      fixed_point.cycle_number++;      
    }
    return true;
  }
  
  // Here we allow fixed point to go past the last element
  inline bool Increment(FixedPointCQ& fixed_point) const {
    if (fixed_point.cycle_number==0 && fixed_point.element_index==-1) {
      fixed_point.cycle_number = 1;
      fixed_point.element_index = 0;
      return true;
    }
    fixed_point.element_index++;
    if (fixed_point.element_index > size_-1) {
      fixed_point.element_index = 0;
      fixed_point.cycle_number++;      
    }
    return true;
  }
  
  inline bool DecrementFixedPoint(FixedPointCQ& fixed_point) const {
    if (fixed_point.cycle_number==0 && fixed_point.element_index==-1) {
      return false;
    }
    fixed_point.element_index--;
    if (fixed_point.element_index < 0) {
      fixed_point.element_index = size_-1;
      fixed_point.cycle_number--;
    }
    return true;
  }
  
  // Get the element at fixed point
  inline ElementType* MutableElementAtFixedPoint(const FixedPointCQ& fixed_point,
      bool check_validity=false) {
    if (check_validity) {
      if (!IsFixedPointValid(fixed_point)) return NULL;
    }
    return is_initiated_ ? &queue_.at(fixed_point.element_index) : NULL;    
  }
  
  // Get the element after fixed point
  inline ElementType* MutableElementAfterFixedPoint(const FixedPointCQ& fixed_point,
      bool check_validity=false) {
    if (check_validity) {
      if (!IsFixedPointValid(fixed_point)) return NULL;
    }
    int32_t q_idx = (fixed_point.element_index + 1)%size_;
    return is_initiated_ ? &queue_.at(q_idx) : NULL;    
  }
  
  // Get the element after fixed point
  inline const ElementType* ConstElementAfterFixedPoint(const FixedPointCQ& fixed_point,
      bool check_validity=false) const {
    if (check_validity) {
      if (!IsFixedPointValid(fixed_point)) return NULL;
    }
    int32_t q_idx = (fixed_point.element_index + 1)%size_;
    return is_initiated_ ? &queue_.at(q_idx) : NULL;    
  }
  
  // How many elements are there from provided fixed point till the end? including both
  inline int32_t NumElementsTillEnd(const FixedPointCQ& fixed_point) const {
    if (!IsFixedPointValid(fixed_point)) return -1;
    return (size_ + current_index_ - fixed_point.element_index)%size_ + 1;
  }

  // How many elements are there from provided fixed point till another fixed point? including both
  inline int32_t NumElementsTillFixedPoint(const FixedPointCQ& point0, const FixedPointCQ& point1) const {
    int32_t n0 = NumElementsTillEnd(point0);
    if (n0 < 0) return -1;
    int32_t n1 = NumElementsTillEnd(point1);
    if (n1 < 0) return -1;
    if (n0 < n1) return -1;
    return n0-n1+1;
  }

  // Provide const pointer access to an element in the queue starting from fixed point
  inline ElementType* MutableElementFromFixedPoint(const FixedPointCQ& fixed_point,
      const int32_t offset, bool check_validity=true) {
    if (check_validity) {
      int32_t num_elements_till_end = NumElementsTillEnd(fixed_point);
      if (num_elements_till_end < 0) return NULL;
      if (offset >= num_elements_till_end) return NULL;
    }
    int32_t q_idx = (fixed_point.element_index + offset)%size_;
    return is_initiated_ ? &queue_.at(q_idx) : NULL;    
  }
  
  // How many elements are there from provided fixed point till the end? excluding starting point
  inline int32_t NumElementsAfterFixedPoint(const FixedPointCQ& fixed_point) const {
    if (!IsFixedPointValid(fixed_point)) return -1;
    return (size_ + current_index_ - fixed_point.element_index)%size_;
  }
  
  // How many elements are there from provided fixed point till another fixed point? excluding starting point
  inline int32_t NumElementsAfterFixedPoint(const FixedPointCQ& point0, const FixedPointCQ& point1) const {
    int32_t n0 = NumElementsAfterFixedPoint(point0);
    if (n0 < 0) return -1;
    int32_t n1 = NumElementsAfterFixedPoint(point1);
    if (n1 < 0) return -1;
    if (n0 < n1) return -1;
    return n0-n1;
  }
  
  // Provide pointer access to an element in the queue after the fixed point
  inline ElementType* MutableElementAfterFixedPoint(const FixedPointCQ& fixed_point,
      const int32_t offset, bool check_validity=true) {
    if (check_validity) {
      int32_t num_elements_till_end = NumElementsAfterFixedPoint(fixed_point);
      if (num_elements_till_end < 0) return NULL;
      if (offset >= num_elements_till_end) return NULL;
    }
    int32_t q_idx = (fixed_point.element_index + offset + 1)%size_;
    return is_initiated_ ? &queue_.at(q_idx) : NULL;    
  }
  
  // Create DataSegmentCQ starting with a given msg index going to another msg idx
  inline bool GetDataSegment(int32_t msg_idx0, int32_t msg_idx1, DataSegmentCQ* data_seg) const {
    msg_idx0 = std::min(msg_idx0, n_msgs_-1);
    int32_t q_idx0 = (oldest_index_ + msg_idx0)%size_;
    msg_idx1 = std::min(msg_idx1, n_msgs_-1);
    int32_t q_idx1 = (oldest_index_ + msg_idx1)%size_;
    FixedPointCQ fp0(cycle_number_, q_idx0);
    FixedPointCQ fp1(cycle_number_, q_idx1);
    int32_t sz = NumElementsTillFixedPoint(fp0, fp1);
    data_seg->begin_point = fp0;
    data_seg->end_point = fp1;
    data_seg->size = sz;
    return true;
  }
  
  // Create DataSegmentCQ starting with a given msg index going to the end of the queue
  inline bool GetDataSegmentTillEnd(int32_t msg_idx, DataSegmentCQ* data_seg) const {
    msg_idx = std::min(msg_idx, n_msgs_-1);
    int32_t q_idx = (oldest_index_ + msg_idx)%size_;
    FixedPointCQ fp0(cycle_number_, q_idx);
    int32_t sz = NumElementsTillEnd(fp0);
    FixedPointCQ fp1(cycle_number_, current_index_);
    data_seg->begin_point = fp0;
    data_seg->end_point = fp1;
    data_seg->size = sz;
    return true;
  }
  
  // Create DataSegmentCQ after a given fixed point going to the end of the queue
  inline bool GetDataSegmentAfterFixedPoint(const FixedPointCQ& fixed_point, DataSegmentCQ* data_seg,
      bool check_validity=false) const {
    if (check_validity) {
      if (!IsFixedPointValid(fixed_point)) return false;
    }
    int32_t elem_idx = fixed_point.element_index + 1;
    FixedPointCQ fp0(fixed_point.cycle_number + (elem_idx/size_), elem_idx%size_);
    int32_t sz = NumElementsTillEnd(fp0);
    FixedPointCQ fp1(cycle_number_, current_index_);
    data_seg->begin_point = fp0;
    data_seg->end_point = fp1;
    data_seg->size = sz;
    return true;
  }
  
  /** Accessors */
  inline bool is_initiated() const {return is_initiated_;}
  //inline const std::string& name() const {return name_;}
  inline int32_t size() const {return size_;}
  inline int32_t n_msgs() const {return n_msgs_;}
  inline int32_t current_index() const {return current_index_;}
  inline int32_t oldest_index() const {return oldest_index_;}
  inline uint64_t cycle_number() const {return cycle_number_;}
  inline const ElementVectorType& queue() const {return queue_;}
  
  inline std::string to_string() {
    return std::to_string(size_)+" "+std::to_string(n_msgs_)+" "+std::to_string(current_index_)+" "
        +std::to_string(oldest_index_)+" "+std::to_string(cycle_number_);
  }
  
  //std::string name_; /**< Name of this queue - help identify it, debug etc.*/
  bool is_initiated_; /**< Is this queue initiated yet? */
  ElementVectorType queue_; /**< Holds the data */
  int32_t size_; /**< Size of the circular queue */
  int32_t n_msgs_; /**< Number of messages stored in the queue */
  int32_t current_index_; /**< Current element index */
  int32_t oldest_index_; /**< Oldest element index */
  uint64_t cycle_number_; /**< Cycle number of the circular queue */

  static ElementType empty_element_type_;
};

} // namespace anantak
