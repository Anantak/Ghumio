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
    return true;
  }

  /** Initiate - allocate memory */
  bool Initiate(int32_t size, const ElementType& value) {
    is_initiated_ = true;
    //name_ = name;
    size_ = size;
    n_msgs_ = 0;
    queue_.resize(size_, value);
    queue_.shrink_to_fit();
    current_index_ = -1;
    oldest_index_ = -1;
    return true;
  }
  
  /** Clear the queue - this only resets the indexes. Elements are not 'reset' or 'zero-ed' */
  bool Clear() {
    n_msgs_ = 0;
    current_index_ = -1;
    oldest_index_ = -1;    
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
  
  /** Accessors */
  inline bool is_initiated() const {return is_initiated_;}
  //inline const std::string& name() const {return name_;}
  inline int32_t size() const {return size_;}
  inline int32_t n_msgs() const {return n_msgs_;}
  inline int32_t current_index() const {return current_index_;}
  inline int32_t oldest_index() const {return oldest_index_;}
  inline const ElementVectorType& queue() const {return queue_;}
  
  //std::string name_; /**< Name of this queue - help identify it, debug etc.*/
  bool is_initiated_; /**< Is this queue initiated yet? */
  ElementVectorType queue_; /**< Holds the data */
  int32_t size_; /**< Size of the circular queue */
  int32_t n_msgs_; /**< Number of messages stored in the queue */
  int32_t current_index_; /**< Current element index */
  int32_t oldest_index_; /**< Oldest element index */

  static ElementType empty_element_type_;
};

} // namespace anantak
