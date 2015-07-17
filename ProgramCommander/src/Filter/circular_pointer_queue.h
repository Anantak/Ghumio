/**
 *  Circular Pointer Queue
 *
 *  Utility to help keep a circular pointer queue. This pattern keeps coming up. This is templated
 *  so we keep all code in the header file.
 *  
 **/

#pragma once

/** std includes */
#include <string>
#include <memory>
#include <vector>
#include <cstdint>

/** Google libraries */
#include <glog/logging.h>

namespace anantak {

template<typename ElementType>
class CircularPointerQueue {
 public:
  /** Element pointer */
  typedef std::unique_ptr<ElementType> ElementPtrType;
  
  /** Constructor - empty constructor */
  CircularPointerQueue() {
    is_initiated_ = false;
    name_ = "UninitiatedQueue";
    size_ = 0;
    n_msgs_ = 0;
    queue_.reserve(0);  // no space reserved
    queue_.resize(0);   // there are no elements
    current_index_ = -1;
    oldest_index_ = -1;
  }
  
  /** Destructor */
  virtual ~CircularPointerQueue() {
    VLOG(1) << "Destructing CircularPointerQueue " << name_;
    // everything should self destruct
  }
  
  /** Initiate - allocate memory for the pointers
   *  IMPORTANT: Memory of the underlying objects that are pointed to by the circular queue is NOT
   *  allocated by queue methods. This is done by the calling functions. This is done so that
   *  this queue can store object derived from ElementType. E.g. A queue of type State can store
   *  objects dervied from State. This is the primary reason for the queue.
   */
  bool Initiate(std::string name, int32_t size) {
    name_ = name;
    size_ = size;
    queue_.resize(size_);
    queue_.shrink_to_fit();
    is_initiated_ = true;
    return true;
  }
  
  /** Resize - resize the queue by moving the pointers */
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
      VLOG(2) << "Resized queue " << name_ << " to " << new_size;
    } else {
      std::vector<ElementPtrType> temp_queue;
      temp_queue.resize(size_);
      // move all elements from queue_ to temp_queue in order starting from index=0,size_-1,...;
      for (int i=0; i<size_; i++) {
        int32_t idx = (current_index_ + size_ - i) % size_;
        int32_t temp_idx = (size_ - i) % size_;
        temp_queue[temp_idx] = std::move(queue_[idx]);
      }
      // resize the empty queue_ 
      queue_.resize(new_size);
      queue_.shrink_to_fit();
      // put elements back
      int32_t n_elements_to_put_back = (new_size < size_) ? new_size : size_;
      for (int i=0; i<n_elements_to_put_back; i++) {
        int32_t idx = (size_ - i) % size_;
        int32_t new_idx = (new_size - i) % new_size;
        queue_[new_idx] = std::move(temp_queue[idx]);
      }
      // reset the indexes and sizes
      size_ = new_size;
      n_msgs_ = n_elements_to_put_back;
      current_index_ = 0;
      oldest_index_ = (new_size - (n_elements_to_put_back-1)) % new_size;
      VLOG(2) << "Resized queue " << name_ << " to " << new_size << " shifted indexes ";
      // temp queue destructs here along with all elements in it.
    }
    return true;
  }
  
  /** This provides a const reference to the current element */
  inline const ElementType& element() const {
    return is_initiated_ ? *(queue_.at(current_index_)) : empty_element_type_;
  }

  /** Mutable element ref - still owned by the queue. But the calling function can modify it */
  inline ElementType& mutable_element() {
    return is_initiated_ ? *(queue_.at(current_index_)) : empty_element_type_;
  }
  
  /** Mutable element ptr - still owned by the queue. But the calling function can modify it */
  inline ElementType* mutable_element_ptr() {
    return is_initiated_ ? queue_.at(current_index_).get() : NULL;    
  }
  
  /** Set the current pointer to the provided one. Ownership is transferred to the queue */
  inline bool set_element(ElementPtrType element_ptr) {
    if (!is_initiated_) {
      LOG(ERROR) << "First initialize the queue. provided element is now lost! Allocate again.";
      return false;
    }
    queue_[current_index_] = std::move(element_ptr);  // original contents are auto-destructed
    return true;
  }

  /** Increment and set the element pointer to the provided one. Queue owns the element now */
  inline bool add_element(ElementPtrType element_ptr) {
    if (!is_initiated_) {
      LOG(ERROR) << "First initialize the queue. provided element is now lost! Allocate again.";
      return false;
    }
    increment();
    queue_[current_index_] = std::move(element_ptr);  // original contents are auto-destructed
    return true;
  }
  
  /** Increment the queue counter */
  inline bool increment() {
    if (n_msgs_ == 0) {current_index_=0; oldest_index_=0; n_msgs_=1;}
    else if (n_msgs_ <  size_) {current_index_=next_index(); n_msgs_++;}
    else if (n_msgs_ == size_) {current_index_=oldest_index_; oldest_index_=next_index();}
    else {LOG(ERROR) << "n_msgs_ > size_!! in queue " << name_; return false;}
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
  
  /** Accessors */
  inline bool is_initiated() {return is_initiated_;}
  inline const std::string& name() const {return name_;}
  inline int32_t size() {return size_;}
  inline int32_t n_msgs() {return n_msgs_;}
  inline int32_t current_index() {return current_index_;}
  inline int32_t oldest_index() {return oldest_index_;}
  
 private:
  std::string name_; /**< Name of this queue - help identify it, debug etc.*/
  bool is_initiated_; /**< Is this queue initiated yet? */
  std::vector<ElementPtrType> queue_; /**< Holds the data */
  int32_t size_; /**< Size of the circular queue */
  int32_t n_msgs_; /**< Number of messages stored in the queue */
  int32_t current_index_; /**< Current element index */
  int32_t oldest_index_; /**< Oldest element index */

  static ElementType empty_element_type_;
};

template<typename ElementType>
ElementType CircularPointerQueue<ElementType>::empty_element_type_ = ElementType();

} // namespace anantak
