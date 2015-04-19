/**
 *  Stereo Camera Calibration Constant Model
 *
 *  Non-varying stereo camera calibration parameters.
 *
 *  Doing this with a header-only file. 
 **/

#pragma once

/** std includes */
#include <array>

/** main header include */
#include "Filter/model.h"

/** Google Logging library */
#include <glog/logging.h>

namespace anantak {

class StereoCameraCalibrationConstantModel : public anantak::Model {
 public:
  /** Model's typedefs */
  
  /** Main constructor - model is meaningless without name, type and config file */
  StereoCameraCalibrationConstantModel(std::string model_name, std::string model_type,
                                       std::string config_file) :
      anantak::Model(model_name, model_type, config_file) {
  }
  
  /** Destructor - all should self destruct */
  ~StereoCameraCalibrationConstantModel() {
    VLOG(1) << "Destructing StereoCameraCalibrationConstantModel " << model_name_;
  }

  /** Initiator - called before model can be used */
  bool Initiate(
      int64_t max_sliding_window_interval,
      ObservationsVectorStoreMapCirPtrQueue* observations_tracker,
      StateCirPtrQueuePtrMap* states_tracker,
      EstimatesPtrMapCirPtrQueue* estimates_tracker) {
    
    // Call base class initiator
    if (!Model::Initiate(max_sliding_window_interval, observations_tracker, states_tracker,
        estimates_tracker)) {
      LOG(ERROR) << "Could not initiate base Model for " << model_name_;
      return false;
    }
    
    /** Read the config file
     *  Stereo calibration that stays constant will load Extrinsic and Intrinsic calibrations of
     *  the stereo pair. For pin-hole projection, we have 5 parameters per camera and 6 parameters
     *  for relative position. So a total of 16 parameters. This can be represented as a 16x1 array
     *  kept in Estimates.
     *  StereoCameraCalibrationConstantModel keeps the calibration constant. So there is a single
     *  state that is marked as kConstant in the lifecycle. In estimates, space is allocated in
     *  each estimate map with constant parameters copied.
     *  For solving, no constraints are built on these estimates. These are treated as constants.
     **/
    num_of_values_per_state_ = 16;
    max_num_of_states_per_iteration_ = 1;
    
    // Done!
    VLOG(1) << "Initiated model " << model_name_;
    is_initiated_ = true;
    return true;
  }
  
  /** Allocate memory for states on the heap */
  bool AllocateMemoryForStates() {    
    /** StereoCameraCalibrationConstantModel has a constant state. State_id could be kept as a
     *  fixed number say 0. So we can keep the state of type StateInt64. **/
    // Create a circular queue with a single element
    StateCirPtrQueuePtr queue_ptr(new StateCirPtrQueue());   // allocate queue object 
    queue_ptr->Initiate(model_name_+".States", max_num_of_states_per_iteration_); // Allocate queue
    for (int i=0; i<max_num_of_states_per_iteration_; i++) {
      // Allocate a single new StateInt64 - use default iteration history size
      std::unique_ptr<anantak::StateInt64> state_ptr(new anantak::StateInt64());
      if (!state_ptr) {
        LOG(ERROR) << "Could not allocate memory for states for " << model_name_;
        return false;
      }
      // Pass on the StateInt64 to queue
      if (!queue_ptr->add_element(std::move(state_ptr))) {
        LOG(ERROR) << "Could not move states to state circular queue for " << model_name_;
        return false;
      }
    }
    // Move the Circular queue to the map
    (*states_tracker_)[model_name_] = std::move(queue_ptr);
    VLOG(2) << "Allocated memory for " << max_num_of_states_per_iteration_ << " states for "
        << model_name_;
    return true;
  }
  
  bool AllocateMemoryForEstimates() {
    /* Go through all estimate maps, and allocate memory */
    for (int i=0; i<estimates_tracker_->size(); i++) {
      EstimatesPtr estimate_ptr(new EstimatesType()); // Allocate on heap
      estimate_ptr->estimates.resize(num_of_values_per_state_ * 
                                     max_num_of_states_per_iteration_); // Allocate the VectorXd
      estimate_ptr->state_ids.resize(max_num_of_states_per_iteration_);  // Allocate memory
      // Move it into the map
      EstimatesPtrMap* est_map = estimates_tracker_->mutable_element_ptr();
      (*est_map)[model_name_] = std::move(estimate_ptr);
      estimates_tracker_->increment();
    }
    VLOG(2) << "Allocated memory in " << estimates_tracker_->size() << " estimates for "
        << model_name_;
        
    /* Given this is a constant model, it might make sense to fill up the values in estimates too.
     * But this is the job of InitiateEstimates(). We can set the estimates here and just
     * return empty there */
    
    return true;
  }
  
  bool CreateStates() {}
  bool MarkStates() {}
  
  /** Before solving begins, we initiate the estimates */
  bool InitiateEstimates() {
    /* As this is a constant model, estimates are initiated upfront. */
    return true;
  }
  
  /** Solving here */
  
  bool ProcessEstimates() {}
  bool GetResultsAsMessage() {}
  
 private:
  int32_t num_of_values_per_state_;
  int32_t max_num_of_states_per_iteration_;
  
};


} // namespace anantak
