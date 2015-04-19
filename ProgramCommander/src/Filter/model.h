/**
 *  Model
 *
 *  Model creates new states, creates constraints, marginalizes old states.
 *
 *  Model gets called by the Filter in every time iteration where Model can start its work for
 *  the iteration. New data/readings for the iteration are provided by the filter.
 *
 *  The base Model is an abstract class.
 *  E.g. A derived model can implement a constant state model, one that does not change the state.
 *  This is used in keeping the calibration parameters. E.g. Stereo camera extrinsic calibration
 *  parameters can be kept as constants if they are not being updated. A derived Model can then
 *  extend the constant base Model to make the parameters update in time.
 *
 */

#pragma once

/** std includes */
#include <string>
#include <map>

/** anantak includes */
#include "Filter/observation.h"
#include "Filter/states_tracker.h"
#include "Filter/estimates_tracker.h"

namespace anantak {

class Model {
 public:
  /** Constructor - gets a config file */
  Model(std::string model_name, std::string model_type, std::string config_file) {
    model_name_ = model_name;
    model_type_ = model_type;
    config_file_ = config_file;
    VLOG(1) << "Creating Model " << model_name_;
    
    is_initiated_ = false;
    has_allocated_states_memory_ = false;
  }
  
  /** Destructor - This should never be called - derived class destructor should be called */
  virtual ~Model() {}

  /** Initiator - get pointers to observations keeper and states tracker
   *    Stores the pointers to observations keeper and states tracker
   *    Loads the config file
   **/
  virtual bool Initiate(
      int64_t max_sliding_window_interval,
      ObservationsVectorStoreMapCirPtrQueue* observations_tracker,
      StateCirPtrQueuePtrMap* states_tracker,
      EstimatesPtrMapCirPtrQueue* estimates_tracker) {
    
    // Initiate the variables and pointers
    max_filter_sliding_window_interval_ = max_sliding_window_interval;
    observations_tracker_ = observations_tracker;
    states_tracker_ = states_tracker;
    estimates_tracker_ = estimates_tracker;
    
    return true;
  }
  
  /** AllocateMemoryForStates
   *  Models calculate states in every iteration. Depending on the model its states could depend on:
   *  (1) Time passed since last iteration: e.g.
   *        Kinematic model will add new velocity, acceleration, ego-motion states.
   *  (2) Observations: e.g.
   *        MSCKF would instantiate new states with camera images.
   *        Hybrid MSCKF would instantiate based on camera images and feature tracks.
   *  (3) States of other models: e.g.
   *        Can't think of an example right now, but it will come...
   *  In several implementations states are linked to camera images or IMU readings. For dynamics
   *  we might prefer to link states to wall time, creating states at say 100Hz.
   *  
   *  Filter will call this method after creating the model.
   **/
  virtual bool AllocateMemoryForStates() {}
  
  /** AllocateMemoryForEstimates
   *  Models know the length of their states array. E.g. a pose estimate will have length of six,
   *  a quaternion will have length of four etc. Here the model allocates space for the estimates
   *  in the EstimatesTracker. 
   **/
  virtual bool AllocateMemoryForEstimates() = 0;    // making this class abstract
  
  /** Create States
   *  At the beginning of every iteration, Filter calls this method of each Model to let it build
   *  its new states based on new observations.
   **/
  virtual bool CreateStates() {}

  /** Mark States
   *  After new states are created, models mark the lifecycle of the states:
   *  { kToBeCreated, kCreated, kToBeEstimated, kEstimated,
   *    kToBeMarginalized, kMarginalized, kToBeDeleted, kDeleted }
   *  StatesTracker owns the state markings.
   **/
  virtual bool MarkStates() {}
  
  /** Initiate Estimates
   *  Models initiate the values of estimates, fill up state_ids, set num_states
   *  in EstimatesTracker
   **/
  virtual bool InitiateEstimates() {}
  
  /** Process Estimates
   *  After solver is done solving the problem, EstimatesTracker has all the states. 
   **/
  virtual bool ProcessEstimates() {}
  
  /** Get Results As Message
   *  Returns a Protobuf message with all latest iteration results. This can be serialized and
   *  transmitted to a DataQueue as needed.
   **/
  virtual bool GetResultsAsMessage() {}
  
 protected:
  // Basics
  std::string model_name_;          /**< model name */
  std::string model_type_;          /**< model type */
  std::string config_file_;         /**< config_file path */
  
  // Variables and pointers to trackers for Observations, States and Estimates
  int64_t max_filter_sliding_window_interval_;
  anantak::ObservationsVectorStoreMapCirPtrQueue* observations_tracker_;
  anantak::StateCirPtrQueuePtrMap* states_tracker_;
  anantak::EstimatesPtrMapCirPtrQueue* estimates_tracker_;
  
  // Usability indicators
  bool is_initiated_;                   /**< if the model has been initiated */
  bool has_allocated_states_memory_;    /**< if the memory has been allocated for states */
  

};

/** Filter owns all Models. These are kept in a map with keys of model names. We always use
 *  unique pointers inside containers to save from derived models getting 'sliced' as memory
 *  allocated inside the container using base Model may not be large enough to hold the derived
 *  Model. We also define an iterator to go through the map. */
typedef std::unique_ptr<Model> ModelPtr;
typedef std::map<std::string, ModelPtr> ModelPtrMap;
typedef ModelPtrMap::iterator ModelPtrMapIterator;

} // namespace anantak
