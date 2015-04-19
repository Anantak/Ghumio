/**
 *  State
 *
 *  A model's state is usually a set of variables on real number line. But they could be enumerations 
 *  and integers depending on the model. E.g. pose of the machine at a time instant can be a vector
 *  of 7 doubles holding a quaternion and a position. 
 *
 *  State estimates are held in the estimates_tracker_ object. Each state could be estimated several
 *  times. 
 */

/**
 *  Design of State
 *  
 *  A state is usually a set of variables on real number line. But they could be enumerations and
 *  integers too depending on the model.
 *  
 *  Say we design probit/logit type models, we would solve a binary variable with a continuum [0,1].
 *  When we will solve for a model with classes, we would solve for probability(of this class). If
 *  we have an enum with 5 classes, its state representation will be a vector<double>.size=5, where
 *  the values can vary in the interval [0,1].
 *  
 *  Say, we are picking and placing pots in a yard. States are something like {kApproachingPot,
 *  kCloseToPot, kPickingPot, kPickedUpPot, ... etc.} Such a state machine will need to know with
 *  some certainty what the current state is. A Filter can be used to calculate the probs of
 *  being in a certain state at a time. When picking up a pot, kPickedUpPot will depend on measured 
 *  (1) pot location, (2) weight on the machine, (3) tension in pickup strings etc.
 *
 *  Say we are going from one place to another delivering things. A state machine could have states
 *  {kAtStartingWaitingForLoad, kGotLoad, kTravellingToDestination, kReachedDestination, ... etc.}
 *  Here the navigation filter's states will be inputs to the probabilities of each state. Plus 
 *  other inputs like user's inputs can help determine the state probabilities. kGotLoad could have
 *  input from the dynamics engine's mass estimate if there are no load sensors.
 *  
 *  If the number of states is variable new states are added by the model to its states. The model
 *  simply maintains the states.
 *
 *  Bottomline is that for the target of solving in a sliding window filter, we can assume that
 *  states are a array<double> or array<float>.
 *
 *  Inheritance versus templates
 *
 *  Each model's snapshot at any instant of time is its State. E.g. A simple kinematic model's
 *  state for a car-like machine consists of [timestamp, velocity, steering_angle]. For a visual
 *  odometry model that keeps track of last M camera poses, a state would be a 6d camera pose.
 *  If the visual odometry model also keeps track of 3d points in the environment, 3d location of
 *  a point in the environment (wrt a keyframe or global map) is a state.
 *
 *  State exists so that a Filter can keep track of states produces from any model without knowing
 *  about the type of state upfront. There are three ways of doing this:
 *  (1) Inheritance - All model states will derive from this class.
 *  (2) Templates - State is just a template that implements certain functions using a model-
 *      specific state. 
 *  (3) Curiously Recurring Template Pattern (CRTP) - State is a base class but other classes
 *      will derive from it using CRTP.
 *
 *  Inheritance will define ModelState: public State. This is the usual way for polymorphism. 
 *  Inheritance allows dynamic polymorphism. It allows us to store all states in a single
 *  container like std::vector<std::unique_ptr<State>> without knowing different types of model
 *  states. Downside is that this requires compiler to use virtual tables that is slightly less
 *  efficient than using static polymorphism (that is allowed by CRTP).
 *
 *  Templates used simply where a model will define a typedef State<ModelStructType> ModelState is
 *  another way of doing this. This is save a virtual function lookup call. But with this there is
 *  no obvious way to create a single container as each ModelState is its own class type.
 *
 *  CRTP allows static polymorphism that will save virtual method lookup table. But here too each
 *  ModelState will be a different class and so can not be saved in a single container.
 *
 *  Bottomline is that we will use inheritance. It allows us to store multiple state-types in a
 *  single container maintained by StatesTracker. I think it will also keep the code simple.
 *  
 */

#pragma once

/** std includes */
#include <memory>
#include <cstdint>
#include <vector>
#include <string>

/** anantak includes */
#include "Filter/circular_queue.h"

namespace anantak {

class State {
 public:
  /** State lifecycle goes as follows */
  enum StateLifeCycle { kAllocated, kConstant, kToBeCreated, kCreated, kToBeEstimated, kEstimated,
      kToBeMarginalized, kMarginalized, kToBeDeleted, kDeleted };
  
  /** Constructor */
  State(int32_t estimates_history_size = 100) {
    lifecycle_ = kAllocated;
    estimates_history_.Initiate(estimates_history_size);
  }
  
  /** Destructor */
  virtual ~State() {}

  /** Save the current estimate in history */
  bool SaveEstimate(int64_t iteration_id) {
    estimates_history_.add_element(iteration_id);
  }

  /** Accessors */
  inline int32_t num_estimates() {return estimates_history_.n_msgs();}
  inline int32_t estimates_history_size() {return estimates_history_.size();}
  inline int32_t estimates_history_current_index() {return estimates_history_.current_index();}
  inline int32_t estimates_history_oldest_index() {return estimates_history_.oldest_index();}
  
 protected:
  /** State estimates are arrays, but the state itself just holds an id */
  /** Lifecycle holder */
  StateLifeCycle lifecycle_; 
  /** Estimates history is a circular queue of iteration ids where the state was estimated */
  anantak::CircularQueue<int64_t> estimates_history_; 
  
};

/** StatePtr is used in containers of States. States are created by the Models and are specific
 *  to the model. Pointer help in keeping all derived classes in a single container maintained by
 *  StatesTracker.
 *  A StatePtr could be nullptr, but when a State is instantiated its memory is allocated
 */
typedef std::unique_ptr<State> StatePtr;


/** State Template
 *  A templated State implementation implementing the most important functions performed by State.
 *  When a State is created, its memory is allocated. 
 **/
template <typename StateType>
class StateTemplate : public State {
 public:
  /** Constructor */
  StateTemplate(int32_t estimates_history_size = 100) : State(estimates_history_size) {}
  
  /** Destructor */
  ~StateTemplate() {}
  
  /** Mutable accessor of the state_ by reference - caller has to make sure that reference it owns
   *  destructs before the state destructs */
  StateType& mutable_state() {
    return &state_;
  }
  
  /** Return a non-mutable accessor by reference to state_ */
  const StateType& state() const {
    return state_;
  }
  
  /** Return a copy of the state_ */
  StateType state() {
    return state_;    // send a copy
  }
  
  /** Set state by copying */
  bool set_state(const StateType& state) {
    state_ = state;   // copy
  }
  
  /** Returns iteration_id to the first state estimate since >= given_timestamp.
   *  If timestamp is older than kept state history, oldest available estimate is given.
   *  Returns by copy. It is OK to send a int64_t by copy, as a pointer will be same size */
  int64_t GetFirstEstimateSince(int64_t given_timestamp) {
    // Keep running backwards till previous_estimate_time < given_timestamp or at oldest_estimate
    // Get the history queue by reference
    const std::vector<int64_t>& estimates_history = estimates_history_.queue();
    int32_t current_idx = estimates_history_.current_index();
    int32_t oldest_idx = estimates_history_.oldest_index();
    int32_t history_size = estimates_history_.size();
    int32_t idx = current_idx;
    while ( (estimates_history[idx] >= given_timestamp) || (idx != oldest_idx) ) {
      idx = (idx == 0) ? history_size-1 : idx-1;
    }
    if (idx == current_idx) {
      LOG(WARNING) << "given_timestamp > latest estimate time! Asking for future estimate?";
    } else if (idx == oldest_idx) {
      LOG(WARNING) << "given_timestamp < oldest estimate time! Not enough history has been saved";
    } else {
      idx = (idx == history_size-1) ? 0 : idx+1;
    }
    return estimates_history[idx];
  }
  
 private:
  StateType state_;     /**< State data holder */
  
};


/** Declare some simple StateTemplates
 *    int64_t will be the most common
 *    string might be needed as well
**/
typedef StateTemplate<int64_t> StateInt64;
typedef StateTemplate<std::string> StateString;


} // namespace anantak

