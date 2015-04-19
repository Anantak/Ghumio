/**
 *  State
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
 *  Technical implementation of State
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
 *  Abstract base State
 *
 *  Should we model this as an abstract class - something that is always intended to be derived?
 *  That may make sense.
 *  
 */

/** std includes */
#include <memory>
#include <cstdint>
#include <vector>

namespace anantak {

class State {
 public:
  /** Constructor */
  State(int32_t estimates_history_size) {
    estimates_history_size_ = estimates_history_size;
    num_estimates_ = 0;
    current_index_ = -1;
    oldest_index_ = -1;
  }
  
  /** Destructor */
  virtual ~State();

  /** Save the current estimate in history */
  virtual bool SaveEstimate(int64_t estimate_time) = 0;   // Makes State an abstract class

  /** Accessors */
  inline int32_t estimates_history_size() {return estimates_history_size_;}
  inline int32_t num_estimates_in_history() {return num_estimates_;}
  inline int32_t estimates_history_current_index() {return current_index_;}
  inline int32_t estimates_history_oldest_index() {return oldest_index_;}
  
 protected:
  int32_t estimates_history_size_;  /**< These many states in the history will be kept */
  int32_t num_estimates_;           /**< Number of estimates stored in the queue */
  int32_t current_index_;           /**< Index of current estimate in circular queue */
  int32_t oldest_index_;            /**< Index of oldest estimate in circular queue */

  /** Increment the queue counter */
  inline bool increment() {
    if (num_estimates_ == 0) {current_index_=0; oldest_index_=0; num_estimates_=1;}
    else if (num_estimates_ <  estimates_history_size_) {current_index_=next_index(); num_estimates_++;}
    else if (num_estimates_ == estimates_history_size_) {current_index_=oldest_index_; oldest_index_=next_index();}
    else {LOG(ERROR) << "num_estimates_ > estimates_history_size_!! in queue "; return false;}
    return true;
  }
  
  /** Next queue index */
  inline int32_t next_index() {
    return (current_index_+1)%estimates_history_size_;
  }
  
  /** Previous queue index */
  inline int32_t prev_index() {
    return (current_index_ == 0) ? estimates_history_size_-1
        : (current_index_-1)%estimates_history_size_;
  }
  
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
  StateTemplate(int32_t estimates_history_size) : State(estimates_history_size) {
    estimates_.resize(estimates_history_size_);  // here the memory is allocated with default instances of StateType
  }
  
  /** Destructor */
  ~StateTemplate() {}
  
  /** Mutable accessor of the state_ */
  StateType& mutable_state() {
    return &state_;
  }
  
  /** Non-nutable accessor by reference to state_ */
  const StateType& state() const {
    return state_;
  }
  
  /** Save the current estimate */
  bool SaveEstimate(int64_t estimate_time) {
    increment();  // call the base class' increment to move pointer to next element
    estimates_[current_index_].estimate_time = estimate_time;
    estimates_[current_index_].estimate = state_; // copy - requires StateType to be copy-able
    return true;
  }
  
  /** Returns reference to the first state estimate since >= a given_timestamp.
   *  If timestamp is older than kept state history, oldest available estimate is given */
  const StateType& GetFirstEstimateAfter(int64_t given_timestamp) const {
    // Keep running backwards till previous_estimate_time < given_timestamp or at oldest_estimate
    int32_t idx = current_index_;
    while ( (estimates_[idx].estimate_time >= given_timestamp) || (idx != oldest_index_) ) {
      idx = (idx == 0) ? estimates_history_size_-1 : idx-1;
    }
    if (idx == current_index_) {
      LOG(WARNING) << "given_timestamp > latest estimate time?!";
    } else if (idx == oldest_index_) {
      LOG(WARNING) << "given_timestamp < oldest estimate time!, not enough history saved";
    } else {
      idx = (idx == estimates_history_size_-1) ? 0 : idx+1;
    }
    return estimates_[idx].estimate;
  }
  
 private:
  StateType state_;     /**< State data holder */
  
  struct StateEstimateType {
    int64_t estimate_time;
    StateType estimate;
  };
  std::vector<StateEstimateType> estimates_;  /**< keeps a circular history of estimates */
  
};

} // namespace anantak

