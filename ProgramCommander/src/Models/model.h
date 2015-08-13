/** Model
 *
 * Interface for a model
 */

#pragma once

namespace anantak {

/* Notes:
 * - Model knows about time only from the starting timestamp and each iteration timestamp. It does
 *   not use a get time method to fetch time.
 * -
 */

class Model {
 public:
  /** Constructor - gets a config file */
  Model() {}
  
  /** Destructor - This should never be called - derived class destructor should be called */
  virtual ~Model() {}
  
  // Iteration interval
  virtual const uint64_t& IterationInterval() const = 0;
  
  // Run iteration with observations
  virtual bool RunIteration(
      const int64_t& iteration_end_ts,
      const anantak::ObservationsVectorStoreMap& observations_map) {}
  
  // Run iteration without any observations
  virtual bool RunIteration(
      const int64_t& iteration_end_ts) {}
  
  // Start a new iteration
  //virtual bool StartIteration(const int64_t& iteration_end_ts) {}
  
  // Create new states for an iteration
  //virtual bool CreateIterationStates(const int64_t& iteration_end_ts) {}
  
  // Process the observations
  //virtual bool CreateIterationResiduals(const anantak::ObservationsVectorStoreMap& observations_map) {}
  
  // Run filtering for the iteration
  //virtual bool RunFiltering(const int64_t& iteration_end_ts) = 0;
  
  // Are the results ready to be published? e.g. did the results change? 
  virtual bool AreResultsReady() const {}
  
  // Get results in form of a message
  virtual inline const anantak::MessageType& GetResultsMessage() {}
  
  // Show results
  virtual bool Show() {}
  
  // Hide results
  virtual bool Hide() {}
  
  // Coninue running?
  virtual bool KeepRunning() {}
  
};  // Model



/* Sliding Window Iterations
 * Implements functions to help run the sliding window filter
 */
class SlidingWindowFilterIterations {
 public:
  
  struct Options {
    uint64_t longest_problem_interval;    // Problem will be built to this longest length
    uint64_t shortest_problem_interval;   // Problem will be built to this length at a minimum
    uint64_t sliding_window_interval;     // This is the length of time in which states will be solved
    uint64_t solving_problem_interval;    // Problem will be solved after every this interval
    
    Options() :
      longest_problem_interval(10000000),
      shortest_problem_interval(3000000),
      sliding_window_interval(2000000),
      solving_problem_interval(1000000)
    {Check();}
    
    Options(uint64_t lpi, uint64_t spi, uint64_t swi, uint64_t vpi) :
      longest_problem_interval(lpi),
      shortest_problem_interval(spi),
      sliding_window_interval(swi),
      solving_problem_interval(vpi)
    {Check();}
    
    Options(const SlidingWindowOptionsConfig& config):
      longest_problem_interval(config.longest_problem_interval()),
      shortest_problem_interval(config.shortest_problem_interval()),
      sliding_window_interval(config.sliding_window_interval()),
      solving_problem_interval(config.solving_problem_interval())
    {Check();}    
    
    bool Check() {
      if (longest_problem_interval < shortest_problem_interval)
        LOG(ERROR) << "longest_problem_interval < shortest_problem_interval! "
          << longest_problem_interval << " " << shortest_problem_interval;
      if (shortest_problem_interval < sliding_window_interval)
        LOG(ERROR) << "shortest_problem_interval < sliding_window_interval! "
          << shortest_problem_interval << " " <<  sliding_window_interval;
    }  // Check
    
    std::string ToString() {
      return "Longest: "+std::to_string(longest_problem_interval)+" Shortest: "+std::to_string(shortest_problem_interval)+
          " Sliding: "+std::to_string(sliding_window_interval)+" Solving: "+std::to_string(solving_problem_interval);
    }
  }; // Options
  
  SlidingWindowFilterIterations::Options options_;
  
  int64_t start_ts_;            // Algorithm begins at this ts
  int64_t data_begin_ts_;       // Data in the problem begins at this ts
  int64_t solve_begin_ts_;      // Solving of the 
  int64_t data_end_ts_;
  int64_t sliding_window_ts_;   // ts where we begin solving data
  
  bool reset_problem_;
  bool solve_problem_;
  
  SlidingWindowFilterIterations():
    options_(), start_ts_(0),
    data_begin_ts_(0), solve_begin_ts_(0), data_end_ts_(0), sliding_window_ts_(0),
    reset_problem_(false), solve_problem_(false)
    {Check();}
  
  SlidingWindowFilterIterations(const SlidingWindowFilterIterations::Options& op):
    options_(op), start_ts_(0),
    data_begin_ts_(0), solve_begin_ts_(0), data_end_ts_(0), sliding_window_ts_(0),
    reset_problem_(false), solve_problem_(false)
    {Check();}
  
  SlidingWindowFilterIterations(const SlidingWindowFilterIterations::Options& op,
                                const int64_t& start_ts):
    options_(op), start_ts_(0),
    data_begin_ts_(0), solve_begin_ts_(0), data_end_ts_(0), sliding_window_ts_(0),
    reset_problem_(false), solve_problem_(false)
    {Check(); StartFiltering(start_ts);}
  
  bool Check() {
    if (options_.solving_problem_interval >= options_.sliding_window_interval) {
      LOG(WARNING) << "solving_problem_interval >= sliding_window_interval. "
          << options_.solving_problem_interval << " " << options_.sliding_window_interval;
      LOG(WARNING) << "Usually we expect solving_problem_interval < sliding_window_interval";
      return false;
    }
    return true;
  }
  
  // Starting of the filter timestamp
  bool StartFiltering(const int64_t& start_ts) {
    start_ts_ = start_ts;
    data_begin_ts_ = start_ts;
    solve_begin_ts_ = start_ts;
    data_end_ts_ = start_ts;
    sliding_window_ts_ = start_ts;
    reset_problem_ = true;
    VLOG(1) << "Starting filtering at " << start_ts_;
  }
  
  // Regular updates to the data, with ending data timestamp provided
  bool AddData(const int64_t& data_ts) {
    
    // Check if new data end ts makes sense
    if (data_ts < data_end_ts_) {
      LOG(ERROR) << "Recieved data end ts < last end ts. Not expected. "
          << data_ts << " " << data_end_ts_;
      return false;
    }
    data_end_ts_ = data_ts;
    
    // Sliding window to solve begins before data end ts
    sliding_window_ts_ = data_end_ts_ - options_.sliding_window_interval;
    if (sliding_window_ts_ < start_ts_) sliding_window_ts_ = start_ts_;
    
    // Is it time to reset the problem?
    reset_problem_ = (data_end_ts_ >= data_begin_ts_ + options_.longest_problem_interval);
    
    if (reset_problem_) {
      // Move forward
      data_begin_ts_ = data_end_ts_ - options_.shortest_problem_interval;
      if (data_begin_ts_ < start_ts_) data_begin_ts_ = start_ts_;
      // Check solve begin ts
      if (solve_begin_ts_ < data_begin_ts_) {
        LOG(WARNING) << "Data solve ts fell before data begin. Has the problem not been solved for a while?"
            << " solve_begin_ts_ was " << data_begin_ts_ - solve_begin_ts_ << " musecs before data_begin_ts_";
        solve_begin_ts_ = data_begin_ts_;
      }
    } else {
      // Data begin ts and solve begin ts both remain at the same place
    }
    
    // Is it time to solve the problem?
    solve_problem_ = (data_end_ts_ >= solve_begin_ts_ + options_.solving_problem_interval);
    
    return true;
  }
  
  /** DataBeginTimestamp marks where the data for the problem begins. From this timestamp to
   *  SlidingWindowTimestamp the states are kept constant. */
  const int64_t& DataBeginTimestamp() const {return data_begin_ts_;}
  
  /** SlidingWindowTimestamp marks where the states are solved in the problem. */
  const int64_t& SlidingWindowTimestamp() const {return sliding_window_ts_;}
  
  /** Problem is reset when its age becomes greater than longest_problem_interval */
  bool IsItTimeToResetProblem() const {return reset_problem_;}
  
  /** Problem is solved when it has not been solved for solving_problem_interval */
  bool IsItTimeToSolveProblem() const {return solve_problem_;}
  
  // Solve problem update to data. Gets the ts of the last data point when problem is solved
  bool SolveProblem() {
    // Problem was solved, so update solve ts
    solve_begin_ts_ = data_end_ts_;
    return true;
  }
  
};  // SlidingWindowFilterIterations

/* IterationRecord - keeps track of iterations */
struct IterationRecord {
  uint64_t iteration_number;  // Iterations counter
  int64_t begin_ts;           // Beginning timestamp
  int64_t end_ts;             // Ending timestamp
  
  // Counters of the iteration
  std::map<std::string, uint32_t> iteration_counters;
  std::map<std::string, uint64_t> algorithm_counters;
  
  IterationRecord():
    iteration_number(0),
    begin_ts(0), end_ts(0)
  {}
  
  IterationRecord(const int64_t& ts):
    iteration_number(0),
    begin_ts(ts), end_ts(ts)
  {}
  
  bool Reset(const int64_t& ts) {
    iteration_number = 0;
    begin_ts = ts;
    end_ts = ts;
    return true;
  }
  
  bool Increment(const int64_t& new_ts) {
    if (new_ts <= end_ts) {
      LOG(ERROR) << "Can not increment iteration as new_ts <= end_ts "
          << new_ts << " <= " << end_ts;
      return false;
    }
    iteration_number++;
    begin_ts = end_ts;
    end_ts = new_ts;
    return true;
  }
  
  bool ResetIterationCounters() {
    for (auto &pair : iteration_counters) {
      pair.second = 0;
    }
    return true;
  }
  
  std::string IterationCountersToString() const {
    std::string str = std::to_string(iteration_number) + ", ";
    for (const auto &pair : iteration_counters) {
      str += (pair.first + " " + std::to_string(pair.second) + ", ");
    }
    return str;
  }
  
  std::string AlgorithmCountersToString() const {
    std::string str = std::to_string(iteration_number) + ", ";
    for (const auto &pair : algorithm_counters) {
      str += (pair.first + " " + std::to_string(pair.second) + ", ");
    }
    return str;
  }
  
}; // IterationRecord


}  // anantak
