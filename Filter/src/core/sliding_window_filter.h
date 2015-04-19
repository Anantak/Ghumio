/**
 *  SlidingWindowFilter
 *
 *  This implements a sliding window filter. It reads data in real time from multiple data queues.
 *  It owns a bunch of models that help it update its states and create constraints. Readings are
 *  data that are read from the data queues (also state queues). Readings are used by models to
 *  create constraints. Models with Readings help create a linear system of equations as Ax+b=0.
 *  Filter uses a solver to solve this system. It saves the state calculations for future
 *  marginalization and sends the latest states to its state queue.
 *
 */

#pragma once

namespace anantak {

class SlidingWindowFilter {
 public:
  /**
   *  SlidingWindowFilter constructor takes in a configuration filename.
   *  Config file is a human-readable protobuf of the type SlidingWindowFilterSettings
   */
  SlidingWindowFilter(const std::string& config_filename);
  
  /**
   *  Clean up
   */
  virtual ~SlidingWindowFilter();
  
 private:
  anantak::SlidingWindowFilterSettings filter_settings_;   /**< Protobuf settings object */
  
}
  
}   // namespace
