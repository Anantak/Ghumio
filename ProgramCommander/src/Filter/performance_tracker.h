/**
 *  Performance Tracker
 *
 *  Keeps a map of timers. Timer keeps a circular queue of accummulated times and number of events.
 **/

#pragma once

/** std includes */
#include <map>

/** anantak includes */
#include "Filter/time_keeper.h"

/** Google libraries */
#include <glog/logging.h>

namespace anantak {

class PerformanceTracker {
 public:
  typedef std::map<std::string, anantak::TimeKeeper> TimeKeeperMap;
  typedef TimeKeeperMap::iterator TimeKeeperMapIterator;
  
  /** Default constructor */
  PerformanceTracker() {
    LOG(INFO) << "Performance Tracker created";
  }
  
  /** Destructor - all should self destruct */
  virtual ~PerformanceTracker() {
    LOG(INFO) << "Destructing Performance Tracker";
  }
  
  /** Add a new timer */
  bool AddTimer(std::string name, int32_t size=100) {
    TimeKeeperMapIterator i_map = time_keepers_map_.find(name);
    if (i_map != time_keepers_map_.end()) {
      LOG(ERROR) << name << " timer already exists. can not add again.";
      return false;
    }
    time_keepers_map_[name] = TimeKeeper(name, size);
    return true;
  }
  
  /** Return a reference to the timer object in the map
   *  Doing this is a large source of debate. If the object being returned by reference is deleted
   *  say goes out of scope, and the calling function stores it in a variable, core_dump happens.
   *  The only way this function is supposed to be used is to easily call methods of TimeKeeper by
   *  its name. E.g. performance_tracker("NameOfKeeper").Start() We never intend to store the
   *  PerformanceKeeper object. **/
  anantak::TimeKeeper& operator()(std::string name) {
    TimeKeeperMapIterator i_map = time_keepers_map_.find(name);
    if (i_map == time_keepers_map_.end()) {
      LOG(INFO) << name << " timer does not exist. Adding it.";
      time_keepers_map_[name] = TimeKeeper(name, 100);
    }
    return time_keepers_map_[name];
  }
  
 private:
  std::map<std::string, TimeKeeper> time_keepers_map_;
  
};

}
