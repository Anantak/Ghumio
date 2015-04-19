/**
 *  Observation
 *
 *  Observations are measurements from sensors. Each observation has an associated error.
 *
 *  The difference between the state and the observation is that state is to be estimated and 
 *  observation is fixed. As observations from sensors flow into the filter, they will be saved
 *  in an ObservationsTracker.
 *
 *  Models build constraints on their states using the observations. Filter builds the problem
 *  and solves it - meaning finds the values of states that minimize a residual or maximize a
 *  posteriori probability.
 *  
 **/

#pragma once

/** std includes */
#include <string>
#include <memory>
#include <map>
#include <sys/time.h>

/** anantak includes */
#include "common_config.h"
#include "Filter/circular_pointer_queue.h"

/** Protocol buffers */
#include "configurations.pb.h"

namespace anantak {

  typedef std::map<std::string, anantak::FilterConfig::ObservationType> ObservationTypeMap;
  typedef ObservationTypeMap::iterator ObservationTypeMapIterator;
  
  typedef std::unique_ptr<std::vector<anantak::MessagePtrType>> ObservationsPtrVectorPtr;
  struct ObservationsVectorStore {
    int32_t n_observations;
    ObservationsPtrVectorPtr observations;
  };
  typedef std::map<std::string, ObservationsVectorStore> ObservationsVectorStoreMap;
  typedef ObservationsVectorStoreMap::iterator ObservationsVectorStoreMapIterator;
  typedef anantak::CircularPointerQueue<ObservationsVectorStoreMap>
      ObservationsVectorStoreMapCirPtrQueue;

} // namespace anantak
