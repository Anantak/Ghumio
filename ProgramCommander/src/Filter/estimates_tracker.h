/**
 *  Estimates Tracker
 *
 *  Estimates tracker owns all states. Models allocate memory in the EstimatesTracker.
 *
 *  This keeps a circular queue of maps of vectors.
 *    Circular queue has number of elements equal to maximum history of estimates to be kept
 *    Map has entries equal to number of Models with each key equal to model name
 *    Vector has (aligned) array of states. More on this later.
 *
 */

#pragma once

/** std includes */
#include <string>
#include <memory>
#include <vector>
#include <map>

/** anantak includes */

/** Eigen includes */
#include <Eigen/Eigen>

namespace anantak {

/** Data structures */

/** Estimate definition */
struct EstimatesType {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Eigen matrix component for estimates, errors, state-ids
  Eigen::VectorXd estimates;    // this will be allocated on the heap
  std::vector<int64_t> state_ids;
};
typedef std::unique_ptr<EstimatesType> EstimatesPtr;

/** Map uses names of the Models as keys. */
typedef std::map<std::string, EstimatesPtr> EstimatesPtrMap;
typedef std::unique_ptr<EstimatesPtrMap> EstimatesPtrMapPtr;
typedef EstimatesPtrMap::iterator EstimatesPtrMapIterator;

/** Circular pointer queue of EstimatesPtrMap */
typedef anantak::CircularPointerQueue<EstimatesPtrMap> EstimatesPtrMapCirPtrQueue;

/** Memory allocation
 *  Memory will be allocated to the estimates tracker by the models. Filter will ask each model
 *  to allocate memory for each map in the EstimatesType. Model should know the maximum number of
 *  states it expects to calculate per iteration. This would be based on max_sliding_window_
 *  interval of the filter and settings of the filter. E.g. A kinematic model set to 100Hz with
 *  max sliding window of 2sec will have a maximum of 200 states per iteration. A mapping model
 *  detecting tags could have say 200 tags set to be mapped in one area. etc.
 **/


} // namespace anantak

