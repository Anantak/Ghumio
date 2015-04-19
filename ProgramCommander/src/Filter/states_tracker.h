/**
 *  States Tracker
 *
 *  States tracker owns all states. Models create and marginalize states. 
 *
 *  StateTracker keeps a fixed-length circular vector of states for each model. Filter owns the
 *  StatesTracker. At the startup Filter asks each Model to create its states by passing it a
 *  pointer to the StatesTracker. 
 */

/** std includes */
#include <string>
#include <memory>
#include <vector>
#include <map>

/** anantak includes */
#include "Filter/state.h"

namespace anantak {

/** Data structures */

/** States for every model are stored in a circular queue. Pointers stored are to State. All model
 * specific states are derived from State, so we can store all states in a single type */
typedef anantak::CircularPointerQueue<anantak::State> StateCirPtrQueue;

/** Pointer to circular pointer queue of States to be stored in a map per model */
typedef std::unique_ptr<StateCirPtrQueue> StateCirPtrQueuePtr;

/** Map uses names of the Models as keys. */
typedef std::map<std::string, StateCirPtrQueuePtr> StateCirPtrQueuePtrMap;


} // namespace anantak

