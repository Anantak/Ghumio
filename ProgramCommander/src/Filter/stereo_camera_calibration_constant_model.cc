/**
 *  Model implementation
 *
 **/

/** main header include */
#include "Filter/stereo_camera_calibration_constant_model.h"

/** Google Logging library */
#include <glog/logging.h>

namespace anantak {

/** Model - main constructor */

/** Model - destructor, all should be self destructing */

/** Initiator - called before model can be used */

/** Allocate memory for the states in the filter's states_tracker_ */
bool StereoCameraCalibrationConstantModel::AllocateMemoryForStates() {
  //states_tracker_
}

/** Allocate memory for the estimates in the filter's estimates_tracker_ */
bool StereoCameraCalibrationConstantModel::AllocateMemoryForEstimates() {}

bool StereoCameraCalibrationConstantModel::CreateStates() {}
bool StereoCameraCalibrationConstantModel::MarkStates() {}
bool StereoCameraCalibrationConstantModel::InitiateEstimates() {}
bool StereoCameraCalibrationConstantModel::ProcessEstimates() {}
bool StereoCameraCalibrationConstantModel::GetResultsAsMessage() {}


} // namespace anantak
