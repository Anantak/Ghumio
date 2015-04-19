/**
 *  Model Factory
 *
 *  Models are of different types. E.g.
 *    Ackerman.Kinematics - [linear_velocity, angular_velocity]
 *    StereoCameraCalibration.Constant - constant Stereo camera calibration parameters
 *    StereoCameraCalibration.Variable - above, but with parameters calibrated from data
 *    IMUCameraPose.Constant/Variable - 6d pose between IMU and camera - constant or variable
 *    etc.
 *
 *  This class knows what different models exist and returns an instance of the model. Each model
 *  implements the methods of the base abstract Model class.
 *
 *  Downside of a factory is that everytime a new model is created it needs to be declared here.
 *  Upside is that this can serve as a repository of all models.
 **/

#pragma once

/** anantak includes */
#include "Filter/model.h"
#include "Filter/stereo_camera_calibration_constant_model.h"

/** Includes for Google Logging library */
#include <glog/logging.h>

namespace anantak {

class ModelFactory {
 public:  
  /** Empty constructor */
  ModelFactory() {
    LOG(INFO) << "Creating ModelFactory";
  }
  
  /** Destructor - all should just self destruct */
  virtual ~ModelFactory() {
    LOG(INFO) << "Destructing ModelFactory";
  }
  
  static anantak::ModelPtr CreateModel(std::string model_name, std::string model_type,
      std::string config_filename) {
    
    anantak::ModelPtr model_ptr; // empty pointer
    
    if (model_type == "StereoCameraCalibration.Constant") {
      anantak::ModelPtr ptr(new anantak::StereoCameraCalibrationConstantModel(
                            model_name, model_type, config_filename));
      model_ptr = std::move(ptr);
    }
    
    else {
      LOG(ERROR) << "ModelFactory does not know how to create this model. Returning empty pointer";
    }
    
    return model_ptr;
  }
  
};

} // namespace anantak
