/**
 *  Camera intrinsics calibrator filter
 *  
 */


/** Google libraries */
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <google/protobuf/message.h>

// Anantak includes
#include "common_config.h"
#include "Utilities/common_functions.h"
#include "Filter/sliding_window_filter.h"
#include "Models/camera_calibration_filter.h"

/** Command line flags */
DEFINE_string(name, "", "Name of the component");
DEFINE_string(setup_file, "", "Programs setup file");

static std::string target_calibrator_none_hist_command_ = "";
static std::string target_calibrator_none_live_command_ = "";
static std::string target_calibrator_camera00_hist_command_ = "";
static std::string target_calibrator_camera00_live_command_ = "";
static std::string target_calibrator_camera01_hist_command_ = "";
static std::string target_calibrator_camera01_live_command_ = "";

static std::unique_ptr<anantak::Model> model_ = nullptr;
static std::unique_ptr<anantak::SlidingWindowFilter> filter_ = nullptr;
static anantak::FilterConfig::Model model_config;

// Operating variables
static int64_t results_publish_interval_ = 1000000;

// Set operating variables from model_config
bool SetOperatingVariables() {
  results_publish_interval_ = 1000000 / model_config.results_frequency();
  return true;
}

// Return true if this was a model command, false otherwise
//  set is_live = true if this is a live model else false
//  set model_name = name of model used by create model sproc
bool ProcessModelCommand(const std::string& cmd_str, std::string& model_name, bool& is_live) {
  if (cmd_str == target_calibrator_none_hist_command_ ||
      cmd_str == target_calibrator_none_live_command_) {
    model_name = "";
    is_live = true;
    return true;
  }
  else if (cmd_str == target_calibrator_camera00_live_command_) {
    model_name = "Camera00.Intrinsics";
    is_live = true;
    return true;
  }
  else if (cmd_str == target_calibrator_camera00_hist_command_) {
    model_name = "Camera00.Intrinsics";
    is_live = false;
    return true;
  }
  else if (cmd_str == target_calibrator_camera01_live_command_) {
    model_name = "Camera01.Intrinsics";
    is_live = true;
    return true;
  }
  else if (cmd_str == target_calibrator_camera01_hist_command_) {
    model_name = "Camera01.Intrinsics";
    is_live = false;
    return true;
  }
  return false; // if this was not a command, return false;
}

bool CreateModel(const std::string& model_name, const int64_t& start_ts) {
  
  if (model_) {
    LOG(ERROR) << "A model already exists. Can not create model. Skip";
    return false;
  }
  
  if (model_name == "Camera00.Intrinsics" || model_name == "Camera01.Intrinsics") {
    
    // Get starting calibrations from a calibration queue
    const anantak::SensorMsg* calib_msg = nullptr;
    if (model_config.has_results_queue()) {
      VLOG(1) << "Getting starting data from " << model_config.results_queue();
      std::string sensor_name = "";
      if (model_config.has_results_sensor()) sensor_name = model_config.results_sensor();
      bool got_calib = filter_->GetLatestDataFromDataQueue(model_config.results_queue(),
          results_publish_interval_, sensor_name);
      // Check if the observations queues have the expected message
      const anantak::ObservationsVectorStoreMap& obs_store_map =
          filter_->GetLatestObservationsMap();
      const std::string msg_type = "CameraIntrinsics";
      auto i_obs_store = obs_store_map.find(msg_type);
      if (i_obs_store == obs_store_map.end()) {
        LOG(ERROR) << "Could not find " << msg_type << "type in the observations.";
      } else {
        // Are there any messages?
        if (i_obs_store->second.n_observations < 1) {
          LOG(ERROR) << "Did not find any reading of type " << msg_type << " in observations.";
        } else {
          // Point the pointer to the message
          anantak::MessageType* calib_msg_ptr = i_obs_store->second.observations->at(0).get();
          // Cast the message to sensor message
          anantak::SensorMsg* sensor_msg = nullptr;
          if (!(sensor_msg = dynamic_cast<anantak::SensorMsg*>(calib_msg_ptr))) {
            LOG(ERROR) << "Could not cast observation message to SensorMsg.";
          } else {
            // Check if there is a CameraIntrinsicsStateMessage present
            if (!sensor_msg->has_camera_intrinsics_state_msg()) {
              LOG(ERROR) << "Calibration message does not have a camera intrinsics message";
            } else {
              //calib_msg = &sensor_msg->camera_intrinsics_state_msg();
              calib_msg = sensor_msg;
            }
          }
        }
      }
    }
    
    // Create the Beacon filter options using the configuration file provided
    anantak::CameraIntrinsicsFilter::Options camera_intrincs_filter_options(model_config.config_file());
    // If calibration message was recieved, set the starting calibration
    if (calib_msg) {
      VLOG(1) << "Setting starting calibration for model using calibration queue reply";
      camera_intrincs_filter_options.SetStartingCameraIntrinsics(*calib_msg);
    }
    std::unique_ptr<anantak::Model> model_ptr(new anantak::CameraIntrinsicsFilter(camera_intrincs_filter_options, start_ts));
    model_ = std::move(model_ptr);
  }
  
  return true;
}

bool DestructModel() {
  
  if (!model_) {
    LOG(ERROR) << "Model does not exist. There is nothing to destruct";
    return false;
  }
  
  model_.reset();
  
  return true;
}

int main(int argc, char** argv) {
  
  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);
  // Parse the commandline flags 
  google::ParseCommandLineFlags(&argc, &argv, true);
  // Verify the version of protobuf library 
  GOOGLE_PROTOBUF_VERIFY_VERSION;
  // Setting the glog output to stderr by default
  FLAGS_logtostderr = 1;
  
  std::string programs_setup_filename = FLAGS_setup_file;
  std::string component_name = FLAGS_name;
  VLOG(1) << "Setup file = " << programs_setup_filename;
  VLOG(1) << "Component name = " << component_name;

  // Directories and filenames
  std::string project_root_dir = anantak::GetProjectSourceDirectory() + "/";
  std::string plots_dir = project_root_dir + "src/Models/Plots/";
  
  // Set command strings
  target_calibrator_none_hist_command_ = "COMMAND "+component_name+" model None Hist";
  target_calibrator_none_live_command_ = "COMMAND "+component_name+" model None Live";
  target_calibrator_camera00_hist_command_ = "COMMAND "+component_name+" model Camera00 Hist";
  target_calibrator_camera00_live_command_ = "COMMAND "+component_name+" model Camera00 Live";
  target_calibrator_camera01_hist_command_ = "COMMAND "+component_name+" model Camera01 Hist";
  target_calibrator_camera01_live_command_ = "COMMAND "+component_name+" model Camera01 Live";
  
  // Build the sliding window filter component
  std::unique_ptr<anantak::SlidingWindowFilter> filter_ptr(
      new anantak::SlidingWindowFilter(programs_setup_filename, component_name));
  filter_ = std::move(filter_ptr);
  
  // Check if filter was created alright
  if (!filter_->is_initiated()) {
    LOG(ERROR) << "An error occurred, could not initiate filter_-> Exiting.";
    return -1;
  }
  
  // There are multiple models that are run by the filter. These are recorded in the config file
  // of the filter. Each model has its own config file for model-specific parameters.
  // This filter can run any model that is chosen in live or historical mode. The process is:
  //  At start initiate filter and wait for knowledge of the model to run
  //  When the model to run is known, is it to be run live or historical?
  //  If live, start the model live
  //  If historical, wait for first timestamp of data. When have it set the offsets, create filter.
  //  In each loop check if a model change has been requested.
  //    If a model change has been requested, destruct this model, go back to filter init step
  enum RunStateType {kWaitingForModel, kCreateModel, kWaitingForHistoricalData, kRunningModel};
  RunStateType run_state = kWaitingForModel;
  
  // Create looper that helps keep track of sleeping in loops
  anantak::Looper looper(filter_->loop_frequency());
  const int64_t default_iteration_interval = 1000000 / int64_t(filter_->loop_frequency());
  VLOG(1) << "Starting loop frequency and iteration interval = "
      << filter_->loop_frequency() << " " << default_iteration_interval;
  
  std::string model_name_ = "";
  bool is_live_ = false;
  int64_t historical_time_offset_ = 0;
  int64_t last_results_publish_ts_ = 0;
  
  // Start looping - request data, create states, get data, process it, repeat.
  while (!filter_->exit_loop()) {
    
    // Get current wall time - this is the iteration end timestamp
    const int64_t iteration_end_ts = filter_->wall_time();
    
    // If model is there, use the model's iteration interval
    int64_t iteration_interval;
    if (model_) {
      if(iteration_interval != model_->IterationInterval()) {
        iteration_interval = model_->IterationInterval();
        looper.SetInterval(iteration_interval, iteration_end_ts);
      }
    // If model is not there use default iteration interval
    } else {
      if(iteration_interval != default_iteration_interval) {
        iteration_interval = default_iteration_interval;
        looper.SetInterval(iteration_interval, iteration_end_ts);
      }
    }
    
    // Iteration end ts
    const int64_t next_iteration_begin_ts = iteration_end_ts + iteration_interval;
    //VLOG(1) << "Iteration end ts = " << iteration_end_ts;
    
    // Request new data ending at iteration end timestamp
    filter_->RequestNewData(iteration_end_ts);
    
    bool got_a_reply = filter_->WaitForData(next_iteration_begin_ts);
    bool got_new_obs = (got_a_reply && (filter_->num_observations_received()>0));
    bool just_created_model = false;
    
    if (run_state == kWaitingForModel) {
      VLOG(1) << "Waiting for model";
      
      // Check if a model command comes in, if so transition to create model
      if (filter_->got_command()) {
        if (ProcessModelCommand(filter_->command_message(), model_name_, is_live_)) {
          VLOG(1) << "Got model command name, liveness '" << model_name_ << "' " << is_live_;
          if (model_name_ != "") {
            if (filter_->GetModelConfiguration(model_name_, &model_config)) {
              SetOperatingVariables();
              run_state = kCreateModel;              
            }
          }
        }
      }
    }
    
    if (run_state == kCreateModel) {
      
      // If live model, build the model and transition to running model
      if (is_live_) {
        VLOG(1) << "Creating model";
        if (!CreateModel(model_name_, iteration_end_ts-iteration_interval)) {
          LOG(ERROR) << "Could not create model " << model_name_;
          run_state = kWaitingForModel;   // back to create model
        } else {
          just_created_model = true;
          // Created model successfully, transition to running model
          run_state = kRunningModel;
        }
      }
      // If historical model, transition to waiting for historical data
      else { 
        run_state = kWaitingForHistoricalData;
      }
    }
    
    if (run_state == kWaitingForHistoricalData) {
      VLOG(1) << "Waiting for historical data";
      
      // Check if any data was received
      // If data is there, calculate historical data offset, initiate model, transition to running
      if (got_new_obs) {
        VLOG(1) << "Got an observation, calculating historical offset";
        int64_t hist_ts_marker = ((filter_->max_timestamp_received() / iteration_interval)+1) * iteration_interval;
        historical_time_offset_ = hist_ts_marker - iteration_end_ts;
        VLOG(1) << "Historical time offset = " << historical_time_offset_;
        
        VLOG(1) << "Creating model";
        if (!CreateModel(model_name_, hist_ts_marker-iteration_interval)) {
          LOG(ERROR) << "Could not create model " << model_name_;
          run_state = kWaitingForModel;   // back to create model
        } else {
          just_created_model = true;
          // Created model successfully, transition to running model
          run_state = kRunningModel;
        }
        
      }
      
    }
    
    if (run_state == kRunningModel) {
      
      // Modified iteration end ts depending on live/historical modes
      const int64_t mod_iteration_end_ts = iteration_end_ts + historical_time_offset_;
      
      // Run iteration in the model
      if (got_a_reply) {
        if (!model_->RunIteration(mod_iteration_end_ts, filter_->GetLatestObservationsMap())) {
          LOG(ERROR) << "Could not run iteration for model " << model_name_;
        }
      } else {
        if (!model_->RunIteration(mod_iteration_end_ts)) {
          LOG(ERROR) << "Could not run iteration for model " << model_name_;
        }
      }
      
      
      //// Start new iteration
      //if (!model_->StartIteration(mod_iteration_end_ts)) {
      //  LOG(ERROR) << "Could not start iteration for model " << model_name_;
      //}
      //VLOG(1) << "Started iteration";
      //
      //// Create new states
      //if (!model_->CreateIterationStates(mod_iteration_end_ts)) {
      //  LOG(ERROR) << "Could not create iteration states for model " << model_name_;
      //}
      //VLOG(1) << "Created iteration states";
      //
      //// Process the observations if something was received
      //if (got_a_reply) {
      //  if (!model_->CreateIterationResiduals(filter_->GetLatestObservationsMap())) {
      //    LOG(ERROR) << "Could not create iteration residuals for model " << model_name_;
      //  }
      //  VLOG(1) << "Created iteration residuals";
      //}
      //
      //// Run filtering for the iteration
      //if (!model_->RunFiltering(mod_iteration_end_ts)) {
      //  LOG(ERROR) << "Could not run filtering for model " << model_name_;        
      //}
      //VLOG(1) << "Ran filtering";
      
      // Publish results if it is time and results changed
      if (iteration_end_ts - last_results_publish_ts_ > results_publish_interval_) {
        if (model_->AreResultsReady()) {
          filter_->PublishMessage(model_->GetResultsMessage(), model_config.results_subject());
          VLOG(1) << "Published results message on " << model_config.results_subject();
          last_results_publish_ts_ = iteration_end_ts;
        } else {
          VLOG(3) << "No change in model result. Not publishing.";
        }
      }
      
      // Is the model finished working?
      bool model_is_finished = false;
      if (model_->Finished()) {
        LOG(INFO) << "Filter is finished with operations. Exiting.";
        model_is_finished = true;
      }
      
      // check if there is a model close command. If so, close model, transition to waiting for model
      // check if there is a model change command. If so, close model, transition to create model
      if ((filter_->got_command() && !just_created_model) || model_is_finished) {
        // Reset operating variables
        std::string model_name_ = "";
        bool is_live_ = false;
        int64_t historical_time_offset_ = 0;
        int64_t last_results_publish_ts_ = 0;
        // Process command
        if (ProcessModelCommand(filter_->command_message(), model_name_, is_live_) || model_is_finished) {
          // Close the model
          if (!DestructModel()) {
            LOG(ERROR) << "Could not destruct the model. Exit.";
            return -1;
          }
          if (model_name_ == "") {
            // Transition to waiting for model
            run_state = kWaitingForModel;            
          } else {
            // Transition to create another model
            if (filter_->GetModelConfiguration(model_name_, &model_config)) {
              SetOperatingVariables();
              run_state = kCreateModel;              
            }
          }
        }
      } // change command
      
    }
    
    // Sleep if needed
    if (!filter_->exit_loop()) looper.Sleep();
    filter_->set_max_loop_frequency(looper.max_frequency());
  }
  
  return 0;
}
