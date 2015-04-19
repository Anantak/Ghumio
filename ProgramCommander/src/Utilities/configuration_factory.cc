/**
 *  Configuration Factory
 *
 */

/** Includes for Google Logging library */
#include <glog/logging.h>

/** Include protocol buffers */
#include "configurations.pb.h"

/** Anantak includes */
#include "Utilities/configuration_factory.h"

namespace anantak {
  
/**
 *  Configuration Factory
 */
 
/** Constructs ConfigurationFactory */
ConfigurationFactory::ConfigurationFactory() {
  LOG(INFO) << "Created ConfigurationFactory";
};

/** Deletes the ComponentStatusKeeperFactory */
ConfigurationFactory::~ConfigurationFactory() {
  LOG(INFO) << "Destructed ConfigurationFactory";
};

/** Creates and returns ComponentStatusKeeper (derived) object */
std::unique_ptr<anantak::ConfigurationType> ConfigurationFactory::CreateConfiguration(
    std::string configuration_type) {
  std::unique_ptr<anantak::ConfigurationType> config;
  
  if (configuration_type == "DataQueueConfig") {
    std::unique_ptr<anantak::ConfigurationType> tmp(new anantak::DataQueueConfig());
    VLOG(1) << "Created DataQueueConfig object";
    config = std::move(tmp);
  } else {
    LOG(ERROR) << "Dont know how to create " << configuration_type;
  }
  
  return config;
}
  
}


/**

std::unique_ptr<anantak::ConfigurationType> ReadConfiguration(std::string config_filename,
    std::string config_type) {
  anantak::ConfigurationFactory config_factory;
  std::unique_ptr<anantak::ConfigurationType> config
      = config_factory.CreateConfiguration(config_type);
  // 'Fillup' the 'empty' config object with data read from config_filename
  LOG(INFO) << "Reading the file " << config_filename;
  int in_file_descriptor = open(config_filename.c_str(), O_RDONLY);
  if (in_file_descriptor < 0) {
    LOG(ERROR) << " Error opening the file: " << strerror(errno);
    return false;
  }
  // Parse the file
  google::protobuf::io::FileInputStream file_input_stream(in_file_descriptor);
  google::protobuf::TextFormat::Parse(&file_input_stream, &(*config));
  file_input_stream.Close();
  // convert to string to display
  std::string in_config_str;
  google::protobuf::TextFormat::PrintToString(*config, &in_config_str);
  VLOG(2) << "in_config = \n" << in_config_str;
  // Done
  return config;
}

template<typename Derived, typename Base>
std::unique_ptr<Derived> static_unique_ptr_cast(std::unique_ptr<Base> p) {
  Derived* d = static_cast<Derived*>(p.release());
  return std::unique_ptr<Derived>(d);
}


  //std::string config_type = "DataQueueConfig";
  //std::unique_ptr<anantak::ConfigurationType> unusable_config = 
  //    anantak::ReadConfiguration(config_filename, config_type);
  // In order to get information out of ConfigurationType, we will need to get DataQueueConfig.
  // Here we have to cast downwards. As we are very sure that the derived class is DataQueueConfig.
  // And inheritance is not virtual. So it should be OK to cast the pointer using static_cast. */
  //std::unique_ptr<anantak::DataQueueConfig> config =
  //    static_unique_ptr_cast<anantak::DataQueueConfig, anantak::ConfigurationType>(
  //      std::move(unusable_config)
  //    );
  //VLOG(1) << "Number of sensors: " << config->sensor_name_size();
  


*/

