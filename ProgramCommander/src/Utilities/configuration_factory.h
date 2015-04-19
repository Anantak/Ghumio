/**
 *  Configuration Factory
 *  Header file
 */

#pragma once

// includes from std
#include <string>
#include <memory>

#include <common_config.h>

namespace anantak {
  
/**
 *  Configuration Factory
 *  There are many configuration formats and files. This class knows about all configs. Other
 *  classes use this to create configuration.
 */
class ConfigurationFactory {
 public:
  /** Constructs ConfigurationFactory */
  ConfigurationFactory();
  
  /** Deletes the ConfigurationFactory */
  virtual ~ConfigurationFactory();
  
  /** Creates and returns ComponentStatusKeeper (derived) object */
  static std::unique_ptr<anantak::ConfigurationType> CreateConfiguration(
      std::string configuration_type);
};

} // namespace anantak
