/**
 *  Component Status Keeper Factory
 *  Header file
 */

#pragma once

// includes from std
#include <string>
#include <memory>

// include for ComponentStatusKeeper
#include <ComponentCommander/component_status_keeper.h>

namespace anantak {
  
/**
 *  Component Status Keeper Factory
 *  This can supply an instance of the ComponentStatusKeeper derived class. Main reason for this is
 *  that it is aware of all the derived classes of ComponentStatusKeeper that are used by different
 *  components. For example a sensor returns a SensorStatus message but a filter returns a different
 *  type of message. The ComponentCommander instantiates component status keeper using the factory
 *  when it does not know about the derived instances itself.
 */
class ComponentStatusKeeperFactory {
 public:
  /** Constructs ComponentStatusKeeperFactory */
  ComponentStatusKeeperFactory();
  
  /** Deletes the ComponentStatusKeeperFactory */
  virtual ~ComponentStatusKeeperFactory();
  
  /** Creates and returns ComponentStatusKeeper (derived) object */
  static std::unique_ptr<anantak::ComponentStatusKeeper> CreateComponentStatusKeeper(
      std::string status_keeper_type, std::string status_keeper_name);
};

} // namespace anantak
