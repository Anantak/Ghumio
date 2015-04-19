/**
 *  Component Status Keeper Factory
 *
 */

// includes
#include "ComponentCommander/component_status_keeper_factory.h"
#include "DataQueue/message_files_publisher_status_keeper.h"

/** Includes for Google Logging library */
#include <glog/logging.h>

namespace anantak {
  
/**
 *  Component Status Keeper Factory
 *  This can supply an instance of the ComponentStatusKeeper derived class. Main reason for this is
 *  that it is aware of all the derived classes of ComponentStatusKeeper that are used by different
 *  components. For example a sensor returns a SensorStatus message but a filter returns a different
 *  type of message. The ComponentCommander instantiates component status keeper using the factory
 *  when it does not know about the derived instances itself.
 */

 
/** Constructs ComponentStatusKeeperFactory */
ComponentStatusKeeperFactory::ComponentStatusKeeperFactory() {
  LOG(INFO) << "Created ComponentStatusKeeperFactory";
};

/** Deletes the ComponentStatusKeeperFactory */
ComponentStatusKeeperFactory::~ComponentStatusKeeperFactory() {};

/** Creates and returns ComponentStatusKeeper (derived) object */
std::unique_ptr<anantak::ComponentStatusKeeper> ComponentStatusKeeperFactory::CreateComponentStatusKeeper(
    std::string status_keeper_type, std::string status_keeper_name) {
  // create an empty pointer
  std::unique_ptr<anantak::ComponentStatusKeeper> status_keeper_ptr;
  
  // Create a generic ComponentStatusKeeper object
  if (status_keeper_type == "ComponentStatus") {
    std::unique_ptr<anantak::ComponentStatusKeeper> tmp_ptr(new ComponentStatusKeeper(status_keeper_name));
    status_keeper_ptr = std::move(tmp_ptr);
  }
  // Create a DataPublisherStatusKeeper object that derives from ComponentStatusKeeper
  else if (status_keeper_type == "MessageFilesPublisherStatus") {
    std::unique_ptr<anantak::MessageFilesPublisherStatusKeeper>
        tmp_ptr(new MessageFilesPublisherStatusKeeper(status_keeper_name));
    status_keeper_ptr = std::move(tmp_ptr);
  } else {
    LOG(ERROR) << "Dont know how to create " << status_keeper_type;
  }
  
  return status_keeper_ptr;
}
  

}