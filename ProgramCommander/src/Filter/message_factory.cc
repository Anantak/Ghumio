/**
 *  Message Factory implementation
 **/

/** main header include */
#include "Filter/message_factory.h"

/** Protocol buffers */
#include "sensor_messages.pb.h"

/** Includes for Google Logging library */
#include <glog/logging.h>

namespace anantak {

/** Constructs ComponentStatusKeeperFactory */
MessageFactory::MessageFactory() {
  LOG(INFO) << "Created MessageFactory";
};

/** Deletes the ComponentStatusKeeperFactory */
MessageFactory::~MessageFactory() {
  LOG(INFO) << "Destructed MessageFactory";
};

/** Creates and returns Message (derived) object */
anantak::MessagePtrType MessageFactory::CreateMessage(std::string message_type) {
  // create an empty pointer
  anantak::MessagePtrType message_ptr;
  
  // Create a MachineInterfaceMsg
  if (message_type == "MachineInterface") {
    anantak::MessagePtrType tmp_ptr(new anantak::MachineInterfaceMsg());
    message_ptr = std::move(tmp_ptr);
  }
  // Create a MachineInterfaceMsg
  else if (message_type == "ImuQuatAccel") {
    anantak::MessagePtrType tmp_ptr(new anantak::ImuMsg());
    message_ptr = std::move(tmp_ptr);
  } else {
    LOG(ERROR) << "Dont know how to create " << message_type;
  }
  
  return message_ptr;
}


} // namespace anantak
