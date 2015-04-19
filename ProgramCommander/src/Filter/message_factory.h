/**
 *  Message Factory
 *
 *  Message factory returns empty instances of message objects. It is used when the type of
 *  message to be created is not known at the compile time. This is used e.g. by the filter when
 *  it needs to instantiate a message in the memory.
 *
 *  Downside of using a factory like this is that it can only instantiate objects it knows about.
 *  Everytime a new object is created, one needs to add the knowledge of the object to the factory.
 **/

#pragma once

/** std includes */
#include <string>
#include <memory>

/** anantak includes */
#include "common_config.h"

namespace anantak {

class MessageFactory {
 public:
  /** Constructs MessageFactory */
  MessageFactory();
  
  /** Deletes the MessageFactory */
  virtual ~MessageFactory();
  
  /** Creates and returns Messages (derived) object */
  static anantak::MessagePtrType CreateMessage(std::string message_type);
  
};


} // namespace anantak
