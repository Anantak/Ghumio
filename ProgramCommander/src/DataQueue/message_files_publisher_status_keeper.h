/**
 *  Message Files Publisher Status Keeper
 *
 *  This derives from the Component Status Keeper and implements specific members for Message Files
 *  Publisher.
 */

#include "ComponentCommander/component_status_keeper.h"

namespace anantak {

class MessageFilesPublisherStatusKeeper : public ComponentStatusKeeper {
 public:
  /** Constructor */
  MessageFilesPublisherStatusKeeper(std::string component_name);
  
  /** Destructer */
  ~MessageFilesPublisherStatusKeeper();
  
  /** Generates a status query message, returns a unique pointer to serialized string
   *  This method could use protocol buffers or any other way of representing a message. But the
   *  ProgramCommander assumes that the message is created as a string. The string is then owned
   *  by the calling function.
   *  Calling function takes ownership of the string and is responsible for destructing the string.
   *  Making this virtual as we always want a derived class to override this method.
   */
  std::unique_ptr<std::string> GenerateStatusQueryMessage();

  /** Generates the reply for the status query message
   *  This is owned by the component that is running some task such as DataQueue, Filter etc.
   *  The status reply has some basic information such as component's up-time, cycling-rate,
   *  processing times of different parts of the component, etc. It also has the identifier of
   *  StatusQuery message it recieved, this allows the StatusKeeper to calculate the status-reply
   *  delay for the component.
   *  This method is particular to the implementation. For all implementations we provide the
   *  up-time of the component. Filters would have cycle-rates, delays of calculation pieces with
   *  other things. This method can then generate the StatusQueryReply using all the information
   *  needed to generate the reply. That information can be supplied here as a list of parameters,
   *  a serialized string or whatever.
   */
  std::unique_ptr<std::string> GenerateStatusQueryReplyMessage(
      std::unique_ptr<std::string> status_query_msg);

  /** Updates status from a status reply message
   *  This takes in a serialized representation of the message object as a string. StatusKeeper can
   *  decide to use Protocol Buffers or whatever way to repesent its objects, but the transfer is
   *  done using a simple string object. Returns success or not in update operation.
   *  This method takes ownership of the string and is responsible for destructing it.
   *  Making this virtual as we always want a derived class to override this method.
   */
  bool ProcessStatusReply(std::unique_ptr<std::string> status_reply_message);
  
  /** Generates a component status message
   *  This generates a serialized message as string that represents the status of the component
   *  being tracked. Typically Protocol Buffer library is used, but any thing can be used.
   *  Calling function takes ownership of the string and is responsible for destructing it.
   *  Making this virtual as we always want a derived class to override this method.
   */
  std::unique_ptr<std::string> GenerateStatusMessage();
  
  /** Generate a component display panel in HTML
   *  Every component displays its status in a panel on a webpage. This method generates the HTML
   *  with all scripts to recieve the HTML via socket.io and update it via jQuery. At the startup
   *  ProgramCommander asks each ComponentKeeper for this HTML and then saves it to a fixed location
   *  on the file system that is shared with the WebCommander. WebCommander periodically loads the
   *  contents of the directory to render its display page. The number of panels is conveyed by the
   *  number of HTML panels stored and each panel has div-tagged HTML for display. WebCommander owns
   *  the stylesheets, locations where panels are displayed on webpage etc.
   */
  std::unique_ptr<std::string> GenerateDisplayPanel();

};

} // namespace anantak
