/**
 *  Message Files Publisher Status Keeper implementation
 */

/** Header file */
#include "DataQueue/message_files_publisher_status_keeper.h"
#include "Utilities/common_functions.h"

/** CTemplate include */
#include <ctemplate/template.h>

namespace anantak {

MessageFilesPublisherStatusKeeper::MessageFilesPublisherStatusKeeper(std::string component_name)
    : ComponentStatusKeeper(component_name) {
  VLOG(3) << "Created MessageFilesPublisherStatusKeeper with name " << component_name;
  // overriding only specific values over the base class settings
  display_template_filename_ = "src/DataQueue/message_files_publisher_status.tpl";
  if (LoadDisplayTemplate()) {
    ctemplate::StringToTemplateCache("data_publisher_panel_tpl",
        display_template_.c_str(), ctemplate::DO_NOT_STRIP);
  }
}

MessageFilesPublisherStatusKeeper::~MessageFilesPublisherStatusKeeper() {
  VLOG(1) << "Destructing the MessageFilesPublisherStatusKeeper";
}

/** Generate a Status Query message
 *  Returns a unique pointer to serialized string 
 */
std::unique_ptr<std::string> MessageFilesPublisherStatusKeeper::GenerateStatusQueryMessage() {
  // generate the query message, serialize and return
  last_status_query_time_ = get_wall_time_microsec();
  // ZMQ message subject
  std::unique_ptr<std::string> ptr(new std::string(component_name_));
  *ptr += " ";
  // Generate a StatusQuery message
  status_query_msg_.set_query_time(last_status_query_time_);
  std::string msg_str;
  ::google::protobuf::TextFormat::PrintToString(status_query_msg_, &msg_str);
  *ptr += msg_str;
  VLOG(1) << component_name_ << ": generating query message = " << *ptr;
  return ptr;
}

/** Generate Status Query Reply message
 *  Different derived implementations will add additional parameters to this method depending on
 *  what information they want to keep for their status.
 *  Every component instantiates a ComponentStatusKeeper object. Everytime it gets a status query
 *  message, the component passes the message over to this method along with all new information
 *  it needs to add to the status. This method generates the reply that the component then sends
 *  via its publisher according to its priorities.
 */
std::unique_ptr<std::string> MessageFilesPublisherStatusKeeper::GenerateStatusQueryReplyMessage(
    std::unique_ptr<std::string> status_query_msg) {
  // Parse the query message
  std::string query_str = (*status_query_msg).substr(component_name_.length()+1, (*status_query_msg).length());
  //VLOG(2) << "  query_str " << query_str;
  ::google::protobuf::TextFormat::ParseFromString(query_str, &status_query_msg_);
  int64_t query_time = status_query_msg_.query_time();
  int64_t current_time = get_wall_time_microsec();
  status_reply_msg_.set_query_time(query_time);
  status_reply_msg_.set_reply_time(current_time);
  status_reply_msg_.set_query_reply_delay(current_time - query_time);
  status_reply_msg_.set_up_time(current_time - start_time_);
  status_reply_msg_.set_process_id(this_process_id_);
  if (component_status_function_) {         // Testing if status function exists
    status_reply_msg_.set_status_str(*component_status_function_());  // unique_ptr<string> comes
    // unique_ptr<string> is destructed here
  }
  std::string msg_str;
  ::google::protobuf::TextFormat::PrintToString(status_reply_msg_, &msg_str);  
  // Create a reply mesage and return.
  std::unique_ptr<std::string> ptr(new std::string("StatusReply "));
  *ptr += msg_str;
  VLOG(2) << component_name_ << ": generating query reply message = " << *ptr;
  return ptr;  
}

/** Updates status from a status reply message
 *  This takes in a serialized representation of the message object as a string. StatusKeeper can
 *  decide to use Protocol Buffers or whatever way to repesent its objects, but the transfer is
 *  done using a simple string object. Returns success or not in update operation.
 *  This method takes ownership of the string and is responsible for destructing it.
 *  Making this virtual as we always want a derived class to override this method.
 */
bool MessageFilesPublisherStatusKeeper::ProcessStatusReply(
    std::unique_ptr<std::string> status_reply_message) {
  last_status_reply_time_ = get_wall_time_microsec();
  component_state_ = kPendingSendQuery;
  manual_start_requested_ = false;
  VLOG(2) << component_name_ << ": Got status reply = " << *status_reply_message;
  // Generate a status reply
  std::string reply_str = (*status_reply_message).substr(12, (*status_reply_message).length());
  VLOG(2) << "  reply_str " << reply_str;
  ::google::protobuf::TextFormat::ParseFromString(reply_str, &status_reply_msg_);
  up_time_ = status_reply_msg_.up_time();
  if (status_reply_msg_.has_status_str()) status_reply_str_ = status_reply_msg_.status_str();
      else status_reply_str_ = "";
  if (status_reply_msg_.has_process_id()) component_process_id_ = status_reply_msg_.process_id();
  // msg string is destructed here
}

/** Generates a component status message
 *  This generates a serialized message as string that represents the status of the component
 *  being tracked. Typically Protocol Buffer library is used, but any thing can be used.
 *  Calling function takes ownership of the string and is responsible for destructing it.
 *  Making this virtual as we always want a derived class to override this method.
 */
std::unique_ptr<std::string> MessageFilesPublisherStatusKeeper::GenerateStatusMessage() {
  // Initiate status message
  std::unique_ptr<std::string> msg_ptr(new std::string("ComponentStatus "));
  *msg_ptr += ("{\"name\": \""+component_name_+"\", \"status\":\"");
  // Add information on the status of the component
  if (component_state_ == kPendingSendQuery) *msg_ptr += "Alive";
  else if (component_state_ == kWaitingForReply) *msg_ptr += "Alive";
  else if (component_state_ == kDelayed) *msg_ptr += "Delayed";
  else if (component_state_ == kDead) *msg_ptr += "Dead";
  else if (component_state_ == kStartRequested) *msg_ptr += "StartRequested";
  else *msg_ptr += "ERROR state";
  *msg_ptr += "\", \"up_time\":\"" + anantak::microsec_to_time_str(up_time_) + "\"";
  *msg_ptr += ", \"status_str\":\"" + status_reply_str_ + "\"";
  *msg_ptr += ", \"process_id\":\"" + std::to_string(component_process_id_) + "\"}";
  VLOG(2) << component_name_ << ": generated status message = " << *msg_ptr;
  return msg_ptr;
}

/** Generate a component display panel in HTML
 *  Every component displays its status in a panel on a webpage. This method generates the HTML
 *  with all scripts to recieve the HTML via socket.io and update it via jQuery. At the startup
 *  ProgramCommander asks each ComponentKeeper for this HTML and then saves it to a fixed location
 *  on the file system that is shared with the WebCommander. WebCommander periodically loads the
 *  contents of the directory to render its display page. The number of panels is conveyed by the
 *  number of HTML panels stored and each panel has div-tagged HTML for display. WebCommander owns
 *  the stylesheets, locations where panels are displayed on webpage etc.
 */
std::unique_ptr<std::string> MessageFilesPublisherStatusKeeper::GenerateDisplayPanel() {
  // Create a ctemplate from the string
  std::unique_ptr<std::string> panel_str(new std::string());
  ctemplate::TemplateDictionary dict("Panel");
  dict.SetValue("component_name", component_name_);
  dict.SetValue("status", "Unknown");
  dict.SetValue("up_time", "Unknown");
  ctemplate::ExpandTemplate("data_publisher_panel_tpl", ctemplate::DO_NOT_STRIP, &dict,
      panel_str.get()); /** unfortunately using .get() here. This works but might be risky if
                         * ctemplate system is going to keep pointers to the string */
  VLOG(3) << "Display panel for " << component_name_ << "\n" << *panel_str;
  return panel_str;
}


}