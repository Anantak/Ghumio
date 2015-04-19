/**
 *  Component Status Keeper
 *
 *  This declares the ComponentStatusKeeper object. These are used by the ProgramCommander to keep
 *  track of status of each compnent running on the machine.
 */

// std includes
#include <string>
#include <memory>
#include <cstdint>
#include <sys/time.h>
#include <stdexcept>      // std::invalid_argument

/** Google Logging library */
#include <glog/logging.h>

// Anantak includes
#include <ComponentCommander/component_status_keeper.h>
#include <Utilities/common_functions.h>

// include for CTemplate
#include <ctemplate/template.h>
// for reading in template file
#include <fstream>

// include for microsec to string conversion
#include <cstdlib>

namespace anantak {

/**
 *  ComponentStatusKeeper
 */
/** Constructor creates a ComponentStatusKeeper object with default settings */
ComponentStatusKeeper::ComponentStatusKeeper(std::string component_name) {
  component_name_ = component_name;
  status_query_interval_ = (int64_t) 2000000;
  raise_warning_interval_ = (int64_t) 4000000;
  assume_dead_interval_ = (int64_t) 6000000;
  restart_wait_interval_ = (int64_t) 3000000;
  status_interval_ = (int64_t) 2000000;
  auto_restart_component_ = false;
  manual_start_requested_ = false;
  start_time_ = get_wall_time_microsec();
  up_time_ = 0;
  component_state_ = kPendingSendQuery;
  last_status_query_time_ = (int64_t) 0;
  last_status_reply_time_ = (int64_t) 0;
  restart_request_time_ = (int64_t) 0;
  last_status_time_ = (int64_t) 0;
  start_command_ = "COMMAND " + component_name_ + " start";
  exit_command_ = "COMMAND " + component_name_ + " exit";
  display_template_filename_ = "src/ComponentCommander/component_status.tpl";
  if (LoadDisplayTemplate()) {
    ctemplate::StringToTemplateCache("panel_tpl", display_template_.c_str(), ctemplate::DO_NOT_STRIP);
  }
  this_process_id_ = int32_t(getpid());
  component_process_id_ = 0;
  // Report
  VLOG(1) << "Built a StatusKeeper. Type = ComponentStatus, Name = " << component_name_;
  VLOG(3) << "Start command = " << start_command_;
  VLOG(3) << "Exit command = " << exit_command_;
  VLOG(3) << "Start time = " << start_time_;
  VLOG(3) << "Display template = " << display_template_;
}

/** Destructs the ComponentStatusKeeper */
ComponentStatusKeeper::~ComponentStatusKeeper() {
  VLOG(1) << "Destructing the status keeper";
}

/** Set a startup time offset */
bool ComponentStatusKeeper::set_status_query_time_offset(
    int this_component_num, int total_components) {
  start_offset_ = int64_t(float(status_query_interval_)
                          *(1.0f-float(this_component_num)/float(total_components)));
  last_status_query_time_ = start_time_ - start_offset_;
  int64_t status_offset = int64_t(float(status_interval_)
                          *(1.0f-float(this_component_num)/float(total_components)));
  last_status_time_ = start_time_ - status_offset;
  VLOG(1) << "Setting start offset = " << start_offset_;
  return true;
}

/** Utility to get wall time in microseconds */
int64_t ComponentStatusKeeper::get_wall_time_microsec() {
  struct timeval tv;
  gettimeofday (&tv, NULL);
  return (int64_t) (tv.tv_sec * 1000000 + tv.tv_usec);  
}

bool ComponentStatusKeeper::ProcessCommand(std::unique_ptr<std::string> command_string) {
  VLOG(4) << "Keeper " << component_name_ << " got a command = " << *command_string;
  if (start_command_.compare(*command_string) == 0) {
    manual_start_requested_ = true;
    VLOG(2) << component_name_ << ": manual_start_requested_ = true";
  }
  return true;
}

bool ComponentStatusKeeper::SendStatusQueryNow(int64_t current_time) {
  wait_time_ = current_time - last_status_query_time_;
  if (component_state_ == kPendingSendQuery) {
    if (wait_time_ < status_query_interval_) {
      // Stay in the PendingSendQuery state
      return false; // No query needed
    } else {
      // Send a query and transition to waiting state
      component_state_ = kWaitingForReply;
      return true; // Send a status query
    }
  } else if (component_state_ == kWaitingForReply) {
    if (wait_time_ < raise_warning_interval_) {
      // Stay in waiting state
      return false; // No query needed
    } else {
      // Send a second query and transition to Delayed state
      component_state_ = kDelayed;
      return true; // Send a status query
    }
  } else if (component_state_ == kDelayed) {
    if (wait_time_ < assume_dead_interval_) {
      // Stay in Delayed state
      return false; // No query needed
    } else {
      // Transition to Dead state
      component_state_ = kDead;
      return false; // No query needed
    }
  } else if (component_state_ == kDead) {
    // in Dead state, no status queries are sent
    return false;
  } else if (component_state_ == kStartRequested) {
    time_since_restart_ = current_time - restart_request_time_;
    if (time_since_restart_ < restart_wait_interval_) {
      // Stay in RestartRequested state
      return false; // No query needed
    } else {
      // Transition to PendingSendQuery state
      component_state_ = kPendingSendQuery;
      return false; // No query needed here
    }
  } else {
    LOG(ERROR) << "Keeper for " << component_name_ << " is in ERROR state.";
    return false;
  }
  return false;
}

bool ComponentStatusKeeper::StartComponentNow(int64_t current_time, bool* is_manual_start) {
  *is_manual_start = manual_start_requested_;
  if (component_state_ == kDead) {
    if (auto_restart_component_) {
      manual_start_requested_ = false;
      component_state_ = kStartRequested;
      restart_request_time_ = get_wall_time_microsec();
      time_since_restart_ = 0;
      return true;
    } else {
      if (manual_start_requested_) {
        manual_start_requested_ = false;
        component_state_ = kStartRequested;
        restart_request_time_ = get_wall_time_microsec();
        time_since_restart_ = 0;
        return true;
      }
    }
  }
  return false;
}

bool ComponentStatusKeeper::SendStatusNow(int64_t current_time) {
  if (last_status_time_+status_interval_ < current_time) {
    last_status_time_ = get_wall_time_microsec();
    //VLOG(2) << component_name_ << ": Yes to send status";
    return true;
  } else {
    //VLOG(2) << component_name_ << ": No to send status " << last_status_time_+status_interval_ << " " << current_time;
    return false;
  }
}

/** Generate a Status Query message
 *  Returns a unique pointer to serialized string 
 */
std::unique_ptr<std::string> ComponentStatusKeeper::GenerateStatusQueryMessage() {
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
std::unique_ptr<std::string> ComponentStatusKeeper::GenerateStatusQueryReplyMessage(
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
bool ComponentStatusKeeper::ProcessStatusReply(std::unique_ptr<std::string> status_reply_message) {
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
std::unique_ptr<std::string> ComponentStatusKeeper::GenerateStatusMessage() {
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
std::unique_ptr<std::string> ComponentStatusKeeper::GenerateDisplayPanel() {
  // Create a ctemplate from the string
  std::unique_ptr<std::string> panel_str(new std::string());
  ctemplate::TemplateDictionary dict("Panel");
  dict.SetValue("component_name", component_name_);
  dict.SetValue("status", "Unknown");
  dict.SetValue("up_time", "Unknown");
  ctemplate::ExpandTemplate("panel_tpl", ctemplate::DO_NOT_STRIP, &dict,
      panel_str.get()); /** unfortunately using .get() here. This works but might be risky if
                         * ctemplate system is going to keep pointers to the string */
  VLOG(3) << "Display panel for " << component_name_ << "\n" << *panel_str;
  return panel_str;
}

/** Load the display template file */
bool ComponentStatusKeeper::LoadDisplayTemplate() {
  std::string template_file_path =
      anantak::GetProjectSourceDirectory() + "/" + display_template_filename_;
  std::ifstream in(template_file_path, std::ios::in);
  if (in) {
    in.seekg(0, std::ios::end);
    display_template_.resize(in.tellg());
    in.seekg(0, std::ios::beg);
    in.read(&display_template_[0], display_template_.size());
    in.close();
  } else {
    LOG(ERROR) << "Could not open template file: " << display_template_filename_;
    return false;
  }
  return true;
}


} // namespace anantak
