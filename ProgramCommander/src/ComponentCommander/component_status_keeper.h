/**
 *  Component Status Keeper
 *
 *  This declares the ComponentStatusKeeper object. These are used by the ProgramCommander to keep
 *  track of status of each compnent running on the machine.
 */

#pragma once

// std includes
#include <string>
#include <memory>
#include <cstdint>
#include <functional>
#include <unistd.h>

/** Google Logging library */
#include <glog/logging.h>

// anantak includes
#include "common_config.h"

/** We can use protocol buffers to generate messages. Includes for these. */
#include "status_messages.pb.h"
#include <google/protobuf/text_format.h>

namespace anantak {

/**
 *  ComponentStatusKeeper
 */
class ComponentStatusKeeper {
 public: 
  /** Constructor creates a ComponentStatusKeeper object with default settings */
  ComponentStatusKeeper(std::string component_name);
  
  /** Same as above with a different template **/
  ComponentStatusKeeper(std::string component_name, std::string display_template_filename);
  
  bool Initiate();
  
  /** Destructs the ComponentStatusKeeper */
  virtual ~ComponentStatusKeeper();
  
  // ProcessCommand - check if the command is meant for this component then process.
  bool ProcessCommand(std::unique_ptr<std::string> command_string);
  bool SendStatusQueryNow(int64_t current_time);
  bool StartComponentNow(int64_t current_time, bool* is_manual_start);
  bool SendStatusNow(int64_t current_time);
    
  /** Generates a status query message, returns a unique pointer to serialized string
   *  This method could use protocol buffers or any other way of representing a message. But the
   *  ProgramCommander assumes that the message is created as a string. The string is then owned
   *  by the calling function.
   *  Calling function takes ownership of the string and is responsible for destructing the string.
   *  Making this virtual as we always want a derived class to override this method.
   */
  virtual std::unique_ptr<std::string> GenerateStatusQueryMessage();

  /** Updates status from a status reply message
   *  This takes in a serialized representation of the message object as a string. StatusKeeper can
   *  decide to use Protocol Buffers or whatever way to repesent its objects, but the transfer is
   *  done using a simple string object. Returns success or not in update operation.
   *  This method takes ownership of the string and is responsible for destructing it.
   *  Making this virtual as we always want a derived class to override this method.
   */
  virtual bool ProcessStatusReply(std::unique_ptr<std::string> status_reply_message);
  
  /** Generates a component status message
   *  This generates a serialized message as string that represents the status of the component
   *  being tracked. Typically Protocol Buffer library is used, but any thing can be used.
   *  Calling function takes ownership of the string and is responsible for destructing it.
   *  Making this virtual as we always want a derived class to override this method.
   */
  virtual std::unique_ptr<std::string> GenerateStatusMessage();
  
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
  virtual std::unique_ptr<std::string> GenerateStatusQueryReplyMessage(
      std::unique_ptr<std::string> status_query_msg);

  /** Generate a component display panel in HTML
   *  Every component displays its status in a panel on a webpage. This method generates the HTML
   *  with all scripts to recieve the HTML via socket.io and update it via jQuery. At the startup
   *  ProgramCommander asks each ComponentKeeper for this HTML and then saves it to a fixed location
   *  on the file system that is shared with the WebCommander. WebCommander periodically loads the
   *  contents of the directory to render its display page. The number of panels is conveyed by the
   *  number of HTML panels stored and each panel has div-tagged HTML for display. WebCommander owns
   *  the stylesheets, locations where panels are displayed on webpage etc.
   */
  virtual std::unique_ptr<std::string> GenerateDisplayPanel();
  
  /** Set a query time offset */
  bool set_status_query_time_offset(int this_component_num, int total_components);
    
  /** Set the component_status_function_ */
  typedef std::function<std::unique_ptr<std::string>()> StatusGenerationCallbackType;
  inline bool set_component_status_function(StatusGenerationCallbackType func) {
    component_status_function_ = func;
  }

  // accessors and mutators
  inline bool set_auto_restart_component(bool auto_restart_component) {
    auto_restart_component_ = auto_restart_component;
    VLOG(3) << component_name_ << ": set auto restart = " << auto_restart_component_;
    return true;
  }
  
 protected:
  // Settings - all times are in microseconds
  std::string component_name_;      /**< String name of the component */
  int64_t status_query_interval_;   /**< Time interval between status queries */
  int64_t raise_warning_interval_;  /**< Raise warning if not heard back for this long */
  int64_t assume_dead_interval_;    /**< Assume component is dead if not heard back for this long */
  int64_t restart_wait_interval_;   /**< Time to wait till the component restarts */
  int64_t status_interval_;         /**< Time interval for sending status updates to display */
  bool auto_restart_component_;     /**< Indicates if this is an auto restart component */
  
  // String commands
  std::string start_command_;       /**< Command to start the component */
  std::string exit_command_;        /**< Command to exit the component */
  
  // Operating variables
  int64_t start_time_;              /**< Time the component was started */
  int64_t start_offset_;            /**< Offest to start time so that queries can be staggerred */
  int64_t up_time_;                 /**< Time interval since component was created */
  std::string status_reply_str_;    /**< Status string that came back in the status reply */
  int64_t wait_time_;               /**< wait_time = current_time - last_query_send_time */
  int64_t last_status_query_time_;  /**< Last time a status query was sent */
  int64_t last_status_reply_time_;  /**< Last time a status query reply was recieved */
  int64_t last_status_time_;        /**< Last time when the component status was sent */
  int64_t restart_request_time_;    /**< Time when restart was requested */
  int64_t time_since_restart_;      /**< Time elapsed since restart was requested */
  bool manual_start_requested_;     /**< Indicates if a manual start is requested */
  int32_t this_process_id_;         /**< Process id of this process */
  int32_t component_process_id_;    /**< Process if of the component */
  
  enum ComponentState {
    kPendingSendQuery,
    kWaitingForReply,
    kDelayed,
    kDead,
    kStartRequested
  };
  ComponentState component_state_;  /**< Component state keeper */
  
  int64_t get_wall_time_microsec(); /**< Returns current wall time in microseconds */
  
  // Display Panel variables
  std::string display_template_filename_;  /**< Filename of the display template */
  std::string display_template_name_;  /**< Filename of the display template */
  std::string display_template_;           /**< Display template as string loaded from file */
  bool LoadDisplayTemplate();              /**< Function to load the display template from file */
  
  // Using protocol buffers for generating status query and reply messages
  anantak::StatusQuery status_query_msg_;  /**< StatusQuery msg holder used for serialization */
  anantak::StatusReply status_reply_msg_;  /**< StatusReply msg holder used for serialization */
  
  /** Callback function - provided by component
   *  Gives StatusKeeper a way to generate component-specific status string */
  StatusGenerationCallbackType component_status_function_;
  
};  
  
} // namespace anantak