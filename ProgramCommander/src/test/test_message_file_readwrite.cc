/**
 *  Test messages file creation and reading
 *
 *  Tests: creating a message file from older messages files, reading the files and extracting
 *  headers of the messages.
 */


/** Google libraries */
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <google/protobuf/message.h>

/** Old Anantak includes */
#include "../../../vision/include/anantak/parse_messages.hpp"

/** Anantak includes */
#include "DataQueue/message_file_writer.h"
#include "DataQueue/message_file_reader.h"
#include "DataQueue/message_file_stats.h"

/** Protocol buffers */
#include "sensor_messages.pb.h"

/** Command line flags */

int main(int argc, char** argv) {
  
  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);
  // Parse the commandline flags 
  google::ParseCommandLineFlags(&argc, &argv, true);
  // Verify the version of protobuf library 
  GOOGLE_PROTOBUF_VERIFY_VERSION;
  // Setting the glog output to stderr by default
  FLAGS_logtostderr = 1;
  
  // Machine interface data
  // Read the data file using old interface
  std::string mi_msgs_filename = "/home/manujnaman/Videos/machine_interface_data.txt";
  std::string mi_pb_msgs_filename = "src/test/machine_interface.pb.data";

  if (false) {
    std::string machine_interface_type = "MachineInterface";
    VLOG(1) << "  Reading machine interface data from " << mi_msgs_filename;
    std::vector<anantak::machine_interface_msg> mi_msgs =
        anantak::parse_messages_file<anantak::machine_interface_msg>(mi_msgs_filename);
    // Assemble the machine interface data
    VLOG(1) << "  Assembling machine interface data";
    std::vector<double>  mi_timing_vec; 
    for (int i_reading=0; i_reading < mi_msgs.size(); ++i_reading) { 
      mi_timing_vec.push_back(double(mi_msgs[i_reading].in_time));
    }
    anantak::straighten_out_ring_buffer<anantak::machine_interface_msg, double>(mi_msgs, mi_timing_vec);
    // Create protobuf messages and write to file
    VLOG(1) << "Total number of machine interface messages = " << mi_msgs.size();
    // Create a message writer
    anantak::MessageFileWriter mi_pb_file_writer;
    if (!mi_pb_file_writer.Open(mi_pb_msgs_filename)) {
      LOG(ERROR) << "File " << mi_pb_msgs_filename << " could not be opened.";
      return -1;
    }
    mi_pb_file_writer.StartTrackingPerformance();
    for (int i_reading=0; i_reading < mi_msgs.size(); ++i_reading) {
      // Create a new protobuf machine interface message and populate it with data
      anantak::SensorMsg msg;
      anantak::HeaderMsg* hdr_msg = msg.mutable_header();
      anantak::MachineInterfaceMsg* mi_msg = msg.mutable_mi_msg();
      // out_time is the time command was sent to the machine motors
      // in_time is when readings were read from the machine's sensors
      // we use machine interface to mainly get the sensor values. So message timestamp is in_time.
      hdr_msg->set_timestamp(int64_t(mi_msgs[i_reading].in_time));
      hdr_msg->set_type(machine_interface_type);
      hdr_msg->set_recieve_timestamp(int64_t(mi_msgs[i_reading].out_time));
      hdr_msg->set_send_timestamp(int64_t(mi_msgs[i_reading].in_time));
      mi_msg->set_out_time(int64_t(mi_msgs[i_reading].out_time));
      mi_msg->set_in_time(int64_t(mi_msgs[i_reading].in_time));
      mi_msg->set_left_motor(int32_t(mi_msgs[i_reading].left_motor));
      mi_msg->set_right_motor(int32_t(mi_msgs[i_reading].right_motor));
      mi_msg->set_left_servo(int32_t(mi_msgs[i_reading].left_servo));
      mi_msg->set_right_servo(int32_t(mi_msgs[i_reading].right_servo));
      mi_msg->set_in_counter(int32_t(mi_msgs[i_reading].in_counter));
      mi_msg->set_out_counter(int32_t(mi_msgs[i_reading].out_counter));
      mi_msg->set_rear_left_encoder(int32_t(mi_msgs[i_reading].rear_left_encoder));
      mi_msg->set_rear_right_encoder(int32_t(mi_msgs[i_reading].rear_right_encoder));
      mi_msg->set_front_left_encoder(int32_t(mi_msgs[i_reading].front_left_encoder));
      mi_msg->set_front_right_encoder(int32_t(mi_msgs[i_reading].front_right_encoder));
      mi_msg->set_delay(int32_t(mi_msgs[i_reading].delay));
      mi_msg->set_rear_left_current(int32_t(mi_msgs[i_reading].rear_left_current));
      mi_msg->set_rear_right_current(int32_t(mi_msgs[i_reading].rear_right_current));
      mi_msg->set_front_left_current(int32_t(mi_msgs[i_reading].front_left_current));
      mi_msg->set_front_right_current(int32_t(mi_msgs[i_reading].front_right_current));
      mi_msg->set_temperature(int32_t(mi_msgs[i_reading].temperature));
      // Message is now assembled. Write it to the file now.
      if (!mi_pb_file_writer.WriteMessage(msg)) {
        LOG(ERROR) << "Message not written";
      }
      VLOG_EVERY_N(3, 1000) << "Average message write time " << mi_pb_file_writer.MessageWriteTime();
    }
    mi_pb_file_writer.Close();
  }
  
  if (false) { /** Read the message file statistics */
    anantak::MessageFileStats mi_pb_file_stats;
    mi_pb_file_stats.CalculateFileStats(mi_pb_msgs_filename);
    /** We access a reference to the vector of send_timestamps. We avoid making a copy of vector.
     *  There is no need to ever change this. We could make a copy easily too by doing:
     *  std::vector<int64_t> mi_pb_ts = mi_pb_file_stats.timestamps(); but copy is not needed. */
    const std::vector<int64_t>& mi_pb_ts = mi_pb_file_stats.timestamps();   // returns a reference
    // Now that we have the timestamps, we need to build a schedule.
  }
  
  // IMU data conversion
  // Read the data file using old interface
  std::string imu_msgs_filename = "/home/manujnaman/Videos/imu1_interface_data.txt";
  std::string imu_pb_msgs_filename = "src/test/imu1_data.pb.data";
  if (true) {
    /** There could be multiple types of IMUs. One we use provides already-integrated rotations.
     *  So we get a quaternion for rotation and linear acclerations. Usually Imu provides raw
     *  Angular velocities and linear accelerations. ImuQuatAccel is first type. ImuAngveloAccel is
     *  the second type. */
    std::string imu_type = "ImuQuatAccel";
    VLOG(1) << "  Reading imu data from " << imu_msgs_filename;
    std::vector<anantak::imu_message> imu_msgs =
        anantak::parse_messages_file<anantak::imu_message>(imu_msgs_filename);
    // Assemble the machine interface data
    VLOG(1) << "  Assembling IMU data";
    std::vector<double>  imu_timestamps; 
    for (int i=0; i<imu_msgs.size(); ++i) { 
      imu_timestamps.push_back( double(imu_msgs[i].timestamp) );
    }
    anantak::straighten_out_ring_buffer<anantak::imu_message, double>(imu_msgs, imu_timestamps);
    // Create protobuf messages and write to file
    VLOG(1) << "Total number of imu messages = " << imu_msgs.size();
    // Create a message writer
    anantak::MessageFileWriter imu_pb_file_writer;
    if (!imu_pb_file_writer.Open(imu_pb_msgs_filename)) {
      LOG(ERROR) << "File " << imu_pb_msgs_filename << " could not be opened.";
      return -1;
    }
    imu_pb_file_writer.StartTrackingPerformance();
    for (int i_reading=0; i_reading < imu_msgs.size(); ++i_reading) {
      // Create a new protobuf machine interface message and populate it with data
      anantak::SensorMsg msg;
      anantak::HeaderMsg* hdr_msg = msg.mutable_header();
      hdr_msg->set_timestamp(int64_t(imu_msgs[i_reading].timestamp));
      hdr_msg->set_type(imu_type);
      hdr_msg->set_recieve_timestamp(int64_t(imu_msgs[i_reading].timestamp));
      hdr_msg->set_send_timestamp(int64_t(imu_msgs[i_reading].timestamp));
      anantak::ImuMsg* imu_msg = msg.mutable_imu_msg();
      imu_msg->set_imu_num(int32_t(imu_msgs[i_reading].imu_num));
      imu_msg->add_quaternion(int32_t(imu_msgs[i_reading].quaternion[0]));
      imu_msg->add_quaternion(int32_t(imu_msgs[i_reading].quaternion[1]));
      imu_msg->add_quaternion(int32_t(imu_msgs[i_reading].quaternion[2]));
      imu_msg->add_quaternion(int32_t(imu_msgs[i_reading].quaternion[3]));
      if (i_reading < 10) {
        VLOG(1) << i_reading << " " << hdr_msg->timestamp();
        VLOG(1) << i_reading << " " << imu_msgs[i_reading].quaternion[0] << " " <<
          int32_t(imu_msgs[i_reading].quaternion[1]) << " " << imu_msgs[i_reading].quaternion[2] << " " <<
          imu_msgs[i_reading].quaternion[3];
        VLOG(1) << imu_msg->quaternion_size();
      }
      imu_msg->add_linear(int32_t(imu_msgs[i_reading].linear[0]));
      imu_msg->add_linear(int32_t(imu_msgs[i_reading].linear[1]));
      imu_msg->add_linear(int32_t(imu_msgs[i_reading].linear[2]));
      imu_msg->add_angular(int32_t(imu_msgs[i_reading].angular[0]));
      imu_msg->add_angular(int32_t(imu_msgs[i_reading].angular[1]));
      imu_msg->add_angular(int32_t(imu_msgs[i_reading].angular[2]));
      // Message is now assembled. Write it to the file now.
      if (!imu_pb_file_writer.WriteMessage(msg)) {
        LOG(ERROR) << "Message not written";
      }
      VLOG_EVERY_N(1, 1000) << "Average message write time " << imu_pb_file_writer.MessageWriteTime();
    }
    imu_pb_file_writer.Close();
    VLOG(1) << "Closed file " << imu_pb_msgs_filename;
  }

  if (true) { /** Read the message file statistics */
    anantak::MessageFileStats imu_pb_file_stats;
    imu_pb_file_stats.CalculateFileStats(imu_pb_msgs_filename);
    /** We access a reference to the vector of send_timestamps. We avoid making a copy of vector.
     *  There is no need to ever change this. We could make a copy easily too by doing:
     *  std::vector<int64_t> mi_pb_ts = mi_pb_file_stats.timestamps(); but copy is not needed. */
    const std::vector<int64_t>& imu_pb_ts = imu_pb_file_stats.timestamps();   // returns a reference
    // Now that we have the timestamps, we need to build a schedule.
  }
  
  if (true) { /** Read messages file. Collect some stats from it. */
    std::string pb_msgs_filename = imu_pb_msgs_filename;
    anantak::MessageFileReader pb_file_reader;
    if (!pb_file_reader.Open(pb_msgs_filename)) {
      LOG(ERROR) << "File " << pb_msgs_filename << " could not be opened.";
      return -1;
    }
    pb_file_reader.StartTrackingPerformance();
    // Read messages one-by-one
    anantak::SensorMsg msg;
    int n_msgs_in_file = 0;
    // Message is read 'in place'. Message's fields will be written 'over' the existing message. 
    while (pb_file_reader.ReadMessage(&msg)) {
      // get the message header by reference
      const anantak::HeaderMsg& hdr = msg.header();
      // do something with the header
      VLOG_EVERY_N(3, 1000) << "Average message read time " << pb_file_reader.MessageReadTime();
      // do something with payload
      const anantak::ImuMsg& imu = msg.imu_msg();
      if (n_msgs_in_file < 10) {
        VLOG(1) << n_msgs_in_file << " " << hdr.timestamp();
        VLOG(1) << imu.quaternion_size();
        int32_t q0 = imu.quaternion(0);
        int32_t q1 = imu.quaternion(1);
        int32_t q2 = imu.quaternion(2);
        int32_t q3 = imu.quaternion(3);
        VLOG(1) << "Quaternion = " << q0 << " " << q1 << " " << q2 << " " << q3;
      }
      // Clear the message before next one is read, increment number of messages
      msg.Clear();
      n_msgs_in_file++;
    }
    //\pb_file_reader.Close(); /** no closing is necessary as file is already closed */
    VLOG(1) << "Read " << n_msgs_in_file << " messages from " << pb_msgs_filename;
  }
  
  
  return 0;
}
