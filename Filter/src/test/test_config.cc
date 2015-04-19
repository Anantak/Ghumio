/**
 *  TestConfig
 *
 *  Used for testing writing/loading of config files for the filter and writing it back.
 *
 */

#include <fcntl.h>

#include <glog/logging.h>
#include <gflags/gflags.h>
#include <google/protobuf/text_format.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>

// including protocol buffers
#include "settings_messages.pb.h"

// default commandline flags. These work with the gflags library
DEFINE_string(filter_config_out, "./filter_out.cfg", "output config file");
DEFINE_string(filter_config_in, "./filter_in.cfg", "input config file");

int main(int argc, char** argv) {
  
  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);
  // Parse the commandline flags 
  google::ParseCommandLineFlags(&argc, &argv, true);
  // Verify that the version of the library that we linked against is
  // compatible with the version of the headers we compiled against.
  GOOGLE_PROTOBUF_VERIFY_VERSION;
  
  // Setting the output to stderr by default
  FLAGS_logtostderr = 1;
  LOG(INFO) << "filter_config = " << FLAGS_filter_config_out;
  
  // Create a Sliding Window Filter Settings object
  anantak::SlidingWindowFilterSettings filter_settings;
  // add data
  filter_settings.set_name("fast_filter");
  filter_settings.set_sliding_window_length(2.0);
  filter_settings.set_update_frequency(200.0);
  // display the text format of the object
  std::string filter_settings_text_format;
  google::protobuf::TextFormat::PrintToString(filter_settings, &filter_settings_text_format);
  LOG(INFO) << "filter_settings text format = \n" << filter_settings_text_format;
  // now output it to the given config file in text_format
  int file_descriptor = open(FLAGS_filter_config_out.c_str(), O_WRONLY | O_CREAT, S_IRWXU);
  if (file_descriptor < 0) {
    LOG(ERROR) << " Error opening the file: " << strerror(errno);
    return -1;
  }
  LOG(INFO) << "Writing the message out to file " << FLAGS_filter_config_out;
  google::protobuf::io::FileOutputStream file_output_stream(file_descriptor);
  google::protobuf::TextFormat::Print(filter_settings, &file_output_stream);
  file_output_stream.Close();
  //int result = close(file_descriptor);
  //if (result < 0) {
  //  LOG(ERROR) << " Error on closing Errno = " << strerror(errno);
  //}
  
  // Read a settings object and convert to a new settings object
  anantak::SlidingWindowFilterSettings filter_settings_read;
  // open the file
  int in_file_descriptor = open(FLAGS_filter_config_in.c_str(), O_RDONLY);
  if (in_file_descriptor < 0) {
    LOG(ERROR) << " Error opening the file: " << strerror(errno);
    return -1;
  }
  // parse the file
  google::protobuf::io::FileInputStream file_input_stream(in_file_descriptor);
  google::protobuf::TextFormat::Parse(&file_input_stream, &filter_settings_read);
  file_input_stream.Close();
  // convert to string to display
  std::string in_filter_settings_text_format;
  google::protobuf::TextFormat::PrintToString(filter_settings_read, &in_filter_settings_text_format);
  LOG(INFO) << "input filter_settings text format = \n" << in_filter_settings_text_format;
  
  return 0;
}
