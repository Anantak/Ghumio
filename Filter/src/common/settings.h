/**
 *  Settings class
 *  
 *  Settings of different objects such as filters, models, data queues will derive from this. There
 *  are a number of settings that drive each filter, model, etc. Each setting object provides the
 *  capability to load the settings from a file. We have chosen to use Google's protocol buffers to
 *  keep a human-readable way to provide the settings.
 *
 *  Example usage is as follows. ModelSettings class derives from this. When a filter initiation
 *  is called, it gets the type-names of models to be created along with protobuf files with
 *  desired settings. A ModelFactory class is used to instantiate the Model objects using the 
 *  type-names. Model class instantiates a ModelSettings object loading its members from settings
 *  file. Model then starts its activities using the model settings.
 *  
 */


namespace anantak {

class Settings {
 public:
  Settings();
  virtual ~Settings();
  
  /**
   *  Load settings from a given filename
   *  Filename is an implementation of the protocol buffer message. This may not be the intended
   *  usage of the protocol buffers, but it seems very useful for this purpose. The derived class
   *  will always implement the loading procedure
   */
  virtual bool LoadSettingsFromProtobuf ( std::string config_filename );
  
  inline std::string get_settings_type () { return settings_type_; }

 private:
  
  std::string settings_type_;  /**< Identifies the type of the settings object */

}


} // namespace
