/**
 *  Data Loader
 *
 *  This loads data from files on disk on a schedule. Data Loader does something with every block
 *  of data it loads. E.g. Data Loader can pass each block of data to a Data Publisher that will
 *  store and publish the data as needed. Another example is to load the data and simply pass it
 *  to a Data Queue that will keep a data buffer an provide it as needed.
 */

namespace anantak {

class DataLoader {
 public:
  /** Constructor - creates an empty DataLoader */
  DataLoader();
  
  /** Destructor - all objects are self-destructing */
  virtual ~DataLoader();
  
  /** Load Configuration
   *  This specifies: Number of seconds of each block of data to be loaded, Looping frequency,
   *  Starting time of message loading (optional), Safety margin (a time interval), names of the
   *  sensors and their data files, 
   */
  
  /** Create Loading Schedule
   *  This uses the number of seconds of data to be loaded in every block to calculate a schedule
   *  of data loading. Loads for each file are staggered in time. Current loading time includes the
   *  safety margin. 
   */
  
  /** Open Data Files
   *  This opens all data files and creates a map of MessageFileReaders. Each reader will be used
   *  to read its file
   */
  
  /** Create Data Reciever
   *  Create the thread that will listen to the messages loaded on an inproc socket.
   */
  
  /** Create Commands and Status Subscribers
   *  Create a subscriber to commands from the manual/auto commander plus a commands handler.
   *  Create a subscriber to status queries from Program Commander, plus a StatusKeeper.
   */
  
  /** Start Looping
   *  Starts the main loop at the looping frequency. In each loop Data Loader checks if it is time
   *  to load data from a file and if there are any messages from status queries, commands or
   *  Queueing times, handles those with corresponding handlers.
   */
  
  /** Start Loading Data - main work horse. This gets a config file that specifies which files are
   *  to be loaded. Each file is associated with a 'name' that is usually the name of the sensor.
   *  A certain number of seconds of data (provded as config) is loaded for every file in sequence.
   *  This makes an effort to stagger the loads. Loading is triggerred inside a loop that runs at
   *  a frequency specified in config. 
   */
  
  /** Update Schedule
   *  As data is loaded and transferred to the function needing the data, the time taken for the
   *  data to be usable for the calling function is important so that the delay can be addressed
   *  in the schedule. This is the objective of update schedule.
   */
 
 private:

};

} // namespace anantak