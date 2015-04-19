/** Testing connection to multiple pixys */

/** std includes */
#include <libudev.h>
#include <stdio.h>
#include <stdlib.h>
#include <locale.h>
#include <unistd.h>
#include <libusb.h>

#include <ios>
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <algorithm>
#include <memory>

/** Google libraries */
#include <glog/logging.h>
#include <gflags/gflags.h>

namespace anantak {

  class InfraRedTrackerCamera {
   public:
   
    // Symlink is used to identify a camera from another
    std::string symlink_;
    
    // Pixy camera Vendor and Product Ids
    uint16_t vendor_id_;
    uint16_t product_id_;
    std::string vendor_id_str_;
    std::string product_id_str_;
    
    // Camera finding 
    bool device_found_;
    uint8_t bus_num_;
    uint8_t address_;
    
    // USB device finding
    bool usb_device_found_;
    libusb_context *m_context;
    libusb_device_handle *m_handle;
    
    InfraRedTrackerCamera(const std::string& symlink) {
      symlink_ = symlink;
      
      vendor_id_ = 0xb1ac;
      product_id_ = 0xf000;
      std::stringstream vss; vss << std::hex << vendor_id_;
      std::stringstream pss; pss << std::hex << product_id_;
      vendor_id_str_ = vss.str();
      product_id_str_ = pss.str();
      
      device_found_ = false;
      bus_num_ = 0;
      address_ = 0;
      
      usb_device_found_ = false;
      m_context = NULL;
      m_handle = NULL;
      
      VLOG(1) << "Created (uninitiated) camera with: ";
      VLOG(1) << "  symlink = " << symlink_;
      VLOG(1) << "  vendor id = " << vendor_id_str_;
      VLOG(1) << "  product id = " << product_id_str_;
    }
    
    virtual ~InfraRedTrackerCamera() {
      if (m_handle) {
        libusb_close(m_handle);
        LOG(INFO) << "Closed USB handle for camera " << symlink_;
      }
      if (m_context) {
        libusb_exit(m_context);
        LOG(INFO) << "Closed USB context.";
      }
    }
    
    bool Initiate() {
      // Find camera symlink using UDev
      if (!FindCameraSymlink()) {
        LOG(ERROR) << "Could not find camera symlink. Check /dev directory, udev rules and if camera is connected.";
        return false;
      }
      
      if (!FindUSBDevice()) {
        LOG(ERROR) << "Could not find the USB device using libusb.";
        return false;
      }
      
      return true;
    }
    
    // Find the Symlink using UDev library
    bool FindCameraSymlink() {
	    struct udev *udev;
	    struct udev_enumerate *enumerate;
	    struct udev_list_entry *devices, *dev_list_entry;
	    struct udev_device *dev;
      unsigned int bus_num, address;
      bus_num = 0;
      address = 0;
	
	    /* Create the udev object */
	    udev = udev_new();
	    if (!udev) {
		    LOG(ERROR) << "Can't create udev";
		    return false;
	    }
	
	    /* Create a list of the devices in the 'hidraw' subsystem. */
	    enumerate = udev_enumerate_new(udev);
	    udev_enumerate_add_match_sysattr(enumerate, "idVendor", vendor_id_str_.c_str());
	    udev_enumerate_add_match_sysattr(enumerate, "idProduct", product_id_str_.c_str());
	    udev_enumerate_scan_devices(enumerate);
	    devices = udev_enumerate_get_list_entry(enumerate);
	    /* For each item enumerated, print out its information.
	       udev_list_entry_foreach is a macro which expands to
	       a loop. The loop will be executed for each member in
	       devices, setting dev_list_entry to a list entry
	       which contains the device's path in /sys. */
	    udev_list_entry_foreach(dev_list_entry, devices) {
		    const char *path;
		    
		    if (!device_found_) {
		
		      /* Get the filename of the /sys entry for the device
		         and create a udev_device object (dev) representing it */
		      path = udev_list_entry_get_name(dev_list_entry);
		      dev = udev_device_new_from_syspath(udev, path);
		      
		      // Lookup symlinks for the device
		      struct udev_list_entry *list = udev_device_get_devlinks_list_entry(dev);
          while (list) {
            std::string dev_symlink(udev_list_entry_get_name(list));
            VLOG(1) << "Looking at: " << dev_symlink;
            device_found_ = (dev_symlink == symlink_);
            list = udev_list_entry_get_next(list);
          }
          
          if (device_found_) {
            VLOG(1) << "Found the Camera!";
		        /* usb_device_get_devnode() returns the path to the device node
		           itself in /dev. */
		        VLOG(1) << "Device Node Path: " << udev_device_get_devnode(dev);
		        VLOG(1) << "Device Dev Path: " << udev_device_get_devpath(dev);
		        VLOG(1) << "Device Sys Path: " << udev_device_get_syspath(dev);

		        /* From here, we can call get_sysattr_value() for each file
		           in the device's /sys entry. The strings passed into these
		           functions (idProduct, idVendor, serial, etc.) correspond
		           directly to the files in the directory which represents
		           the USB device. Note that USB strings are Unicode, UCS2
		           encoded, but the strings returned from
		           udev_device_get_sysattr_value() are UTF-8 encoded. */
		        VLOG(1) << "  VID/PID: "
		                << udev_device_get_sysattr_value(dev,"idVendor") << ", "
		                << udev_device_get_sysattr_value(dev, "idProduct");
		        VLOG(1) << "  "
		                << udev_device_get_sysattr_value(dev,"manufacturer") << ", "
		                << udev_device_get_sysattr_value(dev,"product");
		        VLOG(1) << "  serial: " 
		                << udev_device_get_sysattr_value(dev, "serial");		        
		        std::string bus_num_str(udev_device_get_sysattr_value(dev, "busnum"));
		        std::string address_str(udev_device_get_sysattr_value(dev, "devnum"));
		        VLOG(1) << "  busnum: " << bus_num_str;
		        VLOG(1) << "  devnum: " << address_str;
		        
		        bus_num = std::stoi(bus_num_str);
		        address = std::stoi(address_str);
		                 
		        /*struct udev_list_entry *properties_list = udev_device_get_properties_list_entry(dev);
            while (properties_list) {
              VLOG(1) << "  property " << udev_list_entry_get_name(properties_list)
                      << " " << udev_list_entry_get_value(properties_list);
              properties_list = udev_list_entry_get_next(properties_list);
            }

		        struct udev_list_entry *tags_list = udev_device_get_tags_list_entry(dev);
            while (tags_list) {
              VLOG(1) << "  tag name = " << udev_list_entry_get_name(tags_list);
              tags_list = udev_list_entry_get_next(tags_list);
            }*/
          }
          
		      udev_device_unref(dev);
		    } // device found
	    }
	    /* Free the enumerator object */
	    udev_enumerate_unref(enumerate);

	    udev_unref(udev);
	    
	    if (device_found_) {
	      bus_num_ = bus_num;
	      address_ = address;
  	    VLOG(1) << "Found camera " << symlink_ << " at bus,address " << int(bus_num_) << "," << int(address_);
	    } else {
	      VLOG(1) << "Could not find camera " << symlink_;
	    }
	    
      return true;
    } // FindCameraSymlink
    
    bool FindUSBDevice() {
    
      libusb_device **devs; //pointer to pointer of device, used to retrieve a list of devices
	    int r; //for return values
	    ssize_t cnt; //holding number of devices in list
	    r = libusb_init(&m_context); //initialize a library session
	    if (r < 0) {
		    LOG(ERROR) << "Init Error " << r; //there was an error
				return false;
	    }
	    libusb_set_debug(m_context, 3); //set verbosity level to 3, as suggested in the documentation
	    cnt = libusb_get_device_list(m_context, &devs); //get the list of devices
	    if (cnt < 0) {
		    LOG(ERROR) << "Get Device Error"; //there was an error
	    }
	    VLOG(1) << cnt << " Devices in list."; //print total number of usb devices
	    for (ssize_t i=0; i<cnt; i++) {
	    
	      if (!usb_device_found_) {
	        
	        libusb_device_descriptor desc;
	        int r = libusb_get_device_descriptor(devs[i], &desc);
	        if (r < 0) {
		        LOG(ERROR) << "Failed to get device descriptor";
		        continue;
	        }
	        
	        if (desc.idVendor == vendor_id_ && desc.idProduct == product_id_) {
	          uint8_t bus_num = libusb_get_bus_number(devs[i]);
	          uint8_t address = libusb_get_device_address(devs[i]);
	          
	          if (bus_num == bus_num_ && address == address_) {
	            usb_device_found_ = true;
	            VLOG(1) << "Found USB device for camera";
	            VLOG(1) << "Bus, address = " << int(bus_num) << " " << int(address);
	            VLOG(1) << "Number of possible configurations: "<<(int)desc.bNumConfigurations<<"  "
	              <<"Device Class: "<<(int)desc.bDeviceClass<<"  "
	              <<"VendorID: "<<desc.idVendor<<"  "
	              <<"ProductID: "<<desc.idProduct;
	              
	            VLOG(1) << "Getting a handle to the camera";
	            int r_open = libusb_open(devs[i], &m_handle);
	            if (r_open == 0) {
	              LOG(INFO) << "USB device handle was received successfuly";
	            } else {
	              LOG(ERROR) << "USB device handle was not recieved.";
	              LOG(ERROR) << "Error code = " << r_open;
	            }
	            
	            /*libusb_config_descriptor *config;
	            libusb_get_config_descriptor(dev, 0, &config);
	            cout<<"Interfaces: "<<(int)config->bNumInterfaces<<" ||| ";
	            const libusb_interface *inter;
	            const libusb_interface_descriptor *interdesc;
	            const libusb_endpoint_descriptor *epdesc;
	            for(int i=0; i<(int)config->bNumInterfaces; i++) {
		            inter = &config->interface[i];
		            cout<<"Number of alternate settings: "<<inter->num_altsetting<<" | ";
		            for(int j=0; j<inter->num_altsetting; j++) {
			            interdesc = &inter->altsetting[j];
			            cout<<"Interface Number: "<<(int)interdesc->bInterfaceNumber<<" | ";
			            cout<<"Number of endpoints: "<<(int)interdesc->bNumEndpoints<<" | ";
			            for(int k=0; k<(int)interdesc->bNumEndpoints; k++) {
				            epdesc = &interdesc->endpoint[k];
				            cout<<"Descriptor Type: "<<(int)epdesc->bDescriptorType<<" | ";
				            cout<<"EP Address: "<<(int)epdesc->bEndpointAddress<<" | ";
			            }
		            }
	            }
	            cout<<endl<<endl<<endl;
            	libusb_free_config_descriptor(config);  */
            	
	          } // if bus number and address match
          	
	        } // if vid,pid match
	      
	      } // if not found yet	      
	      
	    }
	    libusb_free_device_list(devs, 1); //free the list, unref the devices in it
    
      if (usb_device_found_) {
        VLOG(1) << "USB Camera was found successfully";
      } else {
        VLOG(1) << "Could not find USB camera";
      }
    
      return true;
    }  // FindUSBDevice
    
    
    // Open 
    
  }; // InfraRedTrackerCamera

}


int main(int argc, char** argv) {  
  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);
  // Parse the commandline flags 
  google::ParseCommandLineFlags(&argc, &argv, true);
  // Verify the version of protobuf library 
  //GOOGLE_PROTOBUF_VERIFY_VERSION;
  // Setting the glog output to stderr by default
  FLAGS_logtostderr = 1;
  
  std::vector<std::string> pixycam_symlinks = {
    "/dev/PixyCam1",
    "/dev/PixyCam2"
  };
  
  // Startup pixy cams
  for (int i=0; i<pixycam_symlinks.size(); i++) {
    anantak::InfraRedTrackerCamera ir_cam(pixycam_symlinks[i]);
    ir_cam.Initiate();
  }

  return 0;
}

