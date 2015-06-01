cmake_minimum_required(VERSION 2.8)
project(ProgramCommander)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_MODULE_PATH   ${PROJECT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})

find_package(Gflags REQUIRED)
find_package(Glog REQUIRED)
find_package(Protobuf REQUIRED)
find_package(ZMQ REQUIRED)
find_package(Ctemplate REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
#find_package(Boost 1.55 REQUIRED)
# Includes for Pixy Camera - should we keep adding to this CMakeLists? Or should all be separate?
find_package(libpixyusb REQUIRED)
find_package(libusb-1.0 REQUIRED)
find_package(Boost 1.55 COMPONENTS thread system chrono REQUIRED)
find_package(UDev REQUIRED)

# Packages needed for building the pixycam

# use sse3 instruction set
#SET(CMAKE_CXX_FLAGS "-march=native -msse3 -O3")
# C++11 std
#add_definitions("-march=native -msse4 -O3 -std=c++0x") 
add_definitions("-msse4 -O3 -std=c++0x") 
# make release version
set(CMAKE_BUILD_TYPE Release)

# compile protocol buffers
set(ProtoFiles
  ${PROJECT_SOURCE_DIR}/src/Protobuf/programs_setup.proto
  ${PROJECT_SOURCE_DIR}/src/Protobuf/status_messages.proto
  ${PROJECT_SOURCE_DIR}/src/Protobuf/configurations.proto
  ${PROJECT_SOURCE_DIR}/src/Protobuf/sensor_messages.proto
)
PROTOBUF_GENERATE_CPP(ProtoSources ProtoHeaders ${ProtoFiles})

# configure header files
configure_file(${PROJECT_SOURCE_DIR}/src/Utilities/common_config.h.in ${PROJECT_SOURCE_DIR}/build/common_config.h)

# Add OpenGV's library
SET(OPENGV_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/src/Thirdparty/opengv/include)
add_library(opengv SHARED IMPORTED)
set_target_properties(opengv PROPERTIES IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/src/Thirdparty/opengv/lib/libopengv.so)

# Add AprilTag library from Prof. Edwin Olson
SET(APRILTAG_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/src/Thirdparty/apriltag)
SET(APRILTAG_LIBRARY ${PROJECT_SOURCE_DIR}/src/Thirdparty/apriltag/libapriltag.a)

# include directories
include_directories(
  ${GFLAGS_INCLUDE_DIRS}
  ${GLOG_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIRS}
  ${ZMQ_INCLUDE_DIRS}
  ${CTEMPLATE_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
  ${OPENGV_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/build
  ${PROJECT_SOURCE_DIR}/src/Thirdparty
  ${PROJECT_SOURCE_DIR}/src/Thirdparty/cv2cg
  /usr/include/flycapture
  ${APRILTAG_INCLUDE_DIR}
  # Pixy camera includes - should this be separated?
  ${LIBPIXYUSB_0_INCLUDE_DIR}
  ${LIBUSB_1_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${UDEV_INCLUDE_DIRS}
)

# set source files
set(SOURCE_FILES
  ${ProtoSources}
  ${PROJECT_SOURCE_DIR}/src/Utilities/common_functions.cc
  ${PROJECT_SOURCE_DIR}/src/ComponentCommander/component_commander.cc
  ${PROJECT_SOURCE_DIR}/src/ComponentCommander/component_status_keeper.cc
  ${PROJECT_SOURCE_DIR}/src/ComponentCommander/component_status_keeper_factory.cc
  ${PROJECT_SOURCE_DIR}/src/DataQueue/message_queue.cc
  ${PROJECT_SOURCE_DIR}/src/DataQueue/data_queue.cc
  ${PROJECT_SOURCE_DIR}/src/DataQueue/message_file_writer.cc
  ${PROJECT_SOURCE_DIR}/src/DataQueue/message_file_reader.cc
  ${PROJECT_SOURCE_DIR}/src/DataQueue/message_file_stats.cc
  ${PROJECT_SOURCE_DIR}/src/DataQueue/message_files_publisher.cc
  ${PROJECT_SOURCE_DIR}/src/DataQueue/message_files_publisher_status_keeper.cc
  ${PROJECT_SOURCE_DIR}/src/Filter/sliding_window_filter.cc
  ${PROJECT_SOURCE_DIR}/src/Filter/message_factory.cc
)

# set source files
set(COMPONENT_SOURCE_FILES
  ${ProtoSources}
  ${PROJECT_SOURCE_DIR}/src/Utilities/common_functions.cc
  ${PROJECT_SOURCE_DIR}/src/ComponentCommander/component_commander.cc
  ${PROJECT_SOURCE_DIR}/src/ComponentCommander/component_status_keeper.cc
)

# libviso2 source files
FILE(GLOB LIBVISO2_SRC_FILES "${PROJECT_SOURCE_DIR}/src/Thirdparty/libviso2/*.cpp")

# test

add_executable(test_component_commander src/test/test_component_commander.cc ${SOURCE_FILES})
target_link_libraries(test_component_commander ${GLOG_LIBRARIES})
target_link_libraries(test_component_commander gflags)
target_link_libraries(test_component_commander ${PROTOBUF_LIBRARIES})
target_link_libraries(test_component_commander ${ZMQ_LIBRARIES})
target_link_libraries(test_component_commander ${CTEMPLATE_LIBRARIES})

add_executable(test_component src/test/test_component.cc ${SOURCE_FILES})
target_link_libraries(test_component ${GLOG_LIBRARIES})
target_link_libraries(test_component gflags)
target_link_libraries(test_component ${PROTOBUF_LIBRARIES})
target_link_libraries(test_component ${ZMQ_LIBRARIES})
target_link_libraries(test_component ${CTEMPLATE_LIBRARIES})

add_executable(test_message_queue src/test/test_message_queue.cc ${SOURCE_FILES})
target_link_libraries(test_message_queue ${GLOG_LIBRARIES})
target_link_libraries(test_message_queue gflags)
target_link_libraries(test_message_queue ${PROTOBUF_LIBRARIES})
target_link_libraries(test_message_queue ${ZMQ_LIBRARIES})
target_link_libraries(test_message_queue ${CTEMPLATE_LIBRARIES})

add_executable(test_data_queue src/test/test_data_queue.cc ${SOURCE_FILES})
target_link_libraries(test_data_queue ${GLOG_LIBRARIES})
target_link_libraries(test_data_queue gflags)
target_link_libraries(test_data_queue ${PROTOBUF_LIBRARIES})
target_link_libraries(test_data_queue ${ZMQ_LIBRARIES})
target_link_libraries(test_data_queue ${CTEMPLATE_LIBRARIES})

#add_executable(test_message_file_readwrite src/test/test_message_file_readwrite.cc ${SOURCE_FILES})
#target_link_libraries(test_message_file_readwrite ${GLOG_LIBRARIES})
#target_link_libraries(test_message_file_readwrite gflags)
#target_link_libraries(test_message_file_readwrite ${PROTOBUF_LIBRARIES})
#target_link_libraries(test_message_file_readwrite ${ZMQ_LIBRARIES})
#target_link_libraries(test_message_file_readwrite ${CTEMPLATE_LIBRARIES})

add_executable(test_data_publisher src/test/test_data_publisher.cc ${SOURCE_FILES})
target_link_libraries(test_data_publisher ${GLOG_LIBRARIES})
target_link_libraries(test_data_publisher gflags)
target_link_libraries(test_data_publisher ${PROTOBUF_LIBRARIES})
target_link_libraries(test_data_publisher ${ZMQ_LIBRARIES})
target_link_libraries(test_data_publisher ${CTEMPLATE_LIBRARIES})

add_executable(test_filter src/test/test_filter.cc ${SOURCE_FILES})
target_link_libraries(test_filter ${GLOG_LIBRARIES})
target_link_libraries(test_filter gflags)
target_link_libraries(test_filter ${PROTOBUF_LIBRARIES})
target_link_libraries(test_filter ${ZMQ_LIBRARIES})
target_link_libraries(test_filter ${CTEMPLATE_LIBRARIES})

add_executable(test_monocular_features_tracker EXCLUDE_FROM_ALL src/test/test_monocular_features_tracker.cc
    ${SOURCE_FILES} ${LIBVISO2_SRC_FILES})
target_link_libraries(test_monocular_features_tracker ${GLOG_LIBRARIES})
target_link_libraries(test_monocular_features_tracker gflags)
target_link_libraries(test_monocular_features_tracker ${PROTOBUF_LIBRARIES})
target_link_libraries(test_monocular_features_tracker ${ZMQ_LIBRARIES})
target_link_libraries(test_monocular_features_tracker ${CTEMPLATE_LIBRARIES})
target_link_libraries(test_monocular_features_tracker ${OpenCV_LIBS})

add_executable(test_batch_vio src/Models/test_batch_vio.cc ${SOURCE_FILES})
target_link_libraries(test_batch_vio ${GLOG_LIBRARIES})
target_link_libraries(test_batch_vio gflags)
target_link_libraries(test_batch_vio ${PROTOBUF_LIBRARIES})
target_link_libraries(test_batch_vio ${ZMQ_LIBRARIES})
target_link_libraries(test_batch_vio ${CTEMPLATE_LIBRARIES})
target_link_libraries(test_batch_vio ${CERES_LIBRARIES})
target_link_libraries(test_batch_vio ${BOOST_LIBRARIES})
target_link_libraries(test_batch_vio opengv)

add_executable(test_sync_images_broadcaster
  src/test/test_sync_images_broadcaster.cc
  src/ImagesBroadcaster/sync_images_broadcaster.cc ${COMPONENT_SOURCE_FILES})
target_link_libraries(test_sync_images_broadcaster ${GLOG_LIBRARIES})
target_link_libraries(test_sync_images_broadcaster ${GFLAGS_LIBRARIES})
target_link_libraries(test_sync_images_broadcaster ${PROTOBUF_LIBRARIES})
target_link_libraries(test_sync_images_broadcaster ${ZMQ_LIBRARIES})
target_link_libraries(test_sync_images_broadcaster ${CTEMPLATE_LIBRARIES})
target_link_libraries(test_sync_images_broadcaster flycapture-c flycapture)

add_executable(test_show_save_images
  src/test/test_show_save_images.cc
  src/ImagesBroadcaster/show_save_images.cc ${COMPONENT_SOURCE_FILES})
target_link_libraries(test_show_save_images ${GLOG_LIBRARIES})
target_link_libraries(test_show_save_images ${GFLAGS_LIBRARIES})
target_link_libraries(test_show_save_images ${PROTOBUF_LIBRARIES})
target_link_libraries(test_show_save_images ${ZMQ_LIBRARIES})
target_link_libraries(test_show_save_images ${CTEMPLATE_LIBRARIES})
target_link_libraries(test_show_save_images ${OpenCV_LIBS})
target_link_libraries(test_show_save_images ${APRILTAG_LIBRARY})

add_executable(test_pixy_camera src/test/test_pixy_camera.cc
  src/PixyBroadcaster/pixy_camera_component.cc ${COMPONENT_SOURCE_FILES})
target_link_libraries(test_pixy_camera ${LIBPIXYUSB_0_LIBRARY}) # libpixyusb should always come before libboost and libusb #
target_link_libraries(test_pixy_camera ${GLOG_LIBRARIES})
target_link_libraries(test_pixy_camera ${GFLAGS_LIBRARIES})
target_link_libraries(test_pixy_camera ${Boost_LIBRARIES})
target_link_libraries(test_pixy_camera ${LIBUSB_1_LIBRARIES})
target_link_libraries(test_pixy_camera ${UDEV_LIBRARIES})
target_link_libraries(test_pixy_camera ${PROTOBUF_LIBRARIES})
target_link_libraries(test_pixy_camera ${ZMQ_LIBRARIES})
target_link_libraries(test_pixy_camera ${CTEMPLATE_LIBRARIES})
target_link_libraries(test_pixy_camera ${OpenCV_LIBS})

add_executable(test_beacon_serial_monitor src/test/test_beacon_serial_monitor.cc
  src/SerialBroadcaster/beacon_serial_monitor.cc ${COMPONENT_SOURCE_FILES})
target_link_libraries(test_beacon_serial_monitor ${GLOG_LIBRARIES})
target_link_libraries(test_beacon_serial_monitor ${GFLAGS_LIBRARIES})
target_link_libraries(test_beacon_serial_monitor ${PROTOBUF_LIBRARIES})
target_link_libraries(test_beacon_serial_monitor ${ZMQ_LIBRARIES})
target_link_libraries(test_beacon_serial_monitor ${CTEMPLATE_LIBRARIES})
target_link_libraries(test_beacon_serial_monitor usb)

add_executable(test_front_rangers_monitor src/test/test_front_rangers_monitor.cc
  src/SerialBroadcaster/front_rangers_monitor.cc ${COMPONENT_SOURCE_FILES})
target_link_libraries(test_front_rangers_monitor ${GLOG_LIBRARIES})
target_link_libraries(test_front_rangers_monitor ${GFLAGS_LIBRARIES})
target_link_libraries(test_front_rangers_monitor ${PROTOBUF_LIBRARIES})
target_link_libraries(test_front_rangers_monitor ${ZMQ_LIBRARIES})
target_link_libraries(test_front_rangers_monitor ${CTEMPLATE_LIBRARIES})
target_link_libraries(test_front_rangers_monitor usb)

# old executables - may no longer compile

#add_executable(test_fork src/test/test_fork.cc)

#add_executable(test_proto_fork src/test/test_proto_fork.cc ${ProtoSources})
#target_link_libraries(test_proto_fork ${GLOG_LIBRARIES})
#target_link_libraries(test_proto_fork ${GFLAGS_LIBRARIES})
#target_link_libraries(test_proto_fork ${PROTOBUF_LIBRARIES})
