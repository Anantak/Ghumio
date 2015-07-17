cmake_minimum_required(VERSION 2.8)
project(PixyReader)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_MODULE_PATH   ${PROJECT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})

find_package(Gflags REQUIRED)
find_package(Glog REQUIRED)
find_package(libpixyusb REQUIRED)
find_package(libusb-1.0 REQUIRED)
find_package(Boost 1.55 COMPONENTS thread system chrono REQUIRED)
find_package(UDev REQUIRED)

# use sse3 instruction set
#SET(CMAKE_CXX_FLAGS "-march=native -msse3 -O3")
# C++11 std
add_definitions("-march=native -msse4 -O3 -std=c++0x") 
# make release version
set(CMAKE_BUILD_TYPE Release)

# libpixy version
file(STRINGS "cmake/VERSION" LIBPIXY_VERSION)
add_definitions(-D__LIBPIXY_VERSION__="${LIBPIXY_VERSION}")

# include directories
include_directories(
  ${GFLAGS_INCLUDE_DIRS}
  ${GLOG_INCLUDE_DIRS}
  ${LIBPIXYUSB_0_INCLUDE_DIR}
  ${LIBUSB_1_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${UDEV_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}
)

# set source files
set(SOURCE_FILES
  ${PROJECT_SOURCE_DIR}/src/test_udev.cc
  ${PROJECT_SOURCE_DIR}/src/test_libusb.cc
)

# test
add_executable(test_udev src/test_udev.cc)
target_link_libraries(test_udev ${GLOG_LIBRARIES})
target_link_libraries(test_udev ${GFLAGS_LIBRARIES})
target_link_libraries(test_udev ${UDEV_LIBRARIES})

add_executable(test_libusb src/test_libusb.cc)
target_link_libraries(test_libusb ${GLOG_LIBRARIES})
target_link_libraries(test_libusb ${GFLAGS_LIBRARIES})
target_link_libraries(test_libusb ${LIBUSB_1_LIBRARIES})

add_executable(test_pixys src/test_pixys.cc)
target_link_libraries(test_pixys ${GLOG_LIBRARIES})
target_link_libraries(test_pixys ${GFLAGS_LIBRARIES})
target_link_libraries(test_pixys ${UDEV_LIBRARIES})
target_link_libraries(test_pixys ${LIBUSB_1_LIBRARIES})

add_executable(run_pixy src/run_pixy.cc)
target_link_libraries(run_pixy ${LIBPIXYUSB_0_LIBRARY}) # libpixyusb should always come before libboost and libusb #
target_link_libraries(run_pixy ${GLOG_LIBRARIES})
target_link_libraries(run_pixy ${GFLAGS_LIBRARIES})
target_link_libraries(run_pixy ${Boost_LIBRARIES})
target_link_libraries(run_pixy ${LIBUSB_1_LIBRARIES})
target_link_libraries(run_pixy ${UDEV_LIBRARIES})




