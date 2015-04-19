cmake_minimum_required(VERSION 2.8)
project(sliding_window_filter)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_MODULE_PATH   ${PROJECT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})

find_package(gflags REQUIRED)
find_package(Glog REQUIRED)
find_package(Protobuf REQUIRED)

# use sse3 instruction set
SET(CMAKE_CXX_FLAGS "-msse3")
# C++11 std
add_definitions("-std=c++0x") 
# make release version
set(CMAKE_BUILD_TYPE Release)

# compile protocol buffers
file(GLOB ProtoFiles "${PROJECT_SOURCE_DIR}/src/protobuf/*.proto")
PROTOBUF_GENERATE_CPP(ProtoSources ProtoHeaders ${ProtoFiles})

# include directories
include_directories(
  ${GFLAGS_INCLUDE_DIRS},
  ${GLOG_INCLUDE_DIRS},
  ${PROTOBUF_INCLUDE_DIRS},
  ${PROJECT_SOURCE_DIR}/src,
  ${PROJECT_SOURCE_DIR}/build
)

# Set source files
set(swf_SOURCE_FILES
  ${PROJECT_SOURCE_DIR}/src/test/test_config.cc
)

set(SOURCE_FILES
  ${swf_SOURCE_FILES}
)

# test
add_executable(test_config src/test/test_config.cc ${ProtoSources})
target_link_libraries(test_config ${GLOG_LIBRARIES})
target_link_libraries(test_config ${GFLAGS_LIBRARIES})
target_link_libraries(test_config ${PROTOBUF_LIBRARIES})
