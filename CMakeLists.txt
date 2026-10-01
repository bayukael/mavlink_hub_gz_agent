cmake_minimum_required(VERSION 3.13)
project(mavlink_hub_gz_agent VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_RPATH_USE_ORIGIN TRUE)

find_package(gz-cmake3 REQUIRED)
find_package(PendarlabMavlinkHubSdk 1.0.0 REQUIRED)

find_package(gz-msgs10 REQUIRED)
set(GZ_MSGS_VER ${gz-msgs10_VERSION_MAJOR})

find_package(gz-transport13 QUIET REQUIRED OPTIONAL_COMPONENTS log)
set(GZ_TRANSPORT_VER ${gz-transport13_VERSION_MAJOR})
find_package(sdformat14 REQUIRED)

add_library(MavlinkHubGzAgent SHARED
  src/GzAgent.cpp
  src/GzAgentDefinition.cpp
  src/PeriodicSender.cpp
)
target_include_directories(MavlinkHubGzAgent PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  $<BUILD_INTERFACE:${sdformat14_INCLUDE_DIRS}>
)
target_link_libraries(MavlinkHubGzAgent
  PRIVATE pendarlab::MavlinkHubSdk
  PRIVATE gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER}
  PRIVATE gz-transport${GZ_TRANSPORT_VER}::core
  PRIVATE ${sdformat14_LIBRARIES}
)
set_target_properties(MavlinkHubGzAgent PROPERTIES OUTPUT_NAME mavlink_hub_gz_agent)

find_package(GTest CONFIG REQUIRED)

add_executable(GzAgentConfigTest
  test/gz_agent_definition_parse_config_test.cpp
)
target_include_directories(GzAgentConfigTest PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(GzAgentConfigTest
  PRIVATE MavlinkHubGzAgent
  PRIVATE pendarlab::MavlinkHubSdk
  PRIVATE GTest::gtest_main
)

add_executable(GzAgentActuatorTest
  test/gz_agent_actuator_bridging_test.cpp
)
target_include_directories(GzAgentActuatorTest PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(GzAgentActuatorTest
  PRIVATE MavlinkHubGzAgent
  PRIVATE pendarlab::MavlinkHubSdk
  PRIVATE gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER}
  PRIVATE GTest::gtest_main
)

add_executable(PeriodicSenderTest
  test/periodic_sender_test.cpp
)
target_include_directories(PeriodicSenderTest PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(PeriodicSenderTest
  PRIVATE MavlinkHubGzAgent
  PRIVATE pendarlab::MavlinkHubSdk
  PRIVATE GTest::gtest_main
)
