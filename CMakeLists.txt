cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
project(srv_mav_behaviours LANGUAGES CXX)

# Setting build type to Release
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
message(STATUS "Build type: " ${CMAKE_BUILD_TYPE})

# Setting the flags for profiling information or not
if(CMAKE_BUILD_TYPE MATCHES Release)
  message(STATUS "Setting Release options")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3 -march=native")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native")
elseif(CMAKE_BUILD_TYPE MATCHES Debug)
  message(STATUS "Setting Debug options")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O1 -pg -march=native")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O1 -pg -march=native")
endif()

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support.
                         Please use a different C++ compiler.")
endif()

# Printing the compiling flags
message(STATUS "Compiler flags: ${CMAKE_CXX_FLAGS}")

# Catkin dependencies
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs geometry_msgs nav_msgs tf srv_mav_msgs srv_mav_control dynamic_reconfigure message_generation)

# Other packages
find_package(OpenCV REQUIRED) # OpenCV
find_package(Boost REQUIRED COMPONENTS thread)
find_package(OpenMP REQUIRED) # OpenMP
if (OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

#Add dynamic reconfigure api
generate_dynamic_reconfigure_options(
  cfg/safety_manager.cfg
  cfg/mission_manager.cfg
)

# Generate services in the 'srv' folder
add_service_files(
  FILES
  RequestControl.srv
  GiveUpControl.srv
  StartSweep.srv
  StartVerticalInspection.srv
  GoToPoint.srv
  SavePoint.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

# Defining the package
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES safety_manager
    CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs nav_msgs tf srv_mav_msgs srv_mav_control dynamic_reconfigure message_runtime
    DEPENDS OpenCV Boost
)

include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    )

### Targets ###

# --------------Safety Manager----------------
# Library
add_library(safety_manager
            src/safety_manager.cc)
target_link_libraries(safety_manager
                      ${catkin_LIBRARIES}
                      ${OpenCV_LIBRARIES}
                      ${Boost_LIBRARIES}
                      )
add_dependencies(safety_manager
                      ${catkin_EXPORTED_TARGETS}
                      ${OpenCV_EXPORTED_TARGETS}
                      ${Boost_EXPORTED_TARGETS}
                      srv_mav_behaviours_generate_messages_cpp)

# Node
add_executable(safety_manager_node
               src/safety_manager_node.cc)
target_link_libraries(safety_manager_node
                      safety_manager
                      ${catkin_LIBRARIES}
                      ${OpenCV_LIBRARIES}
                      ${Boost_LIBRARIES}
                      )
add_dependencies(safety_manager_node 
                      ${PROJECT_NAME}_gencfg) 

# --------------Mission Manager----------------
# Library
add_library(mission_manager
            src/mission_manager.cc)
target_link_libraries(mission_manager
                      ${catkin_LIBRARIES}
                      ${OpenCV_LIBRARIES}
                      ${Boost_LIBRARIES}
                      )
add_dependencies(mission_manager
                      ${catkin_EXPORTED_TARGETS}
                      ${OpenCV_EXPORTED_TARGETS}
                      ${Boost_EXPORTED_TARGETS}
                      srv_mav_behaviours_generate_messages_cpp)

# Node
add_executable(mission_manager_node
               src/mission_manager_node.cc)
target_link_libraries(mission_manager_node
                      mission_manager
                      ${catkin_LIBRARIES}
                      ${OpenCV_LIBRARIES}
                      ${Boost_LIBRARIES}
                      )
add_dependencies(mission_manager_node 
                      ${PROJECT_NAME}_gencfg) 

