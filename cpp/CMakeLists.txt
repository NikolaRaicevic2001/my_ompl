CMAKE_MINIMUM_REQUIRED(VERSION 3.1)
PROJECT(my_ompl)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-deprecated-declarations")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Find dependencies
INCLUDE_DIRECTORIES(include)

# Eigen
FIND_PACKAGE(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

# Ompl
FIND_PACKAGE(ompl REQUIRED)
INCLUDE_DIRECTORIES(${OMPL_INCLUDE_DIRS})
#include "ompl/base/spaces/SE3StateSpace.h"

## Boost
FIND_PACKAGE(Boost REQUIRED COMPONENTS system filesystem program_options)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
#FIND_PACKAGE(Boost REQUIRED COMPONENTS thread system filesystem program_options date_time timer chrono serialization unit_test_framework)


## CGAL
#find_package(CGAL REQUIRED COMPONENTS Core)
#include(${CGAL_USE_FILE})
#INCLUDE_DIRECTORIES(${CGAL_INCLUDE_DIRS})


# Executable
add_executable(testRigidBodyPlanning src/RigidBodyPlanning.cpp)
target_link_libraries(testRigidBodyPlanning ${OMPL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(testStateSampling src/StateSampling.cpp)
target_link_libraries(testStateSampling ${OMPL_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS testRigidBodyPlanning testStateSampling
  DESTINATION lib/${PROJECT_NAME})


