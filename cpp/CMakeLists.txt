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

## Find dependencies
INCLUDE_DIRECTORIES(include)

# Eigen
FIND_PACKAGE(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

# Ompl
FIND_PACKAGE(ompl REQUIRED)
INCLUDE_DIRECTORIES(${OMPL_INCLUDE_DIRS})

## Boost
FIND_PACKAGE(Boost REQUIRED COMPONENTS system filesystem program_options)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})


## Executable
add_executable(testCForestCircleGridBenchmark src/CForestCircleGridBenchmark.cpp)
target_link_libraries(testCForestCircleGridBenchmark ${OMPL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(testConstrainedPlanningImplicitChain src/ConstrainedPlanningImplicitChain.cpp)
target_link_libraries(testConstrainedPlanningImplicitChain ${OMPL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(testConstrainedPlanningImplicitParallel src/ConstrainedPlanningImplicitParallel.cpp)
target_link_libraries(testConstrainedPlanningImplicitParallel ${OMPL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(testConstrainedPlanningKinematicChain src/ConstrainedPlanningKinematicChain.cpp)
target_link_libraries(testConstrainedPlanningKinematicChain ${OMPL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(testConstrainedPlanningSphere src/ConstrainedPlanningSphere.cpp)
target_link_libraries(testConstrainedPlanningSphere ${OMPL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(testConstrainedPlanningTorus src/ConstrainedPlanningTorus.cpp)
target_link_libraries(testConstrainedPlanningTorus ${OMPL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(testGeometricCarPlanning src/GeometricCarPlanning.cpp)
target_link_libraries(testGeometricCarPlanning ${OMPL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(testHybridSystemPlanning src/HybridSystemPlanning.cpp)
target_link_libraries(testHybridSystemPlanning ${OMPL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(testHypercubeBenchmark src/HypercubeBenchmark.cpp)
target_link_libraries(testHypercubeBenchmark ${OMPL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(testKinematicChainBenchmark src/KinematicChainBenchmark.cpp)
target_link_libraries(testKinematicChainBenchmark ${OMPL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(testLTLWithTriangulation src/LTLWithTriangulation.cpp src/TriangularDecomposition.cpp src/PropositionalTriangularDecomposition.cpp src/triangle.c)
target_compile_definitions(testLTLWithTriangulation PRIVATE TRILIBRARY)
target_link_libraries(testLTLWithTriangulation ${OMPL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(testOptimalPlanning src/OptimalPlanning.cpp)
target_link_libraries(testOptimalPlanning ${OMPL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(testPoint2DPlanning src/Point2DPlanning.cpp)
target_link_libraries(testPoint2DPlanning ${OMPL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(testRigidBodyPlanning src/RigidBodyPlanning.cpp)
target_link_libraries(testRigidBodyPlanning ${OMPL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(testRigidBodyPlanningWithControls src/RigidBodyPlanningWithControls.cpp)
target_link_libraries(testRigidBodyPlanningWithControls ${OMPL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(testRigidBodyPlanningWithIK src/RigidBodyPlanningWithIK.cpp)
target_link_libraries(testRigidBodyPlanningWithIK ${OMPL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(testRigidBodyPlanningWithIntegrationAndControls src/RigidBodyPlanningWithIntegrationAndControls.cpp)
target_link_libraries(testRigidBodyPlanningWithIntegrationAndControls ${OMPL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(testRigidBodyPlanningWithODESolverAndControls src/RigidBodyPlanningWithODESolverAndControls.cpp)
target_link_libraries(testRigidBodyPlanningWithODESolverAndControls ${OMPL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(testStateSampling src/StateSampling.cpp)
target_link_libraries(testStateSampling ${OMPL_LIBRARIES} ${Boost_LIBRARIES})

## Install
install(TARGETS  #Scripts that need fixing 
  testCForestCircleGridBenchmark
  testConstrainedPlanningImplicitChain
  testConstrainedPlanningImplicitParallel
  testConstrainedPlanningKinematicChain
  testConstrainedPlanningSphere
  testConstrainedPlanningTorus
  testGeometricCarPlanning
  testHybridSystemPlanning
  testHypercubeBenchmark
  testKinematicChainBenchmark
  testOptimalPlanning 
  testPoint2DPlanning
  testRigidBodyPlanning 
  testRigidBodyPlanningWithControls
  testRigidBodyPlanningWithIK
  testRigidBodyPlanningWithIntegrationAndControls
  testRigidBodyPlanningWithODESolverAndControls
  testStateSampling 
  DESTINATION lib/${PROJECT_NAME})