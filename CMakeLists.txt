cmake_minimum_required(VERSION 2.8.3)
project(mav_localization)

# -----------------------------------------------
# USER DEFINED VARIABLES
# -----------------------------------------------
option(BUILD_EXECUTABLE "Build as executable to test functionality"                     ON)
option(WITH_OPENMP      "Build with OpenMP support for multithreading"                  ON)

find_package(OpenMP)
find_package(catkin REQUIRED COMPONENTS 
roscpp pcl_conversions pcl_ros message_filters sensor_msgs cmake_modules  octomap_msgs)
find_package(Boost REQUIRED COMPONENTS random)
find_package(Eigen REQUIRED)
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})

message(STATUS "OCTOMAP_INCLUDE_DIRS = ${OCTOMAP_INCLUDE_DIRS}")
message(STATUS "OCTOMAP_LIBRARIES = ${OCTOMAP_LIBRARIES}")

find_package(dynamicEDT3D)
if (${dynamicEDT3D_FOUND})                                                                       
  include_directories(${DYNAMICEDT3D_INCLUDE_DIRS})
  message(STATUS "DYNAMICEDT3D_INCLUDE_DIRS = ${DYNAMICEDT3D_INCLUDE_DIRS}")
  link_directories(${DYNAMICEDT3D_LIBRARY_DIRS})
  list(APPEND LIB_FILES src/EndpointModel.cpp)                                      
else (${dynamicEDT3D_FOUND})
  MESSAGE(WARNING "dynamicEDT3D library (part of OctoMap >1.5) not found, skipping endpoint model")
  add_definitions(-DSKIP_ENDPOINT_MODEL)
endif (${dynamicEDT3D_FOUND})


# -----------------------------------------------
# COMPILER FLAGS
# -----------------------------------------------
# add OpenMP support
if (WITH_OPENMP)
    if (OPENMP_FOUND)
      SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
      SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
      SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
    endif()

    if (NOT OPENMP_FOUND)
        set(WITH_OPENMP OFF)
    endif()
endif()


# set the build type: "Choose the type of build. Options are: None, Debug, Release, RelWithDebInfo, MinSizeRel"
set(CMAKE_BUILD_TYPE Release)


# add vectorization support
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse4.1")
# set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -msse4.1")

# use highest level of optimization in Release mode
if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
    set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -O3")
endif ()



catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mapmodel motionmodel observationmodel raycastingmodel observationmodel mavlocalization
  CATKIN_DEPENDS tf
  DEPENDS octomap OpenMP
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

if (${dynamicEDT3D_FOUND})
  add_library(endpointmodel src/EndpointModel.cpp)
endif (${dynamicEDT3D_FOUND})

add_library(mapmodel src/MapModel.cpp)
add_library(motionmodel src/MotionModel.cpp)
add_library(observationmodel src/ObservationModel.cpp)

add_library(raycastingmodel src/RaycastingModel.cpp)
target_link_libraries(raycastingmodel observationmodel ${catkin_LIBRARIES})

add_library(mavlocalization src/MAVLocalization.cpp)
target_link_libraries(mavlocalization mapmodel motionmodel observationmodel raycastingmodel endpointmodel ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} ${DYNAMICEDT3D_LIBRARIES} )

add_executable(localization_node src/Localization_node.cpp)
target_link_libraries(localization_node mavlocalization ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} ${DYNAMICEDT3D_LIBRARIES})

# install
install(TARGETS mapmodel motionmodel observationmodel raycastingmodel mavlocalization
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
) 
install(TARGETS localization_node
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# -----------------------------------------------
# BUILD SUMARY
# -----------------------------------------------
message("")
message("---------------------------------------------")
message(${PROJECT_NAME} " Configuration Summary")
message("")
message("Build type:                    ${CMAKE_BUILD_TYPE}")
message("Build with threading (OpenMP): ${WITH_OPENMP}")
message("Build as executable:           ${BUILD_EXECUTABLE}")
message("---------------------------------------------")
message("")
