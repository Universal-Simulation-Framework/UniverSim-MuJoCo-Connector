cmake_minimum_required(VERSION 2.8...3.14)
project(mujoco_sim)

# # Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

# # Find catkin macros and libraries
# # if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# # is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	mujoco_msgs
	roslib
	controller_manager
	urdf
	multiverse_client
)

find_package(Doxygen)

if(DOXYGEN_FOUND)
	# set output file
	set(DOXYGEN_OUT ${PROJECT_SOURCE_DIR}/docs/Doxyfile)

	# note the option ALL which allows to build the docs together with the application
	add_custom_target(doc_doxygen ALL
		COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
		WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/docs
		COMMENT "Generating API documentation with Doxygen"
		VERBATIM)
else(DOXYGEN_FOUND)
	message(VERBOSE "Doxygen need to be installed to generate the doxygen documentation")
endif(DOXYGEN_FOUND)

set(MUJOCO_BUILD_EXAMPLES OFF)
set(MUJOCO_BUILD_SIMULATE OFF)
set(MUJOCO_BUILD_TESTS OFF)
set(MUJOCO_TEST_PYTHON_UTIL OFF)
set(MUJOCO_SOURCE_DIR ${PROJECT_SOURCE_DIR}/../../../../../src/mujoco)
add_subdirectory(${MUJOCO_SOURCE_DIR} build)

# ##################################
# # catkin specific configuration ##
# ##################################
# # The catkin_package macro generates cmake config files for your package
# # Declare things to be passed to dependent projects
# # INCLUDE_DIRS: uncomment this if your package contains header files
# # LIBRARIES: libraries you create in this project that dependent projects also need
# # CATKIN_DEPENDS: catkin_packages dependent projects also need
# # DEPENDS: system dependencies of this project that dependent projects also need

set(mujoco_INCLUDE_DIRS ${MUJOCO_SOURCE_DIR}/include)
set(mujoco_LIBRARIES ${PROJECT_SOURCE_DIR}/../../../../build/mujoco_sim/lib/libmujoco.so)
catkin_package(
	INCLUDE_DIRS ${mujoco_INCLUDE_DIRS}

	# LIBRARIES
	CATKIN_DEPENDS roscpp rospy mujoco_msgs roslib

	DEPENDS mujoco
)

# ##########
# # Build ##
# ##########

# # Specify additional locations of header files
# # Your package locations should be listed before other locations
include_directories(
	${catkin_INCLUDE_DIRS}
	${PROJECT_SOURCE_DIR}/include/mujoco_sim
	${mujoco_INCLUDE_DIRS}
)

set(MUJOCO_SIM_HEADLESS_NODE mujoco_sim_headless_node)
add_executable(${MUJOCO_SIM_HEADLESS_NODE} src/mujoco_sim_headless.cpp)
add_library(${MUJOCO_SIM_HEADLESS_NODE}_lib
	${PROJECT_SOURCE_DIR}/src/mujoco_sim/mj_hw_interface.cpp
	${PROJECT_SOURCE_DIR}/src/mujoco_sim/mj_ros.cpp
	${PROJECT_SOURCE_DIR}/src/mujoco_sim/mj_model.cpp
	${PROJECT_SOURCE_DIR}/src/mujoco_sim/mj_sim.cpp
	${PROJECT_SOURCE_DIR}/src/mujoco_sim/mj_multiverse_client.cpp
)
target_link_libraries(${MUJOCO_SIM_HEADLESS_NODE}
	${catkin_LIBRARIES}
	${MUJOCO_SIM_HEADLESS_NODE}_lib
	$ENV{ROS_ROOT}/../../lib/liburdf.so
	mujoco
	tinyxml2
	jsoncpp
)

set(MUJOCO_SIM_NODE mujoco_sim_node)
add_executable(${MUJOCO_SIM_NODE} src/mujoco_sim.cpp)
add_library(${MUJOCO_SIM_NODE}_lib
	${PROJECT_SOURCE_DIR}/src/mujoco_sim/mj_visual.cpp
)
target_link_libraries(${MUJOCO_SIM_NODE}
	${catkin_LIBRARIES}
	${MUJOCO_SIM_HEADLESS_NODE}_lib
	${MUJOCO_SIM_NODE}_lib
	$ENV{ROS_ROOT}/../../lib/liburdf.so
	mujoco
	tinyxml2
	jsoncpp
	glfw
	GL
)