cmake_minimum_required(VERSION 2.8...3.14)
project(mujoco_sim)

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

# ##################################
# # catkin specific configuration ##
# ##################################
# # The catkin_package macro generates cmake config files for your package
# # Declare things to be passed to dependent projects
# # INCLUDE_DIRS: uncomment this if your package contains header files
# # LIBRARIES: libraries you create in this project that dependent projects also need
# # CATKIN_DEPENDS: catkin_packages dependent projects also need
# # DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(
	CATKIN_DEPENDS roslib
)

# ##########
# # Build ##
# ##########
set(MULTIVERSE_DIR ${CATKIN_DEVEL_PREFIX}/../../../../multiverse)

# # Specify additional locations of header files
# # Your package locations should be listed before other locations
include_directories(
	${catkin_INCLUDE_DIRS}
	${MULTIVERSE_DIR}/external/mujoco/include
	${MULTIVERSE_DIR}/src/multiverse_client/include
	${PROJECT_SOURCE_DIR}/include/mujoco_sim
)

set(MUJOCO_SIM_HEADLESS mujoco_sim_headless)
add_executable(${MUJOCO_SIM_HEADLESS}_node src/mujoco_sim_headless.cpp)
add_dependencies(${MUJOCO_SIM_HEADLESS}_node ${catkin_EXPORTED_TARGETS})
add_library(${MUJOCO_SIM_HEADLESS}
	${PROJECT_SOURCE_DIR}/src/mujoco_sim/mj_hw_interface.cpp
	${PROJECT_SOURCE_DIR}/src/mujoco_sim/mj_ros.cpp
	${PROJECT_SOURCE_DIR}/src/mujoco_sim/mj_model.cpp
	${PROJECT_SOURCE_DIR}/src/mujoco_sim/mj_sim.cpp
	${PROJECT_SOURCE_DIR}/src/mujoco_sim/mj_multiverse_client.cpp
)
target_link_libraries(${MUJOCO_SIM_HEADLESS}_node
	${catkin_LIBRARIES}
	${MUJOCO_SIM_HEADLESS}
	$ENV{ROS_ROOT}/../../lib/liburdf.so
	mujoco
	tinyxml2
	jsoncpp
	zmq
	${MULTIVERSE_DIR}/lib/libstdc++/libmultiverse_client_json_libstdc++.so
)

set(MUJOCO_SIM mujoco_sim)
add_executable(${MUJOCO_SIM}_node src/mujoco_sim.cpp)
add_dependencies(${MUJOCO_SIM}_node ${catkin_EXPORTED_TARGETS})
add_library(${MUJOCO_SIM}
	${PROJECT_SOURCE_DIR}/src/mujoco_sim/mj_visual.cpp
)
target_link_libraries(${MUJOCO_SIM}_node
	${catkin_LIBRARIES}
	${MUJOCO_SIM}
	${MUJOCO_SIM_HEADLESS}
	$ENV{ROS_ROOT}/../../lib/liburdf.so
	mujoco
	tinyxml2
	jsoncpp
	glfw
	GL
	zmq
	${MULTIVERSE_DIR}/lib/libstdc++/libmultiverse_client_json_libstdc++.so
)