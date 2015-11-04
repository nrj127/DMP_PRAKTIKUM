include_directories("include")
add_subdirectory(src)

# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(DMP_Praktikum)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Declare ROS messages and services
# add_message_files(FILES Num.msg)
# add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
# generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(dmp
	main.cpp
)

#add_executable(talker src/talker.cpp)
#target_link_libraries(talker ${catkin_LIBRARIES})

#add_executable(listener src/listener.cpp)
#target_link_libraries(listener ${catkin_LIBRARIES})

## Build service client and server
# %Tag(SRVCLIENT)%
#add_executable(add_two_ints_server src/add_two_ints_server.cpp)
#target_link_libraries(add_two_ints_server ${catkin_LIBRARIES})
#add_dependencies(add_two_ints_server beginner_tutorials_gencpp)

#add_executable(add_two_ints_client src/add_two_ints_client.cpp)
#target_link_libraries(add_two_ints_client ${catkin_LIBRARIES})
#add_dependencies(add_two_ints_client beginner_tutorials_gencpp)

# %EndTag(SRVCLIENT)%

# %EndTag(FULLTEXT)%




#################################
# Beginn Affan CMakeLists.txt
#################################



#cmake_minimum_required(VERSION 2.4.6)
#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

#rosbuild_init()

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#set(INCLUDE $ENV{COTESYS_LWR_DIR}/include)
#set(INCLUDE_FRI $ENV{COTESYS_LWR_DIR}/FRILibrary/include)
#set(LIB_FRI $ENV{COTESYS_LWR_DIR}/FRILibrary/Linux/x86/release/lib)
#set(SOURCES_FRI $ENV{COTESYS_LWR_DIR}/FRILibrary/src)


#include_directories(
#    ${INCLUDE}
#    ${INCLUDE_FRI}
#)

#link_directories(
#    ${LIB_FRI}
#)

#set( sources_irml
     # Reflexxes Motion Library sources
#     ${SOURCES_FRI}/TypeIRML/TypeIRML.cpp
#     ${SOURCES_FRI}/TypeIRML/TypeIRMLDecision.cpp
#     ${SOURCES_FRI}/TypeIRML/TypeIRMLMath.cpp
#     ${SOURCES_FRI}/TypeIRML/TypeIRMLPolynomial.cpp
#     ${SOURCES_FRI}/TypeIRML/TypeIRMLProfiles.cpp
#)

#set( sources_affan
#     src/mainFRITest.cpp
#     src/RunTrajectorySimple.cpp
#)

#add_definitions( -D__LINUX__ )

#rosbuild_add_executable(FastResearchInterfaceTest ${sources_irml} ${sources_test})
#target_link_libraries(FastResearchInterfaceTest -lFastResearchInterfaceLibrary -lrt)

#rosbuild_add_executable(mainFRITest ${sources_irml} ${sources_affan})
#target_link_libraries(mainFRITest -lFastResearchInterfaceLibrary -lrt)

#rosbuild_add_executable(mainFRITest_rviz ${sources_irml} src/mainFRITest_rviz.cpp src/RunTrajectorySimple.cpp)
#target_link_libraries(mainFRITest_rviz -lFastResearchInterfaceLibrary -lrt)
