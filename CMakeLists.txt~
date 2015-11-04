cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

set(INCLUDE_FRI /home/tomelloso/dmp_praktikum/lib/FRILibrary/include)
set(LIB_FRI /home/tomelloso/dmp_praktikum/lib/FRILibrary/Linux/x64/release/lib)
set(SOURCES_FRI /home/tomelloso/dmp_praktikum/lib/FRILibrary/src)


include_directories(
    ${INCLUDE_FRI}
)

link_directories(
    ${LIB_FRI}
)

set( sources_irml
     # Reflexxes Motion Library sources
     ${SOURCES_FRI}/TypeIRML/TypeIRML.cpp
     ${SOURCES_FRI}/TypeIRML/TypeIRMLDecision.cpp
     ${SOURCES_FRI}/TypeIRML/TypeIRMLMath.cpp
     ${SOURCES_FRI}/TypeIRML/TypeIRMLPolynomial.cpp
     ${SOURCES_FRI}/TypeIRML/TypeIRMLProfiles.cpp
)

set( sources_affan
     src/mainFRITest.cpp
     src/RunTrajectorySimple.cpp
)

add_definitions( -D__LINUX__ )

#rosbuild_add_executable(FastResearchInterfaceTest ${sources_irml} ${sources_test})
#target_link_libraries(FastResearchInterfaceTest -lFastResearchInterfaceLibrary -lrt)

rosbuild_add_executable(mainFRITest ${sources_irml} ${sources_affan})
target_link_libraries(mainFRITest -lFastResearchInterfaceLibrary -lrt)

rosbuild_add_executable(mainFRITest_rviz ${sources_irml} src/mainFRITest_rviz.cpp src/RunTrajectorySimple.cpp)
target_link_libraries(mainFRITest_rviz -lFastResearchInterfaceLibrary -lrt)


