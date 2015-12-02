cmake_minimum_required(VERSION 2.4.6)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

project(testproject)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#find required packages:
find_package(Armadillo 4.4 REQUIRED)
#set(Matlab_ROOT_DIR /usr/local/MATLAB/R2015b)
#find_package(Matlab)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

set(INCLUDE_FRI utility/FRILibrary/include)
set(LIB_FRI utility/FRILibrary/Linux/x64/release/lib)
set(SOURCES_FRI utility/FRILibrary/src)
set(MATLAB_INCLUDE_DIR /usr/local/MATLAB/R2015b/extern/include)
set(MATLAB_LIBRARIES /usr/local/MATLAB/R2015b/bin/glnxa64)


include_directories(
    include
    ${INCLUDE_FRI}
    ${ARMADILLO_INCLUDE_DIRS}
    ${MATLAB_INCLUDE_DIR}
)


link_directories(
    ${LIB_FRI}
    ${MATLAB_LIBRARIES}
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
     src/RunTrajectorySimple.cpp
)

set( sources_frederik
     src/dmp_integrator.cpp
)

set( sources_thomas
      src/onlinegmr.cpp
      src/camerahandle.cpp
      src/utility.cpp
)


add_definitions( -D__LINUX__ )

#rosbuild_add_executable(FastResearchInterfaceTest ${sources_irml} ${sources_test})
#target_link_libraries(FastResearchInterfaceTest -lFastResearchInterfaceLibrary -lrt)

rosbuild_add_executable(mainFRITest ${sources_irml} ${sources_affan})
target_link_libraries(mainFRITest -lFastResearchInterfaceLibrary -lrt)

rosbuild_add_executable(mainFRITest_rviz ${sources_irml} src/mainFRITest_rviz.cpp src/RunTrajectorySimple.cpp)
target_link_libraries(mainFRITest_rviz -lFastResearchInterfaceLibrary -lrt)

rosbuild_add_executable(test_dmp  src/test_dmp_integrator  ${sources_thomas} ${sources_frederik})
target_link_libraries(test_dmp libmat.so libmx.so -larmadillo -lrt)

rosbuild_add_executable(maindmpFRI src/maindmpFRI.cpp ${sources_thomas} ${sources_frederik} ${sources_irml} ${sources_affan})


target_link_libraries(maindmpFRI
    -lFastResearchInterfaceLibrary
    -lrt
    libmat.so # matlab library
    libmx.so # matlab library
    -larmadillo
)

rosbuild_add_executable(mainFRITest  src/mainFRITest.cpp ${sources_irml} ${sources_affan})
target_link_libraries(mainFRITest -lFastResearchInterfaceLibrary -lrt)


# TODO: TE: how to link which target to which library???
#target_link_libraries(pbd ${ARMADILLO_LIBRARIES})
#target_link_libraries(pbd ${Matlab_LIBRARIES})
add_executable(mainOnlineGmrTest src/mainOnlineGmrTest.cpp ${sources_thomas})
target_link_libraries(mainOnlineGmrTest libmat.so libmx.so -larmadillo)
# other matlab libs: libmx.lib libmatlb.lib libmmfile.lib


