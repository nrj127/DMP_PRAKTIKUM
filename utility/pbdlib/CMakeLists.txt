cmake_minimum_required(VERSION 2.8)
PROJECT(pbdlib)

find_package(Armadillo 4.4 REQUIRED)
set(CMAKE_MAJOR_VERSION 1)
set(CMAKE_MINOR_VERSION 0)
set(CMAKE_PATCH_VERSION 0)

include_directories(include)
include_directories(${ARMADILLO_INCLUDE_DIRS})

ADD_LIBRARY(pbd
		src/datapoints.cpp
		src/demonstration.cpp
		src/tpdemonstration.cpp
		src/taskparameters.cpp
		src/mvn.cpp
		src/gmm.cpp
		src/gmr.cpp
		src/tpgmm.cpp
		src/lqr.cpp
		src/hmm.cpp
		src/hsmm.cpp
)

#Uncomment for debugging
#add_definitions(-g)

################################################################################
### samples
################################################################################
OPTION(PBDLIB_BUILD_TEST "Build PbDLib examples" 1)
IF (PBDLIB_BUILD_TEST)
  ADD_SUBDIRECTORY(examples)
ENDIF ()

target_link_libraries(pbd ${ARMADILLO_LIBRARIES})

################################################################################
### install
################################################################################
set(ROOT_INSTALL_LIBDIR lib)
set(ROOT_INSTALL_INCLUDEDIR include)
file(GLOB headers ${CMAKE_CURRENT_SOURCE_DIR}/include/pbdlib/*.h)

install(TARGETS pbd DESTINATION ${ROOT_INSTALL_LIBDIR})	
install(FILES ${headers} DESTINATION ${ROOT_INSTALL_INCLUDEDIR}/pbdlib/)

