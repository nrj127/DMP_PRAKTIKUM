CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

# external libraries
find_package(Boost REQUIRED COMPONENTS system filesystem)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${BOOST_LIBRARY_DIRS})

# application

add_executable(main main.cpp)
target_link_libraries(main ${Boost_LIBRARIES})



