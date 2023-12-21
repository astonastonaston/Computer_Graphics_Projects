cmake_minimum_required(VERSION 2.8)
project( hw2 )

# include OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# include .hpps
include_directories(include)

# include src .cpps
file(GLOB SOURCES "src/*.cpp")

# compiling and linking
add_executable( main ${SOURCES} )
target_link_libraries( main ${OpenCV_LIBS} )