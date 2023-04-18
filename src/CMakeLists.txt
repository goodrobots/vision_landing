cmake_minimum_required (VERSION 2.8)
project (track_targets)
find_package(aruco REQUIRED )
find_package(OpenCV REQUIRED)

option(USE_TIMERS "Set to OFF to disable timers" OFF)
iF(USE_TIMERS)
add_definitions(-DUSE_TIMERS)
ENDIF()

add_executable(track_targets track_targets.cpp apriltag.cpp raw-tcp-video.cpp)

set(EXTRA_C_FLAGS_RELEASE "${EXTRA_C_FLAGS_RELEASE} -std=c++0x -pthread -march=armv8-a+crc -mfpu=neon-vfpv4 -mtune=cortex-a53 -ftree-vectorize -mfloat-abi=hard -O3 ")
set(cpp_compile_flags "-std=gnu++11 -pthread")
add_definitions(${cpp_compile_flags})

include_directories(${aruco_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(track_targets aruco ${aruco_LIBS} ${OpenCV_LIBS} pthread)
link_directories(${aruco_LIB_DIR} ${OpenCV_INSTALL_PATH}/lib})
link_libraries(pthread)
install(PROGRAMS track_targets DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/..)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -latomic -lapriltag")
