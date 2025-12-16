cmake_minimum_required(VERSION 2.8)
project(open_cap)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(open_cap src/open_cap.cpp)
target_link_libraries(open_cap ${OpenCV_LIBS})


add_executable(01test src/01test.cpp)
target_link_libraries(01test ${OpenCV_LIBS})


add_executable(recognition src/recognition.cpp)
target_link_libraries(recognition ${OpenCV_LIBS})

add_executable(get_hsv src/get_hsv.cpp)
target_link_libraries(get_hsv ${OpenCV_LIBS})