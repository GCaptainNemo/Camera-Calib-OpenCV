cmake_minimum_required(VERSION 2.8)

project(camera_calib)

find_package(OpenCV REQUIRED)

INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
include_directories("C:/opencv/opencv/build/install/include")
include_directories("C:/opencv/opencv/build/install/include/opencv2")
include_directories("include")

add_executable(circle_board_calib src/circle_board.cpp src/blob_detector.cpp)
add_executable(chessboard_calib src/chessboard.cpp)
add_executable(stereo_calib src/stereo_calib.cpp src/blob_detector.cpp)

message(STATUS "OPENCV_DIR = ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OPENCV_LIBS = ${OpenCV_LIBRARIES}")

target_link_libraries(circle_board_calib ${OpenCV_LIBRARIES})
target_link_libraries(chessboard_calib ${OpenCV_LIBRARIES})
target_link_libraries(stereo_calib ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )

