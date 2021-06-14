cmake_minimum_required (VERSION 3.3.1)
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS "/EHsc ${CMAKE_CXX_FLAGS}")

PROJECT("calibration")

# PCL
find_package(PCL REQUIRED)
# OpenCV
find_package(OpenCV REQUIRED)

MESSAGE(STATUS "Project: SERVER") 


include_directories(
${PROJECT_SOURCE_DIR}/include/
${PCL_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
)

set(SRC 
${PROJECT_SOURCE_DIR} ./src/main.cpp
${PROJECT_SOURCE_DIR} ./src/cal_extrinsic.cpp
)

add_executable(${PROJECT_NAME} ${SRC})
target_link_libraries(${PROJECT_NAME} 
${PCL_LIBRARIES}
${OpenCV_LIBRARIES}
)
