cmake_minimum_required (VERSION 3.3.1)
set(CMAKE_CXX_FLAGS "-std=c++11")

PROJECT("calibration")

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)


# include_directories("/usr/include/eigen3")
include_directories(
${PCL_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
include
)

# file(GLOB_RECURSE DIR_SRCS    ${CMAKE_CURRENT_SOURCE_DIR}/src/*)
# file(GLOB_RECURSE DIR_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h)

add_executable(${PROJECT_NAME} 
src/cal_extrinsic.cpp
src/main.cpp    
)


target_link_libraries(${PROJECT_NAME} 
${PCL_LIBRARIES}
${OpenCV_LIBRARIES}
#lidar_visualize
)

add_executable(ceres_PnP src/ceres_PnP.cpp)
target_link_libraries(ceres_PnP ${CERES_LIBRARIES})

