cmake_minimum_required (VERSION 3.3.1)
set(CMAKE_CXX_FLAGS "-std=c++11")

PROJECT("calibration")

# PCL
find_package(PCL REQUIRED)
# OpenCV
find_package(OpenCV REQUIRED)

include_directories(
${PCL_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
)

#add_library(lidar_visualize STATIC src/lidar_visualize.hpp)
#set_target_properties(lidar_visualize  PROPERTIES LINKER_LANGUAGE C++)


file(GLOB_RECURSE DIR_SRCS    ${CMAKE_CURRENT_SOURCE_DIR}/src/*)
file(GLOB_RECURSE DIR_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h)


add_executable(${PROJECT_NAME} 
${DIR_SRCS}    
${DIR_HEADERS}
)


target_link_libraries(${PROJECT_NAME} 
${PCL_LIBRARIES}
${OpenCV_LIBRARIES}
#lidar_visualize
)
