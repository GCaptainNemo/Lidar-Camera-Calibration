cmake_minimum_required (VERSION 3.3.1)
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS "/EHsc ${CMAKE_CXX_FLAGS}")

PROJECT("calibration")

# PCL
find_package(PCL REQUIRED)
# OpenCV
find_package(OpenCV REQUIRED)

include_directories(
${PCL_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
)

file(GLOB_RECURSE DIR_SRCS    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB_RECURSE DIR_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h)


add_executable(${PROJECT_NAME} 
${DIR_SRCS}    
${DIR_HEADERS}
)

target_link_libraries(${PROJECT_NAME} 
${PCL_LIBRARIES}
${OpenCV_LIBRARIES}
)
