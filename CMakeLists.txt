cmake_minimum_required (VERSION 3.3.1)
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS "/EHsc ${CMAKE_CXX_FLAGS}")

PROJECT("calibration")

# PCL
find_package(PCL REQUIRED)


MESSAGE(STATUS "Project: SERVER") 


include_directories(
${PCL_INCLUDE_DIRS}
)

set(SRC 
${PROJECT_SOURCE_DIR} ./src/main.cpp
)

add_executable(${PROJECT_NAME} ${SRC})
target_link_libraries(${PROJECT_NAME} 
${PCL_LIBRARIES}
)
