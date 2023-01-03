cmake_minimum_required(VERSION 3.5)
project(example_one)

find_package(PCL 1.8 REQUIRED COMPONENTS common io)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(CMAKE_CXX_STANDARD 11)

add_executable(example_one main.cpp)
target_link_libraries(example_one 
${PCL_COMMON_LIBRARIES} 
${PCL_IO_LIBRARIES}
pcl_visualization
)

