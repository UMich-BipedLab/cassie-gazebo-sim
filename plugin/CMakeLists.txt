cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

# Eigen 
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
message("EIGEN3_INCLUDE_DIR: " ${EIGEN3_INCLUDE_DIR})

# Adding all files
file(GLOB src_files
    "${PROJECT_SOURCE_DIR}/../../../matlab/UDP/gen/*.c*",
    "${PROJECT_SOURCE_DIR}/../../../matlab/Cassie_Model/gen/dyn/eigen/src/De_cassie.cpp",
    "${PROJECT_SOURCE_DIR}/../../../matlab/Cassie_Model/gen/dyn/eigen/src/Ce_cassie.cpp",
    "${PROJECT_SOURCE_DIR}/../../../matlab/Cassie_Model/gen/dyn/eigen/src/Ge_cassie.cpp"
)
include_directories(
    "${PROJECT_SOURCE_DIR}/../../../matlab/UDP/gen/",
    "${PROJECT_SOURCE_DIR}/../../../matlab/Cassie_Model/gen/dyn/eigen/include/",
    "${PROJECT_SOURCE_DIR}/../../../matlab/Cassie_Model/gen/kin/eigen/include/",
)

add_library(cassie_plugin SHARED src/CassiePlugin.cpp src/udp.c ${src_files})
include_directories(include)
target_link_libraries(cassie_plugin ${GAZEBO_LIBRARIES} ${CMAKE_SOURCE_DIR}/libagilitycassie.a)

#add_executable(cassiectrl example/cassiectrl.c)
#target_link_libraries(cassiectrl cassie_plugin)
