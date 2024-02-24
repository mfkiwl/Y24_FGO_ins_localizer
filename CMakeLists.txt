cmake_minimum_required(VERSION 3.1)
project(ins_localizer)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
add_compile_options(-std=c++17)

find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Iridescence REQUIRED)
find_package(PCL REQUIRED)

add_library(ins_localizer SHARED
  src/ins_localizer/ins_localizer_core.cpp
  src/ins_localizer/viewer.cpp
)
target_include_directories(ins_localizer PUBLIC
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${Iridescence_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(ins_localizer
  gtsam
  ${YAML_CPP_LIBRARIES}
  ${Iridescence_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(offline_ins_localizer
  src/sample/offline_ins_localizer.cpp
)
target_link_libraries(offline_ins_localizer
  ins_localizer
)
