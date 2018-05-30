cmake_minimum_required(VERSION 2.8.3)
project(distributed_slam_karto)

set(CMAKE_BUILD_TYPE Debug)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
    cmake_modules
    message_filters
    nav_msgs
    rosconsole
    roscpp
    sensor_msgs
    geometry_msgs
    tf
    visualization_msgs
    message_generation
)

add_message_files(
  FILES
  Edge.msg
  LaserDevice.msg
  Vertex.msg
  Link.msg
  Graph.msg
)

add_service_files(
  FILES
  SyncGraph.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  #distributed_slam_karto
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")


find_package(Eigen REQUIRED) 
find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(CSparse REQUIRED)
find_package(G2O REQUIRED)
find_package(Cholmod REQUIRED)

include_directories(
    include/
    ${catkin_INCLUDE_DIRS} 
    ${EIGEN_INCLUDE_DIRS}
    ${CHOLMOD_INCLUDE_DIR}
)

add_custom_target(
    prebuild
    COMMAND cd ${PROJECT_SOURCE_DIR}/src/utils/lua-5.3.4 &&  make -j1 linux
)

add_definitions(${EIGEN_DEFINITIONS})

catkin_package()

add_library(dsk
  src/open_karto/G2OSolver.cpp
  src/open_karto/Karto.cpp
  src/open_karto/Mapper.cpp
  src/utils/luaconf.cpp
  src/utils/ConfValue.cpp
  src/dslam.cpp
)



target_link_libraries(dsk ${catkin_LIBRARIES} ${CSPARSE_LIBRARY} ${CHOLMOD_LIBRARIES})

target_link_libraries(dsk
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_TYPES_SLAM2D}
  ${G2O_SOLVER_CHOLMOD}
  ${G2O_SOLVER_CSPARSE}
  ${G2O_SOLVER_CSPARSE_EXTENSION}
  ${PROJECT_SOURCE_DIR}/src/utils/lua-5.3.4/liblua.a
)

add_executable(d_slam_karto
  src/dslam_karto.cpp
)
add_dependencies(d_slam_karto prebuild dsk)
target_link_libraries(d_slam_karto dsk)


add_executable(graph_sync
  src/graph_sync.cpp
)

target_link_libraries(graph_sync ${catkin_LIBRARIES} )


add_executable(2laser_slam_karto
  src/dslam_karto_2_laser.cpp
)
add_dependencies(2laser_slam_karto prebuild dsk)
target_link_libraries(2laser_slam_karto dsk)

#install(TARGETS slam_karto_g2o
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
