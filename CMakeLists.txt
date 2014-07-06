cmake_minimum_required(VERSION 2.8.3)
project(ndt_mapper)

find_package(catkin REQUIRED
  roscpp
  geometry_utils
  parameter_utils
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    geometry_utils
    parameter_utils
)

include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})
add_definitions(${catkin_DEFINITIONS})

add_library(${PROJECT_NAME}
   src/NDTMapper.cc
   src/NDTMap.cc
   src/GaussianVoxel.cc
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}.cc)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
