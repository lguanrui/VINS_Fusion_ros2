cmake_minimum_required(VERSION 3.5)
project(global_fusion)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_BUILD_TYPE Release)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(Ceres QUIET)
if(NOT Ceres_FOUND)
  set(VINS_FUSION_CERES_HINT "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/ceres-install")
  if(EXISTS "${VINS_FUSION_CERES_HINT}/lib/cmake/Ceres/CeresConfig.cmake")
    list(PREPEND CMAKE_PREFIX_PATH "${VINS_FUSION_CERES_HINT}")
    find_package(Ceres QUIET)
  endif()
endif()
if(NOT Ceres_FOUND)
  message(FATAL_ERROR
    "Ceres was not found. Build the bundled Ceres with: ./scripts/build_ceres.sh")
endif()

add_subdirectory(./ThirdParty/GeographicLib/)

include_directories(
  # ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ./ThirdParty/GeographicLib/include/
)




add_executable(global_fusion_node 
	src/globalOptNode.cpp
	src/globalOpt.cpp)

# added from DEBUG
ament_target_dependencies(global_fusion_node rclcpp nav_msgs geometry_msgs sensor_msgs visualization_msgs rclpy)

target_link_libraries(global_fusion_node ${CERES_LIBRARIES} libGeographiccc) # ${catkin_LIBRARIES})


# Install
install(TARGETS
    global_fusion_node
    DESTINATION lib/${PROJECT_NAME}
)


install(TARGETS global_fusion_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(TARGETS global_fusion_node
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)



ament_package()
