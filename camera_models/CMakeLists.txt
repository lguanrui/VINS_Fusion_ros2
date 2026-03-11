cmake_minimum_required(VERSION 3.5)
project(camera_models)


set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wextra -Wpedantic)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
include_directories(${Boost_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
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
include_directories(${CERES_INCLUDE_DIRS})


ament_export_include_directories(include)
ament_export_libraries(camera_models)
ament_export_dependencies(std_msgs)

# include_directories(
#     ${catkin_INCLUDE_DIRS}
#     )

include_directories("include")

add_executable(Calibrations 
    src/intrinsic_calib.cc
    src/chessboard/Chessboard.cc
    src/calib/CameraCalibration.cc
    src/camera_models/Camera.cc
    src/camera_models/CameraFactory.cc
    src/camera_models/CostFunctionFactory.cc
    src/camera_models/PinholeCamera.cc
    src/camera_models/PinholeFullCamera.cc
    src/camera_models/CataCamera.cc
    src/camera_models/EquidistantCamera.cc
    src/camera_models/ScaramuzzaCamera.cc
    src/sparse_graph/Transform.cc
    src/gpl/gpl.cc
    src/gpl/EigenQuaternionParameterization.cc)


# added from DEBUG
ament_target_dependencies(Calibrations rclcpp std_msgs OpenCV)

add_library(camera_models
    src/chessboard/Chessboard.cc
    src/calib/CameraCalibration.cc
    src/camera_models/Camera.cc
    src/camera_models/CameraFactory.cc
    src/camera_models/CostFunctionFactory.cc
    src/camera_models/PinholeCamera.cc
    src/camera_models/PinholeFullCamera.cc
    src/camera_models/CataCamera.cc
    src/camera_models/EquidistantCamera.cc
    src/camera_models/ScaramuzzaCamera.cc
    src/sparse_graph/Transform.cc
    src/gpl/gpl.cc
    src/gpl/EigenQuaternionParameterization.cc)

target_link_libraries(Calibrations ${Boost_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})
target_link_libraries(camera_models ${Boost_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})


# Install
install(TARGETS
    Calibrations
    DESTINATION lib/${PROJECT_NAME}
    )


install(TARGETS Calibrations camera_models
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(TARGETS Calibrations
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)
  
install(DIRECTORY include/
    DESTINATION include/
)
  


ament_package()
