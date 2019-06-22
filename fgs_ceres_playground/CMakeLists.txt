cmake_minimum_required(VERSION 3.5)
project(fgs_ceres_playground)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenCV 4 REQUIRED)

find_package(rclcpp REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

ament_export_dependencies(OpenCV)
ament_export_dependencies(Ceres)

add_executable(${PROJECT_NAME}_hello_world
  src/${PROJECT_NAME}/hello_world.cpp)
target_link_libraries(${PROJECT_NAME}_hello_world
  ${CERES_LIBRARIES}
)

add_library(${PROJECT_NAME}
  # opencv vizをインストールするまで封印
  # src/${PROJECT_NAME}/bundle_adjustment_in_the_large.cpp
  # src/${PROJECT_NAME}/cv_viz.cpp
  src/${PROJECT_NAME}/optimization_context_factory.cpp)
target_link_libraries(${PROJECT_NAME}
  ${CERES_LIBRARIES}
  ${OpenCV_LIBS}
)

add_executable(${PROJECT_NAME}_solve
  src/${PROJECT_NAME}/solve.cpp)
target_link_libraries(${PROJECT_NAME}_solve
  ${PROJECT_NAME}
)

install(
  TARGETS
    ${PROJECT_NAME}
    ${PROJECT_NAME}_hello_world
    ${PROJECT_NAME}_solve
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
