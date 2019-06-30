cmake_minimum_required(VERSION 3.5)
project(fgs_g2o_playground)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(EIGEN3 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(rclcpp REQUIRED)

set(g2o_INCLUDE_DIRS /usr/local/Cellar/g2o/2017-07-30/include)
set(g2o_LIB_DIR /usr/local/Cellar/g2o/2017-07-30/lib)
set(g2o_LIBS
  ${g2o_LIB_DIR}/libg2o_core.dylib
  ${g2o_LIB_DIR}/libg2o_csparse_extension.dylib
  ${g2o_LIB_DIR}/libg2o_ext_freeglut_minimal.dylib
  ${g2o_LIB_DIR}/libg2o_incremental.dylib
  ${g2o_LIB_DIR}/libg2o_interactive.dylib
  ${g2o_LIB_DIR}/libg2o_interface.dylib
  ${g2o_LIB_DIR}/libg2o_opengl_helper.dylib
  ${g2o_LIB_DIR}/libg2o_parser.dylib
  ${g2o_LIB_DIR}/libg2o_simulator.dylib
  ${g2o_LIB_DIR}/libg2o_solver_cholmod.dylib
  ${g2o_LIB_DIR}/libg2o_solver_csparse.dylib
  ${g2o_LIB_DIR}/libg2o_solver_dense.dylib
  ${g2o_LIB_DIR}/libg2o_solver_eigen.dylib
  ${g2o_LIB_DIR}/libg2o_solver_pcg.dylib
  ${g2o_LIB_DIR}/libg2o_solver_slam2d_linear.dylib
  ${g2o_LIB_DIR}/libg2o_solver_structure_only.dylib
  ${g2o_LIB_DIR}/libg2o_stuff.dylib
  ${g2o_LIB_DIR}/libg2o_types_data.dylib
  ${g2o_LIB_DIR}/libg2o_types_icp.dylib
  ${g2o_LIB_DIR}/libg2o_types_sba.dylib
  ${g2o_LIB_DIR}/libg2o_types_sclam2d.dylib
  ${g2o_LIB_DIR}/libg2o_types_sim3.dylib
  ${g2o_LIB_DIR}/libg2o_types_slam2d.dylib
  ${g2o_LIB_DIR}/libg2o_types_slam2d_addons.dylib
  ${g2o_LIB_DIR}/libg2o_types_slam3d.dylib
  ${g2o_LIB_DIR}/libg2o_types_slam3d_addons.dylib
)

ament_export_dependencies(OpenCV)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${g2o_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_optimizer src/${PROJECT_NAME}/optimizer.cpp)
target_link_libraries(${PROJECT_NAME}_optimizer
  ${g2o_LIBS}
)

add_executable(${PROJECT_NAME}_fitting src/${PROJECT_NAME}/fitting.cpp)
target_link_libraries(${PROJECT_NAME}_fitting
  ${PROJECT_NAME}_optimizer
  ${OpenCV_LIBS}
)

install(TARGETS ${PROJECT_NAME}_fitting
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
