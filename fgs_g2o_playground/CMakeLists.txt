cmake_minimum_required(VERSION 2.8.3)
project(fgs_g2o_playground)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  fgs_opt_data_storage
  roscpp
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS fgs_opt_data_storage roscpp
)


# TODO(fugashy) More smarter
set(G2O_LIBS
 /usr/local/lib/libg2o_cli.so
 /usr/local/lib/libg2o_core.so
 /usr/local/lib/libg2o_csparse_extension.so
 /usr/local/lib/libg2o_ext_csparse.so
 /usr/local/lib/libg2o_ext_freeglut_minimal.so
 /usr/local/lib/libg2o_incremental.so
 /usr/local/lib/libg2o_interactive.so
 /usr/local/lib/libg2o_interface.so
 /usr/local/lib/libg2o_opengl_helper.so
 /usr/local/lib/libg2o_parser.so
 /usr/local/lib/libg2o_simulator.so
 /usr/local/lib/libg2o_solver_cholmod.so
 /usr/local/lib/libg2o_solver_csparse.so
 /usr/local/lib/libg2o_solver_dense.so
 /usr/local/lib/libg2o_solver_eigen.so
 /usr/local/lib/libg2o_solver_pcg.so
 /usr/local/lib/libg2o_solver_slam2d_linear.so
 /usr/local/lib/libg2o_solver_structure_only.so
 /usr/local/lib/libg2o_stuff.so
 /usr/local/lib/libg2o_types_data.so
 /usr/local/lib/libg2o_types_icp.so
 /usr/local/lib/libg2o_types_sba.so
 /usr/local/lib/libg2o_types_sclam2d.so
 /usr/local/lib/libg2o_types_sim3.so
 /usr/local/lib/libg2o_types_slam2d.so
 /usr/local/lib/libg2o_types_slam2d_addons.so
 /usr/local/lib/libg2o_types_slam3d.so
 /usr/local/lib/libg2o_types_slam3d_addons.so
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_fitting src/${PROJECT_NAME}/fitting.cpp)
target_link_libraries(${PROJECT_NAME}_fitting
  ${catkin_LIBRARIES}
  ${G2O_LIBS}
)

install(TARGETS ${PROJECT_NAME}_fitting
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
