cmake_minimum_required(VERSION 3.5)
project(nav2_social_mpc_controller)

set(CMAKE_BUILD_TYPE Release) # significant Ceres optimization speedup
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(angles REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(people_msgs REQUIRED)
find_package(obstacle_distance_msgs REQUIRED)
find_package(obstacle_distance REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)
find_package(nav_2d_utils REQUIRED)

set(CMAKE_CXX_STANDARD 17)

nav2_package()

include_directories(
  /usr/local/include  #to find lightsfm, just in case
  include
  /usr/local/include/eigen3
  ${CERES_INCLUDES}
)

set(dependencies
  angles
  rclcpp
  geometry_msgs
  people_msgs
  obstacle_distance_msgs
  obstacle_distance
  nav2_costmap_2d
  pluginlib
  nav_msgs
  nav2_util
  nav2_core
  tf2
  nav_2d_utils
)

set(library_name ${PROJECT_NAME})

add_library(mpc_critics SHARED
  src/critics/agent_angle_cost_function.cpp
  src/critics/angle_cost_function.cpp
  src/critics/curvature_cost_function.cpp
  src/critics/distance_cost_function.cpp
  src/critics/goal_align_cost_function.cpp
  src/critics/obstacle_cost_function.cpp
  src/critics/social_work_cost_function.cpp
  src/critics/velocity_cost_function.cpp
  src/critics/velocity_feasibility_cost_function.cpp
  src/critics/proxemics_cost_function.cpp
)

add_library(${library_name} SHARED
        src/path_trajectorizer.cpp
        src/social_mpc_controller.cpp
        src/people_interface.cpp
        src/obstacle_distance_interface.cpp
        src/optimizer.cpp
        src/path_handler.cpp
)

set(libraries 
  mpc_critics
  ${library_name}
)

foreach(lib IN LISTS libraries)
  target_link_libraries(${lib} ceres)
  ament_target_dependencies(${lib}
    ${dependencies})
endforeach()

target_link_libraries(${library_name} mpc_critics)
# prevent pluginlib from using boost
target_compile_definitions(${library_name} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

install(TARGETS ${library_name} mpc_critics
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
#   add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_libraries(${libraries})
ament_export_dependencies(${dependencies})
ament_export_definitions("PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(nav2_core nav2_social_mpc_controller.xml)

ament_package()

