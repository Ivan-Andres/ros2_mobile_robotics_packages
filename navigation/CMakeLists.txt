cmake_minimum_required(VERSION 3.8)
project(navigation)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)                         # nodo base              :contentReference[oaicite:0]{index=0}
find_package(tf2 REQUIRED)                            # cuaterniones           :contentReference[oaicite:1]{index=1}
find_package(tf2_ros REQUIRED)                        # broadcasters TF        :contentReference[oaicite:2]{index=2}
find_package(tf2_geometry_msgs REQUIRED)              # conversión mensajes    :contentReference[oaicite:3]{index=3}
find_package(geometry_msgs REQUIRED)                  # Pose, TransformStamped :contentReference[oaicite:4]{index=4}
find_package(nav_msgs REQUIRED)                       # Odometry               :contentReference[oaicite:5]{index=5}
find_package(rosgraph_msgs REQUIRED)                  # Clock                  :contentReference[oaicite:6]{index=6}
find_package(sensor_msgs REQUIRED)  


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(manual_odometry_node src/manual_odometry_node.cpp)
ament_target_dependencies(manual_odometry_node
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  rosgraph_msgs
  sensor_msgs
)

add_executable(manual_odometry_node_slam src/manual_odometry_node_slam.cpp)
ament_target_dependencies(manual_odometry_node_slam
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  rosgraph_msgs
  sensor_msgs
)

install(TARGETS
  manual_odometry_node
  manual_odometry_node_slam
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config description launch worlds src
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
