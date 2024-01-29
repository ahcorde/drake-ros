add_executable(rgbd rgbd.cc)
target_link_libraries(rgbd
  ament_index_cpp::ament_index_cpp
  drake::drake
  drake_ros::drake_ros_core
  drake_ros::drake_ros_viz
  gflags_shared
)

# sync_cameras_pointcloud
add_library(sync_cameras_pointcloud SHARED
  SyncCamerasPointCloud.cpp
)
target_link_options(sync_cameras_pointcloud PRIVATE "-Wl,--no-undefined")
target_link_libraries(sync_cameras_pointcloud
  image_transport::image_transport
  message_filters::message_filters
  rclcpp::rclcpp
  rclcpp_components::component
  ${sensor_msgs_TARGETS}
)
target_include_directories(sync_cameras_pointcloud PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

install(
  TARGETS
  rgbd
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  rgbd.sdf
  rgbd.rviz
  DESTINATION share/${PROJECT_NAME}/rgbd/
)

install(
  PROGRAMS rgbd.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*_launch.py"
)
