cmake_minimum_required(VERSION 3.8)
project(icp_slam)

add_compile_options(-O3)

find_package(extension_ament_node REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()
extension_ament_init()

# ICP Scan Matcher
extension_ament_create_node(icp_scan_matcher_component
  NODE_NAME "icp_scan_matcher"
  NODE_PLUGIN "ICPScanMatcher"
)
target_link_libraries(icp_scan_matcher_component
  tbb
)
target_include_directories(icp_scan_matcher_component PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(icp_scan_matcher_component ${PCL_LIBRARIES})
target_compile_definitions(icp_scan_matcher_component PRIVATE ${PCL_DEFINITIONS})
# tf broadcaster
extension_ament_create_node(tf_broadcaster_component
  NODE_NAME "tf_broadcaster"
  NODE_PLUGIN "TFBroadcaster"
)
target_link_libraries(tf_broadcaster_component
  tbb
)
target_include_directories(tf_broadcaster_component PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(tf_broadcaster_component ${PCL_LIBRARIES})
target_compile_definitions(tf_broadcaster_component PRIVATE ${PCL_DEFINITIONS})

# other settings
install(
  DIRECTORY launch rviz config
  DESTINATION share/${PROJECT_NAME}/
)
extension_ament_final()
