
add_executable( testSegmented fitSegmentedPointcloud.cpp )
target_link_libraries( testSegmented  object_pose_estimation ${PCL_LIBRARIES} )
