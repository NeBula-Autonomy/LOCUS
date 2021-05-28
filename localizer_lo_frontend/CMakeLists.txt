cmake_minimum_required(VERSION 2.8.3)
project(localizer_lo_frontend)
find_package(catkin REQUIRED
    lo_frontend
    point_cloud_odometry
    point_cloud_filter)


    catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS
        lo_frontend
        point_cloud_odometry
        point_cloud_filter
      )
