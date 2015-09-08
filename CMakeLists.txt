cmake_minimum_required(VERSION 2.8.3)
project(tsv_to_pointcloud)



#######################################################################################################################
## packages
#######################################################################################################################

set(${PROJECT_NAME}_CATKIN_COMPONENTS
    roscpp
    sensor_msgs
)

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_COMPONENTS})


#######################################################################################################################
## catkin specific configuration
#######################################################################################################################


catkin_package(
    INCLUDE_DIRS include
    LIBRARIES
        tsv_to_pointcloud_parser
    CATKIN_DEPENDS
        ${${PROJECT_NAME}_CATKIN_COMPONENTS}
)



#######################################################################################################################
## build
#######################################################################################################################

#==================
# includes
#==================

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)



#==================
# libraries
#==================

add_library(tsv_pointcloud_builder
    src/pointcloud2_builder.cpp
)

add_library(tsv_to_pointcloud_parser
    src/tsv_parser.cpp
)


#==================
# executables
#==================

add_executable(tsv_to_pointcloud_node
    src/tsv_to_pointcloud_node.cpp
)



#==================
# library link
#==================

target_link_libraries(tsv_pointcloud_builder
    ${catkin_LIBRARIES}
)

target_link_libraries(tsv_to_pointcloud_parser
    tsv_pointcloud_builder
    ${catkin_LIBRARIES}
)

target_link_libraries(tsv_to_pointcloud_node
    tsv_to_pointcloud_parser
    ${catkin_LIBRARIES}
)
