cmake_minimum_required(VERSION 2.8.3)
project(qualysis_tsv_parsers)



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
        tsv_pointcloud_builder
        tsv_parser
        tsv_to_pointcloud
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

add_library(tsv_parser
    src/tsv_parser.cpp
)

add_library(tsv_to_pointcloud
    src/tsv_to_pointcloud.cpp
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

target_link_libraries(tsv_parser
    ${catkin_LIBRARIES}
)

target_link_libraries(tsv_to_pointcloud
    tsv_pointcloud_builder
    tsv_parser
    ${catkin_LIBRARIES}
)

target_link_libraries(tsv_to_pointcloud_node
    tsv_to_pointcloud
    ${catkin_LIBRARIES}
)
