/**\file tsv_to_pointcloud_node.cpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */


// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <ros/ros.h>
#include <qualysis_tsv_parsers/tsv_to_pointcloud.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<



// ###################################################################################   <main>   ##############################################################################
int main(int argc, char** argv) {
	ros::init(argc, argv, "tsv_to_pointcloud_node");

	ros::NodeHandlePtr node_handle(new ros::NodeHandle());
	ros::NodeHandlePtr private_node_handle(new ros::NodeHandle("~"));

	std::string tsv_filename;
	private_node_handle->param("tsv_filename", tsv_filename, std::string("pointclouds.tsv"));

	std::string pointclouds_frame_id;
	private_node_handle->param("pointclouds_frame_id", pointclouds_frame_id, std::string("map"));

	tsv_to_pointcloud::TSVToPointCloud tsv_to_cloud;
	tsv_to_cloud.setupConfigurationFromParameterServer(node_handle, private_node_handle);
	tsv_to_cloud.publishDataFromTSV(tsv_filename, pointclouds_frame_id);

	return 0;
}
// ###################################################################################   </main>   #############################################################################
