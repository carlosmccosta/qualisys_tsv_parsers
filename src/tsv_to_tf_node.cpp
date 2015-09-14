/**\file tsv_to_tf_node.cpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */


// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <ros/ros.h>

#include <qualisys_tsv_parsers/tsv_to_tf.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<



// ###################################################################################   <main>   ##############################################################################
int main(int argc, char** argv) {
	ros::init(argc, argv, "tsv_to_tf_node");

	ros::NodeHandlePtr node_handle(new ros::NodeHandle());
	ros::NodeHandlePtr private_node_handle(new ros::NodeHandle("~"));

	std::string tsv_filename;
	private_node_handle->param("tsv_filename", tsv_filename, std::string("ground_truth.tsv"));

	qualisys_tsv_parsers::TSVToTF tsv_to_cloud;
	tsv_to_cloud.setupConfigurationFromParameterServer(node_handle, private_node_handle);
	tsv_to_cloud.publishDataFromTSVFile(tsv_filename);

	return 0;
}
// ###################################################################################   </main>   #############################################################################
