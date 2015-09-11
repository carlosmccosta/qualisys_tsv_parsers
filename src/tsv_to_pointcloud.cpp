/**\file tsv_to_pointcloud.cpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <qualysis_tsv_parsers/tsv_to_pointcloud.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace tsv_to_pointcloud {

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <imports>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </imports>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// =============================================================================  <public-section>  ============================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
TSVToPointCloud::TSVToPointCloud() : TSVParser(Point6D) {}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <TSVToPointCloud-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void TSVToPointCloud::setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle) {
	TSVParser::setupConfigurationFromParameterServer(node_handle, private_node_handle);

	std::string pointcloud_publish_topic;
	private_node_handle->param("pointcloud_publish_topic", pointcloud_publish_topic, std::string("tsv_pointcloud"));
	if (!pointcloud_publish_topic.empty()) pointcloud_publisher_ = node_handle->advertise<sensor_msgs::PointCloud2>(pointcloud_publish_topic, 1, true);
}


size_t TSVToPointCloud::publishDataFromTSV(const std::string& filename, const std::string& frame_id) {
	if (pointcloud_publisher_.getTopic().empty() || filename.empty() || frame_id.empty()) {
		return 0;
	}

	ros::Time::waitForValid();

	size_t number_published_pointclouds = 0;
	std::ifstream input_stream(filename.c_str());
	sensor_msgs::PointCloud2::Ptr pointcloud;

	if (input_stream.is_open()) {
		if (!parseTSVHeader(input_stream)) { return 0; }
		while (loadTSVPointCloud(input_stream, pointcloud, frame_id)) {
			double delay_for_next_msg_publish = pointcloud->header.stamp.toSec() - ros::Time::now().toSec() - 0.005;
			if (delay_for_next_msg_publish > -3.0) {
				if (delay_for_next_msg_publish > 0.0 && ros::Time::now().sec != 0) { ros::Duration(delay_for_next_msg_publish).sleep(); }
				pointcloud_publisher_.publish(pointcloud);
				++number_published_pointclouds;
			}
		}
	}

	return number_published_pointclouds;
}


bool TSVToPointCloud::loadTSVPointCloud(std::ifstream& input_stream, sensor_msgs::PointCloud2::Ptr& pointcloud, const std::string& frame_id) {
	std::string line;
	std::getline(input_stream, line);
	if (line.empty()) { return false; }
	std::stringstream ss(line);

	double value;
	ros::Time time_stamp = base_time_stamp_;
	double x = 0.0;
	double y = 0.0;
	double z = 0.0;
	std::vector<size_t>::iterator it_indexes = tsv_points_offsets_.begin();
	size_t current_target_index = *it_indexes;
	size_t current_token_index = 0;

	PointCloud2Builder pointcloud_builder;
	pointcloud_builder.createNewCloud(frame_id);

	while (ss >> value) {
		if (current_token_index == 1) {
			time_stamp += ros::Duration(value * tsv_time_multiplier_);
		}

		value *= tsv_data_multiplier_;

		if (current_token_index == current_target_index) {
			x = value + tsv_data_offset_x_;
		}

		if (current_token_index == current_target_index + 1) {
			y = value + tsv_data_offset_y_;
		}

		if (current_token_index == current_target_index + 2) {
			z = value + tsv_data_offset_z_;
		}

		if ((current_token_index == current_target_index + 1 && tsv_point_type_ == Point2D) || current_token_index == current_target_index + 2) {
			pointcloud_builder.addNewPoint(x, y, z);

			++it_indexes;
			if (it_indexes == tsv_points_offsets_.end()) {
				pointcloud = pointcloud_builder.getPointcloudMsg();
				pointcloud->header.stamp = time_stamp;
				return true;
			}
			current_target_index = *it_indexes;
		}

		++current_token_index;
	}

	return loadTSVPointCloud(input_stream, pointcloud, frame_id); // robust against malformed lines in middle of tsv file
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </TSVToPointCloud-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================


} /* namespace tsv_to_pointcloud */

