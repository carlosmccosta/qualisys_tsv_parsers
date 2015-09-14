/**\file tsv_to_pointcloud.cpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <qualisys_tsv_parsers/tsv_to_pointcloud.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace qualisys_tsv_parsers {

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <imports>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </imports>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// =============================================================================  <public-section>  ============================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
TSVToPointCloud::TSVToPointCloud() :
		TSVParser(Point6D),
		pointclouds_frame_id_("map")
		{}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <TSVToPointCloud-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void TSVToPointCloud::setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle) {
	TSVParser::setupConfigurationFromParameterServer(node_handle, private_node_handle);

	private_node_handle->param("pointclouds_frame_id", pointclouds_frame_id_, std::string("map"));

	std::string pointcloud_publish_topic;
	private_node_handle->param("pointcloud_publish_topic", pointcloud_publish_topic, std::string("tsv_pointcloud"));
	if (!pointcloud_publish_topic.empty()) pointcloud_publisher_ = node_handle->advertise<sensor_msgs::PointCloud2>(pointcloud_publish_topic, 1, true);
}


size_t TSVToPointCloud::publishDataFromTSVFile(const std::string& filename) {
	if (pointcloud_publisher_.getTopic().empty() || filename.empty() || pointclouds_frame_id_.empty()) {
		return 0;
	}

	ros::Time::waitForValid();

	size_t number_published_pointclouds = 0;
	std::ifstream input_stream(filename.c_str());
	sensor_msgs::PointCloud2::Ptr pointcloud;

	if (input_stream.is_open()) {
		if (!parseTSVHeader(input_stream)) {
			ROS_ERROR("Error parsing tsv header");
			return 0;
		}
		if (tsv_point_type_ == Point2D || tsv_point_type_ == Point3D) {
			while (loadTSVPointCloud(input_stream, pointcloud, pointclouds_frame_id_)) {
				double delay_for_next_msg_publish = pointcloud->header.stamp.toSec() - ros::Time::now().toSec() - 0.005;
				if (delay_for_next_msg_publish > -3.0) {
					if (delay_for_next_msg_publish > 0.0 && ros::Time::now().sec != 0) { ros::Duration(delay_for_next_msg_publish).sleep(); }
					pointcloud_publisher_.publish(pointcloud);
					++number_published_pointclouds;
				}
			}
		} else {
			ROS_ERROR("tsv data is not 2D or 3D");
		}
	}

	return number_published_pointclouds;
}


bool TSVToPointCloud::loadTSVPointCloud(std::ifstream& input_stream, sensor_msgs::PointCloud2::Ptr& pointcloud, const std::string& frame_id) {
	std::string line;
	std::getline(input_stream, line);
	if (line.empty()) { return false; }
	std::stringstream ss_line(line);

	std::string value_str;
	ros::Time time_stamp = base_time_stamp_;
	double x = 0.0;
	double y = 0.0;
	double z = 0.0;
	std::vector<size_t>::iterator it_indexes = tsv_data_columns_.begin();
	size_t current_target_index = *it_indexes;
	size_t current_token_index = 0;

	PointCloud2Builder pointcloud_builder;
	pointcloud_builder.createNewCloud(frame_id);
	bool point_valid = true;

	while (ss_line >> value_str) {
		if (!point_valid || value_str == tsv_null_string_) {
			point_valid = false;
		} else {
			std::stringstream ss_number(value_str);
			double value_number;
			if (ss_number >> value_number) {
				if (current_token_index == 1) {
					time_stamp += ros::Duration(value_number * tsv_time_multiplier_);
				}

				value_number *= tsv_data_multiplier_;

				if (current_token_index == current_target_index) {
					x = value_number;
				} else if (current_token_index == current_target_index + 1) {
					y = value_number;
				} else if (current_token_index == current_target_index + 2) {
					z = value_number;
				}
			} else {
				point_valid = false;
			}
		}

		if ((current_token_index == current_target_index + 1 && tsv_point_type_ == Point2D) || current_token_index == current_target_index + 2) {
			if (point_valid) {
				tf2::Transform tsv_tf;
				tsv_tf.setOrigin(tf2::Vector3(x, y, z));

				tf2::Transform tsv_tf_transformed;
				if (tsv_data_offset_post_multiplication_) {
					tsv_tf_transformed = tsv_tf * transform_offset_;
				} else {
					tsv_tf_transformed = transform_offset_ * tsv_tf;
				}
				pointcloud_builder.addNewPoint(tsv_tf_transformed.getOrigin().getX(), tsv_tf_transformed.getOrigin().getY(), tsv_tf_transformed.getOrigin().getZ());
			}
			point_valid = true;

			++it_indexes;
			if (it_indexes == tsv_data_columns_.end()) {
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


} /* namespace qualisys_tsv_parsers */

