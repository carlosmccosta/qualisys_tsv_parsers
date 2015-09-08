/**\file tsv_parser.cpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <tsv_to_pointcloud/tsv_parser.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace tsv_to_pointcloud {

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <imports>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </imports>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// =============================================================================  <public-section>  ============================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
TSVParser::TSVParser() : tsv_point_type_(Point3D) {}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <TSVParser-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void TSVParser::setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle) {
	std::string pointcloud_publish_topic;
	private_node_handle->param("pointcloud_publish_topic", pointcloud_publish_topic, std::string("tsv_pointcloud"));
	if (!pointcloud_publish_topic.empty()) pointcloud_publisher_ = node_handle->advertise<sensor_msgs::PointCloud2>(pointcloud_publish_topic, 1, true);

	private_node_handle->param("load_pointclouds_base_time_from_tsv_header", load_pointclouds_base_time_from_tsv_header_, true);

	double time_offset = 0.0;
	private_node_handle->param("pointclouds_time_offset", time_offset, 0.0);
	time_offset_.fromSec(time_offset);

	std::string tsv_points_offsets;
	private_node_handle->param("tsv_points_offsets", tsv_points_offsets, std::string("2+5+8+11+14+17+20+23"));
	parsePointsOffsets(tsv_points_offsets);

	private_node_handle->param("tsv_data_multiplier", tsv_data_multiplier_, 0.001);
	private_node_handle->param("tsv_data_offset_x", tsv_data_offset_x_, 0.0);
	private_node_handle->param("tsv_data_offset_y", tsv_data_offset_y_, 0.0);
	private_node_handle->param("tsv_data_offset_z", tsv_data_offset_z_, 0.0);

	private_node_handle->param("tsv_time_multiplier", tsv_time_multiplier_, 1.0);
}


void TSVParser::parsePointsOffsets(std::string& offsets) {
	std::replace(offsets.begin(), offsets.end(), '+', ' ');

	std::stringstream ss(offsets);
	size_t offset;

	while (ss >> offset) {
		tsv_points_offsets_.push_back(offset);
	}
}


size_t TSVParser::publishPointCloudsFromTSV(const std::string& filename, const std::string& frame_id) {
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


bool TSVParser::parseTSVHeader(std::ifstream& input_stream) {
	// time offset
	if (load_pointclouds_base_time_from_tsv_header_) {
		std::string tsv_base_time;
		if (!loadTSVValue(input_stream, "TIME_STAMP", tsv_base_time)) { return false; }
		base_time_stamp_.fromSec(parseUTCBaseTime(tsv_base_time));
		base_time_stamp_ += time_offset_;
	} else {
		base_time_stamp_ = ros::Time::now();
	}

	//data type
	std::string data_type;
	if (!loadTSVValue(input_stream, "DATA_INCLUDED", data_type))  { return false; }
	std::stringstream ss(data_type);
	ss >> data_type;

	if (data_type == "2D") {
		tsv_point_type_ = Point2D;
	} else if (data_type == "3D") {
		tsv_point_type_ = Point3D;
	} else if (data_type == "6D") {
		tsv_point_type_ = Point6D;
	} else {
		return false;
	}

	// discard remaining info
	std::string remaining_data;
	if (!loadTSVValue(input_stream, "Frame", remaining_data)) { return false; }

	return true;
}


bool TSVParser::loadTSVValue(std::ifstream& input_stream, const std::string& key, std::string& value_out) {
	std::string line;
	while (std::getline(input_stream, line)) {
		std::stringstream ss(line);
		std::string tsv_key;

		ss >> tsv_key;
		if (tsv_key == key) {
			std::getline(ss, value_out);
			return true;
		}
	}

	return false;
}


double TSVParser::parseUTCBaseTime(const std::string& tsv_base_time) {
	std::string temp_data;
	std::stringstream ss(tsv_base_time);
	ss >> temp_data;
	ss >> temp_data;
	double time_from_boot;
	ss >> time_from_boot;

	return time_from_boot;
}


bool TSVParser::loadTSVPointCloud(std::ifstream& input_stream, sensor_msgs::PointCloud2::Ptr& pointcloud, const std::string& frame_id) {
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
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </TSVParser-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

// =============================================================================   <private-section>   =========================================================================
// =============================================================================   </private-section>  =========================================================================

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <template instantiations>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </template instantiations>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


} /* namespace tsv_to_pointcloud */

