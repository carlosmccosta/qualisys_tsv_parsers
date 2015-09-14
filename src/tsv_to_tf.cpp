/**\file tsv_to_tf.cpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <qualisys_tsv_parsers/tsv_to_tf.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace qualisys_tsv_parsers {

// =============================================================================  <public-section>  ============================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
TSVToTF::TSVToTF() :
		TSVParser(Point6D),
		orientation_offset_(0.0, 0.0, 0.0, 1.0)
		{}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <TSVToPointCloud-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void TSVToTF::setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle) {
	TSVParser::setupConfigurationFromParameterServer(node_handle, private_node_handle);

	double qx, qy, qz, qw;
	private_node_handle->param("tsv_data_offset_qx", qx, 0.0);
	private_node_handle->param("tsv_data_offset_qy", qy, 0.0);
	private_node_handle->param("tsv_data_offset_qz", qz, 0.0);
	private_node_handle->param("tsv_data_offset_qw", qw, 1.0);
	orientation_offset_.setValue(qx, qy, qz, qw);
	orientation_offset_.normalize();

	std::string source_frame_ids;
	private_node_handle->param("source_frame_ids", source_frame_ids, std::string("base_link+camera2+camera1"));
	extractValues(source_frame_ids, source_frame_ids_);

	std::string target_frame_ids;
	private_node_handle->param("target_frame_ids", target_frame_ids, std::string("map+map+map"));
	extractValues(target_frame_ids, target_frame_ids_);

	std::string pose_topics;
	private_node_handle->param("pose_topics", pose_topics, std::string("pose_robot+pose_camera2+pose_camera1"));
	std::vector<std::string> pose_topics_vector;
	extractValues(pose_topics, pose_topics_vector);
	for (size_t i = 0; i < pose_topics_vector.size(); ++i) {
		if (pose_topics_vector[i] == "none") {
			pose_publishers_.push_back(ros::Publisher());
		} else {
			pose_publishers_.push_back(node_handle->advertise<geometry_msgs::PoseStamped>(pose_topics_vector[i], 1, true));
		}
	}

	std::string pose_array_topics;
	private_node_handle->param("pose_array_topics", pose_array_topics, std::string("poses_robot+poses_camera2+poses_camera1"));
	std::vector<std::string> pose_array_topics_vector;
	extractValues(pose_array_topics, pose_array_topics_vector);
	for (size_t i = 0; i < pose_array_topics_vector.size(); ++i) {
		if (pose_array_topics_vector[i] == "none") {
			pose_array_publishers_.push_back(ros::Publisher());
		} else {
			pose_array_publishers_.push_back(node_handle->advertise<geometry_msgs::PoseArray>(pose_array_topics_vector[i], 1, true));
		}
	}
	pose_arrays_.resize(pose_array_topics_vector.size());


	std::string number_msgs_to_skip_in_pose_array_msgs_str;
	private_node_handle->param("number_msgs_to_skip_in_pose_array_msgs", number_msgs_to_skip_in_pose_array_msgs_str, std::string("0+0+0"));
	extractValues(number_msgs_to_skip_in_pose_array_msgs_str, pose_arrays_number_msgs_to_skip_in_pose_array_msgs_);
	pose_arrays_number_msgs_skipped_in_pose_array_msgs_.resize(pose_arrays_number_msgs_to_skip_in_pose_array_msgs_.size(), 0);
}


size_t TSVToTF::publishDataFromTSVFile(const std::string& filename) {
	if (filename.empty()) {
		return 0;
	}

	ros::Time::waitForValid();

	size_t number_published_tfs = 0;
	std::ifstream input_stream(filename.c_str());

	if (input_stream.is_open()) {
		if (!parseTSVHeader(input_stream)) {
			ROS_ERROR("Error parsing tsv header");
			return 0;
		}

		size_t number_of_tracking_bodies = tsv_data_columns_.size();
		if (source_frame_ids_.size() != number_of_tracking_bodies || target_frame_ids_.size() != number_of_tracking_bodies) {
			ROS_ERROR("Incorrect configuration of frame_ids");
			return 0;
		}

		if (!pose_publishers_.empty() && pose_publishers_.size() != number_of_tracking_bodies) {
			ROS_ERROR("Incorrect setup of the PoseStamped publishers");
			return 0;
		}

		if (!pose_array_publishers_.empty() && (
				pose_array_publishers_.size() != number_of_tracking_bodies ||
				pose_arrays_.size() != number_of_tracking_bodies ||
				pose_arrays_number_msgs_to_skip_in_pose_array_msgs_.size() != number_of_tracking_bodies ||
				pose_arrays_number_msgs_skipped_in_pose_array_msgs_.size() != number_of_tracking_bodies)) {
			ROS_ERROR("Incorrect setup of the PoseArray publishers");
			return 0;
		}

		if (tsv_point_type_ == Point6D) {
			ros::Time time_tsv_line;
			while (loadAndPublishTSVTFs(input_stream, time_tsv_line)) {
				double delay_for_next_msg_publish = time_tsv_line.toSec() - ros::Time::now().toSec() - 0.005;
				if (delay_for_next_msg_publish > -3.0) {
					if (delay_for_next_msg_publish > 0.0 && ros::Time::now().sec != 0) { ros::Duration(delay_for_next_msg_publish).sleep(); }
					++number_published_tfs;
				}
			}
		} else {
			ROS_ERROR("tsv data is not 6D");
		}
	}

	return number_published_tfs;
}


bool TSVToTF::loadAndPublishTSVTFs(std::ifstream& input_stream, ros::Time& time_tsv_line_out) {
	std::string line;
	std::getline(input_stream, line);
	if (line.empty()) { return false; }
	std::stringstream ss_line(line);

	std::string value_str;
	double value_number = 0.0;
	bool point_valid = true;
	ros::Time time_stamp = base_time_stamp_;
	std::vector<size_t>::iterator it_indexes = tsv_data_columns_.begin();
	size_t current_target_index = *it_indexes;
	size_t current_token_index = 0;
	size_t current_tracking_body_number = 0;

	double x = 0.0;
	double y = 0.0;
	double z = 0.0;
	double xx = 0.0;
	double xy = 0.0;
	double xz = 0.0;
	double yx = 0.0;
	double yy = 0.0;
	double yz = 0.0;
	double zx = 0.0;
	double zy = 0.0;
	double zz = 0.0;

	while (ss_line >> value_str) {
		if (!point_valid || value_str == tsv_null_string_) {
			point_valid = false;
		} else {
			std::stringstream ss_number(value_str);
			double value_number;
			if (ss_number >> value_number) {
				if (current_token_index == 1) { time_stamp += ros::Duration(value_number * tsv_time_multiplier_); } 						else
				if (current_token_index == current_target_index     ) {  x = value_number * tsv_data_multiplier_ + tsv_data_offset_x_; } 	else
				if (current_token_index == current_target_index +  1) {  y = value_number * tsv_data_multiplier_ + tsv_data_offset_y_; } 	else
				if (current_token_index == current_target_index +  2) {  z = value_number * tsv_data_multiplier_ + tsv_data_offset_z_; } 	else
				if (current_token_index == current_target_index +  7) { xx = value_number; } 												else
				if (current_token_index == current_target_index +  8) { yx = value_number; } 												else
				if (current_token_index == current_target_index +  9) { zx = value_number; } 												else
				if (current_token_index == current_target_index + 10) { xy = value_number; } 												else
				if (current_token_index == current_target_index + 11) { yy = value_number; } 												else
				if (current_token_index == current_target_index + 12) { zy = value_number; } 												else
				if (current_token_index == current_target_index + 13) { xz = value_number; } 												else
				if (current_token_index == current_target_index + 14) { yz = value_number; } 												else
				if (current_token_index == current_target_index + 15) { zz = value_number; }
			} else {
				point_valid = false;
			}
		}

		if (current_token_index == current_target_index + 15) {
			if (point_valid) {
				tf2::Matrix3x3 rotation_matrix;
				rotation_matrix.setValue(xx, xy, xz, yx, yy, yz, zx, zy, zz);
				tf2::Quaternion orientation;
				rotation_matrix.getRotation(orientation);
				orientation.normalize();
				orientation = orientation_offset_ * orientation;
				orientation.normalize();
				publishNewBodyTF(x, y, z, orientation, current_tracking_body_number, time_stamp);
				publishNewBodyPose(x, y, z, orientation, current_tracking_body_number, time_stamp);
			}
			point_valid = true;

			++current_tracking_body_number;
			++it_indexes;
			if (it_indexes == tsv_data_columns_.end()) {
				time_tsv_line_out = time_stamp;
				return true;
			}
			current_target_index = *it_indexes;
		}

		++current_token_index;
	}

	return loadAndPublishTSVTFs(input_stream, time_tsv_line_out); // robust against malformed lines in middle of tsv file
}


void TSVToTF::publishNewBodyTF(double x, double y, double z, const tf2::Quaternion& orientation, size_t tracking_body_number, const ros::Time& time_stamp) {
	geometry_msgs::TransformStamped tf;
	tf.child_frame_id = source_frame_ids_[tracking_body_number];
	tf.header.frame_id = target_frame_ids_[tracking_body_number];
	tf.header.stamp = time_stamp;

	tf.transform.translation.x = x;
	tf.transform.translation.y = y;
	tf.transform.translation.z = z;
	tf.transform.rotation.x = orientation.getX();
	tf.transform.rotation.y = orientation.getY();
	tf.transform.rotation.z = orientation.getZ();
	tf.transform.rotation.w = orientation.getW();

	transform_broadcaster_.sendTransform(tf);
}


void TSVToTF::publishNewBodyPose(double x, double y, double z, const tf2::Quaternion& orientation, size_t tracking_body_number, const ros::Time& time_stamp) {
	if (!pose_publishers_[tracking_body_number].getTopic().empty() || !pose_array_publishers_[tracking_body_number].getTopic().empty()) {
		geometry_msgs::PoseStamped pose;
		pose.header.frame_id = target_frame_ids_[tracking_body_number];
		pose.header.stamp = time_stamp;
		pose.pose.position.x = x;
		pose.pose.position.y = y;
		pose.pose.position.z = z;
		pose.pose.orientation.x = orientation.getX();
		pose.pose.orientation.y = orientation.getY();
		pose.pose.orientation.z = orientation.getZ();
		pose.pose.orientation.w = orientation.getW();

		if (!pose_publishers_[tracking_body_number].getTopic().empty()) {
			pose_publishers_[tracking_body_number].publish(pose);
		}

		if (pose_arrays_number_msgs_to_skip_in_pose_array_msgs_[tracking_body_number] <= 0 || pose_arrays_number_msgs_skipped_in_pose_array_msgs_[tracking_body_number] >= pose_arrays_number_msgs_to_skip_in_pose_array_msgs_[tracking_body_number]) {
			if (!pose_array_publishers_[tracking_body_number].getTopic().empty()) {
				pose_arrays_[tracking_body_number].poses.push_back(pose.pose);
				pose_arrays_[tracking_body_number].header.frame_id = target_frame_ids_[tracking_body_number];
				pose_arrays_[tracking_body_number].header.stamp = time_stamp;
				pose_array_publishers_[tracking_body_number].publish(pose_arrays_[tracking_body_number]);
			}
			pose_arrays_number_msgs_skipped_in_pose_array_msgs_[tracking_body_number] = 0;
		} else {
			++pose_arrays_number_msgs_skipped_in_pose_array_msgs_[tracking_body_number];
		}
	}
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </TSVToPointCloud-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================


} /* namespace qualisys_tsv_parsers */

