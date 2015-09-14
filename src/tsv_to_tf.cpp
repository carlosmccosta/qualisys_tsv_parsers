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
		tsv_data_poses_offset_post_multiplication_(true)
		{}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <TSVToPointCloud-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void TSVToTF::setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle) {
	TSVParser::setupConfigurationFromParameterServer(node_handle, private_node_handle);

	std::string source_frame_ids;
	private_node_handle->param("source_frame_ids", source_frame_ids, std::string("base_link+camera2+camera1"));
	extractValues(source_frame_ids, source_frame_ids_);

	std::string target_frame_ids;
	private_node_handle->param("target_frame_ids", target_frame_ids, std::string("map+map+map"));
	extractValues(target_frame_ids, target_frame_ids_);

	std::string target_frame_ids_poses;
	private_node_handle->param("target_frame_ids_poses", target_frame_ids_poses, std::string("map+map+map"));
	extractValues(target_frame_ids_poses, target_frame_ids_poses_);

	std::string invert_tf_transforms_str;
	private_node_handle->param("invert_tf_transforms", invert_tf_transforms_str, std::string("0+0+0"));
	extractValues(invert_tf_transforms_str, invert_tf_transforms_);

	std::string invert_tf_hierarchies_str;
	private_node_handle->param("invert_tf_hierarchies", invert_tf_hierarchies_str, std::string("0+0+0"));
	extractValues(invert_tf_hierarchies_str, invert_tf_hierarchies_);

	double tsv_data_offset_x, tsv_data_offset_y, tsv_data_offset_z;
	private_node_handle->param("tsv_data_multiplier", tsv_data_multiplier_, 0.001);
	private_node_handle->param("tsv_data_poses_offset_x", tsv_data_offset_x, 0.0);
	private_node_handle->param("tsv_data_poses_offset_y", tsv_data_offset_y, 0.0);
	private_node_handle->param("tsv_data_poses_offset_z", tsv_data_offset_z, 0.0);

	double qx, qy, qz, qw;
	private_node_handle->param("tsv_data_poses_offset_qx", qx, 0.0);
	private_node_handle->param("tsv_data_poses_offset_qy", qy, 0.0);
	private_node_handle->param("tsv_data_poses_offset_qz", qz, 0.0);
	private_node_handle->param("tsv_data_poses_offset_qw", qw, 1.0);

	transform_poses_offset_.setOrigin(tf2::Vector3(tsv_data_offset_x, tsv_data_offset_y, tsv_data_offset_z));
	transform_poses_offset_.setRotation(tf2::Quaternion(qx, qy, qz, qw));

	private_node_handle->param("tsv_data_poses_offset_post_multiplication", tsv_data_poses_offset_post_multiplication_, true);

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
		if (source_frame_ids_.size() != number_of_tracking_bodies || target_frame_ids_.size() != number_of_tracking_bodies || target_frame_ids_poses_.size() != number_of_tracking_bodies) {
			ROS_ERROR("Incorrect configuration of frame_ids");
			return 0;
		}

		if (!pose_publishers_.empty() && pose_publishers_.size() != number_of_tracking_bodies) {
			ROS_ERROR("Incorrect setup of the PoseStamped publishers");
			return 0;
		}

		if ((!invert_tf_hierarchies_.empty() && invert_tf_hierarchies_.size() != number_of_tracking_bodies) ||
			(!invert_tf_transforms_.empty()  && invert_tf_transforms_.size()  != number_of_tracking_bodies)) {
			ROS_ERROR("Incorrect setup of the inversion of the TF transforms");
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
				if (current_token_index == current_target_index     ) {  x = value_number * tsv_data_multiplier_; } 	else
				if (current_token_index == current_target_index +  1) {  y = value_number * tsv_data_multiplier_; } 	else
				if (current_token_index == current_target_index +  2) {  z = value_number * tsv_data_multiplier_; } 	else
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

				tf2::Transform tsv_tf;
				tsv_tf.setOrigin(tf2::Vector3(x, y, z));
				tsv_tf.setRotation(orientation);

				tf2::Transform tsv_tf_transformed;
				if (tsv_data_offset_post_multiplication_) {
					tsv_tf_transformed = tsv_tf * transform_offset_;
				} else {
					tsv_tf_transformed = transform_offset_ * tsv_tf;
				}

				tf2::Transform tsv_tf_poses_transformed;
				if (tsv_data_poses_offset_post_multiplication_) {
					tsv_tf_poses_transformed = tsv_tf * transform_poses_offset_;
				} else {
					tsv_tf_poses_transformed = transform_poses_offset_ * tsv_tf;
				}

				publishNewBodyTF(tsv_tf_transformed.getOrigin().getX(), tsv_tf_transformed.getOrigin().getY(), tsv_tf_transformed.getOrigin().getZ(), tsv_tf_transformed.getRotation(), current_tracking_body_number, time_stamp);
				publishNewBodyPose(tsv_tf_poses_transformed.getOrigin().getX(), tsv_tf_poses_transformed.getOrigin().getY(), tsv_tf_poses_transformed.getOrigin().getZ(), tsv_tf_poses_transformed.getRotation(), current_tracking_body_number, time_stamp);
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
	tf.child_frame_id  = (!invert_tf_hierarchies_.empty() && invert_tf_hierarchies_[tracking_body_number]) ? target_frame_ids_[tracking_body_number] : source_frame_ids_[tracking_body_number];
	tf.header.frame_id = (!invert_tf_hierarchies_.empty() && invert_tf_hierarchies_[tracking_body_number]) ? source_frame_ids_[tracking_body_number] : target_frame_ids_[tracking_body_number];
	tf.header.stamp = time_stamp;

	if (!invert_tf_transforms_.empty() && invert_tf_transforms_[tracking_body_number]) {
		tf2::Transform tf2;
		tf2.setOrigin(tf2::Vector3(x, y, z));
		tf2.setRotation(orientation);
		tf2::Transform tf2_inverse = tf2.inverse();

		tf.transform.translation.x = tf2_inverse.getOrigin().getX();
		tf.transform.translation.y = tf2_inverse.getOrigin().getY();
		tf.transform.translation.z = tf2_inverse.getOrigin().getZ();
		tf.transform.rotation.x = tf2_inverse.getRotation().getX();
		tf.transform.rotation.y = tf2_inverse.getRotation().getY();
		tf.transform.rotation.z = tf2_inverse.getRotation().getZ();
		tf.transform.rotation.w = tf2_inverse.getRotation().getW();
	} else {
		tf.transform.translation.x = x;
		tf.transform.translation.y = y;
		tf.transform.translation.z = z;
		tf.transform.rotation.x = orientation.getX();
		tf.transform.rotation.y = orientation.getY();
		tf.transform.rotation.z = orientation.getZ();
		tf.transform.rotation.w = orientation.getW();
	}

	transform_broadcaster_.sendTransform(tf);
}


void TSVToTF::publishNewBodyPose(double x, double y, double z, const tf2::Quaternion& orientation, size_t tracking_body_number, const ros::Time& time_stamp) {
	if (!pose_publishers_[tracking_body_number].getTopic().empty() || !pose_array_publishers_[tracking_body_number].getTopic().empty()) {
		geometry_msgs::PoseStamped pose;
		pose.header.frame_id = target_frame_ids_poses_[tracking_body_number];
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
				pose_arrays_[tracking_body_number].header.frame_id = target_frame_ids_poses_[tracking_body_number];
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

