#pragma once

/**\file tsv_to_tf.h
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */


// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// std includes
#include <fstream>
#include <string>
#include <vector>

// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/transform_broadcaster.h>

// external libs includes

// project includes
#include <qualisys_tsv_parsers/tsv_parser.h>

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


namespace qualisys_tsv_parsers {
// ###############################################################################   tsv_to_tf   ###############################################################################
/**
 * \brief Description...
 */
class TSVToTF : public TSVParser {
	// ========================================================================   <public-section>   ===========================================================================
	public:
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		TSVToTF();
		virtual ~TSVToTF() {}
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <TSVToTF-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		virtual void setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle);
		virtual size_t publishDataFromTSVFile(const std::string& filename);
		bool loadAndPublishTSVTFs(std::ifstream& input_stream, ros::Time& time_tsv_line_out);
		void publishNewBodyTF(double x, double y, double z, const tf2::Quaternion& orientation, size_t tracking_body_number, const ros::Time& time_stamp = ros::Time::now());
		void publishNewBodyPose(double x, double y, double z, const tf2::Quaternion& orientation, size_t tracking_body_number, const ros::Time& time_stamp = ros::Time::now());
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </TSVToTF-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	// ========================================================================   </public-section>  ===========================================================================

	// ========================================================================   <protected-section>   ========================================================================
	protected:
		// configuration fields
		std::vector<std::string> source_frame_ids_;
		std::vector<std::string> target_frame_ids_;
		std::vector<std::string> target_frame_ids_poses_;
		std::vector<int> pose_arrays_number_msgs_to_skip_in_pose_array_msgs_;
		std::vector<bool> invert_tf_transforms_;
		std::vector<bool> invert_tf_hierarchies_;
		tf2::Transform transform_poses_offset_;
		bool tsv_data_poses_offset_post_multiplication_;

		// state fields
		std::vector<geometry_msgs::PoseArray> pose_arrays_;
		std::vector<int> pose_arrays_number_msgs_skipped_in_pose_array_msgs_;

		// ros communication fields
		tf2_ros::TransformBroadcaster transform_broadcaster_;
		std::vector<ros::Publisher> pose_publishers_;
		std::vector<ros::Publisher> pose_array_publishers_;
	// ========================================================================   </protected-section>  ========================================================================
};

} /* namespace tsv_to_pointcloud */
