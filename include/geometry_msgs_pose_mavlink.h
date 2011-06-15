#include "mavlink.h"
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/Pose.h"

static inline void convertROSPoseToMavlink(uint8_t sysid, uint8_t compid, const geometry_msgs::Pose &pose_msg, uint64_t mavlink_timestamp, mavlink_message_t *msg)
{
	//convert quaternion to euler angles
	const btQuaternion quat(pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w);
	const btMatrix3x3 mat(quat);
	double roll, pitch, yaw;
	mat.getEulerYPR(yaw, pitch, roll);	

	mavlink_msg_vision_position_estimate_pack(sysid, compid, msg, mavlink_timestamp, pose_msg.position.x, pose_msg.position.y, pose_msg.position.z, roll, pitch, yaw);
}

static inline void convertROSPoseStampedToMavlink(uint8_t sysid, uint8_t compid, const geometry_msgs::PoseStamped &pose_msg, uint64_t mavlink_timestamp, mavlink_message_t *msg)
{
	//convert quaternion to euler angles
	const btQuaternion quat(pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w);
	const btMatrix3x3 mat(quat);
	double roll, pitch, yaw;
	mat.getEulerYPR(yaw, pitch, roll);

	mavlink_msg_vision_position_estimate_pack(sysid, compid, msg, mavlink_timestamp, pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z, roll, pitch, yaw);
}
