#include "ros/ros.h"

#include "tf/transform_broadcaster.h"

#include "lcm_mavlink_ros/Mavlink.h"
#include "geometry_msgs/Pose.h"

#include "mavconn.h"
#include "mavlinkros.h"

#include <sstream>
#include <glib.h>

std::string lcmurl = "udpm://"; ///< host name for UDP server
bool verbose;

int sysid = 42;
int compid = 199;

/**
 * Grabs all mavlink-messages from the ROS-Topic "mavlink" and publishes them on the LCM-Mavlink-Channel
 */

lcm_t *lcm;

ros::Subscriber mavlink_sub;
ros::Subscriber pose_sub;
ros::Subscriber poseStamped_sub;

void mavlinkCallback(const lcm_mavlink_ros::Mavlink::ConstPtr& mavlink_ros_msg)
{
	//Check if the message is coming from lcm so that it is not send back
	if (mavlink_ros_msg->fromlcm)
		return;

	/**
	 * Convert lcm_mavlink_ros::Mavlink to mavlink_message_t
	 */
	mavlink_message_t msg;
	createMavlinkFromROS(mavlink_ros_msg, &msg);

	/**
	 * Send mavlink_message to LCM (so that the rest of the MAVConn world can hear us)
	 */
	mavlink_message_t_publish(lcm, "MAVLINK", &msg);

	if (verbose)
		ROS_INFO("Sent Mavlink from ROS to LCM, Message-ID: [%i]", mavlink_ros_msg->msgid);
}

void poseCallback(const geometry_msgs::Pose &pose_msg)
{
	//set timestamp TODO: implement a more intelligent timestamping here...
	uint64_t timestamp = 0;

	//convert quaternion to euler angles
	const btQuaternion quat(pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w);
	const btMatrix3x3 mat(quat);
	double roll, pitch, yaw;
	mat.getEulerYPR(yaw, pitch, roll);

	//send MAVLINK vision position estimate message
	mavlink_message_t msg;
	mavlink_msg_vision_position_estimate_pack(sysid, compid, &msg, timestamp, pose_msg.position.x, pose_msg.position.y, pose_msg.position.z, roll, pitch, yaw);
	mavlink_message_t_publish(lcm, "MAVLINK", &msg);

	if (verbose)
		ROS_INFO("Sent Mavlink vision position estimate message");
}

void poseStampedCallback(const geometry_msgs::PoseStamped &pose_msg)
{
	//set timestamp (get NSec from ROS and convert to us)
	uint64_t timestamp = pose_msg.header.stamp.toNSec() / 1000;

	//convert quaternion to euler angles
	const btQuaternion quat(pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w);
	const btMatrix3x3 mat(quat);
	double roll, pitch, yaw;
	mat.getEulerYPR(yaw, pitch, roll);

	//send MAVLINK vision position estimate message
	mavlink_message_t msg;
	mavlink_msg_vision_position_estimate_pack(sysid, compid, &msg, timestamp, pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z, roll, pitch, yaw);
	mavlink_message_t_publish(lcm, "MAVLINK", &msg);

	if (verbose)
		ROS_INFO("Sent Mavlink vision position estimate message");
}

// Handling Program options
static GOptionEntry entries[] =
{
		{ "sysid", 'a', 0, G_OPTION_ARG_INT, &sysid, "ID of this system, 1-255", "42" },
		{ "compid", 'c', 0, G_OPTION_ARG_INT, &compid, "ID of this component, 1-255", "199" },
		{ "lcmurl", 'l', 0, G_OPTION_ARG_STRING, &lcmurl, "LCM Url to connect to", "udpm://" },
		{ "verbose", 'v', 0, G_OPTION_ARG_NONE, &verbose, "Be verbose", NULL },
		{ NULL, 0, 0, G_OPTION_ARG_NONE, NULL, NULL, 0 } };

int main(int argc, char **argv)
{
	ros::init(argc, argv, "rostolcm");

	GError *error = NULL;
	GOptionContext *context;

	context = g_option_context_new("- translate MAVLink messages from ROS to LCM");
	g_option_context_add_main_entries(context, entries, NULL);
	//g_option_context_add_group (context, NULL);
	if (!g_option_context_parse(context, &argc, &argv, &error))
	{
		g_print("Option parsing failed: %s\n", error->message);
		exit(1);
	}

	ros::NodeHandle mavlink_nh;
	mavlink_sub = mavlink_nh.subscribe("/toMAVLINK", 1000, mavlinkCallback);

	ros::NodeHandle pose_nh;
	pose_sub = pose_nh.subscribe("/toMAVLINK/bodyPose", 10, poseCallback);

	ros::NodeHandle poseStamped_nh;
	poseStamped_sub = poseStamped_nh.subscribe("/toMAVLINK/bodyPoseStamped", 10, poseStampedCallback);

	/**
	 * Connect to LCM Channel and register for MAVLink messages ->
	 */
	lcm = lcm_create(lcmurl.c_str());
	if (!lcm)
	{
		return 1;
	}

	/**
	 * Now pump callbacks (execute mavlinkCallback) until CTRL-c is pressed
	 */
	ros::spin();

	lcm_destroy(lcm);

	return 0;
}
