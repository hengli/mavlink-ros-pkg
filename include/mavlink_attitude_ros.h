#include "mavlink.h"
#include "tf/transform_broadcaster.h"
#include "sensor_msgs/Imu.h"

static inline void convertMavlinkAttitudeToROS(const mavlink_message_t *msg, sensor_msgs::Imu &imu_msg)
{
	mavlink_attitude_t amsg;
	mavlink_msg_attitude_decode(msg, &amsg);

	btQuaternion quat;
	quat.setRPY(amsg.roll, amsg.pitch, amsg.yaw);
	imu_msg.orientation.x = quat.x();
	imu_msg.orientation.y = quat.y();
	imu_msg.orientation.z = quat.z();
	imu_msg.orientation.w = quat.w();
	imu_msg.angular_velocity.x = amsg.rollspeed;
	imu_msg.angular_velocity.y = amsg.pitchspeed;
	imu_msg.angular_velocity.z = amsg.yawspeed;
}
