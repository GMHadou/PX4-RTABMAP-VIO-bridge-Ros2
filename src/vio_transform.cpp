#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Transform.h"

class VioTransform : public rclcpp::Node
{
public:
explicit VioTransform() : Node("vio_transform")
{
	_vio_pub = this->create_publisher<px4_msgs::msg::VehicleOdometry>("/fmu/in/vehicle_visual_odometry", 10);

	// QoS profile, PX4 specific
	rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
	auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

	_vslam_sub = this->create_subscription<nav_msgs::msg::Odometry>("/visual_slam/tracking/odometry", qos,
		std::bind(&VioTransform::publish, this, std::placeholders::_1));
}

private:
	void publish(const nav_msgs::msg::Odometry::UniquePtr msg);

	rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr _vio_pub;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _vslam_sub;
};

void VioTransform::publish(const nav_msgs::msg::Odometry::UniquePtr msg) 
{
	px4_msgs::msg::VehicleOdometry vio;

	vio.timestamp = msg->header.stamp.sec * 1000000 + msg->header.stamp.nanosec / 1000;
	vio.timestamp_sample = vio.timestamp;

	vio.pose_frame = vio.POSE_FRAME_FRD; // FRD world-fixed frame, arbitrary heading reference

	// Create vectors for position and orientation so we can rotate them into the correct frame 
	tf2::Vector3 p;
	p.setX(msg->pose.pose.position.x);
	p.setY(msg->pose.pose.position.y);
	p.setZ(msg->pose.pose.position.z);

	tf2::Quaternion q;
	q.setX(msg->pose.pose.orientation.x);
	q.setY(msg->pose.pose.orientation.y);
	q.setZ(msg->pose.pose.orientation.z);
	q.setW(msg->pose.pose.orientation.w);


	// ROS --> PX4

	tf2::Quaternion enu_ned_q;;
	enu_ned_q.setRPY(M_PI, 0.0, M_PI_2);
	// tf2::Quaternion flu_frd_q;;
	// flu_frd_q.setRPY(M_PI, 0.0, 0.0);

	// TODO: camera is mounted upside down, param to specify orientation?
	// q = enu_ned_q * q * flu_frd_q;
	q = enu_ned_q * q; // camera is mounted upside down so we're in FRD already
	p = tf2::quatRotate(enu_ned_q, p);

	vio.position[0] = p[0];
	vio.position[1] = p[1];
	vio.position[2] = p[2];

	vio.q[0] = q[0];
	vio.q[1] = q[1];
	vio.q[2] = q[2];
	vio.q[3] = q[3];

	vio.velocity_frame = vio.VELOCITY_FRAME_FRD;

	vio.velocity[0] = msg->twist.twist.linear.x;
	vio.velocity[1] = msg->twist.twist.linear.y;
	vio.velocity[2] = msg->twist.twist.linear.z;

	vio.angular_velocity[0] = msg->twist.twist.angular.x;
	vio.angular_velocity[1] = msg->twist.twist.angular.y;
	vio.angular_velocity[2] = msg->twist.twist.angular.z;

	vio.position_variance[0] = msg->pose.covariance[0];
	vio.position_variance[1] = msg->pose.covariance[7];
	vio.position_variance[2] = msg->pose.covariance[14];
	
	vio.orientation_variance[0] = msg->pose.covariance[21];
	vio.orientation_variance[1] = msg->pose.covariance[28];
	vio.orientation_variance[2] = msg->pose.covariance[35];

	vio.velocity_variance[0] = msg->twist.covariance[0];
	vio.velocity_variance[1] = msg->twist.covariance[7];
	vio.velocity_variance[2] = msg->twist.covariance[14];

	vio.reset_counter = 0; // TODO: look into issac_ros_vslam code to see if we can expose it
	vio.quality = 0; // 0 = unknown/unset quality

	_vio_pub->publish(vio);
}

int main(int argc, char *argv[])
{
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<VioTransform>());
	rclcpp::shutdown();
	return 0;
}
