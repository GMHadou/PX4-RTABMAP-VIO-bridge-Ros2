#include <rclcpp/rclcpp.hpp>
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Transform.h"
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

class VioTransform : public rclcpp::Node
{
public:
	explicit VioTransform() : Node("vio_transform")
	{
		// QoS profile, PX4 specific
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		// PX4 formatted VIO publisher
		_vio_pub = this->create_publisher<px4_msgs::msg::VehicleOdometry>("/fmu/in/vehicle_visual_odometry", 10);

		// ROS2 formatted IMU publisher
		_imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("/vio_transform/imu", 10);

		// RTAB-Map subscriptions
		_vslam_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom", qos,
			std::bind(&VioTransform::odometryCallback, this, std::placeholders::_1));

		// FC IMU subscription
		_fc_imu_sub = this->create_subscription<px4_msgs::msg::SensorCombined>("/fmu/out/sensor_combined", qos,
			std::bind(&VioTransform::sensorCombinedCallback, this, std::placeholders::_1));

		// Timer for 10 Hz
		_timer = this->create_wall_timer(
			std::chrono::milliseconds(100), // 10 Hz
			std::bind(&VioTransform::timerCallback, this));
	}

private:
	// Subscription callbacks
	void odometryCallback(const nav_msgs::msg::Odometry::UniquePtr msg);
	void sensorCombinedCallback(const px4_msgs::msg::SensorCombined::UniquePtr msg);
	void timerCallback();

	// Publishers
	rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr _vio_pub;
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr _imu_pub;

	// Subscribers
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _vslam_odom_sub;
	rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr _fc_imu_sub;

	// Timer
	rclcpp::TimerBase::SharedPtr _timer;

	// Variables to store latest messages
	std::unique_ptr<nav_msgs::msg::Odometry> _latest_odom;
	std::unique_ptr<px4_msgs::msg::SensorCombined> _latest_sensor_combined;
};

void VioTransform::sensorCombinedCallback(const px4_msgs::msg::SensorCombined::UniquePtr msg)
{
	_latest_sensor_combined = std::make_unique<px4_msgs::msg::SensorCombined>(*msg);
}

void VioTransform::odometryCallback(const nav_msgs::msg::Odometry::UniquePtr msg)
{
	_latest_odom = std::make_unique<nav_msgs::msg::Odometry>(*msg);
}

void VioTransform::timerCallback()
{
	if (_latest_sensor_combined)
	{
		auto msg = std::move(_latest_sensor_combined);
		auto fc_imu_acc = tf2::Vector3(msg->accelerometer_m_s2[0], msg->accelerometer_m_s2[1], msg->accelerometer_m_s2[2]);
		auto fc_imu_gyro = tf2::Vector3(msg->gyro_rad[0], msg->gyro_rad[1], msg->gyro_rad[2]);

		auto imu_msg = sensor_msgs::msg::Imu();
		imu_msg.header.stamp = get_clock()->now();
		imu_msg.linear_acceleration.x = fc_imu_acc[0];
		imu_msg.linear_acceleration.y = fc_imu_acc[1];
		imu_msg.linear_acceleration.z = fc_imu_acc[2];
		imu_msg.angular_velocity.x = fc_imu_gyro[0];
		imu_msg.angular_velocity.y = fc_imu_gyro[1];
		imu_msg.angular_velocity.z = fc_imu_gyro[2];
		imu_msg.orientation_covariance[0] = -1.0;
		_imu_pub->publish(imu_msg);
	}

	if (_latest_odom)
	{
		auto msg = std::move(_latest_odom);
		tf2::Vector3 position(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
		tf2::Quaternion quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
		tf2::Vector3 velocity(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
		tf2::Vector3 angular_velocity(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);

		tf2::Quaternion rotation;
		rotation.setRPY(M_PI, 0.0, 0.0);

		position = tf2::quatRotate(rotation, position);
		quaternion = rotation * quaternion * rotation.inverse();
		velocity = tf2::quatRotate(rotation, velocity);
		angular_velocity = tf2::quatRotate(rotation, angular_velocity);

		// Fill the message
		px4_msgs::msg::VehicleOdometry vio;
		vio.timestamp = msg->header.stamp.sec * 1000000 + msg->header.stamp.nanosec / 1000;
		vio.timestamp_sample = vio.timestamp;
		vio.pose_frame = vio.POSE_FRAME_FRD;

		vio.q[0] = quaternion.getW();
		vio.q[1] = quaternion.getX();
		vio.q[2] = quaternion.getY();
		vio.q[3] = quaternion.getZ();

		vio.position[0] = position.getX();
		vio.position[1] = position.getY();
		vio.position[2] = position.getZ();

		vio.velocity_frame = vio.VELOCITY_FRAME_BODY_FRD;
		vio.velocity[0] = velocity.getX();
		vio.velocity[1] = velocity.getY();
		vio.velocity[2] = velocity.getZ();

		vio.angular_velocity[0] = angular_velocity.getX();
		vio.angular_velocity[1] = angular_velocity.getY();
		vio.angular_velocity[2] = angular_velocity.getZ();

		_vio_pub->publish(vio);
	}
}

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<VioTransform>());
	rclcpp::shutdown();
	return 0;
}
