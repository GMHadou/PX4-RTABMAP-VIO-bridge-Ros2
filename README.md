## PX4 VIO using NVIDIA Isaac ROS Visual SLAM
Bridge node between RTABMAP-ROS2 [RTABMAP-ROS2]((https://github.com/introlab/rtabmap_ros/tree/ros2#rtabmap_ros))
and PX4 using the [PX4-ROS2 DDS Bridge](https://docs.px4.io/main/en/middleware/uxrce_dds.html)

| Subscribed Topics | Interface |
| --------- | --------- |
| `/odom` | [`nav_msgs/Odometry`] | 
| `/fmu/out/sensor_combined` | [`px4_msgs/sensor_combined`](https://github.com/PX4/px4_msgs/blob/main/msg/SensorCombined.msg) |

| Published Topics | Interface |
| --------- | --------- |
| `/fmu/in/vehicle_visual_odometry` | [`px4_msgs/VehicleOdometry`](https://github.com/PX4/px4_msgs/blob/main/msg/VehicleOdometry.msg) |
| `/vio_transform/imu` | [`sensor_msgs/Imu`](https://docs.ros2.org/humble/api/sensor_msgs/msg/Imu.html) |
