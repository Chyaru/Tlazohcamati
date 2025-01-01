#include<rclcpp/rclcpp.hpp>
#include<geometry_msgs/msg/twist_stamped.hpp>
#include<std_msgs/msg/float64_multi_array.hpp>
#include<sensor_msgs/msg/joint_state.hpp>
#include<nav_msgs/msg/odometry.hpp>
#include<Eigen/Core>
#include<tf2_ros/transform_broadcaster.h>
#include<geometry_msgs/msg/transform_stamped.hpp>

class NoisyController  : public rclcpp::Node {
public:
    NoisyController(const std::string &name);

private:

    double wheel_radius_;
    double wheel_separation_;


    void jointCallback(const sensor_msgs::msg::JointState &msg);
    double left_wheel_prev_pos_;
    double right_wheel_prev_pos_;
    rclcpp::Time prev_time_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;

    double X;
    double Y;
    double O;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    nav_msgs::msg::Odometry odom_msg_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tb_;
    geometry_msgs::msg::TransformStamped ts_;

};
