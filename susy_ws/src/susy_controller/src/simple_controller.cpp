#include "susy_controller/simple_controller.hpp"
#include<tf2/LinearMath/Quaternion.h>
#include<Eigen/Geometry>
const double Z = 0.04;
using std::placeholders::_1;

SimpleController::SimpleController(const std::string &name) : Node(name) , left_wheel_prev_pos_(0.0), right_wheel_prev_pos_(0.0), X(0.0), Y(0.0), O(0.0){ 
    declare_parameter("wheel_radius", Z*3.2);
    declare_parameter("wheel_separation", Z*21);

    wheel_radius_ = get_parameter("wheel_radius").as_double();
    wheel_separation_ = get_parameter("wheel_separation").as_double();

    RCLCPP_INFO_STREAM(get_logger(), "Radio: " << wheel_radius_);
    RCLCPP_INFO_STREAM(get_logger(), "Separación entre las ruedas: " << wheel_separation_);

    prev_time_ = get_clock()->now();

    wheel_cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/simple_velocity_controller/commands", 10);
    vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>("/susy_controller/cmd_vel", 10,  
                    std::bind(&SimpleController::velCallback, this, _1));
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, 
        std::bind(&SimpleController::jointCallback, this, _1) );

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/susy_controller/odom", 10);

    

    speed_conversion_<< wheel_radius_/2.0, wheel_radius_/2.0, wheel_radius_/wheel_separation_, -wheel_radius_/wheel_separation_;
    RCLCPP_INFO_STREAM(get_logger(), "La matriz es: " << speed_conversion_);

    odom_msg_.header.frame_id = "odom";
    odom_msg_.child_frame_id = "base_footprint";
    odom_msg_.pose.pose.orientation.x = 0.0;
    odom_msg_.pose.pose.orientation.y = 0.0;
    odom_msg_.pose.pose.orientation.z = 0.0;
    odom_msg_.pose.pose.orientation.w = 1.0;

    tb_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    ts_.header.frame_id = "odom";
    ts_.child_frame_id = "base_footprint";

}

void SimpleController::velCallback(const geometry_msgs::msg::TwistStamped &msg){
    Eigen::Vector2d robot_speed(msg.twist.linear.x, msg.twist.angular.z);

    Eigen::Vector2d wheel_speed = speed_conversion_.inverse() * robot_speed;
 
    std_msgs::msg::Float64MultiArray wheel_speed_msg;
    wheel_speed_msg.data.push_back(wheel_speed.coeff(1));
    wheel_speed_msg.data.push_back(wheel_speed.coeff(0));

    wheel_cmd_pub_->publish(wheel_speed_msg);
    return;
}


void SimpleController::jointCallback(const sensor_msgs::msg::JointState &msg){
    double dp_left = msg.position.at(1) - left_wheel_prev_pos_;
    double dp_right = msg.position.at(0) - right_wheel_prev_pos_;

    rclcpp::Time current_time = msg.header.stamp;
    rclcpp::Duration dt = current_time - prev_time_;

    left_wheel_prev_pos_ = msg.position.at(1);
    right_wheel_prev_pos_ = msg.position.at(0);
    prev_time_ = current_time;

    double fi_left = dp_left / dt.seconds();
    double fi_right = dp_right / dt.seconds();

    double linear_vel =  (wheel_radius_ * fi_right + wheel_radius_ * fi_left) / 2.0;
    double angular_vel =   (wheel_radius_ * fi_right - wheel_radius_ * fi_left) / wheel_separation_;
    
    // RCLCPP_INFO_STREAM(get_logger(), "Velocidad linear: " << linear_vel << "\nVelocidad angular: " << angular_vel);


    double d_s = (wheel_radius_ * dp_right + wheel_radius_ * dp_left)/2.0;
    double d_O = (wheel_radius_ * dp_right - wheel_radius_ * dp_left)/wheel_separation_;

    O += d_O;
    X += d_s * cos(O);
    Y += d_s * sin(O);

    // RCLCPP_INFO_STREAM(get_logger(), "Posicion actual: " << X << " " << Y << ": " << O);

    tf2::Quaternion q;
    q.setRPY(0,0,O);
    odom_msg_.pose.pose.orientation.x = q.x();
    odom_msg_.pose.pose.orientation.y = q.y();
    odom_msg_.pose.pose.orientation.z = q.z();
    odom_msg_.pose.pose.orientation.w = q.w();
    odom_msg_.header.stamp = get_clock()->now();

    odom_msg_.pose.pose.position.x = X;
    odom_msg_.pose.pose.position.y = Y;

    odom_msg_.twist.twist.linear.x = linear_vel;
    odom_msg_.twist.twist.angular.z = angular_vel;
    odom_pub_->publish(odom_msg_);

    ts_.transform.translation.x = X;
    ts_.transform.translation.y = Y;
    ts_.transform.rotation.x = q.x();
    ts_.transform.rotation.y = q.y();
    ts_.transform.rotation.z = q.z();
    ts_.transform.rotation.w = q.w();
    ts_.header.stamp = get_clock()->now();

    tb_->sendTransform(ts_);

    return;
}

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleController>("simple_controller");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}