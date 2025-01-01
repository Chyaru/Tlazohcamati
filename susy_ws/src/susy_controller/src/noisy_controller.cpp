#include "susy_controller/noisy_controller.hpp"
#include<tf2/LinearMath/Quaternion.h>
#include<Eigen/Geometry>
#include <random>
const double Z = 0.04;
using std::placeholders::_1;

NoisyController::NoisyController(const std::string &name) : Node(name) , left_wheel_prev_pos_(0.0), right_wheel_prev_pos_(0.0), X(0.0), Y(0.0), O(0.0){ 
    declare_parameter("wheel_radius", Z*3.2);
    declare_parameter("wheel_separation", Z*21);

    wheel_radius_ = get_parameter("wheel_radius").as_double();
    wheel_separation_ = get_parameter("wheel_separation").as_double();

    RCLCPP_INFO_STREAM(get_logger(), "Radio: " << wheel_radius_);
    RCLCPP_INFO_STREAM(get_logger(), "Separación entre las ruedas: " << wheel_separation_);

    prev_time_ = get_clock()->now();

    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, 
        std::bind(&NoisyController::jointCallback, this, _1) );

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/susy_controller/odom_noisy", 10);

    

    odom_msg_.header.frame_id = "odom";
    odom_msg_.child_frame_id = "base_footprint_efk";
    odom_msg_.pose.pose.orientation.x = 0.0;
    odom_msg_.pose.pose.orientation.y = 0.0;
    odom_msg_.pose.pose.orientation.z = 0.0;
    odom_msg_.pose.pose.orientation.w = 1.0;

    tb_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    ts_.header.frame_id = "odom";
    ts_.child_frame_id = "base_footprint_noisy";

}



void NoisyController::jointCallback(const sensor_msgs::msg::JointState &msg){

    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine noise_generator(seed);
    std::normal_distribution<double> left_encoder_noise(0.0,0.005);
    std::normal_distribution<double> right_encoder_noise(0.0,0.005);

    double wheel_controller_left = msg.position.at(1) + left_encoder_noise(noise_generator);
    double wheel_controller_right = msg.position.at(0) + right_encoder_noise(noise_generator);
    
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
    auto node = std::make_shared<NoisyController>("noisy_controller");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}