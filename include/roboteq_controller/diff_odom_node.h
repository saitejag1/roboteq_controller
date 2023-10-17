#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/String.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "roboteq_motor_controller_driver/msg/channel_values.hpp"
#include <cmath>

class OdometryCalc : public rclcpp::Node
{
public:
    OdometryCalc();

    void spin();

private:
    rclcpp::Node::SharedPtr n;
    rclcpp::Subscription<roboteq_motor_controller_driver::msg::ChannelValues>::SharedPtr l_wheel_sub;
    rclcpp::Subscription<roboteq_motor_controller_driver::msg::ChannelValues>::SharedPtr r_wheel_sub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    tf2_ros::TransformBroadcaster odom_broadcaster;

    // Encoder related variables
    double encoder_min;
    double encoder_max;
    double encoder_low_wrap;
    double encoder_high_wrap;
    double prev_lencoder;
    double prev_rencoder;
    double lmult;
    double rmult;
    double left;
    double right;
    double rate;
    rclcpp::Duration t_delta;
    rclcpp::Time t_next;
    rclcpp::Time then;
    double enc_left;
    double enc_right;
    double ticks_meter;
    double base_width;
    double dx;
    double dr;
    double x_final, y_final, theta_final;
    rclcpp::Time current_time, last_time;

    void leftEncoderCallback(const roboteq_motor_controller_driver::msg::ChannelValues::SharedPtr left_ticks);
    void rightEncoderCallback(const roboteq_motor_controller_driver::msg::ChannelValues::SharedPtr right_ticks);
    void initVariables();
    void update();
};
