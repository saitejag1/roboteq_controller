#include "rclcpp/rclcpp.hpp"
#include "roboteq_interfaces/msg/channel_values.hpp"
#include "std_msgs/msg/header.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <math.h>

class OdometryCalc : public rclcpp::Node
{
public:
    OdometryCalc() : Node("odometry_calc"), odom_broadcaster(this)
    {
        initVariables();

        odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        l_wheel_sub = this->create_subscription<roboteq_interfaces::msg::ChannelValues>(
            "/hall_count", 10, std::bind(&OdometryCalc::leftEncoderCallback, this, std::placeholders::_1)
        );

        r_wheel_sub = this->create_subscription<roboteq_interfaces::msg::ChannelValues>(
            "/hall_count", 10, std::bind(&OdometryCalc::rightEncoderCallback, this, std::placeholders::_1)
        );

        publish_timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&OdometryCalc::publishOdometry, this));
    }

private:
    rclcpp::Subscription<roboteq_interfaces::msg::ChannelValues>::SharedPtr l_wheel_sub;
    rclcpp::Subscription<roboteq_interfaces::msg::ChannelValues>::SharedPtr r_wheel_sub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    tf2_ros::TransformBroadcaster odom_broadcaster;

    double prev_lencoder;
    double prev_rencoder;
    double left;
    double right;
    double ticks_meter;
    double base_width;

    rclcpp::Time then;
    double x, y, theta;

    rclcpp::TimerBase::SharedPtr publish_timer;

    void initVariables()
    {
        prev_lencoder = 0;
        prev_rencoder = 0;
        left = 0;
        right = 0;
        ticks_meter = 400;
        base_width = 0.8786;
        x = 0;
        y = 0;
        theta = 0;
        then = this->now();
    }

    void leftEncoderCallback(const roboteq_interfaces::msg::ChannelValues::SharedPtr left_ticks)
    {
        double enc = left_ticks->value[0];
        left = 1.0 * enc;
    }

    void rightEncoderCallback(const roboteq_interfaces::msg::ChannelValues::SharedPtr right_ticks)
    {
        double enc = right_ticks->value[0];
        right = 1.0 * enc;
    }

    void publishOdometry()
    {
        rclcpp::Time now = this->now();
        double elapsed = (now - then).seconds();
        
        double d_left = (left - prev_lencoder) / ticks_meter;
        double d_right = (right - prev_rencoder) / ticks_meter;

        double d = (d_left + d_right) / 2.0;
        double th = (d_right - d_left) / base_width;
        double dx = d * cos(theta);
        double dy = d * sin(theta);

        x += dx;
        y += dy;
        theta += th;

        geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, theta)));

        geometry_msgs::msg::TransformStamped odom_trans;
        odom_trans.header.stamp = now;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;
        odom_broadcaster.sendTransform(odom_trans);

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = now;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;
        odom.twist.twist.linear.x = dx / elapsed;
        odom.twist.twist.linear.y = 0;
        odom.twist.twist.angular.z = th / elapsed;
        odom_pub->publish(odom);

        prev_lencoder = left;
        prev_rencoder = right;
        then = now;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdometryCalc>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
