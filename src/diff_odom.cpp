#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/int16.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "roboteq_interfaces/msg/channel_values.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <math.h>
#include <chrono>


class OdometryCalc : public rclcpp::Node
{
public:
    OdometryCalc() : Node("odometry_calc", rclcpp::NodeOptions()), odom_broadcaster(this) {
        initVariables();
        RCLCPP_INFO(this->get_logger(), "Started odometry computing node");

        l_wheel_sub = this->create_subscription<roboteq_interfaces::msg::ChannelValues>(
            "/hall_count", 1000, std::bind(&OdometryCalc::leftEncoderCallback, this, std::placeholders::_1)
        );

        r_wheel_sub = this->create_subscription<roboteq_interfaces::msg::ChannelValues>(
            "/hall_count", 1000, std::bind(&OdometryCalc::rightEncoderCallback, this, std::placeholders::_1)
        );

        odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odom", 50);
    }

private:
    rclcpp::Subscription<roboteq_interfaces::msg::ChannelValues>::SharedPtr l_wheel_sub;
    rclcpp::Subscription<roboteq_interfaces::msg::ChannelValues>::SharedPtr r_wheel_sub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    tf2_ros::TransformBroadcaster odom_broadcaster;

    //std::chrono::nanoseconds t_delta = std::chrono::nanoseconds(0);
    rclcpp::Time t = Node::now();
    double seconds = t.seconds();
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

    void initVariables()
    {
        prev_lencoder = 0;
        prev_rencoder = 0;
        lmult = 0;
        rmult = 0;
        left = 0;
        right = 0;
        encoder_min = -2147483648;
        encoder_max = 2147483648;
        rate = 10;
        ticks_meter = 400;
        base_width = 0.8786;
        encoder_low_wrap = (encoder_max - encoder_min) * 0.3 + encoder_min;
        encoder_high_wrap = (encoder_max - encoder_min) * 0.7 + encoder_min;
        rclcpp::Time t(static_cast<uint64_t>(seconds * 1e9));
        rclcpp::Time t_now = this->now();
        rclcpp::Duration duration = rclcpp::Duration::from_seconds(1.0);
        rclcpp::Time t_next = t_now + duration;
        then = this->now();
        enc_left = 0;
        enc_right = 0;
        dx = 0;
        dr = 0;
        x_final = 0;
        y_final = 0;
        theta_final = 0;
        current_time = this->now();
        last_time = this->now();
    }

    void leftEncoderCallback(const roboteq_interfaces::msg::ChannelValues::SharedPtr left_ticks)
    {
        double enc = left_ticks->value[1];

        if ((enc < encoder_low_wrap) && (prev_lencoder > encoder_high_wrap))
        {
            lmult = lmult + 1;
        }

        if ((enc > encoder_high_wrap) && (prev_lencoder < encoder_low_wrap))
        {
            lmult = lmult - 1;
        }

        left = 1.0 * (enc + lmult * (encoder_max - encoder_min));
        prev_lencoder = enc;
    }

    void rightEncoderCallback(const roboteq_interfaces::msg::ChannelValues::SharedPtr right_ticks)
    {
        double enc = right_ticks->value[2];

        if ((enc < encoder_low_wrap) && (prev_rencoder > encoder_high_wrap))
        {
            rmult = rmult + 1;
        }

        if ((enc > encoder_high_wrap) && (prev_rencoder < encoder_low_wrap))
        {
            rmult = rmult - 1;
        }

        right = 1.0 * (enc + rmult * (encoder_max - encoder_min));
        prev_rencoder = enc;
    }

    void update()
    {
        rclcpp::Time t(static_cast<uint64_t>(seconds * 1e9));
        rclcpp::Time now = this->now();
        double elapsed = now.seconds() - then.seconds();
        

        if (now > t_next)
        {
            double d_left, d_right, d, th, x, y;

            if (enc_left == 0)
            {
                d_left = 0;
                d_right = 0;
            }
            else
            {
                d_left = (left - enc_left) / (ticks_meter);
                d_right = (right - enc_right) / (ticks_meter);
            }

            enc_left = left;
            enc_right = right;

            d = (d_left + d_right) / 2.0;
            th = (d_right - d_left) / base_width;
            dx = d / elapsed;
            dr = th / elapsed;

            if (d != 0)
            {
                x = cos(th) * d;
                y = -sin(th) * d;
                x_final = x_final + (cos(th) * x - sin(th) * y);
                y_final = y_final + (sin(th) * x + cos(th) * y);
            }

            if (th != 0)
            {
                theta_final = theta_final + th;
            }

            geometry_msgs::msg::Quaternion odom_quat;
            odom_quat.x = 0.0;
            odom_quat.y = 0.0;
            odom_quat.z = sin(th / 2);
            odom_quat.w = cos(th / 2);

            geometry_msgs::msg::TransformStamped odom_trans;
            odom_trans.header.stamp = now;
            odom_trans.header.frame_id = "odom";
            odom_trans.child_frame_id = "base_link";
            odom_trans.transform.translation.x = x_final;
            odom_trans.transform.translation.y = y_final;
            odom_trans.transform.translation.z = 0.0;
            odom_trans.transform.rotation = odom_quat;
            std::vector<geometry_msgs::msg::TransformStamped> transforms;
            transforms.push_back(odom_trans);
            odom_broadcaster.sendTransform(transforms);


            nav_msgs::msg::Odometry odom;
            odom.header.stamp = now;
            odom.header.frame_id = "odom";
            odom.pose.pose.position.x = x_final;
            odom.pose.pose.position.y = y_final;
            odom.pose.pose.position.z = 0.0;
            odom.pose.pose.orientation = odom_quat;
            odom.child_frame_id = "base_link";
            odom.twist.twist.linear.x = dx;
            odom.twist.twist.linear.y = 0;
            odom.twist.twist.angular.z = dr;
            odom_pub->publish(odom);
            then = now;
        }
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
