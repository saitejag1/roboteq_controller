import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Quaternion
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from math import sin, cos
from roboteq_interfaces.msg import ChannelValues
from geometry_msgs.msg import TransformStamped
import math
from rclpy.qos import QoSProfile

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci * ck
    cs = ci * sk
    sc = si * ck
    ss = si * sk

    q = Quaternion()
    q.x = cj * sc - sj * cs
    q.y = cj * ss + sj * cc
    q.z = cj * cs - sj * sc
    q.w = cj * cc + sj * ss
    return q

class OdometryCalc(Node):

    def __init__(self):
        super().__init__('odometry_calc')

        self.init_variables()
        self.get_logger().info('Started odometry computing node')

        self.l_wheel_sub = self.create_subscription(
            ChannelValues, '/hall_count', self.leftencoder_cb, 10)
        self.r_wheel_sub = self.create_subscription(
            ChannelValues, '/hall_count', self.rightencoder_cb, 10)
        self.odom_pub = self.create_publisher(
            Odometry, 'odom', QoSProfile(depth=10))

        self.odom_broadcaster = TransformBroadcaster(self)

    def init_variables(self):
        self.prev_lencoder = 0
        self.prev_rencoder = 0
        self.lmult = 0
        self.rmult = 0
        self.left = 0
        self.right = 10
        self.encoder_min = -2147483648
        self.encoder_max = 2147483648
        self.rate = 10
        self.ticks_meter = 420
        self.base_width = 0.8786
        self.encoder_low_wrap = (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min
        self.encoder_high_wrap = (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min
        self.t_delta = rclpy.duration.Duration(seconds=1.0 / self.rate)
        self.t_next = self.get_clock().now() + self.t_delta
        self.then = self.get_clock().now()
        self.enc_left = 0
        self.enc_right = 0
        self.dx = 0
        self.dr = 0
        self.x_final = 0.0
        self.y_final = 0.0
        self.theta_final = 0
        self.current_time = self.get_clock().now()
        self.last_time = self.get_clock().now()
        self.initial_left_enc = 0
        self.initial_right_enc = 0

    def spin(self):
        timer_period = 1.0 / self.rate
        timer = self.create_timer(timer_period, self.update)

    def update(self):
        now = self.get_clock().now()
        elapsed = (now - self.then).nanoseconds / 1e9

        if self.enc_left == 0:
            d_left = 0
            d_right = 0
        else:
            d_left = (self.left - self.enc_left) / self.ticks_meter
            d_right = (self.right - self.enc_right) / self.ticks_meter

        self.enc_left = self.left
        self.enc_right = self.right

        d = (d_left + d_right) / 2.0
        th = (d_right - d_left) / self.base_width
        self.dx = d / elapsed
        self.dr = th / elapsed

        if d != 0:
            x = math.cos(th) * d
            y = -math.sin(th) * d
            self.x_final += math.cos(self.theta_final) * x - math.sin(self.theta_final) * y
            self.y_final += math.sin(self.theta_final) * x + math.cos(self.theta_final) * y

        if th != 0:
            self.theta_final += th

        odom_quat = Quaternion()
        odom_quat.x = 0.0
        odom_quat.y = 0.0
        odom_quat.z = math.sin(self.theta_final / 2)
        odom_quat.w = math.cos(self.theta_final / 2)

        odom_trans = TransformStamped()
        odom_trans.header.stamp = now.to_msg()
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = 'base_footprint'
        odom_trans.transform.translation.x = self.x_final
        odom_trans.transform.translation.y = self.y_final
        odom_trans.transform.translation.z = 0.0
        odom_trans.transform.rotation = odom_quat

        self.odom_broadcaster.sendTransform(odom_trans)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.pose.pose.position.x = self.x_final
        odom.pose.pose.position.y = self.y_final
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = odom_quat
        odom.child_frame_id = 'base_footprint'
        odom.twist.twist.linear.x = self.dx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.dr

        self.odom_pub.publish(odom)

        self.then = now


    def leftencoder_cb(self, left_ticks):
        if self.initial_left_enc == 0:
            self.initial_left_enc = left_ticks.value[1]
        enc = left_ticks.value[1] - self.initial_left_enc

        if enc < self.encoder_low_wrap and self.prev_lencoder > self.encoder_high_wrap:
            self.lmult += 1

        if enc > self.encoder_high_wrap and self.prev_lencoder < self.encoder_low_wrap:
            self.lmult -= 1

        self.left = 1.0 * (enc + self.lmult * (self.encoder_max - self.encoder_min))
        self.prev_lencoder = enc + self.encoder_min

    def rightencoder_cb(self, right_ticks):
        if self.initial_right_enc == 0:
            self.initial_right_enc = right_ticks.value[1]
        enc = right_ticks.value[1] - self.initial_right_enc

        if enc < self.encoder_low_wrap and self.prev_rencoder > self.encoder_high_wrap:
            self.rmult += 1

        if enc > self.encoder_high_wrap and self.prev_rencoder < self.encoder_low_wrap:
            self.rmult -= 1

        self.right = 1.0 * (enc + self.rmult * (self.encoder_max - self.encoder_min))
        self.prev_rencoder = enc + self.encoder_min

def main(args=None):
    rclpy.init(args=args)
    odometry_calc = OdometryCalc()
    odometry_calc.spin()
    rclpy.spin(odometry_calc)
    odometry_calc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


