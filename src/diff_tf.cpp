#include "diff_tf.h"
#include "twist_to_motors.h"

#include <ros/console.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>

long OdomPublisher::kEncoderLowWrap = (kEncoderMax - kEncoderMin) * 0.3 + kEncoderMin;
long OdomPublisher::kEncoderHighWrap = (kEncoderMax - kEncoderMin) * 0.7 + kEncoderMin;

OdomPublisher::OdomPublisher(ros::NodeHandle &nh):
        m_tf_broadcaster(tf::TransformBroadcaster()), m_left_enc_mult(0), m_right_enc_mult(0) {
    if (nh.getParam("/base_frame_id", m_base_frame_id)) {
        ROS_FATAL("Failed to get param: base_frame_id");
        return;
    }
    if (nh.getParam("/odom_frame_id", m_odom_frame_id)) {
        ROS_FATAL("Failed to get param: odom_frame_id");
        return;
    }
    m_odom_publisher = nh.advertise<nav_msgs::Odometry>("odom", 10);
    ros::Duration update_freq(1.0/kRate);
    nh.createTimer(update_freq, &OdomPublisher::spin, this);
    ROS_INFO("OdomPublisher launched");
}

void OdomPublisher::spin(const ros::TimerEvent&) {
    if(ros::ok()) {
        read();
        update();
    }
}

void OdomPublisher::update() {
    ros::Time now = ros::Time::now();
    if (now <= m_t_next)
        return;
    double elapsed_secs = (now - m_prev).toSec();
    m_prev = now;

    double d_left=0, d_right=0;
    if (m_left_enc_pos != 0)
        d_left = (m_left_enc_pos - m_left_last_pos) / kTicksPerMeter;
    if (m_right_enc_pos != 0)
        d_right = (m_right_enc_pos - m_right_last_pos) / kTicksPerMeter;
    m_left_last_pos = m_left_enc_pos;
    m_right_last_pos = m_right_enc_pos;

    double dist = (d_left + d_right) / 2;
    double th = (d_right - d_left) / kBaseWidth;

    m_linear_v = dist / elapsed_secs;
    m_angular_v = th / elapsed_secs;

    if (dist != 0) {
        double x = cos(th) * dist;
        double y = -sin(th) * dist;

        m_x = m_x + (cos(th) * x - sin(th) * y);
        m_y = m_y + (sin(th) * x + cos(th) * y);
    }
    if (th != 0) {
        m_th = m_th + th;
    }

    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(m_th / 2);
    quaternion.w = cos(m_th / 2);

    nav_msgs::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = m_odom_frame_id;
    odom.pose.pose.position.x = m_x;
    odom.pose.pose.position.y = m_y;
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation = quaternion;
    odom.child_frame_id = m_base_frame_id;
    odom.twist.twist.linear.x = m_linear_v;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = m_angular_v;

    tf::Transform tfm;
    tfm.setOrigin({m_x, m_y, 0});
    tf::Quaternion tf_quaternion;
    tf::quaternionMsgToTF(quaternion, tf_quaternion);
    tfm.setRotation(tf_quaternion);
    m_tf_broadcaster.sendTransform(
        tf::StampedTransform(tfm, ros::Time::now(), m_base_frame_id, m_odom_frame_id));
    m_odom_publisher.publish(odom);
}

void OdomPublisher::read() {
    unsigned char rbuff[9];
    int l_pos, r_pos;
    m_motors_control.readBytes(rbuff, 9);
    l_pos = rbuff[3];
    for (int i=2;i>=0;--i) {
        l_pos = l_pos << 8;
        l_pos = l_pos | rbuff[i];
    }
    if (rbuff[9] & 1)
        l_pos *= -1;

    r_pos = rbuff[7];
    for (int i=6;i>=4;--i) {
        r_pos = r_pos << 8;
        r_pos = r_pos | rbuff[i];
    }
    if (rbuff[9] & 2)
        r_pos *= -1;

    if (l_pos < kEncoderLowWrap && m_last_left_enc_pos > kEncoderHighWrap)
        m_left_enc_mult += 1;
    if (l_pos > kEncoderHighWrap && m_last_left_enc_pos < kEncoderLowWrap)
        m_left_enc_mult -= 1;
    m_left_enc_pos = 1.0 * (l_pos + m_left_enc_mult * (kEncoderMax - kEncoderMin));
    m_last_left_enc_pos = l_pos;

    if (r_pos < kEncoderLowWrap && m_last_right_enc_pos > kEncoderHighWrap)
        m_right_enc_mult += 1;

    if (r_pos > kEncoderHighWrap && m_last_right_enc_pos < kEncoderLowWrap)
        m_right_enc_mult -= 1;

    m_right_enc_pos = 1.0 * (r_pos + m_right_enc_mult * (kEncoderMax - kEncoderMin));
    m_last_right_enc_pos = r_pos;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "robot_driver");
    ros::NodeHandle nh;
    TwistToMotors tm(nh);
    ros::Subscriber sub = nh.subscribe("cmd_vel", 1000, &TwistToMotors::twistCallback, &tm);
    nh.createTimer(0.03, &TwistToMotors::spin, &tm); 
    OdomPublisher op(nh);
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();
    return 0;
}