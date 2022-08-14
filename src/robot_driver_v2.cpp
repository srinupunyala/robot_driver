#include "robot_driver_v2.h"

#include <ros/console.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

using hardware_interface::JointStateHandle;
using hardware_interface::JointHandle;
using hardware_interface::VelocityJointInterface;
using joint_limits_interface::VelocityJointSaturationInterface;
using joint_limits_interface::VelocityJointSaturationHandle;

long RobotDriverV2::kEncoderLowWrap = (kEncoderMax - kEncoderMin) * 0.3 + kEncoderMin;
long RobotDriverV2::kEncoderHighWrap = (kEncoderMax - kEncoderMin) * 0.7 + kEncoderMin;

RobotDriverV2::RobotDriverV2(ros::NodeHandle& nh): m_nh(nh), m_tf_broadcaster(tf::TransformBroadcaster()),
        m_base_frame_id("base_link"), m_odom_frame_id("odom"), m_left_last_pos(0), m_right_last_pos(0),
        m_left_enc_pos(0), m_right_enc_pos(0), m_last_left_enc_pos(0), m_last_right_enc_pos(0), m_left_enc_mult(0),
        m_right_enc_mult(0), m_x(0), m_y(0), m_th(0), m_left(0), m_right(0), m_dx(0), m_dy(0), m_dr(0),
        m_tick_counter(4), m_ticks_since_target(m_tick_counter) {
    m_odom_publisher = m_nh.advertise<nav_msgs::Odometry>("odom", 10);
    m_cmd_vel_subscriber = m_nh.subscribe("cmd_vel", 1000, &RobotDriverV2::twistCallback, this);

    for (int i=0; i<2; ++i) {
        JointStateHandle joint_state_handle(m_joint_names[i], &m_joint_pos[i], &m_joint_vel[i], &m_joint_eff[i]);
        m_joint_state_interface.registerHandle(joint_state_handle);

        JointHandle joint_velocity_handle(joint_state_handle, &m_joint_cmd[i]);
        m_velocity_joint_interface.registerHandle(joint_velocity_handle);

        joint_limits_interface::JointLimits limits;
        joint_limits_interface::getJointLimits(m_joint_names[i], m_nh, limits);
        VelocityJointSaturationHandle joint_limits_handle(joint_velocity_handle, limits);
        m_velocity_joint_saturation_interface.registerHandle(joint_limits_handle);
    }
    registerInterface(&m_joint_state_interface);
    registerInterface(&m_velocity_joint_interface);
    registerInterface(&m_velocity_joint_saturation_interface);

    m_controller_manager.reset(new controller_manager::ControllerManager(this, m_nh));

    ros::Duration update_freq(1.0/kRate);
    m_non_realtime_read_loop = m_nh.createTimer(update_freq, &RobotDriverV2::updatePosition, this);
    m_non_realtime_write_loop = m_nh.createTimer(update_freq, &RobotDriverV2::writeCmdToMotors, this);
}

void RobotDriverV2::updatePosition(const ros::TimerEvent& event) {
    readMotorsPosition();

    auto elapsed_time = ros::Duration(event.current_real - event.last_real);
    double elapsed_secs = elapsed_time.toSec();

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

        m_x = m_x + (cos(m_th) * x - sin(m_th) * y);
        m_y = m_y + (sin(m_th) * x + cos(m_th) * y);
    }
    m_joint_pos[0] = m_x;
    m_joint_pos[1] = m_y;
    if (th != 0) {
        m_th = m_th + th;
    }
    publishOdomAndTf();
    m_controller_manager->update(ros::Time::now(), elapsed_time);
}

void RobotDriverV2::publishOdomAndTf() {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(m_th / 2);
    quaternion.w = cos(m_th / 2);

    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
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
        tf::StampedTransform(tfm, ros::Time::now(), m_odom_frame_id, m_base_frame_id));
    m_odom_publisher.publish(odom);
}

void RobotDriverV2::readMotorsPosition() {
    unsigned char rbuff[9];
    int l_pos, r_pos;
    m_motors_control.readBytes(rbuff, 9);
    l_pos = rbuff[3];
    for (int i=2;i>=0;--i) {
        l_pos = l_pos << 8;
        l_pos = l_pos | rbuff[i];
    }
    if (rbuff[8] & 1)
        l_pos *= -1;

    r_pos = rbuff[7];
    for (int i=6;i>=4;--i) {
        r_pos = r_pos << 8;
        r_pos = r_pos | rbuff[i];
    }
    if (rbuff[8] & 2)
        r_pos *= -1;
    l_pos *= -1;  

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

void RobotDriverV2::writeCmdToMotors(const ros::TimerEvent& event) {
    if (m_ticks_since_target < m_tick_counter) {
        ros::Duration elapsed_time = ros::Duration(event.current_real - event.last_real);
        unsigned char wbuff[3];
        m_joint_cmd[0] = (1.0 * m_dx - m_dr * kBaseWidth / 2);
        m_joint_cmd[1] = (1.0 * m_dx + m_dr * kBaseWidth / 2);
        //m_velocity_joint_saturation_interface.enforceLimits(elapsed_time);
        if (m_joint_cmd[0] > 0.5)
            m_joint_cmd[0] = 0.5;
        if (m_joint_cmd[0] < -0.5)
            m_joint_cmd[0] = -0.5;
                
        if (m_joint_cmd[1] > 0.5)
            m_joint_cmd[1] = 0.5;
        if (m_joint_cmd[1] < -0.5)
            m_joint_cmd[1] = -0.5;
        
        if (1) {//m_joint_cmd[0] != m_left_last_cmd || m_joint_cmd[1] != m_right_last_cmd) {
            int l_cmd_rpm = (int)(m_joint_cmd[0] *60.0/0.25);
            int r_cmd_rpm = (int)(m_joint_cmd[1] *60.0/0.25);
            int l_dir = 0;
            int r_dir = 0;
            wbuff[2] = 0;
            if (l_cmd_rpm < 0) {
                l_cmd_rpm *= -1;
            wbuff[2] = 1;
            }
            if (r_cmd_rpm < 0) {
                r_cmd_rpm *= -1;
            wbuff[2] |= 2;
            }
            wbuff[0] = l_cmd_rpm;
            wbuff[1] = r_cmd_rpm;
            m_motors_control.writeData(wbuff, 3);
        }
        m_ticks_since_target++;
    }
}

void RobotDriverV2::twistCallback(const geometry_msgs::Twist& msg) {
    m_ticks_since_target = 0;
    m_dx = msg.linear.x;
    m_dr = msg.angular.z;
    m_dy = msg.linear.y;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "robot_driver");
    ros::NodeHandle nh;
    ros::MultiThreadedSpinner spinner(2);
    RobotDriverV2 robot(nh);
    spinner.spin();
    return 0;
}