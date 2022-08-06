#include "twist_to_motors.h"

TwistToMotors::TwistToMotors(ros::NodeHandle& nh)
: m_nh(nh),m_left(0), m_right(0), m_dx(0), m_dy(0), m_dr(0), m_tick_counter(4), m_ticks_since_target(0) {
    ROS_INFO("TwistToMotors launching");
    m_ticks_since_target = m_tick_counter;
    ros::Duration update_freq(1.0/kRate);
    ROS_INFO("TwistToMotors launced");
}

void TwistToMotors::spin(const ros::TimerEvent&) {
    ros::Rate idle(10);

    if (ros::ok()) {
        if (m_ticks_since_target < m_tick_counter) {
            write();
            m_ticks_since_target++;
        }
        idle.sleep();
    }
}

void TwistToMotors::write() {
  unsigned char wbuff[3];
  if (1) {//m_joint_cmd[0] != m_left_last_cmd || m_joint_cmd[1] != m_right_last_cmd) {
    int l_cmd_rpm = (int)((1.0 * m_dx - m_dr * kBaseWidth / 2) *60.0/0.25);
    int r_cmd_rpm = (int)((1.0 * m_dx + m_dr * kBaseWidth / 2) *60.0/0.25);
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
}

void TwistToMotors::twistCallback(const geometry_msgs::Twist& msg) {
    m_ticks_since_target = 0;
    m_dx = msg.linear.x;
    m_dr = msg.angular.z;
    m_dy = msg.linear.y;
}