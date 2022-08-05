#include <diff_drive_controller/diff_drive_controller.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include "robot_driver.h"

using diff_drive_controller::DiffDriveController;
using hardware_interface::JointStateHandle;
using hardware_interface::JointHandle;
using hardware_interface::VelocityJointInterface;
using joint_limits_interface::VelocityJointSaturationInterface;
using joint_limits_interface::VelocityJointSaturationHandle;

RobotHardwareInterface::RobotHardwareInterface(ros::NodeHandle& nh): m_nh(nh) {
  init();

  m_controller_manager.reset(new controller_manager::ControllerManager(this, m_nh));
  m_loop_hz = 10;
  ros::Duration update_freq(1.0/m_loop_hz);
  m_non_realtime_loop = m_nh.createTimer(update_freq, &RobotHardwareInterface::update, this);
}

void RobotHardwareInterface::init() {
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
}

void RobotHardwareInterface::update(const ros::TimerEvent& e) {
  m_elapsed_time = ros::Duration(e.current_real - e.last_real);
  read();
  m_controller_manager->update(ros::Time::now(), m_elapsed_time);
  write(m_elapsed_time);
}

void RobotHardwareInterface::read() {
  unsigned char rbuff[8];
  int pos;
  m_motors_control.readBytes(rbuff, 8);
  pos = rbuff[3];
  for (int i=2;i>=0;--i) {
    pos = pos << 8;
    pos = pos | rbuff[i];
  }
  //m_left_motor_pos += pos;
  double pos_p = (pos/618.75)*0.2513274; 
  m_joint_pos[0] = pos_p;

  pos = rbuff[7];
  for (int i=6;i>=4;--i) {
    pos = pos << 8;
    pos = pos | rbuff[i];
  }
  //m_right_motor_pos += pos;
  pos_p = (pos/618.75)*0.2513274; 
  m_joint_pos[1] = pos_p;
}

void RobotHardwareInterface::write(ros::Duration elapsed_time) {
  m_velocity_joint_saturation_interface.enforceLimits(elapsed_time);
  unsigned char wbuff[3];
  if (1) {//m_joint_cmd[0] != m_left_last_cmd || m_joint_cmd[1] != m_right_last_cmd) {
    int l_cmd_rpm = m_joint_cmd[0]*60.0/0.25;
    int r_cmd_rpm = m_joint_cmd[1]*60.0/0.25;
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
    m_left_last_cmd = m_joint_cmd[0];;    
    m_right_last_cmd = m_joint_cmd[1];
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "robot_driver");
  ros::NodeHandle nh;
  ros::MultiThreadedSpinner spinner(2);
  RobotHardwareInterface robot(nh);
  spinner.spin();
  return 0;
}
