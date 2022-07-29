#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>

#include "i2c_ros.h"

class RobotHardwareInterface: public hardware_interface::RobotHW {
  public:
    RobotHardwareInterface(ros::NodeHandle& nh);
    ~RobotHardwareInterface() {}
    void init();
    void read();
    void update(const ros::TimerEvent& e);
    void write(ros::Duration elapsed_time);
	     
  protected:
    hardware_interface::JointStateInterface m_joint_state_interface;
    hardware_interface::VelocityJointInterface m_velocity_joint_interface;
    joint_limits_interface::VelocityJointSaturationInterface m_velocity_joint_saturation_interface;

    std::string m_joint_names[2] = {"left_wheel_joint", "right_wheel_joint"};
    double m_joint_pos[2];
    double m_joint_vel[2];
    double m_joint_eff[2];
    double m_joint_cmd[2];

    double m_left_motor_pos=0, m_right_motor_pos=0;
    int m_left_last_cmd=0, m_right_last_cmd=0;
    i2c_ros::I2C m_motors_control=i2c_ros::I2C(0, 0x08);

    ros::NodeHandle m_nh;
    ros::Timer m_non_realtime_loop;
    ros::Duration m_elapsed_time;
    double m_loop_hz;
    std::shared_ptr<controller_manager::ControllerManager> m_controller_manager;
};
