#pragma once

#include "i2c_ros.h"

#include "ros/ros.h"
#include "ros/time.h"
#include <tf/transform_broadcaster.h>
#include <controller_manager/controller_manager.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <geometry_msgs/Twist.h>

#include "string"

class RobotDriverV2: public hardware_interface::RobotHW {
    public:
        RobotDriverV2(ros::NodeHandle& nh);

        void updatePosition(const ros::TimerEvent& event);

        void readMotorsPosition();

        void publishOdomTfAndRange();

        void writeCmdToMotors(const ros::TimerEvent& event);

        void publishSonarRange();

        void twistCallback(const geometry_msgs::Twist& msg);
    private:
        ros::NodeHandle &m_nh;
        ros::Publisher m_odom_publisher;
        ros::Publisher m_sonar_range_publisher;
        ros::Subscriber m_cmd_vel_subscriber;
        ros::Timer m_non_realtime_read_loop;
        ros::Timer m_non_realtime_write_loop;

        tf::TransformBroadcaster m_tf_broadcaster;
        std::shared_ptr<controller_manager::ControllerManager> m_controller_manager;
        hardware_interface::JointStateInterface m_joint_state_interface;
        hardware_interface::VelocityJointInterface m_velocity_joint_interface;
        joint_limits_interface::VelocityJointSaturationInterface m_velocity_joint_saturation_interface;

    private:
        i2c_ros::I2C m_motors_control=i2c_ros::I2C(0, 0x08);
        
        std::string m_base_frame_id;
        std::string m_odom_frame_id;
        std::string m_sonar_frame_id;

        double m_sonar_measurement;

        int m_left_last_pos;
        int m_right_last_pos;
        int m_left_enc_pos;
        int m_right_enc_pos;
        int m_last_left_enc_pos;
        int m_last_right_enc_pos;
        int m_left_enc_mult;
        int m_right_enc_mult;
        double m_linear_v;
        double m_angular_v;
        double m_x;
        double m_y;
        double m_th;

        ros::Time m_t_next;

    private:
        double m_joint_vel[2];
        double m_joint_eff[2];
        double m_joint_pos[2];
        double m_joint_cmd[2];

    private:
        std::string m_joint_names[2] = {"left_wheel_joint", "right_wheel_joint"};
        static constexpr double kBaseWidth = 0.193;
        static const int kRate = 30;
        //static constexpr double kTicksPerMeter = 9847.712104092492;
        static constexpr double kTicksPerMeter = 2475.0;
        static const long kEncoderMin = -2147483648;
        static const long kEncoderMax = 2147483647;

        static long kEncoderLowWrap;
        static long kEncoderHighWrap;

    private:
        double m_left;
        double m_right;
        double m_dx;
        double m_dy;
        double m_dr;
        int m_tick_counter;
        int m_ticks_since_target;
};