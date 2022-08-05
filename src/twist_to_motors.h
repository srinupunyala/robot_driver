#pragma once

#include "i2c_ros.h"

#include "ros/ros.h"
#include "ros/time.h"
#include <geometry_msgs/Twist.h>


#include "string"

class TwistToMotors {
    public:
        TwistToMotors(ros::NodeHandle& nh);

        void spin(const ros::TimerEvent&);
        
        void write();

        void twistCallback(const geometry_msgs::Twist& msg);

    private:
        i2c_ros::I2C m_motors_control=i2c_ros::I2C(0, 0x08);

    private:
        double m_left;
        double m_right;
        double m_dx;
        double m_dy;
        double m_dr;
        int m_tick_counter;
        int m_ticks_since_target;
    
    private:
        static constexpr double kBaseWidth = 0.19;
        static const int kRate = 30;
};