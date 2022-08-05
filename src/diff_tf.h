#pragma once

#include "i2c_ros.h"

#include "ros/ros.h"
#include "ros/time.h"
#include <tf/transform_broadcaster.h>

#include "string"


class OdomPublisher {
    public:
        OdomPublisher(ros::NodeHandle &nh);

        void spin(const ros::TimerEvent&);

        void update();

        void read();

    private:
        ros::Publisher m_odom_publisher;
        tf::TransformBroadcaster m_tf_broadcaster;
        ros::Time m_prev;
        
    private:
        i2c_ros::I2C m_motors_control=i2c_ros::I2C(0, 0x08);
        
        std::string m_base_frame_id;
        std::string m_odom_frame_id;

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
        static constexpr double kBaseWidth = 0.19;
        static const int kRate = 30;
        static constexpr double kTicksPerMeter = 9847.712104092492;
        static const long kEncoderMin = -2147483648;
        static const long kEncoderMax = 2147483647;

        static long kEncoderLowWrap;
        static long kEncoderHighWrap;
};