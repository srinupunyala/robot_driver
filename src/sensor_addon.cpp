#include "tf_luna_i2c.h"

#include <ros/ros.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "sensor_addon");
    ros::NodeHandle nh;
    TFLuna tf_luna(nh);
    ros::Duration update_freq(1.0/4);
    ros::Timer non_realtime_loop = nh.createTimer(update_freq, &TFLuna::spinOnce, &tf_luna);
    ros::spin();
    return 0;
}