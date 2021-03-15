#include <ros/ros.h>
#include "smart_car_base.h"

int main(int argc, char** argv )
{
    ros::init(argc, argv, "smart_car_base_node");
    SmartCarBase SCObject;
    ros::spin();
    return 0;
}
