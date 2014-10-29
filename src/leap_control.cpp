#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Empty.h"
#include "leap_motion/leapros.h"
#include <cmath>
#include <string>
ros::Publisher velocityPublisher;
/*
    Header header
    geometry_msgs/Vector3 direction
    geometry_msgs/Vector3 normal
    geometry_msgs/Point palmpos
    geometry_msgs/Vector3 ypr
    ypr.x = yaw
    ypr.y = pitch
    ypr.z = roll
*/
void leapMotionCallback(const  leap_motion::leapros::ConstPtr& msg)
{
    geometry_msgs::Twist velocityOut;
    velocityOut.linear.x = ((std::abs (msg->ypr.y)) - 90) / 30;
    if (velocityOut.linear.x > 1)
        velocityOut.linear.x = 1;
    else if (velocityOut.linear.x < -1)
        velocityOut.linear.x = -1;
    velocityOut.linear.y = msg->ypr.z / 70;
    if (velocityOut.linear.y > 1)
        velocityOut.linear.y = 1;
    else if (velocityOut.linear.y < -1)
        velocityOut.linear.y = -1;
    else if(msg->palmpos.y < 6000)
        velocityOut.linear.z = -.75;
    else if(msg->palmpos.z > 24000)
        velocityOut.linear.z = .75;
    velocityPublisher.publish(velocityOut);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "leap_control");
    ros::NodeHandle n;
    velocityPublisher   = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    ros::Subscriber sub = n.subscribe("leapros", 1000, leapMotionCallback);
    ros::Rate loop_rate(50);
    ros::spin();
    return 0;
}