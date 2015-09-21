#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Empty.h"
#include <cmath>
#include <string>
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_broadcaster.h>
#include "leap_control/leapros.h"
#include <Eigen/Eigen>

ros::Publisher velocityPublisher;
ros::Publisher pub_marker_array;
ros::Publisher marker_pub;
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
void leapMotionCallback(const  leap_control::leapros::ConstPtr& msg)
{
    geometry_msgs::TwistStamped velocityOut;
    // Changing from Leapmotion coordinates to robot body coordinates : https://developer.leapmotion.com/documentation/csharp/devguide/Leap_Overview.html
    double x =  -msg->palmpos.x;
    double y =   msg->palmpos.z;
    double z =   msg->palmpos.y;

    /*
    double x =  -msg->palmpos.z;
    double y =   msg->palmpos.x;
    double z =   msg->palmpos.y;
    */
    // Assume that the working area is 500 mm, this will map to a speed of 1m/sec
    velocityOut.header.stamp = ros::Time::now();
    velocityOut.twist.linear.x = x/500.0f;
    velocityOut.twist.linear.y = y/500.0f;
    velocityOut.twist.linear.z = z/500.0f;
    /*
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
    */
    ROS_INFO("TEST %f,%f,%f",velocityOut.twist.linear.x,velocityOut.twist.linear.y,velocityOut.twist.linear.z);
    velocityPublisher.publish(velocityOut);
    
    visualization_msgs::Marker joint_msg;
    visualization_msgs::MarkerArray marker_array_msg;
    joint_msg.header.frame_id="/leap_optical_frame";
    joint_msg.header.stamp=ros::Time::now();
    joint_msg.ns="joint";
    joint_msg.id = 0;
    joint_msg.type = visualization_msgs::Marker::ARROW;
    joint_msg.action = visualization_msgs::Marker::ADD;
    joint_msg.scale.x = 0.15; 
    joint_msg.scale.y = 0.01;
    joint_msg.scale.z = 0.01;
    joint_msg.color.r = 1.0f;
    joint_msg.color.g = 0.0f;
    joint_msg.color.b = 1.0f;
    joint_msg.color.a = 0.7f;

    joint_msg.lifetime = ros::Duration(0.1);
    
    joint_msg.pose.position.x =  x/1000.0;
    joint_msg.pose.position.y =  y/1000.0;
    joint_msg.pose.position.z =  z/1000.0;
    
    //Eigen::Matrix<double, 3, 3> rotation_matrix;
    //rotation_matrix << msg->ypr.x, msg->ypr.y, msg->ypr.z;
    //Eigen::Quaternion<double> q(rotation_matrix);
    
    
    tf::Quaternion q;
    q.setRPY(msg->ypr.z*M_PI/180.0, msg->ypr.y*M_PI/180.0, msg->ypr.x*M_PI/180.0);
    
    joint_msg.pose.orientation.x = q.x();
    joint_msg.pose.orientation.y = q.y();
    joint_msg.pose.orientation.z = q.z();
    joint_msg.pose.orientation.w = q.w();
    //_pub_marker_array.publish(marker_array_msg);
    marker_pub.publish(joint_msg);
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "leap_control");
    ros::NodeHandle n;
    velocityPublisher = n.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 1000);
    marker_pub        = n.advertise<visualization_msgs::Marker>("hands_links", 1);
    ros::Subscriber sub = n.subscribe("leapmotion/data", 1000, leapMotionCallback);
    ros::Rate loop_rate(50);
    ros::spin();
    return 0;
}
