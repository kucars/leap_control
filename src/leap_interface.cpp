#include <iostream>
#include <string.h>
#include "ros/ros.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "leap_control/leapros.h"
#include "Leap.h"
#include <sstream>

using namespace Leap;
using namespace std;

class HandsListener : public Listener
{
public:
    ros::NodeHandle node;
    ros::Publisher handJointMarkersPub;
    ros::Publisher handLinkMarkersPub;
    ros::Publisher handTrackerMarkersPub;
    ros::Publisher leapInfoPub;
    unsigned int seq;
    virtual void onInit(const Controller&);
    virtual void onConnect(const Controller&);
    virtual void onDisconnect(const Controller&);
    virtual void onExit(const Controller&);
    virtual void onFrame(const Controller&);
    virtual void onFocusGained(const Controller&);
    virtual void onFocusLost(const Controller&);
    virtual void onDeviceChange(const Controller&);
    virtual void onServiceConnect(const Controller&);
    virtual void onServiceDisconnect(const Controller&);
    void createMarkers(std::vector<geometry_msgs::Point>,std::vector<geometry_msgs::Point> );
    void generateVelCmd(geometry_msgs::Point trackedJoints[4]);
private:
};

void HandsListener::onInit(const Controller&)
{
    ROS_INFO("Initialized");
    handJointMarkersPub   = node.advertise<visualization_msgs::MarkerArray>("hands_joints", 10);
    handLinkMarkersPub    = node.advertise<visualization_msgs::Marker>("hands_links", 10);
    handTrackerMarkersPub = node.advertise<visualization_msgs::Marker>("hands_tracker", 10);
    leapInfoPub         = node.advertise<leap_control::leapros>("leapmotion/data", 10);
}


void HandsListener::onConnect(const Controller& controller)
{
    ROS_INFO("Connected");
    controller.enableGesture(Gesture::TYPE_CIRCLE);
    controller.enableGesture(Gesture::TYPE_KEY_TAP);
    controller.enableGesture(Gesture::TYPE_SCREEN_TAP);
    controller.enableGesture(Gesture::TYPE_SWIPE);
}

void HandsListener::onDisconnect(const Controller&)
{
    ROS_INFO("Disconnected");
}

void HandsListener::onExit(const Controller&)
{
    ROS_INFO("Exited");
}

void HandsListener::onFrame(const Controller& controller)
{
    leap_control::leapros leapInfo;
    // Get the most recent frame and report some basic information
    const Frame frame = controller.frame();
    HandList hands = frame.hands();
    std::vector<geometry_msgs::Point> links;
    std::vector<geometry_msgs::Point> joints;
    geometry_msgs::Point trackedJoints[4];
    unsigned int numTrackedJoints = 0;
    for (HandList::const_iterator hl = hands.begin(); hl != hands.end(); ++hl)
    {
        const Hand hand = *hl;
        Leap::Vector handNormal = hand.palmNormal();
        leapInfo.normal.x = handNormal.x;
        leapInfo.normal.y = handNormal.y;
        leapInfo.normal.z = handNormal.z;

        Leap::Vector palmPose = hand.palmPosition();

        leapInfo.palmpos.x = palmPose.x;
        leapInfo.palmpos.y = palmPose.y;
        leapInfo.palmpos.z = palmPose.z;

        Leap::Vector handDirection = hand.direction();

        leapInfo.direction.x = handDirection.x;
        leapInfo.direction.y = handDirection.y;
        leapInfo.direction.z = handDirection.z;

        leapInfo.ypr.x = hand.direction().yaw();
        leapInfo.ypr.y = hand.direction().pitch();
        leapInfo.ypr.z = hand.direction().roll();
        if(hand.isRight())
            leapInfo.hand_id = std::string("right");
        else
            leapInfo.hand_id = std::string("left");
        leapInfoPub.publish(leapInfo);

        ROS_INFO("Hand yaw:%f pitch:%f roll:%f",leapInfo.ypr.x,leapInfo.ypr.y,leapInfo.ypr.z);

        const FingerList fingers = hand.fingers();
        for (FingerList::const_iterator fl = fingers.begin(); fl != fingers.end(); ++fl)
        {
            const Finger finger = *fl;
            if(finger.type() == Leap::Finger::TYPE_INDEX && hand.isRight() && finger.isValid())
            {
                ROS_INFO("Found Index");
                Bone bone = finger.bone(Leap::Bone::TYPE_METACARPAL);
                geometry_msgs::Point point;
                point.x = -bone.nextJoint().x/1000;
                point.y =  bone.nextJoint().z/1000;
                point.z =  bone.nextJoint().y/1000;
                trackedJoints[0] = point;
                numTrackedJoints++;
            }
            if(finger.type() == Leap::Finger::TYPE_PINKY && hand.isRight() && finger.isValid())
            {
                ROS_INFO("Found Pinky");
                Bone bone = finger.bone(Leap::Bone::TYPE_METACARPAL);
                geometry_msgs::Point point;
                point.x = -bone.nextJoint().x/1000;
                point.y =  bone.nextJoint().z/1000;
                point.z =  bone.nextJoint().y/1000;
                trackedJoints[1] = point;
                numTrackedJoints++;
            }
            if(finger.type() == Leap::Finger::TYPE_MIDDLE && hand.isRight() && finger.isValid())
            {
                ROS_INFO("Found Middle");
                Bone bone = finger.bone(Leap::Bone::TYPE_METACARPAL);
                geometry_msgs::Point point;
                point.x = -bone.prevJoint().x/1000;
                point.y =  bone.prevJoint().z/1000;
                point.z =  bone.prevJoint().y/1000;
                trackedJoints[2] = point;
                numTrackedJoints++;

                point.x = -bone.nextJoint().x/1000;
                point.y =  bone.nextJoint().z/1000;
                point.z =  bone.nextJoint().y/1000;
                trackedJoints[3] = point;
                numTrackedJoints++;
            }
            for (int b = 0; b < 4; ++b)
            {
                Bone::Type boneType = static_cast<Bone::Type>(b);
                Bone bone = finger.bone(boneType);
                geometry_msgs::Point point;
                point.x = -bone.prevJoint().x/1000;
                point.y =  bone.prevJoint().z/1000;
                point.z =  bone.prevJoint().y/1000;
                links.push_back(point);
                point.x = -bone.nextJoint().x/1000;
                point.y =  bone.nextJoint().z/1000;
                point.z =  bone.nextJoint().y/1000;
                joints.push_back(point);
                links.push_back(point);
            }
        }
    }
    if(numTrackedJoints == 4)
        generateVelCmd(trackedJoints);
    createMarkers(links,joints);
}

void HandsListener::onFocusGained(const Controller&)
{
    ROS_INFO("Focus Gained");
}

void HandsListener::onFocusLost(const Controller&)
{
    ROS_INFO("Focus Lost");
}

void HandsListener::onDeviceChange(const Controller& controller)
{
    ROS_INFO("Device Changed");
    const DeviceList devices = controller.devices();
    for (int i = 0; i < devices.count(); ++i)
    {
        std::cout << "id: " << devices[i].toString() << std::endl;
        std::cout << "  isStreaming: " << (devices[i].isStreaming() ? "true" : "false") << std::endl;
    }
}

void HandsListener::onServiceConnect(const Controller&)
{
    ROS_INFO("Service Connected");
}

void HandsListener::onServiceDisconnect(const Controller&)
{
    ROS_INFO("Service Disconnected");
}

void HandsListener::createMarkers(std::vector<geometry_msgs::Point> links,std::vector<geometry_msgs::Point> joints)
{
    visualization_msgs::Marker linksMarkerMsg, jointsMarkerMsg;
    visualization_msgs::MarkerArray jointsMarkerArrayMsg;
    linksMarkerMsg.header.frame_id=jointsMarkerMsg.header.frame_id="/leapmotion_optical_frame";
    linksMarkerMsg.header.stamp=jointsMarkerMsg.header.stamp=ros::Time::now();
    linksMarkerMsg.ns="link_marker";
    jointsMarkerMsg.ns="joint_marker";
    linksMarkerMsg.id = 0;
    jointsMarkerMsg.id = 0;
    linksMarkerMsg.type = visualization_msgs::Marker::LINE_LIST;
    jointsMarkerMsg.type = visualization_msgs::Marker::SPHERE;
    linksMarkerMsg.scale.x = 0.005;
    jointsMarkerMsg.scale.x = jointsMarkerMsg.scale.y = jointsMarkerMsg.scale.z = 0.015;
    jointsMarkerMsg.color.r = .0f;
    jointsMarkerMsg.color.g = 1.0f;
    jointsMarkerMsg.color.b = 1.0f;
    jointsMarkerMsg.color.a = 0.7f;
    linksMarkerMsg.action = jointsMarkerMsg.action = visualization_msgs::Marker::ADD;
    linksMarkerMsg.lifetime = jointsMarkerMsg.lifetime = ros::Duration(0.1);
    std_msgs::ColorRGBA color;
    color.r = 1.0f; color.g=.0f; color.b=.0f, color.a=1.0f;
    vector<geometry_msgs::Point>::iterator linksIterator;
    for(linksIterator = links.begin();linksIterator != links.end();linksIterator++)
    {
        linksMarkerMsg.points.push_back(*linksIterator);
        linksMarkerMsg.colors.push_back(color);
    }
    handLinkMarkersPub.publish(linksMarkerMsg);

    vector<geometry_msgs::Point>::iterator jointsIterator;
    for(jointsIterator = links.begin();jointsIterator != links.end();jointsIterator++)
    {
        jointsMarkerMsg.pose.position.x = (*jointsIterator).x;
        jointsMarkerMsg.pose.position.y = (*jointsIterator).y;
        jointsMarkerMsg.pose.position.z = (*jointsIterator).z;
        jointsMarkerMsg.id+=1;
        jointsMarkerArrayMsg.markers.push_back(jointsMarkerMsg);
    }
    handJointMarkersPub.publish(jointsMarkerArrayMsg);
}

void HandsListener::generateVelCmd(geometry_msgs::Point trackedJoints[4])
{
    visualization_msgs::Marker handsTrackerMarkerMsg;
    handsTrackerMarkerMsg.header.frame_id = "/leapmotion_optical_frame";
    handsTrackerMarkerMsg.header.stamp=ros::Time::now();
    handsTrackerMarkerMsg.ns="tracker_marker";
    handsTrackerMarkerMsg.id = 0;
    handsTrackerMarkerMsg.type = visualization_msgs::Marker::LINE_LIST;
    handsTrackerMarkerMsg.scale.x = 0.005;
    handsTrackerMarkerMsg.action = visualization_msgs::Marker::ADD;
    handsTrackerMarkerMsg.lifetime = ros::Duration(0.1);
    std_msgs::ColorRGBA color1,color2;
    color1.r = 0.0f; color1.g=1.0f; color1.b=.0f, color1.a=1.0f;
    color2.r = 1.0f; color2.g=1.0f; color2.b=0.0f, color2.a=1.0f;
    for(uint i=0; i<4;i++)
    {
        handsTrackerMarkerMsg.points.push_back(trackedJoints[i]);
        if(i==0 || i==1)
            handsTrackerMarkerMsg.colors.push_back(color1);
        else
            handsTrackerMarkerMsg.colors.push_back(color2);
    }
    handTrackerMarkersPub.publish(handsTrackerMarkerMsg);
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "leap_interface");
    // LeapMotion Controller and Listener
    HandsListener listener;
    Controller controller;
    controller.addListener(listener);
    controller.setPolicyFlags(static_cast<Leap::Controller::PolicyFlag> (Leap::Controller::POLICY_IMAGES));
    ros::spin();
    //Done: stop listening
    controller.removeListener(listener);
    return 0;
}
