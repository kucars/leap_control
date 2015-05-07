#include <iostream>
#include <string.h>
#include "ros/ros.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "leap_control/leapros.h"
#include "Leap.h"
#include <sstream>
#include <Eigen/Eigen>
#include <pcl/io/pcd_io.h>
#include <pcl/common/eigen.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/PointIndices.h>
#include <pcl/range_image/range_image.h>
#include "geometry_msgs/Twist.h"
#include <tf/transform_broadcaster.h>

using namespace Leap;
using namespace std;

inline void leap2BodyCoordinate(double x,double y,double z, geometry_msgs::Point &bodyPose)
{
    bodyPose.x = -z/1000.0f;
    bodyPose.y = -x/1000.0f;
    bodyPose.z =  y/1000.0f;
}



class HandsListener : public Listener
{
public:
    ros::NodeHandle node;
    ros::Publisher handJointMarkersPub;
    ros::Publisher handLinkMarkersPub;
    ros::Publisher handTrackerMarkersPub;
    ros::Publisher handNormalsMarkersPub;
    ros::Publisher leapInfoPub;
    ros::Publisher velocityPublisher;
    ros::Subscriber poseUpdate;
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
    void generateVelCmdUsingHandPose(Leap::Vector vector);
    void drawVector(Leap::Vector vector);
    void poseUpdateCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    double prevAngularVel, prevLinearVel;
    std::string cmdVelTopic,poseTopic;
    geometry_msgs::PoseStamped currentPose;
private:
};

void HandsListener::onInit(const Controller&)
{
    ROS_INFO("Initialized");
    handJointMarkersPub   = node.advertise<visualization_msgs::MarkerArray>("hands_joints", 10);
    handLinkMarkersPub    = node.advertise<visualization_msgs::Marker>("hands_links", 10);
    handTrackerMarkersPub = node.advertise<visualization_msgs::Marker>("hands_tracker", 10);
    handNormalsMarkersPub = node.advertise<visualization_msgs::Marker>("hand_vectors", 10);
    leapInfoPub           = node.advertise<leap_control::leapros>("/data", 10);

    ros::NodeHandle privateNh("~");
    privateNh.param<std::string>("cmd_vel_topic", cmdVelTopic,   std::string("/cmd_vel"));
    privateNh.param<std::string>("pose_topic",    poseTopic, std::string("/pose"));

    velocityPublisher     = node.advertise<geometry_msgs::Twist>(cmdVelTopic, 100);
    poseUpdate            = node.subscribe(poseTopic, 1, &HandsListener::poseUpdateCallback, this);

    ROS_INFO("cmd_vel:%s pose_topic is:%s socket",cmdVelTopic.c_str(),poseTopic.c_str());


    prevAngularVel = prevLinearVel = 0;
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
        handNormal.pitch(); handNormal.roll();handNormal.yaw();

        Leap::Vector palmPose = hand.palmPosition();
        Leap::Vector palmVel  = hand.palmVelocity();

        leapInfo.palmpos.x = palmPose.x;
        leapInfo.palmpos.y = palmPose.y;
        leapInfo.palmpos.z = palmPose.z;

        leapInfo.palmvel.x = palmVel.x;
        leapInfo.palmvel.y = palmVel.y;
        leapInfo.palmvel.z = palmVel.z;

        Leap::Vector handDirection = hand.direction();

        generateVelCmdUsingHandPose(handDirection);
        //drawVector(handDirection);

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

        const FingerList fingers = hand.fingers();
        for (FingerList::const_iterator fl = fingers.begin(); fl != fingers.end(); ++fl)
        {
            const Finger finger = *fl;
            if(finger.type() == Leap::Finger::TYPE_INDEX && hand.isRight() && finger.isValid())
            {
                ROS_INFO("Found Index");
                Bone bone = finger.bone(Leap::Bone::TYPE_METACARPAL);
                leap2BodyCoordinate(bone.nextJoint().x,bone.nextJoint().y,bone.nextJoint().z,trackedJoints[0]);
                numTrackedJoints++;
            }
            if(finger.type() == Leap::Finger::TYPE_PINKY && hand.isRight() && finger.isValid())
            {
                ROS_INFO("Found Pinky");
                Bone bone = finger.bone(Leap::Bone::TYPE_METACARPAL);
                leap2BodyCoordinate(bone.nextJoint().x,bone.nextJoint().y,bone.nextJoint().z,trackedJoints[1]);
                numTrackedJoints++;
            }
            if(finger.type() == Leap::Finger::TYPE_MIDDLE && hand.isRight() && finger.isValid())
            {
                ROS_INFO("Found Middle");
                Bone bone = finger.bone(Leap::Bone::TYPE_METACARPAL);
                leap2BodyCoordinate(bone.prevJoint().x,bone.prevJoint().y,bone.prevJoint().z,trackedJoints[2]);
                numTrackedJoints++;
                leap2BodyCoordinate(bone.nextJoint().x,bone.nextJoint().y,bone.nextJoint().z,trackedJoints[3]);
                numTrackedJoints++;
            }
            for (int b = 0; b < 4; ++b)
            {
                Bone::Type boneType = static_cast<Bone::Type>(b);
                Bone bone = finger.bone(boneType);
                geometry_msgs::Point point;
                leap2BodyCoordinate(bone.prevJoint().x,bone.prevJoint().y,bone.prevJoint().z,point);
                links.push_back(point);
                leap2BodyCoordinate(bone.nextJoint().x,bone.nextJoint().y,bone.nextJoint().z,point);
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

void HandsListener::drawVector(Leap::Vector vector)
{
    visualization_msgs::Marker vectorMsg;
    vectorMsg.header.frame_id = "/leapmotion_optical_frame";
    vectorMsg.header.stamp    = ros::Time::now();
    vectorMsg.ns      = "vectors";
    vectorMsg.id      = 0;
    vectorMsg.type    = visualization_msgs::Marker::ARROW;
    vectorMsg.action  = visualization_msgs::Marker::ADD;
    vectorMsg.scale.x = 0.15;
    vectorMsg.scale.y = 0.01;
    vectorMsg.scale.z = 0.01;
    vectorMsg.color.r = 1.0f;
    vectorMsg.color.g = 0.0f;
    vectorMsg.color.b = 1.0f;
    vectorMsg.color.a = 0.7f;

    vectorMsg.lifetime = ros::Duration(0.1);

    vectorMsg.pose.position.x =  vector.x/1000.0;
    vectorMsg.pose.position.y =  vector.y/1000.0;
    vectorMsg.pose.position.z =  vector.z/1000.0;

    tf::Quaternion q;
    q.setRPY(vector.roll(), vector.pitch(), vector.yaw());
    ROS_INFO(" Hand roll:%f, pitch:%f , yaw:%f",pcl::rad2deg(vector.roll()),pcl::rad2deg(vector.pitch()),pcl::rad2deg(vector.yaw()));
    vectorMsg.pose.orientation.x = q.x();
    vectorMsg.pose.orientation.y = q.y();
    vectorMsg.pose.orientation.z = q.z();
    vectorMsg.pose.orientation.w = q.w();
    handNormalsMarkersPub.publish(vectorMsg);
}

void HandsListener::generateVelCmdUsingHandPose(Leap::Vector vector)
{
    drawVector(vector);
    tf::Quaternion qt = tf::createQuaternionFromYaw(pcl::deg2rad(-90.0));
    ROS_INFO("QT x:%f y:%f z:%f w:%f",qt.x(),qt.y(),qt.z(),qt.w());

    double yaw,pitch,roll;
    tf::Quaternion q(currentPose.pose.orientation.x,currentPose.pose.orientation.y,currentPose.pose.orientation.z,currentPose.pose.orientation.w);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    ROS_INFO("Pose Yaw:%f",pcl::rad2deg(yaw));

    geometry_msgs::TwistStamped velocityOutStamped;
    geometry_msgs::Twist velocityOut;

    double maxLinVel = 0.1;
    double maxAngVel = 0.1;

    double vx             = -vector.pitch()*maxLinVel/(M_PI/2.0);
    double vy             = 0;
    velocityOut.linear.x  = (vx *cos(yaw)) - (vy * sin(yaw));
    velocityOut.linear.y  = (vx *sin(yaw)) + (vy * cos(yaw));
    velocityOut.linear.z  = 0 ;
    velocityOut.angular.z = vector.yaw()*maxAngVel/(M_PI/2.0);

    velocityOutStamped.twist        = velocityOut ;
    velocityOutStamped.header.stamp = ros::Time::now() ;
    ROS_INFO("VELs %f,%f,%f",velocityOut.linear.x,velocityOut.linear.y,velocityOut.angular.z);
    if(abs(velocityOut.linear.x - prevLinearVel)<0.1 && abs(velocityOut.angular.z-prevAngularVel)<0.2)
    {
        velocityPublisher.publish(velocityOut);
        prevLinearVel  = velocityOut.linear.x;
        prevAngularVel = velocityOut.angular.z;
    }
    velocityPublisher.publish(velocityOut);
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

    Eigen::Vector4f v1(trackedJoints[0].x - trackedJoints[1].x,trackedJoints[0].y - trackedJoints[1].y,trackedJoints[0].z - trackedJoints[1].z,0);
    Eigen::Vector4f v2(trackedJoints[2].x - trackedJoints[3].x,trackedJoints[2].y - trackedJoints[3].y,trackedJoints[2].z - trackedJoints[3].z,0);
    Eigen::Vector4f vXAxis(5,0,0,0);
    Eigen::Vector4f vYAxis(0,5,0,0);
    Eigen::Vector3f v1Dir = v1.head(3), vDir2 = v2.head(3), x = vXAxis.head(3), y = vYAxis.head(3);

    double angleX = pcl::getAngle3D(v1, vXAxis);
    double angleY = pcl::getAngle3D(v2, vYAxis);

    /* A hacky way to find directionality*/
    int xDir = v1Dir.cross(x)(1)>=0?1:-1;
    int yDir = vDir2.cross(y)(0)>=0?-1:1;
    double xAng = pcl::rad2deg(angleX)*xDir;
    double yAng = pcl::rad2deg(angleY)*yDir;
    ROS_INFO("Angle X-Axis:%f Y-Axis:%f Dirx:%d Diry:%d",xAng,yAng,xDir,yDir);
    double maxLinVel = 0.3;
    double maxAngVel = 1.0;
    geometry_msgs::Twist velocityOut;
    velocityOut.linear.x  =  yAng*maxLinVel/180.0f;
    velocityOut.angular.z = -xAng*maxAngVel/180.0f;
    //velocityOut.linear.z = 0;

    ROS_INFO("VELs %f,%f,%f",velocityOut.linear.x,velocityOut.linear.y,velocityOut.linear.z);

    if(abs(velocityOut.linear.x - prevLinearVel)<0.1 && abs(velocityOut.angular.z-prevAngularVel)<0.2)
    {
        velocityPublisher.publish(velocityOut);
        prevLinearVel  = velocityOut.linear.x;
        prevAngularVel = velocityOut.angular.z;
    }
    handTrackerMarkersPub.publish(handsTrackerMarkerMsg);
}

void HandsListener::poseUpdateCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    currentPose = *msg;
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
