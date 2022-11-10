// ------------------------------------------------------------
//                         INCLUDE
// ------------------------------------------------------------
#include <string>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
//#include <tf2/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include "sensor_msgs/Imu.h"
#include "std_msgs/Int32.h"
#include "idmind_herkulex/Motors.h"
#include "idmind_herkulex/MotorStatus.h"

// ------------------------------------------------------------
//                          DEFINES
// ------------------------------------------------------------
#define MINARM 750.0
#define MAXARM 873.0
#define MINANGLE (-33.0*M_PI)/180
#define MAXANGLE (20.0*M_PI)/180

// ------------------------------------------------------------
//                          GLOBALS
// ------------------------------------------------------------
ros::Subscriber sub_arm, sub_herk_motors, sub_imu;
//ros::Publisher pub_imu;

double arm;
idmind_herkulex::MotorStatus hrk_motors[4];

// ------------------------------------------------------------
//                         FUNCTIONS
// ------------------------------------------------------------
//Function to remap from one range to another range
//parameters: x - input value; omin - original range minimum; omax - orignial range max; nmin - new range minimum; nmax - new range max
double remap_range(const double &x, const double &omin, const double &omax, const double &nmin, const double &nmax)
{
    bool reverseInput = false;
    bool reverseOutput = false;
    double oldMin, oldMax, newMin, newMax, portion, newValue;


    //range check
    if (omin == omax)
    {
        ROS_INFO("[RAPOSANG_StatePublisher] Warning: zero output original range.");
        return NULL;
    }

    if (nmin == nmax)
    {
        ROS_INFO("[RAPOSANG_StatePublisher] Warning: zero output new range.");
        return NULL;
    }

    //check reversed input range
    oldMin = std::min( omin, omax );
    oldMax = std::max( omin, omax );
    if (oldMin != omin)
        reverseInput = true;

    //check reversed output range
    newMin = std::min( nmin, nmax );
    newMax = std::max( nmin, nmax );
    if (newMin != nmin)
        reverseOutput = true;

    portion = ((x-oldMin)*(newMax-newMin))/(oldMax-oldMin);
    if (reverseInput)
        portion = ((oldMax-x)*(newMax-newMin))/(oldMax-oldMin);

    newValue = portion + newMin;
    if (reverseOutput)
        newValue = newMax - portion;
    return newValue;
}

//function to convert from degree to radians
//parameter: deg - input degree value
double degtorad(int deg)
{
    double rad;
    rad = (deg*M_PI)/180;

    return rad;
}

void get_arm(const std_msgs::Int32 &msg)
{
    float x = (float)msg.data;
    arm = remap_range(x, MINARM, MAXARM, MINANGLE, MAXANGLE);
}

void get_herk_motors(const idmind_herkulex::Motors &msg)
{
    hrk_motors[0] = msg.motors[0]; //pan motor
    hrk_motors[1] = msg.motors[1]; //tilt motor
    hrk_motors[2] = msg.motors[2]; //roll motor
    hrk_motors[3] = msg.motors[3]; //pitch motor
}

/*void get_imu(const sensor_msgs::Imu &msg)
{
    tf2::Quaternion q_tf, q_imu, q_new;
    tf2Scalar yaw, pitch, roll;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    tf2_ros::TransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped transformStamped;
    sensor_msgs::Imu new_imu = msg;

    try
    {
        transformStamped = tfBuffer.lookupTransform("odom", "base_link", ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
        return;
    }

    // Get the original orientation of 'imu' msg convert to tf2
    tf2::convert(msg.orientation, q_imu);

    //Take out the rotation in Z
    tf2::Matrix3x3 q_mat(q_imu);
    q_mat.getEulerYPR(yaw, pitch, roll);
    q_imu.setEulerZYX(0.0,pitch,roll);
    ROS_DEBUG("Pitch %s, Roll %s", (pitch,roll));

    // Convert form msg->Quaternion to tf2->quaternion
    tf2::fromMsg(transformStamped.transform.rotation, q_tf);

    q_new = q_imu*q_tf;  // Calculate the new orientation
    q_new.normalize();

    // Stuff the new rotation back into the imu_msg. This requires conversion into a msg type
    //tf2::convert(q_new, new_imu.orientation);
    tf2::convert(q_new, transformStamped.transform.rotation);
    transformStamped.header.stamp = ros::Time::now();

    // update transform
    //transform.setOrigin(tf::Vector3(t(0),t(1),t(2))); //base_link origin
    //transform.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w())); //imu rotation
    broadcaster.sendTransform(transformStamped);

    //pub_imu.publish(new_imu);
}*/

// ------------------------------------------------------------
//                           MAIN
// ------------------------------------------------------------
int main(int argc, char** argv) {
    ros::init(argc, argv, "state_publisher");

    ROS_INFO("RAPOSA-NG TF has started.");

    ros::NodeHandle n;

    //tf2_ros::Buffer tfBuffer;
    //tf2_ros::TransformListener tfListener(tfBuffer);
    //tf2_ros::TransformBroadcaster broadcaster;

    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    //ros::Publisher pub_imu = n.advertise<sensor_msgs::Imu>("output_imu", 10);

    sub_arm = n.subscribe("arm_angle", 100, get_arm);
    sub_herk_motors = n.subscribe("herkulex_motor_status", 100, get_herk_motors);
    //sub_imu = n.subscribe("input_imu", 100, get_imu);

    ros::Rate loop_rate(30);

    // message declarations
    geometry_msgs::TransformStamped odom_trans;
    sensor_msgs::JointState joint_state;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    joint_state.name.resize(5);
    joint_state.position.resize(5);
    joint_state.name[0] = "joint_arm";
    joint_state.name[1] = "joint_pan";
    joint_state.name[2] = "joint_tilt";
    joint_state.name[3] = "joint_roll";
    joint_state.name[4] = "joint_pitch";


    while (ros::ok()) {
        //update joint_state
        joint_state.header.stamp = ros::Time::now();
        joint_state.position[0] = arm;
        joint_state.position[1] = degtorad(hrk_motors[0].angle); //pan motor
        joint_state.position[2] = degtorad(hrk_motors[1].angle); //tilt motor
        joint_state.position[3] = degtorad(hrk_motors[2].angle); //roll motor
        joint_state.position[4] = degtorad(hrk_motors[3].angle); //pitch motor


        // update transform
        // (moving in a circle with radius=2)
        //odom_trans.header.stamp = ros::Time::now();
        //odom_trans.transform.translation.x = cos(angle)*2;
        //odom_trans.transform.translation.y = sin(angle)*2;
        //odom_trans.transform.translation.z = .7;
        //odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle+M_PI/2);

        //send the joint state and transform
        joint_pub.publish(joint_state);
        //broadcaster.sendTransform(odom_trans);

        ros::spinOnce();

        // This will adjust as needed per iteration
        loop_rate.sleep();
    }


    return 0;
}
