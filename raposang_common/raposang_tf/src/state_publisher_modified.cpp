// ------------------------------------------------------------
//                         INCLUDE
// ------------------------------------------------------------
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Quaternion.h"
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

double arm;
geometry_msgs::Quaternion attitude;
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

void get_imu(const sensor_msgs::Imu &msg)
{
    attitude = msg.orientation;
}

void get_arm(const std_msgs::Int32 &msg) 
{
    float x = (float)msg.data;

    //if(x >= MINARM || x <= MAXARM)
    arm = remap_range(x, MINARM, MAXARM, MINANGLE, MAXANGLE);
}

void get_herk_motors(const idmind_herkulex::Motors &msg) 
{
    hrk_motors[0] = msg.motors[0]; //pan motor
    hrk_motors[1] = msg.motors[1]; //tilt motor
    hrk_motors[2] = msg.motors[2]; //roll motor
    hrk_motors[3] = msg.motors[3]; //pitch motor
}

// ------------------------------------------------------------
//                           MAIN
// ------------------------------------------------------------
int main(int argc, char** argv) {
    ros::init(argc, argv, "state_publisher");

    ROS_INFO("RAPOSA-NG TF has started.");

    ros::NodeHandle n;

    tf2_ros::TransformBroadcaster broadcaster;

    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    
    sub_arm = n.subscribe("/raposang/arm", 100, get_arm);
    sub_herk_motors = n.subscribe("/raposang/herkulex/motor_status", 100, get_herk_motors);
    sub_imu = n.subscribe("/imu/data", 100, get_imu);

    ros::Rate loop_rate(30);
    
    // message declarations
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped laser_trans, transform_pub;
    sensor_msgs::JointState joint_state;
    transform_pub.header.frame_id = "imu";
    transform_pub.child_frame_id = "hokuyo";

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

        //listener = tf.transformListener()
        //listener.waitforTransform("imu","hokuyo", now, rospy.duration(5.0))
        //(trans, rot) = listener.lookupTransform("base_link","hokuyo", now)
        //use the rotation for the values of the laser position

        //update transform
        //transform IMU rotation to hokuyo rotation
        try{
            laser_trans = tfBuffer.lookupTransform("imu", "hokuyo", ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        /*transform_pub.header.stamp = ros::Time::now();
        transform_pub.transform.rotation.x = attitude.x-laser_trans.transform.rotation.x;
        transform_pub.transform.rotation.y = attitude.y-laser_trans.transform.rotation.y;
        transform_pub.transform.rotation.z = attitude.z-laser_trans.transform.rotation.z;
        transform_pub.transform.rotation.w = attitude.w-laser_trans.transform.rotation.w; */

        //send the joint state and transform
        joint_pub.publish(joint_state);
        //broadcaster.sendTransform(transform_pub);

        ros::spinOnce();

        // This will adjust as needed per iteration
        loop_rate.sleep();
    }


    return 0;
}