#include <stdlib.h> 
#include <stdio.h> 
#include <unistd.h> 
#include <assert.h>
#include <errno.h> 
#include <time.h> 
#include <float.h> 
#include <math.h>

// C++
#include <iostream> 
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
// ROS
#include <ros/ros.h> 
#include <tf/transform_broadcaster.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <ros/console.h>

// IMU
#include "sensor_msgs/Imu.h"

// Odometry
#include <idmind_motorsboard/WheelsMB.h>

// Pose
#include <nav_msgs/Odometry.h>

// ------------------------------------------------------------ // DEFINES // ------------------------------------------------------------

#define RADTODEG 57.2957795131
#define DEGTORAD 0.017453293 
#define WHEEL_DIST 0.42

// ------------------------------------------------------------ // STRUCT // ------------------------------------------------------------

class Wheel_Odometry{
	
	double x, y, yaw, L, v_x, w_z;

	public:
		
    Wheel_Odometry(): x(0.0), y(0.0), yaw(0.0), L(WHEEL_DIST), v_x(0.0), w_z(0.0) {}
	
    Wheel_Odometry(double _L): x(0.0), y(0.0), yaw(0.0), L(_L), v_x(0.0), w_z(0.0) {}
		
    ~Wheel_Odometry() {}
	
	void setTranslation(double left, double right) {
		
		const double ang = (right - left)/this->L;
		const double T = (left + right)/2.0;
		
		this->yaw += ang;
		this->x   += T * cos(this->yaw);
		this->y   += T * sin(this->yaw);
		
        //ROS_INFO("%lf", ang);
			
    }
	
	Eigen::Vector3d getPosition() {
		return Eigen::Vector3d(this->x, this->y, 0.0);
    }
	
	Eigen::Quaterniond getOrientation() {

		Eigen::Matrix3d R;	
		
		R << cos(this->yaw),  -sin(this->yaw), 0,
				 sin(this->yaw),  cos(this->yaw), 0,
				 0,           0,        1.0;
		
		Eigen::Quaterniond q(R);
		q.normalize();
						 		
		return q;
    }

	void changeDistBetweenTracks(double _L) {
		this->L = _L;
    }

	void resetOdometry() {
		this->yaw = 0.0;
		this->x   = 0.0;
		this->y   = 0.0;
                this->v_x = 0.0;
                this->w_z = 0.0;
    }
	
};

class Odometry{
	
	ros::NodeHandle n;

    ros::Subscriber sub_odo, sub_imu, sub_cmd;
    ros::Publisher  pub_data, pub_imu;

	tf::Transform transform;
	tf::TransformBroadcaster br;

	nav_msgs::Odometry odometry;
	idmind_motorsboard::WheelsMB odo_prev;
	
	Wheel_Odometry wheel;
	
	double left, right, v_x, w_z;
	double roll, pitch, yaw; 
    double q_current_yaw, q_previous_yaw;

	bool first_odo, publish_tf;
	
	std::string base_frame, odom_frame;
	
	Eigen::Vector3d t;
	Eigen::Quaterniond q;
        geometry_msgs::Twist v_odom;
        
	Eigen::Vector3d previous_t;

        ros::Time current_time_stamp;
        ros::Time previous_time_stamp;
	
	public:
	
		// Constructor
	
		Odometry(): wheel(), left(0.0), right(0.0), first_odo(true)
		{
		
			double dist_tracks;

			pub_data = n.advertise<nav_msgs::Odometry>("output_pose", 1);
            pub_imu = n.advertise<sensor_msgs::Imu>("output_imu", 1);
			sub_odo = n.subscribe("input_odo", 1, &Odometry::getOdo, this);	
            sub_imu = n.subscribe("input_imu", 1, &Odometry::getIMU, this);
            sub_cmd = n.subscribe("syscommand", 1, &Odometry::getCmd, this);

            ros::NodeHandle nh("~");
			nh.param("publish_tf", publish_tf, true);
			nh.param("dist_between_tracks", dist_tracks, 0.42);
			nh.param<std::string>("base_frame", base_frame, "base_link");
			nh.param<std::string>("odom_frame", odom_frame, "odom");

			wheel.changeDistBetweenTracks(dist_tracks);

        }
		
		// Destructor

        ~Odometry() {}

		// You spin me round baby
		
		void rosSpin() {

			ros::Rate rate(30);

			while(ros::ok()) {
				
				ros::spinOnce();
				
				if(!first_odo) {
				
					setPose();

					pub_data.publish(odometry);

                    if(publish_tf){
					
						transform.setOrigin(tf::Vector3(t(0),t(1),t(2)));
						transform.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));																		 
						br.sendTransform(tf::StampedTransform(transform, 
                                                              ros::Time::now(),
                                                              odom_frame.c_str(),
                                                              base_frame.c_str()));
					}
				}

                rate.sleep();

			}	


			
        }
		
	private:
	
        void getIMU(const sensor_msgs::Imu &imu) {

            sensor_msgs::Imu new_imu = imu;

            double y, p, r;

            tf::Matrix3x3 coord_change;

            coord_change.setValue(-1.0, 0.0, 0.0,
                                  0.0, 1.0, 0.0,
                                  0.0, 0.0, -1.0);

            tf::Quaternion q, q_new;

            tf::quaternionMsgToTF(imu.orientation, q);
            ((tf::Matrix3x3(q))*coord_change).getRotation(q_new);

            new_imu.orientation.w = q_new.getW();
            new_imu.orientation.x = q_new.getX();
            new_imu.orientation.y = q_new.getY();
            new_imu.orientation.z = q_new.getZ();

            new_imu.angular_velocity.x = imu.angular_velocity.x;
            new_imu.angular_velocity.y = imu.angular_velocity.y;
            new_imu.angular_velocity.z = imu.angular_velocity.z;

            new_imu.linear_acceleration.x = imu.linear_acceleration.x;
            new_imu.linear_acceleration.y = imu.linear_acceleration.y;
            new_imu.linear_acceleration.z = imu.linear_acceleration.z;

            pub_imu.publish(new_imu);

            if(publish_tf) {

                tf::Matrix3x3(q_new).getRPY(y, p, r);
                q_new.setRPY(r, p, 0.0);
            }

        }


		void getOdo(const idmind_motorsboard::WheelsMB &odo){
				
			if(first_odo) {
				first_odo = false;
				right = 0.0;
				left = 0.0;
                                v_x = 0.0;
                                w_z = 0.0;
                                q_current_yaw = 0.0;
                                //previous_time_stamp = ros::Time::now();
                                //current_time_stamp = ros::Time::now();
			}
			else {
				right = odo.front_right - odo_prev.front_right;
				left = odo.front_left - odo_prev.front_left;
			}

			odo_prev = odo;				
			previous_t = t;            // saving previous position (Vector3d - translation) 
                        q_previous_yaw = q_current_yaw;
                        previous_time_stamp = current_time_stamp;

			wheel.setTranslation(left, right);						
                        t = wheel.getPosition();
			q = wheel.getOrientation();
			
                        // get orientation (yaw) from the quaternion
                        /*
                        tf::Quaternion q( q.x(), q.y(), q.z(), q.w() );
                        tf::Matrix3x3 m(q);
                        m.getRPY(roll, pitch, yaw);
                        q_current_yaw = yaw;
                        */
                        //geometry_msgs::Pose2D pose2d;
                        //pose2d.x = msg->pose.pose.position.x;
                        //pose2d.y = msg->pose.pose.position.y;
    
                        tf::Quaternion q_new( q.x(), q.y(), q.z(), q.w() );
                        tf::Matrix3x3 m(q_new);
                        double roll, pitch, yaw;
                        m.getRPY(roll, pitch, yaw);
                        q_current_yaw = yaw;
           
                        //ROS_INFO("Current_yaw = %f", q_current_yaw);

                        current_time_stamp = ros::Time::now();
                        
                        // Calculate the velocities based on current/previous position and orientation
                        v_x = ( t(0) - previous_t(0) ) / ( current_time_stamp.toSec() - previous_time_stamp.toSec());
                        w_z = ( q_current_yaw - q_previous_yaw ) / ( current_time_stamp.toSec() - previous_time_stamp.toSec());
                        
                        //v_odom.linear.x = v_x;
                        //v_odom.angular.z = w_z;
                        
        }

        void getCmd(const std_msgs::String &cmd){

            if(cmd.data == "reset") {
                first_odo = true;
                wheel.resetOdometry();
                right = 0.0;
                left = 0.0;
                v_x = 0.0;
                w_z = 0.0;
                q_current_yaw = 0.0;
            }

        }

		void setPose() {

			odometry.header.stamp = odo_prev.header.stamp;
			odometry.header.frame_id = odom_frame.c_str();
			odometry.child_frame_id = base_frame.c_str();
					
			odometry.pose.pose.position.x = t(0);
			odometry.pose.pose.position.y = t(1);
			odometry.pose.pose.position.z = t(2);
			
			odometry.pose.pose.orientation.w = q.w();
			odometry.pose.pose.orientation.x = q.x();
			odometry.pose.pose.orientation.y = q.y();
			odometry.pose.pose.orientation.z = q.z();

			odometry.pose.covariance[0]  = 0.001;
			odometry.pose.covariance[7]  = 0.001;
			odometry.pose.covariance[14] = 100000;
			odometry.pose.covariance[21] = 100000;
			odometry.pose.covariance[28] = 100000;
			odometry.pose.covariance[35] = 0.001;
                        
            odometry.twist.twist.linear.x = v_x;
            odometry.twist.twist.angular.z = w_z;
			//ROS_DEBUG("Current_yaw = %s", w_z);
        }
		
	};


