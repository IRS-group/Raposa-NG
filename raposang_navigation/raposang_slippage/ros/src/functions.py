
import math
import tf
import rospy
import classes
import matplotlib.pyplot as plt
import numpy as np
import sys

from data_visualization import *

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Joy


laser_d = classes.measurements()
odometry_d = classes.measurements()

laser_d_tf_odom = classes.measurements()
laser_d_tf_base_link = classes.measurements()

roc_data = classes.roc_curve_data()

position_difference = classes.difference_data()
orientation_difference = classes.difference_data()

position_difference_vector_dif = classes.difference_data()
position_difference_vector_angles = classes.difference_data()

threshold_position = classes.distance_data()
threshold_orientation = classes.distance_data()


# FRAMES ROTATION:
transf_odom = classes.rotation_data()
transf_laser = classes.rotation_data()


current_time = 0
atualizar_odom = True
atualizar_laser = True

# VALUES USED DURING THE USER_STUDY:
#minimum_threshold_linear = 0.03
#minimum_threshold_angular = 0.266

minimum_threshold_linear = 0.07
minimum_threshold_angular = 0.266

#minimum_threshold_linear = 0.09
#minimum_threshold_angular = 0.14

#minimum_threshold_linear = 0.3
#minimum_threshold_angular = 0.13

#minimum_threshold_angular = 0.35

#minimum_threshold_linear = 0.5
#minimum_threshold_angular = 0.6

#minimum_threshold_linear = 0.05
#minimum_threshold_angular = 0.07

thresh_factor = 0.7
#thresh_factor = 0.45
frequency= 10.0
T_sample = 0.1
#T_sample = 1/frequency
size_moving_window = 7
#time_window = 1.0 #seconds
#size_moving_window = int(time_window*frequency)     # n

measurements_odom = 0
measurements_laser = 0

dif_vector = np.array([0, 0])
d_p_odom = np.array([0, 0])

#----------------------FILTERING-------------------------#

#   window is defined based on a certain time interval

#--------------------------------------------------------#

#time_window = 1.0 #seconds
#dim_window = int(time_window*frequency)     # n
#dim_window = 20 # dimension of Moving Average Window
init_p_values = True


#----------------------DETECTION-------------------------#

#threshold_const = False
#threshold_linear_const = 0.05  #wheels moving
#factor = 0
#threshold_linear_const = 0.1    #wheels stopped
#threshold_angular_const = 0.3    # 20 = NOT DETECTING!!!!

# TRUE VECTORS:

#true_slip_data = classes.true_slip(true_position=true_pos, true_orientation=true_rot)

#first = True
duration = 0
begining_time = 0



# ------------------------------------------------------------------------------------------#
#
# SIMPLE MOVING AVERAGE FILTERING - ANY MEASUREMENT
#
# ------------------------------------------------------------------------------------------#
def filter_SMA(new_measurement, SMA, p_M_values):
    p_new = new_measurement
    p_old = p_M_values.pop(0)
    p_M_values.append(p_new)
    SMA = SMA + p_new / size_moving_window - p_old / size_moving_window

    return SMA, p_M_values


#------------------------------------------------------------------------------------------#


def angles_minus_pi_to_pi(theta):

    test = theta >= -math.pi and theta < math.pi  # -pi < theta < pi

    while test == False:
        if theta >= math.pi:
            theta = theta - 2 * math.pi
        if theta < 0 - math.pi:
            theta = theta + 2 * math.pi
        test = theta >= -math.pi and theta < math.pi

    return theta



# -----------------------------------------------------------------------------------------#


def calculate_velocities():
    global current_time

    if len(laser_d.trajectory.x) > 1 and len(odometry_d.trajectory.x) > 1:

        velocity_x_laser = (laser_d.trajectory.x[-1] - laser_d.trajectory.x[-2]) * frequency
        velocity_y_laser = (laser_d.trajectory.y[-1] - laser_d.trajectory.y[-2]) * frequency

        velocity_x_odom = (odometry_d.trajectory.x[-1] - odometry_d.trajectory.x[-2]) * frequency
        velocity_y_odom = (odometry_d.trajectory.y[-1] - odometry_d.trajectory.y[-2]) * frequency

        velocity_laser = math.hypot(velocity_x_laser, velocity_y_laser)
        velocity_odom = math.hypot(velocity_x_odom, velocity_y_odom)

        theta0 = angles_minus_pi_to_pi(laser_d.trajectory.theta[-1])
        theta1 = angles_minus_pi_to_pi(laser_d.trajectory.theta[-2])

        theta2 = angles_minus_pi_to_pi(odometry_d.trajectory.theta[-1])
        theta3 = angles_minus_pi_to_pi(odometry_d.trajectory.theta[-2])

        dif_laser = angles_minus_pi_to_pi(theta0 - theta1)
        dif_odom = angles_minus_pi_to_pi(theta2 - theta3)

        velocity_theta_laser = (dif_laser) * frequency
        velocity_theta_odom = (dif_odom) * frequency

        vel_difference = abs(velocity_laser - velocity_odom)
        vel_angular_dif = abs(velocity_theta_laser - velocity_theta_odom)

        return velocity_laser, velocity_odom, velocity_theta_odom, velocity_theta_laser, vel_difference, vel_angular_dif

    else:
        return 0, 0, 0, 0, 0, 0


# -----------------------------------------------------------------------------------------#


def tests_slip(threshold_linear_current, threshold_angular_current):
    #global threshold_const, threshold_linear_const

    test1 = position_difference.dif[-1] > threshold_linear_current

    if len(orientation_difference.dif) > 0:
        test2 = orientation_difference.dif[-1] > threshold_angular_current
    else:
        test2 = False

    test3 = odometry_d.position.velocity.velocity[-1] > laser_d.position.velocity.velocity[-1]

    if len(odometry_d.rotation.velocity.velocity) > 0:
        test4 = odometry_d.rotation.velocity.velocity[-1] > laser_d.rotation.velocity.velocity[-1]
    else:
        test4 = False

    return test1, test2, test3, test4

# -----------------------------------------------------------------------------------------#


def testing_directions(odom_vector, dif_vector):
    alpha = angle_between(odom_vector, dif_vector)*180/math.pi

    # assuming that the amplitude of the noise is contained in 1 degree
    test1 = alpha >= 0
    test2 = alpha < 1

    testing = test1 and test2
    #print("alpha:", alpha)

    return testing

# -----------------------------------------------------------------------------------------#


def testing_small_laser(laser_vector):

    testing = vector_size(laser_vector) < 0.03


    return testing



# -----------------------------------------------------------------------------------------#


def test_slip_and_get_int(test1, test2, test3, test4, d_p_odom, dif_vector, d_p_laser):
    global m_pos, m_rot

    str_time = "%s" % position_difference.time[-1]

    # did it lose traction? / Is it slipping?

    if test1:           # position
        m_pos = True
    else:
        m_pos = False
    if test2:           # orientation
        m_rot = True
    else:
        m_rot = False

    """
    if test1 or test2:
        print("~~~~~~~")
    """

    # test3: |dif_pos_odom| > |dif_pos_laser|       Wheels moving more than the laser?
    # test4: |dif_rot_odom| > |dif_rot_laser|

    # Stuck:
    if test1 and test3:

        if testing_small_laser(d_p_laser):
            #print("stuck, position, new_test_layer")
            #return "T1rue;position;Stuck: wheels_moving_raposa_stopped;1"
            return 1;
        else:
            #print("testing stuck or slid:")
            new_test = testing_directions(d_p_odom, d_p_laser)
            if new_test:
                #print("stuck, position")
                #return "T1rue;position;Stuck: wheels_moving_raposa_stopped;1"
                return 1
            else:
                #print("sliding, position")
                #return "T2rue;position;Sliding: wheels_stopped_raposa_moving;2"
                return 2
                # Sliding:
            
    elif test1 and test3 == False:
        # print("sliding, position")
        #return "T2rue;position;Sliding: wheels_stopped_raposa_moving;2"
        return 2

    elif test2 and test4:
        # print("stuck, orientation")
        #return "T3rue;orientation;Stuck: wheels_moving_raposa_stopped;3"
        return 3

    elif test2 and test4 == False:
        #print("sliding, orientation")
        #return "T4rue;orientation;Sliding: wheels_stopped_raposa_moving;4"
        return 4

    # Normal:
    else:
        #return "False;0;0"
        return -1



#------------------------------------------------------------------------------------------#


def publish_slippage(pub, vel_difference, vel_angular_dif, current_time):
    #global threshold_linear_const

    threshold_linear_current = update_threshold(odometry_d.position.velocity.velocity,
                                                laser_d.position.velocity.velocity,
                                                threshold_position,
                                                minimum_threshold_linear,
                                                current_time)

    threshold_angular_current = update_threshold(odometry_d.rotation.velocity.velocity,
                                                 laser_d.rotation.velocity.velocity,
                                                 threshold_orientation,
                                                 minimum_threshold_angular,
                                                 current_time)

    if len(position_difference.dif) > 0:
        slipping = test_slippage(threshold_linear_current, threshold_angular_current)
        #str = "%s" % slipping
        pub.publish(slipping)

    #roc_curves_add_value(current_time)      # DETECTION EVALUATION


# ------------------------------------------------------------------------------------------#

def test_slippage(threshold_linear_current, threshold_angular_current):

    test1, test2, test3, test4 = tests_slip(threshold_linear_current, threshold_angular_current)
    slip_int = test_slip_and_get_int(test1, test2, test3, test4)

    return slip_int


# ------------------------------------------------------------------------------------------#

def update_threshold(list_vel_odom, list_vel_laser, threshold_class, minimum_threshold, current_time):
    if len(list_vel_odom) > 0 and len(list_vel_laser) > 0:
        thresh = max(abs(list_vel_odom[-1]), abs(list_vel_laser[-1])) * thresh_factor
        if thresh < minimum_threshold:
            thresh = minimum_threshold

        threshold_class.add_value(thresh, current_time)

        return thresh


# ------------------------------------------------------------------------------------------#

def test_slippage_position(threshold_linear_current, threshold_angular_current, d_p_odom, dif_vector, d_p_laser):
    test1, test2, test3, test4 = tests_slip_position(threshold_linear_current, threshold_angular_current)
    slip_int = test_slip_and_get_int(test1, test2, test3, test4, d_p_odom, dif_vector, d_p_laser)

    return slip_int


#------------------------------------------------------------------------------------------#


def publish_slippage_position(pub, vel_difference, vel_angular_dif, current_time, d_p_odom, dif_vector, d_p_laser):
    #global threshold_linear_const

    threshold_linear_current = update_threshold(odometry_d.distance.dist,
                                                laser_d.distance.dist,
                                                threshold_position,
                                                minimum_threshold_linear,
                                                current_time)

    threshold_angular_current = update_threshold(odometry_d.rotation.theta,
                                                 laser_d.rotation.theta,
                                                 threshold_orientation,
                                                 minimum_threshold_angular,
                                                 current_time)


    if len(position_difference.dif) > 0:
        slipping = test_slippage_position(threshold_linear_current, threshold_angular_current, d_p_odom, dif_vector, d_p_laser)
        #str = "%s" % slipping
        pub.publish(slipping)


# -----------------------------------------------------------------------------------------#

def tests_slip_position(threshold_linear_current, threshold_angular_current):
    #global threshold_const, threshold_linear_const
    """
    if threshold_const == True:
        threshold_linear_current = threshold_linear_const
    """

    # DID IT LOST TRACTION IN POSITION ??
    test1 = position_difference.dif[-1] > threshold_linear_current

    # DID IT LOST TRACTION IN ORIENTATION ??

    if len(orientation_difference.dif) > 0:
        test2 = orientation_difference.dif[-1] > threshold_angular_current
    else:
        test2 = False

    # ARE THE WHEELS MOVING MORE THAN THE LASER ??
    # POSITION:
    test3 = abs(odometry_d.distance.dist[-1]) > abs(laser_d.distance.dist[-1])

    # ORIENTATION:
    if len(odometry_d.rotation.theta) > 0:
        test4 = abs(odometry_d.rotation.theta[-1]) > abs(laser_d.rotation.theta[-1])
    else:
        test4 = False

    return test1, test2, test3, test4


#------------------------------------------------------------------------------------------#

def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

#------------------------------------------------------------------------------------------#

def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'::3    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

#------------------------------------------------------------------------------------------#

def vector_size(v1):
    size = math.sqrt(v1[0]*v1[0] + v1[1]*v1[1])
    return size

#------------------------------------------------------------------------------------------#




