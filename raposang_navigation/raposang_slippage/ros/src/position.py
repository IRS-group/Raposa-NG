#!/usr/bin/env python2

from functions import *
from std_msgs.msg import Int16

#------------------------------------------------------------------------------------------#

def callback_posed2D(data):
    global atualizar_laser, laser_raw, current_time, listener

    if atualizar_laser == True:
        laser_d.trajectory.add_new_position(data.x, data.y, data.theta, current_time)
        atualizar_laser = False

#------------------------------------------------------------------------------------------#

def callback_odom(data):

    global atualizar_odom, current_time

    if atualizar_odom == True:
        quaternions = (data.pose.pose.orientation.x,
                       data.pose.pose.orientation.y,
                       data.pose.pose.orientation.z,
                       data.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternions)
        odometry_d.trajectory.add_new_position(data.pose.pose.position.x,
                                               data.pose.pose.position.y,
                                               euler[-1],
                                               current_time)
        atualizar_odom = False

# ------------------------------------------------------------------------------------------#

"""
def callback_slip(data):
    global first, duration, now, begining_time

    if first == True and begining_time!=0:
        duration_secs = rospy.Time.now().secs - begining_time.secs
        duration_nsecs = rospy.Time.now().nsecs - begining_time.nsecs
        duration = duration_secs + duration_nsecs * 0.0000000001
        first = False
"""

# ------------------------------------------------------------------------------------------#

def estimate_displacement(list_x, list_y, list_theta):
    t0 = -size_moving_window    # previous
    t1 = -1             # current

    if len(list_theta) > -t0:

        #   d_x_1 = cos(theta_0) * (x_1 -x_0) + sin(theta_0) * (y_1 -y_0)
        d_x = math.cos(list_theta[t0]) * (list_x[t1] - list_x[t0]) + math.sin(list_theta[t0]) * (list_y[t1] - list_y[t0])

        #   d_y_1 = -sin(theta_0) * (x_1 -x_0) + cos(theta_0) * (y_1 -y_0)
        d_y = -math.sin(list_theta[t0]) * (list_x[t1] - list_x[t0]) + math.cos(list_theta[t0]) * (list_y[t1] - list_y[t0])

        theta0 = angles_minus_pi_to_pi(list_theta[t0])
        theta1 = angles_minus_pi_to_pi(list_theta[t1])

        d_theta = angles_minus_pi_to_pi( theta1 - theta0)

    else:
        d_x = 0
        d_y = 0
        d_theta = 0

    return d_x, d_y, d_theta



# ------------------------------------------------------------------------------------------#

def update_threshold(list_vel_odom, list_vel_laser, threshold_list, minimum_threshold):
    if len(list_vel_odom) > 0 and len(list_vel_laser) > 0:
        thresh = max(abs(list_vel_odom[-1]), abs(list_vel_laser[-1])) * thresh_factor
        if thresh < minimum_threshold:
            thresh = minimum_threshold
        threshold_list.append(thresh)

        return thresh

# -----------------------------------------------------------------------------------------#


def vector_size(v1):
    size = math.sqrt(v1[0] * v1[0] + v1[1] * v1[1])
    return size


# -----------------------------------------------------------------------------------------#
#                                   MAIN
#------------------------------------------------------------------------------------------#

def estimate_positions():
    global atualizar_odom, atualizar_laser, current_time, now, duration, begining_time, t_slip, wrong_time, wrong_data

    global listener, dif_vector, d_p_odom

    current_time = 0.0
    interval = 0.0

    wrong_time = []
    wrong_data = []

    #print("subscribing...")

    rospy.init_node('traction_detection')


    # PUBLISHING:
    pub = rospy.Publisher('slipping_broadcaster', Int16, queue_size=1)  # name of topic; message type

    # SUBSCRIBING:
    rospy.Subscriber("pose2D", Pose2D, callback_posed2D)  # From laser_scan_matcher - data from laser
    rospy.Subscriber("/raposang/odometry_pose", Odometry, callback_odom)
    #rospy.Subscriber("/republished/joy", Joy, callback_slip)


    listener = tf.TransformListener()

    #print("done")

    # NODE:


    # SAMPLING FREQUENCY:
    rate = rospy.Rate(frequency)  # hz

    print("Let's start detecting using displacement vectors...")

    while not rospy.is_shutdown():

        # ---------------------- TRUE VALUES ------------------------------- #

        now = rospy.Time.now()
        if now.secs != 0 and begining_time == 0:
            begining_time = now
            #print(begining_time.secs, begining_time.nsecs)

        #interval = true_values_general(interval, current_time)


        # ---------------------- DATA PROCESSING --------------------------- #

        d_x_odom, d_y_odom, d_theta_odom = estimate_displacement(odometry_d.trajectory.x,
                                                     odometry_d.trajectory.y,
                                                     odometry_d.trajectory.theta)
        # Displacement Vector from Odometry
        d_p_odom = np.array([d_x_odom, d_y_odom])


        d_x_laser, d_y_laser, d_theta_laser = estimate_displacement(laser_d.trajectory.x,
                                                     laser_d.trajectory.y,
                                                     laser_d.trajectory.theta)
        # Displacement Vector from Laser
        d_p_laser = np.array([d_x_laser, d_y_laser])

        """
        if len(laser_d.trajectory.theta)>0:
            print("theta_laser",laser_d.trajectory.theta[-1]*180/math.pi)
            print("theta_odom", odometry_d.trajectory.theta[-1] * 180 / math.pi)

        """

        if d_x_odom < 95 and d_x_laser < 95 and d_y_odom < 95 and d_y_laser < 95:     # Eliminate crazy big/ non-sense values

            # Saving Trajectory given by both sensors for representation/visualization purposes:
            odometry_d.position.add_value(d_x_odom, d_y_odom, current_time)
            laser_d.position.add_value(d_x_laser, d_y_laser, current_time)

            # GET DISPLACEMENT DIFFERENCES AND RESPECTIVE MODULE:
            dif_vector = d_p_odom - d_p_laser
            dif_module = vector_size(dif_vector)

            # MODULE OF DISPLACEMENT VECTORS OF POSITION:
            displacement_odom = vector_size(d_p_odom)
            displacement_laser = vector_size(d_p_laser)

            # SAVING DATA FOR ANALYSIS
            odometry_d.distance.add_value(displacement_odom, current_time)
            laser_d.distance.add_value(displacement_laser, current_time)

            position_difference.add_value(dif_module, current_time)


        if abs(d_theta_odom) < 1500 and abs(d_theta_laser) < 1500:          # Eliminate crazy big/ non-sense values

            odometry_d.rotation.add_value(d_theta_odom, current_time)
            laser_d.rotation.add_value(d_theta_laser, current_time)

            dif = abs(laser_d.rotation.latest() - odometry_d.rotation.latest())
            orientation_difference.add_value(dif, current_time)


        # ----------------- DATA SAVING AND VISUALIZATION ----------------- #
        #bag_time=12.8
        #if current_time > bag_time:
            #print("tp:", roc_data.tp , "fp:", roc_data.fp, "tn:", roc_data.tn, "fn: ", roc_data.fn)

            #write_roc_data_to_file(method_roc,
            #                        threshold_roc_pos,
            #                        threshold_roc_rot,
            #                        factor_roc,
            #                        window_roc,
            #                        frequency_roc,
            #                        roc_data)


            #data_visualization(method_roc,
            #                       threshold_roc_pos,
            #                       threshold_roc_rot,
            #                       factor_roc,
            #                       window_roc,
            #                       frequency_roc,
            #                       roc_data,
            #                       testing_file)

        # ----------------------------------------------------------------- #



        current_time = current_time + T_sample
        #print(current_time)


        if len(position_difference.dif) > 0:
            publish_slippage_position(pub, position_difference.dif[-1], orientation_difference.dif[-1], current_time, d_p_odom, dif_vector, d_p_laser)

        rate.sleep()
        atualizar_odom = True
        atualizar_laser = True



#------------------------------------------------------------------------------------------#


if __name__ == '__main__':

    try:

        """
        minimum_threshold_linear, minimum_threshold_angular, thresh_factor, frequency, T_sample, size_moving_window = get_testing_parameters(testing_file)
        print(minimum_threshold_linear, minimum_threshold_angular, thresh_factor, size_moving_window, frequency, T_sample)
        """
        #frequency_roc = frequency
        #window_roc = size_moving_window
        #threshold_roc_pos = minimum_threshold_linear
        #threshold_roc_rot = minimum_threshold_angular
        #factor_roc = thresh_factor

        estimate_positions()

    except rospy.ROSInterruptException: pass