import matplotlib.pyplot as plt
import sys


# -----------------------------------------------------------------------------------------#

def data_representation_velocity(method,
                                   threshold_pos,
                                   threshold_rot,
                                   factor,
                                   window,
                                   frequency,
                                   roc_curve_data,
                                 odometry_d,
                                 laser_d,
                                 threshold_position,
                                 position_difference,
                                 threshold_orientation,
                                 orientation_difference):
    params = {'legend.fontsize': 10,
              'axes.labelsize': 15,
              'axes.titlesize': 13,
              'xtick.labelsize': 10,
              'ytick.labelsize': 10}
    plt.rcParams.update(params)

    # GRAPHS:
    plt.figure(1, figsize=(12, 6))

    plt.subplot(2, 2, 1)
    plt.xlabel('time')
    plt.ylabel('|(v_x, v_y)|')
    plt.title('Velocity - position')
    plt.plot(odometry_d.position.velocity.time, odometry_d.position.velocity.velocity, 'b', label="odometry")
    plt.plot(laser_d.position.velocity.time, laser_d.position.velocity.velocity, 'r', label="laser")
    plt.grid()
    plt.legend()

    plt.subplot(2, 2, 2)
    plt.xlabel('time')
    plt.ylabel('w_theta')
    plt.title('Velocity - orientation')
    plt.plot(odometry_d.rotation.velocity.time, odometry_d.rotation.velocity.velocity, 'b', label="odometry")
    plt.plot(laser_d.rotation.velocity.time, laser_d.rotation.velocity.velocity, 'r', label="laser")
    plt.grid()
    plt.legend()

    plt.subplot(2, 2, 3)
    plt.xlabel('time')
    plt.ylabel('v_odom-v_laser')
    plt.title('velocity difference')
    #plt.plot(true_slip_data.true_graph_position.time, true_slip_data.true_graph_position.values, 'r', label="true_slip")
    plt.plot(threshold_position.time, threshold_position.dist, 'y', label="threshold")
    plt.plot(position_difference.time, position_difference.dif, 'g', label="difference")
    #plt.plot(true_slip_data.wrong_position.time, true_slip_data.wrong_position.values, 'k^', label="wrong detections")
    plt.grid()
    plt.legend()

    plt.subplot(2, 2, 4)
    plt.xlabel('time')
    plt.ylabel('w_odom-w_laser')
    plt.title('rotation difference')
    #plt.plot(true_slip_data.true_graph_orientation.time, true_slip_data.true_graph_orientation.values, 'r', label="true_slip")
    plt.plot(threshold_orientation.time, threshold_orientation.dist, 'y', label="threshold")
    plt.plot(orientation_difference.time, orientation_difference.dif, 'g', label="difference")
    #plt.plot(true_slip_data.wrong_orientation.time, true_slip_data.wrong_orientation.values, 'k^',label="wrong detections")
    plt.grid()
    plt.legend()

    plt.tight_layout()

    id_string = str(method) + '_' + str(threshold_pos) + '_' + str(threshold_rot) + '_' + str(factor) + '_' + str(window) + '_' + str(frequency) + '_' + str(roc_curve_data.tp) + '_' + str(roc_curve_data.fp) + '_' + str(roc_curve_data.fn) + '_' + str(roc_curve_data.tn) + ".png"
    location = '/home/rute/Dropbox/Tese/RAPOSA/roc_experiments_figs/'
    total_location = location + id_string
    plt.savefig(total_location)

    #plt.show()
    #print("closing detection...")

    #delete_first_line(file_name_parameters)

    sys.exit()



# -----------------------------------------------------------------------------------------#

def data_visualization(odometry_d,
                                 laser_d,
                                 threshold_position,
                                 position_difference,
                                 threshold_orientation,
                                 orientation_difference):
    params = {'legend.fontsize': 10,
              'axes.labelsize': 15,
              'axes.titlesize': 13,
              'xtick.labelsize': 10,
              'ytick.labelsize': 10}
    plt.rcParams.update(params)


    # GRAPHS:
    plt.figure(1, figsize=(12, 6))

    plt.subplot(2, 2, 1)
    plt.xlabel(r'$t_k$')
    plt.ylabel(r'$\Delta p(t_k)$')
    #plt.title('Displacement: position')
    plt.plot(odometry_d.distance.time, odometry_d.distance.dist, 'b', label=r'$\Delta p(t_k)_{tracks}$',)
    plt.plot(laser_d.distance.time, laser_d.distance.dist, 'r', label="$\Delta p(t_k)_{laser}$")
    plt.grid()
    plt.legend()


    plt.subplot(2, 2, 2)
    plt.xlabel(r'$t_k$')
    plt.ylabel(r'$\Delta \theta(t_k)$')
    #plt.title('Displacement: orientation')
    plt.plot(odometry_d.rotation.time, odometry_d.rotation.theta, 'b', label=r'$\Delta \theta(t_k)_{tracks}$')
    plt.plot(laser_d.rotation.time, laser_d.rotation.theta, 'r', label=r'$\Delta \theta(t_k)_{laser}$')
    plt.grid()
    plt.legend()


    plt.subplot(2, 2, 3)
    plt.xlabel(r'$t_k$')
    plt.ylabel(r'$\delta p(t_k)$')
    #plt.title('Difference: position')
    #plt.plot(true_slip_data.true_graph_position.time, true_slip_data.true_graph_position.values, 'darkslategray', label=r'$true$ $traction$', alpha=0.7)
    plt.plot(threshold_position.time, threshold_position.dist, 'y', label=r'$\eta_p(t_k)$')
    plt.plot(position_difference.time, position_difference.dif, 'g', label=r'$\delta p(t_k)$')
    #plt.plot(true_slip_data.wrong_position.time, true_slip_data.wrong_position.values, "ko", markersize=3, label=r'$wrong$ $detection$', alpha=0.7)
    plt.grid()
    plt.legend()

    plt.subplot(2, 2, 4)
    plt.xlabel(r'$t_k$')
    plt.ylabel(r'$\delta \theta(t_k)$')
    #plt.title('Difference: orientation')
    #plt.plot(true_slip_data.true_graph_orientation.time, true_slip_data.true_graph_orientation.values, 'darkslategray', label=r'$true$ $traction$', alpha=0.7)
    plt.plot(threshold_orientation.time, threshold_orientation.dist, 'y', label=r'$\eta_{\theta}(t_k)$')
    plt.plot(orientation_difference.time, orientation_difference.dif, 'g', label=r'$\delta \theta(t_k)$')
    #plt.plot(true_slip_data.wrong_orientation.time, true_slip_data.wrong_orientation.values, "ko" , markersize=3 ,label=r'$wrong$ $detection$', alpha=0.7)
    plt.grid()
    plt.legend()

    plt.tight_layout()
    plt.show()


    # GRAPHS:
    plt.figure(1, figsize=(12, 6))

    plt.subplot(2, 2, 1)
    plt.xlabel(r'$t_k$')
    plt.ylabel(r'$\Delta p(t_k)$')
    # plt.title('Displacement: position')
    plt.plot(odometry_d.distance.time, odometry_d.distance.dist, 'b', label=r'$\Delta p(t_k)_{tracks}$', )
    plt.plot(laser_d.distance.time, laser_d.distance.dist, 'r', label="$\Delta p(t_k)_{laser}$")
    plt.grid()
    plt.legend()

    plt.subplot(2, 2, 2)
    plt.xlabel(r'$t_k$')
    plt.ylabel(r'$\Delta \theta(t_k)$')
    # plt.title('Displacement: orientation')
    plt.plot(odometry_d.rotation.time, odometry_d.rotation.theta, 'b', label=r'$\Delta \theta(t_k)_{tracks}$')
    plt.plot(laser_d.rotation.time, laser_d.rotation.theta, 'r', label=r'$\Delta \theta(t_k)_{laser}$')
    plt.grid()
    plt.legend()

    plt.subplot(2, 2, 3)
    plt.xlabel(r'$t_k$')
    plt.ylabel(r'$\delta p(t_k)$')
    # plt.title('Difference: position')
    plt.plot(threshold_position.time, threshold_position.dist, 'y', label=r'$\eta_p(t_k)$')
    plt.plot(position_difference.time, position_difference.dif, 'g', label=r'$\delta p(t_k)$')
    #plt.plot(true_slip_data.wrong_position.time, true_slip_data.wrong_position.values, "ko", markersize=3,
             #label=r'$wrong$ $detection$', alpha=0.7)
    plt.grid()
    plt.legend()

    plt.subplot(2, 2, 4)
    plt.xlabel(r'$t_k$')
    plt.ylabel(r'$\delta \theta(t_k)$')
    # plt.title('Difference: orientation')
    plt.plot(threshold_orientation.time, threshold_orientation.dist, 'y', label=r'$\eta_{\theta}(t_k)$')
    plt.plot(orientation_difference.time, orientation_difference.dif, 'g', label=r'$\delta \theta(t_k)$')
    #plt.plot(true_slip_data.wrong_orientation.time, true_slip_data.wrong_orientation.values, "ko", markersize=3,
             #label=r'$wrong$ $detection$', alpha=0.7)
    plt.grid()
    plt.legend()

    plt.tight_layout()
    plt.show()

    # GRAPHS:
    plt.figure(1, figsize=(12, 6))

    plt.subplot(2, 2, 1)
    plt.xlabel(r'$t_k$')
    plt.ylabel(r'$\Delta p(t_k)$')
    # plt.title('Displacement: position')
    plt.plot(odometry_d.distance.time, odometry_d.distance.dist, 'b', label=r'$\Delta p(t_k)_{tracks}$', )
    plt.plot(laser_d.distance.time, laser_d.distance.dist, 'r', label="$\Delta p(t_k)_{laser}$")
    plt.grid()
    plt.legend()

    plt.subplot(2, 2, 2)
    plt.xlabel(r'$t_k$')
    plt.ylabel(r'$\Delta \theta(t_k)$')
    # plt.title('Displacement: orientation')
    plt.plot(odometry_d.rotation.time, odometry_d.rotation.theta, 'b', label=r'$\Delta \theta(t_k)_{tracks}$')
    plt.plot(laser_d.rotation.time, laser_d.rotation.theta, 'r', label=r'$\Delta \theta(t_k)_{laser}$')
    plt.grid()
    plt.legend()

    plt.subplot(2, 2, 3)
    plt.xlabel(r'$t_k$')
    plt.ylabel(r'$\delta p(t_k)$')
    # plt.title('Difference: position')
    plt.plot(threshold_position.time, threshold_position.dist, 'y', label=r'$\eta_p(t_k)$')
    plt.plot(position_difference.time, position_difference.dif, 'g', label=r'$\delta p(t_k)$')
    plt.grid()
    plt.legend()

    plt.subplot(2, 2, 4)
    plt.xlabel(r'$t_k$')
    plt.ylabel(r'$\delta \theta(t_k)$')
    # plt.title('Difference: orientation')
    plt.plot(threshold_orientation.time, threshold_orientation.dist, 'y', label=r'$\eta_{\theta}(t_k)$')
    plt.plot(orientation_difference.time, orientation_difference.dif, 'g', label=r'$\delta \theta(t_k)$')
    plt.grid()
    plt.legend()

    plt.tight_layout()
    plt.show()

    sys.exit()



# -----------------------------------------------------------------------------------------#

def data_visualization2(laser_d,
                                 threshold_position,
                                 position_difference,
                                 threshold_orientation,
                                 orientation_difference):
    params = {'legend.fontsize': 10,
              'axes.labelsize': 15,
              'axes.titlesize': 13,
              'xtick.labelsize': 10,
              'ytick.labelsize': 10}
    plt.rcParams.update(params)


    # GRAPHS:
    plt.figure(1, figsize=(12, 6))

    plt.subplot(2, 2, 1)
    plt.xlabel('time')
    plt.ylabel('|p_sensor|')
    plt.title('Distance per time interval')
    plt.plot(odometry_d.distance.time, odometry_d.distance.dist, 'b', label="odometry")
    plt.plot(laser_d.distance.time, laser_d.distance.dist, 'r', label="laser")
    #plt.plot(laser_d_tf_odom.distance.time, laser_d_tf_odom.distance.dist, 'g', label="world_odom")
    #plt.plot(laser_d_tf_base_link.distance.time, laser_d_tf_base_link.distance.dist, 'y', label="world_base_link")
    plt.grid()
    plt.legend()


    plt.subplot(2, 2, 2)
    plt.xlabel('time')
    plt.ylabel('d_theta')
    plt.title('Rotation per time interval')
    plt.plot(odometry_d.rotation.time , odometry_d.rotation.theta, 'b', label="odometry")
    plt.plot(laser_d.rotation.time, laser_d.rotation.theta, 'r', label="laser")
    plt.grid()
    plt.legend()

    """
    plt.subplot(2, 2, 2)
    plt.xlabel('time')
    plt.ylabel('angle - degrees')
    plt.title('angles between odom and laser position vectors')
    plt.plot(position_difference_vector_angles.time, position_difference_vector_angles.dif, 'b')
    plt.grid()
    #plt.legend()
    """

    plt.subplot(2, 2, 3)
    plt.xlabel('time')
    plt.ylabel('|dif_position|')
    plt.title('Module of the vector dif_pos = p_odom - p_laser')
    #plt.plot(true_slip_data.true_graph_position.time, true_slip_data.true_graph_position.values, 'r', label="true_slip")
    plt.plot(threshold_position.time, threshold_position.dist, 'y', label="threshold")
    plt.plot(position_difference.time, position_difference.dif, 'g', label="difference")
    #plt.plot(true_slip_data.wrong_position.time,true_slip_data.wrong_position.values, 'k^', label="wrong detections")
    #plt.plot(position_difference_vector_dif.time, position_difference_vector_dif.dif, 'g', label="difference")


    plt.grid()
    plt.legend()


    plt.subplot(2, 2, 4)
    plt.xlabel('time')
    plt.ylabel('|dif_orientation|')
    plt.title('Module of the vector dif_rot = p_odom - p_laser')
    #plt.plot(true_slip_data.true_graph_orientation.time, true_slip_data.true_graph_orientation.values, 'r', label="true_slip")
    plt.plot(threshold_orientation.time, threshold_orientation.dist, 'y',label="threshold")
    plt.plot(orientation_difference.time, orientation_difference.dif, 'g', label="difference")
    #plt.plot(true_slip_data.wrong_orientation.time, true_slip_data.wrong_orientation.values, 'k^', label="wrong detections")
    plt.grid()
    plt.legend()

    plt.tight_layout()

    plt.show()
    sys.exit()
