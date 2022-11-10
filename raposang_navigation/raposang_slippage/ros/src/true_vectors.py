# ------------ CONTROLLED EXPERIMENTS ------------ #

"""

0 - random
1 - STUCK2.bag
2 - STUCK6.bag
3 - CONTROLLED_TRAJECTORY.bag
4 - TEST_STRAIGHT_TRAJ1.bag
5 - TEST_STRAIGHT_TRAJ2.bag
6 - TEST_SLIDING_POSITION1.bag
7 - TEST_SLIDING_ROTATION1.bag

"""

choice = 7


# random
if choice == 0:
    true_pos = [0, 2.0 , 4.3]
    true_rot = [0, 2.0 , 4.3]
    bag_time = 20.0

# STUCK2.bag
if choice == 1:
    true_pos = [0 , 19.4, 2.3+19.4, 1.4+2.3+19.4, 2.5+1.4+2.3+19.4]
    true_rot = [0]
    bag_time = 45.5

"""
rosbag play --pause --clock -r 5 /home/rute/bagfiles/RAPOSA/controlled_exps/test_position_stuck6.bag
"""
# STUCK6.bag
if choice == 2:
    true_pos = [0 , 3.7 + 0.5 , 3.7+2.5 +  0.5*2, 3.7+2.5+2.3 + 0.5*3, 3.7+2.5+2.3+8.4 + 0.5*4] #+ [0 , 0.5 , 0.5*2, 0.5*3 , 0.5*4]
    true_rot = [0]
    bag_time = 36.5

"""
rosbag play --pause --clock -r 5 /home/rute/bagfiles/RAPOSA/controlled_exps/controlled_trajectopry.bag
"""
# CONTROLLED_TRAJECTORY.bag
if choice == 3:
    true_pos = [0]
    true_rot = [0]
    bag_time = 22.0

"""
rosbag play --pause --clock -r 5 /home/rute/bagfiles/RAPOSA/controlled_exps/test_straight_traj1.bag
"""
# UGLYYYYYY
#TEST_STRAIGHT_TRAJ1.bag
if choice == 4:
    true_pos = [0 , 16.7 ,16.7 + 8.5 , 16.7 + 8.5 + 15.9 , 16.7 + 8.5 + 15.9 + 2.7 ]
    true_rot = [0, 41.4, 41.4 + 2.2]
    bag_time = 56.0

#TEST_STRAIGHT_TRAJ2.bag
if choice == 5:
    true_pos = [0, 11.8 , 11.8+17.8 ]
    true_rot = [0]
    bag_time = 30.5

"""
rosbag play --pause --clock /home/rute/bagfiles/RAPOSA/controlled_exps/test_sliding_position1.bag
"""
#TEST_SLIDING_POSITION1.bag
if choice == 6:
    true_pos = [ 0,
                7.8 ,
                7.8 + 2.2 ,
                7.8 + 2.2 + 2.6 ,
                7.8 + 2.2 + 2.6 + 1.7 ,
                7.8 + 2.2 + 2.6 + 1.7 + 2.8,
                7.8 + 2.2 + 2.6 + 1.7 + 2.8 + 2.0 ,
                7.8 + 2.2 + 2.6 + 1.7 + 2.8 + 2.0 + 2.5 ,
                7.8 + 2.2 + 2.6 + 1.7 + 2.8 + 2.0 + 2.5 + 5.9]
    true_rot = [0]
    bag_time = 40.0

"""
rosbag play --pause --clock /home/rute/bagfiles/RAPOSA/controlled_exps/test_sliding_rotation1.bag
"""
#TEST_SLIDING_ROTATION1.bag
if choice == 7:
    true_pos = [ 0, 5.5 , 5.5+4.9 , 5.5+4.9+3.4, 5.5+4.9+3.4+4.1]
    true_rot = [ 0, 5.5 , 5.5+4.9 , 5.5+4.9+3.4, 5.5+4.9+3.4+4.1]
    bag_time = 40.0








