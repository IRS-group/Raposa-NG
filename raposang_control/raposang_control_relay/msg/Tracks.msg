# This message is used for track velocity communication with the control relay.
# Receives inputs to tracks velocity and mode, then relays that information to the hardware
std_msgs/Header header	# Typical header
int32 mode         	# Defines the mode tracks mode 0-Slow, 1-Base, 2-Turbo 
float32 linear_vel     	# Gives the linear velocity of the robot [-1,1]
float32 angular_vel     # Gives the angular velocity of the robot [-1,1]
