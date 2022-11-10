#!/usr/bin/env python
import rospy
import numpy
import math
import time
from sensor_msgs.msg import Joy
from raposang_control_relay.msg import Tracks
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3

#----------------
# Definition Values
#---------------
TIMER = 500.0

#Arm value positions
ARM_MAX = 1
ARM_MIN = -1
ARM_MID = 0

#Pan & Tilt Values
PAN_MAX = 80
PAN_MIN = -80
TILT_MAX = 50
TILT_MIN = -70


#----------------
# Move Herkulex motors
#----------------
class MoveServoPanTilt(object):
    def __init__(self):
        self.next_pos = Vector3()
        self.timer = rospy.Timer(rospy.Duration(1.0 / TIMER), self.on_timer) # This is a workaround to initiate the timer
        rospy.Timer.shutdown(self.timer)
        self.stop = True
        self.incrementPAN = 0
        self.incrementTILT = 0
        self.pantilt_pub = rospy.Publisher("output_pantilt", Vector3, queue_size=10)

    def on_timer(self, event):
        self.next_pos.z = max(min(PAN_MAX, self.next_pos.z + self.incrementPAN), PAN_MIN)
        self.next_pos.y = max(min(TILT_MAX, self.next_pos.y + self.incrementTILT), TILT_MIN)
        self.move_motor()

    def reset_pos(self):
        self.next_pos = Vector3()
        self.move_motor()

    def update(self, axes, msg):
        self.stop = True
        #PAN
        if msg.axes[axes['stereo_cam_pan']] > 0.1:
            self.incrementPAN = 0.1
            self.stop = False
        elif msg.axes[axes['stereo_cam_pan']] < -0.1:
            self.incrementPAN = -0.1
            self.stop = False
        elif -0.1 < msg.axes[axes['stereo_cam_pan']] < 0.1:
            self.incrementPAN = 0
        #TILT
        if msg.axes[axes['stereo_cam_tilt']] > 0.1:
            self.incrementTILT = -0.1
            self.stop = False
        elif msg.axes[axes['stereo_cam_tilt']] < -0.1:
            self.incrementTILT = 0.1
            self.stop = False
        elif -0.1 < msg.axes[axes['stereo_cam_tilt']] < 0.1:
            self.incrementTILT = 0

        if self.stop:
            rospy.Timer.shutdown(self.timer)
        else:
            rospy.Timer.shutdown(self.timer) # This is a workaround to ensure no other timer is running
            self.timer = rospy.Timer(rospy.Duration(1.0 / TIMER), self.on_timer)

    def move_motor(self):
        self.pantilt_pub.publish(self.next_pos)

    def run(self):
        pass

class Teleop(object):
    def __init__(self):
        #####################
        #  Joystick VARIABLES
        #####################
        # Check for Axes information
        try:
            self.axes = rospy.get_param("/axes")
        except KeyError:
            self.log("No axes information was found", 1, alert="err")
            raise KeyError("No axes information was found")

        # Check for Buttons information
        try:
            self.buttons = rospy.get_param("/buttons")
        except KeyError:
            self.log("No buttons information was found", 1, alert="err")
            raise KeyError("No buttons information was found")

        # Check for Axis range of input information
        try:
            self.axis_range = rospy.get_param("/joy_arm_range")
        except KeyError:
            self.log("No axis range information was found", 1, alert="err")
            raise KeyError("No axis range information was found")

        # Check for Joy axes range
        try:
            self.joy_range = rospy.get_param("/axes_range")
        except KeyError:
            self.log("No axis range information was found", 1, alert="err")
            raise KeyError("No axis range information was found")

        self.pantilt = MoveServoPanTilt()
        self.previous_msg = Joy()
        self.previous_msg.axes = [0] * 8
        self.previous_msg.buttons = [0] * 14
        rospy.Subscriber("/joy", Joy, self.on_joy)
        #rospy.Subscriber("/idmind_motors/motors/ticks", WheelsMB, self.on_tracks_move)
        self.tracks_pub = rospy.Publisher("output_tracks", Tracks, queue_size=10)
        self.arm_pub = rospy.Publisher("output_arm", Float32, queue_size=10)

        self.arm = 0
        self.vel_scale = 1

        rospy.loginfo("Raposang Teleop: running")

    def remap_range(self, msg_joy_arm):
        """
        Remaps the values from the joy to the values of the linear motor position
        :param msg_joy_arm: value from the joy relating to arm position
        """
        portion = ((msg_joy_arm - self.axis_range['min']) * (ARM_MAX - ARM_MIN)) / (self.axis_range['max'] - self.axis_range['min'])
        self.arm = portion + ARM_MIN

    def set_arm(self):
        msg_arm = Float32()
        msg_arm.data = self.arm
        self.arm_pub.publish(msg_arm)

    def switch_lights(self):
        rospy.wait_for_service('/idmind_sensors/switch_lights')
        try:
            switch = rospy.ServiceProxy('/idmind_sensors/switch_lights', Trigger)
            switch()
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def set_velocity_scale(self, msg_buttons):
        on_turbo = msg_buttons[self.buttons['turbo']]
        on_slow = msg_buttons[self.buttons['slow']]

        if on_turbo != 0 and on_slow == 0:
            self.vel_scale = 2
        elif on_turbo == 0 and on_slow != 0:
            self.vel_scale = 0
        else:
            self.vel_scale = 1

    def move_tracks(self, msg_vel):
        msg = Tracks()
        msg.header.frame_id = "Joy"
        msg.header.stamp = rospy.Time.now()
        msg.mode = self.vel_scale

        vell = msg_vel[self.axes['linear_vel']]
        velr = msg_vel[self.axes['angular_vel']]

        msg.linear_vel = vell
        msg.angular_vel = velr
        self.tracks_pub.publish(msg)

    def on_joy(self, msg_joy):
        if self.joy_range == len(msg_joy.axes):
            # Move Stereo Camera Pan and Tilt
            on_reset_stereo = msg_joy.buttons[self.buttons['reset_stereo_pos']]
            if on_reset_stereo != 0:
                self.pantilt.reset_pos()

            self.pantilt.update(self.axes, msg_joy)
            #----------
            # Move Tracks with Turbo or Slow or Base velocity
            self.set_velocity_scale(msg_joy.buttons)
            self.move_tracks(msg_joy.axes)
            #----------
            # Move Arm to Joy position
            if msg_joy.axes[self.axes['arm_pos']] != self.previous_msg.axes[self.axes['arm_pos']]:
                if self.buttons.has_key('act_arm'): #to check if using joystick or gamepad
                    if msg_joy.buttons[self.buttons['act_arm']] !=0:
                        self.remap_range(msg_joy.axes[self.axes['arm_pos']])
                else:
                    self.remap_range(msg_joy.axes[self.axes['arm_pos']])
                self.set_arm()
            #----------
            # Move Arm to predefined positions
            if msg_joy.buttons[self.buttons['min_arm']] != 0:
                self.arm = ARM_MIN
                self.set_arm()

            if msg_joy.buttons[self.buttons['max_arm']] != 0:
                self.arm = ARM_MAX
                self.set_arm()

            if msg_joy.buttons[self.buttons['mid_arm']] != 0:
                self.arm = ARM_MID
                self.set_arm()
            # ----------
            if msg_joy.buttons[self.buttons['lights']] != 0:
                self.switch_lights()
            self.previous_msg = msg_joy

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.pantilt.run()
            rate.sleep()

if __name__ == '__main__':
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node("raposang_teleop", anonymous=True)
    node = Teleop()
    node.run()
    rospy.spin()

