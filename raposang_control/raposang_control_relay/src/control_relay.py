#!/usr/bin/env python
import rospy
import numpy
import math
import time
from idmind_herkulex.msg import TorqueCommand, LedCommand, MotorCommand, Motors
from idmind_motorsboard.msg import WheelsMB
from raposang_control_relay.msg import Tracks
from std_msgs.msg import Int32, Float32
from geometry_msgs.msg import Vector3


#----------------
# Definition Values
#----------------

#Pan & Tilt Positions
PAN_MIN = -90.0 #-80
PAN_MAX = 90.0 #80
TILT_MIN = -70.0
TILT_MAX = 50.0
HERK_VEL = 10

#Arm value positions
ARM_MAX = 873.0
ARM_MIN = 750.0
ARM_MID = 812.0
ARM_MIN_RANGE = -1.0
ARM_MAX_RANGE = 1.0

#Tracks velocity scale values
VEL_BASE = 0.4
VEL_TURBO = 1.0
VEL_SLOW = 0.2


#----------------
# Move Herkulex motors
#----------------
class MoveServoPan(object):
    def __init__(self):
        self.motor = 0
        #self.next_pos = 0 # angle to move to
        self.prev_pos = 0 # previous angle to move to
        self.torque_pub = rospy.Publisher("idmind_herkulex/set_torque"          , TorqueCommand , queue_size=10)
        self.led_pub    = rospy.Publisher("idmind_herkulex/set_led"             , LedCommand    , queue_size=10)
        self.angle_pub  = rospy.Publisher("idmind_herkulex/send_motor_command"  , MotorCommand  , queue_size=10)

    def update(self, pos):
        if pos != self.prev_pos:
            led = LedCommand(self.motor, "green")
            self.led_pub.publish(led)
            torque = TorqueCommand(self.motor, True)
            self.torque_pub.publish(torque)

            if PAN_MIN <= pos <= PAN_MAX:
                angle = MotorCommand(self.motor, pos, HERK_VEL)
                self.angle_pub.publish(angle)
            elif pos < PAN_MIN:
                angle = MotorCommand(self.motor, PAN_MIN, HERK_VEL)
                self.angle_pub.publish(angle)
            elif pos > PAN_MAX:
                angle = MotorCommand(self.motor, PAN_MAX, HERK_VEL)
                self.angle_pub.publish(angle)

        self.prev_pos = pos

    def run(self):
        pass

class MoveServoTilt(object):
    def __init__(self):
        self.motor = 1
        #self.next_pos = 0  # angle to move to
        self.prev_pos = 0  # previous angle to move to
        #self.no_command = True
        #rospy.Subscriber("/raposang/herkulex/motor_status", Motors, self.on_motor_mov)
        self.torque_pub = rospy.Publisher("idmind_herkulex/set_torque"  , TorqueCommand , queue_size=10)
        self.led_pub    = rospy.Publisher("idmind_herkulex/set_led"     , LedCommand    , queue_size=10)
        self.angle_pub  = rospy.Publisher("idmind_herkulex/send_motor_command"  , MotorCommand  , queue_size=10)


    def update(self, pos):
        if pos != self.prev_pos:
            led = LedCommand(self.motor, "green")
            self.led_pub.publish(led)
            torque = TorqueCommand(self.motor, True)
            self.torque_pub.publish(torque)

            if TILT_MIN <= pos <= TILT_MAX:
                angle = MotorCommand(self.motor, pos, HERK_VEL)
                self.angle_pub.publish(angle)
            elif pos < TILT_MIN:
                angle = MotorCommand(self.motor, TILT_MIN, HERK_VEL)
                self.angle_pub.publish(angle)
            elif pos > TILT_MAX:
                angle = MotorCommand(self.motor, TILT_MAX, HERK_VEL)
                self.angle_pub.publish(angle)

        self.prev_pos = pos

    def run(self):
        pass

class Control_relay(object):
    def __init__(self):
        self.handlers = [
            MoveServoPan(),
            MoveServoTilt()
        ]

        rospy.Subscriber("input_pantilt", Vector3, self.move_pantilt)
        rospy.Subscriber("input_arm", Float32, self.move_arm)
        rospy.Subscriber("input_velocity", Tracks, self.move_tracks)

        rospy.Subscriber("/idmind_motors/arm", Int32, self.on_arm_move)
        rospy.Subscriber("/idmind_motors/motors/ticks", WheelsMB, self.on_tracks_move)
        self.tracks_pub = rospy.Publisher("/idmind_motors/set_velocities", WheelsMB, queue_size=10)
        self.arm_pub = rospy.Publisher("/idmind_motors/set_arm", Int32, queue_size=10)
        self.arm = ARM_MID
        self.velR = 0.0
        self.velL = 0.0
        self.vel_scale = VEL_BASE
        rospy.loginfo("Raposang control relay: running")

    ##### Pan & Tilt Area #####
    def move_pantilt(self, msg):
        self.handlers[0].update(msg.z) #Pan
        self.handlers[1].update(msg.y) #Tilt

    ##### End Pan & Tilt #####

    ##### Arm Area #####
    def remap_range(self, arm_pos):
        """
        Remaps the values from the msg to the values of the linear motor position
        :param arm_pos: value from the joy relating to arm position
        """
        portion = ((arm_pos - ARM_MIN_RANGE) * (ARM_MAX - ARM_MIN)) / (ARM_MAX_RANGE - ARM_MIN_RANGE)
        self.arm = portion + ARM_MIN

    # Function to verify if the arm is on the desired position
    def on_arm_move(self, msg_arm):
        if msg_arm != self.arm:
            self.set_arm()

    def set_arm(self):
        msg_arm = Int32()
        msg_arm.data = self.arm
        self.arm_pub.publish(msg_arm)

    def move_arm(self, msg):
        self.remap_range(msg.data)
    ####### End Arm #####

    ##### Tracks Area ######
    def velocity_mode(self, mode):
        if mode == 0:
            self.vel_scale = VEL_SLOW
        elif mode == 2:
            self.vel_scale = VEL_TURBO
        else:
            self.vel_scale = VEL_BASE

    def clamp(self, n, minn, maxn):
        """
        Clamps the values from the motor velocity to certain values
        :param n: input value
        :param minn: minimum allowed value
        :param maxn: maximum allowed value
        """
        return max(min(maxn, n), minn)

    # Function to keep giving commands to the tracks motors
    def on_tracks_move(self, m):
        msg = WheelsMB()
        msg.header.frame_id = "odom"
        msg.header.stamp = rospy.Time.now()
        msg.front_right = self.velR
        msg.front_left = self.velL
        self.tracks_pub.publish(msg)

    def set_tracks(self, linVel, angVel):
        y = linVel
        x = angVel

        vr= self.clamp((x + y),-1,1)
        vl= self.clamp((x - y),-1,1)

        self.velR = numpy.interp(vr, [-1, 1], [-self.vel_scale, self.vel_scale])
        self.velL = numpy.interp(vl, [-1, 1], [-self.vel_scale, self.vel_scale])
        # backtrack = (y < 0);

    def move_tracks(self, msg):
        self.velocity_mode(msg.mode)
        self.set_tracks(msg.linear_vel, msg.angular_vel)
    ###### End Tracks #######

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            for handler in self.handlers:
                handler.run()
            rate.sleep()

if __name__ == '__main__':
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node("raposang_control_relay", anonymous=True)
    node = Control_relay()
    node.run()
    rospy.spin()

