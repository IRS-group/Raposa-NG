#!/usr/bin/env python
import rospy
import numpy
import math
from geometry_msgs.msg import QuaternionStamped
from idmind_herkulex.msg import TorqueCommand, LedCommand, MotorCommand
from tf.transformations import euler_from_quaternion
from transforms3d import euler, quaternions as qua #https://matthew-brett.github.io/transforms3d/index.html

#--------------------------
# Method to use the Pitch and Roll motors
#--------------------------
class MoveServoPitch(object):

    def __init__(self):
        self.motor = 3
        self.torque_pub = rospy.Publisher("raposang/herkulex/set_torque"          , TorqueCommand , queue_size=10)
        self.led_pub    = rospy.Publisher("raposang/herkulex/set_led"             , LedCommand    , queue_size=10)
        self.angle_pub  = rospy.Publisher("raposang/herkulex/send_motor_command"  , MotorCommand  , queue_size=10)

    def update(self, msg, previous_msg):
        if msg.quaternion !=  previous_msg.quaternion:
            led = LedCommand(self.motor, "green")
            self.led_pub.publish(led)

            # transform form quaternion to euler ZYX take the X and feed it to the motor
            q_for_trans3D = (
                msg.quaternion.w,
                msg.quaternion.x,
                msg.quaternion.y,
                msg.quaternion.z)
            yaw, pitch, roll = euler.quat2euler(q_for_trans3D, 'rzyx')
            pitch = math.degrees(pitch)

            angle = MotorCommand(self.motor, pitch, 10)
            self.angle_pub.publish(angle)
            torque = TorqueCommand(self.motor, True)
            self.torque_pub.publish(torque)

    def run(self):
        pass

class MoveServoRoll(object):

    def __init__(self):
        self.motor = 2
        self.torque_pub = rospy.Publisher("raposang/herkulex/set_torque"  , TorqueCommand , queue_size=10)
        self.led_pub    = rospy.Publisher("raposang/herkulex/set_led"     , LedCommand    , queue_size=10)
        self.angle_pub  = rospy.Publisher("raposang/herkulex/send_motor_command"  , MotorCommand  , queue_size=10)

    def update(self, msg, previous_msg):
        if msg.quaternion !=  previous_msg.quaternion:
            led = LedCommand(self.motor, "green")
            self.led_pub.publish(led)

            # transform form quaternion to euler ZYX take the X and feed it to the motor
            q_for_trans3D = (
                msg.quaternion.w,
                msg.quaternion.x,
                msg.quaternion.y,
                msg.quaternion.z)
            yaw, pitch, roll = euler.quat2euler(q_for_trans3D, 'rzyx')
            roll = math.degrees(roll)

            angle = MotorCommand(self.motor, roll, 10)
            self.angle_pub.publish(angle)
            torque = TorqueCommand(self.motor, True)
            self.torque_pub.publish(torque)

    def run(self):
        pass


class Stabilise(object):
    def __init__(self):

        self.handlers = [
            MoveServoPitch(),
            MoveServoRoll()
        ]
        self.previous_msg = QuaternionStamped()
        rospy.Subscriber("/raposang/laser_transform", QuaternionStamped, self.callback)
        rospy.loginfo("Laser_stabilization: running")

    def callback(self, msg):
        for handler in self.handlers:
            handler.update(msg, self.previous_msg)
        self.previous_msg = msg

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
    rospy.init_node("raposang_laser_stabilizer", anonymous=True)
    node = Stabilise()
    node.run()
    rospy.spin()