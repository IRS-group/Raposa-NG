#! /usr/bin/env python


import rospy
from sensor_msgs.msg import Joy

from idmind_herkulex.msg import TorqueCommand, LedCommand, MotorCommand


class X(object):
    # axes
    LX = 0
    LY = 1
    LT = 2
    RX = 3
    RY = 4
    RT = 5
    AX = 6
    AY = 7

    # buttons
    A = 0
    B = 1
    X = 2
    Y = 3
    LB = 4
    RB = 5
    BACK = 6
    START = 7
    LK = 9
    RK = 10


class D(object):
    # axes
    LX = 0
    LY = 1
    RX = 2
    RY = 3
    AX = 4
    AY = 5

    # buttons
    X = 0
    A = 1
    B = 2
    Y = 3
    LB = 4
    RB = 5
    LT = 6
    RT = 7
    BACK = 8
    START = 9
    LK = 10
    RK = 11


class MoveServo0(object):

    def __init__(self):
        self.motor = 0
        self.torque_pub = rospy.Publisher("idmind_herkulex/set_torque"          , TorqueCommand , queue_size=10)
        self.led_pub    = rospy.Publisher("idmind_herkulex/set_led"             , LedCommand    , queue_size=10)
        self.angle_pub  = rospy.Publisher("idmind_herkulex/send_motor_command"  , MotorCommand  , queue_size=10)

    def update(self, msg, previous_msg, mode):
        if msg.axes[mode.RX] > 0.1 and not previous_msg.axes[mode.RX] > 0.1:
            led = LedCommand(self.motor, "green")
            self.led_pub.publish(led)
            angle = MotorCommand(self.motor, 150, 250)
            self.angle_pub.publish(angle)
            torque = TorqueCommand(self.motor, True)
            self.torque_pub.publish(torque)

        if msg.axes[mode.RX] < -0.1 and not previous_msg.axes[mode.RX] < -0.1:
            led = LedCommand(self.motor, "green")
            self.led_pub.publish(led)
            angle = MotorCommand(self.motor, -150, 250)
            self.angle_pub.publish(angle)
            torque = TorqueCommand(self.motor, True)
            self.torque_pub.publish(torque)

        if -0.1 < msg.axes[mode.RX] < 0.1 and not -0.1 < previous_msg.axes[mode.RX] < 0.1:
            led = LedCommand(self.motor, "black")
            self.led_pub.publish(led)
            torque = TorqueCommand(self.motor, False)
            self.torque_pub.publish(torque)

    def run(self):
        pass

class MoveServo1(object):

    def __init__(self):
        self.motor = 1
        self.torque_pub = rospy.Publisher("idmind_herkulex/set_torque"  , TorqueCommand , queue_size=10)
        self.led_pub    = rospy.Publisher("idmind_herkulex/set_led"     , LedCommand    , queue_size=10)
        self.angle_pub  = rospy.Publisher("idmind_herkulex/send_motor_command"  , MotorCommand  , queue_size=10)

    def update(self, msg, previous_msg, mode):
        if msg.axes[mode.RY] > 0.1 and not previous_msg.axes[mode.RY] > 0.1:
            led = LedCommand(self.motor, "green")
            self.led_pub.publish(led)
            angle = MotorCommand(self.motor, 150, 250)
            self.angle_pub.publish(angle)
            torque = TorqueCommand(self.motor, True)
            self.torque_pub.publish(torque)

        if msg.axes[mode.RY] < -0.1 and not previous_msg.axes[mode.RY] < -0.1:
            led = LedCommand(self.motor, "green")
            self.led_pub.publish(led)
            angle = MotorCommand(self.motor, -150, 250)
            self.angle_pub.publish(angle)
            torque = TorqueCommand(self.motor, True)
            self.torque_pub.publish(torque)

        if -0.1 < msg.axes[mode.RY] < 0.1 and not -0.1 < previous_msg.axes[mode.RY] < 0.1:
            led = LedCommand(self.motor, "black")
            self.led_pub.publish(led)
            torque = TorqueCommand(self.motor, False)
            self.torque_pub.publish(torque)

    def run(self):
        pass

class MoveServo2(object):

    def __init__(self):
        self.motor = 2
        self.torque_pub = rospy.Publisher("idmind_herkulex/set_torque"  , TorqueCommand , queue_size=10)
        self.led_pub    = rospy.Publisher("idmind_herkulex/set_led"     , LedCommand    , queue_size=10)
        self.angle_pub  = rospy.Publisher("idmind_herkulex/send_motor_command"  , MotorCommand  , queue_size=10)

    def update(self, msg, previous_msg, mode):
        if msg.axes[mode.LX] > 0.1 and not previous_msg.axes[mode.LX] > 0.1:
            led = LedCommand(self.motor, "green")
            self.led_pub.publish(led)
            angle = MotorCommand(self.motor, 150, 250)
            self.angle_pub.publish(angle)
            torque = TorqueCommand(self.motor, True)
            self.torque_pub.publish(torque)

        if msg.axes[mode.LX] < -0.1 and not previous_msg.axes[mode.LX] < -0.1:
            led = LedCommand(self.motor, "green")
            self.led_pub.publish(led)
            angle = MotorCommand(self.motor, -150, 250)
            self.angle_pub.publish(angle)
            torque = TorqueCommand(self.motor, True)
            self.torque_pub.publish(torque)
            
        if -0.1 < msg.axes[mode.LX] < 0.1 and not -0.1 < previous_msg.axes[mode.LX] < 0.1:
            led = LedCommand(self.motor, "black")
            self.led_pub.publish(led)
            torque = TorqueCommand(self.motor, False)
            self.torque_pub.publish(torque)

    def run(self):
        pass

class MoveServo3(object):

    def __init__(self):
        self.motor = 3
        self.torque_pub = rospy.Publisher("idmind_herkulex/set_torque"  , TorqueCommand , queue_size=10)
        self.led_pub    = rospy.Publisher("idmind_herkulex/set_led"     , LedCommand    , queue_size=10)
        self.angle_pub  = rospy.Publisher("idmind_herkulex/send_motor_command"  , MotorCommand  , queue_size=10)

    def update(self, msg, previous_msg, mode):
        if msg.axes[mode.LY] > 0.1 and not previous_msg.axes[mode.LY] > 0.1:
            led = LedCommand(self.motor, "green")
            self.led_pub.publish(led)
            angle = MotorCommand(self.motor, 150, 250)
            self.angle_pub.publish(angle)
            torque = TorqueCommand(self.motor, True)
            self.torque_pub.publish(torque)

        if msg.axes[mode.LY] < -0.1 and not previous_msg.axes[mode.LY] < -0.1:
            led = LedCommand(self.motor, "green")
            self.led_pub.publish(led)
            angle = MotorCommand(self.motor, -150, 250)
            self.angle_pub.publish(angle)
            torque = TorqueCommand(self.motor, True)
            self.torque_pub.publish(torque)
            
        if -0.1 < msg.axes[mode.LY] < 0.1 and not -0.1 < previous_msg.axes[mode.LY] < 0.1:
            led = LedCommand(self.motor, "black")
            self.led_pub.publish(led)
            torque = TorqueCommand(self.motor, False)
            self.torque_pub.publish(torque)

    def run(self):
        pass


class Teleop(object):
    def __init__(self):

        self.handlers = [
            MoveServo0(),
            MoveServo1(),
            MoveServo2(),
            MoveServo3()
        ]

        self.previous_msg = Joy()
        self.previous_msg.axes = [0] * 8
        self.previous_msg.buttons = [0] * 12
        rospy.Subscriber("/joy", Joy, self.on_joy)
        rospy.loginfo("joy_teleop: running")

    def on_joy(self, msg):
        mode = X if len(msg.axes) == 8 else D
        for handler in self.handlers:
            handler.update(msg, self.previous_msg, mode)
        self.previous_msg = msg

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            for handler in self.handlers:
                handler.run()
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node("joy_teleop")
    node = Teleop()
    node.run()
    rospy.spin()
