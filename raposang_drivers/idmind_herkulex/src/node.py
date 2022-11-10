#! /usr/bin/env python


from builtins import range
from builtins import object
import herkulex
from herkulex import servo

import rospy
from idmind_herkulex.msg import TorqueCommand, LedCommand, MotorCommand, Motors, MotorStatus


class Colour:
    BLACK = 0x00
    GREEN = 0x04
    BLUE = 0x08
    RED = 0x10


class Node(object):

    def __init__(self):
        try:
            port = rospy.get_param("/port")
            default_angles = rospy.get_param("/default_angles") if rospy.has_param("/default_angles") else [0, 0, 0, 0]
            self.servo_ids = rospy.get_param("/servos")
            n_servos = len(self.servo_ids)
            self.angle_min = -150 
            self.angle_max = 150
            self.playtime_min = 10
            self.playtime_max = 250

            herkulex.connect(port, 115200)
            herkulex.clear_errors()
            self.servos = [servo(id) for id in self.servo_ids]
            self.previous_torque_cmd = [None] * n_servos
            self.torque_cmd = [True] * n_servos
            self.previous_colors = [None] * n_servos
            self.colors = [ Colour.BLACK ] * n_servos
            self.previous_angles = [None] * n_servos
            self.angles = [ a for a in default_angles if self.angle_min < a < self.angle_max ]
            self.playtimes = [ 100 ] * n_servos

            self.color_names = {
                "black" : Colour.BLACK,
                "red"   : Colour.RED,
                "green" : Colour.GREEN,
                "blue"  : Colour.BLUE
            }
            
            self.initialization_routine()

        except herkulex.HerkulexError as e:
            rospy.logerr("failed to initialize herkulex. reason: %s" % e)
        
        rospy.Subscriber("idmind_herkulex/set_torque"   , TorqueCommand , self.on_torque)
        rospy.Subscriber("idmind_herkulex/set_led"      , LedCommand    , self.on_led)
        rospy.Subscriber("idmind_herkulex/send_motor_command"    , MotorCommand  , self.on_motor_command)

        self.pub = rospy.Publisher("idmind_herkulex/motor_status" , Motors  , queue_size=10)

    def on_torque(self, msg):
        motor = self.servo_ids.index(msg.motor)
        enabled = msg.enabled
        self.torque_cmd[motor] = enabled 

    def on_led(self, msg):
        motor = self.servo_ids.index(msg.motor)
        color = self.color_names[msg.color]
        self.colors[motor] = color

    def on_motor_command(self, msg):
        motor = self.servo_ids.index(msg.motor)
        angle = msg.angle
        playtime = msg.playtime
        if motor not in self.servo_ids                                  : return
        if not self.angle_min       <= angle        <= self.angle_max   : return
        if not self.playtime_min    <= msg.playtime <= self.playtime_max: return
        
        self.angles[motor]      = angle
        self.playtimes[motor]   = playtime

    def initialization_routine(self):
        for s in self.servos:
            s.set_led(0x03)
        rospy.sleep(0.2)
        for s in self.servos:
            s.set_led(0x00)
        rospy.sleep(0.5)
        for s in self.servos:
            s.set_led(0x03)
        rospy.sleep(0.2)
        for s in self.servos:
            s.set_led(0x00)
        rospy.sleep(0.5)
        for s in self.servos:
            s.set_led(0x03)
        rospy.sleep(0.2)
        for s in self.servos:
            s.set_led(0x00)

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            # build motor status array
            motors_msg = Motors()

            for i in range(4):
                # access servo
                s = self.servos[i]

                # toggle torque
                if self.previous_torque_cmd[i] != self.torque_cmd[i]:
                    s.torque_on() if self.torque_cmd[i] else s.torque_off()
                    self.previous_torque_cmd[i] = self.torque_cmd[i]

                # send command
                if self.previous_angles[i] != self.angles[i] or self.previous_colors[i] != self.colors[i]:
                    s.set_servo_angle(self.angles[i], self.playtimes[i], self.colors[i])
                    #rospy.loginfo("%s -> angle: %s color: %s" % (i, self.angles[i], self.colors[i]))
                    self.previous_angles[i] = self.angles[i]
                    self.previous_colors[i] = self.colors[i]

                # add motor to status array
                msg = MotorStatus()
                msg.motor = i
                msg.angle = s.get_servo_angle()
                msg.temperature = s.get_servo_temperature()
                motors_msg.motors.append(msg)
            

            # publish motor status array
            self.pub.publish(motors_msg)

            rate.sleep()


    def cleanup(self):
        rospy.loginfo("idmind_herkulex: cleaning up")
        rospy.sleep(1.0)
        for s in self.servos:
            s.torque_off()
            s.set_led(Colour.BLACK)
        herkulex.close()


if __name__ == '__main__':
    rospy.init_node("idmind_herkulex")
    node = Node()
    rospy.on_shutdown(node.cleanup)
    rospy.loginfo("idmind_herkulex: running")
    node.run()
    rospy.spin()
