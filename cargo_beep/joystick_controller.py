import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from sensor_msgs.msg._joy import Joy
from beep_interfaces.msg import Setpoints, MotorData

import sys, select, termios, tty
import signal
import numpy as np

#Button Mapping Indices
A_BUTTON = 0
B_BUTTON = 1
X_BUTTON = 2
Y_BUTTON = 3 
LEFT_BUMPER = 4
RIGHT_BUMPER = 5

LEFT_OPTION = 6
RIGHT_OPTION = 7
LEFT_STICK = 9
RIGHT_STICK = 10


#Axes Mapping Indices
LEFT_STICK_YAW = 0
LEFT_STICK_PITCH = 1

RIGHT_STICK_YAW = 3
RIGHT_STICK_PITCH = 4

LEFT_TRIGGER = 2
RIGHT_TRIGGER = 5

D_PAD_YAW = 6
D_PAD_PITCH = 7

MAX_VELOCITY = .50

VELOCITY_SCALE = 5.0
LEAN_SCALE = 20
YAW_SCALE = 1
YAW_INCREMENT_SCALE = .35

# Mode constants
NUM_MODES = 2

LEAN_MODE = 1
VELOCITY_MODE = 2

def normalize_trigger(val):
    return (-val + 1) / 2

def joy_to_setpoint (joy):
    """Normalize the triggers"""
    # triggers are 1 at rest, -1 at full
    forward  = normalize_trigger(joy.axes[RIGHT_TRIGGER]) * VELOCITY_SCALE
    backward = normalize_trigger(joy.axes[LEFT_TRIGGER]) * VELOCITY_SCALE
    velocity = forward - backward


    lean = joy.axes[LEFT_STICK_PITCH] * LEAN_SCALE
    yaw = joy.axes[RIGHT_STICK_YAW] * YAW_SCALE
    velocity = velocity_cap(velocity)

    return velocity, lean, yaw

def velocity_cap (velocity):
    return min(MAX_VELOCITY, max(velocity, -MAX_VELOCITY))

class JoystickControllerNode(Node):
    def __init__ (self):
        super().__init__("joystick_control")
        
        # SETPOINTS
        self.velocity = 0.0
        self.lean_angle = 0.0
        self.yaw = 0.0
        self.desired_yaw = 0

        self.true_yaw = 0
        self.yaw_control = False

        self.joystick = Joy()
        self.left_trigger = 0.0
        self.right_trigger = 0.0
        
        self.mode_toggle = True
        self.mode = LEAN_MODE

        self.prior_setpoints = []
    
        self.shutdown_pub = self.create_publisher(
             Bool,
             "shutdown",
             10
        )

        self.joystick_sub = self.create_subscription(
            Joy,
            "joy",
            self.joystick_cb,
            10
        )

        self.turning_angle_sub = self.create_subscription(
            Float32,
            "/output/turn_angle",
            self.turning_angle_cb,
            10
        )

        self.setpoint_pub = self.create_publisher(
            Setpoints,
            "setpoints",
            10
        )

        self.timer = self.create_timer(.01, self.timer_cb)

    def change_mode(self):
        self.mode += 1;
        if (self.mode > NUM_MODES):
            self.mode = 1

    def turning_angle_cb(self, msg):
        self.true_yaw = msg.data

    def joystick_cb(self, msg):
        velocity, lean, yaw = joy_to_setpoint(msg)

        self.desired_yaw += YAW_INCREMENT_SCALE * yaw
        self.yaw_control = yaw != 0
        if (self.desired_yaw < 0):
            self.desired_yaw = 359
        if (self.desired_yaw > 360):
            self.desired_yaw = 0
        
        if (self.yaw_control == False):
            self.desired_yaw = self.true_yaw

        mode_switch = msg.buttons[A_BUTTON]
        if (mode_switch == 1 and self.mode_toggle):
            print("press")
            self.change_mode()
            self.mode_toggle = False
        if (mode_switch == 0 and not self.mode_toggle):
            print("unpress")
            self.mode_toggle = True

        #if x is presesd, shutdown the robot
        shutdown = msg.buttons[X_BUTTON]
        if (shutdown):
            shutdown_msg = Bool()
            shutdown_msg.data = True
            self.shutdown_pub.publish(shutdown_msg)

        # smoothing
        self.prior_setpoints.append([velocity, lean, yaw])
        if len(self.prior_setpoints) > 3:
            self.prior_setpoints = self.prior_setpoints[-3:]
        smoothed_setpoints = np.mean(self.prior_setpoints, axis=0)

        # right = velocity, left = velocity, lean_angle = lean, yaw = yaw
        self.velocity = smoothed_setpoints[0]
        self.lean_angle = smoothed_setpoints[1]
        self.yaw = yaw

        # if(msg.buttons[LEFT_TRIGGER]):
        #     self.lean_angle = 15.0
        # elif(msg.buttons[RIGHT_TRIGGER]):
        #     self.lean_angle = -15.0
    
    def timer_cb (self):
        
        setpoints = Setpoints()


        setpoints.velocity = self.velocity
        setpoints.lean_angle = self.lean_angle
        setpoints.yaw = float(self.desired_yaw)
        setpoints.mode = self.mode
        self.setpoint_pub.publish(setpoints)


def main(args=None):
    rclpy.init(args=args)

    node = JoystickControllerNode()
    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()

    
