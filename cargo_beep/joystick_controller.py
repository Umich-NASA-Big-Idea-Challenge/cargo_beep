import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from sensor_msgs.msg._joy import Joy
from beep_interfaces.msg import Setpoints

import sys, select, termios, tty
import signal

#Button Mapping Indices
A_BUTTON = 1
B_BUTTON = 2
X_BUTTON = 0
Y_BUTTON = 3 
LEFT_BUMPER = 4
RIGHT_BUMPER = 5

LEFT_OPTION = 8
RIGHT_OPTION = 9
LEFT_STICK = 10
RIGHT_STICK = 11
SHARE = 13


#Axes Mapping Indices
LEFT_STICK_YAW = 0
LEFT_STICK_PITCH = 1

RIGHT_STICK_YAW = 2
RIGHT_STICK_PITCH = 5

LEFT_TRIGGER = 3
RIGHT_TRIGGER = 4

D_PAD_YAW = 6
D_PAD_PITCH = 7


settings = termios.tcgetattr(sys.stdin)

MAX_VELOCITY = 10.0

VELOCITY_SCALE = 10.0
LEAN_SCALE = 1.5

def normalize_trigger(val):
    return (-val + 1) / 2

def joy_to_setpoint (joy):
    """Normalize the triggers"""
    # triggers are 1 at rest, -1 at full
    forward  = normalize_trigger(joy.axes[RIGHT_TRIGGER]) * VELOCITY_SCALE
    backward = normalize_trigger(joy.axes[LEFT_TRIGGER]) * VELOCITY_SCALE
    velocity = forward - backward

    lean = joy.axes[LEFT_STICK_PITCH] * LEAN_SCALE

    return velocity, lean

def velocity_cap (velocity):
    return min(MAX_VELOCITY, max(velocity, -MAX_VELOCITY))

class JoystickControllerNode(Node):
    def __init__ (self):
        super().__init__("joystick_control")
        
        # SETPOINTS
        self.left_velocity = 0.0
        self.right_velocity = 0.0
        self.lean_angle = 0.0

        self.joystick = Joy()
        self.left_trigger = 0.0
        self.right_trigger = 0.0
        

        signal.signal(signal.SIGINT, self.shutdown_cb)
    
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

        self.setpoint_pub = self.create_publisher(
            Setpoints,
            "setpoints",
            f"/setpoints",
            10
        )
        
        self.timer = self.create_timer(.01, self.timer_cb)

    def joystick_cb(self, msg):
        velocity, lean = joy_to_setpoint(msg)
        self.right_velocity = velocity
        self.left_velocity = velocity
        self.lean_angle = lean
    
    def timer_cb (self):
        
        setpoints = Setpoints()
        setpoints.left_velocity = self.left_velocity
        setpoints.right_velocity = self.right_velocity
        setpoints.lean_angle = self.lean_angle
        self.setpoint_pub.publish(setpoints)

    
    def shutdown_cb(self, signum, frame):
         shutdown_msg = Bool()
         shutdown_msg.data = True
         self.shutdown_pub.publish(shutdown_msg)
         sys.exit(0)
        

        


def main(args=None):
    rclpy.init(args=args)

    node = JoystickControllerNode()
    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()

    