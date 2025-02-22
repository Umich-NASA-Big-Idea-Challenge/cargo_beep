import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Bool, Float32
from beep_interfaces.msg import Setpoints

import sys, select, termios, tty
import signal

settings = termios.tcgetattr(sys.stdin)

MAX_VELOCITY = 3

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

def velocity_cap (velocity):
     return min(MAX_VELOCITY, max(velocity, -MAX_VELOCITY))

class KeyboardControllerNode(Node):
    def __init__ (self):
        super().__init__("keyboard_control")
        
        # SETPOINTS
        self.velocity = 0.0
        self.lean_angle = 0.0
        self.yaw = 0.0

        self.true_yaw = 0
        self.yaw_control = False

        self.left_trigger = 0.0
        self.right_trigger = 0.0
        
        self.mode_toggle = True
        self.mode = 1 # LEAN_MODE

        self.prior_setpoints = []

        signal.signal(signal.SIGINT, self.shutdown_cb)
        
        self.setpoint_pub = self.create_publisher(
            Setpoints,
            "setpoints",
            10
        )
    
        self.shutdown_pub = self.create_publisher(
             Bool,
             "shutdown",
             10
        )

        self.turning_angle_sub = self.create_subscription(
            Float32,
            "/output/turn_angle",
            self.turning_angle_cb,
            10
        )

        self.timer = self.create_timer(.01, self.timer_cb)

    def turning_angle_cb(self, msg):
        self.true_yaw = msg.data

    def timer_cb (self):
        setpoints = Setpoints()

        setpoints.velocity = float(0)
        setpoints.lean_angle = float(0)
        setpoints.yaw = self.yaw
        setpoints.mode = self.mode
        key = getKey()
        print("printing:", key)

        if (key == 'w'):
            setpoints.lean_angle = -15.0
            self.lean_angle = setpoints.lean_angle
        if (key == 's'):
            setpoints.lean_angle = 15.0
            self.lean_angle = setpoints.lean_angle
        if (key == 'a'):
            setpoints.yaw += .3
            self.yaw = setpoints.yaw
        if (key == 'd'):
            setpoints.yaw -= .3
            self.yaw = setpoints.yaw
        if (key == 'x'):
            shutdown_msg = Bool()
            shutdown_msg.data = True
            self.shutdown_pub.publish(shutdown_msg)
            self.shutdown_cb()

        # smoothing
        '''self.prior_setpoints.append([setpoints.velocity, setpoints.lean_angle, setpoints.yaw])
        if len(self.prior_setpoints) > 3:
            self.prior_setpoints = self.prior_setpoints[-3:]
        smoothed_setpoints = np.mean(self.prior_setpoints, axis=0)


        setpoints.velocity = smoothed_setpoints[0]
        setpoints.lean_angle = smoothed_setpoints[1]
        setpoints.mode = self.mode
        
        self.velocity = smoothed_setpoints[0]
        self.lean_angle = smoothed_setpoints[1]
        self.yaw = setpoints.yaw
        '''

        self.setpoint_pub.publish(setpoints)
    
    def shutdown_cb(self):
         shutdown_msg = Bool()
         shutdown_msg.data = True
         self.shutdown_pub.publish(shutdown_msg)
         sys.exit(0)
        

        


def main(args=None):
    rclpy.init(args=args)

    node = KeyboardControllerNode()
    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()

    