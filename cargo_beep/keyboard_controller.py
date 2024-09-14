import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64
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
        
        self.left_velocity = 0
        self.right_velocity = 0
        self.lean_angle = 0.0

        signal.signal(signal.SIGINT, self.shutdown_cb)
        
        # self.velocity0_pub = self.create_publisher(
        #      Float64,
        #      "dev0/velocity",
        #      10
        # )

        # self.velocity1_pub = self.create_publisher(
        #      Float64,
        #      "dev1/velocity",
        #      10
        # )
    
        self.shutdown_pub = self.create_publisher(
             Bool,
             "shutdown",
             10
        )

        self.setpoint_pub = self.create_publisher(
            Setpoints,
            "setpoints",
            10
        )

        self.timer = self.create_timer(.01, self.timer_cb)

    def timer_cb (self):
        key = getKey()
        if (key == 'w'):
            self.left_velocity+=.05
            self.right_velocity+=.05
            self.lean_angle = float(10.0)
        elif (key == 's'):
            self.left_velocity-=.05
            self.right_velocity-=.05
            self.lean_angle = float(-10.0)
        elif (key == 'a'):
            self.left_velocity+=.05
            self.right_velocity-=.05
        elif (key == 'd'):
            self.left_velocity-=.05
            self.right_velocity+=.05
        elif (key == 'x'):
            self.lean_angle = 0.0
        if (key == chr(27)):
            self.shutdown_cb()

        setpoints = Setpoints()
        setpoints.left_velocity = self.left_velocity
        setpoints.right_velocity = self.right_velocity
        # msg = Float64()
        # msg.data = self.lean_angle
        setpoints.lean_angle = self.lean_angle
        self.setpoint_pub.publish(setpoints)
        print(f"angle: {self.lean_angle}\n\n")

        # def timer_cb (self):
            #     key = getKey()
            #     if (key == 'w'):
            #         self.left_velocity+=.05
            #         self.right_velocity+=.05
            #     if (key == 's'):
            #         self.left_velocity-=.05
            #         self.right_velocity-=.05
            #     if (key == 'a'):
            #         self.left_velocity+=.05
            #         self.right_velocity-=.05
            #     if (key == 'd'):
            #         self.left_velocity-=.05
            #         self.right_velocity+=.05
            #     if (key == chr(27)):
            #         self.shutdown_cb()

        #     print(f"left velocity: {self.left_velocity}")
        #     print(f"right velocity: {self.right_velocity}\n\n")
            
        #     msg0 = Float64()
        #     msg0.data = velocity_cap(self.right_velocity)
        #     self.velocity0_pub.publish(msg0)

        #     msg1 = Float64()
        #     msg1.data = velocity_cap(-self.left_velocity)
        #     self.velocity1_pub.publish(msg1)

    
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

    