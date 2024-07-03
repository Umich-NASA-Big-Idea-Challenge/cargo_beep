import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32

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
        
        self.velocity = 0

        signal.signal(signal.SIGINT, self.shutdown_cb)
        
        self.velocity0_pub = self.create_publisher(
             Float32,
             "dev0/velocity",
             10
        )

        self.velocity1_pub = self.create_publisher(
             Float32,
             "dev1/velocity",
             10
        )
    
        self.shutdown_pub = self.create_publisher(
             Bool,
             "shutdown",
             10
        )

        self.timer = self.create_timer(.01, self.timer_cb)

    def timer_cb (self):
        key = getKey()
        if (key == 'w'):
            self.velocity+=.05
        if (key == 'd'):
            self.velocity-=.05
        if (key == chr(27)):
            self.shutdown_cb()
        
        msg0 = Float32()
        msg0.data = velocity_cap(self.velocity)
        self.velocity0_pub.publish(msg0)

        msg1 = Float32()
        msg1.data = velocity_cap(-self.velocity)
        self.velocity1_pub.publish(msg1)
        

    
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

    