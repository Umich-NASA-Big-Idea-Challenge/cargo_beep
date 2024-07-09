import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from sensor_msgs.msg._joy import Joy

import sys, select, termios, tty
import signal

settings = termios.tcgetattr(sys.stdin)

MAX_VELOCITY = 3.0
TRIGGER_SCALE = .5

# def getKey():
# 	tty.setraw(sys.stdin.fileno())
# 	select.select([sys.stdin], [], [], 0)
# 	key = sys.stdin.read(1)
# 	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
# 	return key

def velocity_cap (velocity = Float32):
     if velocity > MAX_VELOCITY:
         return MAX_VELOCITY
     return velocity

    #  return min(MAX_VELOCITY, max(velocity, -MAX_VELOCITY))

class JoystickControllerNode(Node):
    def __init__ (self):
        super().__init__("joystick_control")
        
        self.left_velocity = 0.0
        self.right_velocity = 0.0
        self.joystick = Joy()
        self.left_trigger = 0.0
        self.right_trigger = 0.0
        

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

        self.joystick_sub = self.create_subscription(
            Joy,
            "joy",
            self.joystick_cb,
            10
        )
        
        self.timer = self.create_timer(.01, self.timer_cb)

    def joystick_cb(self, msg):
        self.joystick = msg
        self.left_trigger = (-1 * self.joystick.axes[3] + 1) * TRIGGER_SCALE
        self.right_trigger = (-1 * self.joystick.axes[4] + 1) * TRIGGER_SCALE     
        print(f"Left: {self.left_trigger}", f"Right {self.right_trigger}")
    
    def timer_cb (self):
       
        self.left_velocity = self.left_trigger
        self.right_velocity = self.right_trigger
     
        print(f"left velocity: {self.left_velocity}")
        print(f"right velocity: {self.right_velocity}\n\n")
        
        msg0 = Float32()
        msg0.data = velocity_cap(self.right_velocity)
        self.velocity0_pub.publish(msg0)

        msg1 = Float32()
        msg1.data = velocity_cap(-self.left_velocity)
        self.velocity1_pub.publish(msg1)
        

    
    def shutdown_cb(self):
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

    