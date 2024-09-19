import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool

import numpy as np
from scipy.ndimage import gaussian_filter1d
import signal, sys

WHEEL_RADIUS = .36 # m (very rough measurement of pool floaties)

# Velocity profile
max_velocity = 2.2 / WHEEL_RADIUS
dt = .01
acceleration_duration = 5 #s
steady_duration = 20 #s

acceleration_profile = np.linspace(0, max_velocity, int((1/dt) * acceleration_duration))
steady_profile = max_velocity * np.ones(int(steady_duration * (1/dt)))

velocity_profile = np.concatenate((acceleration_profile, steady_profile))
class TractionTesting(Node):

    def __init__ (self):
        super().__init__('traction_testing')

        self.motor_velocity = 0
        self.velocity_profile = velocity_profile
        self.idx = 0

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

        self.timer = self.create_timer(.01, self.timer_cb)

        signal.signal(signal.SIGINT, self.shutdown_cb)
        self.shutdown_pub = self.create_publisher(
            Bool,
            "shutdown",
            10
        )
    
    def timer_cb(self):
        
        if (self.idx < velocity_profile.shape[0]):
            self.motor_velocity = self.velocity_profile[self.idx]
            self.idx+=1
        else:
            self.motor_velocity = 0.0

        dev0_msg = Float32()
        dev0_msg.data = -self.motor_velocity

        dev1_msg = Float32()
        dev1_msg.data = self.motor_velocity

        self.velocity0_pub.publish(dev0_msg)
        self.velocity1_pub.publish(dev1_msg)

    def shutdown_cb (self, signum, frame):
        shutdown_msg = Bool()
        shutdown_msg.data = True
        self.shutdown_pub.publish(shutdown_msg)
        sys.exit(0)

        

def main(args=None):
    rclpy.init(args=args)

    testing_node = TractionTesting()

    rclpy.spin(testing_node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()