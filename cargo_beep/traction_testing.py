import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool

import numpy as np
from scipy.ndimage import gaussian_filter1d
import signal, sys

WHEEL_RADIUS = .78 # m (very rough measurement of pool floaties)

# Velocity profile
velocity_max = 3 # m/s
acceleration = .5 # m/s^2
zero_duration = 3 # s
steady_duration = 10 # s

dt = 1/100 # s
samples_per_second = int (1/dt)

zero_profile = np.zeros(zero_duration * samples_per_second)
run_up_profile = acceleration * np.linspace(0, velocity_max / acceleration, int((samples_per_second * velocity_max) / acceleration)) 
steady_profile = velocity_max * np.ones(steady_duration * samples_per_second)
run_down_profile = acceleration * np.linspace(velocity_max / acceleration, 0 , int((samples_per_second * velocity_max) / acceleration)) 
 
linear_velocity_profile = np.concatenate((zero_profile, run_up_profile, steady_profile, run_down_profile, zero_profile), axis=0)
linear_velocity_profile = gaussian_filter1d(linear_velocity_profile, 50)
angular_velocity_profile = linear_velocity_profile / WHEEL_RADIUS

class TractionTesting(Node):

    def __init__ (self):
        super().__init__('traction_testing')

        self.motor_velocity = 0
        self.angular_velocity_profile = angular_velocity_profile
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
        
        if (self.idx < angular_velocity_profile.shape[0]):
            self.motor_velocity = self.angular_velocity_profile[self.idx]
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