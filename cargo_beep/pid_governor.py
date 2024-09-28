import rclpy
import signal, sys

from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from beep_interfaces.msg import DutyPair

from cargo_beep.pid_helper import *

class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('motor_control')
        
        self.turn_dev0 = 0
        self.lean_dev0 = 0

        self.turn_dev1 = 0
        self.lean_dev1 = 0

        self.lean_gain = 1
        self.turn_gain = 1

        self.dt = GLOBAL_DT

        signal.signal(signal.SIGINT, self.shutdown_cb)

        # create publishers to output duties

        self.motor0_duty_pub = self.create_publisher(
            Float32,
            "dev0/duty",
            10
        )

        self.motor1_duty_pub = self.create_publisher(
            Float32,
            "dev1/duty",
            10
        )

        # subscribe to turning and lean angle pids
        self.turn_sub = self.create_subscription(
            DutyPair,
            "/pid/turn",
            self.turn_cb,
            10
        )

        self.lean_sub = self.create_subscription(
            DutyPair,
            "/pid/lean",
            self.lean_cb,
            10
        )

        self.shutdown_pub = self.create_publisher(
            Bool,
            "shutdown",
            10
        )

        self.timer = self.create_timer(self.dt, self.timer_cb)

    def turn_cb(self, msg):
        self.turn_dev0 = msg.dev0
        self.turn_dev1 = msg.dev1

    def lean_cb(self, msg):
        self.lean_dev0 = msg.dev0
        self.lean_dev1 = msg.dev1

    def timer_cb(self):
        duty0 = float(self.turn_gain * self.turn_dev0 + self.lean_gain * self.lean_dev0)
        duty1 = float(self.turn_gain * self.turn_dev1 + self.lean_gain * self.lean_dev1)
        
        duty_msg0 = Float32()
        duty_msg0.data = duty0
        self.motor0_duty_pub.publish(duty_msg0)

        duty_msg1 = Float32()
        duty_msg1.data = duty1
        self.motor1_duty_pub.publish(duty_msg1)
     

    def shutdown_cb (self, signum, frame):
        shutdown_msg = Bool()
        shutdown_msg.data = True
        self.shutdown_pub.publish(shutdown_msg)
        sys.exit(0)


def main(args=None):
    rclpy.init(args=args)

    controller_node = PIDControllerNode()

    rclpy.spin(controller_node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()