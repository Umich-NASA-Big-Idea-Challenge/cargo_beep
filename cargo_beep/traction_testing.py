import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

LINEAR_VELOCITY = 5 #m/s
WHEEL_RADIUS = 1 #m
MOTOR_VELOCITY = LINEAR_VELOCITY / WHEEL_RADIUS

class TractionTesting(Node):
    def __init__ (self):
        self.motor_velocity = 0

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

        self.timer = create_timer(.05, self.timer_cb)
    
    def timer_cb(self):
        if (self.motor_velocity != MOTOR_VELOCITY):
            self.motor_velocity += .1

        velocity0_pub.publish(self.motor_velocity)
        velocity1_pub.publish(-self.motor_velocity)

def main(args=None):
    rclpy.init(args=args)

    testing_node = TractionTesting()

    rclpy.spin(testing_node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()