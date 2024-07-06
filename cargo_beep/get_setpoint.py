import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from .pid_controller import euler_from_quat

class GetSetpoint(Node):
    def __init__(self):
        super().__init__("get_setpoint")

        self.running_angle = 0
        self.running_cnt = 0

        self.imu_sub = self.create_subscription(
            Imu,
            "imu0/data",
            self.imu_cb,
            10
        )

    def imu_cb (self, msg):
        lean_angle = euler_from_quat(msg.orientation)[1]
        self.running_cnt += 1
        self.running_angle += lean_angle
        print(f'lean angle {lean_angle}')
        print(self.running_angle / self.running_cnt)
        print('\n\n')

def main(args=None):
    rclpy.init(args=args)

    node = GetSetpoint()

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
