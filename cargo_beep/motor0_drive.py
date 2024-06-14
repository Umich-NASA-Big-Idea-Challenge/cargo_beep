import rclpy
from .motor_node import MotorNode

from TMotorCANControl.servo_serial import *


def main(args=None):
    rclpy.init(args=args)

    with TMotorManager_servo_serial(port = "/dev/ttyUSB0") as dev0:
        motor0_node = MotorNode(device = dev0)
        rclpy.spin(motor0_node)

    rclpy.shutdown()
    
if __name__ == "__main__":
    main()