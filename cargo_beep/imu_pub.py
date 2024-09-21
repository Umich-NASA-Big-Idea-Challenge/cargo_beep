# ROS Imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np

# IMU imports
import board
import busio
from adafruit_bno08x import (
    BNO_REPORT_LINEAR_ACCELERATION,
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_ROTATION_VECTOR
)
from adafruit_bno08x.i2c import BNO08X_I2C

    
class ImuNode(Node):
    def __init__(self):
        super().__init__("imu_node")

        self.i2c = busio.I2C(board.SCL, board.SDA)

        self.bno0 = BNO08X_I2C(self.i2c, address=0x4a)
        #self.bno1 = BNO08X_I2C(self.i2c, address=0x4b)

        self.bno0.enable_feature(BNO_REPORT_LINEAR_ACCELERATION)
        self.bno0.enable_feature(BNO_REPORT_GYROSCOPE)
        self.bno0.enable_feature(BNO_REPORT_ROTATION_VECTOR)

        self.prior_orientations = []

        self.bno0_pub = self.create_publisher(
            Imu,
            "imu0/data",
            10
        )

        self.bno1_pub = self.create_publisher(
            Imu,
            "imu1/data",
            10
        )


        print("Collecting IMU Information")
        self.timer = self.create_timer(.002, self.timer_cb)

    def smoothing(self, current_orientation):
        self.prior_orientations.append(current_orientation)
        if len(self.prior_orientations) > 3:
            self.prior_orientations = self.prior_orientations[-3:]
        avg_orientation = np.mean(self.prior_orientations, axis=0)
        
        return avg_orientation

    def create_message(self, bno):
        imu_msg = Imu()

        imu_msg.linear_acceleration.x, \
        imu_msg.linear_acceleration.y, \
        imu_msg.linear_acceleration.z = bno.linear_acceleration

        imu_msg.angular_velocity.x, \
        imu_msg.angular_velocity.y, \
        imu_msg.angular_velocity.z = bno.gyro
        
        smoothed_data = self.smoothing(bno.quaternion)
        imu_msg.orientation.x, \
        imu_msg.orientation.y, \
        imu_msg.orientation.z = smoothed_data[0:3]
        imu_msg.orientation.w = float(smoothed_data[3])

        return imu_msg

    def timer_cb(self):
        imu_msg_0 = self.create_message(self.bno0)
        self.bno0_pub.publish(imu_msg_0)

        #imu_msg_1 = self.create_message(self.bno1)
        #self.bno1_pub.publish(imu_msg_1)




def main(args=None):
    rclpy.init(args=args)

    imu_node = ImuNode()
    
    rclpy.spin(imu_node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
