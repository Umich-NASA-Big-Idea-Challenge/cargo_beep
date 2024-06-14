import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

from scipy.spatial.transform import Rotation

# takes in a Quaternian msg and returns a 3 tuple (x, y, z)
def euler_from_quat (quat):
    rot = Rotation.from_quat((quat.x, quat.y, quat.z, quat.w))
    rot_euler = rot.as_euler("xyz", degrees=True)
    return rot_euler

def lean_angle_to_duty_power (angle):
    MAX_DUTY = .3
    MIN_DUTY = -.3
    return min(MAX_DUTY, max(angle * .01, MIN_DUTY))

class MotorControllerNode(Node):

    def __init__(self):
        super().__init__('motor_control')

        self.imu_data = Imu()

        self.imu_data_sub = self.create_subscription(
            Imu,
            "imu0/data",
            self.imu_data_cb,
            10
        )

        self.motor0_duty_pub = self.create_publisher(
            Float32,
            "dev0/duty",
            10
        )


        self.timer = self.create_timer(.01, self.timer_cb)

        print("Controlling Motors")

    def imu_data_cb(self, msg):
        self.imu_data = msg

    
    def timer_cb(self):
        euler_rot = euler_from_quat(self.imu_data.orientation)
        duty = lean_angle_to_duty_power(euler_rot[0])

        duty_msg = Float32()
        duty_msg.data = duty
        self.motor0_duty_pub.publish(duty_msg)
        



def main(args=None):
    rclpy.init(args=args)

    controller_node = MotorControllerNode()

    rclpy.spin(controller_node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()