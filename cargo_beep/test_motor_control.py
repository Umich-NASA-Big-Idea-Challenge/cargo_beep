import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

from scipy.spatial.transform import Rotation

imu_axes = {'x': 1, 'y': 0, 'z': 2}

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

        self.motor0_output_sub = self.create_subscription(
            Float32MultiArray,
            "dev0/output",
            self.motor0_output_cb,
            10
        )

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


        self.timer = self.create_timer(.01, self.timer_cb)

        print("Controlling Motors")

    def imu_data_cb(self, msg):
        self.imu_data = msg

    def motor0_output_cb(self, msg):
        print(msg.data[1])
        print(msg.data[4])

    
    def timer_cb(self):
        rotation_axis = imu_axes['x']
        euler_rot = euler_from_quat(self.imu_data.orientation)
        duty = lean_angle_to_duty_power(euler_rot[rotation_axis])

        duty_msg0 = Float32()
        duty_msg0.data = -duty

        duty_msg1 = Float32()
        duty_msg1.data = duty

        self.motor0_duty_pub.publish(duty_msg0)
        self.motor1_duty_pub.publish(duty_msg1)
        



def main(args=None):
    rclpy.init(args=args)

    controller_node = MotorControllerNode()

    rclpy.spin(controller_node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()