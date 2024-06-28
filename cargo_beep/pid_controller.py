import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

from scipy.spatial.transform import Rotation

imu_axes = {'x': 1, 'y': 0, 'z': 2}

# takes in a Quaternian msg and returns a 3 tuple (x, y, z)
def euler_from_quat (quat):
    rot = Rotation.from_quat((quat.x, quat.y, quat.z, quat.w))
    rot_euler = rot.as_euler("xyz", degrees=True)
    return rot_euler

def output_to_duty_power (angle):
    MAX_DUTY = .2
    MIN_DUTY = -.2
    return min(MAX_DUTY, max(angle, MIN_DUTY))

class PIDControllerNode(Node):

    def __init__(self):
        super().__init__('motor_control')

        self.imu_data = Imu()

        self.error_prior = 0
        self.integral_prior = 0
        self.kp = 1
        self.ki = 0
        self.kd = 0
        self.bias = 0

        self.desired_angle = 0
        self.dt = .01
        
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

        self.motor1_duty_pub = self.create_publisher(
            Float32,
            "dev1/duty",
            10
        )


        self.timer = self.create_timer(self.dt, self.timer_cb)

        print("Controlling Motors")

    def imu_data_cb(self, msg):
        self.imu_data = msg

    
    def timer_cb(self):
        # Get current rotation angle
        rotation_axis = imu_axes['x']
        euler_rot = euler_from_quat(self.imu_data.orientation)

        error = self.desired_angle - euler_rot[rotation_axis]

        integral = integral_prior + error * self.dt
        derivative = (error - error_prior) / self.dt
        output = self.kp*error + self.ki*integral + self.kd*derivative + bias
        error_prior = error
        integral_prior = integral

        duty = output_to_duty_power(output)
        duty_msg0 = Float32()
        duty_msg0.data = -duty

        duty_msg1 = Float32()
        duty_msg1.data = duty

        self.motor0_duty_pub.publish(duty_msg0)
        self.motor1_duty_pub.publish(duty_msg1)
        



def main(args=None):
    rclpy.init(args=args)

    controller_node = PIDControllerNode()

    rclpy.spin(controller_node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()