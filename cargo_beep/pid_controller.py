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
        self.kp = 0
        self.ki = 0
        self.kd = 0
        self.bias = 0

        self.desired_angle = 0
        self.dt = .01
        
        # CRITICAL CONTROLS
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

        # OUTPUT FOR GRAPHING

        self.lean_angle_pub = self.create_publisher(
            Float32,
            'output/lean_angle',
            10
        )

        # TUNING HELPERS

        self.tune_kp_sub = self.create_subscription(
            Float32,
            "tune/kp",
            self.tune_kp_cb,
            10
        )

        self.tune_ki_sub = self.create_subscription(
            Float32,
            "tune/ki",
            self.tune_ki_cb,
            10
        )

        self.tune_kd_sub = self.create_subscription(
            Float32,
            "tune/kd",
            self.tune_kd_cb,
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

        integral = self.integral_prior + error * self.dt
        derivative = (error - self.error_prior) / self.dt
        output = self.kp*error + self.ki*integral + self.kd*derivative + self.bias
        self.error_prior = error
        self.integral_prior = integral

        duty = output_to_duty_power(output)
        duty_msg0 = Float32()
        duty_msg0.data = -duty
        self.motor0_duty_pub.publish(duty_msg0)

        duty_msg1 = Float32()
        duty_msg1.data = duty
        self.motor1_duty_pub.publish(duty_msg1)
        
        lean_angle_msg = Float32()
        lean_angle_msg.data = error
        self.lean_angle_pub.publish(lean_angle_msg)

    # TUNING HELPER CALLBACKS

    def tune_kp_cb (self, msg) :
        self.kp = msg.data

    def tune_ki_cb (self, msg) :
        self.ki = msg.data

    def tune_kd_cb (self, msg) :
        self.kd = msg.data
        



def main(args=None):
    rclpy.init(args=args)

    controller_node = PIDControllerNode()

    rclpy.spin(controller_node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()