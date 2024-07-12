import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from beep_interfaces.msg import TuningValues, Setpoints

from scipy.spatial.transform import Rotation
import signal, sys

imu_axes = {'x': 1, 'y': 0, 'z': 2}

INITIAL_ANGLE = .143

# takes in a Quaternian msg and returns a 3 tuple (x, y, z)
def euler_from_quat (quat):
    rot = Rotation.from_quat((quat.x, quat.y, quat.z, quat.w))
    rot_euler = rot.as_euler("xyz", degrees=True)
    return rot_euler

MAX_DUTY = .15
def output_to_duty_power (output):
    duty = min(MAX_DUTY, max(output, -MAX_DUTY))
    return float(duty)

class PIDControllerNode(Node):

    def __init__(self):
        super().__init__('motor_control')

        self.imu_data0 = Imu()
        self.imu_data1 = Imu()
        self.setpoints = Setpoints()

        self.error_prior = 0
        self.integral_prior = 0
        self.kp = .009 # BEST .0055
        self.ki = 0 # BEST .001
        self.kd = .000005 #.000075 BEST .0001
        self.bias = 0

        self.desired_angle = INITIAL_ANGLE
        self.dt = .002

        signal.signal(signal.SIGINT, self.shutdown_cb)
        
        # CRITICAL CONTROLS
        self.imu0_data_sub = self.create_subscription(
            Imu,
            "imu0/data",
            self.imu0_data_cb,
            10
        )

        self.imu1_data_sub = self.create_subscription(
            Imu,
            "imu1/data",
            self.imu1_data_cb,
            10
        )

        self.setpoint_sub = self.create_subscription(
            Setpoints,
            "setpoints",
            self.setpoint_cb,
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

        self.shutdown_pub = self.create_publisher(
            Bool,
            "shutdown",
            10
        )

        # OUTPUT FOR GRAPHING

        self.lean_angle_pub = self.create_publisher(
            Float32,
            'output/lean_angle',
            10
        )

        # TUNING HELPERS

        self.tuning_pub = self.create_publisher(
            TuningValues,
            "output/tuning_values",
            10
        )


        self.timer = self.create_timer(self.dt, self.timer_cb)

        print("Controlling Motors")

    def imu0_data_cb(self, msg):
        self.imu_data0 = msg

    def imu1_data_cb(self, msg):
        self.imu_data1 = msg

    def setpoint_cb(self, msg):
        self.setpoints.left_velocity = msg.left_velocity
        self.setpoints.right_velocity = msg.right_velocity
        self.setpoints.lean_angle = msg.lean_angle

    def timer_cb(self):
        # Get current rotation angle
        rotation_axis = imu_axes['x']
        euler_rot = euler_from_quat(self.imu_data0.orientation)
        
        # print(f'Lean Angle: {self.setpoints.lean_angle}')
        self.desired_angle = self.setpoints.lean_angle
        error = self.desired_angle - euler_rot[rotation_axis]
        
        clearence = 0

        
    
        integral = self.integral_prior + error * self.dt
        derivative = (error - self.error_prior) / self.dt
        output = self.kp*error + self.ki*integral + self.kd*derivative + self.bias
        self.error_prior = error
        self.integral_prior = integral

        if (-clearence < error and error < clearence):
            duty = float(0)
        else:
            duty = output_to_duty_power(output)
        

        #print(f'duty: {duty}')
        #print(f'rotation: {euler_rot}')
        #print('\n\n\n\n\n\n\n\n\n\n\n\n\n')

        duty_msg0 = Float32()
        duty_msg0.data = -duty
        self.motor0_duty_pub.publish(duty_msg0)

        duty_msg1 = Float32()
        duty_msg1.data = duty
        self.motor1_duty_pub.publish(duty_msg1)
        
        lean_angle_msg = Float32()
        lean_angle_msg.data = euler_rot[rotation_axis]
        self.lean_angle_pub.publish(lean_angle_msg)

        tune_msg = TuningValues()
        tune_msg.kp = error * self.kp
        tune_msg.ki = integral * self.ki
        tune_msg.kd = derivative * self.kd
        self.tuning_pub.publish(tune_msg)

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