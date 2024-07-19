import rclpy
import timeit
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from beep_interfaces.msg import TuningValues, Setpoints, MotorData

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
        self.motor0_data = MotorData()
        self.motor1_data = MotorData()
        self.setpoints = Setpoints()
        self.driving = TuningValues()
        self.setpoints.lean_angle = 0.0

        self.balance_error_prior = 0
        self.balance_integral = 0
        self.balance_bias = 0

        self.balance = TuningValues()
        self.balance.kp = .0115 # BEST .009
        self.balance.ki = .0 #BEST .002
        self.balance.kd = .00005 #BEST .000005 

        self.left_velocity_error_prior = 0
        self.left_integral = 0
        
        self.right_velocity_error_prior = 0
        self.right_integral = 0

        self.velocity = TuningValues()
        self.velocity.kp = 0.005
        self.velocity.ki = 0.000 # BEST .005
        self.velocity.kd = 0.0
        
        self.desired_angle = INITIAL_ANGLE
        self.desired_left_velocity = 0.0
        self.desired_right_velocity = 0.0
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

        self.motor0_data_sub = self.create_subscription(
            MotorData,
            "dev0/motor_data",
            self.motor0_data_cb,
            10
        )

        self.motor0_data_sub = self.create_subscription(
            MotorData,
            "dev1/motor_data",
            self.motor1_data_cb,
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

    def motor0_data_cb(self, msg):
        self.motor0_data = msg
        #print(f'Motor0 Velocity: {self.motor0_data.velocity}')

    def motor1_data_cb(self, msg):
        self.motor1_data = msg
        #print(f'Motor1 Velocity: {self.motor1_data.velocity}')

    def setpoint_cb(self, msg):
        self.setpoints.left_velocity = msg.left_velocity
        self.setpoints.right_velocity = msg.right_velocity
        self.setpoints.lean_angle = msg.lean_angle

    def timer_cb(self):
        # Get current rotation angle
        rotation_axis = imu_axes['x']
        euler_rot = euler_from_quat(self.imu_data0.orientation)
        
        # Balance PID 
        self.desired_angle = self.setpoints.lean_angle
        balance_error = self.desired_angle - euler_rot[rotation_axis]
        
        balance_clearance = .01
        velocity_clearance = .01
        
        if (-balance_clearance < balance_error and balance_error < balance_clearance):
            self.balance_integral = 0
        else:
            self.balance_integral += balance_error * self.dt
        
        balance_derivative = (balance_error - self.balance_error_prior) / self.dt
        balance_output = - (self.balance.kp * balance_error + self.balance.ki * self.balance_integral + self.balance.kd * balance_derivative + self.balance_bias)
        self.balance_error_prior = balance_error

        #Motor1 PID
        self.desired_left_velocity = self.setpoints.left_velocity
        left_velocity_error = self.desired_left_velocity - self.motor1_data.velocity

        if (-velocity_clearance < left_velocity_error and left_velocity_error < velocity_clearance):
            self.left_velocity_integral = 0
        else:
            self.left_velocity_integral += left_velocity_error * self.dt

        left_velocity_derivative = (left_velocity_error - self.left_velocity_error_prior) / self.dt
        left_output = self.velocity.kp * left_velocity_error + self.velocity.ki * self.left_velocity_integral + self.velocity.kd * left_velocity_derivative

        #Motor0 PID
        self.desired_right_velocity = self.setpoints.right_velocity
        right_velocity_error = self.desired_right_velocity - self.motor0_data.velocity
        
        if (-velocity_clearance < right_velocity_error and right_velocity_error < velocity_clearance):
            self.right_velocity_integral = 0
        else:
            self.right_velocity_integral += right_velocity_error * self.dt

        right_velocity_derivative = (right_velocity_error - self.right_velocity_error_prior) / self.dt
        right_output = self.velocity.kp * right_velocity_error + self.velocity.ki * self.right_velocity_integral + self.velocity.kd * right_velocity_derivative
        
        print(f'Right Velcoity Error: {right_velocity_error}')
        print(f'Left Velcoity Error: {left_velocity_error}')
        print(f'Balance: {balance_output}\tLeft: {left_output}\tRight: {right_output}')

        left_duty = output_to_duty_power(balance_output + left_output)
        right_duty = -output_to_duty_power(balance_output - right_output)
        
        print(f'Left Duty: {left_duty}, Right Duty: {right_duty}\n\n')
        
        #print(f'duty: {duty}')
        #print(f'rotation: {euler_rot}')
        #print('\n\n\n\n\n\n\n\n\n\n\n\n\n')

        duty_msg0 = Float32()
        duty_msg0.data = right_duty
        self.motor0_duty_pub.publish(duty_msg0)

        duty_msg1 = Float32()
        duty_msg1.data = left_duty
        self.motor1_duty_pub.publish(duty_msg1)
        
        lean_angle_msg = Float32()
        lean_angle_msg.data = euler_rot[rotation_axis]
        self.lean_angle_pub.publish(lean_angle_msg)

        tune_msg = TuningValues()
        tune_msg.kp = balance_error * self.balance.kp
        tune_msg.ki = self.balance_integral * self.balance.ki
        tune_msg.kd = balance_derivative * self.balance.kd
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