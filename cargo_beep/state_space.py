import rclpy
from rclpy.node import Node
import signal, sys
import numpy as np
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Bool
from cargo_beep.pid_helper import *  # Keeping your helper functions

class StateSpaceControllerNode(Node):
    def __init__(self):
        super().__init__('state_space_controller')

        # Initialize IMU data
        self.imu_data0 = Imu()
        
        # Set time step
        self.dt = GLOBAL_DT
        
        self.motor0_velocity = 0.0
        self.motor1_velocity = 0.0
        self.wheel_radius = .4
        # Desired angle and velocity
        self.desired_angle = 0.0
        self.desired_velocity = 0.0
        self.yaw = 0
        
        # Controller states
        self.controller_state = np.zeros(3)
        
        # State space matrices - extracted from Julia code
        # Dirty derivative time constant
        self.tau_d = 0.01
        
        # Controller gains
        self.K1 = .021658  # Angle controller gain
        self.K2 = 0.15    # Velocity controller gain 
        self.K3 = 7.128922   # Pre-compensator gain
        
        # Initialize the state space controller matrices
        self.initialize_controller()
        
        # Set up signal handling for graceful shutdown
        signal.signal(signal.SIGINT, self.shutdown_cb)
        
        # SUBSCRIBERS
        
        # IMU data subscription
        self.imu0_data_sub = self.create_subscription(
            Imu,
            "imu0/data",
            self.imu0_data_cb,
            10
        )
        
        # Setpoints subscription
        self.setpoints_sub = self.create_subscription(
            Setpoints,
            "setpoints",
            self.setpoints_cb,
            10
        )

        self.dev0_data_sub = self.create_subscription(
            MotorData,
            "/dev0/motor_data",
            self.dev0_data_cb,
            10
        )

        self.dev1_data_sub = self.create_subscription(
            MotorData,
            "/dev1/motor_data",
            self.dev1_data_cb,
            10
        )
        
        
        # PUBLISHERS
        
        # Main duty cycle output
        self.lean_duty_pub = self.create_publisher(
            DutyPair,
            "/pid/lean",
            10
        )
        
        # Shutdown publisher
        self.shutdown_pub = self.create_publisher(
            Bool,
            "shutdown",
            10
        )
        
        # Diagnostic publishers
        self.lean_angle_pub = self.create_publisher(
            Float32,
            'output/lean_angle',
            10
        )

        self.velocity_pub = self.create_publisher(
            Float32,
            'output/velocity',
            10
        )
        
        self.tuning_pub = self.create_publisher(
            TuningValues,
            "output/lean_tuning_values",
            10
        )
        
        # Main control loop timer
        self.timer = self.create_timer(self.dt, self.timer_cb)
        
        print("State Space Controller Initialized")
    
    def initialize_controller(self):
        """Initialize state space controller matrices"""
        
        # Controller A matrix
        self.Ac = np.array([
            [-100.0, 0.0, 0.0],
            [1.0, -2.722, 0.0],
            [0.0, 0.0, -26.802867]
        ])
        
        # Controller B matrix
        self.Bc = np.array([
            [100.0, 0.0],
            [0.0, 0.0],
            [0.0, 26.802867]
        ])
        
        # Controller C matrix
        self.Cc = np.array([[-0.176162, -0.15, 4.721658]])
        
        # Controller D matrix
        self.Dc = np.array([[0.15, 4.721658]])
    
    def imu0_data_cb(self, msg):
        """IMU data callback"""
        self.imu_data0 = msg
    
    def setpoints_cb(self, msg):
        """Handle setpoints messages"""
        self.mode = msg.mode
        self.desired_velocity = msg.velocity

    def dev0_data_cb(self, msg):
        self.motor0_velocity = msg.velocity

    def dev1_data_cb(self, msg):
        self.motor1_velocity = -msg.velocity
    
    def timer_cb(self):
        """Main control loop following the full textbook approach"""
        # Get current rotation angle from IMU
        rotation_axis = imu_axes['y']
        
        # Convert quaternion to euler angles
        euler_rot = euler_from_quat(self.imu_data0.orientation)
        
        # Current lean angle (with offset correction)
        current_angle = euler_rot[rotation_axis] - IMU_ANGLE_ERROR
        
        # Calculate velocity from wheel encoders if available
        # This is a critical measurement for the controller to work well
        current_velocity = (self.motor0_velocity + self.motor1_velocity)/2 * self.wheel_radius
        
        # In the textbook approach, the desired_angle is not explicitly calculated
        # Instead, the velocity error directly feeds into the state-space controller
        
        # Form input to the controller
        # First input: velocity error (desired - actual)
        # Second input: current angle (for the inner loop)
        controller_input = np.array([self.desired_velocity - current_velocity, current_angle * math.pi /180])
        
        # Update controller state (using Euler integration)
        # This internally handles both the velocity and angle control loops
        state_derivative = self.Ac @ self.controller_state + self.Bc @ controller_input
        self.controller_state += state_derivative * self.dt
        
        # Calculate controller output (torque)
        output = self.Cc @ self.controller_state + self.Dc @ controller_input
        output = output[0]  # Extract the scalar value
        
        # Define clearance zone (deadband)
        clearance = 0
        
        # Check if we're close to balanced (near zero angle) and velocity is small
        if (abs(current_angle) < clearance and abs(current_velocity) < 0.05):
            duty = float(0)
        else:
            # Convert controller output to duty cycle
            duty = output_to_duty_power(output)
        
        # Publish duty cycle command
        duty_msg = DutyPair()
        duty_msg.dev0 = duty
        duty_msg.dev1 = -duty
        self.lean_duty_pub.publish(duty_msg)
        
        # Publish lean angle for diagnostics
        lean_angle_msg = Float32()
        lean_angle_msg.data = current_angle
        self.lean_angle_pub.publish(lean_angle_msg)

        current_vel_msg = Float32()
        current_vel_msg.data = current_velocity
        self.velocity_pub.publish(current_vel_msg)
        
        # For diagnostics - calculate the implied desired angle from state
        # This approximates what the velocity controller is requesting
        implied_desired_angle = self.controller_state[1] * (-self.K2/self.K1)
        
        # Publish tuning values for diagnostics
        tune_msg = TuningValues()
        tune_msg.kp = current_angle * self.K1  # Proportional component
        tune_msg.ki = self.controller_state[2] * self.K3  # Integral-like component
        tune_msg.kd = self.controller_state[0] * self.K2  # Derivative-like component
        self.tuning_pub.publish(tune_msg)
    
    def shutdown_cb(self, signum, frame):
        """Handle shutdown signal"""
        shutdown_msg = Bool()
        shutdown_msg.data = True
        self.shutdown_pub.publish(shutdown_msg)
        sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    
    controller_node = StateSpaceControllerNode()
    
    rclpy.spin(controller_node)
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()