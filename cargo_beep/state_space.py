import rclpy
from rclpy.node import Node
import signal, sys
import numpy as np
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Bool
from cargo_beep.pid_helper import *  # Keeping your helper functions

MODE_LEAN = 1
MODE_VELOCITY = 2

class StateSpaceControllerNode(Node):
    def __init__(self):
        super().__init__('state_space_controller')

        # Initialize IMU data
        self.imu_data0 = Imu()
        
        # Set time step
        self.dt = GLOBAL_DT
        
        # Controller mode
        self.mode = MODE_LEAN
        
        # Desired angle and velocity
        self.desired_angle = IMU_ANGLE_ERROR
        self.desired_velocity = 0.0
        self.yaw = 0
        
        # Controller states
        self.controller_state = np.zeros(3)
        
        # State space matrices - extracted from Julia code
        # Dirty derivative time constant
        self.tau_d = 0.01
        
        # Controller gains
        self.K1 = -1.041  # Angle controller gain
        self.K2 = 0.15    # Velocity controller gain 
        self.K3 = 0.833   # Pre-compensator gain
        
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
        
        # Goal lean subscription
        self.goal_lean_sub = self.create_subscription(
            Float32,
            "/setpoints/goal_lean",
            self.goallean_cb,
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
            [0.0, 0.0, -13.679]
        ])
        
        # Controller B matrix
        self.Bc = np.array([
            [100.0, 0.0],
            [0.0, 0.0],
            [0.0, 13.679]
        ])
        
        # Controller C matrix
        self.Cc = np.array([[0.0, -0.15, -1.041]])
        
        # Controller D matrix
        self.Dc = np.array([[0.15, -1.041]])
    
    def imu0_data_cb(self, msg):
        """IMU data callback"""
        self.imu_data0 = msg
    
    def setpoints_cb(self, msg):
        """Handle setpoints messages"""
        self.mode = msg.mode
        if(self.mode == MODE_LEAN):
            self.desired_angle = msg.lean_angle + IMU_ANGLE_ERROR
    
    def goallean_cb(self, msg):
        """Handle goal lean messages"""
        if(self.mode == MODE_VELOCITY):
            self.desired_angle = msg.data + IMU_ANGLE_ERROR
    
    def timer_cb(self):
        """Main control loop"""
        # Get current rotation angle from IMU
        rotation_axis = imu_axes['y']
        
        # Convert quaternion to euler angles
        euler_rot = euler_from_quat(self.imu_data0.orientation)
        
        # Current lean angle
        current_angle = euler_rot[rotation_axis]
        
        # Calculate velocity (this is a simplification - improve with wheel encoders if available)
        # In a real implementation, you should use wheel encoders or other sensors for velocity
        # This is just a crude estimate based on angle changes
        # You may already have a better velocity calculation in your system
        current_velocity = 0.0  # Replace with actual velocity measurement if available
        
        # Define clearance zone (deadband)
        clearance = 0.1
        
        # Form input to the controller
        angle_error = self.desired_angle - current_angle
        controller_input = np.array([self.desired_velocity - current_velocity, angle_error])
        
        # Update controller state (using Euler integration)
        state_derivative = self.Ac @ self.controller_state + self.Bc @ controller_input
        self.controller_state += state_derivative * self.dt
        
        # Calculate controller output
        output = self.Cc @ self.controller_state + self.Dc @ controller_input
        output = output[0]  # Extract the scalar value
        
        # Apply deadband if very close to desired angle
        if (-clearance < angle_error and angle_error < clearance):
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
        lean_angle_msg.data = current_angle - IMU_ANGLE_ERROR
        self.lean_angle_pub.publish(lean_angle_msg)
        
        # Publish tuning values for diagnostics
        # Approximating the PID components for monitoring
        tune_msg = TuningValues()
        tune_msg.kp = angle_error * self.K1
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