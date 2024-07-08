import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, Bool
from beep_interfaces.msg import MotorData
import sys 

from TMotorCANControl.servo_serial import *

CONTROL_MODES = {
    "POSITION": 0,
    "VELOCITY": 1,
    "CURRENT":  2,
    "DUTY":     3,
}


class MotorNode(Node):
    def __init__(self, device):
        super().__init__('motor_node')
        self.device = device
        self.dev_name = f"dev{self.device.port[-1]}"
        self.mode = 10              #CONTROL_MODES["POSITION"]
        self.motor_data = 0

        self.device.set_zero_position()
        
        self.device.update()

        self.velocity_sub = self.create_subscription(
            Float32,
            f"{self.dev_name}/velocity",
            self.velocity_cb,
            10
        )

        self.duty_sub = self.create_subscription(
            Float32,
            f"{self.dev_name}/duty",
            self.duty_cb,
            10
        )

        self.position_sub = self.create_subscription(
            Float32,
            f"{self.dev_name}/position",
            self.position_cb,
            10
        )

        self.current_sub = self.create_subscription(
            Float32,
            f"{self.dev_name}/current",
            self.current_cb,
            10
        )

        self.shutdown_sub = self.create_subscription (
            Bool,
            "shutdown",
            self.shutdown_cb,
            10
        )

        self.output_pub = self.create_publisher(
            MotorData,
            f"{self.dev_name}/motor_data",
            10
        )
        
        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        if self.mode == CONTROL_MODES["VELOCITY"]:
            self.device.set_output_velocity_radians_per_second(self.motor_data)
            self.device.update()
        elif self.mode == CONTROL_MODES["DUTY"]:
            self.device.set_duty_cycle_percent(self.motor_data)
            self.device.update()
        elif self.mode == CONTROL_MODES["POSITION"]:
            self.device.set_output_angle_radians(self.motor_data)
            self.device.update()
        elif self.mode == CONTROL_MODES["CURRENT"]:
            self.device.current_qaxis = self.motor_data
            self.device.update()

        data_msg = MotorData()
        data_msg.angle = self.device.get_output_angle_radians()
        data_msg.velocity = self.device.get_output_velocity_radians_per_second()
        data_msg.acceleration = self.device.get_output_acceleration_radians_per_second_squared()
        data_msg.torque = self.device.get_output_torque_newton_meters()
        data_msg.q_current = float(self.device.get_current_qaxis_amps())
        data_msg.q_voltage = float(self.device.get_voltage_qaxis_volts())
        data_msg.bus_current = float(self.device.get_current_bus_amps())
        data_msg.bus_voltage = float(self.device.get_voltage_bus_volts())

        self.output_pub.publish(data_msg)

    def velocity_cb(self, msg):
        
        if self.mode != CONTROL_MODES["VELOCITY"]:
            self.mode = CONTROL_MODES["VELOCITY"]
            self.device.enter_velocity_control()
            self.device.update()
            print("vel")
            
        self.motor_data = msg.data

    def duty_cb(self, msg):
        
        if self.mode != CONTROL_MODES["DUTY"]:
            self.mode = CONTROL_MODES["DUTY"]
            self.device.enter_duty_cycle_control()
            self.device.update()
            print("duty")
            print(msg.data)
            
        self.motor_data = msg.data

    def position_cb(self, msg):
        
        if self.mode != CONTROL_MODES["POSITION"]:
            self.mode = CONTROL_MODES["POSITION"]
            self.device.enter_position_control()
            self.device.update()
            print("position")
            
        self.motor_data = msg.data

    def current_cb(self, msg):
        
        if self.mode != CONTROL_MODES["CURRENT"]:
            self.mode = CONTROL_MODES["CURRENT"]
            self.device.enter_current_control()
            self.device.update()
            print("current")
            
        self.motor_data = msg.data
    
    def shutdown_cb(self, msg):
        if (msg.data):
            sys.exit(0)



