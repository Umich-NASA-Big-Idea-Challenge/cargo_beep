import rclpy
import signal, sys
from cargo_beep.pid_helper import *


class LeanControllerNode(Node):

    def __init__(self):
        super().__init__('motor_control')

        self.imu_data0 = Imu()

        self.error_prior = 0
        self.integral_prior = 0
        #past, .008, .0015, 0
        self.kp = .007 # .016 last
        self.ki = 0 # BEST .001
        self.kd = 0.00003 # .0001
        self.bias = 0

        self.desired_angle = IMU_ANGLE_ERROR
        self.yaw = 0
        self.dt = GLOBAL_DT

        signal.signal(signal.SIGINT, self.shutdown_cb)
        
        # CRITICAL CONTROLS
        self.imu0_data_sub = self.create_subscription(
            Imu,
            "imu0/data",
            self.imu0_data_cb,
            10
        )

        self.lean_duty_pub = self.create_publisher(
            DutyPair,
            "/pid/lean",
            10
        )

        self.shutdown_pub = self.create_publisher(
            Bool,
            "shutdown",
            10
        )

        # JOYSTICK INPUTS

        self.setpoints_sub = self.create_subscription(
            Setpoints,
            "setpoints",
            self.setpoints_cb,
            10
        )

        # OUTPUT FOR GRAPHING

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


        self.timer = self.create_timer(self.dt, self.timer_cb)

        print("Controlling Motors")

    def imu0_data_cb(self, msg):
        self.imu_data0 = msg

    def imu1_data_cb(self, msg):
        self.imu_data1 = msg

    def setpoints_cb(self, msg):
        self.desired_angle = msg.lean_angle + IMU_ANGLE_ERROR

    def timer_cb(self):
        # Get current rotation angle
        rotation_axis = imu_axes['y']

        # convert from quaternion to euler
        euler_rot = euler_from_quat(self.imu_data0.orientation)
    
        # calculate lean angle error
        error = self.desired_angle - euler_rot[rotation_axis]
        
        clearance = 0.1 #untested value --> arbitrary number, needs to be tested 
    
        integral = self.integral_prior + error * self.dt
        
        #zero the integral if we are close to the 0 mark so we don't have runaway integral stuff
        #if (-clearance < error and error < clearance):
            #integral = float(0)


        #back to our regularly scheduled programming
        derivative = (error - self.error_prior) / self.dt
        output = self.kp*error + self.ki*integral + self.kd*derivative + self.bias
        self.error_prior = error
        self.integral_prior = integral

        if (-clearance < error and error < clearance):
            duty = float(0)
        else:
            duty = output_to_duty_power(output)

        duty_msg = DutyPair()
        duty_msg.dev0 = duty
        duty_msg.dev1 = -duty
        self.lean_duty_pub.publish(duty_msg)

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

    controller_node = LeanControllerNode()

    rclpy.spin(controller_node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
