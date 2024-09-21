from cargo_beep.pid_helper import *
import signal
import rclpy
import signal, sys

# Calculate the desired turning angle based on the quaternian
def get_turning_angle(q1):
    angle = 2 * math.atan2(q1.z, q1.w)
    return angle
    
class TurnControllerNode(Node):

    def __init__(self):
        super().__init__('motor_control')

        self.imu_data0 = Imu()

        self.error_prior = 0
        self.integral_prior = 0
        
        #past, .008, .0015, 0
        self.kp = .014 # .016 last
        self.ki = 0 # BEST .001
        self.kd = 0.000008 # .000075
        self.bias = 0

        self.desired_yaw = 0
        self.dt = GLOBAL_DT

        self.prior_orientations = []

        signal.signal(signal.SIGINT, self.shutdown_cb)
        
        # CRITICAL CONTROLS
        self.imu0_data_sub = self.create_subscription(
            Imu,
            "imu0/data",
            self.imu0_data_cb,
            10
        )

        # JOYSTICK INPUTS
        self.setpoints_sub = self.create_subscription(
            Setpoints,
            "setpoints",
            self.setpoints_cb,
            10
        )

        # OUTPUT FOR PID GOVERNOR
        self.turn_duty_pub = self.create_publisher(
            DutyPair,
            "/pid/turn",
            10
        )

        # OUTPUT FOR GRAPHING
        self.turn_angle_pub = self.create_publisher(
            Float32,
            'output/turn_angle',
            10
        )

        self.turn_tuning_pub = self.create_publisher(
            TuningValues,
            "output/turn_tuning_values",
            10
        )

        self.shutdown_pub = self.create_publisher(
            Bool,
            "shutdown",
            10
        )


        self.timer = self.create_timer(self.dt, self.timer_cb)

        print("Controlling Motors")

    def imu0_data_cb(self, msg):
        self.imu_data0 = msg

    def setpoints_cb(self, msg):
        self.desired_yaw -= msg.yaw
        if (self.desired_yaw < 0):
            self.desired_yaw = 359
        if (self.desired_yaw > 360):
            self.desired_yaw = 0


    def timer_cb(self):

        # calculate turning angle error
        error = self.desired_yaw - get_turning_angle(self.imu_data0.orientation)

        clearance = 0.1 # untested value --> arbitrary number, needs to be tested 

        integral = self.integral_prior + error * self.dt
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
        duty_msg.dev1 = duty
        self.turn_duty_pub.publish(duty_msg)
        
        turn_angle_msg = Float32()
        turn_angle_msg.data = get_turning_angle(self.imu_data0.orientation)
        self.turn_angle_pub.publish(turn_angle_msg)

        tune_msg = TuningValues()
        tune_msg.kp = error * self.kp
        tune_msg.ki = integral * self.ki
        tune_msg.kd = derivative * self.kd
        self.turn_tuning_pub.publish(tune_msg)
    
    def shutdown_cb (self, signum, frame):
        shutdown_msg = Bool()
        shutdown_msg.data = True
        self.shutdown_pub.publish(shutdown_msg)
        sys.exit(0)

def main(args=None):
    rclpy.init(args=args)

    controller_node = TurnControllerNode()

    rclpy.spin(controller_node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()