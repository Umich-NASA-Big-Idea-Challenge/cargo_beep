import rclpy
import signal, sys
from cargo_beep.pid_helper import *
from beep_interfaces.msg import MotorData

class VelocityControllerNode(Node):

    def __init__(self):
        super().__init__('motor_control')

        self.error_prior = 0
        self.integral_prior = 0
        #past, .008, .0015, 0
        self.kp = 12.5 # .016 last
        self.ki = 0 # BEST .001
        self.kd = 0.0 # .0001
        self.bias = 0
        
        self.dt = GLOBAL_DT

        self.motor0_velocity = 0
        self.motor1_velocity = 0
        self.goal_velocity = 0
        self.wheel_radius = 0.4064
        self.current_lean_angle = 0

        signal.signal(signal.SIGINT, self.shutdown_cb)
        
        # CRITICAL CONTROLS
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

        self.current_lean_sub = self.create_subscription(
            Float32,
            "output/lean_angle",
            self.current_lean_cb,
            10
        )

        self.goal_lean_pub = self.create_publisher(
            Float32,
            "/setpoints/goal_lean",
            10
        )

        self.velocity_pub = self.create_publisher(
            Float32,
            "output/velocity",
            10
        )

        self.velocity_tuning_pub = self.create_publisher(
            TuningValues,
            "output/velocity_tuning_values",
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

        self.timer = self.create_timer(self.dt, self.timer_cb)

        print("Controlling Motors")

    def dev0_data_cb(self, msg):
        self.motor0_velocity = msg.velocity

    def dev1_data_cb(self, msg):
        self.motor1_velocity = -msg.velocity

    def setpoints_cb(self, msg):
        self.goal_velocity = msg.velocity

    def current_lean_cb(self, msg):
        self.current_lean_angle = msg.data

    def timer_cb(self):
        current_velocity = (self.motor0_velocity + self.motor1_velocity)/2
        current_velocity = -(current_velocity * self.wheel_radius)
        # calculate lean angle error
        error = self.goal_velocity - current_velocity

        
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

        lean_angle = output
        if (-clearance < error and error < clearance):
            lean_angle = float(0)

        angle_diff = self.current_lean_angle - lean_angle
        lean_angle = lean_angle + 0.75*angle_diff

        # if(math.fabs(angle_diff) > 1):
        #     #lean_angle = lean_angle + angle_diff
        #     if(lean_angle < self.current_lean_angle):
        #         lean_angle = self.current_lean_angle - 1
        #     else:
        #         lean_angle = self.current_lean_angle + 1



        
        goal_lean_angle_msg = Float32()
        goal_lean_angle_msg.data = lean_angle
        self.goal_lean_pub.publish(goal_lean_angle_msg)

        velocity_msg = Float32()
        velocity_msg.data = current_velocity
        self.velocity_pub.publish(velocity_msg)

        tune_msg = TuningValues()
        tune_msg.kp = error * self.kp
        tune_msg.ki = integral * self.ki
        tune_msg.kd = derivative * self.kd
        self.velocity_tuning_pub.publish(tune_msg)

    def shutdown_cb (self, signum, frame):
        shutdown_msg = Bool()
        shutdown_msg.data = True
        self.shutdown_pub.publish(shutdown_msg)
        sys.exit(0)


def main(args=None):
    rclpy.init(args=args)

    controller_node = VelocityControllerNode()

    rclpy.spin(controller_node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
