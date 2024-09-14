import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from beep_interfaces.msg import TuningValues
from beep_interfaces.msg import Setpoints

from scipy.spatial.transform import Rotation
import signal, sys
import math

imu_axes = {'x': 0, 'y': 1, 'z': 2}


DESIRED_ANGLE = 0
YAW_SCALE = .1

# takes in a Quaternian msg and returns a 3 tuple (x, y, z)
def euler_from_quat(quat):
    rot = Rotation.from_quat((quat.x, quat.y, quat.z, quat.w))
    rot_euler = rot.as_euler("xyz", degrees=True)
    return rot_euler

def rad_to_deg(x, y, z):
    return [x * 180.0 / (2 * math.pi), y * 180.0 / (2 * math.pi), z * 180.0 / (2 * math.pi)]

def euler_from_quaternion(q1):
    sqw = q1.w * q1.w
    sqx = q1.x * q1.x
    sqy = q1.y * q1.y
    sqz = q1.z * q1.z

    unit = sqx + sqy + sqz + sqw
    test = q1.x*q1.y + q1.z*q1.w

    #heading, attitude, bank = y,z,x
    if (test > 0.499*unit): # singularity at north pole
        #print("singularity at north pole")
        y = 2 * math.atan2(q1.x,q1.w)
        z = math.pi/2
        x = 0
        return rad_to_deg(x, y, z)

    if (test < -0.499*unit): # singularity at south pole
        #print("singularity at south pole")
        y = -2 * math.atan2(q1.x,q1.w)
        z = -math.pi/2
        x = 0
        return rad_to_deg(x, y, z)

    y = math.atan2(2 * q1.y * q1.w - 2 * q1.x * q1.z , sqx - sqy - sqz + sqw)
    z = math.asin(2 * test/unit)
    x = math.atan2(2 * q1.x * q1.w-2 * q1.y * q1.z , -sqx + sqy - sqz + sqw)

    return rad_to_deg(x, y, z)


MAX_DUTY = .25
def output_to_duty_power (output):
    duty = min(MAX_DUTY, max(output, -MAX_DUTY))
    return float(duty)


class PIDControllerNode(Node):

    def __init__(self):
        super().__init__('motor_control')

        self.imu_data0 = Imu()
        self.imu_data1 = Imu()

        self.error_prior = 0
        self.integral_prior = 0
        #past, .008, .0015, 0
        self.kp = .0035 # BEST .0055
        self.ki = .001 # BEST .001
        self.kd = .0001 #.000075 BEST .0001
        self.bias = 0

        self.desired_angle = DESIRED_ANGLE
        self.yaw = 0
        self.dt = .002

        signal.signal(signal.SIGINT, self.shutdown_cb)
        
        # CRITICAL CONTROLS
        self.imu0_data_sub = self.create_subscription(
            Imu,
            "imu0/data",
            self.imu0_data_cb,
            10
        )

        # self.imu1_data_sub = self.create_subscription(
        #     Imu,
        #     "imu1/data",
        #     self.imu1_data_cb,
        #     10
        # )

        self.motor0_duty_pub = self.create_publisher(
            Float64,
            "dev0/duty",
            10
        )

        self.motor1_duty_pub = self.create_publisher(
            Float64,
            "dev1/duty",
            10
        )

        self.motor0_sim_pub = self.create_publisher(
            Float64,
            "dev0/sim",
            10
        )

        self.motor1_sim_pub = self.create_publisher(
            Float64,
            "dev1/sim",
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
            Float64,
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

    def setpoints_cb(self, msg):
        self.desired_angle = msg.lean_angle
        self.yaw = msg.yaw

    def timer_cb(self):
        # Get current rotation angle
        rotation_axis = imu_axes['y']
        euler_rot = euler_from_quaternion(self.imu_data0.orientation)
        
        error = self.desired_angle - euler_rot[rotation_axis]
        
        clearence = 0.1 #untested value --> arbitrary number, needs to be tested 
    
        integral = self.integral_prior + error * self.dt
        
        #zero the integral if we are close to the 0 mark so we don't have runaway integral stuff
        if (-clearence < error and error < clearence):
            integral = float(0)


        #back to our regularly scheduled programming
        derivative = (error - self.error_prior) / self.dt
        output = self.kp*error + self.ki*integral + self.kd*derivative + self.bias
        self.error_prior = error
        self.integral_prior = integral

        if (-clearence < error and error < clearence):
            duty = float(0)
        else:
            duty = output_to_duty_power(output)


        sim_msg0 = Float64()
        duty_msg0 = Float64()
        duty_msg0.data = duty + (self.yaw*YAW_SCALE)
        sim_msg0.data = (- duty + (-self.yaw*YAW_SCALE))* (40.84070445 * 2)
        self.motor0_duty_pub.publish(duty_msg0)
        self.motor0_sim_pub.publish(sim_msg0)
        
        sim_msg1 = Float64()
        duty_msg1 = Float64()
        duty_msg1.data = -duty + (self.yaw*YAW_SCALE)
        sim_msg1.data = (- duty + (self.yaw*YAW_SCALE))* (40.84070445 * 2)

        self.motor1_duty_pub.publish(duty_msg1)
        self.motor1_sim_pub.publish(sim_msg1)
        
        lean_angle_msg = Float64()
        lean_angle_msg.data = euler_rot[rotation_axis]
        self.lean_angle_pub.publish(lean_angle_msg)

        tune_msg = TuningValues()
        tune_msg.kp = error
        tune_msg.ki = integral
        tune_msg.kd = derivative
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