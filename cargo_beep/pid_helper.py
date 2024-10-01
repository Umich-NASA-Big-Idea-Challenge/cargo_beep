from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import Imu
from beep_interfaces.msg import TuningValues, Setpoints, DutyPair, MotorData
from scipy.spatial.transform import Rotation
import numpy as np
import math

imu_axes = {'x': 0, 'y': 1, 'z': 2}

GLOBAL_DT = .002

IMU_ANGLE_ERROR = 16.5
YAW_SCALE = .05

MAX_DUTY = .25

def output_to_duty_power (output):
    duty = min(MAX_DUTY, max(output, -MAX_DUTY))
    return float(duty)


# helper func for euler_from_quaternion
def rad_to_deg(x, y, z):
    return [x * 180.0 / (2 * math.pi), y * 180.0 / (2 * math.pi), z * 180.0 / (2 * math.pi)]

def euler_from_quat(quat):
    rot = Rotation.from_quat((quat.x, quat.y, quat.z, quat.w))
    rot_euler = rot.as_euler("xyz", degrees=True)
    return rot_euler

# takes in a Quaternian msg and returns a 3 tuple (x, y, z)
def euler_from_quaternion(q1):
    sqw = q1.w * q1.w
    sqx = q1.x * q1.x
    sqy = q1.y * q1.y
    sqz = q1.z * q1.z

    unit = sqx + sqy + sqz + sqw
    test = q1.x*q1.y + q1.z*q1.w

    #heading, attitude, bank = y,z,x
    if (test > 0.499*unit): # singularity at north pole
        y = 2 * math.atan2(q1.x,q1.w)
        z = math.pi/2
        x = 0
        return rad_to_deg(x, y, z)

    if (test < -0.499*unit): # singularity at south pole
        y = -2 * math.atan2(q1.x,q1.w)
        z = -math.pi/2
        x = 0
        return rad_to_deg(x, y, z)

    y = math.atan2(2 * q1.y * q1.w - 2 * q1.x * q1.z , sqx - sqy - sqz + sqw)
    z = math.asin(2 * test/unit)
    x = math.atan2(2 * q1.x * q1.w-2 * q1.y * q1.z , -sqx + sqy - sqz + sqw)

    return rad_to_deg(x, y, z)
