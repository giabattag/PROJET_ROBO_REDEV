#WE NEED A BETTER NAME
import math
import numpy as np
from geometry_msgs.msg import Quaternion, Vector3



def euler_to_quaternion(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)


def quaternion_to_euler_angle(quat):
    ysqr = quat.y * quat.y

    t0 = +2.0 * (quat.w * quat.x + quat.y * quat.z)
    t1 = +1.0 - 2.0 * (quat.x * quat.x + ysqr)
    X = (math.atan2(t0, t1))

    t2 = +2.0 * (quat.w * quat.y - quat.z * quat.x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = (math.asin(t2))

    t3 = +2.0 * (quat.w * quat.z + quat.x * quat.y)
    t4 = +1.0 - 2.0 * (ysqr + quat.z * quat.z)
    Z = (math.atan2(t3, t4))

    return X, Y, Z

def rotate_wrt(quaternion_ref, vector_to_transform):
    [rot_x, rot_y, rot_z] = quaternion_to_euler_angle(quaternion_ref)
    
    cx, sx = np.cos(rot_x), np.sin(rot_x) 
    cy, sy = np.cos(rot_y), np.sin(rot_y) 
    cz, sz = np.cos(rot_z), np.sin(rot_z) 

    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])

    R = Rx@Ry@Rz

    wrt_vec = R@np.array([vector_to_transform.x, vector_to_transform.y, vector_to_transform.z])
    return Vector3(x=wrt_vec[0], y=wrt_vec[1], z=wrt_vec[2])