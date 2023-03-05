from math import sin, cos, pi
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion, TwistStamped, PoseStamped, AccelStamped, WrenchStamped, TransformStamped, Wrench
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped, TransformListener, Buffer
import tf2_ros

import time

# from functions import euler_to_quaternion, quaternion_to_euler_angle, rotate_wrt

######


#WE NEED A BETTER NAME
import math
import numpy as np
from geometry_msgs.msg import Quaternion, Vector3
from scipy.spatial.transform import Rotation as R



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
    # [rot_x, rot_y, rot_z] = quaternion_to_euler_angle(quaternion_ref)
    
    # cx, sx = np.cos(rot_x), np.sin(rot_x) 
    # cy, sy = np.cos(rot_y), np.sin(rot_y) 
    # cz, sz = np.cos(rot_z), np.sin(rot_z) 

    # Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    # Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    # Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])

    r = R.from_quat([quaternion_ref.x, quaternion_ref.y, quaternion_ref.z, quaternion_ref.w])

    # R = Rx@Ry@Rz

    #MY QUESTION ABOUT MY OWN CODE --- WHY I DONT USE r (rotation matrix)

    wrt_vec = np.array([vector_to_transform.x, vector_to_transform.y, vector_to_transform.z])
    return Vector3(x=wrt_vec[0], y=wrt_vec[1], z=wrt_vec[2])

def quaternion_multiply(quaternion1, quaternion0):
    
    w0, x0, y0, z0 = quaternion0.w, quaternion0.x, quaternion0.y, quaternion0.z
    w1, x1, y1, z1 = quaternion1.w, quaternion1.x, quaternion1.y, quaternion1.z
    qutNP =  np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                     x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)

    quatRos = Quaternion()
    quatRos.w = qutNP[0]
    quatRos.x = qutNP[1]
    quatRos.y = qutNP[2]
    quatRos.z = qutNP[3]

    return quatRos


#################



class StatePublisher(Node):

    def __init__(self):

        #Initializing ROS
        rclpy.init()
        super().__init__('state_publisher')

        qos_profile = QoSProfile(depth=10)

            # message declarations
        odom_trans = TransformStamped()
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = 'drone'
        
        joint_state = JointState()
        self.propeller_speed = JointState()

            #PUBS
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
            
            #SUBS
        self.subs = self.create_subscription(JointState, 'motor_speed', self.proppeler_callback, qos_profile)
        self.subs
            #TF
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.tfBuffer = Buffer()
        self.tf_listener = TransformListener(self.tfBuffer, self)
        
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        degree = pi / 180.0
        self.rate = 30
        loop_rate = self.create_rate(self.rate)

        # robot state
            #x means the position in x
            #dx means the derivate of x. The number of d means the order of derivation
            #x_d the _d means w.r.t the drone frame, while _g global, and etc
            #roll pitch and yaw are thx, thy, thz respective (making the representation more homogeneos)
        
        pose_d = PoseStamped()
        twist_d = TwistStamped()
        accel_d = AccelStamped()
        config = '4_0'

        self.State_d = {
            "p"      : pose_d,
            "t"      : twist_d,
            "a"      : accel_d,
            "condig" : config
        }

        self.config_drone{
            '4_0':{
                'I'     : np.diag([4.21e-3, 3.70e-3, 7.79e-3]),
                'tht'   : [0., 0., 0., 0.]
            },
            '2_2':{
                'I'     : np.matrix([[4.12e-3, -1.07e-3, 0.0], [-1.07e-3, 3.33e-3, 0.0], [0.0, 0.0, 5.68e-3]]),
                'tht'   : [math.pi/2, 0., math.pi/2, 0.]
            },
            '0_4':{
                'I'     : np.matrix([3.53e-3, 2.37e-3, 3.52e-3]),
                'tht'   : [math.pi/2, math.pi/2, math.pi/2, math.pi/2]
            },
        }

        print("ESTADOO")

 
        
        
        try:
            while rclpy.ok():
                rclpy.spin_once(self)

                # update joint_state
                now = self.get_clock().now()
                joint_state.header.stamp = now.to_msg()
                joint_state.name = ["prop_to_arm_1", "prop_to_arm_2", "prop_to_arm_3", "prop_to_arm_4",'base_to_arm_1', "base_to_arm_2", "base_to_arm_3", "base_to_arm_4"]
                joint_state.position = [0.,#self.propeller_speed.velocity[self.propeller_speed.name.index("prop_to_arm_1")],
                                        0.,#self.propeller_speed.velocity[self.propeller_speed.name.index("prop_to_arm_2")],
                                        0.,#self.propeller_speed.velocity[self.propeller_speed.name.index("prop_to_arm_3")],
                                        0.,#,#self.propeller_speed.velocity[self.propeller_speed.name.index("prop_to_arm_4")],
                                        self.config_drone[self.State_d['config']]['tht'][0],
                                        self.config_drone[self.State_d['config']]['tht'][1],
                                        self.config_drone[self.State_d['config']]['tht'][2],
                                        self.config_drone[self.State_d['config']]['tht'][3]]

                # update transform
                # (moving in a circle with radius=2)
                start_trans = [frame for frame in ["prop_1", "prop_2", "prop_3", "prop_4"] if self.tfBuffer.can_transform("prop_1","odom", rclpy.time.Time())]
                # self.get_logger().info("Can Start Trans {0}".format(start_trans))
                if len(start_trans)==4:
                    self.drone_physics()
                odom_trans.header.stamp = now.to_msg()

                odom_trans.transform.translation.x = self.State_d['p'].pose.position.x
                odom_trans.transform.translation.y = self.State_d['p'].pose.position.y
                odom_trans.transform.translation.z = self.State_d['p'].pose.position.z

                # odom_trans.transform.translation.x = 0.# cos(angle)*2
                # odom_trans.transform.translation.y = 0.#sin(angle)*2
                # odom_trans.transform.translation.z = self.State_d['p'].pose.position.z
                odom_trans.transform.rotation = self.State_d['p'].pose.orientation
                    # euler_to_quaternion(0, 0, 0) # roll,pitch,yaw

                # send the joint state and transform
                self.joint_pub.publish(joint_state)
                self.broadcaster.sendTransform(odom_trans)
                self.get_logger().info("Drone in world {0}".format(self.State_d["p"].pose))


                # Create new robot state
                # tilt += tinc
                # if tilt < -0.5 or tilt > 0.0:
                #     tinc *= -1
                # height += hinc
                # if height > 0.2 or height < 0.0:
                #     hinc *= -1
                # # propeller += degree
                # angle += degree/4

                # This will adjust as needed per iteration
                # loop_rate.sleep()
                # self.get_logger().info("SLEEEP")
            


        except KeyboardInterrupt:
            pass

    def drone_physics(self):
        self.get_logger().info("PHYSIC")
        body_wrent = WrenchStamped()
        body_wrent.header.frame_id = "drone"

        #Pass this to a config file (extract from UDRF of YAML file)
        Kl = 0.0172 #Const for motor force
        Ki = 0.0172
        m_arm = 0.067
        m_body = 0.356    
        m_body = m_body + 4*m_arm #Drone mass
        g = 10.0 # gravity
        I_body = self.config_drone[self.State_d['config']][I]
        inverI = np.linalg.pinv(I_body)
        Jr = np.diag([0.0001, 0.0001, 0.0001]) #Rotor inertia

        body_wrent.wrench.force.z = -m_body*g

        for motor in self.propeller_speed.name:
            motor_wrent = WrenchStamped()
            motor_wrent.header.frame_id = motor

            # motor_sign = -1 if motor in ['prop_2', 'prop_4'] else 1
            
            try:

                #Generating linear force
                wi = self.propeller_speed.position[self.propeller_speed.name.index(motor)]
                motor_wrent.wrench.force.z = np.sign(wi)*(wi**2)*Kl
                motor_wrent.wrench.torque.z = motor_sign*np.sign(wi)*(wi**2)*Ki


                motor_to_body_wrent = Wrench()
                motor_to_drone = self.tfBuffer.lookup_transform(body_wrent.header.frame_id, motor, rclpy.time.Time())
                
                motor_to_body_wrent.force = rotate_wrt(motor_to_drone.transform.rotation, motor_wrent.wrench.force)
                motor_to_body_wrent.torque = rotate_wrt(motor_to_drone.transform.rotation, motor_wrent.wrench.torque)
                # self.get_logger().warning("Transform  {0}".format(motor_to_drone))



                body_wrent.wrench.force.x = body_wrent.wrench.force.x + motor_to_body_wrent.force.x
                body_wrent.wrench.force.y = body_wrent.wrench.force.y + motor_to_body_wrent.force.y
                body_wrent.wrench.force.z = body_wrent.wrench.force.z + motor_to_body_wrent.force.z

                #GYROSCOPE EFFECT
                # gyro =  np.cross(np.array([self.State_d['t'].twist.angular.x, self.State_d['t'].twist.angular.y, self.State_d['t'].twist.angular.z])
                #                 ,Jr@np.array([0, 0, wi]))
                # self.get_logger().warning("GYRO {0}".format(gyro))
                force_moment = np.cross(np.array([motor_to_drone.transform.translation.x, motor_to_drone.transform.translation.y, motor_to_drone.transform.translation.z]),
                                        np.array([motor_to_body_wrent.force.x, motor_to_body_wrent.force.y, motor_to_body_wrent.force.z]))


                body_wrent.wrench.torque.x = body_wrent.wrench.torque.x + motor_to_body_wrent.torque.x + force_moment[0] #+ gyro[0] #motor_to_body_wrent.force.y*motor_to_drone.transform.translation.z - motor_to_body_wrent.force.z*motor_to_drone.transform.translation.y + gyro[0]
                body_wrent.wrench.torque.y = body_wrent.wrench.torque.y + motor_to_body_wrent.torque.y + force_moment[1] #+ gyro[1] #motor_to_body_wrent.force.z*motor_to_drone.transform.translation.x - motor_to_body_wrent.force.x*motor_to_drone.transform.translation.z + gyro[1]
                body_wrent.wrench.torque.z = body_wrent.wrench.torque.z + motor_to_body_wrent.torque.z + force_moment[2] #+ gyro[2] #motor_to_body_wrent.force.x*motor_to_drone.transform.translation.y - motor_to_body_wrent.force.y*motor_to_drone.transform.translation.x + gyro[2]





            except Exception as e:
                self.get_logger().warning("ERRRORRR FORCE  {0}".format(e))

                continue

        self.get_logger().info("Force {0} {1} {2}".format(body_wrent.wrench.force.x,body_wrent.wrench.force.y,body_wrent.wrench.force.z))
        self.get_logger().info("Torque {0} {1} {2}".format(body_wrent.wrench.torque.x,body_wrent.wrench.torque.y,body_wrent.wrench.torque.z))



        self.State_d["a"].accel.linear.z = 1/m_body*(body_wrent.wrench.force.z)
        self.State_d["t"].twist.linear.z += 1/self.rate*(self.State_d["a"].accel.linear.z)

        self.State_d["a"].accel.linear.y = 1/m_body*(body_wrent.wrench.force.y)
        self.State_d["t"].twist.linear.y += 1/self.rate*(self.State_d["a"].accel.linear.y)
        
        self.State_d["a"].accel.linear.x = 1/m_body*(body_wrent.wrench.force.x)
        self.State_d["t"].twist.linear.x += 1/self.rate*(self.State_d["a"].accel.linear.x)

        self.get_logger().info("Lin - Aceel: {0} {1} {2}".format(self.State_d["a"].accel.linear.x, self.State_d["a"].accel.linear.y, self.State_d["a"].accel.linear.z))
        self.get_logger().info("Lin - SPEED: {0} {1} {2}".format(self.State_d["t"].twist.linear.x, self.State_d["t"].twist.linear.y, self.State_d["t"].twist.linear.z))


        torq = np.array([body_wrent.wrench.torque.x, body_wrent.wrench.torque.y, body_wrent.wrench.torque.z])
        ang_tw = np.array([self.State_d['t'].twist.angular.x, self.State_d['t'].twist.angular.y, self.State_d['t'].twist.angular.z])
        ac = np.clip(inverI@(torq), -0.3, 0.3) - np.clip(np.around(inverI@np.cross(ang_tw, I_body@ang_tw), decimals=5, -10, 10))
        
        self.State_d["a"].accel.angular.z = ac[2]
        self.State_d["t"].twist.angular.z += 1/self.rate*(self.State_d["a"].accel.angular.z)

        self.State_d["a"].accel.angular.y = ac[1]
        self.State_d["t"].twist.angular.y += 1/self.rate*(self.State_d["a"].accel.angular.y)
        
        self.State_d["a"].accel.angular.x = ac[0]
        self.State_d["t"].twist.angular.x += 1/self.rate*(self.State_d["a"].accel.angular.x)

        self.get_logger().info("Ang - Aceel: {0} {1} {2}".format(self.State_d["a"].accel.angular.x, self.State_d["a"].accel.angular.y, self.State_d["a"].accel.angular.z))
        self.get_logger().info("Ang - SPEED: {0} {1} {2}".format(self.State_d["t"].twist.angular.x, self.State_d["t"].twist.angular.y, self.State_d["t"].twist.angular.z))


        try:
            drone_to_world = self.tfBuffer.lookup_transform("odom", "drone", rclpy.time.Time())
            
            drone_velocity_world = rotate_wrt(drone_to_world.transform.rotation, self.State_d["t"].twist.linear)
            drone_rotationxyz_world = rotate_wrt(drone_to_world.transform.rotation, self.State_d["t"].twist.angular)
            drone_rottationquat_world = euler_to_quaternion(1/self.rate*drone_rotationxyz_world.x, 1/self.rate*drone_rotationxyz_world.y, 1/self.rate*drone_rotationxyz_world.z)

            self.State_d["p"].pose.position.x += 1/self.rate*(drone_velocity_world.x)     
            self.State_d["p"].pose.position.y += 1/self.rate*(drone_velocity_world.y)
            self.State_d["p"].pose.position.z += 1/self.rate*(drone_velocity_world.z)
            if self.State_d["p"].pose.position.z <= 0.0:
                self.State_d["p"].pose.position.z = 0.0

            self.State_d['p'].pose.orientation = quaternion_multiply(drone_rottationquat_world, self.State_d['p'].pose.orientation) 


        except Exception as e:
            self.get_logger().warning("ERRRORRR POSE  {0}".format(e))



        return False

    
    
    def proppeler_callback(self, msg):

        self.propeller_speed = msg



def main():

    node = StatePublisher()

if __name__ == '__main__':
    main()