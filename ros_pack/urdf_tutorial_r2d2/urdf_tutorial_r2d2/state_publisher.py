from math import sin, cos, pi
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion, TwistStamped, PoseStamped, AccelStamped, WrenchStamped, TransformStamped, Wrench
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped, TransformListener, Buffer
import tf2_ros


# from functions import euler_to_quaternion, quaternion_to_euler_angle, rotate_wrt

######


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

        self.State_d = {
            "p" : pose_d,
            "t" : twist_d,
            "a" : accel_d
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
                                        0.0, 0.0, 0.0, 0.0]

                # update transform
                # (moving in a circle with radius=2)

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
                self.get_logger().info("{0} Z".format(self.State_d["p"].pose.position.z))


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
                self.get_logger().info("SLEEEP")
            


        except KeyboardInterrupt:
            pass

    def drone_physics(self):
        self.get_logger().info("PHYSIC")
        body_wrent = WrenchStamped()
        body_wrent.header.frame_id = "drone"

        #Pass this to a config file (extract from UDRF of YAML file)
        Kl = 0.025 #Const for motor force
        Ki = 0.1    
        m = 1.0 #Drone mass
        g = 10.0 # gravity
        inverI = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])

        body_wrent.wrench.force.z = -m*g

        for motor in self.propeller_speed.name:
            motor_wrent = WrenchStamped()
            motor_wrent.header.frame_id = motor
            
            try:

                #Generating linear force
                wi = self.propeller_speed.position[self.propeller_speed.name.index(motor)]
                motor_wrent.wrench.force.z = (wi**2)*Kl
                motor_wrent.wrench.torque.z = np.sign(wi)*(wi**2)*Ki


                motor_to_body_wrent = Wrench()
                motor_to_drone = self.tfBuffer.lookup_transform(motor, body_wrent.header.frame_id, rclpy.time.Time())
                
                motor_to_body_wrent.force = rotate_wrt(motor_to_drone.transform.rotation, motor_wrent.wrench.force)
                motor_to_body_wrent.torque = rotate_wrt(motor_to_drone.transform.rotation, motor_wrent.wrench.torque)



                body_wrent.wrench.force.x = body_wrent.wrench.force.x + motor_to_body_wrent.force.x
                body_wrent.wrench.force.y = body_wrent.wrench.force.y + motor_to_body_wrent.force.y
                body_wrent.wrench.force.z = body_wrent.wrench.force.z + motor_to_body_wrent.force.z

                body_wrent.wrench.torque.x = body_wrent.wrench.torque.x + motor_to_body_wrent.force.y*motor_to_drone.transform.translation.z - motor_to_body_wrent.force.z*motor_to_drone.transform.translation.y + motor_to_body_wrent.torque.x 
                body_wrent.wrench.torque.y = body_wrent.wrench.torque.y + motor_to_body_wrent.force.z*motor_to_drone.transform.translation.x - motor_to_body_wrent.force.x*motor_to_drone.transform.translation.z + motor_to_body_wrent.torque.y
                body_wrent.wrench.torque.z = body_wrent.wrench.torque.z + motor_to_body_wrent.force.x*motor_to_drone.transform.translation.y - motor_to_body_wrent.force.y*motor_to_drone.transform.translation.x + motor_to_body_wrent.torque.z




                self.get_logger().info("Wrench {0} {1} {2}".format(body_wrent.wrench.force.x,body_wrent.wrench.force.y,body_wrent.wrench.force.z))

            except Exception as e:
                self.get_logger().info("ERRRORRR FORCE  {0}".format(e))

                continue


        self.State_d["a"].accel.linear.z = 1/m*(body_wrent.wrench.force.z)
        self.State_d["t"].twist.linear.z = 1/self.rate*(self.State_d["a"].accel.linear.z)

        self.State_d["a"].accel.linear.y = 1/m*(body_wrent.wrench.force.y)
        self.State_d["t"].twist.linear.y += 1/self.rate*(self.State_d["a"].accel.linear.y)
        
        self.State_d["a"].accel.linear.x = 1/m*(body_wrent.wrench.force.x)
        self.State_d["t"].twist.linear.x += 1/self.rate*(self.State_d["a"].accel.linear.x)

        self.get_logger().info("Aceel: {0} {1} {2}".format(self.State_d["a"].accel.linear.x, self.State_d["a"].accel.linear.y, self.State_d["a"].accel.linear.z))
        self.get_logger().info("SPEED: {0} {1} {2}".format(self.State_d["t"].twist.linear.x, self.State_d["t"].twist.linear.y, self.State_d["t"].twist.linear.z))


        torq = np.array([body_wrent.wrench.torque.x, body_wrent.wrench.torque.y, body_wrent.wrench.torque.z])
        ac = inverI@(torq)
        
        self.State_d["a"].accel.angular.z = ac[2]
        self.State_d["t"].twist.angular.z = 1/self.rate*(self.State_d["a"].accel.angular.z)

        self.State_d["a"].accel.angular.y = ac[1]
        self.State_d["t"].twist.angular.y += 1/self.rate*(self.State_d["a"].accel.angular.y)
        
        self.State_d["a"].accel.angular.x = ac[0]
        self.State_d["t"].twist.angular.x += 1/self.rate*(self.State_d["a"].accel.angular.x)


        try:
            drone_to_world = self.tfBuffer.lookup_transform("drone", "odom", rclpy.time.Time())
            
            drone_velocity_world = rotate_wrt(drone_to_world.transform.rotation, self.State_d["t"].twist.linear)
            drone_rotationxyz_world = rotate_wrt(drone_to_world.transform.rotation, self.State_d["t"].twist.angular)
            drone_rottationquat_world = euler_to_quaternion(drone_rotationxyz_world.x, drone_rotationxyz_world.y, drone_rotationxyz_world.z)

            self.State_d["p"].pose.position.x += 1/self.rate*(drone_velocity_world.x)     
            self.State_d["p"].pose.position.y += 1/self.rate*(drone_velocity_world.y)
            self.State_d["p"].pose.position.z += 1/self.rate*(drone_velocity_world.z)

            self.State_d['p'].pose.orientation = quaternion_multiply(drone_rottationquat_world, self.State_d['p'].pose.orientation) 


        except Exception as e:
            self.get_logger().info("ERRRORRR POSE  {0}".format(e))



        return False

    
    
    def proppeler_callback(self, msg):

        self.propeller_speed = msg



def main():
    print("AAAAAAAAAAAAAAAAAAAAAAA  ")

    node = StatePublisher()

if __name__ == '__main__':
    main()