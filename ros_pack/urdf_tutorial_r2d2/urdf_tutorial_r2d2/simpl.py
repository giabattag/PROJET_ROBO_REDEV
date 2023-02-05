from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion, TwistStamped, PoseStamped, AccelStamped, WrenchStamped, Vector3
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped, TransformListener, Buffer

class pub(Node):

    def __init__(self):

        #Initializing ROS
        rclpy.init()
        super().__init__('pub')

        qos_profile = QoSProfile(depth=10)


        
        joint_state = JointState()

            #PUBS
        self.joint_pub = self.create_publisher(JointState, 'motor_speed', qos_profile)
            
            #SUBS
            
            #TF
      
        # robot state
        self.rate = 30

        loop_rate = self.create_rate(self.rate)


        
        try:
            while rclpy.ok():
                rclpy.spin_once(self)

                # update joint_state
                now = self.get_clock().now()
                joint_state.header.stamp = now.to_msg()
                joint_state.name = ["prop_1", "prop_2", "prop_3", "prop_4"]
                #For convention even motors are negative (clockwise)
                joint_state.position = [10.0,#self.propeller_speed.velocity[self.propeller_speed.name.index("prop_to_arm_1")],
                                        -10.0,#self.propeller_speed.velocity[self.propeller_speed.name.index("prop_to_arm_2")],
                                        10.0,#self.propeller_speed.velocity[self.propeller_speed.name.index("prop_to_arm_3")],
                                        -10.0#self.propeller_speed.velocity[self.propeller_speed.name.index("prop_to_arm_4")],
                                       ]# 0.0, 0.0, 0.0, 0.0]

                # update transform
                # (moving in a circle with radius=2)

                # send the joint state and transform
                self.joint_pub.publish(joint_state)

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
                loop_rate.sleep()

        except KeyboardInterrupt:
            pass


    
    
    def proppeler_callback(self, msg):
        self.propeller_speed = msg

def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)



def main():
    print("AAAAAAAAAAAAAAAAAAAAAAA  ")

    node = pub()

if __name__ == '__main__':
    main()