from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion, TwistStamped, PoseStamped, AccelStamped, WrenchStamped, Vector3, Accel
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped, TransformListener, Buffer
import numpy as np

class control(Node):

    def __init__(self):

        #Initializing ROS
        rclpy.init()
        super().__init__('control')

        qos_profile = QoSProfile(depth=10)


        
        self.joint_state = JointState()
        self.joint_state.name = ["prop_1", "prop_2", "prop_3", "prop_4"]
        self.joint_state.position = [0., 0., 0., 0.]

            #PUBS
        self.joint_pub = self.create_publisher(JointState, 'motor_speed', qos_profile)
            
            #SUBS
        self.cmd_subs =self.create_subscription(Accel, 'cmd', self.cmd_callback, qos_profile)
            #TF
      
        # robot state
        self.rate = 30

        loop_rate = self.create_rate(self.rate)


        
        try:
            while rclpy.ok():
                rclpy.spin_once(self)

                # update joint_state
                now = self.get_clock().now()
                self.joint_state.header.stamp = now.to_msg()
                #For convention even motors are negative (clockwise)

                # update transform
                # (moving in a circle with radius=2)

                # send the joint state and transform
                self.joint_pub.publish(self.joint_state)

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
                loop_rate.spin()

        except KeyboardInterrupt:
            pass


    def cmd_callback(self, msg):
        Ki = 0.0172
        m_arm = 0.067
        m_body = 0.356
        m_drone = m_body + 4*m_arm
        g = 10

        wi_eq = (g*m_drone/(4*Ki))
        ar_input = np.array([msg.linear.z,
                            msg.angular.x,
                            msg.angular.y,
                            msg.angular.z])

        thrust_map = np.matrix([[1, 1, 1, 1],
                                [1, -1, -1, 1],
                                [1, 1, -1, -1],
                                [1, -1, 1, -1]])
        aa = np.linalg.pinv(thrust_map)@ar_input

        self.joint_state.position[0] = (wi_eq*(1+aa[0,0]/5))**(1/2)
        self.joint_state.position[1] = (wi_eq*(1+aa[0,1]/5))**(1/2)
        self.joint_state.position[2] = (wi_eq*(1+aa[0,2]/5))**(1/2)
        self.joint_state.position[3] = (wi_eq*(1+aa[0,3]/5))**(1/2)
    




def main():

    node = control()
    # rclpy.spin(node)

if __name__ == '__main__':
    main()