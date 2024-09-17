from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped

class StatePublisher(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('state_publisher')

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        loop_rate = self.create_rate(30)

        # robot state
        shoulder = 0.1
        wrist = 2.0
        elbow = 0.
        sinc = 0.01
        einc = 0.05
        winc = 0.1

        # message declarations
        joint_state = JointState()

        try:
            while rclpy.ok():
                rclpy.spin_once(self)

                # update joint_state
                now = self.get_clock().now()
                joint_state.header.stamp = now.to_msg()
                joint_state.name = ['standalone_shoulder_joint', 'standalone_elbow_joint', 'standalone_wrist_joint']
                joint_state.position = [shoulder, elbow, wrist]


                # send the joint state and transform
                self.joint_pub.publish(joint_state)

                # Create new robot state
                if shoulder < -0.3 or shoulder > 0.3:
                    sinc *= -1
                shoulder += sinc

                if elbow < -1.0 or elbow > 1.0:
                    einc *= -1
                elbow += einc

                if wrist <= 0 or wrist > 2.2:
                    winc *= -1
                wrist += winc

                # This will adjust as needed per iteration
                loop_rate.sleep()

        except KeyboardInterrupt:
            pass

def main():
    node = StatePublisher()

if __name__ == '__main__':
    main()