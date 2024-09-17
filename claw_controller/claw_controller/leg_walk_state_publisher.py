from math import atan, acos, sin, cos, pi
from numpy import linspace
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

class StatePublisher(Node):
    update_freq = 50
    cycle_length = 3
    stage_length = 0
    step_count = 0
    stage = 0 # 0 push, 1 lift, 2 place

    max_half_stride = 0.05
    stride_percentage = 0.75

    shoulder_length = 0.02
    thigh_length = 0.05
    shank_length = 0.04
    shoulder = 0.
    elbow = 0.
    wrist = 0.
    

    shoulder_upper = pi/8.
    shoulder_lower = -pi/8.
    elbow_upper = pi/3.
    elbow_lower = -pi/3.
    wrist_upper = 3*pi/4.
    wrist_lower = 0.

    x = 0.
    y = 0.04
    z = 0.05
    xy_limit = thigh_length + shank_length




    def __init__(self):
        super().__init__('state_publisher')

        qos_profile = QoSProfile(depth=10)
        self.cb_1_ = MutuallyExclusiveCallbackGroup()
        self.cb_2_ = MutuallyExclusiveCallbackGroup()

        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.nodeName = self.get_name()
        self.subscriber_ = self.create_subscription(
            Twist, 'cmd_vel', self.callback_twist, 10, callback_group=self.cb_1_)
        self.update_timer = self.create_timer(
            1.0/self.update_freq, self.publish_states, callback_group=self.cb_1_)
        self.cycle_timer = self.create_timer(
            1.0/self.update_freq, self.update_cycle, callback_group=self.cb_2_)
        self.get_logger().info("{0} started".format(self.nodeName))

        self.loop_rate = self.create_rate(30)

        # message declarations
        self.joint_state = JointState()
        self.publish_states()
        self.enter_cycle_stage(0)

        
    def callback_twist(self, msg):
        self.stride_percentage = msg.linear.x/100

            
    def update_cycle(self):
        self.x = self.xsteps[self.step_count]
        self.y = self.ysteps[self.step_count]
        self.step_count += 1
        if self.step_count == self.stage_length:
            self.step_count = 0
            self.stage += 1
            if self.stage == self.cycle_length:
                self.stage = 0
            self.enter_cycle_stage(self.stage)
        
    def enter_cycle_stage(self, stage):
        self.step_count = 0
        match stage:
            case 0:
                self.target_pos = [self.max_half_stride*self.stride_percentage, 0.07]
                self.stage_length = 20
            case 1:
                self.target_pos = [0.0, 0.06-0.02*self.stride_percentage]
                self.stage_length = 10
            case 2:
                self.target_pos = [-self.max_half_stride*self.stride_percentage, 0.07]
                self.stage_length = 10
        self.get_logger().info("stride_length {0}".format(self.stride_percentage))
        self.xsteps = linspace(self.x, self.target_pos[0], self.stage_length)
        self.ysteps = linspace(self.y, self.target_pos[1], self.stage_length)
       


    def legIK(self):

        q2_t = self.x**2+self.y**2-(self.shank_length**2)-(self.thigh_length**2)
        q2_b = 2.0*self.shank_length*self.thigh_length
        q2_ = q2_t/q2_b
        if q2_ > 1:
            q2_ = 1
        if q2_ < -1:
            q2_ = 1
        q2 = acos(q2_)


        q1_t = self.shank_length*sin(q2)
        q1_b = self.thigh_length+self.shank_length*cos(q2)

        q1_ = q1_t/q1_b
        if q1_ > 1:
            q1_ = 1
        if q1_ < -1:
            q1_ = 1
        q1 = atan(self.x/self.y)-atan(q1_)

        self.elbow = q1
        self.wrist = q2

        if self.shoulder > self.shoulder_upper:
            self.shoulder = self.shoulder_upper
        if self.shoulder < self.shoulder_lower:
            self.shoulder = self.shoulder_lower
        
        if self.elbow > self.elbow_upper:
            self.elbow = self.elbow_upper
        if self.elbow < self.elbow_lower:
            self.elbow = self.elbow_lower
        
        if self.wrist > self.wrist_upper:
            self.wrist = self.wrist_upper
        if self.wrist < self.wrist_lower:
            self.wrist = self.wrist_lower

    def publish_states(self):
        # Create new robot state
        self.legIK()

        # update joint_state
        now = self.get_clock().now()
        self.joint_state.header.stamp = now.to_msg()
        self.joint_state.name = ['standalone_shoulder_joint', 'standalone_elbow_joint', 'standalone_wrist_joint']
        self.joint_state.position = [self.shoulder, self.elbow, self.wrist]

        # send the joint state and transform
        self.joint_pub.publish(self.joint_state)



def main(args=None):
    rclpy.init(args=args)
    node = StatePublisher()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    rclpy.shutdown()

if __name__ == '__main__':
    main()