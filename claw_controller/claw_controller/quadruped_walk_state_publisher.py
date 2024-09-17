from math import atan, acos, sin, cos, pi
from numpy import linspace
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

class ClawLeg:
    cycle_length = 3
    stage_length = 0
    step_count = 0
    stage = 0 # 0 push, 1 lift, 2 place

    xsteps = []
    ysteps = []
    z_inc = 0.0001

    max_half_stride = 0.05
    stride_percentage = 0.8
    height = 0.06
    step_height = 0.04

    shoulder_length = -0.02
    thigh_length = 0.05
    shank_length = 0.04

    shoulder = 0.
    elbow = 0.
    wrist = 0.
    
    shoulder_limit = [-pi/3., pi/3.]

    elbow_limit = [-pi/3., pi/3.]
    wrist_limit = [0., 3*pi/4.]

    x = 0.
    y = 0.07
    z = shoulder_length

    xy_limit = thigh_length + shank_length

    def __init__(self, stage, side):
        self.stage = stage
        self.enter_cycle_stage(stage)
        self.shoulder_length = self.shoulder_length*side

    def update_cycle(self, stride_percentage):
        self.stride_percentage = stride_percentage
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
        self.stage = stage
        match self.stage:
            case 0:
                self.target_pos = [self.max_half_stride*self.stride_percentage, self.height]
                self.stage_length = 200
            case 1:
                self.target_pos = [0.0, self.step_height]
                self.stage_length = 100
            case 2:
                self.target_pos = [-self.max_half_stride*self.stride_percentage, self.height]
                self.stage_length = 100
        self.xsteps = linspace(self.x, self.target_pos[0], self.stage_length)
        self.ysteps = linspace(self.y, self.target_pos[1], self.stage_length)

    def set_z(self, z):
        if self.z < z:
            self.z += self.z_inc
        if self.z > z:
            self.z -= self.z_inc

    def legIK(self):
        #virtual joint angle using z and y
        self.v_joint = atan(self.z/self.y)
        #virtual leg length using y and v_joint
        self.v_length = self.z/(sin(self.v_joint))
        #offset joint angle
        self.o_joint = acos(self.shoulder_length/self.v_length)
        #real leg length
        self.leg_length = self.v_length*sin(self.o_joint)
        #shoulder joint
        self.shoulder = self.o_joint + self.v_joint - pi/2.

        # determine elbow and wrist angles using x and leg_length to replace y
        self.q2_t = self.x**2+self.leg_length**2-(self.shank_length**2)-(self.thigh_length**2)
        self.q2_b = 2.0*self.shank_length*self.thigh_length
        self.q2_ = self.q2_t/self.q2_b
        if self.q2_ > 1:
            self.q2_ = 1
        if self.q2_ < -1:
            self.q2_ = 1
        self.q2 = acos(self.q2_)

        self.q1_t = self.shank_length*sin(self.q2)
        self.q1_b = self.thigh_length+self.shank_length*cos(self.q2)

        self.q1_ = self.q1_t/self.q1_b
        if self.q1_ > 1:
            self.q1_ = 1
        if self.q1_ < -1:
            self.q1_ = 1
        self.q1 = atan(self.x/self.leg_length)-atan(self.q1_)

        self.elbow = self.q1
        self.wrist = self.q2

        # if self.shoulder > self.shoulder_limit[1]:
        #     self.shoulder = self.shoulder_limit[1]
        # if self.shoulder < self.shoulder_limit[0]:
        #     self.shoulder = self.shoulder_limit[0]
        
        # if self.elbow > self.elbow_limit[1]:
        #     self.elbow = self.elbow_limit[1]
        # if self.elbow < self.elbow_limit[0]:
        #     self.elbow = self.elbow_limit[0]
        
        # if self.wrist > self.wrist_limit[1]:
        #     self.wrist = self.wrist_limit[1]
        # if self.wrist < self.wrist_limit[0]:
        #     self.wrist = self.wrist_limit[0]



class StatePublisher(Node):
    update_freq = 500
    stride_percentage = 0
    z_target = 0.01
    fr_leg = ClawLeg(0, 1)
    fl_leg = ClawLeg(1, -1)
    rr_leg = ClawLeg(1, 1)
    rl_leg = ClawLeg(0, -1)
    
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
        
    def callback_twist(self, msg):
        self.stride_percentage = msg.linear.x/100
        self.z_target = msg.angular.z/1000
        #self.get_logger().info("stride_length {0}".format(self.stride_percentage))

    def update_cycle(self):
        self.fr_leg.update_cycle(self.stride_percentage)
        self.fl_leg.update_cycle(self.stride_percentage)
        self.rr_leg.update_cycle(self.stride_percentage)
        self.rl_leg.update_cycle(self.stride_percentage)
        self.fr_leg.set_z(self.fr_leg.shoulder_length + self.z_target)
        self.fl_leg.set_z(self.fl_leg.shoulder_length + self.z_target)
        self.rr_leg.set_z(self.rr_leg.shoulder_length + self.z_target)
        self.rl_leg.set_z(self.rl_leg.shoulder_length + self.z_target)
        self.get_logger().info("fr.s {0} fr.e {1} fr.w {2}".format(self.fr_leg.shoulder, self.fr_leg.elbow, self.fr_leg.wrist))

    def publish_states(self):
        # Create new robot state
        self.fr_leg.legIK()
        self.fl_leg.legIK()
        self.rr_leg.legIK()
        self.rl_leg.legIK()

        # update joint_state
        now = self.get_clock().now()
        self.joint_state.header.stamp = now.to_msg()
        self.joint_state.name = [
            'fr_shoulder_joint', 'fr_elbow_joint', 'fr_wrist_joint', 
            'fl_shoulder_joint', 'fl_elbow_joint', 'fl_wrist_joint',
            'rr_shoulder_joint', 'rr_elbow_joint', 'rr_wrist_joint',
            'rl_shoulder_joint', 'rl_elbow_joint', 'rl_wrist_joint'
        ]
        self.joint_state.position = [
            self.fr_leg.shoulder, self.fr_leg.elbow, self.fr_leg.wrist,
            self.fl_leg.shoulder, self.fl_leg.elbow, self.fl_leg.wrist,
            self.rr_leg.shoulder, self.rr_leg.elbow, self.rr_leg.wrist,
            self.rl_leg.shoulder, self.rl_leg.elbow, self.rl_leg.wrist
        ]

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