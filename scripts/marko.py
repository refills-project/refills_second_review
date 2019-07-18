import rospy
from actionlib import SimpleActionClient
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Vector3Stamped
from slipping_control_common.msg import SlippingControlAction, HomeGripperAction, GraspAction, HomeGripperGoal, \
    GraspGoal, SlippingControlGoal
from tf.transformations import quaternion_about_axis
import numpy as np

from giskardpy.python_interface import GiskardWrapper
from giskardpy.tfwrapper import lookup_pose
from refills_second_review.gripper import Gripper


class Plan(object):
    def __init__(self):
        self.without_base = 'base_footprint'
        self.with_base = 'odom'
        self.tip = 'refills_tool_frame'
        self.gripper = Gripper()
        self.giskard = GiskardWrapper()
        rospy.sleep(1)
        self.giskard.clear_world()

    def current_base_goal(self):
        base_goal = PoseStamped()
        base_goal.header.frame_id = self.with_base
        base_goal.pose.orientation.w = 1
        self.giskard.set_cart_goal(self.without_base, self.with_base, base_goal)

    def keep_horizontal(self, tip):
        root_normal = Vector3Stamped()
        root_normal.header.frame_id = 'map'
        root_normal.vector.z = 1

        tip_normal = Vector3Stamped()
        tip_normal.header.frame_id = 'refills_finger'
        tip_normal.vector.y = 1
        self.giskard.align_planes(tip, tip_normal, self.without_base, root_normal)

    def add_table(self, table_name='table'):
        box_pose = PoseStamped()
        box_pose.header.frame_id = 'map'
        box_pose.pose.position = Point(2.798, 2.768, 0.36)
        box_pose.pose.orientation.w = 1
        self.giskard.add_box(table_name, [2,2,0.72], pose=box_pose)

    def add_tulip(self, name='tulip'):
        box_pose = PoseStamped()
        box_pose.header.frame_id = 'map'
        box_pose.pose.position = Point(2.798, 2.768, 0.36)
        box_pose.pose.orientation.w = 1
        box_pose.pose.position.y -= 1 # table length
        box_pose.pose.position.y += 0.0475 # box length
        box_pose.pose.position.z += 0.36 # table height
        box_pose.pose.position.z += 0.045 # box height
        self.giskard.add_box(name, [0.05,0.095,0.09], pose=box_pose)

    def add_tulip_in_finger(self, name='tulip'):
        tulip_pose = PoseStamped()
        tulip_pose.header.frame_id = 'refills_finger'
        tulip_pose.pose.position.y = -0.01
        tulip_pose.pose.orientation.w = 1
        self.giskard.add_box(name, [0.05, 0.09, 0.095], pose=tulip_pose)

    def execute(self):
        tulip = 'tulip'

        self.gripper.home()
        self.add_table()
        self.add_tulip()

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position = Point(2.798, 1.849, 0.788)
        goal_pose.pose.orientation = Quaternion(-0.707, 0.017, -0.016, 0.707)
        self.giskard.set_cart_goal(self.without_base, self.tip, goal_pose)
        self.current_base_goal()
        self.giskard.plan_and_execute()

        self.gripper.grasp(5)
        self.gripper.gripper_pivoting()
        self.giskard.attach_object(tulip, 'refills_finger')

        gripper_goal = lookup_pose('map', self.tip)
        gripper_goal.pose.position.z += 0.1
        self.giskard.set_cart_goal(self.without_base, self.tip, gripper_goal)
        self.keep_horizontal(tulip)
        self.current_base_goal()

        self.giskard.plan_and_execute()

        rotation_goal = PoseStamped()
        rotation_goal.header.frame_id = self.tip
        rotation_goal.pose.orientation = Quaternion(*quaternion_about_axis(-0.3*np.pi, [1,0,0]))
        self.giskard.set_cart_goal(self.without_base, self.tip, rotation_goal)
        self.current_base_goal()
        self.keep_horizontal(tulip)
        self.giskard.plan_and_execute()

        rotation_goal = PoseStamped()
        rotation_goal.header.frame_id = self.tip
        rotation_goal.pose.orientation = Quaternion(*quaternion_about_axis(0.3*np.pi, [1,0,0]))
        self.giskard.set_cart_goal(self.without_base, self.tip, rotation_goal)
        self.current_base_goal()
        self.keep_horizontal(tulip)
        self.giskard.plan_and_execute()

        gripper_goal = lookup_pose('map', self.tip)
        gripper_goal.pose.position.z -= 0.1
        self.giskard.set_cart_goal(self.without_base, self.tip, gripper_goal)
        self.keep_horizontal(tulip)
        self.current_base_goal()
        self.giskard.plan_and_execute()
        self.gripper.home()

    def pick_up(self):
        tulip = 'tulip'

        self.gripper.home()
        self.add_table()
        self.add_tulip_in_finger()

        self.gripper.grasp(5)
        self.giskard.attach_object(tulip, 'refills_finger')

        gripper_goal = lookup_pose('map', self.tip)
        gripper_goal.pose.position.z += 0.02
        self.giskard.set_cart_goal(self.without_base, self.tip, gripper_goal)
        self.keep_horizontal(tulip)
        self.current_base_goal()
        self.giskard.plan_and_execute()

        self.gripper.gripper_pivoting()

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position = Point(2.798, 1.849, 0.788)
        goal_pose.pose.orientation = Quaternion(-0.707, 0.017, -0.016, 0.707)
        self.giskard.set_cart_goal(self.with_base, self.tip, goal_pose)
        self.keep_horizontal(tulip)
        self.giskard.plan_and_execute()
        self.gripper.home()


rospy.init_node('markos_demo')

plan = Plan()
plan.execute()



