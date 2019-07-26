import numpy as np
from itertools import product

import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Vector3Stamped, Vector3
from giskard_msgs.msg import CollisionEntry, MoveResult
from tf.transformations import quaternion_about_axis, quaternion_from_matrix

from giskardpy.python_interface import GiskardWrapper
from giskardpy.tfwrapper import lookup_pose
from refills_second_review.gripper import Gripper
from utils_for_tests import compare_poses


class Plan(object):
    def __init__(self):
        self.without_base = 'base_footprint'
        self.with_base = 'odom'
        self.tip = 'refills_tool_frame'
        self.gripper = Gripper(False)
        self.giskard = GiskardWrapper()
        rospy.sleep(1)
        self.giskard.clear_world()

    def start_config(self):
        if not self.gripper.active:
            js = {
                'odom_x_joint': 2.9,
                'odom_y_joint': 0.2,
                'odom_z_joint': -1.57,
                'ur5_shoulder_pan_joint': -2.09534116068e-16,
                'ur5_shoulder_lift_joint': -2.17000000194,
                'ur5_elbow_joint': -1.66,
                'ur5_wrist_1_joint': -0.2,
                'ur5_wrist_2_joint': 1.57,
                'ur5_wrist_3_joint': -1.57,
            }
            self.giskard.set_joint_goal(js)
            self.giskard.plan_and_execute()

    def current_base_goal(self):
        base_goal = PoseStamped()
        base_goal.header.frame_id = self.with_base
        base_goal.pose.orientation.w = 1
        self.giskard.set_cart_goal(self.without_base, self.with_base, base_goal)

    def keep_horizontal(self, tip, normal=(0,1,0)):
        root_normal = Vector3Stamped()
        root_normal.header.frame_id = 'map'
        root_normal.vector.z = 1

        tip_normal = Vector3Stamped()
        tip_normal.header.frame_id = 'refills_finger'
        tip_normal.vector = Vector3(*normal)
        self.giskard.align_planes(tip, tip_normal, self.without_base, root_normal)

    def add_table(self, table_name='table'):
        box_pose = PoseStamped()
        box_pose.header.frame_id = 'map'
        box_pose.pose.position = Point(2.798, 2.768, 0.36)
        box_pose.pose.orientation.w = 1
        self.giskard.add_box(table_name, [2, 2, 0.72], pose=box_pose)

    def add_tulip(self, name='tulip'):
        box_pose = PoseStamped()
        box_pose.header.frame_id = 'map'
        box_pose.pose.position = Point(2.798, 2.768, 0.36)
        box_pose.pose.orientation.w = 1
        box_pose.pose.position.y -= 1  # table length
        box_pose.pose.position.y += 0.0475  # box length
        box_pose.pose.position.z += 0.36  # table height
        box_pose.pose.position.z += 0.045  # box height
        self.giskard.add_box(name, [0.05, 0.095, 0.09], pose=box_pose)

    def add_tulip_on_floor(self, name='tulip'):
        box_pose = PoseStamped()
        box_pose.header.frame_id = 'map'
        box_pose.pose.position = Point(2.8, 1.1, 0.00)
        box_pose.pose.orientation.w = 1
        box_pose.pose.position.z += 0.045  # box height
        self.giskard.add_box(name, [0.05, 0.095, 0.09], pose=box_pose)
        return box_pose

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
        rotation_goal.pose.orientation = Quaternion(*quaternion_about_axis(-0.3 * np.pi, [1, 0, 0]))
        self.giskard.set_cart_goal(self.without_base, self.tip, rotation_goal)
        self.current_base_goal()
        self.keep_horizontal(tulip)
        self.giskard.plan_and_execute()

        rotation_goal = PoseStamped()
        rotation_goal.header.frame_id = self.tip
        rotation_goal.pose.orientation = Quaternion(*quaternion_about_axis(0.3 * np.pi, [1, 0, 0]))
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

    def goal_reached(self, goal, tip):
        current_pose = lookup_pose(goal.header.frame_id, tip)
        try:
            compare_poses(current_pose.pose, goal.pose)
        except AssertionError:
            return False
        return True

    def pick_up(self, start_angle=0, goal_angle=0):
        if start_angle is not None:
            start_angle -= np.pi / 2
        if goal_angle is not None:
            goal_angle -= np.pi / 2
        tulip = 'tulip'

        self.gripper.home()
        self.add_table()
        # self.add_tulip_in_finger()
        arm_goal = self.add_tulip_on_floor()
        arm_goal.pose.orientation = Quaternion(*quaternion_from_matrix([[1, 0, 0, 0],
                                                                        [0, 0, -1, 0],
                                                                        [0, 1, 0, 0],
                                                                        [0, 0, 0, 1]]))
        self.giskard.set_cart_goal(self.without_base, 'refills_finger', arm_goal)
        self.giskard.allow_collision([CollisionEntry.ALL], tulip, [CollisionEntry.ALL])
        if start_angle is not None:
            self.giskard.set_joint_goal({'refills_finger_joint': start_angle})
        if not self.giskard.plan_and_execute().error_code == MoveResult.SUCCESS or \
                not self.goal_reached(arm_goal, 'refills_finger'):
            print('no solution found; stopping test')
            return 'start'


        self.gripper.grasp(5)
        self.giskard.attach_object(tulip, 'refills_finger')

        gripper_goal = lookup_pose('map', self.tip)
        gripper_goal.pose.position.z += 0.02
        self.giskard.set_cart_goal(self.without_base, self.tip, gripper_goal)
        self.keep_horizontal(tulip)
        self.current_base_goal()
        if not self.giskard.plan_and_execute().error_code == MoveResult.SUCCESS or \
                not self.goal_reached(gripper_goal, self.tip):
            print('no solution found; stopping test')
            return 'mid'

        self.gripper.gripper_pivoting()

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position = Point(2.798, 1.849, 0.788)
        goal_pose.pose.orientation = Quaternion(0,0,0,1)
        self.giskard.set_cart_goal(self.with_base, tulip, goal_pose)
        if goal_angle is not None:
            self.giskard.set_joint_goal({'refills_finger_joint': goal_angle})
        self.keep_horizontal(tulip)
        if not self.giskard.plan_and_execute().error_code == MoveResult.SUCCESS or \
                not self.goal_reached(goal_pose, tulip):
            print('no solution found; stopping test')
            return 'end'
        self.gripper.home()
        return 'ok'


rospy.init_node('markos_demo')

plan = Plan()
failed_starts = set()
for start_angle, goal_angle in product(np.arange(-np.pi/2, np.pi/2, np.pi/4).tolist()+[np.pi/2, None], repeat=2):
    print('executing {} {}'.format(start_angle, goal_angle))
    if start_angle not in failed_starts:
        plan.giskard.clear_world()
        plan.start_config()
        result = plan.pick_up(start_angle, goal_angle)
        if result == 'start':
            failed_starts.add(start_angle)
        elif result == 'ok':
            print('success')

    else:
        print('no solution found; stopping test')
