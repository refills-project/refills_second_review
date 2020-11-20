from __future__ import division

import json
from copy import deepcopy

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Vector3Stamped, Vector3
from giskard_msgs.msg import CollisionEntry, MoveResult
from rosgraph_msgs.msg import Log
from rospy_message_converter.message_converter import convert_ros_message_to_dictionary
from sensor_msgs.msg import JointState
from tf.transformations import quaternion_about_axis

from giskardpy.python_interface import GiskardWrapper
from giskardpy.tfwrapper import lookup_pose, np_to_kdl, msg_to_kdl, kdl_to_quaternion
from giskardpy.utils import to_joint_state_position_dict
from refills_second_review.gripper import Gripper
from utils_for_tests import compare_poses


class PlanningTimeRecorder(object):
    def __init__(self):
        self.sub = rospy.Subscriber('rosout', Log, self.cb)
        self.reset()

    def reset(self):
        self.traj_length = 0
        self.planning_time = 0
        # self.compile_time = 0

    def cb(self, data):
        """
        :type data: Log
        """
        if data.name == '/giskard':
            if 'found goal traj' in data.msg:
                runtime, planning_time = data.msg.split(' in ')
                self.traj_length += float(runtime.split(' ')[-1][:-1])
                self.planning_time += float(planning_time[:-1])
            if 'jacobian' in data.msg:
                self.planning_time -= float(data.msg.split(' ')[-1])
            if 'autowrap' in data.msg:
                self.planning_time -= float(data.msg.split(' ')[-1])


def make_entry(start_angle, goal_angle, success, planning_time, execution_time, table_height, chosen_angle):
    return {
        'start_angle': start_angle,
        'goal_angle': goal_angle,
        'success': success,
        'planning_time': planning_time,
        'execution_time': execution_time,
        'table_height': table_height,
        'chosen_angle': chosen_angle
    }


class Plan(object):
    def __init__(self):
        self.load_database()
        self.translation_limit = 0.1
        self.rotation_limit = np.pi / 14
        self.without_base = 'base_footprint'
        self.with_base = 'odom'
        self.tip = 'refills_tool_frame'
        self.gripper = Gripper(True)
        self.giskard = GiskardWrapper()
        rospy.sleep(1)
        self.giskard.clear_world()

    def load_database(self):
        with open('./database.json', 'r') as f:
            self.db = json.load(f)

    def get_object_size(self, name):
        return [self.db[name]['depth'],
                self.db[name]['width'],
                self.db[name]['height']]

    def get_object_friction_coefficient(self, name):
        return self.db[name]['friction_coefficient']

    def get_object_grasp_height(self, name):
        return self.db[name]['grasp_height']

    def start_config(self):
        js = {
            'ur5_shoulder_pan_joint': 3.14151906967,
            'ur5_shoulder_lift_joint': -0.999989811574,
            'ur5_elbow_joint': 1.00003242493,
            'ur5_wrist_1_joint': -1.57066423098,
            'ur5_wrist_2_joint': -1.57076150576,
            'ur5_wrist_3_joint': -1.58478862444,
        }
        self.giskard.set_joint_goal(js, max_speed=self.rotation_limit)
        self.giskard.plan_and_execute()

    def move_base(self, x, y, rot):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.position.z = 0
        p.pose.orientation = Quaternion(*quaternion_about_axis(rot, [0, 0, 1]))
        self.giskard.set_cart_goal(self.with_base, self.without_base, p)
        self.giskard.plan_and_execute()

    def current_base_goal(self):
        base_goal = PoseStamped()
        base_goal.header.frame_id = self.with_base
        base_goal.pose.orientation.w = 1
        self.giskard.set_cart_goal(self.without_base, self.with_base, base_goal)

    def keep_horizontal(self, tip, normal=(0, 1, 0)):
        root_normal = Vector3Stamped()
        root_normal.header.frame_id = 'map'
        root_normal.vector.z = 1

        tip_normal = Vector3Stamped()
        tip_normal.header.frame_id = 'refills_finger'
        tip_normal.vector = Vector3(*normal)
        self.giskard.align_planes(tip, tip_normal, self.without_base, root_normal)

    def add_table(self, table_name='table', height=0.72):
        box_pose = PoseStamped()
        box_pose.header.frame_id = 'map'
        box_pose.pose.position = Point(2.798, 2.768, height / 2)
        box_pose.pose.orientation.w = 1
        self.giskard.add_box(table_name, [2.05, 2.05, height], pose=box_pose)

    def add_shelf(self):
        back = 'back'
        back_p = PoseStamped()
        back_p.header.frame_id = 'map'
        back_p.pose.position.x = 4.6
        back_p.pose.position.y = -0.2
        back_p.pose.position.z = 1
        back_p.pose.orientation = Quaternion(*quaternion_about_axis(np.pi / 2, [0, 0, 1]))
        self.giskard.add_box(back, size=[1, 0.05, 2], pose=back_p)

        p = deepcopy(back_p)
        for i, height in enumerate([0.2, 0.6, 0.93, 1.31, 1.56]):
            if i == 0:
                layer1 = 'layer{}'.format(i)
                p.pose.position.x = 4.225
                p.pose.position.z = height - 0.01125
                self.giskard.add_box(layer1, size=[1, 0.75, 0.0675], pose=p)
            else:
                layer1 = 'layer{}'.format(i)
                p.pose.position.x = 4.275
                p.pose.position.z = height - 0.01125
                self.giskard.add_box(layer1, size=[1, 0.65, 0.0675], pose=p)
        return back_p

    def spawn_object_on_floor(self, name='tulip'):
        d, w, h = self.get_object_size(name)
        box_pose = PoseStamped()
        box_pose.header.frame_id = 'map'
        box_pose.pose.position = Point(2.8, 1.1, 0.00)
        box_pose.pose.orientation.w = 1
        box_pose.pose.position.z += h / 2  # box height
        self.giskard.add_box(name, [d, w, h], pose=box_pose)
        return box_pose

    def get_joint_state(self):
        js = rospy.wait_for_message('/joint_states', JointState)
        return to_joint_state_position_dict(js)

    def goal_reached(self, goal, tip, angle):
        current_pose = lookup_pose(goal.header.frame_id, tip)
        js = rospy.wait_for_message('/refills_finger/joint_states', JointState).position[0]
        try:
            compare_poses(current_pose.pose, goal.pose, decimal=1)
        except AssertionError:
            return False
        if angle is None or abs(js - angle) < 0.02:
            return True
        else:
            print('grasp angle {} expected {}'.format(js, angle))

    def allow_base_y_movement(self, rot):
        p = lookup_pose(self.without_base, self.with_base)
        self.giskard.set_json_goal('CartesianPositionY', **{'root_link': self.without_base,
                                                            'tip_link': self.with_base,
                                                            'goal': convert_ros_message_to_dictionary(p),
                                                            'max_speed': self.translation_limit
                                                            })
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.pose.orientation = Quaternion(*quaternion_about_axis(rot, [0, 0, 1]))
        self.giskard.set_rotation_goal(self.with_base, self.without_base, p, max_speed=self.rotation_limit)

    def pick_up(self, start_angle=0, goal_angle=0, table_height=0.72):
        if start_angle is not None:
            start_angle -= np.pi / 2
        if goal_angle is not None:
            goal_angle -= np.pi / 2
        tulip = 'tulip'

        # reset
        for i in range(2):
            self.gripper.home()
            self.add_table(height=table_height)
            # self.add_shelf()
            if i == 0:
                self.move_base(3.1, 0.2, -1.57)
                object_pose = self.spawn_object_on_floor()
            self.start_config()
            if i == 1:
                planning_time_recorder.reset()

            # grasp tulip
            arm_goal = deepcopy(object_pose)
            o_R_g = np_to_kdl(np.array([[1, 0, 0, 0],
                                        [0, 0, -1, 0],
                                        [0, 1, 0, 0],
                                        [0, 0, 0, 1]]))
            m_R_o = msg_to_kdl(arm_goal)
            m_R_g = m_R_o.M * o_R_g.M
            arm_goal.pose.orientation = kdl_to_quaternion(m_R_g)
            arm_goal.pose.position.z = 0.06
            self.giskard.set_translation_goal(self.with_base, 'refills_finger', arm_goal, max_speed=self.translation_limit)
            self.giskard.set_rotation_goal(self.with_base, 'refills_finger', arm_goal, max_speed=self.rotation_limit)
            self.allow_base_y_movement(-np.pi/2)
            self.giskard.allow_collision([CollisionEntry.ALL], tulip, [CollisionEntry.ALL])
            if start_angle is not None:
                self.giskard.set_joint_goal({'refills_finger_joint': start_angle}, max_speed=self.rotation_limit)
            if not self.giskard.plan_and_execute().error_code == MoveResult.SUCCESS or \
                    not self.goal_reached(arm_goal, 'refills_finger', start_angle):
                print('no solution found; stopping test')
                return 'start'

            if i == 0:
                if self.gripper.active:
                    raw_input('press a key')
            self.gripper.grasp(5)
            if i == 0:
                if self.gripper.active:
                    raw_input('press a key')
        self.giskard.attach_object(tulip, 'refills_finger')

        # place object
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position = Point(2.798, 1.849, table_height + 0.07)
        goal_pose.pose.orientation = Quaternion(0, 0, 0, 1)
        self.giskard.set_translation_goal(self.with_base, tulip, goal_pose, max_speed=self.translation_limit)
        self.giskard.set_rotation_goal(self.with_base, tulip, goal_pose, max_speed=self.rotation_limit)
        self.allow_base_y_movement(-np.pi/2)
        if goal_angle is not None:
            self.giskard.set_joint_goal({'refills_finger_joint': goal_angle})
        self.keep_horizontal(tulip)
        if not self.giskard.plan_and_execute().error_code == MoveResult.SUCCESS or \
                not self.goal_reached(goal_pose, tulip, goal_angle):
            print('no solution found; stopping test')
            return 'end'
        self.gripper.home()
        return 'ok'

    def pick_up_shelf(self, start_angle=0, goal_angle=0, height=0.72):
        if start_angle is not None:
            start_angle -= np.pi / 2
        if goal_angle is not None:
            goal_angle -= np.pi / 2

        # reset
        for i in range(2):
            self.gripper.home()
            if i == 0:
                # self.add_table(height=height)
                base_pose = self.add_shelf()
                base_pose.pose.position.x -= 2.2
                base_pose.pose.position.y -= 0.3
                base_pose.pose.position.z = 0
                base_pose.pose.orientation = Quaternion(*quaternion_about_axis(np.pi, [0, 0, 1]))
                self.giskard.set_cart_goal(self.with_base, self.without_base, base_pose, trans_max_speed=0.2)
                self.giskard.allow_all_collisions()
                self.giskard.plan_and_execute()

            if i == 0:
                # spawn object
                if height == 0.2:
                    object_name = 'vinegar'
                elif height == 0.6:
                    object_name = 'geschirr'
                elif height == 0.93:
                    object_name = 'shampoo'
                else:
                    object_name = 'deo'
                d, w, h = self.get_object_size(object_name)
                base_pose.pose.position.x += 1
                base_pose.pose.position.y = -0.195
                base_pose.pose.orientation = Quaternion(*quaternion_about_axis(-np.pi / 2, [0, 0, 1]))
                base_pose.pose.position.z = h / 2  # box height
                self.giskard.add_box(object_name, [d, w, h], pose=base_pose)
                print('spawned {}'.format(object_name))

            if i == 1:
                self.giskard.allow_all_collisions()
            self.start_config()
            if i == 1:
                planning_time_recorder.reset()

            # grasp tulip
            arm_goal = deepcopy(base_pose)
            o_R_g = np_to_kdl(np.array([[1, 0, 0, 0],
                                        [0, 0, -1, 0],
                                        [0, 1, 0, 0],
                                        [0, 0, 0, 1]]))
            m_R_o = msg_to_kdl(arm_goal)
            m_R_g = m_R_o.M * o_R_g.M
            arm_goal.pose.orientation = kdl_to_quaternion(m_R_g)
            arm_goal.pose.position.z = self.get_object_grasp_height(object_name)
            self.giskard.set_translation_goal(self.with_base, 'refills_finger', arm_goal,
                                              max_speed=self.translation_limit)
            self.giskard.set_rotation_goal(self.with_base, 'refills_finger', arm_goal, max_speed=self.rotation_limit)
            self.allow_base_y_movement(np.pi)
            self.giskard.allow_collision([CollisionEntry.ALL], object_name, [CollisionEntry.ALL])
            if start_angle is not None:
                self.giskard.set_joint_goal({'refills_finger_joint': start_angle}, max_speed=self.rotation_limit)
            if not self.giskard.plan_and_execute().error_code == MoveResult.SUCCESS or \
                    not self.goal_reached(arm_goal, 'refills_finger', start_angle):
                print('no solution found; stopping test')
                return 'start'

            if self.gripper.active:
                if i == 0:
                    raw_input('press a key')
                self.gripper.change_params(self.get_object_friction_coefficient(object_name))
                self.gripper.grasp(5)
                if i == 0:
                    raw_input('press a key')
            if i == 0:
                self.gripper.home()
        self.giskard.attach_object(object_name, 'refills_finger')

        # place object
        goal1 = PoseStamped()
        goal1.header.frame_id = 'map'
        goal1.pose.position.x = 3.7
        goal1.pose.position.y = -0.195
        goal1.pose.position.z = height + self.get_object_grasp_height(object_name) + 0.01
        goal1.pose.orientation = deepcopy(base_pose.pose.orientation)
        self.giskard.set_translation_goal(self.with_base, object_name, goal1, max_speed=self.translation_limit)
        self.giskard.set_rotation_goal(self.with_base, object_name, goal1, max_speed=self.rotation_limit)
        self.allow_base_y_movement(np.pi)
        self.giskard.allow_collision([object_name], 'iai_donbot', [CollisionEntry.ALL])
        self.keep_horizontal(object_name)

        self.giskard.add_cmd()
        goal2 = deepcopy(goal1)
        goal2.pose.position.x += 0.35
        goal2.pose.position.z -= 0.01
        self.giskard.set_translation_goal(self.with_base, object_name, goal2, max_speed=self.translation_limit)
        self.giskard.set_rotation_goal(self.with_base, object_name, goal2, max_speed=self.rotation_limit)
        self.giskard.allow_collision([object_name], 'iai_donbot', [CollisionEntry.ALL])
        self.allow_base_y_movement(np.pi)
        self.keep_horizontal(object_name)

        if goal_angle is not None:
            self.giskard.set_joint_goal({'refills_finger_joint': goal_angle})
        self.keep_horizontal(object_name)
        if not self.giskard.plan_and_execute().error_code == MoveResult.SUCCESS or \
                not self.goal_reached(goal2, object_name, goal_angle):
            print('no solution found; stopping test')
            return 'end'
        # self.gripper.grasp(0)
        self.gripper.home()
        self.giskard.detach_object(object_name)
        # self.giskard.remove_object(object_name)
        return 'ok'


def pick_and_place_table():
    data = []

    plan = Plan()
    failed_starts = set()
    start_angles = np.arange(-np.pi / 2, np.pi / 2, np.pi / 4).tolist() + [np.pi / 2, None]
    goal_angles = np.arange(-np.pi / 2, np.pi / 2, np.pi / 4).tolist() + [np.pi / 2, None]
    # table_heights = [0.2, 0.6, 0.72, 0.93, 1.31]
    table_heights = [0.72]

    # combinations = [[-np.pi/4, ]]

    # with open('log_video4.json', 'r') as f:
    #     matrix = json.load(f)

    skip = False
    # for start_angle, goal_angle, table_height in product(start_angles, goal_angles, table_heights):
    for start_angle, goal_angle, table_height in [(-np.pi/4, -np.pi/4, 0.72),
                                                  (0, -np.pi/2, 0.72),
                                                  (None, None, 0.72)]:
    # for entry in matrix:
    #     start_angle = entry['start_angle']
    #     goal_angle = entry['goal_angle']
        # table_height = entry['table_height']
        print('executing {} {} {}'.format(start_angle, goal_angle, table_height))
        if start_angle == np.pi / 4 and goal_angle == 0.0:
            skip = False
        if skip:
            continue
        planning_time_recorder.reset()
        result = ''
        if start_angle not in failed_starts:
            plan.giskard.clear_world()
            result = plan.pick_up(start_angle, goal_angle, table_height)
            if result == 'start':
                failed_starts.add(start_angle)
        js = plan.get_joint_state()['refills_finger_joint'] + np.pi / 2
        entry = make_entry(start_angle, goal_angle, result == 'ok', planning_time_recorder.planning_time,
                           planning_time_recorder.traj_length, table_height, js)

        print(entry)
        print('-----------------------------')
        data.append(entry)
        with open('log_real4.json', 'w') as f:
            json.dump(data, f, indent=4, sort_keys=True)


def pick_and_place_shelf():
    data = []

    plan = Plan()
    failed_starts = set()
    start_angles = np.arange(-np.pi / 4, np.pi / 4, np.pi / 4).tolist() + [np.pi / 2, None]
    goal_angles = np.arange(-np.pi / 2, 0, np.pi / 4).tolist() + [np.pi / 2, None]
    table_heights = [0.2, 0.6, 0.93, 1.31]
    # table_heights = [0.72]

    skip = False
    # for start_angle, goal_angle, height in product(start_angles, goal_angles, table_heights):
    plan.giskard.clear_world()
    for start_angle, goal_angle, height in [
        (-np.pi/8, None, 1.31),
        # (None, None, 0.93),
        # (None, None, 0.6),
        # (None, None, 0.2),
    ]:
        print('executing {} {} {}'.format(start_angle, goal_angle, height))
        if start_angle == np.pi / 4 and goal_angle == 0.0:
            skip = False
        if skip:
            continue
        planning_time_recorder.reset()
        result = ''
        if start_angle not in failed_starts:
            result = plan.pick_up_shelf(start_angle, goal_angle, height)
            if result == 'start':
                failed_starts.add(start_angle)
        js = plan.get_joint_state()['refills_finger_joint'] + np.pi / 2
        entry = make_entry(start_angle, goal_angle, result == 'ok', planning_time_recorder.planning_time,
                           planning_time_recorder.traj_length, height, js)

        print(entry)
        print('-----------------------------')
        data.append(entry)
        with open('log_shelf2.json', 'w') as f:
            json.dump(data, f, indent=4, sort_keys=True)


rospy.init_node('markos_demo')
planning_time_recorder = PlanningTimeRecorder()
pick_and_place_shelf()
# pick_and_place_table()
