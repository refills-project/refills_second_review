import json
import numpy as np
from itertools import product

import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Vector3Stamped, Vector3
from giskard_msgs.msg import CollisionEntry, MoveResult
from rosgraph_msgs.msg import Log
from rospy_message_converter.message_converter import convert_ros_message_to_dictionary
from sensor_msgs.msg import JointState
from tf.transformations import quaternion_about_axis

from giskardpy.python_interface import GiskardWrapper
from giskardpy.tfwrapper import lookup_pose, np_to_kdl, msg_to_kdl, kdl_to_quaternion, kdl_to_pose
from giskardpy.utils import to_joint_state_dict2
from refills_second_review.gripper import Gripper
from utils_for_tests import compare_poses


class Plan(object):
    def __init__(self):
        self.translation_limit = 0.1
        self.rotation_limit = np.pi / 14
        self.without_base = 'base_footprint'
        self.with_base = 'odom'
        self.tip = 'refills_tool_frame'
        self.gripper = Gripper(True)
        self.giskard = GiskardWrapper()
        rospy.sleep(1)
        self.giskard.clear_world()

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
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.pose.position.x = 4
        p.pose.position.y = -1.5
        p.pose.position.z = 1
        p.pose.orientation = Quaternion(*quaternion_about_axis(np.pi / 2, [0, 0, 1]))
        self.giskard.add_box(back, size=[1, 0.05, 2], pose=p)

        for i, height in enumerate([0.2, 0.6, 0.93, 1.31]):
            layer1 = 'layer{}'.format(i)
            p.pose.position.x = 3.75
            p.pose.position.z = height
            self.giskard.add_box(layer1, size=[1, 0.5, 0.02], pose=p)

    def add_tulip_on_floor(self, name='tulip'):
        box_pose = PoseStamped()
        box_pose.header.frame_id = 'map'
        box_pose.pose.position = Point(2.8, 1.1, 0.00)
        box_pose.pose.orientation.w = 1
        box_pose.pose.position.z += 0.045  # box height
        self.giskard.add_box(name, [0.05, 0.095, 0.09], pose=box_pose)
        return box_pose

    def get_joint_state(self):
        js = rospy.wait_for_message('/joint_states', JointState)
        return to_joint_state_dict2(js)

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

    def allow_base_y_movement(self, y, rot):
        p = lookup_pose(self.without_base, self.with_base)
        self.giskard.set_json_goal('CartesianPositionY', **{'root_link': self.without_base,
                                                            'tip_link': self.with_base,
                                                            'goal': convert_ros_message_to_dictionary(p),
                                                            'max_speed': self.translation_limit
                                                            })
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.pose.orientation = Quaternion(*quaternion_about_axis(rot, [0,0,1]))
        self.giskard.set_rotation_goal(self.with_base, self.without_base, p, max_speed=self.rotation_limit)

    def pick_up(self, start_angle=0, goal_angle=0, table_height=0.72):
        if start_angle is not None:
            start_angle -= np.pi / 2
        if goal_angle is not None:
            goal_angle -= np.pi / 2
        tulip = 'tulip'

        # reset
        self.gripper.home()
        self.add_table(height=table_height)
        # self.add_shelf()
        self.move_base(3.1, 0.2, -1.57)
        arm_goal = self.add_tulip_on_floor()
        self.start_config()
        planning_time_recorder.reset()

        # grasp tulip
        o_R_g = np_to_kdl(np.array([[1, 0, 0, 0],
                                    [0, 0, -1, 0],
                                    [0, 1, 0, 0],
                                    [0, 0, 0, 1]]))
        m_R_o = msg_to_kdl(arm_goal)
        m_R_g = m_R_o.M * o_R_g.M
        arm_goal.pose.orientation = kdl_to_quaternion(m_R_g)
        arm_goal.pose.position.z = 0.07
        self.giskard.set_translation_goal(self.with_base, 'refills_finger', arm_goal, max_speed=self.translation_limit)
        self.giskard.set_rotation_goal(self.with_base, 'refills_finger', arm_goal, max_speed=self.rotation_limit)
        # js = self.get_joint_state()
        # js = {k:v for k,v in js.items() if k in ['odom_']}
        self.allow_base_y_movement(3.1, -1.57)
        self.giskard.allow_collision([CollisionEntry.ALL], tulip, [CollisionEntry.ALL])
        if start_angle is not None:
            self.giskard.set_joint_goal({'refills_finger_joint': start_angle}, max_speed=self.rotation_limit)
        if not self.giskard.plan_and_execute().error_code == MoveResult.SUCCESS or \
                not self.goal_reached(arm_goal, 'refills_finger', start_angle):
            print('no solution found; stopping test')
            return 'start'

        raw_input('press a key')
        self.gripper.grasp(5)
        raw_input('press a key')
        self.giskard.attach_object(tulip, 'refills_finger')

        # place object
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position = Point(2.798, 1.849, table_height + 0.07)
        goal_pose.pose.orientation = Quaternion(0, 0, 0, 1)
        self.giskard.set_translation_goal(self.with_base, tulip, goal_pose, max_speed=self.translation_limit)
        self.giskard.set_rotation_goal(self.with_base, tulip, goal_pose, max_speed=self.rotation_limit)
        self.allow_base_y_movement(3.1, -1.57)
        # self.giskard.allow_collision(['gripper_base_link'], 'table', [CollisionEntry.ALL])
        if goal_angle is not None:
            self.giskard.set_joint_goal({'refills_finger_joint': goal_angle})
        self.keep_horizontal(tulip)
        if not self.giskard.plan_and_execute().error_code == MoveResult.SUCCESS or \
                not self.goal_reached(goal_pose, tulip, goal_angle):
            print('no solution found; stopping test')
            return 'end'
        self.gripper.home()
        return 'ok'


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


rospy.init_node('markos_demo')

data = []

plan = Plan()
failed_starts = set()
planning_time_recorder = PlanningTimeRecorder()
start_angles = np.arange(-np.pi / 2, np.pi / 2, np.pi / 4).tolist() + [np.pi / 2, None]
goal_angles = np.arange(-np.pi / 2, np.pi / 2, np.pi / 4).tolist() + [np.pi / 2, None]
# table_heights = [0.2, 0.6, 0.72, 0.93, 1.31]
table_heights = [0.72]

# combinations = [[-np.pi/4, ]]

with open('log.json2', 'r') as f:
    matrix = json.load(f)

skip = True
# for start_angle, goal_angle, table_height in product(start_angles, goal_angles, table_heights):
# for start_angle, goal_angle, table_height in [(-np.pi/4, np.pi/4, 0.72)]:
for entry in matrix:
    start_angle = entry['start_angle']
    goal_angle = entry['goal_angle']
    if entry['success'] and (goal_angle is None or not np.isclose(np.array(goal_angle), np.array(np.pi/4))):
        table_height = entry['table_height']
        print('executing {} {} {}'.format(start_angle, goal_angle, table_height))
        if start_angle == np.pi/4 and goal_angle == 0.0:
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
        js = plan.get_joint_state()['refills_finger_joint'] + np.pi/2
        entry = make_entry(start_angle, goal_angle, result == 'ok', planning_time_recorder.planning_time,
                           planning_time_recorder.traj_length, table_height, js)

        print(entry)
        print('-----------------------------')
        data.append(entry)
        with open('log_real4.json', 'w') as f:
            json.dump(data, f, indent=4, sort_keys=True)
