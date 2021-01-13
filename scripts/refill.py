#!/usr/bin/env python
import numpy as np
import PyKDL as kdl
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion, Point, Vector3Stamped
from iai_wsg_50_msgs.msg import PositionCmd, Status
from tf.transformations import quaternion_from_matrix, quaternion_about_axis

from giskardpy.constraints import WEIGHT_ABOVE_CA
from giskardpy.python_interface import GiskardWrapper
from giskardpy.tfwrapper import msg_to_kdl, kdl_to_pose, pose_to_kdl
from giskardpy.utils import msg_to_list, position_dist
from knowrob_refills.knowrob_wrapper import KnowRob
from refills_perception_interface.move_arm import MoveArm
from refills_perception_interface.move_base import MoveBase

# TODO not below 82 during pick up
# TODO table height = 72
from refills_perception_interface.move_gripper import MoveGripper
from refills_perception_interface.robosherlock_wrapper import RoboSherlock, FakeRoboSherlock
from refills_perception_interface.separator_detection import SeparatorClustering
from refills_perception_interface.tfwrapper import lookup_pose, transform_pose
from refills_second_review.gripper import Gripper, GripperWSG50

TABLE_HEIGHT = 0.81


class CRAM(object):
    def __init__(self):
        self.default_object_name = 'box'
        self.refills_finger = u'refills_finger'
        self.camera_link = u'camera_link'
        # self.kr = KnowRob()
        # self.separator_detector = SeparatorClustering(self.kr)
        self.gripper = GripperWSG50()
        self.gripper.home()
        try:
            # self.robo_sherlock = RoboSherlock(self.kr, name='RoboSherlock_scenario3', check_camera=False)
            self.robo_sherlock = RoboSherlock(None, check_camera=False)
        except:
            rospy.logwarn('couldn\'t connect to robo sherlock; using fake mode')
        # self.arm = MoveArm(tip='gripper_tool_frame', root='base_footprint')
        # self.arm.giskard.clear_world()
        self.giskard = GiskardWrapper()
        self.giskard.clear_world()
        self.goal_layer_height = 1.3
        self.width = 0.145
        self.depth = 0.05
        self.height = 0.115

    def goto_see_pose(self):
        js = {
            'iiwa_joint_1': 0.87564548888,
            'iiwa_joint_2': -1.18868177242,
            'iiwa_joint_3': -0.611353182624,
            'iiwa_joint_4': 1.53584067968,
            'iiwa_joint_5': 1.81510903672,
            # 'iiwa_joint_6': -1.63964857509,
            # 'iiwa_joint_7': -0.16138790158,
        }
        self.giskard.set_joint_goal(js)
        self.giskard.plan_and_execute()
        js = {
            # 'iiwa_joint_1': 0.87564548888,
            # 'iiwa_joint_2': -1.18868177242,
            # 'iiwa_joint_3': -0.611353182624,
            # 'iiwa_joint_4': 1.53584067968,
            # 'iiwa_joint_5': 1.81510903672,
            'iiwa_joint_6': -1.63964857509,
            'iiwa_joint_7': -0.16138790158,
        }
        self.giskard.set_joint_goal(js)
        self.giskard.plan_and_execute()

    def goto_carry_pose(self):
        js = {
            'iiwa_joint_1': 0,
            'iiwa_joint_2': -1.28,
            'iiwa_joint_3': 0,
            'iiwa_joint_4': 1.29,
            'iiwa_joint_5': 0,
            'iiwa_joint_6': -1.0,
            'iiwa_joint_7': 1.57,
        }
        self.giskard.set_joint_goal(js)
        self.giskard.plan_and_execute()

    # def goto_pregrasp_pose(self):
    #     js = {
    #         'iiwa_joint_1': 0.8,
    #         'iiwa_joint_2': -1.26293805785,
    #         'iiwa_joint_3': -0.434284463224,
    #         'iiwa_joint_4': 1.74242120759,
    #         'iiwa_joint_5': 2.00150718678,
    #         'iiwa_joint_6': -1.67087526211,
    #         'iiwa_joint_7': -0.484213467998,
    #     }
    #     self.giskard.set_joint_goal(js)
    #     self.giskard.plan_and_execute()

    def sort(self, facing_id):
        self.gripper.home()
        self.add_top_layer()
        self.goto_carry_pose()
        self.goto_see_pose()
        # rospy.sleep(3)
        detected_object = self.detect_object()
        self.grasp_object(detected_object)
        self.goto_carry_pose()
        self.place_on_back(0.145)
        # drive
        self.pick_from_back()
        self.preplace_pose()
        self.place_on_shelf()

    def grasp_object(self, detected_object):
        self.goto_pregrasp_pose()
        self.goto_grasp_pose()
        self.giskard.attach_object('DenkMitGeschirrReinigerNature', self.refills_finger)
        self.gripper.send_cmd(0.02)
        # self.gripper.grasp()
        # self.gripper.gripper_pivoting()
        self.lift_object()
        self.move_back()

    def pick_from_back(self):
        box_goal = PoseStamped()
        box_goal.header.frame_id = self.refills_finger
        box_goal.pose.position.y = 0.3
        box_goal.pose.orientation.w = 1

        self.giskard.set_cart_goal(goal_pose=box_goal,
                                   tip_link=self.refills_finger,
                                   root_link=u'base_link')
        self.giskard.plan_and_execute()

    def preplace_pose(self):
        box_goal = PoseStamped()
        box_goal.header.frame_id = self.refills_finger
        box_goal.pose.position.x = 0.2
        box_goal.pose.orientation.w = 1

        self.giskard.set_cart_goal(goal_pose=box_goal,
                                   tip_link=self.refills_finger,
                                   root_link=u'base_link')
        self.giskard.plan_and_execute()

        box_goal = PoseStamped()
        box_goal.header.frame_id = self.refills_finger
        box_goal.pose.position.x = 0.2
        box_goal.pose.position.z = -0.4
        box_goal.pose.orientation = Quaternion(*quaternion_from_matrix([[0, 0, 1, 0],
                                                                        [0, 1, 0, 0],
                                                                        [-1, 0, 0, 0],
                                                                        [0, 0, 0, 1]]))
        self.giskard.set_cart_goal(goal_pose=box_goal,
                                   tip_link=self.refills_finger,
                                   root_link=u'base_link')
        self.keep_horizontal()
        self.giskard.plan_and_execute()

    def lift_object(self):
        lift_pose = PoseStamped()
        lift_pose.header.frame_id = self.refills_finger
        lift_pose.pose.position.y = 0.04
        lift_pose.pose.orientation.w = 1
        self.giskard.set_cart_goal(lift_pose,
                                   tip_link=self.refills_finger,
                                   root_link='base_link')
        self.giskard.allow_all_collisions()
        self.keep_horizontal()
        self.giskard.plan_and_execute()

    def move_back(self):
        lift_pose = PoseStamped()
        lift_pose.header.frame_id = self.refills_finger
        lift_pose.pose.position.z = -0.3
        lift_pose.pose.orientation.w = 1
        self.giskard.set_cart_goal(lift_pose,
                                   tip_link=self.refills_finger,
                                   root_link='odom')
        self.giskard.allow_all_collisions()
        self.keep_horizontal()
        self.giskard.plan_and_execute()

    def place_on_back(self, object_height):
        box_goal = PoseStamped()
        box_goal.header.frame_id = u'angle_adapter_base'
        box_goal.pose.position.z = 0.145 / 2 + 0.01
        box_goal.pose.position.y = 0.35
        box_goal.pose.orientation = Quaternion(*quaternion_from_matrix([[-1, 0, 0, 0],
                                                                        [0, 0, 1, 0],
                                                                        [0, 1, 0, 0],
                                                                        [0, 0, 0, 1]]))
        self.giskard.set_cart_goal(goal_pose=box_goal,
                                   tip_link=self.refills_finger,
                                   root_link=u'base_link')
        self.giskard.plan_and_execute()

    def goto_pregrasp_pose(self):
        pre_grasp_pose = PoseStamped()
        pre_grasp_pose.header.frame_id = u'DenkMitGeschirrReinigerNature'
        pre_grasp_pose.pose.position.x = 0.25
        pre_grasp_pose.pose.orientation = Quaternion(*quaternion_from_matrix([[0, 0, -1, 0],
                                                                              [1, 0, 0, 0],
                                                                              [0, -1, 0, 0],
                                                                              [0, 0, 0, 1]]))
        self.giskard.set_cart_goal(goal_pose=pre_grasp_pose,
                                   tip_link=self.camera_link,
                                   root_link='odom')
        self.keep_horizontal()
        self.giskard.limit_cartesian_velocity('odom', 'base_link', max_linear_velocity=0.01, max_angular_velocity=0.01)
        print(self.giskard.plan_and_execute().error_messages)

    def place_on_shelf(self):
        # 3.888, -0.506, 0.978
        # shelf height is 0.92
        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = u'map'
        grasp_pose.pose.position.x = 4.08
        grasp_pose.pose.position.y = -0.51
        grasp_pose.pose.position.z = 1.08
        grasp_pose.pose.orientation = Quaternion(*quaternion_from_matrix([[0, 0, 1, 0],
                                                                          [1, 0, 0, 0],
                                                                          [0, 1, 0, 0],
                                                                          [0, 0, 0, 1]]))
        self.giskard.set_cart_goal(goal_pose=grasp_pose,
                                   tip_link=self.refills_finger,
                                   root_link='odom')
        self.keep_horizontal()
        self.giskard.limit_cartesian_velocity('odom', 'base_link', max_linear_velocity=0.1, max_angular_velocity=0.1)
        cam_goal = PoseStamped()
        cam_goal.header.frame_id = 'map'
        cam_goal.pose.orientation = Quaternion(*quaternion_from_matrix([[0, 0, 1, 0],
                                                                        [-1, 0, 0, 0],
                                                                        [0, -1, 0, 0],
                                                                        [0, 0, 0, 1]]))
        self.giskard.set_rotation_goal(cam_goal, 'camera_link', 'base_link')
        self.giskard.plan_and_execute()
        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = self.refills_finger
        grasp_pose.pose.position.z = -0.4
        grasp_pose.pose.orientation.w = 1
        self.giskard.set_cart_goal(goal_pose=grasp_pose,
                                   tip_link=self.refills_finger,
                                   root_link='base_link')
        self.giskard.plan_and_execute()
        self.goto_carry_pose()


    def goto_grasp_pose(self):
        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = u'DenkMitGeschirrReinigerNature'
        grasp_pose.pose.position.x = 0.1
        grasp_pose.pose.position.z = - 0.02
        grasp_pose.pose.orientation = Quaternion(*quaternion_from_matrix([[0, 0, -1, 0],
                                                                          [1, 0, 0, 0],
                                                                          [0, -1, 0, 0],
                                                                          [0, 0, 0, 1]]))
        self.giskard.set_cart_goal(goal_pose=grasp_pose,
                                   tip_link='camera_link',
                                   root_link='odom')
        # self.keep_horizontal()
        # self.giskard.allow_all_collisions()
        self.giskard.limit_cartesian_velocity('odom', 'base_link', max_linear_velocity=0.01, max_angular_velocity=0.01)
        print(self.giskard.plan_and_execute().error_messages)

    def keep_horizontal(self):
        tip_normal = Vector3Stamped()
        tip_normal.header.frame_id = self.refills_finger
        tip_normal.vector.y = 1

        root_normal = Vector3Stamped()
        root_normal.header.frame_id = 'map'
        root_normal.vector.z = 1
        self.giskard.align_planes(tip_link=self.refills_finger,
                                  tip_normal=tip_normal,
                                  root_link=u'base_link',
                                  root_normal=root_normal,
                                  weight=WEIGHT_ABOVE_CA)

    def detect_object(self):
        object_name = 'DenkMitGeschirrReinigerNature'
        detected_object = self.robo_sherlock.see(self.depth, self.width, self.height, object_name)
        self.giskard.add_box(object_name,
                             size=[detected_object.width, detected_object.height, detected_object.depth],
                             pose=detected_object.map_T_object)
        return detected_object

    def add_top_layer(self, height=1.3):
        self.giskard.remove_object('layer')
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.pose.position.x = 4.3
        p.pose.position.y = -0.5
        p.pose.position.z = height
        self.giskard.add_box('layer',
                             size=[0.6, 1, 0.05],
                             pose=p)

    def spawn_table_in_giskard(self):
        p = lookup_pose('map', 'TableFooBar')
        # self.arm.giskard.add_mesh('table', 'package://iai_kitchen/meshes/misc/big_table_1.stl', pose=p)

    def facing_placing_pose(self, facing_id, object_height):
        facing_pose = PoseStamped()
        facing_pose.header.frame_id = self.kr.get_object_frame_id(facing_id)
        facing_depth = self.kr.get_facing_depth(facing_id)
        facing_height = self.kr.get_facing_height(facing_id)
        facing_pose.pose.position.y = -facing_depth / 2. - 0.1
        facing_pose.pose.position.z = -facing_height / 2. + object_height / 2 + .03
        facing_pose.pose.position.x = 0.03  # TODO magic
        facing_pose.pose.orientation = Quaternion(0, 0, 0, 1)
        # facing_pose.pose.orientation = Quaternion(*quaternion_from_matrix(np.array([[1, 0, 0, 0],
        #                                                                             [0, 0, 1, 0],
        #                                                                             [0, -1, 0, 0],
        #                                                                             [0, 0, 0, 1]])))
        return facing_pose

    def refill(self):
        # self.spawn_table_in_giskard()
        # self.arm.drive_pose()
        self.robo_sherlock.set_ring_light(0)
        for shelf_system_id in self.kr.get_shelf_system_ids():
            for shelf_layer_id in self.kr.get_shelf_layer_from_system(shelf_system_id):
                empty_facings = self.kr.get_empty_facings_from_layer(shelf_layer_id)
                shelf_layer_frame_id = self.kr.get_perceived_frame_id(shelf_layer_id)
                for i, facing_id in enumerate(sorted(empty_facings,
                                                     key=lambda x: lookup_pose(shelf_layer_frame_id,
                                                                               self.kr.get_object_frame_id(
                                                                                   x)).pose.position.x)):
                    for j in range(1 if i <= 1 else 1):
                        facing_frame_id = self.kr.get_object_frame_id(facing_id)
                        if lookup_pose('map', facing_frame_id).pose.position.z > 0.6:
                            object_class = self.kr.get_object_of_facing(facing_id)
                            depth, width, height = self.kr.get_object_dimensions(object_class)
                            # self.spawn_fake_object(width, depth, height)
                            self.pickup_object(depth, width, height)
                            self.place_object(facing_id, shelf_layer_id, shelf_system_id, height)

    def calc_x_offset(self, shelf_layer_id, facing_id, threshold=0.04):
        facing_frame_id = self.kr.get_object_frame_id(facing_id)
        old_sep1, old_sep2 = self.kr.get_facing_separator(facing_id)
        old_sep1 = lookup_pose(facing_frame_id, self.kr.get_perceived_frame_id(old_sep1))
        old_sep2 = lookup_pose(facing_frame_id, self.kr.get_perceived_frame_id(old_sep2))
        self.separator_detector.start_listening_separators(shelf_layer_id)
        rospy.sleep(3)

        separator = self.separator_detector.stop_listening()
        separator = [transform_pose(facing_frame_id, x) for x in separator]

        new_sep1 = min(separator, key=lambda x: position_dist(x.pose.position, old_sep1.pose.position))
        new_sep2 = min(separator, key=lambda x: position_dist(x.pose.position, old_sep2.pose.position))

        if position_dist(new_sep1.pose.position, old_sep1.pose.position) > threshold:
            new_sep1 = old_sep1
        if position_dist(new_sep2.pose.position, old_sep2.pose.position) > threshold:
            new_sep2 = old_sep2
        if position_dist(new_sep1.pose.position, new_sep2.pose.position) > \
                position_dist(old_sep1.pose.position, old_sep2.pose.position) + 0.02:
            return 0

        new_mid = new_sep1.pose.position.x + (new_sep2.pose.position.x - new_sep1.pose.position.x) / 2
        return new_mid

    def pickup_object(self, depth, width, height, table_height=.72):
        self.gripper.open()
        self.goto_see_pose()
        pose = self.robo_sherlock.see(depth, width, height)
        pose.pose.position.z = table_height + height / 2
        self.spawn_fake_object(depth, width, height, pose)

        T_map__object = msg_to_kdl(pose)

        # position base
        base_goal = PoseStamped()
        base_goal.header.frame_id = 'map'
        base_goal.pose.position = Point(0, -.45, 0)
        base_goal.pose.orientation = Quaternion(*quaternion_about_axis(np.pi, [0, 0, 1]))
        base_goal.pose = kdl_to_pose(T_map__object * msg_to_kdl(base_goal))
        self.base.move_other_frame(base_goal, 'gripper_tool_frame')

        pose = self.robo_sherlock.see(depth, width, height)
        pose.pose.position.z = table_height + height / 2
        self.arm.giskard.remove_object(self.default_object_name)
        self.spawn_fake_object(depth, width, height, pose)

        T_map__object = msg_to_kdl(pose)

        # pregrasp
        arm_goal = PoseStamped()
        arm_goal.header.frame_id = 'map'
        arm_goal.pose.position.y = -.15
        arm_goal.pose.orientation = Quaternion(*quaternion_from_matrix(np.array([[1, 0, 0, 0],
                                                                                 [0, 0, 1, 0],
                                                                                 [0, -1, 0, 0],
                                                                                 [0, 0, 0, 1]])))
        arm_goal.pose = kdl_to_pose(T_map__object * msg_to_kdl(arm_goal))
        arm_goal.pose.position.z = max(TABLE_HEIGHT, arm_goal.pose.position.z)
        self.arm.giskard.avoid_collision(0.02, body_b='table')
        self.arm.set_and_send_cartesian_goal(arm_goal)

        arm_goal = PoseStamped()
        arm_goal.header.frame_id = 'gripper_tool_frame'
        arm_goal.pose.position.z = .15
        arm_goal.pose.orientation.w = 1
        self.arm.giskard.allow_collision(body_b=self.default_object_name)
        self.arm.giskard.avoid_collision(0.02, body_b='table')
        self.arm.set_and_send_cartesian_goal(arm_goal)

        self.gripper.set_pose(width - 0.01)

        self.arm.giskard.attach_object(self.default_object_name, 'gripper_tool_frame')
        # arm_goal.header.frame_id = 'gripper_tool_frame'
        # arm_goal.pose.position = Point(0, .1, -.1)
        self.arm.see_pose()

    def spawn_fake_object(self, depth, width, height, pose):
        self.arm.giskard.add_box(size=[width, depth, height], pose=pose)

    def place_object(self, facing_id, shelf_layer_id, shelf_system_id, object_height):
        facing_frame_id = self.kr.get_object_frame_id(facing_id)
        self.goto_pre_place(shelf_system_id, shelf_layer_id, facing_id, facing_frame_id, object_height)

        arm_goal = PoseStamped()
        arm_goal.header.frame_id = 'gripper_tool_frame'
        arm_goal.pose.position.z = 0.18
        arm_goal.pose.orientation.w = 1
        self.arm.giskard.allow_all_collisions()
        self.arm.set_and_send_cartesian_goal(arm_goal)
        self.arm.giskard.detach_object(self.default_object_name)
        rospy.sleep(.5)
        self.arm.giskard.remove_object(self.default_object_name)
        self.kr.add_objects(facing_id, 1)
        self.gripper.release()

        arm_goal = PoseStamped()
        arm_goal.header.frame_id = 'gripper_tool_frame'
        arm_goal.pose.position.z = -0.18
        arm_goal.pose.orientation.w = 1
        self.arm.giskard.allow_all_collisions()
        self.arm.set_and_send_cartesian_goal(arm_goal)

        self.arm.giskard.allow_all_collisions()
        self.arm.place_pose_right()

    def goto_pre_place(self, shelf_system_id, shelf_layer_id, facing_id, facing_frame_id, object_height):
        self.arm.giskard.allow_all_collisions()
        if not self.kr.is_left(shelf_system_id):
            self.arm.place_pose_right()
        else:
            self.arm.place_pose_right()
        # x_offset = self.calc_x_offset(shelf_layer_id, facing_id)
        base_pose = PoseStamped()
        base_pose.header.frame_id = facing_frame_id
        base_pose.pose.position.y = -.7
        base_pose.pose.orientation = Quaternion(*quaternion_about_axis(np.pi, [0, 0, 1]))
        self.base.move_other_frame(base_pose, 'gripper_tool_frame')
        goal_pose = self.facing_placing_pose(facing_id, object_height)
        self.arm.giskard.allow_all_collisions()
        self.arm.giskard.set_cart_goal(goal_pose, self.default_object_name, 'ur5_shoulder_link')
        self.arm.giskard.plan_and_execute()


if __name__ == '__main__':
    rospy.init_node('cram')
    plan = CRAM()
    plan.sort('')
