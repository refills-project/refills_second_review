#!/usr/bin/env python
import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from iai_wsg_50_msgs.msg import PositionCmd, Status
from tf.transformations import quaternion_from_matrix, quaternion_about_axis

from giskardpy.tfwrapper import msg_to_kdl, kdl_to_pose
from giskardpy.utils import msg_to_list, position_dist
from refills_perception_interface.knowrob_wrapper import KnowRob
from refills_perception_interface.move_arm import MoveArm
from refills_perception_interface.move_base import MoveBase

# TODO not below 82 during pick up
# TODO table height = 72
from refills_perception_interface.move_gripper import MoveGripper
from refills_perception_interface.robosherlock_wrapper import RoboSherlock, FakeRoboSherlock
from refills_perception_interface.separator_detection import SeparatorClustering
from refills_perception_interface.tfwrapper import lookup_pose, transform_pose

TABLE_HEIGHT = 0.81


class CRAM(object):
    def __init__(self):
        path_to_yaml = rospy.get_param('/perception_interface/path_to_json')
        rospy.set_param('~path_to_json', path_to_yaml)

        self.default_object_name = 'box'
        self.kr = KnowRob()
        self.separator_detector = SeparatorClustering(self.kr)
        self.base = MoveBase()
        self.gripper = MoveGripper()
        try:
            self.robo_sherlock = RoboSherlock(self.kr, name='RoboSherlock_scenario3', check_camera=False)
        except:
            self.robo_sherlock = FakeRoboSherlock(self.kr)
            rospy.logwarn('couldn\'t connect to robo sherlock; using fake mode')
        self.arm = MoveArm(tip='gripper_tool_frame', root='base_footprint')
        self.arm.giskard.clear_world()

    def facing_placing_pose(self, facing_id, object_height):
        facing_pose = PoseStamped()
        facing_pose.header.frame_id = self.kr.get_object_frame_id(facing_id)
        facing_depth = self.kr.get_facing_depth(facing_id)
        facing_height = self.kr.get_facing_height(facing_id)
        facing_pose.pose.position.y = -facing_depth / 2. - 0.1
        facing_pose.pose.position.z = -facing_height / 2. + object_height / 2 + .03
        facing_pose.pose.position.x = 0.01 # TODO magic
        facing_pose.pose.orientation = Quaternion(0, 0, 0, 1)
        # facing_pose.pose.orientation = Quaternion(*quaternion_from_matrix(np.array([[1, 0, 0, 0],
        #                                                                             [0, 0, 1, 0],
        #                                                                             [0, -1, 0, 0],
        #                                                                             [0, 0, 0, 1]])))
        return facing_pose

    def refill(self):
        # self.arm.drive_pose()
        self.robo_sherlock.set_ring_light(0)
        for shelf_system_id in self.kr.get_shelf_system_ids():
            for shelf_layer_id in self.kr.get_shelf_layer_from_system(shelf_system_id):
                empty_facings = self.kr.get_empty_facings_from_layer(shelf_layer_id)
                shelf_layer_frame_id = self.kr.get_perceived_frame_id(shelf_layer_id)
                for i, facing_id in enumerate(sorted(empty_facings,
                                                     key=lambda x: lookup_pose(shelf_layer_frame_id,
                                                                               self.kr.get_object_frame_id(x)).pose.position.x)):
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
                position_dist(old_sep1.pose.position, old_sep2.pose.position)+0.02:
            return 0

        new_mid = new_sep1.pose.position.x + (new_sep2.pose.position.x - new_sep1.pose.position.x)/2
        return new_mid

    def goto_see_pose(self):
        self.arm.see_pose()
        base_pose = PoseStamped()
        base_pose.header.frame_id = 'map'
        base_pose.pose.position = Point(2.668, 1.376, 0)
        base_pose.pose.orientation = Quaternion(*quaternion_about_axis(np.pi, [0, 0, 1]))
        self.base.move_other_frame(base_pose, 'gripper_tool_frame')

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
        self.arm.set_and_send_cartesian_goal(arm_goal)

        arm_goal = PoseStamped()
        arm_goal.header.frame_id = 'gripper_tool_frame'
        arm_goal.pose.position.z = .15
        arm_goal.pose.orientation.w = 1
        self.arm.giskard.allow_collision(body_b=self.default_object_name)
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
        self.arm.giskard.set_cart_goal('ur5_shoulder_link', self.default_object_name, goal_pose)
        self.arm.giskard.plan_and_execute()


if __name__ == '__main__':
    rospy.init_node('cram')
    plan = CRAM()
    plan.refill()
