from copy import deepcopy
from math import pi

import rospy
from geometry_msgs.msg import PoseStamped, Vector3Stamped, Quaternion
from tf.transformations import quaternion_about_axis

from giskardpy.python_interface import GiskardWrapper
from giskardpy.tfwrapper import lookup_pose
from refills_second_review.gripper import Gripper


class Plan(object):

    def __init__(self):

        self.start_conf = {
            'ur5_shoulder_pan_joint': pi,
            'ur5_shoulder_lift_joint': -1,
            'ur5_elbow_joint': 1,
            'ur5_wrist_1_joint': -pi/2,
            'ur5_wrist_2_joint': -pi/2,
            'ur5_wrist_3_joint': -pi/2,

        }

        self.without_base = 'base_footprint'
        self.with_base = 'odom'
        self.tip = 'refills_tool_frame'
        self.finger = 'refills_finger'
        self.gripper = Gripper(False)
        self.giskard = GiskardWrapper()
        rospy.sleep(1)
        self.giskard.clear_world()

    def reset(self):

        self.giskard.set_joint_goal(self.start_conf)
        self.giskard.plan_and_execute()


    def add_tulip_in_finger(self, name='tulip'):
        tulip_pose = PoseStamped()
        tulip_pose.header.frame_id = self.finger
        tulip_pose.pose.position.y = -0.01
        tulip_pose.pose.orientation.w = 1
        self.giskard.add_box(name, [0.05, 0.09, 0.095], pose=tulip_pose)
        self.giskard.attach_object(name, self.finger)

    def keep_horizontal(self, tip):
        root_normal = Vector3Stamped()
        root_normal.header.frame_id = 'map'
        root_normal.vector.z = 1

        tip_normal = Vector3Stamped()
        tip_normal.header.frame_id = 'refills_finger'
        tip_normal.vector.y = 1
        self.giskard.align_planes(tip, tip_normal, self.without_base, root_normal)

    def gravity_joint(self, tip, object_='tulip'):
        self.giskard.set_json_goal('GravityJoint', joint_name='refills_finger_joint', object_name=object_)

    def move(self,Dx,Dy,Dz):
        pose = lookup_pose(self.finger, self.tip)
        pose.header.stamp = rospy.Time()
        pose.pose.position.x += Dx
        pose.pose.position.y += Dy
        pose.pose.position.z += Dz
        # self.giskard.set_cart_goal(self.without_base, self.tip, pose)
        self.giskard.set_translation_goal(self.without_base, self.tip, pose, max_speed=0.5)
        self.giskard.set_rotation_goal(self.without_base, self.tip, pose, max_speed=1)
        # self.keep_horizontal(self.finger)
        self.gravity_joint(self.finger)
        self.giskard.plan_and_execute()

    def rotate(self,axis, angle):
        pose = PoseStamped()
        pose.header.frame_id = self.tip
        pose.pose.orientation = Quaternion(*quaternion_about_axis(angle, axis))
        self.giskard.set_translation_goal(self.without_base, self.tip, pose, max_speed=0.5)
        self.giskard.set_rotation_goal(self.without_base, self.tip, pose, max_speed=0.5)
        # self.keep_horizontal(self.finger)
        self.gravity_joint(self.finger)
        self.giskard.plan_and_execute()

    def rotate_wrist_3(self,angle):
        joints = deepcopy(self.start_conf)
        joints['ur5_wrist_3_joint'] += angle

        self.giskard.set_joint_goal(joints)
        self.giskard.plan_and_execute()


    def execute(self, gripper_piv=True):

        self.reset()

        self.gripper.home()
        raw_input('press a key')
        self.gripper.grasp(5)
        raw_input('press a key')
        if(gripper_piv):
            self.gripper.gripper_pivoting()
        else:
            self.gripper.slipping_avoidance()

        self.add_tulip_in_finger()

        self.move(0.2, 0, 0)
        self.move(-0.2, 0, 0)
        self.move(0, -0.2, 0)
        self.move(0, 0.2, 0)
        self.move(0, 0, 0.2)
        self.move(0, 0, -0.2)

        self.rotate([1, 0, 0], pi/4)
        self.rotate([1, 0, 0], -pi / 4)

        self.rotate_wrist_3(-pi/2)

        self.rotate([0, 1, 0], pi / 4)

        self.rotate([1, 0, 0], pi / 4)
        self.rotate([1, 0, 0], -pi / 4)


        self.rotate([0, 1, 0], -pi / 4)

        self.reset()

        raw_input('open gripper?')
        self.gripper.home()

rospy.init_node('cartesian_exp')

plan = Plan()
plan.execute(gripper_piv=False)


