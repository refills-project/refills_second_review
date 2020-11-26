from __future__ import division

import rospy
from actionlib import SimpleActionClient
from iai_wsg_50_msgs.msg import PositionCmd
from slipping_control_msgs.msg import HomeGripperAction, SlippingControlAction, GraspAction, HomeGripperGoal, \
    GraspGoal, SlippingControlGoal
from slipping_control_msgs.srv import GetState, ChLSParams, ChLSParamsRequest


class Gripper(object):
    def __init__(self, active=True):
        self.active = active
        self.home_action = SimpleActionClient('/wsg50/home_gripper_action', HomeGripperAction)
        self.slipping_control_action = SimpleActionClient('/wsg50/slipping_control_action', SlippingControlAction)
        self.grasp_action = SimpleActionClient('/wsg50/grasp_action', GraspAction)
        self.get_state_service = rospy.ServiceProxy('/wsg50/slipping_control/get_state', GetState)
        self.ch_params0_service = rospy.ServiceProxy('/wsg50/ls_0/change_params', ChLSParams)
        self.ch_params1_service = rospy.ServiceProxy('/wsg50/ls_1/change_params', ChLSParams)
        rospy.sleep(0.5)
        rospy.loginfo('connected to gripper driver')

    def home(self):
        if self.active:
            self.home_action.send_goal_and_wait(HomeGripperGoal())
            rospy.loginfo('gripper homed')

    def grasp(self, force=5):
        if self.active:
            goal = GraspGoal()
            goal.desired_force = force
            self.grasp_action.send_goal_and_wait(goal)

    def gripper_pivoting(self):
        if self.active:
            goal = SlippingControlGoal()
            goal.mode = SlippingControlGoal.MODE_GRIPPER_PIVOTING
            self.slipping_control_action.send_goal_and_wait(goal)

    def slipping_avoidance(self):
        if self.active:
            goal = SlippingControlGoal()
            goal.mode = SlippingControlGoal.MODE_SLIPPING_AVOIDANCE
            self.slipping_control_action.send_goal_and_wait(goal)

    def get_state(self):
        if self.active:
            return self.get_state_service().state
        else:
            return 4

    # define STATE_UNDEFINED                    -1
    # define STATE_HOME                          0
    # define STATE_HOMING                        1
    # define STATE_COMPUTING_BIAS                2
    # define STATE_GRASPING                      3
    # define STATE_GRASPED                       4
    # define STATE_TO_GRIPPER_PIVOTING           5
    # define STATE_GRIPPER_PIVOTING              6
    # define STATE_TO_SLIPPING_AVOIDANCE         7
    # define STATE_SLIPPING_AVOIDANCE            8
    # define STATE_TO_DYN_SLIPPING_AVOIDANCE     9
    # define STATE_DYN_SLIPPING_AVOIDANCE       10
    # define STATE_OBJECT_PIVOTING              11
    def is_grasped(self):
        state = self.get_state()
        if state == 4 or state == 5 or state == 6 or state == 7 or state == 8 or state == 9 or state == 10:
            return True
        return False

    def change_params(self, mu):
        if self.active:
            ch_msg = ChLSParamsRequest()
            ch_msg.delta = -1
            ch_msg.gamma = -1
            ch_msg.mu = mu
            ch_msg.k = -1
            self.ch_params0_service(ch_msg)
            self.ch_params1_service(ch_msg)

class GripperWSG50(object):
    def __init__(self):
        self.position_pub = rospy.Publisher('/wsg_50_driver/goal_position', PositionCmd, queue_size=10)
        self.min = 0.02
        self.max = 0.1

    def mm_to_m(self, mm):
        return mm/1000.

    def m_to_mm(self, m):
        return m*1000.

    def send_cmd(self, goal_position):
        goal = PositionCmd()
        goal.pos = self.m_to_mm(goal_position)
        self.position_pub.publish(goal)

    def home(self):
        self.send_cmd(self.max)




if __name__ == '__main__':
    rospy.init_node('gripper_test')
    gripper = Gripper()
    rospy.sleep(1)
    gripper.home()
    # raw_input('press a key')
    # gripper.grasp(5)
    # raw_input('press a key')
    print('going into pivoting')
    # gripper.gripper_pivoting()