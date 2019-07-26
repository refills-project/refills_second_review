from actionlib import SimpleActionClient


class Gripper(object):
    def __init__(self, active=True):
        if active:
            try:
                from slipping_control_common.msg import HomeGripperAction, SlippingControlAction, GraspAction, HomeGripperGoal, \
                    GraspGoal, SlippingControlGoal
                self.home_action = SimpleActionClient('/wsg50/home_gripper_action', HomeGripperAction)
                self.slipping_control_action = SimpleActionClient('/wsg50/slipping_control_action', SlippingControlAction)
                self.grasp_action = SimpleActionClient('/wsg50/grasp_action', GraspAction)
            except:
                pass
        else:
            global HomeGripperGoal, GraspGoal, SlippingControlGoal
            HomeGripperGoal = None
            GraspGoal = None
            SlippingControlGoal = None
        self.active = active

    def home(self):
        if self.active:
            self.home_action.send_goal_and_wait(HomeGripperGoal())

    def grasp(self, force):
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
            goal.mode = SlippingControlGoal.MODE_DYN_SLIPPING_AVOIDANCE
            self.slipping_control_action.send_goal_and_wait(goal)
