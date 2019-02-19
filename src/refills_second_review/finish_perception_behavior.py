from arrina_msgs.srv import FinishPerception, FinishPerceptionResponse
from py_trees import Behaviour, Status
import rospy


class FinishPerceptionBehavior(Behaviour):
    def __init__(self, name="", *args, **kwargs):
        super(FinishPerceptionBehavior, self).__init__(name, *args, **kwargs)
        self.client = None # type: rospy.ServiceProxy

    def setup(self, timeout):
        self.client = rospy.ServiceProxy('/perception_interface/finish_perception', FinishPerception)
        try:
            self.client.wait_for_service(timeout)
        except rospy.ROSException as e:
            return False
        return super(FinishPerceptionBehavior, self).setup(timeout)

#    def initialise(self):
#        super(FinishPerceptionBehavior, self).initialise()

    def update(self):
        response = self.client.call() # type:FinishPerceptionResponse
        if not response.error == response.SUCCESS:
            self.feedback_message = '{}'.format(response.error_msg)
            return Status.FAILURE
        return Status.SUCCESS

#    def terminate(self, new_status):
#        super(FinishPerceptionBehavior, self).terminate(new_status)
