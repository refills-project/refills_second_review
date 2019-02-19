from refills_msgs.srv import QueryDetectShelfLayersPath, QueryDetectShelfLayersPathResponse
from py_trees import Behaviour, Status, Blackboard
import rospy
from rospy import ROSException


class ConfigureLayerDetectionBehavior(Behaviour):
    def __init__(self, name="", *args, **kwargs):
        super(ConfigureLayerDetectionBehavior, self).__init__(name, *args, **kwargs)
        self.client = None # type: rospy.ServiceProxy

    def setup(self, timeout):
        self.client = rospy.ServiceProxy('/perception_interface/query_detect_shelf_layers_path', QueryDetectShelfLayersPath)
        try:
            self.client.wait_for_service(timeout)
        except ROSException as e:
            return False
        return super(ConfigureLayerDetectionBehavior, self).setup(timeout)

    def initialise(self):
        super(ConfigureLayerDetectionBehavior, self).initialise()

    def update(self):
        shelf_id = Blackboard().get("current_shelf_id")
        if not shelf_id:
            return Status.FAILURE

        response = self.client.call(id=shelf_id) # type:QueryDetectShelfLayersPathResponse
        if not response.error == response.SUCCESS or len(response.path.postures) == 0:
            self.feedback_message = '{}'.format(response)
            return Status.FAILURE

        Blackboard().set("move_base_target_pose", response.path.postures[0].base_pos)
        # TODO: set arm_trajectory of type trajectory_msgs/JointTrajectory
        return Status.SUCCESS

    def terminate(self, new_status):
        super(ConfigureLayerDetectionBehavior, self).terminate(new_status)
