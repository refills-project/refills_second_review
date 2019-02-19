from refills_msgs.srv import QueryDetectShelfLayersPath, QueryDetectShelfLayersPathResponse
from py_trees import Behaviour, Status, Blackboard
import rospy
from rospy import ROSException
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class ConfigureLayerDetectionBehavior(Behaviour):
    def __init__(self, name="", *args, **kwargs):
        super(ConfigureLayerDetectionBehavior, self).__init__(name, *args, **kwargs)
        self.client = None # type: rospy.ServiceProxy
        self.trajectory_sample_period = None # type: float

    def setup(self, timeout):
        self.client = rospy.ServiceProxy('/perception_interface/query_detect_shelf_layers_path', QueryDetectShelfLayersPath)
        try:
            self.client.wait_for_service(timeout)
        except ROSException as e:
            return False
        if not rospy.has_param('~trajectory_sample_period'):
            self.feedback_message = "Could not find ROS param '~trajectory_sample_period'"
            return False
        self.trajectory_sample_period = rospy.get_param('~trajectory_sample_period')
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
        Blackboard().set("arm_trajectory", self._extract_trajectory(response))
        return Status.SUCCESS

    def terminate(self, new_status):
        super(ConfigureLayerDetectionBehavior, self).terminate(new_status)

    def _extract_trajectory(self, response):
        """
        Extracts a trajectory message from the response of a call to query_detect_shelf_layers_path.
        :param response: The response from the service call.
        :type reponse: QueryDetectShelfLayersPathResponse
        :return: The extracted trajectory.
        :rtype: JointTrajectory
        """
        trajectory = JointTrajectory()
        trajectory.joint_names = [joint.name for joint in response.path.postures[0].joints]
        for count, posture in enumerate(response.path.postures):
            p = JointTrajectoryPoint()
            p.time_from_start = rospy.Duration((count+1)*self.trajectory_sample_period)
            p.positions = [joint.position for joint in posture.joints]
            trajectory.points.append(p)
        return trajectory
