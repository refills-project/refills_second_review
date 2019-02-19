from py_trees import Behaviour, Status

class FacingDetectionBehavior(Behaviour):
    def __init__(self, name="", *args, **kwargs):
        super(FacingDetectionBehavior, self).__init__(name, *args, **kwargs)

    def setup(self, timeout):
        # TODO ros services
        return super(FacingDetectionBehavior, self).setup(timeout)

    def initialise(self):
        super(FacingDetectionBehavior, self).initialise()

    def update(self):
        return Status.SUCCESS

    def terminate(self, new_status):
        super(FacingDetectionBehavior, self).terminate(new_status)
