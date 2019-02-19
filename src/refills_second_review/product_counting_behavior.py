from py_trees import Behaviour, Status


class ProductCountingBehavior(Behaviour):
    def __init__(self, name="", *args, **kwargs):
        super(ProductCountingBehavior, self).__init__(name, *args, **kwargs)

    def setup(self, timeout):
        #TODO ros services
        return super(ProductCountingBehavior, self).setup(timeout)

    def initialise(self):
        super(ProductCountingBehavior, self).initialise()

    def update(self):
        return Status.SUCCESS

    def terminate(self, new_status):
        super(ProductCountingBehavior, self).terminate(new_status)
