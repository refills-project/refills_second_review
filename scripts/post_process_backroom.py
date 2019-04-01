#!/usr/bin/env python

import rospy
import rospkg

from arrina_rooming_in.post_processing import PostProcessing

if __name__ == '__main__':
    # setup
    rospy.init_node('kb')
    # path_to_json = '{}/data/donbot_bremen/19_shelves.json'.format(rospkg.RosPack().get_path('refills_second_review'))
    filename = '{}/data/donbot_karlsruhe/partial01_pp.owl'.format(rospkg.RosPack().get_path('refills_second_review'))
    my_pp = PostProcessing('', filename)

    # putting missing properties
    my_pp.assert_missing_shelf_props(11, 2.0, 5, "light")
    my_pp.assert_missing_shelf_props(13, 2.0, 5, "light")
    my_pp.assert_missing_shelf_props(15, 2.0, 5, "light")
    my_pp.assert_missing_shelf_props(17, 2.0, 5, "light")


    # flusing out
    my_pp.correct_shelf_poses()
    my_pp.export_owl()
