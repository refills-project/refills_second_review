#!/usr/bin/env python

import rospy
import rospkg

from arrina_rooming_in.post_processing import PostProcessing

if __name__ == '__main__':
    # setup
    rospy.init_node('kb')
    # path_to_json = '{}/data/donbot_bremen/19_shelves.json'.format(rospkg.RosPack().get_path('refills_second_review'))
    filename = '{}/data/donbot_augsburg/pp_scan'.format(rospkg.RosPack().get_path('refills_second_review'))
    my_pp = PostProcessing('', filename)
    my_pp.kb.start_tf_logging()

    # putting missing properties
    my_pp.assert_missing_shelf_props(43, 2.0, 5, "light")
    my_pp.assert_missing_shelf_props(35, 2.0, 5, "light")
    my_pp.assert_missing_shelf_props(33, 1.6, 5, "light")
    # my_pp.assert_missing_shelf_props(17, 2.0, 5, "light")


    # flusing out
    my_pp.correct_shelf_poses()
    rospy.sleep(2)
    my_pp.kb.stop_tf_logging()
    my_pp.export_owl()
