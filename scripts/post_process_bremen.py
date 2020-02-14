#!/usr/bin/env python

import rospy
import rospkg

from arrina_rooming_in.post_processing import PostProcessing

if __name__ == '__main__':
    rospy.init_node('kb')
    filename = '{}/data/donbot_bremen/test_scan_pp'.format(rospkg.RosPack().get_path('refills_second_review'))
    my_pp = PostProcessing('', filename)
    my_pp.kb.start_tf_logging()
    rospy.sleep(2)

    # putting missing properties
    my_pp.assert_missing_shelf_props(37, 1.6, 6, "light")
    # my_pp.assert_missing_shelf_props(13, 2.0, 5, "light")
    # my_pp.assert_missing_shelf_props(15, 2.0, 5, "light")
    # my_pp.assert_missing_shelf_props(17, 2.0, 5, "light")


    # flusing out
    my_pp.correct_shelf_poses()
    rospy.sleep(2)
    my_pp.kb.stop_tf_logging()
    my_pp.export_owl()
