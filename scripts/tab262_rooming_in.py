#!/usr/bin/env python

import rospy
import rospkg

from arrina_rooming_in.post_processing import PostProcessing

if __name__ == '__main__':
    # setup
    rospy.init_node('kb')
    path_to_json = '{}/data/donbot_bremen/19_shelves.json'.format(rospkg.RosPack().get_path('refills_second_review'))
    filename = '{}/data/donbot_bremen/19_shelves.owl'.format(rospkg.RosPack().get_path('refills_second_review'))
    my_pp = PostProcessing(path_to_json, filename)

    # putting missing properties
    my_pp.assert_missing_shelf_propos(81, 2.0, 6, "heavy")
    my_pp.assert_missing_shelf_propos(59, 2.0, 4, "light")
    my_pp.assert_missing_shelf_propos(9, 1.8, 4, "light")
    my_pp.assert_missing_shelf_propos(45, 1.8, 4, "light")
    my_pp.assert_missing_shelf_propos(11, 2.0, 5, "light")
    my_pp.assert_missing_shelf_propos(53, 2.0, 5, "light")
    my_pp.assert_missing_shelf_propos(51, 2.0, 5, "heavy")
    my_pp.assert_missing_shelf_propos(49, 2.0, 5, "heavy")
    my_pp.assert_missing_shelf_propos(3, 2.0, 6, "heavy")
    my_pp.assert_missing_shelf_propos(13, 2.0, 6, "heavy")
    my_pp.assert_missing_shelf_propos(33, 2.0, 6, "heavy")
    my_pp.assert_missing_shelf_propos(35, 2.0, 5, "light")
    my_pp.assert_missing_shelf_propos(37, 2.0, 5, "light")
    my_pp.assert_missing_shelf_propos(7, 1.6, 5, "light")
    my_pp.assert_missing_shelf_propos(5, 1.6, 5, "light")
    my_pp.assert_missing_shelf_propos(1, 1.6, 5, "light")
    my_pp.assert_missing_shelf_propos(55, 1.6, 4, "light")
    my_pp.assert_missing_shelf_propos(39, 1.6, 4, "light")
    my_pp.assert_missing_shelf_propos(85, 1.6, 4, "light")

    # flusing out
    my_pp.correct_shelf_poses()
    my_pp.export_owl()
