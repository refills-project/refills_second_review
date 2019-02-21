#!/usr/bin/env python

import rospy
import rospkg

from arrina_rooming_in.post_processing import PostProcessing

if __name__ == '__main__':
    # setup
    rospy.init_node('kb')
    path_to_json = '{}/data/donbot_bremen/19_shelves.json'.format(rospkg.RosPack().get_path('refills_second_review'))
    filename = '{}/data/donbot_bremen/19_shelves_pp.owl'.format(rospkg.RosPack().get_path('refills_second_review'))
    my_pp = PostProcessing(path_to_json, filename)

    # putting missing properties
    my_pp.assert_missing_shelf_props(81, 2.0, 7, "heavy")
    my_pp.assert_missing_shelf_props(59, 2.0, 5, "light")
    my_pp.assert_missing_shelf_props(9, 1.8, 5, "light")
    my_pp.assert_missing_shelf_props(45, 1.8, 5, "light")
    my_pp.assert_missing_shelf_props(11, 2.0, 6, "light")
    my_pp.assert_missing_shelf_props(53, 2.0, 6, "light")
    my_pp.assert_missing_shelf_props(51, 2.0, 6, "light")
    my_pp.assert_missing_shelf_props(49, 2.0, 6, "light")
    my_pp.assert_missing_shelf_props(3, 2.0, 7, "heavy")
    my_pp.assert_missing_shelf_props(13, 2.0, 7, "heavy")
    my_pp.assert_missing_shelf_props(33, 2.0, 7, "heavy")
    my_pp.assert_missing_shelf_props(35, 2.0, 6, "light")
    my_pp.assert_missing_shelf_props(37, 2.0, 6, "light")
    my_pp.assert_missing_shelf_props(7, 1.6, 6, "light")
    my_pp.assert_missing_shelf_props(5, 1.6, 6, "light")
    my_pp.assert_missing_shelf_props(1, 1.6, 6, "light")
    my_pp.assert_missing_shelf_props(55, 1.6, 5, "light")
    my_pp.assert_missing_shelf_props(39, 1.6, 5, "light")
    my_pp.assert_missing_shelf_props(85, 1.6, 5, "light")

    # flusing out
    my_pp.correct_shelf_poses()
    my_pp.export_owl()
