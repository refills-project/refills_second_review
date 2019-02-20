#!/usr/bin/env python

import rospy
import rospkg

from arrina_rooming_in.post_processing import PostProcessing

if __name__ == '__main__':
    rospy.init_node('kb')
    path_to_json = '{}/data/donbot_bremen/19_shelves.json'.format(rospkg.RosPack().get_path('refills_second_review'))
    filename = '{}/data/donbot_bremen/19_shelves.owl'.format(rospkg.RosPack().get_path('refills_second_review'))
    my_pp = PostProcessing(path_to_json, filename)

    # TODO: fill me
    # # assert missing shelf properties
    # # Karlsruhe backroom
    # my_pp.assert_missing_shelf_props(135, 2.0, 5, "light")
    # my_pp.assert_missing_shelf_props(137, 2.0, 5, "light")
    # my_pp.assert_missing_shelf_props(121, 2.0, 5, "light")
    # my_pp.assert_missing_shelf_props(139, 2.0, 5, "light")
    # my_pp.assert_missing_shelf_props(133, 2.0, 5, "light")
    # my_pp.assert_missing_shelf_props(123, 2.0, 5, "light")

    # # Karlsruhe store, partial 1
    # my_pp.assert_missing_shelf_props(37, 2.0, 6, "light")
    # my_pp.assert_missing_shelf_props(7, 2.0, 6, "light")
    # my_pp.assert_missing_shelf_props(9, 2.0, 6, "light")
    # my_pp.assert_missing_shelf_props(57, 2.0, 6, "light")
    # my_pp.assert_missing_shelf_props(25, 2.0, 6, "light")
    # my_pp.assert_missing_shelf_props(21, 2.0, 6, "light")
    # my_pp.assert_missing_shelf_props(11, 2.0, 6, "light")
    # my_pp.assert_missing_shelf_props(31, 2.0, 6, "light")
    # my_pp.assert_missing_shelf_props(5, 2.0, 6, "light")
    # my_pp.assert_missing_shelf_props(23, 2.0, 7, "heavy")
    # my_pp.assert_missing_shelf_props(59, 2.0, 7, "heavy")
    # my_pp.assert_missing_shelf_props(29, 2.0, 7, "heavy")
    # my_pp.assert_missing_shelf_props(27, 2.0, 7, "heavy")

    # # Karlsruhe store, partial 2
    # my_pp.assert_missing_shelf_props(89, 1.6, 5, "light")
    # my_pp.assert_missing_shelf_props(77, 1.6, 5, "light")
    # my_pp.assert_missing_shelf_props(81, 1.6, 5, "light")
    # my_pp.assert_missing_shelf_props(74, 1.6, 5, "light")
    # my_pp.assert_missing_shelf_props(64, 1.6, 5, "light")

    my_pp.correct_shelf_poses()
    my_pp.export_owl()
