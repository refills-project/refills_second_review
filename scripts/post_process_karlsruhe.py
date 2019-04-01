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
    # first batch
    my_pp.assert_missing_shelf_props(51, 2.0, 6, "light")
    my_pp.assert_missing_shelf_props(53, 2.0, 6, "light")
    my_pp.assert_missing_shelf_props(55, 2.0, 6, "light")
    my_pp.assert_missing_shelf_props(57, 2.0, 6, "light")
    my_pp.assert_missing_shelf_props(43, 2.0, 6, "light")
    my_pp.assert_missing_shelf_props(59, 2.0, 6, "light")
    my_pp.assert_missing_shelf_props(45, 2.0, 6, "light")
    my_pp.assert_missing_shelf_props(49, 2.0, 6, "light")
    my_pp.assert_missing_shelf_props(33, 2.0, 6, "light")
    my_pp.assert_missing_shelf_props(35, 2.0, 7, "heavy")
    my_pp.assert_missing_shelf_props(37, 2.0, 7, "heavy")
    my_pp.assert_missing_shelf_props(39, 2.0, 7, "heavy")
    my_pp.assert_missing_shelf_props(21, 2.0, 7, "heavy")
    my_pp.assert_missing_shelf_props(23, 2.0, 7, "heavy")
    my_pp.assert_missing_shelf_props(25, 2.0, 7, "heavy")
    my_pp.assert_missing_shelf_props(27, 2.0, 5, "light")
    my_pp.assert_missing_shelf_props(29, 2.0, 7, "heavy")
    my_pp.assert_missing_shelf_props(11, 2.0, 7, "heavy")
    my_pp.assert_missing_shelf_props(13, 2.0, 7, "heavy")
    my_pp.assert_missing_shelf_props(17, 1.6, 6, "light")
    my_pp.assert_missing_shelf_props(15, 1.6, 6, "light")
    my_pp.assert_missing_shelf_props(9, 1.6, 6, "light")
    my_pp.assert_missing_shelf_props(7, 1.6, 6, "light")
    # second batch
    my_pp.assert_missing_shelf_props(107, 1.6, 6, "light")
    my_pp.assert_missing_shelf_props(105, 1.6, 6, "light")
    my_pp.assert_missing_shelf_props(103, 1.6, 6, "light")
    my_pp.assert_missing_shelf_props(101, 1.6, 6, "light")
    my_pp.assert_missing_shelf_props(153, 1.6, 5, "light")
    my_pp.assert_missing_shelf_props(151, 1.6, 5, "light")
    my_pp.assert_missing_shelf_props(149, 1.6, 5, "light")
    my_pp.assert_missing_shelf_props(145, 1.6, 5, "light")
    my_pp.assert_missing_shelf_props(143, 1.6, 5, "light")
    my_pp.assert_missing_shelf_props(139, 1.6, 5, "light")
    my_pp.assert_missing_shelf_props(137, 1.6, 5, "light")
    my_pp.assert_missing_shelf_props(135, 1.6, 5, "light")
    my_pp.assert_missing_shelf_props(123, 1.6, 5, "light")
    my_pp.assert_missing_shelf_props(121, 1.6, 5, "light")
    my_pp.assert_missing_shelf_props(119, 1.6, 5, "light")
    my_pp.assert_missing_shelf_props(117, 1.6, 5, "light")
    my_pp.assert_missing_shelf_props(115, 1.6, 5, "light")
    my_pp.assert_missing_shelf_props(113, 1.6, 5, "light")        
    my_pp.assert_missing_shelf_props(111, 1.6, 5, "light")    
    my_pp.assert_missing_shelf_props(109, 1.6, 5, "light")    
    my_pp.assert_missing_shelf_props(155, 1.6, 5, "light")    
    my_pp.assert_missing_shelf_props(157, 1.6, 5, "light")
    my_pp.assert_missing_shelf_props(159, 1.6, 5, "light")        
    my_pp.assert_missing_shelf_props(125, 1.6, 5, "light")    
   
    # flusing out
    my_pp.correct_shelf_poses()
    my_pp.export_owl()
