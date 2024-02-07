#!/usr/bin/env python

import rospy

from tr_drive.operator.repeater import Repeater


rospy.init_node('tr_repeater', anonymous = False)
repeater = Repeater()
rospy.spin()

