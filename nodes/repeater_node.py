#!/usr/bin/env python

import threading

import rospy

import dearpygui.dearpygui as dpg

from tr_drive.operator.repeater import Repeater


def spin_func():
    rospy.spin()
    dpg.destroy_context()

rospy.init_node('tr_repeater', anonymous = False)
repeater = Repeater()
spin_thread = threading.Thread(target = spin_func)
spin_thread.start()

dpg.create_context()
dpg.create_viewport(title = 'repeater', width = 600, height = 400)
dpg.setup_dearpygui()

with dpg.window(tag = 'main'):
    dpg.add_text('Hello.')

dpg.set_primary_window('main', True)
dpg.show_viewport()

# dpg.start_dearpygui()
while dpg.is_dearpygui_running():
    
    
    dpg.render_dearpygui_frame()

