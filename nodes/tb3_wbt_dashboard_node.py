#!/usr/bin/env python

import threading
import time

import rospy

import dearpygui.dearpygui as dpg

from tr_drive.simulation.webots import WebotsDashboard


def spin_func():
    rospy.spin()
    dpg.destroy_context()

rospy.init_node('tr_tb3_wbt_dashboard', anonymous = False)
dashboard = WebotsDashboard('/tb3/supervisor', '/tb3/controller_manager', 'controller')
spin_thread = threading.Thread(target = spin_func)
spin_thread.start()

dashboard.wait_until_ready()

dpg.create_context()
dpg.create_viewport(title = 'dashboard', width = 600, height = 400)
dpg.setup_dearpygui()

dpg.set_global_font_scale(1.2)

with dpg.window(tag = 'main'):
    reset_button = dpg.add_button(label = 'reset simulation', width = 250, height = 30, callback = dashboard.reset_all)

dpg.set_primary_window('main', True)
dpg.show_viewport()

dpg.start_dearpygui()
# while dpg.is_dearpygui_running():
#     dpg.render_dearpygui_frame()

