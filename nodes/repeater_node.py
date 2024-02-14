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

with dpg.texture_registry():
    dpg.add_dynamic_texture(
        width = repeater.camera.resize[0],
        height = repeater.camera.resize[1],
        default_value = repeater.camera.get_processed_image().to_imgui_texture(),
        tag = '__processed_image_texture'
    )

with dpg.window(tag = 'main'):
    path_input = dpg.add_input_text(label = 'name', default_value = repeater.params.persistent.recording_path)
    
    start_button = dpg.add_button(label = 'start', width = 80, height = 30, callback = repeater.start_repeating)
    pause_button = dpg.add_button(label = 'pause', width = 80, height = 30, callback = repeater.pause_repeating)
    resume_button = dpg.add_button(label = 'resume', width = 80, height = 30, callback = repeater.resume_repeating)
    
    repeating_status_text = dpg.add_text('')
    
    processed_image = dpg.add_image('__processed_image_texture')

dpg.set_primary_window('main', True)
dpg.show_viewport()

# dpg.start_dearpygui()
while dpg.is_dearpygui_running():
    repeater.params.persistent.recording_path = dpg.get_value(path_input)
    dpg.configure_item(path_input, enabled = not repeater.repeating_launched)
    
    dpg.configure_item(start_button, enabled = not repeater.repeating_launched)
    dpg.configure_item(pause_button, enabled = repeater.repeating_launched and (not repeater.repeating_paused))
    dpg.configure_item(resume_button, enabled = repeater.repeating_launched and repeater.repeating_paused)
    
    dpg.set_value(repeating_status_text, 'Status: None.')
    
    dpg.set_value('__processed_image_texture', repeater.camera.get_processed_image().to_imgui_texture())
    
    dpg.render_dearpygui_frame()

