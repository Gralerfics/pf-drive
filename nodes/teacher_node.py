#!/usr/bin/env python

import threading

import rospy

import dearpygui.dearpygui as dpg

from tr_drive.operator.teacher import Teacher


def spin_func():
    rospy.spin()
    dpg.destroy_context()

rospy.init_node('tr_teacher', anonymous = False)
teacher = Teacher()
spin_thread = threading.Thread(target = spin_func)
spin_thread.start()

dpg.create_context()
dpg.create_viewport(title = 'teacher', width = 600, height = 400)
dpg.setup_dearpygui()

with dpg.texture_registry():
    dpg.add_dynamic_texture(
        width = teacher.camera.resize[0],
        height = teacher.camera.resize[1],
        default_value = teacher.camera.get_processed_image().to_imgui_texture(),
        tag = '__processed_image_texture'
    )

with dpg.window(tag = 'main'):
    folder_input = dpg.add_input_text(label = 'folder', default_value = teacher.params.persistent.recording_folder)
    name_input = dpg.add_input_text(label = 'name', default_value = teacher.params.persistent.recording_name)
    auto_naming_checkbox = dpg.add_checkbox(label = 'auto naming', default_value = teacher.params.persistent.auto_naming)
    
    start_button = dpg.add_button(label = 'start', width = 80, height = 30, callback = teacher.start_recording)
    stop_button = dpg.add_button(label = 'stop', width = 80, height = 30, callback = teacher.stop_recording)
    
    recording_status_text = dpg.add_text('')
    
    processed_image = dpg.add_image('__processed_image_texture')

dpg.set_primary_window('main', True)
dpg.show_viewport()

# dpg.start_dearpygui()
while dpg.is_dearpygui_running():
    teacher.params.persistent.recording_folder = dpg.get_value(folder_input)
    teacher.params.persistent.recording_name = dpg.get_value(name_input)
    teacher.params.persistent.auto_naming = dpg.get_value(auto_naming_checkbox)
    dpg.configure_item(name_input, enabled = not dpg.get_value(auto_naming_checkbox))
    
    dpg.configure_item(start_button, enabled = not teacher.recording_launched)
    dpg.configure_item(stop_button, enabled = teacher.recording_launched)
    
    dpg.set_value(recording_status_text, f'Status: {f"{len(teacher.recording.odoms)} goals recorded." if teacher.recording_launched else "Stopped."}')
    
    dpg.set_value('__processed_image_texture', teacher.camera.get_processed_image().to_imgui_texture())
    
    dpg.render_dearpygui_frame()

