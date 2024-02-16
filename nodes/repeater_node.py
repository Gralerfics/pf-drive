#!/usr/bin/env python

import threading

import rospy

import dearpygui.dearpygui as dpg

from tr_drive.operator.repeater import Repeater

from tr_drive.util.image import DigitalImage


def spin_func():
    rospy.spin()
    dpg.destroy_context()

rospy.init_node('tr_repeater', anonymous = False)
repeater = Repeater()
spin_thread = threading.Thread(target = spin_func)
spin_thread.start()

repeater.wait_until_ready()

dpg.create_context()
dpg.create_viewport(title = 'repeater', width = 600, height = 400)
dpg.setup_dearpygui()

with dpg.texture_registry():
    w, h = repeater.camera.resize
    default_img = DigitalImage(w, h, 1).to_imgui_texture()
    processed_image_texture = dpg.add_dynamic_texture(width = w, height = h, default_value = default_img)
    next_goal_image_texture = dpg.add_dynamic_texture(width = w, height = h, default_value = default_img)

def load_recording_cb():
    repeater.load_recording()
    w, h = repeater.camera.resize
    
    # global processed_image_texture, next_goal_image_texture
    # with dpg.texture_registry():
    #     w, h = repeater.camera.resize
    #     default_img = DigitalImage(w, h, 1).to_imgui_texture()
    #     processed_image_texture = dpg.add_dynamic_texture(width = w, height = h, default_value = default_img)
    #     next_goal_image_texture = dpg.add_dynamic_texture(width = w, height = h, default_value = default_img)
    
    dpg.configure_item(processed_image, width = w, height = h)
    dpg.configure_item(next_goal_image, width = w, height = h)

with dpg.window(tag = 'main'):
    path_input = dpg.add_input_text(label = 'path', default_value = repeater.params.persistent.recording_path)
    load_button = dpg.add_button(label = 'load', width = 80, height = 30, callback = load_recording_cb)
    
    start_button = dpg.add_button(label = 'start', width = 80, height = 30, callback = repeater.start_repeating)
    pause_button = dpg.add_button(label = 'pause', width = 80, height = 30, callback = repeater.pause_repeating)
    resume_button = dpg.add_button(label = 'resume', width = 80, height = 30, callback = repeater.resume_repeating)
    
    repeating_status_text = dpg.add_text('')

    processed_image = dpg.add_image(processed_image_texture)
    next_goal_image = dpg.add_image(next_goal_image_texture)

dpg.set_primary_window('main', True)
dpg.show_viewport()

while dpg.is_dearpygui_running():
    repeater.params.persistent.recording_path = dpg.get_value(path_input)
    dpg.configure_item(path_input, enabled = not repeater.repeating_launched)
    dpg.configure_item(load_button, enabled = not repeater.repeating_launched)
    
    dpg.configure_item(start_button, enabled = not repeater.repeating_launched)
    dpg.configure_item(pause_button, enabled = repeater.repeating_launched and (not repeater.repeating_paused))
    dpg.configure_item(resume_button, enabled = repeater.repeating_launched and repeater.repeating_paused)
    
    dpg.set_value(repeating_status_text, f'Status: goal {repeater.get_passed_goal_index()} passed.')
    
    if repeater.camera.is_ready():
        dpg.set_value(processed_image_texture, repeater.camera.get_processed_image().to_imgui_texture())
    dpg.set_value(next_goal_image_texture, repeater.recording.processed_images[min(repeater.get_passed_goal_index() + 1, len(repeater.recording.processed_images) - 1)].to_imgui_texture())
    
    dpg.render_dearpygui_frame()

