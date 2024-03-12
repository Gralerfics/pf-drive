#!/usr/bin/env python

import threading

import rospy

import dearpygui.dearpygui as dpg

from tr_drive.operator.repeater import Repeater

from tr_drive.util.image import DigitalImage, ImageProcessor


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

processed_image_texture, passed_goal_image_texture, next_goal_image_texture = None, None, None

def start_repeating_cb():
    repeater.start()
    
    w, h = repeater.camera.resize
    global processed_image_texture, passed_goal_image_texture, next_goal_image_texture
    
    with dpg.texture_registry():
        w, h = repeater.camera.resize
        default_gray = DigitalImage(w, h, 1).to_imgui_texture()
        default_rgb = DigitalImage(w, h, 3).to_imgui_texture()
        passed_goal_image_texture = dpg.add_dynamic_texture(width = w, height = h, default_value = default_gray)
        processed_image_texture = dpg.add_dynamic_texture(width = w, height = h, default_value = default_rgb)
        next_goal_image_texture = dpg.add_dynamic_texture(width = w, height = h, default_value = default_gray)
        
    dpg.add_image(passed_goal_image_texture, width = w, height = h, parent = 'main')
    dpg.add_image(processed_image_texture, width = w, height = h, parent = 'main')
    dpg.add_image(next_goal_image_texture, width = w, height = h, parent = 'main')

with dpg.window(tag = 'main'):
    path_input = dpg.add_input_text(label = 'path', default_value = repeater.params.persistent.recording_path)
    load_button = dpg.add_button(label = 'load', width = 80, height = 30, callback = repeater.load_recording)
    
    start_button = dpg.add_button(label = 'start', width = 80, height = 30, callback = start_repeating_cb)
    pause_button = dpg.add_button(label = 'pause', width = 80, height = 30, callback = repeater.pause)
    resume_button = dpg.add_button(label = 'resume', width = 80, height = 30, callback = repeater.resume)
    
    repeating_status_text = dpg.add_text('')

dpg.set_primary_window('main', True)
dpg.show_viewport()

while dpg.is_dearpygui_running():
    repeater.params.persistent.recording_path = dpg.get_value(path_input)
    dpg.configure_item(path_input, enabled = not repeater.launched.is_set())
    dpg.configure_item(load_button, enabled = not repeater.launched.is_set())
    
    dpg.configure_item(start_button, enabled = not repeater.launched.is_set())
    dpg.configure_item(pause_button, enabled = repeater.launched.is_set() and (not repeater.paused.is_set()))
    dpg.configure_item(resume_button, enabled = repeater.launched.is_set() and repeater.paused.is_set())
    
    if repeater.launched.is_set():
        if repeater.paused.is_set():
            dpg.set_value(repeating_status_text, 'Status: paused.')
        else:
            dpg.set_value(repeating_status_text, f'Status: goal {repeater.get_passed_goal_index()} passed.')
    else:
        dpg.set_value(repeating_status_text, 'Status: stopped.')
    
    if processed_image_texture is not None and passed_goal_image_texture is not None and next_goal_image_texture is not None:
        passed_goal_image = repeater.recording.processed_images[min(repeater.get_passed_goal_index(), len(repeater.recording.processed_images) - 1)]
        dpg.set_value(passed_goal_image_texture, passed_goal_image.to_imgui_texture())

        if repeater.camera.is_ready():
            processed_img = repeater.camera.get_processed_image().to_rgb()
            passed_goal_match_line_x = -repeater.passed_goal_image_match_offset + processed_img.width // 2
            next_goal_match_line_x = -repeater.next_goal_image_match_offset + processed_img.width // 2
            processed_img = ImageProcessor.draw_line(processed_img, processed_img.width // 2, 0, processed_img.width // 2, processed_img.height - 1, [255, 255, 255])
            processed_img = ImageProcessor.draw_line(processed_img, passed_goal_match_line_x, 0, passed_goal_match_line_x, processed_img.height - 1, [255, 0, 0])
            processed_img = ImageProcessor.draw_line(processed_img, next_goal_match_line_x, 0, next_goal_match_line_x, processed_img.height - 1, [0, 0, 255])
            dpg.set_value(processed_image_texture, processed_img.to_imgui_texture())

        next_goal_image = repeater.recording.processed_images[min(repeater.get_passed_goal_index() + 1, len(repeater.recording.processed_images) - 1)]
        dpg.set_value(next_goal_image_texture, next_goal_image.to_imgui_texture())
    
    dpg.render_dearpygui_frame()

