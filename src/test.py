import dearpygui.dearpygui as dpg
import dearpygui.demo as demo

def save_callback():
    print("Save Clicked")

dpg.create_context()
dpg.create_viewport()

demo.show_demo()

with dpg.window(label="Example Window"):
    dpg.add_text("Hello world")
    dpg.add_button(label="Save", callback=save_callback)
    dpg.add_input_text(label="string")
    dpg.add_slider_float(label="float")

dpg.setup_dearpygui()
dpg.show_viewport()
dpg.start_dearpygui()
dpg.destroy_context()


# import numpy as np
# import tf_conversions as tfc
# from nav_msgs.msg import Odometry
# from tr_drive.util.conversion import Vec3, Quat, Frame


# a = Odometry()
# a.pose.pose.position.x = 1
# a.pose.pose.position.y = 2
# a.pose.pose.position.z = 3
# a.pose.pose.orientation.x = -0.379
# a.pose.pose.orientation.y = 0.282
# a.pose.pose.orientation.z = 0.281
# a.pose.pose.orientation.w = 0.836

# b = Odometry()
# b.pose.pose.position.x = 4
# b.pose.pose.position.y = 5
# b.pose.pose.position.z = 6
# b.pose.pose.orientation.x = 0.112
# b.pose.pose.orientation.y = 0.112
# b.pose.pose.orientation.z = 0.112
# b.pose.pose.orientation.w = 0.981

# c = tfc.fromMsg(a.pose.pose).Inverse() * tfc.fromMsg(b.pose.pose)
# print(tfc.toMsg(c))

# A = Frame(a)
# B = Frame(b)

# C = A.I * B
# print(C.to_Pose())

# print(tfc.fromMsg(a.pose.pose))
# print(A.quaternion.to_rotation_matrix())

# print(Quat.from_rotation_vector(Vec3(0, 0, np.pi)))
# # print(Quat.from_euler(14.454, 11.225, 14.454))
# print(B.quaternion.to_euler() / np.pi * 180)
# print(A.quaternion.to_rotation_vector())

