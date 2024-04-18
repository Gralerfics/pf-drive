#!/usr/bin/env python

import threading
import time

import rospy

from webots_ros.srv import get_bool, get_boolRequest, get_boolResponse
from controller_manager_msgs.srv import LoadController, LoadControllerRequest, LoadControllerResponse, UnloadController, UnloadControllerRequest, UnloadControllerResponse, SwitchController, SwitchControllerRequest, SwitchControllerResponse

import dearpygui.dearpygui as dpg


class Tb3WbtDashboard:
    def __init__(self, supervisor_srv = '/supervisor', controller_mgr_srv = '/controller_manager', controller_name = 'controller'):
        # private
        self.ready = threading.Event()
        
        # public
        self.supervisor_srv = supervisor_srv
        self.controller_mgr_srv = controller_mgr_srv
        self.controller_name = controller_name
        
        # services
        SERVICE_SIMULATION_RESET = self.supervisor_srv + '/simulation_reset'
        SERVICE_UNLOAD_CONTROLLER = self.controller_mgr_srv + '/unload_controller'
        SERVICE_LOAD_CONTROLLER = self.controller_mgr_srv + '/load_controller'
        SERVICE_SWITCH_CONTROLLER = self.controller_mgr_srv + '/switch_controller'
        rospy.wait_for_service(SERVICE_SIMULATION_RESET)
        rospy.wait_for_service(SERVICE_UNLOAD_CONTROLLER)
        rospy.wait_for_service(SERVICE_LOAD_CONTROLLER)
        rospy.wait_for_service(SERVICE_SWITCH_CONTROLLER)
        self.srv_simulation_reset = rospy.ServiceProxy(SERVICE_SIMULATION_RESET, get_bool)
        self.srv_unload_controller = rospy.ServiceProxy(SERVICE_UNLOAD_CONTROLLER, UnloadController)
        self.srv_load_controller = rospy.ServiceProxy(SERVICE_LOAD_CONTROLLER, LoadController)
        self.srv_switch_controller = rospy.ServiceProxy(SERVICE_SWITCH_CONTROLLER, SwitchController)
        
        # ready
        self.ready.set()
    
    def is_ready(self):
        return self.ready.is_set()
    
    def wait_until_ready(self):
        while not rospy.is_shutdown() and not self.is_ready():
            rospy.loginfo('Waiting for simulation dashboard ...')
            time.sleep(0.2)
        rospy.loginfo('Simulation dashboard is ready.')
    
    def reset_simulation(self):
        try:
            request = get_boolRequest(1)
            response = self.srv_simulation_reset(request)
        except rospy.ServiceException as e:
            rospy.logerr('Reset simulation failed: %s' % e)

    def unload_controller(self):
        try:
            request = UnloadControllerRequest(self.controller_name)
            response = self.srv_unload_controller(request)
        except rospy.ServiceException as e:
            rospy.logerr('Unload controller failed: %s' % e)

    def load_controller(self):
        try:
            request = LoadControllerRequest(self.controller_name)
            response = self.srv_load_controller(request)
        except rospy.ServiceException as e:
            rospy.logerr('Load controller failed: %s' % e)
    
    def stop_controller(self):
        try:
            request = SwitchControllerRequest(stop_controllers = [self.controller_name])
            response = self.srv_switch_controller(request)
        except rospy.ServiceException as e:
            rospy.logerr('Stop controller failed: %s' % e)

    def start_controller(self):
        try:
            request = SwitchControllerRequest(start_controllers = [self.controller_name])
            response = self.srv_switch_controller(request)
        except rospy.ServiceException as e:
            rospy.logerr('Start controller failed: %s' % e)
    
    def reset_all(self):
        self.reset_simulation()
        self.stop_controller()
        self.unload_controller()
        self.load_controller()
        self.start_controller()


def spin_func():
    rospy.spin()
    dpg.destroy_context()

rospy.init_node('tr_tb3_wbt_dashboard', anonymous = False)
dashboard = Tb3WbtDashboard('/tb3/supervisor', '/tb3/controller_manager', 'controller')
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

# dpg.start_dearpygui()
while dpg.is_dearpygui_running():
    dpg.render_dearpygui_frame()

