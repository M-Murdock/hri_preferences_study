#!/usr/bin/env python3

# Copyright (c) 2017, Elaine Short, SIM Lab
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# 
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# 
# * Neither the name of the SIM Lab nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Get finger position: rosrun tf tf_echo world gripper_finger1_finger_link
# Start force control: rosservice call /j2s7s300_driver/in/start_force_control "{}" 
# roslaunch poli2_launch arm_table.launch

import rospy
from planning_scene_helper import PlanningSceneHelper

def m(inch):
    """meters from inches"""
    conv = 0.0254
    if type(inch) in (tuple, list) :
        return [x*conv for x in inch]
    return inch*conv

if __name__=="__main__":
    rospy.init_node("planning_scene_setup_tablebot")
    p = PlanningSceneHelper()

    p.remove("table")
    p.remove("left_wall")
    p.remove("back")
    p.remove("shelves_top")
    p.remove("shelves_middle")
    p.remove("shelves_bottom")
    p.remove("shelves_right_foot")
    p.remove("shelves_left_foot")

    robot_height = 0.63262

    table_z = 0.01#-0.04
    table_thickness = 0.2
    shelf_thickness = 0.01

    table_shelf_top_height= m(26)
    table_shelf_middle_height= m(16.5)
    table_shelf_bottom_height= m(6.7)
    foot_width = m(0.5)
    foot_height = m(25)

    shelf_center_y = -0.40#-0.55#-0.73
    shelf_center_x = -0.75 - m(5.1)#-0.66 - m(5.1)

    table_width = m(11)
    table_length = m(25)

    safety_buffer = 0.04 

    p.add_box("back", "base_link", size=(0.3, 5, 2), position=(-0.1, 0, 0+robot_height), color=(0,0,1.,0.4))
    p.add_box("left_wall", "base_link", size=(2, 0.3, 2), position=(0, 0.45, 0+robot_height), color=(0,0,1.,0.4))
    p.add_box("table", "base_link", size=(2, 5, table_thickness), position=(0, 0, table_z - (table_thickness/2)+robot_height-0.05), color=(0,0,1.,0.4))

    p.add_box("shelves_top", "base_link", size=(table_width+safety_buffer, table_length+safety_buffer, shelf_thickness+safety_buffer), position=(-(shelf_center_x), shelf_center_y, table_shelf_top_height+robot_height), color=(0.5,0,1.,0.4))
    p.add_box("shelves_middle", "base_link", size=(table_width+safety_buffer, table_length+safety_buffer, shelf_thickness+safety_buffer), position=(-(shelf_center_x), shelf_center_y, table_shelf_middle_height+robot_height), color=(0.5,0,1.,0.4))
    p.add_box("shelves_bottom", "base_link", size=(table_width+safety_buffer, table_length+safety_buffer, shelf_thickness+safety_buffer), position=(-(shelf_center_x), shelf_center_y, table_shelf_bottom_height+robot_height), color=(0.5,0,1.,0.4))

    p.add_box("shelves_right_foot", "base_link", size=(table_width+safety_buffer, foot_width+safety_buffer, foot_height+safety_buffer+0.1), position=(-(shelf_center_x), shelf_center_y+(table_length/2), 0.32+robot_height), color=(0.5,0,1.,0.4))
    p.add_box("shelves_left_foot", "base_link", size=(table_width+safety_buffer, foot_width+safety_buffer, foot_height+safety_buffer+0.1), position=(-(shelf_center_x), shelf_center_y-(table_length/2), 0.32+robot_height), color=(0.5,0,1.,0.4))
   