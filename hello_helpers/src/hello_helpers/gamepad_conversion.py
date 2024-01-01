#!/usr/bin/env python3
from sensor_msgs.msg import Joy


stretch_gamepad_mapping = {
    'axes':{
        0:{'name':'left_stick_x', 'range':[-1,1]},
        1:{'name':'left_stick_y', 'range':[-1,1]},
        2:{'name':'right_stick_x', 'range':[-1,1]},
        3:{'name':'right_stick_y', 'range':[-1,1]},
        4:{'name':'left_trigger_pulled', 'range':[0,1]},
        5:{'name':'right_trigger_pulled', 'range':[0,1]}
        },
    'buttons':{
        0:{'name':'left_stick_button_pressed'},
        1:{'name':'right_stick_button_pressed'},
        2:{'name':'bottom_button_pressed'},
        3:{'name':'right_button_pressed'},
        4:{'name':'left_button_pressed'},
        5:{'name':'top_button_pressed'},
        6:{'name':'left_shoulder_button_pressed'},
        7:{'name':'right_shoulder_button_pressed'},
        8:{'name':'select_button_pressed'},
        9:{'name':'start_button_pressed'},
        10:{'name':'bottom_pad_pressed'},
        11:{'name':'top_pad_pressed'},
        12:{'name':'left_pad_pressed'},
        13:{'name':'right_pad_pressed'},
        }
    }

def get_default_gamepad_state():
    state = {'middle_led_ring_button_pressed': False, #This button should not be used
            'left_stick_x': 0,
            'left_stick_y': 0,
            'right_stick_x': 0,
            'right_stick_y': 0,
            'left_stick_button_pressed': False,
            'right_stick_button_pressed': False,
            'bottom_button_pressed': False,
            'top_button_pressed': False,
            'left_button_pressed': False,
            'right_button_pressed': False,
            'left_shoulder_button_pressed': False,
            'right_shoulder_button_pressed': False,
            'select_button_pressed': False,
            'start_button_pressed': False,
            'left_trigger_pulled': 0,
            'right_trigger_pulled': 0,
            'bottom_pad_pressed': False,
            'top_pad_pressed': False,
            'left_pad_pressed': False,
            'right_pad_pressed': False}
    return state

def get_default_joy_msg():
    out_msg = Joy()
    out_msg.axes = [0.0]*6
    out_msg.buttons = [0]*14
    return out_msg

def unpack_joy_to_gamepad_state(joy_msg):
    stretch_gamepad_state = get_default_gamepad_state()
    if len(joy_msg.axes)!= 6:
        print("Invalid Joy message Axes length received.")
        return stretch_gamepad_state
    if len(joy_msg.buttons)!= 14:
        print("Invalid Joy message Buttons length received.")
        return stretch_gamepad_state
    
    for i in range(len(joy_msg.axes)):
        name = stretch_gamepad_mapping['axes'][i]['name']
        r = stretch_gamepad_mapping['axes'][i]['range']
        value = joy_msg.axes[i]
        value = value if is_value_between_bounds(value,r) else 0
        stretch_gamepad_state[name] = value
        
    for i in range(len(joy_msg.buttons)):
        name = stretch_gamepad_mapping['buttons'][i]['name']
        value = True if joy_msg.buttons[i]==1 else False
        stretch_gamepad_state[name] = value
    return stretch_gamepad_state

def unpack_gamepad_state_to_joy(gamepad_state):
    msg = get_default_joy_msg()
    
    msg.axes[0] = gamepad_state['left_stick_x']
    msg.axes[1] = gamepad_state['left_stick_y']
    msg.axes[2] = gamepad_state['right_stick_x']
    msg.axes[3] = gamepad_state['right_stick_y']
    msg.axes[4] = gamepad_state['left_trigger_pulled']
    msg.axes[5] = gamepad_state['right_trigger_pulled']
    
    msg.buttons[0] = 1 if gamepad_state['left_stick_button_pressed'] else 0
    msg.buttons[1] = 1 if gamepad_state['right_stick_button_pressed'] else 0
    msg.buttons[2] = 1 if gamepad_state['bottom_button_pressed'] else 0
    msg.buttons[3] = 1 if gamepad_state['right_button_pressed'] else 0
    msg.buttons[4] = 1 if gamepad_state['left_button_pressed'] else 0
    msg.buttons[5] = 1 if gamepad_state['top_button_pressed'] else 0
    msg.buttons[6] = 1 if gamepad_state['left_shoulder_button_pressed'] else 0
    msg.buttons[7] = 1 if gamepad_state['right_shoulder_button_pressed'] else 0
    msg.buttons[8] = 1 if gamepad_state['select_button_pressed'] else 0
    msg.buttons[9] = 1 if gamepad_state['start_button_pressed'] else 0
    msg.buttons[10] = 1 if gamepad_state['bottom_pad_pressed'] else 0
    msg.buttons[11] = 1 if gamepad_state['top_pad_pressed'] else 0
    msg.buttons[12] = 1 if gamepad_state['left_pad_pressed'] else 0
    msg.buttons[13] = 1 if gamepad_state['right_pad_pressed'] else 0
    
    return msg
        
        
def is_value_between_bounds(value, r):
    return r[0] <= value <= r[1]
