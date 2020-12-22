import rospy
import numpy as np
import warnings

from input_handlers.UserInputListener import UserInputData
#from HydraListener import *
from RobotState import *

# finger_weighting = 0.2

#mapping from user input to action
#if we were to execute input via direct teleop
# class UserInputMapper(object):
#   def __init__(self, interface_listener, num_motion_modes=2, num_finger_modes=0):
#     self.num_motion_modes = num_motion_modes
#     self.num_finger_modes = num_finger_modes
#     self.interface_listener = interface_listener

#   def input_to_action(self, user_input_data, robot_state):
#     action_to_ret = Action()
#     #if the first button is was released, and we didn't activate hold, change modes
#     if user_input_data.button_changes[0] == -1 and user_input_data.buttons_held[0] != 1:
#       action_to_ret.switch_mode_to = robot_state.next_mode()
#       return action_to_ret
    
#     curr_robot_mode = robot_state.mode
#     #if we are in the first few modes, it is a end effector velocity command
#     if curr_robot_mode < self.num_motion_modes:
#       if self.num_motion_modes == 2:
#         if curr_robot_mode == 0:
#           action_to_ret.twist[:3] = self.interface_listener.translation_input_conversion(user_input_data.axes[0:3], robot_state)
#         else:
#           action_to_ret.twist[3:] = self.interface_listener.rotation_input_conversion(user_input_data.axes[0:3], robot_state)

#       elif self.num_motion_modes == 3:
#         if curr_robot_mode == 0:
#           action_to_ret.twist[:3] = self.interface_listener.translation_input_conversion(np.append(user_input_data.axes[0:2], 0.), robot_state)
#           #action_to_ret.move[0] *= -1.
#         elif curr_robot_mode == 1:
#           action_to_ret.twist[:3] = self.interface_listener.translation_input_conversion(np.append(np.zeros(2), user_input_data.axes[1]), robot_state)
#           rot_velocity = np.array([0, 0, -user_input_data.axes[0]])
#           action_to_ret.twist[3:] = self.interface_listener.rotation_input_conversion(rot_velocity, robot_state)
#         else:
#           rot_velocity = np.array([user_input_data.axes[1], -user_input_data.axes[0], 0.])
#           action_to_ret.twist[3:] = self.interface_listener.rotation_input_conversion(rot_velocity, robot_state)
#     else:
#       #both left-right and up-down control fingers. With kinova control, whichever input has higher
#       #magnitude overrides the other
#       axis_input_higher_mag = np.argmax(np.abs(user_input_data.axes[0:2]))
#       if robot_state.num_finger_dofs == 3 and axis_input_higher_mag == 1:
#         #if this is the jaco, and the user went up-down, only control two fingers
#         action_to_ret.finger_vel[0:2] = finger_weighting * user_input_data.axes[axis_input_higher_mag]
#       else:
#         action_to_ret.finger_vel[:] = finger_weighting * user_input_data.axes[axis_input_higher_mag]


#     return action_to_ret


## TODO: probably don't need this at all unless we want finger modes
class UserInputMapper:
    def __init__(self, input_profile):
        self._profile = input_profile

    @property
    def num_modes(self):
        return self._profile.num_modes

    def input_to_action(self, user_input_data, robot_state):
        # handle mode switch request
        if user_input_data.button_changes[0] == -1 and user_input_data.buttons_held[0] != 1:
            return Action(switch_mode_to = robot_state.next_mode())
        
        # handle velocity command
        elif robot_state.mode < self._profile.num_modes:
            return Action(twist = self._profile(robot_state.mode, user_input_data.axes, robot_state))

        # handle finger command
        # TODO

class InputProfile:
    def __init__(self, mappings):
        """
        Create an input profile function that accepts arguments (mode, axes) and returns robot twist

        Input format:
        mappings = [ mapping, mapping, ... ] x num_modes
        mapping = [ twist_0, twist_1, ... ] x num_axes
        twist_i = [ x y z r p y ]

        Twist is computed by first setting mapping = mappings[mode_id] then sum_ax mapping[ax_i] * ax

        Returns: function computing robot twist
        """

        # do some basic checks
        if not mappings:
            raise ValueError("mapping not specified")
        num_axes = len(mappings[0])
        for mapping in mappings:
            if len(mapping) != num_axes:
                raise ValueError("mapping has inconsistent length")
            for ax in mapping:
                if len(ax) != 6:
                    raise ValueError("mapping for ax has incorrect twist value")
        self._mappings = [ np.array(m) for m in mappings ]

    @property
    def num_modes(self):
        return len(self._mappings)

    def __call__(self, mode_id, axes, robot_state):
        # collect the twist values
        twist = np.dot(axes, self._mappings[mode_id])

        # transform the rotation twist into the frame of the end-effector
        ee_rot = robot_state.ee_trans[0:3, 0:3]
        twist[3:] = np.dot(ee_rot, twist[3:])
        
        return twist


# Some default profiles -- can easily specify more using this format
JOYSTICK_PROFILE_2D = InputProfile([
    [
        [ 0.2, 0., 0., 0., 0., 0.], # axis 0 -> x
        [ 0., 0.2, 0., 0., 0., 0.], # axis 1 -> y
        [ 0., 0., 0.2, 0., 0., 0.], # axis 2 -> z
    ],
    [
        [ 0., 0., 0., 0.4, 0., 0.], # axis 0 -> roll
        [ 0., 0., 0., 0., 0.4, 0.], # axis 1 -> pitch
        [ 0., 0., 0., 0., 0., 0.4], # axis 2 -> yaw
    ],
])

JOYSTICK_PROFILE_3D = InputProfile([
    [
        [ 0.2, 0., 0., 0., 0., 0.], # axis 0 -> x
        [ 0., 0.2, 0., 0., 0., 0.], # axis 1 -> y
        [ 0., 0., 0., 0., 0., 0.] # axis 2 -> unused
    ],
    [
        [ 0., 0., 0., 0., 0., 0.4], # axis 0 -> yaw
        [ 0., 0., 0.2, 0., 0., 0.], # axis 1 -> z
        [ 0., 0., 0., 0., 0., 0.] # axis 2 -> unused
    ],
    [
        [ 0., 0., 0., 0.4, 0., 0.], # axis 0 -> pitch
        [ 0., 0., 0., 0., 0.4, 0.], # axis 1 -> roll
        [ 0., 0., 0., 0., 0., 0.] # axis 2 -> unused
    ],
])

global _profiles
_profiles = {}

def register_profile(name, val):
    global _profiles
    if name in _profiles:
        warnings.warn('Name {} already registered as a profile, overwriting'.format(name))
    _profiles[name] = val

def create_profile(name, val):
    global _profiles
    if name in _profiles:
        warnings.warn('Name {} already registered as a profile, overwriting'.format(name))
    _profiles[name] = InputProfile(val)

def get_profile(name):
    global _profiles
    return _profiles[name]

def list_profiles():
    global _profiles
    return _profiles.keys()


register_profile('joystick_base_2d', JOYSTICK_PROFILE_2D)
register_profile('joystick_base_3d', JOYSTICK_PROFILE_3D)
