#!/usr/bin/env python

import copy
from pynput.keyboard import Key, Listener
import rospy
import threading

from sensor_msgs.msg import Joy

class KeyState:
    def __init__(self, key_filt=None):
        self._lock = threading.Lock()
        self._key_filt = key_filt
        self.reset()

    def reset(self):
        with self._lock:
            if self._key_filt is None:
                self._state = {}
            else:
                self._state = { k: False for k in self._key_filt }

    def on_pressed(self, key):
        if self._key_filt is None or key in self._key_filt:
            with self._lock:
                self._state[key] = True
    
    def on_released(self, key):
        if self.key_filt is None:
            with self._lock:
                if key in self._state:
                    del(self._state[key])
                else:
                    print('warning: released key {} but not marked as pressed'.format(key))
        elif key in self.key_filt:
            with self._lock:
                if not self._state[key]:
                    print('warning: released key {} but not marked as pressed'.format(key))
                else:
                    self._state[key] = False

    def get_data(self):
        with self._lock:
            return copy.deepcopy(self._state)

KEYS_USED = { Key.right, Key.left, Key.up, Key.down, Key.space }
def get_message_from_key_state(state):
    x_axis = 1.*state.get(Key.right, 0.) - 1.*state.get(Key.left, 0.)
    y_axis = 1.*state.get(Key.up, 0.) - 1.*state.get(Key.down, 0.)
    button = state.get(Key.space, 0)
    return Joy(
        axes = [x_axis, y_axis],
        buttons = [button]
    )

def main():
    rospy.init_node('key_to_joystick', anonymous=True)

    key_state = KeyState(KEYS_USED)
    listener = keyboard.Listener(
        on_press=key_state.on_pressed,
        on_release=key_state.on_released)  
    listener.start()

    pub = rospy.Publisher('joy', Joy, queue_size=1)
    
    def publish_msg(_):
        pub.publish(get_message_from_key_state(key_state.get_data()))

    pub_rate = rospy.get_param('pub_rate', 100.)
    timer = rospy.Timer(rospy.Duration(1./pub_rate), publish_msg)

    rospy.spin()

    # clean up
    listener.stop()
    listener.join()

if __name__ == "__main__":
    main()



