#!/usr/bin/env python3

# TODO: Add a button that lets you adjust the step size
import copy
from pynput.keyboard import Key, KeyCode, Listener
import rospy
import threading
from sensor_msgs.msg import Joy
import yaml

class KeyState:
    def __init__(self, key_filt=None):
        self._lock = threading.Lock()
        self._key_filt = key_filt
        self.reset()

    def reset(self):
        with self._lock:
            if self._key_filt is None:
                self._state = dict()
            else:
                self._state = { k: [False, False] for k in self._key_filt }

    def on_pressed(self, key):
        if self._key_filt is None or key in self._key_filt:
            with self._lock:
                self._state[key] = [True, True]
    
    def on_released(self, key):
        if self._key_filt is None or key in self._key_filt:
            with self._lock:
                if key in self._state and self._state[key][0]:
                    self._state[key][0] = False
                else:
                    print('warning: released key {} but not marked as pressed'.format(key))

    def get_data(self, unlatch=True):
        data = dict()
        with self._lock:
            for k in self._state.keys():
                data[k] = self._state[k][1]
                if unlatch:
                    self._state[k][1] = self._state[k][0]
            return data


KEYS_FORWARD = { KeyCode.from_char('w') }
KEYS_BACK = { KeyCode.from_char('s') }
KEYS_LEFT = { KeyCode.from_char('a') }
KEYS_RIGHT = { KeyCode.from_char('d') }

KEYS_UP = { Key.space}
KEYS_DOWN = { Key.shift}
   
KEYS_OPEN = { KeyCode.from_char('o') }
KEYS_CLOSE = { KeyCode.from_char('c') }

KEYS_USED = KEYS_UP | KEYS_DOWN | KEYS_FORWARD | KEYS_BACK | KEYS_LEFT | KEYS_RIGHT | KEYS_OPEN | KEYS_CLOSE
def get_message_from_key_state(state):
    x_axis = any(state[k] for k in KEYS_RIGHT) - any(state[k] for k in KEYS_LEFT)
    y_axis = any(state[k] for k in KEYS_FORWARD) - any(state[k] for k in KEYS_BACK)
    z_axis = any(state[k] for k in KEYS_UP) - any(state[k] for k in KEYS_DOWN)
    
    gripper = any(state[k] for k in KEYS_OPEN) - any(state[k] for k in KEYS_CLOSE)

    return Joy(
        axes = [-x_axis, -y_axis, z_axis],
        buttons = [gripper]
    ), 10

def main():
    rospy.init_node('keyboard_to_joystick', anonymous=True)

    key_state = KeyState(KEYS_USED)
    listener = Listener(
        on_press=key_state.on_pressed,
        on_release=key_state.on_released)  
    listener.start()

    pub = rospy.Publisher('joy', Joy, queue_size=1)
    
    hold = 0
    def publish_msg(_):
        nonlocal hold
        if hold > 0:
            hold -= 1
        else:
            msg, hold = get_message_from_key_state(key_state.get_data())
        # pub.publish(get_message_from_key_state(key_state.get_data()))
            pub.publish(msg)
        # rospy.sleep(1)

        # print("key pressed!")

    pub_rate = rospy.get_param('pub_rate', 20.)
   
    timer = rospy.Timer(rospy.Duration(1./pub_rate), publish_msg)

    print('Ready')
    rospy.spin()

    # clean up
    listener.stop()
    listener.join()

if __name__ == "__main__":
    main()


