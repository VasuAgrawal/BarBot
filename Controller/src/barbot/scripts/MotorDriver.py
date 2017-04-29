#!/usr/bin/env python
from __future__ import division

import time
import threading

import rospy
from barbot.msg import Thruster
from barbot.msg import Mode


# Attempt to import PWM things
try:
    from Adafruit_PWM_Servo_Driver import PWM
    FAKE_FREQ = 48
    pwm = PWM(0x40)
    pwm.setPWMFreq(FAKE_FREQ)
except ImportError:
    rospy.logerr("Unable to import PWM module!")
    pwm = None


# TODO: turn this into command line arguments?
FREQ = 50
LEFT_CHANNEL = 0
RIGHT_CHANNEL = 3

values = (0.0, 0.0)
values_lock = threading.Lock()

values_teleop = (0.0, 0.0)
values_teleop_lock = threading.Lock()

mode = Mode.TELEOP
mode_lock = threading.Lock()

def value_to_motor_us(value):
    return int(1500 + value * 200)


def I2C_writer():

    us_per_second = 1000000 # us per second
    us_per_pulse = us_per_second / FREQ # us per pulse
    bits_per_pulse = 4096
    bits_per_us = bits_per_pulse / us_per_pulse

    while True:
        start = time.time()

        with mode_lock:
            _mode = mode

        if _mode == Mode.TELEOP:
            with values_teleop_lock:
                _values = values_teleop
        elif _mode == Mode.AUTON:
            with values_lock:
                _values = values

        left_val, right_val = _values
        left_us = value_to_motor_us(left_val)
        right_us = value_to_motor_us(right_val)

        left_out = int(left_us * bits_per_us)
        right_out = int(right_us * bits_per_us)

        rospy.loginfo("%s: Writing %d to channel %d", _mode, left_out, LEFT_CHANNEL)
        rospy.loginfo("%s: Writing %d to channel %d", _mode, right_out, RIGHT_CHANNEL)

        if pwm is not None:
            pwm.setPWM(LEFT_CHANNEL, 0, left_out)
            pwm.setPWM(RIGHT_CHANNEL, 0, right_out)

        if rospy.is_shutdown():
            return

        time.sleep(max(0, .2 - (time.time() - start)))


def handle_data(data):
    rospy.logdebug("Received %f to left, %f to right", data.left, data.right)
    
    global values
    with values_lock:
        values = (data.left, data.right)


def handle_data_teleop(data):
    rospy.logdebug("Received TELEOP %f to left, %f to right", data.left, 
            data.right)

    global values_teleop
    with values_teleop_lock:
        values_teleop = (data.left, data.right)


def handle_mode(data):
    rospy.loginfo("Received mode request: %s", data.mode)

    global mode
    with mode_lock:
        mode = data.mode


def main():
    threading.Thread(target=I2C_writer).start()
    rospy.init_node("Motor_controller", log_level=rospy.DEBUG)
    rospy.Subscriber("thruster", Thruster, handle_data)
    rospy.Subscriber("teleop", Thruster, handle_data_teleop)
    rospy.Subscriber("mode", Mode, handle_mode)
    rospy.spin()


if __name__ == "__main__":
    main()
