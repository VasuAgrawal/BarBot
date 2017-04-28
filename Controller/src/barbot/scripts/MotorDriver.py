#!/usr/bin/env python
from __future__ import division

import time
import threading

import rospy
from barbot.msg import Thruster
from Adafruit_PWM_Servo_Driver import PWM

# Various constants
FAKE_FREQ = 48
FREQ = 50
pwm = PWM(0x40)
pwm.setPWMFreq(FAKE_FREQ)

# TODO: turn this into command line arguments?
LEFT_CHANNEL = 0
RIGHT_CHANNEL = 3

values = (0.0, 0.0)
values_lock = threading.Lock()


def value_to_motor_us(value):
    return int(1500 + value * 200)


def I2C_writer():

    us_per_second = 1000000 # us per second
    us_per_pulse = us_per_second / FREQ # us per pulse
    bits_per_pulse = 4096
    bits_per_us = bits_per_pulse / us_per_pulse

    while True:
        start = time.time()

        global values_lock
        with values_lock:
            _values = values

        left_val, right_val = _values
        left_us = value_to_motor_us(left_val)
        right_us = value_to_motor_us(right_val)

        left_out = int(left_us * bits_per_us)
        right_out = int(right_us * bits_per_us)

        rospy.loginfo("Writing %d to channel %d", left_out, LEFT_CHANNEL)
        rospy.loginfo("Writing %d to channel %d", right_out, RIGHT_CHANNEL)
        pwm.setPWM(LEFT_CHANNEL, 0, left_out)
        pwm.setPWM(RIGHT_CHANNEL, 0, right_out)

        if rospy.is_shutdown():
            return

        time.sleep(max(0, .2 - (time.time() - start)))


def saveData(data):
    rospy.logdebug("Received %f to left, %f to right", data.left, data.right)
    
    global values
    with values_lock:
        values = (data.left, data.right)


def main():
    threading.Thread(target=I2C_writer).start()
    rospy.init_node("Motor_controller")
    rospy.Subscriber("thruster", Thruster, saveData)
    rospy.spin()

if __name__ == "__main__":
    main()
