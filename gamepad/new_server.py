#!/usr/bin/env python3

from Adafruit_PWM_Servo_Driver import PWM
import time
import logging

pwm = PWM(0x40)
FREQ = 50
pwm.setPWMFreq(FREQ) # Apparently default servo is at 50Hz?

# Function to write some number of microseconds of pulse width
def writeMicroseconds(channel, us):
    if us < 0:
        logging.error("Negative time requested")
        us = 0

    us_per_second = 1e6
    us_per_pulse = us_per_second / FREQ
    us = min(us_per_pulse - 1, us)

    frac = us / us_per_pulse
    resolution = 4096
    end = int(frac * resolution)
    pwm.setPWM(channel, 0, end)
