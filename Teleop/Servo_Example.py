#!/usr/bin/python

from Adafruit_PWM_Servo_Driver import PWM
import time

# ===========================================================================
# Example Code
# ===========================================================================

# # Initialise the PWM device using the default address
# pwm = PWM(0x40)
# # Note if you'd like more debug output you can instead run:
# #pwm = PWM(0x40, debug=True)

# servoMin = 307 # Min pulse length out of 4096
# servoMax = 350  # Max pulse length out of 4096

# def setServoPulse(channel, pulse):
  # pulseLength = 1000000                   # 1,000,000 us per second
  # pulseLength /= 60                       # 60 Hz
  # print "%d us per period" % pulseLength
  # pulseLength /= 4096                     # 12 bits of resolution
  # print "%d us per bit" % pulseLength
  # pulse *= 1000
  # pulse /= pulseLength
  # pwm.setPWM(channel, 0, pulse)

# pwm.setPWMFreq(50)                        # Set frequency to 60 Hz
# while (True):
  # # Change speed of continuous servo on channel O
  # pwm.setPWM(0, 0, servoMin)
  # pwm.setPWM(3, 0, servoMin)
  # time.sleep(1)
  # pwm.setPWM(0, 0, servoMax)
  # pwm.setPWM(3, 0, servoMax)
  # time.sleep(1)

FAKE_FREQ = 48
FREQ = 50
pwm = PWM(0x40)
pwm.setPWMFreq(FAKE_FREQ)

us_per_second = 1000000 # us per second
us_per_pulse = us_per_second / FREQ # us per pulse
bits_per_pulse = 4096
bits_per_us = bits_per_pulse / us_per_pulse

def writeMicroseconds(channel, us):
    value = int(us * bits_per_us)
    print("Writing", value, "to channel", channel)
    pwm.setPWM(channel, 0, value)

while True:
    writeMicroseconds(0, 1700)
    writeMicroseconds(3, 1700)
    time.sleep(1)
    writeMicroseconds(0, 1500)
    writeMicroseconds(3, 1500)
    time.sleep(1)


# while True:
    # writeMicroseconds(0, 1600)
    # writeMicroseconds(3, 1500)
    # time.sleep(1)
