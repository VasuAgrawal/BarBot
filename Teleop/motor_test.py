from Adafruit_PWM_Servo_Driver import PWM
import time

pwm = PWM(0x40)
pwm.setPWMFreq(50)

LEFT = 0
RIGHT = 3

def to_out(val):
    pulseLength = 1000000            # 1,000,000 us per second
    pulseLength //= 50                # Hertz
    pulseLength //= 4096              # 12 bits of rsolution
    val *= 1000
    val //= pulseLength
    return val

while True:
    pwm.setPWM(LEFT,  0, to_out(1300))
    pwm.setPWM(RIGHT, 0, to_out(1300))
    time.sleep(1)
    
    pwm.setPWM(LEFT,  0, to_out(1500))
    pwm.setPWM(RIGHT, 0, to_out(1500))
    time.sleep(1)

