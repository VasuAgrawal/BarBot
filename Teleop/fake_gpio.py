import logging

BCM = "Broadcomm Configuration"
BOARD = "Sane Board Pin Configuration"

OUT = "Output"

def setmode(mode):
    logging.info(mode)

def setup(channel, mode):
    logging.info("Setting pin %d to mode %s", channel, mode)

def cleanup():
    logging.info("Cleaning up GPIO")

class PWM(object):
    def __init__(self, channel, frequency):
        logging.info("Initializing PWM on channel %d with frequency %f", 
                channel, frequency)
        self.channel = channel


    def ChangeFrequency(self, frequency):
        logging.info("Changed frequency of PWM on channel %d to %f", 
                self.channel, frequency)


    def ChangeDutyCycle(self, dc):
        logging.info("Changing duty cycle of PWM on channel %d to %f",
                self.channel, dc)


    def start(self, dc):
        logging.info("Starting PWM on channel %d with duty cycle %f",
                self.channel, dc)


    def stop(self):
        logging.info("Stopping PWM on channel %d", self.channel)

