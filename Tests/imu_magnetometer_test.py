import logging
import sys
import time
import math

from Adafruit_BNO055 import BNO055

# Raspberry Pi configuration with serial UART and RST connected to GPIO 18:
bno = BNO055.BNO055(serial_port='/dev/ttyAMA0', rst=18)

# Enable verbose debug logging if -v is passed as a parameter.
if len(sys.argv) == 2 and sys.argv[1].lower() == '-v':
    logging.basicConfig(level=logging.DEBUG)

# Initialize the BNO055 and stop if something went wrong.
if not bno.begin():
    raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')

# Print system status and self test result.
status, self_test, error = bno.get_system_status()
print('System status: {0}'.format(status))
print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
# Print out an error if system status is in error mode.
if status == 0x01:
    print('System error: {0}'.format(error))
    print('See datasheet section 4.3.59 for the meaning.')

# Print BNO055 software revision and other diagnostic data.
sw, bl, accel, mag, gyro = bno.get_revision()
print('Software version:   {0}'.format(sw))
print('Bootloader version: {0}'.format(bl))
print('Accelerometer ID:   0x{0:02X}'.format(accel))
print('Magnetometer ID:    0x{0:02X}'.format(mag))
print('Gyroscope ID:       0x{0:02X}\n'.format(gyro))

print('Reading BNO055 data, press Ctrl-C to quit...')
while True:
	# Read the Euler angles for heading, roll, pitch (all in degrees).
    heading, roll, pitch = bno.read_euler()

    # Magnetometer data (in micro-Teslas):
    x,y,z = bno.read_magnetometer()

    #print('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}\tMagX={3:0.4f} MagY={4:0.4f} MagZ={5:0.4f}'.format(
    #      heading, roll, pitch, x, y, z))

    # Compute compass heading
    """
    if (y > 0):
    	compassHeading = 90 - math.atan2(x,y)*180/math.pi
    elif (y < 0):
    	compassHeading = 270 - math.atan2(x,y)*180/math.pi
    elif ((y == 0) and (x < 0)):
    	compassHeading = 180
    else:
    	compassHeading = 0
    """

    compassHeading = math.atan2(y,x);

    print('Compass Heading = {0}'.format(compassHeading))
