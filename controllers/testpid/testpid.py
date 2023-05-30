"""testpid controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from PID import PID
import math
# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = 32
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
motor_left = robot.getDevice('left wheel motor')
motor_right = robot.getDevice('right wheel motor')
motor_left.setPosition(float('inf'))
motor_right.setPosition(float('inf'))

# motor_left
# motor_right.setMax
gps = robot.getDevice('gps')
gps.enable(timestep)

pid = PID(16.199999999999964, 19.0, 1.0500000000000007, SetPoint = 0.5)
# pid = PID(1, 0.5, 0.01, SetPoint = 0.5)
pid.clear()

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    val = gps.getValues()[0]
    pid.update(val) 
    output = pid.output
    if abs(pid.output)>6.28:
        if pid.output < 0:
            output = -6.28
        else :
            output = 6.28
    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    motor_left.setVelocity(output)
    motor_right.setVelocity(output)
    pass

# Enter here exit cleanup code.
