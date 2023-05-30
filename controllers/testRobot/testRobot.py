from controller import Robot

from PID import PID
import math

# get the time step of the current world.
timestep = 32
robot = Robot()

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
motor_left = robot.getDevice('left wheel motor')
motor_right = robot.getDevice('right wheel motor')
motor_left.setPosition(float('inf'))
motor_right.setPosition(float('inf'))

gps = robot.getDevice('gps')
gps.enable(timestep)

compass = robot.getDevice('compass')
compass.enable(timestep)

pid = PID(8.799999999999986, 10.399999999999979, 0.08300000000000006, SetPoint = 1)
pid.clear()

pid_pos= [8.799999999999986, 10.399999999999979, 0.08300000000000006]
pid_angle = [14.199999999999967, 10.599999999999978, 0.1340000000000001]

mode = [1,2,3]
applied_mode = mode[1]

feedback = 0
def set_vel_limit():
    pid.update(feedback) 
    output = pid.output
    if abs(pid.output)>6.28:
        if pid.output < 0:
            output = -6.28
        else :
            output = 6.28
    return output

while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    position = gps.getValues()
    angle = math.atan2(compass.getValues()[0], compass.getValues()[1]) 

    if abs(position[0] -0) < 0.05 and abs(position[1] -0) <0.05:
        applied_mode = mode[0]
    elif (abs(position[0] -1) < 0.05 and abs(position[1] -0) <0.05) and abs(angle)<0.05:
        applied_mode = mode[1]
    elif (abs(position[0] -1) < 0.05 and abs(position[1] -0) <0.05) and abs(angle- math.pi/2)<0.005:
        applied_mode = mode[2]
    


    if applied_mode == mode[0]:
        pid.Kp, pid.Ki, pid.Kd = pid_pos 
        pid.SetPoint = 1
        feedback = position[0]    
        output =set_vel_limit()
        motor_left.setVelocity(output)
        motor_right.setVelocity(output) 
    elif applied_mode == mode[1]:
        pid.Kp, pid.Ki, pid.Kd = pid_angle 
        pid.SetPoint = math.pi/2
        feedback = angle
        output =set_vel_limit()
        motor_left.setVelocity(-output)
        motor_right.setVelocity(output)  
    elif applied_mode == mode[2]:
        pid.Kp, pid.Ki, pid.Kd = pid_pos 
        pid.SetPoint = 1
        feedback = position[1]  
        output =set_vel_limit()
        motor_left.setVelocity(output)
        motor_right.setVelocity(output)

    
    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    
    pass

# Enter here exit cleanup code.