from PID_tunner import PID_tunner
from PPO_agent import PPOAgent, Transition
import math
class custom_PID_tunner(PID_tunner):
    def __init__(self, kp, ki, kd, setpoint, episode_limit=1000, steps_per_episode=2000, pretrained_model = None):
        PID_tunner.__init__(self, kp, ki, kd, setpoint,
                            episode_limit=1000, steps_per_episode=2000, pretrained_model= None)
        self.sensors_actuators_config()

        self.initial_angle = 0

    def sensors_actuators_config(self):
        self.robot = self.getSelf()
        self.angle_sensor = self.getDevice("compass")
        self.angle_sensor.enable(self.timestep)

        self.wheels = []
        for wheel_name in ['left wheel motor', 'right wheel motor']:
            wheel = self.getDevice(wheel_name)
            wheel.setPosition(float('inf'))
            wheel.setVelocity(0.0)
            self.wheels.append(wheel)

    def is_done(self):

        angle = math.atan2(self.angle_sensor.getValues()[0], self.angle_sensor.getValues()[1])
        if abs(angle - self.SetPoint)<= abs(self.initial_angle - self.SetPoint)*0.02 and self.mv < 0.000001:
            return True

        return False

    def apply_action(self, action):

        action = int(action[0])
        angle = math.atan2(self.angle_sensor.getValues()[0], self.angle_sensor.getValues()[1])

        if self.step_count == 0:
            self.Kp += self.alpha * (action == 0)
            self.Ki += self.alpha * (action == 1)
            self.Kd += self.alpha * 0.01 * (action == 2)
            if action == 4:
                self.Kp -= self.alpha
                self.Ki -= self.alpha
                self.Kd -= self.alpha * 0.01

            print(self.Kp, ",", self.Ki, ",", self.Kd)
        self.update(angle)

        output = self.mv
        if abs(self.mv) > 6.28:
            if self.mv < 0:
                output = -6.28
            else:
                output = 6.28
        
        self.wheels[0].setPosition(float('inf'))
        self.wheels[1].setPosition(float('inf'))
        
        self.wheels[0].setVelocity(-output)
        self.wheels[1].setVelocity(output)


env = custom_PID_tunner(kp = 1, ki = 0.5, kd = 0.01, setpoint=math.pi/2,
                        episode_limit=1500, steps_per_episode=700, 
                        pretrained_model= r"/home/heydan/AI_PRACTICE_FINAL/project_final/controllers/my_controller/models/model_800")
env.main()