from PID_tunner import PID_tunner
from PPO_agent import PPOAgent, Transition


class custom_PID_tunner(PID_tunner):
    def __init__(self, kp, ki, kd, setpoint, episode_limit=1000, steps_per_episode=2000, pretrained_model = None):
        PID_tunner.__init__(self, kp, ki, kd, setpoint,
                            episode_limit=1000, steps_per_episode=2000, pretrained_model= None)
        self.sensors_actuators_config()

    def sensors_actuators_config(self):
        self.robot = self.getSelf()
        self.position_sensor = self.getDevice("gps")
        self.position_sensor.enable(self.timestep)

        self.wheels = []
        for wheel_name in ['left wheel motor', 'right wheel motor']:
            wheel = self.getDevice(wheel_name)
            wheel.setPosition(float('inf'))
            wheel.setVelocity(0.0)
            self.wheels.append(wheel)

    def is_done(self):

        cart_position = round(self.position_sensor.getValues()[0], 2)
        if (cart_position < self.SetPoint*1.1 and cart_position > self.SetPoint*0.9) and (abs(self.wheels[0].getVelocity()) < 0.02):
            return True
        return False

    def apply_action(self, action):

        action = int(action[0])

        if self.step_count == 0:
            self.Kp += self.alpha * (action == 0)
            self.Ki += self.alpha * (action == 1)
            self.Kd += self.alpha * 0.1 * (action == 2)
            if action == 4:
                self.Kp -= self.alpha
                self.Ki -= self.alpha
                self.Kd -= self.alpha * 0.1

            print(self.Kp, ",", self.Ki, ",", self.Kd)
        self.update(self.position_sensor.getValues()[0])
        output = self.mv
        if abs(self.mv) > 6.28:
            if self.mv < 0:
                output = -6.28
            else:
                output = 6.28
        for i in range(len(self.wheels)):
            self.wheels[i].setPosition(float('inf'))
            self.wheels[i].setVelocity(output)

env = custom_PID_tunner(kp = 1, ki = 0.5, kd = 0.01, setpoint=0.5,
                        episode_limit=1500, steps_per_episode=700, pretrained_model= None)
env.main()
