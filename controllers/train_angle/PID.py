import time


class PID:
    def __init__(self, Kp=1, Ki=1.0, Kd=1, SetPoint=None):

        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.cur_time = time.time()
        self.pre_time = self.cur_time
        self.SetPoint = SetPoint
        self.delta_time = 0

        self.clear()

    def clear(self):

        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0
        # self.delta_time = 0
        self.error = 0

        self.output = 0.0

    def update(self, feedback_value):
        self.error = self.SetPoint - feedback_value

        self.cur_time = time.time()
        delta_error = self.error - self.last_error
        self.dt = self.cur_time - self.pre_time

        self.PTerm = self.Kp * self.error
        self.ITerm += self.error * self.dt

        self.DTerm = 0.0
        if self.dt > 0:
            self.DTerm = delta_error / self.dt

        self.last_error = self.error
        self.pre_time = self.cur_time

        self.mv = self.PTerm + self.Ki * self.ITerm + self.Kd * self.DTerm
