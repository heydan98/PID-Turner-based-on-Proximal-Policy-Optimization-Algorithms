import time

class PID:
    """PID Controller
    """

    def __init__(self, P=1, I=1.0, D=1, SetPoint = None, current_time=None):

        self.Kp = P
        self.Ki = I
        self.Kd = D

        self.sample_time = 0.00
        self.current_time = current_time if current_time is not None else time.time()
        self.last_time = self.current_time
        self.SetPoint = SetPoint
        self.delta_time =0

        self.clear()

    def clear(self):

        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0
        self.delta_time =0
        self.error = 0
        # Windup Guard
        self.int_error = 0.0
        self.windup_guard = 20.0

        self.output = 0.0

    def update(self, feedback_value, current_time=None):
        self.error = self.SetPoint - feedback_value

        self.current_time = current_time if current_time is not None else time.time()
        self.delta_time = self.current_time - self.last_time
        delta_error = self.error - self.last_error

        if (self.delta_time >= self.sample_time):
            self.PTerm = self.Kp * self.error
            self.ITerm += self.error * self.delta_time

            if (self.ITerm < -self.windup_guard):
                self.ITerm = -self.windup_guard
            elif (self.ITerm > self.windup_guard):
                self.ITerm = self.windup_guard

            self.DTerm = 0.0
            if self.delta_time > 0:
                self.DTerm = delta_error / self.delta_time

            # Remember last time and last error for next calculation
            self.last_time = self.current_time
            self.last_error = self.error

            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)
