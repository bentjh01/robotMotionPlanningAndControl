import numpy as numpy

class pid:
    def __init__(self):
        self.Kp = 0.
        self.Ki = 0.
        self.Kd = 0.
        self.bias = 0.

        self.integral = 0.
        self.last_error = 0.
        self.time_step = 0.1
        self.control_signal = 0.

    def update(self, feedback):
        error = self.set_point - feedback

        P_term = self.Kp * error

        self.integral += error * self.dt
        I_term = self.Ki * self.integral

        differential = (error - self.last_error)/self.dt
        D_term = self.Kd * differential

        self.control_signal = P_term + I_term + D_term + self.bias

    