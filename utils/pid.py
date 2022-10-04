class PID:
    def __init__(self, kp, ki, kd, base_control=0):
        """
        Simple PID Controller class.
        Assumes steps evenly spaced (I.e. derivative denominator is constant)
        base_control is "center" guessed value
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.init_control = base_control
        self.prev_error = 0
        self.integral = 0

    def control(self, target, measurement):
        error = target - measurement
        self.integral += error
        P = self.kp * error
        I = self.ki * self.integral
        D = self.kd * (error - self.prev_error)
        self.prev_error = error
        output_control = self.base_control + P + I + D
        return output_control