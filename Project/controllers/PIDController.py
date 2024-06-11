class PIDController:
    def __init__(self, Kp, Ki, Kd, output_min, output_max):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.output_min = output_min
        self.output_max = output_max
        self.prev_error = 0
        self.integral = 0

    def get_control_command(self, current_error, dt):
        if dt <= 0:
            dt = 0.001  # Set a small non-zero value to avoid division by zero
        derivative = (current_error - self.prev_error) / dt
        self.integral += current_error * dt
        output = self.Kp * current_error + self.Ki * self.integral + self.Kd * derivative
        output = max(min(output, self.output_max), self.output_min)
        self.prev_error = current_error
        return output

