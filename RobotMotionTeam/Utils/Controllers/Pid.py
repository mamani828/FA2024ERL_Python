class PID:
    def __init__(self, kp=0.1, ki=0.01, kd=0.05,dt=1/240.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0
        self.integral = 0
        self.dt = dt # Simulation time step (default is 1/240 seconds)

    def update(self, kp, ki, kd):
        """
        Update the PID constants.
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def calculateVelocity(self, error):
        """
        Compute the control output based on the error using PID.
        """
        self.integral += error * self.dt
        derivative = (error - self.previous_error) / self.dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return output