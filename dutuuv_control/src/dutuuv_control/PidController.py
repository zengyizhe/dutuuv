import time

class PidController():
    def __init__(self, kp, ki, kd, sat):
        
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.sat =sat

        self.flag = False
        self.prev_time = 0.0
        self.prev_error = 0.0
        self.p_term , self.i_term, self.d_term = 0.0, 0.0, 0.0


    def update(self, current_error, current_time):
        
        if not self.flag:
            self.flag = True
            self.p_term = self.kp * current_error
            output = self.p_term
            return output

        self.p_term = self.kp * current_error
        dt = max(1.0, current_time - self.prev_time)
        self.i_term += self.ki * current_error * dt
        delta_error = current_error - self.prev_error
        self.d_term = self.kd * delta_error / dt

        self.prev_time = current_time
        self.prev_error = current_error
        output = self.p_term + self.i_term + self.d_term

        if output > self.sat:
            self.i_term = 0.0
        
        return output









