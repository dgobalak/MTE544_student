from rclpy.time import Time
from utilities import Logger

# Controller types
P = 0    # Proportional
PD = 1   # Proportional-Derivative
PI = 2   # Proportional-Integral
PID = 3  # Proportional-Integral-Derivative

class PID_ctrl:
    
    def __init__(self, type_, kp=1.2, kv=0.8, ki=0.2, history_length=3, filename_="errors.csv"):
        self.history_length = history_length
        self.history = []
        self.type = type_

        # Controller gains
        self.kp = kp    # Proportional gain
        self.kv = kv    # Derivative gain
        self.ki = ki    # Integral gain
        
        self.logger=Logger(filename_)
        # Remeber that you are writing to the file named filename_ or errors.csv the following:
            # error, error_dot, error_int and time stamp

    
    def update(self, stamped_error, status):
        if status == False:
            self.__update(stamped_error)
            return 0.0
        else:
            return self.__update(stamped_error)

    def __update(self, stamped_error):
        latest_error = stamped_error[0]
        stamp = stamped_error[1]
        
        self.history.append(stamped_error)
        
        if len(self.history) > self.history_length:
            self.history.pop(0)
        
        # If insufficient data points, use only the proportional gain
        if len(self.history) != self.history_length:
            return self.kp * latest_error
        
        # Compute the error derivative
        dt_avg = 0
        error_dot = 0
        
        for i in range(1, len(self.history)):
            t0 = self.history[i-1][1]
            t1 = self.history[i][1]
            
            dt = (t1 - t0) / 1e9
            dt_avg += dt

            # Calculate the error_dot
            error_dot += (self.history[i][0] - self.history[i-1][0]) / dt
        
        error_dot /= len(self.history)
        dt_avg /= len(self.history)
        
        # Compute the error integral
        sum_ = 0
        for hist in self.history:
            sum_ += hist[0]
        
        error_int = sum_ * dt_avg
        
        # Log your errors: error, error_dot, error_int, timestamp
        self.logger.log_values([latest_error, error_dot, error_int, stamp])
        
        # Implement the P-controller
        if self.type == P:
            return self.kp * latest_error
        
        # Implement the PD-controller
        elif self.type == PD:
            return self.kp * latest_error + self.kv * error_dot
        
        # Implement the PI-controller
        elif self.type == PI:
            return self.kp * latest_error + self.ki * error_int
        
        # Implement the PID-controller
        elif self.type == PID:
            return self.kp * latest_error + self.kv * error_dot + self.ki * error_int
