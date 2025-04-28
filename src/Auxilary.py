import math 

class Auxilary:

    

    def __init__(self):
        return
    @staticmethod
    def clamp(value, min_value, max_value):
        return max(min_value, min(value, max_value))
    @staticmethod
    def rangeCheck(value, min_Value, max_value):
        return value >= min_Value and value <= max_value
    
class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.previous_error = 0
        self.integral = 0

    def compute(self, process_variable, dt):
        # Calculate error
        error = self.setpoint - process_variable
        
        # Proportional term
        P_out = self.Kp * error
        
        # Integral term
        self.integral += error * dt
        I_out = self.Ki * self.integral
        
        # Derivative term
        derivative = (error - self.previous_error) / dt #Delta-Error / dt (given)vc fvvfcvcf
        D_out = self.Kd * derivative
        
        # Compute total output
        output = P_out + I_out + D_out
        
        # Update previous error
        self.previous_error = error
        
        return output
