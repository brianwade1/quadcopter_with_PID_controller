# PID Controller for quadcopter

# import libraries
import numpy as np
import math

class PID_Controller():

    def __init__(self, Kp, Kd, Ki, Ki_sat, dt):
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki
        self.Ki_sat = Ki_sat
        self.dt = dt
        
        # Integration total
        self.int = [0., 0., 0.]

    def control_update(self, pos_error, vel_error):
        
        #Update integral controller
        self.int += pos_error * self.dt

        #Prevent windup
        over_mag = np.argwhere(np.array(self.int) > np.array(self.Ki_sat))
        if over_mag.size != 0:
            for i in range(over_mag.size):
                mag = abs(self.int[over_mag[i][0]]) #get magnitude to find sign (direction)
                self.int[over_mag[i][0]] = (self.int[over_mag[i][0]] / mag) * self.Ki_sat[over_mag[i][0]] #maintain direction (sign) but limit to saturation 

        
        #Calculate controller input for desired acceleration
        des_acc = self.Kp * pos_error + self.Ki * self.int + self.Kd * vel_error
        return des_acc

