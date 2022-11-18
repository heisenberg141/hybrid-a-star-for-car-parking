import numpy as np
import math 
import matplotlib.pyplot as plt
from random import shuffle


class AckermanSteering():
    def __init__(self,size,wheel_base_ratio, position=[0,0],theta = 0):
        
        self.type = "Ackerman"
        # [width, height]
        self.size = size
        self.wheel_base = self.size[0] * wheel_base_ratio
        
        # initialization
        self.theta = theta
        self.position = position
        self.psi = 0
        # [[x,y],theta, psi]
        self.state = [self.position, self.theta, self.psi]
        
        linear_motions = [4,-4]
        possible_input_psi = [
                                0,
                                round(np.deg2rad(-45),2),
                                round(np.deg2rad(45),2)
                              ]
        
        self.motions = list()
        for m in linear_motions:
            for n in possible_input_psi:
                self.motions.append([m,n])
        
    
    def get_next_states(self,current):
        next_states = list()
        for motion in self.motions:
            next_states.append(self.next_state(current,motion))
        return next_states
        
    def next_state(self, x, u, dt):
        # x = [[x,y], theta, psi]
        # u = [v, psi]
        x_dot = u[0]*np.cos(x[1])*dt
        y_dot = u[0]*np.sin(x[1])*dt
        theta_dot = u[0]/self.wheel_base*np.tan(u[1])*dt
        theta = self.normalizeAngle(x[1]+ theta_dot)
        return [[round(x[0][0] + x_dot,2), round(x[0][1] + y_dot,2)], round(theta,2), round(u[1],2)]

    def normalizeAngle(self,angle):
        """
        :param angle: (float)
        :return: (float) Angle in radian in [-pi, pi]
        """
        while angle > np.pi:
            angle -= 2.0 * np.pi

        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle

