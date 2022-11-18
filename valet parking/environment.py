import numpy as np
import cv2
from vehicle import AckermanSteering
# from grid_gen.obstacle_grid1 import obstacle_grid_generator

class Environment():
    def __init__(self,side):
        self.side = side
        # self.environment_generator = obstacle_grid_generator(side, 20)
        self.environment = np.ones([side, side], np.uint8)
        # self.environment_generator.place_obstacles()
        # self.environment = self.environment_generator.grid
        # self.environment  = np.ones_like(self.environment)
        environment = cv2.rectangle(self.environment,(40,30), (80,60),(0,0,0),-1)
        environment = cv2.rectangle(environment,(5,90), (25,99),(0,0,0),-1)
        self.environment = cv2.rectangle(environment,(55,90), (75,99),(0,0,0),-1)
        self.environment[:,0:2] = 0
        self.environment[:,side-2 : side] = 0
        self.environment[0:2,:] = 0
        self.environment[side-2 : side,:] = 0
        # cv2.imshow("map", self.environment*255)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        
        # self.env_c = np.ones_like(self.environment)

    def is_colliding(self, state, vehicle):
        veh_mask = np.zeros_like(self.environment, np.uint8)
        rect = self.get_veh_rect(vehicle, state, 1)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        env_mask = cv2.bitwise_not(self.environment*255)

        cv2.drawContours(veh_mask,[box],0,(255,255,255),-1)
        final_mask = cv2.bitwise_and(veh_mask, env_mask)
        # cv2.imshow("env_mask", env_mask)
        # cv2.imshow("veh_mask", veh_mask)
        # cv2.imshow("final_mask", final_mask)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        if cv2.countNonZero(final_mask) == 0:
            return False
        return True
        
    def get_veh_rect(self,vehicle, state, scale):
        rect = None
        # All this work for getting to the center of vehicle.
        if vehicle.type == "Ackerman":
            veh_size  = (vehicle.size[0]*scale, vehicle.size[1]*scale)
            rear_mid_pt = (state[0][0]*scale, state[0][1]*scale) 
            veh_mid_pt = (rear_mid_pt[0] + veh_size[0]/2*np.cos(state[1]), rear_mid_pt[1] + veh_size[0]/2*np.sin(state[1]))
            rect = (veh_mid_pt,veh_size,np.rad2deg(state[1]))
        return rect      
    
    def get_frame(self, vehicle, state, scale):
        rect = self.get_veh_rect(vehicle, state, scale)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        size = scale * self.environment.shape[0]
        env_c = cv2.resize(self.environment*255,(size,size),interpolation=cv2.INTER_AREA)
        env_c = cv2.cvtColor(env_c,cv2.COLOR_GRAY2BGR)
        cv2.drawContours(env_c,[box],0,(0,50,250), -1)
        return env_c
        

    
def main():
    env = Environment(100)
    
    car = AckermanSteering([5,2.5],0.7)
    theta_list = np.arange(0,2*np.pi,0.05)
    print("Press Control + C to snap out")
    
    while(1):
        for theta in theta_list:
            state = [[20,30],theta]
            scale = 5
            env.show(car, state, scale)
    
if __name__ == "__main__":
    main()