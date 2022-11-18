import numpy as np
import math 
import matplotlib.pyplot as plt
from random import shuffle
import heapq
from vehicle import AckermanSteering
from environment import Environment
import cv2
import random

class Astar():
    def __init__( self, start=[10,10], goal=[255,255], time_step=1, dynamic = False):
        self.show_search  = True
        self.env = Environment(100)
        self.start_position = tuple(start)
        self.goal_position = tuple(goal)
        self.get_start_goal()
        self.time_step = [time_step]
        self.scale = 10
        self.size = self.env.environment.shape[0]*self.scale
        size = self.size
        self.search_pattern = cv2.resize(self.env.environment*255,(size,size),interpolation=cv2.INTER_AREA)
        self.search_pattern = cv2.cvtColor(self.search_pattern,cv2.COLOR_GRAY2BGR)
        self.car = AckermanSteering([19,9.5],0.6122)
        self.simulation_dt = 0.05
        self.n_iters = 0
        self.OPEN = []  # priority queue / OPEN set
        self.CLOSED = []  # CLOSED set / VISITED order
        self.PARENT = dict()  # recorded parent
        self.g = dict()  # cost to come
        self.MOTIONS = dict()
    
    def get_start_goal(self):
        return 
        start = (np.random.randint(0,256),np.random.randint(0,256))
        end = (np.random.randint(0,256),np.random.randint(0,256))
        while self.env.environment[start]==0 and self.env.environment[end]==0:
            start = (np.random.randint(0,256),np.random.randint(0,256))
            end = (np.random.randint(0,256),np.random.randint(0,256))
        self.goal_position = end
        self.start_position = start
            
    def search(self):
        print("Searching for a path...")
        size = self.size
        
        self.PARENT[self.start_position] = [self.start_position,[0,0],0]
        self.g[self.start_position] = 0
        self.g[self.goal_position] = math.inf
        self.MOTIONS[self.start_position] = [0,0]
        self.goal_threshold = 4
        # adding final_cost and the start node to self.OPEN in a heap like fashion
        state = [self.start_position,self.car.theta,0]
        heapq.heappush(self.OPEN,
                        [self.g_h_sum(self.start_position),state])
        
        while self.OPEN:
            # print("getting current state")
            _,state = heapq.heappop(self.OPEN)
            current_position = (round(state[0][0]),round(state[0][1]))
            self.CLOSED.append(current_position)
            # print("parents: ", self.PARENTS)
            if self.is_goal(current_position):
                self.PARENT[self.goal_position] = [current_position,[0,0],state[1]]
                break
                self.extract_path(self.goal_position)
                break
            # print("getting next_state")
            for neigh_state,motion in self.get_valid_neigh(state):
                neigh_pos = (round(neigh_state[0][0]), round(neigh_state[0][1]))

                if neigh_pos not in self.g:
                    self.g[neigh_pos] = math.inf
                new_cost = self.g[current_position]+ self.distance(current_position, neigh_pos)
                if new_cost< self.g[neigh_pos]:
                    self.g[neigh_pos] = new_cost
                    self.PARENT[neigh_pos] = [current_position,motion,state[1]]
                    self.MOTIONS[neigh_pos] = motion
                    heapq.heappush(self.OPEN,
                                    [self.g_h_sum(neigh_pos), neigh_state])
                    if self.show_search:
                        self.update_search_pattern(current_position, neigh_pos)
            self.n_iters+=1
        
        
        # print(self.PARENT)
        # for parent in self.PARENT:
            # print(f'{parent}: {self.PARENT[parent]}')
        return(self.extract_path(self.goal_position))
    
    
    def update_search_pattern(self,current, neigh):
        color = [np.random.randint(0,255),np.random.randint(0,255),np.random.randint(0,255)]
        current_scaled = np.array(current)*self.scale
        neigh_scaled = np.array(neigh)*self.scale
        self.search_pattern = cv2.circle(self.search_pattern,(current_scaled[0],current_scaled[1]),int(0.4*self.scale),color,-1)
        self.search_pattern = cv2.circle(self.search_pattern,(neigh_scaled[0],neigh_scaled[1]),int(0.4*self.scale),color,-1)
        cv2.line(self.search_pattern, current_scaled, neigh_scaled, color,int(0.2*self.scale))
                        


    def get_valid_neigh(self,current_state):
        # print(current_state)
        state_motion = list()
        
        for motion in self.car.motions:
            valid_primitive = True
            primitive = self.get_primitive(current_state, motion)
            for state in primitive:
                if self.env.side>state[0][0]>=0 and self.env.side>state[0][1]>=0:
                    if self.env.is_colliding(state, self.car):
                        valid_primitive= False
                        break
                else:
                    valid_primitive = False
                    break
            
            if valid_primitive:
                state_motion.append([primitive[-1],motion])
        # random.shuffle(state_motion)
        return state_motion

    def get_primitive(self, current_state, motion):
        primitive = list()
        next_state = current_state
        for i in range (int(self.time_step[0]/self.simulation_dt)):
            primitive.append(next_state)
            next_state = self.car.next_state(next_state, motion, self.simulation_dt)
        # print(primitive)
        return primitive
            
    def is_goal(self, current_position):
        if self.distance(current_position, self.goal_position)<self.goal_threshold:
            return True
        return False

    def g_h_sum(self, position):
        # g function is L2 dis b/w config and start_config.
        # h function is L2 dis b/w config and goal_config
        return self.g[position]+ self.distance(position,self.goal_position)
    
    def distance(self, p1, p2):
        return math.hypot(p1[0] - p2[0], p1[1] - p2[1])
    
    def extract_path(self, PARENT):
        """
        Extract the path based on the PARENT set.
        :return: The planning path
        """
        
        s = self.goal_position
        path = []
        m = [0,0]
        motions =[]

        while True:
            motions.append(m)
             
            s_theta = [self.PARENT[s][0],self.PARENT[s][2]]
            s = s_theta[0]
            m = self.PARENT[s][1]
            path.append(s_theta)
            if s == self.start_position:
                path.reverse()
                motions.reverse()
                break
        print("FOUND!!")
        if self.show_search:
            cv2.imshow("search pattern", self.search_pattern)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        self.simulate(motions,path)
        return path
        
    def show(self,frame, dt):
        cv2.imshow('frame',frame)
        cv2.waitKey(dt)
    
    def simulate(self, motions,path):
        # print("FOUND!!")
        # print("FOUND!!")
        # cv2.imshow("search pattern", self.search_pattern)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

        current = self.start_position
        current_state = [[current[0],current[1]], 0, 0]
        scale = self.scale
        frame1 = self.env.get_frame(self.car, current_state, scale)

        # for pos in path:
        #     frame1 = cv2.circle(frame1,(pos[0][0]*scale,pos[0][1]*scale),scale,[255,0,0],-1)
        
        i = 0
        for motion in motions:
            current_state = path[i]
            if motion ==[0,0]:
                break
            i+=1
            primitive = self.get_primitive(current_state,motion)
            for state in primitive:
                if round(state[0][0])== path[i][0][0] and round(state[0][1])== path[i][0][1]:
                    break
                # frame = cv2.addWeighted(self.env.get_frame(self.car,state, scale),0.65,frame1,0.35,0)
                frame = self.env.get_frame(self.car,state, scale)
                # frame = cv2.circle(frame,(self.start_position[0]*scale,self.start_position[1]*scale),scale,[255,0,0],-1)
                # frame = cv2.circle(frame,(self.goal_position[0]*scale,self.goal_position[1]*scale),scale,[255,0,0],-1)
                self.show(frame,int(self.simulation_dt*100))

        # cv2.waitKey(0)
            
        # cv2.destroyAllWindows()
            

def main():
    pass
    




if __name__ == "__main__":
    main()