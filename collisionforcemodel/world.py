"""
--------------------------------Class: World----------------------------------
The grid of interest, where the simulation is run
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from .pedestrian import Pedestrian

class World(object):
    def __init__(self):
        self.height = 0
        self.width = 0
        self.pedestrians = []
        self.pedestrian_count = 0
        self.time_step = 0
        self.spawn_1x = 0
        self.spawn_1y = 5
        self.spawn_2x = 10
        self.spawn_2y = 0

    def set_height(self,height):
        self.height = height

    def set_width(self,width):
        self.width = width

    def set_timestep(self,timestep):
        self.time_step = timestep

    def set_spawn_1x(self,pos):
        self.spawn_1x = pos

    def set_spawn_1y(self,pos):
        self.spawn_1y = pos

    def set_spawn_2x(self,pos):
        self.spawn_2x = pos

    def set_spawn_2y(self,pos):
        self.spawn_2y = pos


    def get_height(self):
        return self.height

    def get_width(self):
        return self.width

    def get_time_step(self):
        return self.time_step




    def get_spawn2(self):
        return list(self.spawn_2)

    def set_pedestrian(self,spoint,dir):
        if spoint == 1:
            self.pedestrians.append(Pedestrian(self.spawn_1x,self.spawn_1y,dir))
            self.pedestrian_count += 1
        else:
            self.pedestrians.append(Pedestrian(self.spawn_2x,self.spawn_2y,dir))
            self.pedestrian_count += 1


    def eliminate_exited(self):
        eliminate=[]

        for i in range(self.pedestrian_count):
            if (self.pedestrians[i].position[0] > self.width or self.pedestrians[i].position[1] > self.height):
                eliminate.append(i)

        for i in reversed(range(len(eliminate))):
            del self.pedestrians[eliminate[i]]
        self.pedestrian_count = len(self.pedestrians)

    def simulate(self):
        for i in range(self.pedestrian_count):
            if self.pedestrians[i].desired_direction[0] == 1:
                self.pedestrians[i].position[0] += self.time_step*self.pedestrians[i].max_velocity
            else:
                self.pedestrians[i].position[1] += self.time_step*self.pedestrians[i].max_velocity


    def plot(self):
        plt.style.use('ggplot')
        figure = plt.figure(figsize=(17,17))
        ax = figure.add_subplot(1,1,1)
        ax.set_xlim([0,self.width])
        ax.set_ylim([0,self.height])
        ax.set_aspect('equal')

        x=[]
        y=[]
        for i in range(self.pedestrian_count):
            x.append(self.pedestrians[i].position[0])
            y.append(self.pedestrians[i].position[1])

        ax.scatter(x,y,color='black')

        return figure



