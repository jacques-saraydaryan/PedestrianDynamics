# This Python file uses the following encoding: utf-8
import sys
import numpy as np

class Pedestrian():
    def __init__(self,id):
        self.id = id
        self.mass = 0
        self.radius = 0
        self.desired_velocity = 0
        self.maximum_velocity = 0
        self.relaxation_time = 0
        self.target_point = [0,0]
        self.position = [0,0]
        self.velocity = [0,0]
        self.braking_chance = 0

        self.average_velocity = [0,0]
        self.lambda_f = 1.0

        self.total_force = [0,0]
        self.prefered_force = [0,0]
        self.repulsive_force = [0,0]
        self.wall_force = [0,0]
        self.wall_point = [0,0]


    def set_mass(self,mass):
        self.mass = mass

    def set_radius(self,radius):
        self.radius = radius

    def set_desiredvelocity(self,desired_velocity):
        self.desired_velocity = desired_velocity

    def set_maximumvelocity(self,maximum_velocity):
        self.maximum_velocity = maximum_velocity

    def set_relaxationtime(self,relaxation_time):
        self.relaxation_time = relaxation_time

    def set_targetpoint(self,target_point):
        self.target_point = target_point

    def set_position(self,position):
        self.position = position

    def set_averagevelocity(self,velocity):
        self.average_velocity = velocity

    def set_velocity(self):
        dir = np.subtract(self.target_point,self.position)
        desired_dir = dir/np.linalg.norm(dir)
        self.velocity = np.multiply(desired_dir,self.maximum_velocity)

    def set_brakevelocity(self):
        self.velocity = [0,0]

    def set_zeroforce(self):
        self.total_force = [0,0]
        self.repulsive_force = [0,0]
        self.prefered_force = [0,0]

    def set_preferedforce(self,force):
        self.prefered_force = force

    def set_repulsiveforce(self,force):
        self.repulsive_force = force

    def set_wallforce(self,force):
        self.wall_force = force

    def set_wallpoint(self,point):
        self.wall_point = point



    def calculate_force(self):
        self.total_force = self.prefered_force+self.repulsive_force+self.wall_force



    def update_position(self,timestep):
        self.position = np.add(self.position,np.multiply(self.velocity,timestep))

    def update_velocity(self,timestep):
        self.calculate_force()
        self.velocity = np.add(self.velocity,np.multiply(self.total_force,timestep))
