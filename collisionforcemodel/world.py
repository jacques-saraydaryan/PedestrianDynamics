#**********************************************************************************
# CLASS World - A Class with stores all Pedestrians and does necessary Calculations
#**********************************************************************************

import matplotlib.pyplot as plt
import numpy as np


from .pedestrian import Pedestrian

class World(object):
    def __init__(self):
        # Set Simulation Standard Parameters
        self.height = 25                    # World Height
        self.width = 25                     # World Width
        self.pedestrians=[]                 # List of Pedestrian class
        self.spawnpoints = [0,0,0,0]        # Spawn points provided as input (x1,y1,x2,y2)
        self.delta_t = 0                    # Time Step for simulation (Stability at 0.01)
        self.pedestrian_flux = []           # List of Pedestrian flux given as input
        self.targetpoints = [0,0,0,0]       # Target points provided as input(x1,y1,x2,y2)
        self.pedestrian_fluxright = 0       # Pedestrian flux at 'C' given as input
        self.pedestrian_fluxup = 0          # Pedestrian flux at 'A' given as input

        self.mass = 60                      # Mass of Pedestrian given as input (default:60 kg)
        self.desired_velocity = 0           # Desired velocity of the Pedestrian given as input
        self.maximum_velocity = 0           # Maximum Velocity of the Pedestrian given as input

        self.id = 0                         # System generated id for Pedestrians
        self.print_png = 0                  # PRINT CONTROL from input

        #For Force Calculations Constants
        #Preferred Force Constants
        self.relaxation_time = 0.5

        #Repulsive Force Constants
        self.social_force_constant = 2000
        self.body_force_constant = 12000
        self.friction_force_constant = 24000
        self.relaxation_length = 0.3
        self.max_acceleration = 2
        self.threshold = 2.0


        #Wall Force Constants
        self.obstacle_force_constant = 50
        self.sigma_obst = 0.2

    #---------------------------------------------------------------------------------------
    #***************** Setter Functions for Class Variables*********************************
    #---------------------------------------------------------------------------------------
    def set_height(self,height):
        self.height = height

    def set_width(self,width):
        self.width = width

    def set_pedestrianmass(self,mass):
        self.mass = mass

    def set_fluxright(self,flux):
        self.pedestrian_fluxright = flux

    def set_fluxup(self,flux):
        self.pedestrian_fluxup = flux

    def set_desiredvelocity(self,vel):
        self.desired_velocity = vel

    def set_maxvelocity(self,vel):
        self.maximum_velocity = vel

    def set_printpng(self,index):
        self.print_png = index

    def set_point1x(self,point):
        self.spawnpoints[0] = point

    def set_point1y(self,point):
        self.spawnpoints[1] = point

    def set_point2x(self,point):
        self.spawnpoints[2] = point

    def set_point2y(self,point):
        self.spawnpoints[3] = point

    def set_target1x(self,point):
        self.targetpoints[0] = point

    def set_target1y(self,point):
        self.targetpoints[1] = point

    def set_target2x(self,point):
        self.targetpoints[2] = point

    def set_target2y(self,point):
        self.targetpoints[3] = point

    def set_pedflux1(self,flux):
        self.pedestrian_flux.append(flux)

    def set_pedflux2(self,flux):
        self.pedestrian_flux.append(flux)

    def set_delta_t(self,time):
        self.delta_t = time


    #---------------------------------------------------------------------------------------
    #********************** Functions for Calculation***************************************
    #---------------------------------------------------------------------------------------
    # Get spawn points @source = [1=='C', 2=='A']
    def get_spawnpoint(self,source):
        if source == 1:
            return [self.spawnpoints[0],self.spawnpoints[1]]
        elif source == 2:
            return [self.spawnpoints[2],self.spawnpoints[3]]

    # Get Target points @index = [1=='C', 2=='A']
    def get_targetpoint(self,index):
        if index == 1:
            return [self.targetpoints[0],self.targetpoints[1]]
        elif index == 2:
            return [self.targetpoints[2],self.targetpoints[3]]


    # Add a pedestrian to the LIST [PEDESTRIANS] @spawnindex,@targetindex = [1=='C',2=='A']
    def add_pedestrian(self,spawnindex,targetindex):
        if spawnindex == 1:
            self.pedestrians.append(self.spawn_pedestrian(self.id,spawnindex,targetindex))
        if spawnindex == 2:
            self.pedestrians.append(self.spawn_pedestrian(self.id,spawnindex,targetindex))
        self.id+=1

    # Spawn a Pedestrian at given point @source = Unique id, @spawnindex,@targetindex = [1=='C',2=='A']
    def spawn_pedestrian(self,source,spawnindex,targetindex):
        p = Pedestrian(source)                                      # Creates Pedestrian with unique id
        p.set_mass(self.mass)                                       # Set Pedestrian Mass from input
        p.set_desiredvelocity(self.desired_velocity)                # Set Preferred Velocity from input
        p.set_maximumvelocity(self.maximum_velocity)                # Set Maximum Velocity from input
        p.set_targetpoint(self.get_targetpoint(targetindex))        # Set Target point from input
        p.set_position(self.get_spawnpoint(spawnindex))             # Set Spawn point from input
        p.set_velocity()                                            # Set Velocity based on Maximum Velocity
        p.set_direction(spawnindex)                                 # Set spawn index for correction
        return p                                                    # Returns a Class:Pedestrian


    # Calculate Average Velocity for the World of Pedestrians
    def calc_average_velocity(self):
        avg_velx = 0
        avg_vely = 0
        for p in self.pedestrians:
            avg_velx += p.velocity[0]
            avg_vely += p.velocity[1]
        avg_velx /= len(self.pedestrians)
        avg_vely /= len(self.pedestrians)
        for p in self.pedestrians:
            p.set_averagevelocity([avg_velx,avg_vely])

    # Calculate and return Unit Vector for given two points
    def unit_vector(self,target,position):
            vector = np.subtract(target,position)
            v_hat = vector/np.linalg.norm(vector)
            return v_hat

    # Calculate Preferred Force = (mass/tau)*(preferred Velocity - Velocity)
    # @p: a Pedestrian class element from the List=[PEDESTRIANS]
    def calculate_preferredforce(self,p):
            veldir = np.multiply(self.unit_vector(p.target_point,p.position),p.maximum_velocity)
            term2 = np.multiply(p.average_velocity,(1-p.lambda_f))
            direc = np.subtract(p.target_point,p.position)
            desired_dir = direc/np.linalg.norm(direc)
            prefered_velocity = np.multiply(desired_dir,self.desired_velocity)
            prefered_force = np.multiply(np.subtract(prefered_velocity,p.velocity),p.mass/p.lambda_f)
            p.set_preferedforce(prefered_force)

    # Calculate Repulsive Force = Social_force + Physical_force
    # @p: a Pedestrian class element from the LIST=[PEDESTRIANS]
    def calculate_socialrepulsiveforce(self,p):
        distance = []

        #Iterate over LIST=[PEDESTRIANS] to find relative forces
        for r in self.pedestrians:
            #Reduce calculations by skipping relative_self
            if p.id == r.id:
                continue

            dist = np.subtract(r.position,p.position)
            distance = np.linalg.norm(dist)

            #Reduce calculations by skipping far away pedestrians
            if distance > self.threshold:
                continue

            overlap = 2*p.radius-distance
            normal = -dist/distance
            tangent=[-normal[1],normal[0]]

            #Calculate Social Force
            social_const = self.social_force_constant*np.exp(overlap/self.relaxation_length)
            social_force = np.multiply(normal,social_const)

            force_pushfric = [0,0]

            #Friction Force assumed to be present only on contact
            if distance < 0.5:
                #Calculate Pushing Force
                pushing_const = self.body_force_constant*overlap
                pushing_force = np.multiply(normal,pushing_const)

                #Calculate Friction Force
                fric_const =np.multiply(tangent,self.friction_force_constant*overlap)
                term2 = np.multiply(tangent,np.subtract(r.velocity,p.velocity))
                friction_force = np.multiply(fric_const,term2)

                #Normalize Physical Forces
                push_fric = np.add(pushing_force,friction_force)
                unit_pushfric = push_fric/np.linalg.norm(push_fric)
                force_pushfric = np.multiply(unit_pushfric,self.max_acceleration)

            #Calculate Repulsive force on Pedestrian
            repulsive_force = np.add(social_force,force_pushfric)

            #Write the Social Repulsive Force acting on individual Pedestrian
            p.set_repulsiveforce(np.add(p.repulsive_force,repulsive_force))

    # Return the nearest wall position @point - Pedestrian position @target - Pedestrian target
    # Only works for the given case (Weak Geometry Constraints for Code)
    def get_nearest_wall(self,point,target):
        if target[0] > target[1]:
            dist1 = np.linalg.norm(np.subtract([point[0],10],point))
            dist2 = np.linalg.norm(np.subtract([point[0],15],point))
            if dist1 > dist2:
                return[point[0],15]
            else:
                return[point[0],10]
        if target[1] > target[0]:
            dist1 = np.linalg.norm(np.subtract([10,point[1]],point))
            dist2 = np.linalg.norm(np.subtract([15,point[1]],point))
            if dist1 > dist2:
                return [15,point[1]]
            else:
                return [10,point[1]]

    # Calculate Wall Force = F0*rij*exp(-|rin|/sigma)
    # @p: a Pedestrian class element from the LIST=[PEDESTRIANS]
    def calculate_wallforce(self,p):
        wall_point = self.get_nearest_wall(p.position,p.target_point)
        p.set_wallpoint(wall_point)
        dist = np.subtract(wall_point,p.position)
        distance = np.linalg.norm(dist)
        if distance < 2:
            normal = -dist/distance
            social_const = self.obstacle_force_constant*np.exp(-distance/self.sigma_obst)
            wall_force = np.multiply(normal,social_const)
            p.set_wallforce(np.add(p.wall_force,wall_force))
        else:
            p.set_wallforce([0,0])


    # Simulate step for a given timestep
    def simulate(self):
        for p in self.pedestrians:
            p.set_zeroforce()                       #Set all forces to zero at a time step
            self.calculate_preferredforce(p)        #Calculate Preferred Force
            self.calculate_socialrepulsiveforce(p)  #Calculate Social Force
            self.calculate_wallforce(p)             #Calculate Wall Force

        for p in self.pedestrians:
            p.update_velocity(self.delta_t)         #Update Pedestrian Velocity
            p.update_position(self.delta_t)         #Update Pedestrian Position

    # Eliminate Pedestrians who are out of geometry bound
    def eliminate_exited(self):
        eliminate=[]
        for i in range(len(self.pedestrians)):
            if (self.pedestrians[i].position[0] > self.width or self.pedestrians[i].position[1] > self.height):
                eliminate.append(i)
        for i in reversed(range(len(eliminate))):
            del self.pedestrians[eliminate[i]]

    # Scatter Plot of pedestrians with colors @blue - from 'A', @red - from 'C'
    def plot(self):
        plt.style.use('ggplot')
        figure = plt.figure(figsize=(6,6))
        ax = figure.add_subplot(1,1,1)
        ax.set_xlim([0,self.width])
        ax.set_ylim([0,self.height])
        ax.set_aspect('equal')

        xr=[]                                         #X_Position for Pedestrians spawned at 'C'
        yr=[]                                         #Y_Position for Pedestrians spawned at 'C'
        xu=[]                                         #X_Position for Pedestrians spawned at 'A'
        yu=[]                                         #Y_Position for Pedestrians spawned at 'A'
        for p in self.pedestrians:
            if p.target_point[0] > p.target_point[1]:
                xr.append(p.position[0])
                yr.append(p.position[1])
            else:
                xu.append(p.position[0])
                yu.append(p.position[1])

        ax.scatter(xr,yr,color='r', s=60)
        ax.scatter(xu,yu,color='b', s=60)

        #Set Border walls as Lines
        ax.plot([0,10],[15,15],color='black',linewidth=5)
        ax.plot([10,10],[15,25],color='black',linewidth=5)
        ax.plot([15,15],[15,25],color='black',linewidth=5)
        ax.plot([15,25],[15,15],color='black',linewidth=5)
        ax.plot([10,10],[0,10],color='black',linewidth=5)
        ax.plot([0,10],[10,10],color='black',linewidth=5)
        ax.plot([15,15],[0,10],color='black',linewidth=5)
        ax.plot([15,25],[10,10],color='black',linewidth=5)

        return figure
