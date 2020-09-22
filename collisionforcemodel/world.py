# This Python file uses the following encoding: utf-8
import matplotlib.pyplot as plt
import numpy as np


from .pedestrian import Pedestrian

class World(object):
    def __init__(self):
        self.height = 25
        self.width = 25
        self.pedestrians=[]
        self.spawnpoints = []
        self.no_spawnpoints = 0
        self.delta_t = 0
        self.pedestrian_flux = []
        self.pedestrians = []
        self.no_targetpoints=0
        self.targetpoints = []
        self.pedestrian_fluxright = 0
        self.pedestrian_fluxup = 0

        self.mass = 60
        self.radius = 0.25
        self.desired_velocity = 0
        self.maximum_velocity = 0
        self.relaxation_time = 0
        self.id = 0
        self.threshold = 2.0
        self.relaxation_length = 0.3
        self.max_acceleration = 2

        self.social_force_constant = 2000
        self.body_force_constant = 12000
        self.friction_force_constant = 24000
        self.obstacle_force_constant = 50
        self.sigma_obst = 0.2


    def set_height(self,height):
        self.height = height

    def set_width(self,width):
        self.width = width

    def set_pedestrianmass(self,mass):
        self.mass = mass

    def set_pedestrianradius(self,radius):
        self.radius = radius

    def set_fluxright(self,flux):
        self.pedestrian_fluxright = flux

    def set_fluxup(self,flux):
        self.pedestrian_fluxup = flux

    def set_desiredvelocity(self,vel):
        self.desired_velocity = vel

    def set_maxvelocity(self,vel):
        self.maximum_velocity = vel

    def set_relaxationtime(self,time):
        self.relaxation_time = time

    def set_spawnpoints(points):
        self.spawnpoints.append(points)

    def set_spawnpoints(self,nop):
        self.no_spawnpoints = nop
        for i in range(nop*2):
            self.spawnpoints.append(0)

    def set_targetpoints(self,nop):
        self.no_targetpoints = nop
        for i in range(nop*2):
            self.targetpoints.append(0)

    def set_point1x(self,point):
        self.spawnpoints[0] = point

    def set_point1y(self,point):
        self.spawnpoints[1] = point

    def set_point2x(self,point):
        self.spawnpoints[2] = point

    def set_point2y(self,point):
        self.spawnpoints[3] = point

    def set_point3x(self,point):
        self.spawnpoints[4] = point

    def set_point3y(self,point):
        self.spawnpoints[5] = point
    def set_point4x(self,point):
        self.spawnpoints[6] = point

    def set_point4y(self,point):
        self.spawnpoints[7] = point

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

    def set_pedflux3(self,flux):
        self.pedestrian_flux.append(flux)

    def set_pedflux4(self,flux):
        self.pedestrian_flux.append(flux)

    def set_delta_t(self,time):
        self.delta_t = time


    def dummy(self):
        print(self.targetpoints)

    def get_spawnpoint(self,source):
        if source == 1:
            return [self.spawnpoints[0],self.spawnpoints[1]]
        elif source == 2:
            return [self.spawnpoints[2],self.spawnpoints[3]]



    def add_pedestrian(self,spawnindex,targetindex):
        if spawnindex == 1:
            self.pedestrians.append(self.spawn_pedestrian(self.id,1,targetindex))
        if spawnindex == 2:
            self.pedestrians.append(self.spawn_pedestrian(self.id,2,targetindex))
        self.id+=1

    def spawn_pedestrian(self,source,spawnindex,targetindex):
        p = Pedestrian(source)
        p.set_mass(self.mass)
        p.set_radius(self.radius)
        p.set_desiredvelocity(self.desired_velocity)
        p.set_maximumvelocity(self.maximum_velocity)
        p.set_relaxationtime(self.relaxation_time)
        p.set_targetpoint(self.get_targetpoint(targetindex))
        p.set_position(self.get_spawnpoint(spawnindex))
        p.set_velocity()
        return p


    def get_targetpoint(self,index):
        if index == 1:
            return [self.targetpoints[0],self.targetpoints[1]]
        elif index == 2:
            return [self.targetpoints[2],self.targetpoints[3]]


    def simulate(self):
        for p in self.pedestrians:
            p.set_zeroforce()
            self.calculate_preferredforce(p)
            self.calculate_socialrepulsiveforce(p)
            self.calculate_wallforce(p)

        for p in self.pedestrians:
            p.update_velocity(self.delta_t)
            p.update_position(self.delta_t)


    def eliminate_exited(self):
        eliminate=[]
        for i in range(len(self.pedestrians)):
            if (self.pedestrians[i].position[0] > self.width or self.pedestrians[i].position[1] > self.height):
                eliminate.append(i)
        for i in reversed(range(len(eliminate))):
            del self.pedestrians[eliminate[i]]


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

    def unit_vector(self,target,position):
            vector = np.subtract(target,position)
            v_hat = vector/np.linalg.norm(vector)
            return v_hat

    def calculate_preferredforce(self,p):
            veldir = np.multiply(self.unit_vector(p.target_point,p.position),p.maximum_velocity)
            term2 = np.multiply(p.average_velocity,(1-p.lambda_f))
            dir = np.subtract(p.target_point,p.position)
            desired_dir = dir/np.linalg.norm(dir)
            prefered_velocity = np.multiply(desired_dir,1.1)
#            prefered_velocity = np.add(np.multiply(veldir,p.lambda_f),term2)
            prefered_force = np.multiply(np.subtract(prefered_velocity,p.velocity),p.mass/p.lambda_f)
            p.set_preferedforce(prefered_force)


    def calculate_socialrepulsiveforce(self,p):
        distance = []
        for r in self.pedestrians:
            if p.id == r.id:
                continue

            dist = np.subtract(r.position,p.position)
            distance = np.linalg.norm(dist)

            if distance > self.threshold:
                continue

            overlap = 2*p.radius-distance
            normal = -dist/distance
            tangent=[-normal[1],normal[0]]

            social_const = self.social_force_constant*np.exp(overlap/self.relaxation_length)
            social_force = np.multiply(normal,social_const)

            pushing_const = self.body_force_constant*overlap
            pushing_force = np.multiply(normal,pushing_const)

            fric_const =np.multiply(tangent,self.friction_force_constant*overlap)
            term2 = np.multiply(tangent,np.subtract(r.velocity,p.velocity))
            friction_force = np.multiply(fric_const,term2)

            push_fric = np.add(pushing_force,friction_force)
            unit_pushfric = push_fric/np.linalg.norm(push_fric)
            force_pushfric = np.multiply(unit_pushfric,self.max_acceleration)

            repulsive_force = np.add(social_force,force_pushfric)

            p.set_repulsiveforce(np.add(p.repulsive_force,repulsive_force))


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




    def plot(self):
        plt.style.use('ggplot')
        figure = plt.figure(figsize=(25,25))
        ax = figure.add_subplot(1,1,1)
        ax.set_xlim([0,self.width])
        ax.set_ylim([0,self.height])
        ax.set_aspect('equal')

        x=[]
        y=[]
        for p in self.pedestrians:
            x.append(p.position[0])
            y.append(p.position[1])

        ax.scatter(x,y,color='black')

        return figure
