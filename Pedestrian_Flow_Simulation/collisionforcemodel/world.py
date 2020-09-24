#**********************************************************************************
# CLASS World - A Class with stores all Pedestrians and does necessary Calculations
#**********************************************************************************

"""
CLASS World
----------------
    Contains and Maintains the Pedestrians in the World

MODULES REQUIRED:
-----------------
    numpy, random, matplotlib.pyplot

USES
----
    Maintains relevant list of pedestrians in the World

REMARK
------
    Disable local_attributes while running pylint.
    The private variables are used to scale the code
"""

import matplotlib.pyplot as plt
import numpy as np
import random as rn

from .pedestrian import Pedestrian

class World(object):
    def __init__(self):
        """
        Class World with list of PEDESTRIANS in simulation

        Call
        ----
        World()

        Parameters
        ----------
        height : float
            Height of World (standard for this simulation = 25 m)
        width : float
            Width of world (standard for this simulation = 25 m)
        pedestrians : list
            List holding all pedestrian class
        spawnpoints : list
            Set of spawn points for the spawning pedestrians
        delta_t : float
            Time step for simulation satisfying stability
        pedestrian_flux : list
            List of pedestrian flux given in input file
        targetpoints : list
            Set of target points for the pedestrians
        pedestrian_fluxright : float
            Pedestrian flux for 'C'
        pedestrian_fluxup : float
            Pedestrian flux for 'A'
        mass : float
            Mass of pedestrians (Default 60 kg)
        desired_velocity : float
            Desired Velocity of pedestrians given in input
        maximum_velocity : float
            Maximum velocity of pedestrians given in input
        id_unique : int
            Unique id for pedestrians generated
        print_png : int
            Time at which a plot should be made
        spawn_method : int
            Spawn method random/point (default 'point')
        relaxation_time : float
            Used in calculating Preferred Force
        social_force_constant, body_force_constant, friction_force_constant : float
            Used in calculating Repulsion force
        relaxation_length, threshold : float
            Used in calculating Repulsion force
        obstacle_force_constant, sigma_obst : float
            Used in calculating Wall force

        Function Calls
        --------------
        set_height(args) : Sets self.height as args
        set_width(args) : sets self.width as args
        set_pedestrianmass(args) : sets self.mass as args
        set_fluxright(args) : sets self.pedestrian_fluxright as args
        set_fluxup(args) : sets self.pedestrian_fluxup as args
        set_desiredvelocity(args) : sets self.desired_velocity as args
        set_maxvelocity(args) : sets self.maximum_velocity as args
        set_printpng(args) : sets self.print_png as args
        set_point1x(args): appends args to self.spawnpoints
        set_point1y(args) : appends args to self.spawnpoints
        set_point2x(args) : appends args to self.spawnpoints
        set_point2y(args) : appends args to self.spawnpoints
        set_target1x(args) : appends args to self.targetpoints
        set_target1y(args) : appends args to self.targetpoints
        set_target2x(args) : appends args to self.targetpoints
        set_target2y(args) : appends args to self.targetpoints
        set_pedflux1(args) : appends args to self.pedestrian_flux
        set_pedflux2(args) : appends args to self.pedestrian_flux
        set_delta_t(args) : sets arg to self.delta_t
        set_spawnmethod(args) : sets arg to spawn method

        get_spawnpoint(args):
            Returns spawn points for a given target direction
        get_randomspawnpoint(args):
            Returns random spawn points for a given target direction
        get_targetpoint(args):
            Returns target points for a given target index
        add_pedestrian(args,args):
            Adds a pedestrian to the LIST [PEDESTRIANS]
        add_randompedestrian(args,args):
            Adds/appends a pedestrian to the LIST [PEDESTRIANS]
        spawn_pedestrian(args,args,args):
            Spawns a Pedestrian class based on given input
        spawn_randompedestrian(args,args,args):
            Spawns a random Pedestrian class based on given input
        calc_average_velocity():
            Calculate average velocity of the world
        unit_vector(args,args):
            Returns a unit vector for two points
        calculate_preferredforce(args):
            Calculate prefered force acting on the pedestrian
        calculate_socialrepulsiveforce(args):
            Calculate Social Repulsive Force acting on the pedestrian
        get_nearest_wall(args,args):
            Finds nearest Wall point for pedestrians
        calculate_wallforce(args):
            Calculate Wall force acting on the pedestrian
        simulate():
            Advances one time step in simulation
        eliminate_exited():
            Eliminate Pedestrians from LIST [PEDESTRIANS]
        plot():
            Generate a Scatter Plot of Pedestrian Positions

        Returns
        -------
            None
        """

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

        self.id_unique = 0                  # System generated id for Pedestrians
        self.print_png = 0                  # PRINT CONTROL from input
        self.spawn_method = 'point'         # Spawn method for Pedestrians

        #For Force Calculations Constants
        #Preferred Force Constants
        self.relaxation_time = 1.0

        #Repulsive Force Constants
        self.social_force_constant = 2000.0
        self.body_force_constant = 12000.0
        self.friction_force_constant = 24000.0
        self.relaxation_length = 0.3
        self.max_acceleration = 2.0
        self.threshold = 2.0


        #Wall Force Constants
        self.obstacle_force_constant = 50
        self.sigma_obst = 0.2

    #---------------------------------------------------------------------------------------
    #***************** Setter Functions for Class Variables*********************************
    #---------------------------------------------------------------------------------------
    def set_height(self,height):
        """
        Setter function for private variable in class World

        Parameters
        ----------
        height : float
            A value specified in the input file

        Returns
        -------
            Sets the self.height as argument
        """
        self.height = height

    def set_width(self,width):
        """
        Setter function for private variable in class World

        Parameters
        ----------
        width : float
            A value specified in the input file

        Returns
        -------
            Sets the self.width as argument
        """
        self.width = width

    def set_pedestrianmass(self,mass):
        """
        Setter function for private variable in class World

        Parameters
        ----------
        mass : float
            A value specified in the input file

        Returns
        -------
            Sets the self.mass as argument
        """
        self.mass = mass

    def set_fluxright(self,flux):
        """
        Setter function for private variable in class World

        Parameters
        ----------
        flux : float
            A value specified in the input file
            This is pedestrian flux at the inlet 'C'

        Returns
        -------
            Sets the self.pedestrian_fluxright as argument
        """
        self.pedestrian_fluxright = flux

    def set_fluxup(self,flux):
        """
        Setter function for private variable in class World

        Parameters
        ----------
        flux : float
            A value specified in the input file
            This is pedestrian flux at the inlet 'A'

        Returns
        -------
            Sets the self.pedestrian_fluxup as argument
        """
        self.pedestrian_fluxup = flux

    def set_desiredvelocity(self,vel):
        """
        Setter function for private variable in class World

        Parameters
        ----------
        vel : float
            A value specified in the input file

        Returns
        -------
            Sets the self.desired_velocity as argument
        """
        self.desired_velocity = vel

    def set_maxvelocity(self,vel):
        """
        Setter function for private variable in class World

        Parameters
        ----------
        vel : float
            A value specified in the input file

        Returns
        -------
            Sets the self.maximum_velocity as argument
        """
        self.maximum_velocity = vel

    def set_printpng(self,index):
        """
        Setter function for private variable in class World

        Parameters
        ----------
        index : int
            A value specified in the input file

        Returns
        -------
            Sets the self.print_png as argument
        """
        self.print_png = index

    def set_point1x(self,point):
        """
        Setter function for private variable in class World

        Parameters
        ----------
        point : float
            A value specified in the input file

        Returns
        -------
            Sets the spawn points used for Spawn Type: Points
        """
        self.spawnpoints[0] = point

    def set_point1y(self,point):
        """
        Setter function for private variable in class World

        Parameters
        ----------
        point : float
            A value specified in the input file

        Returns
        -------
            Sets the spawn points used for Spawn Type: Points
        """
        self.spawnpoints[1] = point

    def set_point2x(self,point):
        """
        Setter function for private variable in class World

        Parameters
        ----------
        point : float
            A value specified in the input file

        Returns
        -------
            Sets the spawn points used for Spawn Type: Points
        """
        self.spawnpoints[2] = point

    def set_point2y(self,point):
        """
        Setter function for private variable in class World

        Parameters
        ----------
        point : float
            A value specified in the input file

        Returns
        -------
            Sets the spawn points used for Spawn Type: Points
        """
        self.spawnpoints[3] = point

    def set_target1x(self,point):
        """
        Setter function for private variable in class World

        Parameters
        ----------
        point : float
            A value specified in the input file

        Returns
        -------
            Sets the target points
        """
        self.targetpoints[0] = point

    def set_target1y(self,point):
        """
        Setter function for private variable in class World

        Parameters
        ----------
        point : float
            A value specified in the input file

        Returns
        -------
            Sets the target points
        """
        self.targetpoints[1] = point

    def set_target2x(self,point):
        """
        Setter function for private variable in class World

        Parameters
        ----------
        point : float
            A value specified in the input file

        Returns
        -------
            Sets the target points
        """
        self.targetpoints[2] = point

    def set_target2y(self,point):
        """
        Setter function for private variable in class World

        Parameters
        ----------
        point : float
            A value specified in the input file

        Returns
        -------
            Sets the target points
        """
        self.targetpoints[3] = point

    def set_pedflux1(self,flux):
        """
        Setter function for private variable in class World

        Parameters
        ----------
        flux : float
            A value specified in the input file

        Returns
        -------
            Appends the list of pedestrian flux
        """
        self.pedestrian_flux.append(flux)

    def set_pedflux2(self,flux):
        """
        Setter function for private variable in class World

        Parameters
        ----------
        flux : float
            A value specified in the input file

        Returns
        -------
            Appends the list of pedestrian flux
        """
        self.pedestrian_flux.append(flux)

    def set_delta_t(self,time):
        """
        Setter function for private variable in class World

        Parameters
        ----------
        time : float
            A value specified in the input file

        Returns
        -------
            Sets self.delta_t as argument
        """
        self.delta_t = time

    def set_spawnmethod(self,method):
        """
        Setter function for private variable in class World

        Parameters
        ----------
        method : int
            A value specified in the input file
            0 = spawn pedestrians at given point
            1 = spawn pedestrians at a given area

        Returns
        -------
            Sets self.spawn_method to args
        """
        self.spawn_method = method


    #---------------------------------------------------------------------------------------
    #********************** Functions for Calculation***************************************
    #---------------------------------------------------------------------------------------
    # Get spawn points @source = [1=='C', 2=='A']
    def get_spawnpoint(self,source):
        """
        Returns spawn points for a given target direction

        Parameters
        ----------
        source: int
            Spawn index which defines the location of spawn
            1 == 'C'; 2 == 'A'

        Returns
        -------
            A list containing spawn points [X, Y]
        """
        if source == 1:
            return [self.spawnpoints[0],self.spawnpoints[1]]
        elif source == 2:
            return [self.spawnpoints[2],self.spawnpoints[3]]

    # Get random spawn points @source = [1=='C', 2=='A']
    def get_randomspawnpoint(self,source):
        """
        Returns random spawn points for a given target direction

        Parameters
        ----------
        source: int
            Spawn index which defines the location of spawn
            1 == 'C'; 2 == 'A'

        Returns
        -------
            A list containing spawn points [X, Y]
        """
        if source == 1:
            return [0, rn.randint(120,140)*0.1]
        if source == 2:
            return [rn.randint(120,140)*0.1, 0]

    # Get Target points @index = [1=='C', 2=='A']
    def get_targetpoint(self,index):
        """
        Returns target points for a given target index

        Parameters
        ----------
        index: int
            target index which defines the location of target
            1 == 'C'; 2 == 'A'

        Returns
        -------
            A list containing target points [X, Y]
        """
        if index == 1:
            return [self.targetpoints[0],self.targetpoints[1]]
        elif index == 2:
            return [self.targetpoints[2],self.targetpoints[3]]


    # Add a pedestrian to the LIST [PEDESTRIANS] @spawnindex,@targetindex = [1=='C',2=='A']
    def add_pedestrian(self,spawnindex,targetindex):
        """
        Adds a pedestrian to the LIST [PEDESTRIANS]

        Parameters
        ----------
        spawnindex, targetindex: int
            Defines the location, 1 == 'C'; 2 == 'A'
        id_unique : int
            Sets a unique id to Pedestrian Class

        Function Call
        -------------
        spawn_pedestrian(unique_id, spawnindex,targetindex)
            Spawns a pedestrian class

        Returns
        -------
            Adds a Pedestrian class to the LIST: [Pedestrians]
        """
        if spawnindex == 1:
            self.pedestrians.append(self.spawn_pedestrian(self.id_unique,spawnindex,targetindex))
        if spawnindex == 2:
            self.pedestrians.append(self.spawn_pedestrian(self.id_unique,spawnindex,targetindex))
        self.id_unique+=1

    # Add a random spawn pedestrian to the LIST [PEDESTRIANS]
    def add_randompedestrian(self,spawnindex,targetindex):
        """
        Adds/appends a pedestrian to the LIST [PEDESTRIANS]

        Parameters
        ----------
        spawnindex, targetindex: int
            Defines the location, 1 == 'C'; 2 == 'A'
        id_unique : int
            Sets a unique id to Pedestrian Class

        Function Call
        -------------
        spawn_randompedestrian(unique_id, spawnindex,targetindex)
            Spawns a pedestrian class

        Returns
        -------
            Adds a Pedestrian class to the LIST: [Pedestrians]
        """
        if spawnindex == 1:
            self.pedestrians.append(self.spawn_randompedestrian(self.id_unique,spawnindex,targetindex))
        if spawnindex == 2:
            self.pedestrians.append(self.spawn_randompedestrian(self.id_unique,spawnindex,targetindex))
        self.id_unique+=1

    # Spawn a Pedestrian at given point
    def spawn_pedestrian(self,source,spawnindex,targetindex):
        """
        Spawns a Pedestrian class based on given input

        Parameters
        ----------
        spawnindex, targetindex: int
            Defines the location, 1 == 'C'; 2 == 'A'
        source : int
            Sets a unique id to Pedestrian Class

        Function Call
        -------------
        Pedestrian class Constructor
            Pedestrian(source)
        Setter functions from class pedestrians -- (args = respective arguments)
            set_mass(args)
            set_desiredvelocity(args)
            set_maximumvelocity(args)
            set_targetpoint(args)
            set_position(args)
            set_velocity()
            set_direction(args)

        Methods from World Class -- (args = respective arguments)
        get_targetpoint(args) : Returns target points [X,Y]
        get_spawnpoint(args) : Returns spawn points[X,Y]

        Returns
        -------
            Returns a Pedestrian class
        """
        p = Pedestrian(source)                                # Creates Pedestrian with unique id
        p.set_mass(self.mass)                                 # Set Pedestrian Mass from input
        p.set_desiredvelocity(self.desired_velocity)          # Set Preferred Velocity from input
        p.set_maximumvelocity(self.maximum_velocity)          # Set Maximum Velocity from input
        p.set_targetpoint(self.get_targetpoint(targetindex))  # Set Target point from input
        p.set_position(self.get_spawnpoint(spawnindex))       # Set Spawn point from input
        p.set_velocity()                                      # Set Velocity based on Maximum Velocity
        p.set_direction(spawnindex)                           # Set spawn index for correction
        return p                                              # Returns a Class:Pedestrian

    # Spawn a Pedestrian at a random point
    def spawn_randompedestrian(self,source,spawnindex,targetindex):
        """
        Spawns a Pedestrian class based on given input

        Parameters
        ----------
        spawnindex, targetindex: int
            Defines the location, 1 == 'C'; 2 == 'A'
        source : int
            Sets a unique id to Pedestrian Class

        Function Call
        -------------
        Pedestrian class Constructor
            Pedestrian(source)
        Setter functions from class pedestrians -- (args = respective arguments)
            set_mass(args)
            set_desiredvelocity(args)
            set_maximumvelocity(args)
            set_targetpoint(args)
            set_position(args)
            set_velocity()
            set_direction(args)

        Methods from World Class -- (args = respective arguments)
        get_targetpoint(args) : Returns target points [X,Y]
        get_randomspawnpoint(args) : Returns spawn points[X,Y]

        Returns
        -------
            Returns a Pedestrian class
        """
        p = Pedestrian(source)                                # Creates Pedestrian with unique id
        p.set_mass(self.mass)                                 # Set Pedestrian Mass from input
        p.set_desiredvelocity(self.desired_velocity)          # Set Preferred Velocity from input
        p.set_maximumvelocity(self.maximum_velocity)          # Set Maximum Velocity from input
        p.set_targetpoint(self.get_targetpoint(targetindex))  # Set Target point from input
        p.set_position(self.get_randomspawnpoint(spawnindex)) # Set Spawn point from input
        p.set_velocity()                                      # Set Velocity based on Maximum Velocity
        p.set_direction(spawnindex)                           # Set spawn index for correction
        return p                                              # Returns a Class:Pedestrian

    # Calculate Average Velocity for the World of Pedestrians
    def calc_average_velocity(self):
        """
        Calculate average velocity of the world

        Optional Parameters
        -------------------
        avg_velx : float
            average velocity at x direction
        avg_vely : float
            average velocity at y direction

        Concept
        -------
            for p in pedestrians:
                add velocities
            average velocity = velocities/number of pedestrians

        Returns
        -------
            Sets the self.average_velocity to the calculated value
        """
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
        """
        Calculate a unit vector for two points

        Parameters
        ----------
        target, position : [float, float]
            A list containing points [X, Y]

        Optional Parameters
        --------------------
        vector : [float, float]
            Difference between target and position
        v_hat : [float, float]
            Calculated Unit Vector

        Numpy Calls
        -----------
        np.subtract(a,b), np.linalg.norm(a)

        Returns
        -------
            Returns a Pedestrian class
        """
        vector = np.subtract(target,position)
        v_hat = vector/np.linalg.norm(vector)
        return [v_hat[0],v_hat[1]]

    # Calculate Preferred Force = (mass/tau)*(preferred Velocity - Velocity)
    # @p: a Pedestrian class element from the List=[PEDESTRIANS]
    def calculate_preferredforce(self,p):
        """
        Calculate prefered force acting on the pedestrian

        Parameters
        ----------
        p : Pedestrian Class
            Contains individual pedestrian variables

        Optional Parameters
        --------------------
        desired_dir : [float, float]
            Unit vector calculated from direction
        prefered_velocity : [float, float]
            Desired velocity in the new direction
        prefered_force: [float, float]
            Prefered force [Fx, Fy]

        Numpy Calls
        -----------
        np.subtract(a,b), np.linalg.norm(a), np.multiply(a,b)

        Returns
        -------
            Sets the Pedestrian class Force term with calculated value
        """
        direc = np.subtract(p.target_point,p.position)
        desired_dir = direc/np.linalg.norm(direc)
        prefered_velocity = np.multiply(desired_dir,self.desired_velocity)
        prefered_force = np.multiply(np.subtract(prefered_velocity,p.velocity),
                                                    p.mass/self.relaxation_time)
        p.set_preferedforce(prefered_force)

    # Calculate Repulsive Force = Social_force + Physical_force
    # @p: a Pedestrian class element from the LIST=[PEDESTRIANS]
    def calculate_socialrepulsiveforce(self,p):
        """
        Calculate Social Repulsive Force acting on the pedestrian

        Parameters
        ----------
        p : Pedestrian Class
            Contains individual pedestrian variables

        Optional Parameters
        --------------------
            Other parameters are temporary variables used to reduce computational effort

        Concept
        -------
            Social Repulsion Force = Social_force + Phsyical_force
            Only calculate Social force when pedestrians are within 2 meters
            Only calculate Physical force when pedestrians are very near 0.5 m

        Function Calls
        --------------
        From Pedestrian Class
            set_repulsiveforce(args)

        Numpy Calls
        -----------
        np.subtract(a,b), np.linalg.norm(a), np.multiply(a,b)

        Returns
        -------
            Sets the Pedestrian class Force term with calculated value
        """
        distance = []

        #Iterate over LIST=[PEDESTRIANS] to find relative forces
        for r in self.pedestrians:
            #Reduce calculations by skipping relative_self
            if p.id_ped == r.id_ped:
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
        """
        Finds nearest Wall point for pedestrians

        Parameters
        ----------
        point, target : [float, float]
            Position of pedestrian and target point

        Numpy calls
        -----------
            np.linalg.norm(a), np.subtract(a,b)

        Returns
        -------
            Returns the nearest wall point for pedestrians [X, Y]
        """
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
        """
        Calculate Wall force acting on the pedestrian

        Parameters
        ----------
        p : Pedestrian Class
            Contains individual pedestrian variables

        Optional Parameters
        --------------------
        wall_point : [float, float]
            Nearest wall to the pedestrian [X, Y]
        distance : float
            Distance between wall and pedestrian
        wall_force: [float, float]
            Calculated Wall force [Fx, Fy]

        others are temporary variables used to reduce computational effort

        Concept
        -------
            Wall force = F_wall * unit_vector * exp(-distance/sigma)
            Only calculate when the pedestrian is near wall to reduce computation
            Threshold distance from wall = 2 meters
            Pedestrian has a tendency to avoid walls

        Function Calls
        --------------
        From Pedestrian Class
            set_wallpoint(args)
            set_wallforce(args)

        From World Class
            get_nearest_wall(position,target)

        Numpy Calls
        -----------
        np.subtract(a,b), np.linalg.norm(a), np.multiply(a,b)

        Returns
        -------
            Sets the Pedestrian class Force term with calculated value
        """
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
        """
        Advaces one time step in simulation

        Concept
        -------
            for p in [PEDESTRIANS]:
                set forces to zero
                calculate all forces
                update velocity and position

        Function Call
        -------------
        From Pedestrian Class
            set_zeroforce()
            update_velocity(args)
            update_position(args)

        From World Class
            calculate_preferredforce()
            calculate_socialrepulsiveforce()
            calculate_wallforce()

        Returns
        -------
            Updated Velocity and Position for each Pedestrians
        """
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
        """
        Eliminate Pedestrians from LIST [PEDESTRIANS]

        Concept
        -------
            for p in [PEDESTRIANS]:
                if p.position is out of range:
                    delete p

        Returns
        -------
            Updates the Pedestrians in the World
        """
        eliminate=[]
        for i in range(len(self.pedestrians)):
            if (self.pedestrians[i].position[0] > self.width or self.pedestrians[i].position[1] > self.height):
                eliminate.append(i)
        for i in reversed(range(len(eliminate))):
            del self.pedestrians[eliminate[i]]

    # Scatter Plot of pedestrians with colors @blue - from 'A', @red - from 'C'
    def plot(self):
        """
        Generate a Scatter plot of pedestrian positions

        USES
        ----
            matplotlib, matplotlib.pyplot

        Returns
        -------
            Figure Scattered plots of pedestrian position
        """
        plt.style.use('ggplot')
        figure = plt.figure(figsize=(6,6))
        fig_ax = figure.add_subplot(1,1,1)
        fig_ax.set_xlim([0,self.width])
        fig_ax.set_ylim([0,self.height])
        fig_ax.set_aspect('equal')

        x_r=[]                                         #X_Position for Pedestrians spawned at 'C'
        y_r=[]                                         #Y_Position for Pedestrians spawned at 'C'
        x_u=[]                                         #X_Position for Pedestrians spawned at 'A'
        y_u=[]                                         #Y_Position for Pedestrians spawned at 'A'
        for p in self.pedestrians:
            if p.target_point[0] > p.target_point[1]:
                x_r.append(p.position[0])
                y_r.append(p.position[1])
            else:
                x_u.append(p.position[0])
                y_u.append(p.position[1])

        fig_ax.scatter(x_r,y_r,color='r', s=60)
        fig_ax.scatter(x_u,y_u,color='b', s=60)

        #Set Border walls as Lines
        fig_ax.plot([0,10],[15,15],color='black',linewidth=5)
        fig_ax.plot([10,10],[15,25],color='black',linewidth=5)
        fig_ax.plot([15,15],[15,25],color='black',linewidth=5)
        fig_ax.plot([15,25],[15,15],color='black',linewidth=5)
        fig_ax.plot([10,10],[0,10],color='black',linewidth=5)
        fig_ax.plot([0,10],[10,10],color='black',linewidth=5)
        fig_ax.plot([15,15],[0,10],color='black',linewidth=5)
        fig_ax.plot([15,25],[10,10],color='black',linewidth=5)

        return figure
