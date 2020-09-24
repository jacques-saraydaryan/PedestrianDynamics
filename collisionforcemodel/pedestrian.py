#*********************************************************************************
# CLASS Pedestrian - A Class with Variables for Individual Pedestrians
#*********************************************************************************

"""
CLASS Pedestrian
----------------
    A Class with Variables for Individual Pedestrians

MODULES REQUIRED:
-----------------
    Numpy

USES
----
    Adds Pedestrians as a list object in the World

REMARK
------
    Disable local_attributes while running pylint.
    The private variables are used to scale the code
"""

import numpy as np

class Pedestrian():
    """
    Class Pedestrian with individual properties of pedestrians as private variables

    Call
    ----
    Pedestrian(id_unique)

    id_unique : int
        Unique id generated by code

    Parameters
    ----------
    radius : float
        Radius of pedestrian (standard for this simulation = 0.25 m)
    mass : float
        Mass of pedestrian (Standard: 75 kg, can be changed in 'input.yml')
    id_ped : int
        Unique id for each pedestrian
    desired_velocity : float
        Desired pedestrian velocity given in 'input.yml'
    maximum_velocity : float
        Maximum velocity of pedestrian given in 'input.yml'
    target_point : [float, float]
        Target destination of the pedestrian in (X, Y)
    position : [float, float]
        Current position of the pedestrian in (X, Y)
    velocity : [float, float]
        Current velocity of the pedestrian in (Vx, Vy)
    average_velocity : float
        Current average velocity of all pedestrian in the world
    lambda_f : float
        Winding for prefered force calculation (default:1.0)
    total_force : [float, float]
        Total force acting on the pedestrian in (Fx, Fy)
    prefered_force : [float, float]
        Prefered force acting on the pedestrian in (Fx, Fy)
    repulsive_force : [float, float]
        Repulsive force acting on the pedestrian in (Fx, Fy)
    wall_force : [float, float]
        Wall force acting on the pedestrian in (Fx, Fy)
    wall_point : [float, float]
        Nearest wall for the pedestrian in (X, Y)
    direction : int
        To denote the major axis of pedestrian movement
                1:x-axis, 2: y-axis

    Function Calls
    --------------
    set_mass(arg) : sets mass as arg
    set_radius(arg) : sets radius as arg
    set_desiredvelocity(arg) : sets desired_velocity as arg
    set_maximumvelocity(arg) : sets maximum_velocity as arg
    set_targetpoint(arg) : sets target_point as arg
    set_position(arg) : sets position as arg
    set_averagevelocity(arg) : sets average_velocity as arg
    set_preferedforce(arg) : sets prefered_force as arg
    set_repulsiveforce(arg) : sets repulsive_force as arg
    set_wallforce(arg) : sets wall_force as arg
    set_direction(arg) : sets direction as arg
    set_velocity() : Calculates and sets velocity of pedestrian
    set_zeroforce() : Resets all forces to zero
    calculate_force() : Calculates total_force
    correct_position() : Checks for out of bound pedestrians
    update_velocity(arg): Calculates the velocity of pedestrian
    update_position(arg): Calculates the position of pedestrian

    Returns
    -------
        None
    """
    def __init__(self,id_unique):
        # Standard Parameters for Pedestrians
        self.radius = 0.25
        self.mass = 75.0

        self.id_ped = id_unique             # Unique id generated by Code
        self.desired_velocity = 0.0         # Preferred Velocity given as input
        self.maximum_velocity = 0.0         # Maximum Velocity given as input
        self.target_point = [0,0]           # Target Point given as input
        self.position = [0,0]               # Position of Pedestrian
        self.velocity = [0,0]               # Velocity of the pedestrian

        self.average_velocity = [0,0]       # Average Velocity of the World
        self.lambda_f = 1.0                 # Lambda for Preferred Force Calculation (set at 1)

        self.total_force = [0,0]            # Total force on Pedestrian
        self.prefered_force = [0,0]         # Preferred force on Pedestrian
        self.repulsive_force = [0,0]        # Repulsive force on Pedestrian
        self.wall_force = [0,0]             # Wall force on Pedestrian
        self.wall_point = [0,0]             # The Nearest Wall to Pedestrian

        self.direction = 0                  # Target 1 = x_axis, 2 = y_axis
        self.temp_velocity = [0,0]

    #---------------------------------------------------------------------------------------
    #***************** Setter Functions for Class Variables*********************************
    #---------------------------------------------------------------------------------------
    def set_mass(self,mass):
        """
        Setter function for private variable in class Pedestrian

        Parameters
        ----------
        mass : float
            A value specified in the input file

        Returns
        -------
            Sets the self.mass as argument
        """
        self.mass = mass

    def set_radius(self,radius):
        """
        Setter function for private variable in class Pedestrian

        Parameters
        ----------
        radius : float
            A value specified in the input file

        Returns
        -------
            Sets the self.radius as argument
        """
        self.radius = radius

    def set_desiredvelocity(self,desired_velocity):
        """
        Setter function for private variable in class Pedestrian

        Parameters
        ----------
        desired_velocity : float
            A value specified in the input file

        Returns
        -------
            Sets the self.desired_velocity as argument
        """
        self.desired_velocity = desired_velocity

    def set_maximumvelocity(self,maximum_velocity):
        """
        Setter function for private variable in class Pedestrian

        Parameters
        ----------
        maximum_velocity : float
            A value specified in the input file

        Returns
        -------
            Sets the self.maximum_velocity as argument
        """
        self.maximum_velocity = maximum_velocity

    def set_targetpoint(self,target_point):
        """
        Setter function for private variable in class Pedestrian

        Parameters
        ----------
        target_point : List - [float,float]
            Target position sent from Class:World

        Returns
        -------
            Sets the self.target_point as argument
        """
        self.target_point = target_point

    def set_position(self,position):
        """
        Setter function for private variable in class Pedestrian

        Parameters
        ----------
        position : List - [float,float]
            Pedestrian position sent from Class:World

        Returns
        -------
            Sets the self.position as argument
        """
        self.position = position

    def set_averagevelocity(self,velocity):
        """
        Setter function for private variable in class Pedestrian

        Parameters
        ----------
        velocity : float
            Average velocity calculated from Class:World

        Returns
        -------
            Sets the self.average_velocity as argument
        """
        self.average_velocity = velocity

    def set_preferedforce(self,force):
        """
        Setter function for private variable in class Pedestrian

        Parameters
        ----------
        force : List - [float,float]
            Preferred Force calculated in Class:World by method calculate_preferredforce(arg)

        Returns
        -------
            Sets the self.prefered_force as argument
        """
        self.prefered_force = force

    def set_repulsiveforce(self,force):
        """
        Setter function for private variable in class Pedestrian

        Parameters
        ----------
        force : List - [float,float]
            Repulsive force calculated in Class:World by method calculate_socialrepulsiveforce(arg)

        Returns
        -------
            Sets the self.repulsive_force as argument
        """
        self.repulsive_force = force

    def set_wallforce(self,force):
        """
        Setter function for private variable in class Pedestrian

        Parameters
        ----------
        force : List - [float,float]
            Wall force calculated in Class:World by method calculate_wallforce(arg)

        Returns
        -------
            Sets the self.wall_force as argument
        """
        self.wall_force = force

    def set_wallpoint(self,point):
        """
        Setter function for private variable in class Pedestrian

        Parameters
        ----------
        target_point : List - [float,float]
            Target position sent from Class:World

        Returns
        -------
            Sets the self.target_point as argument
        """
        self.wall_point = point

    def set_direction(self,direc):
        """
        Setter function for private variable in class Pedestrian

        Parameters
        ----------
        direc : int
            Spawn index sent from Class:World
            1 - Pedestrian spawn at 'C'
            2 - Pedestrian spawn at 'A'

        Returns
        -------
            Sets the self.direction as argument
        """
        self.direction = direc

    #---------------------------------------------------------------------------------------
    #********************** Functions for Calculation***************************************
    #---------------------------------------------------------------------------------------

    #Velocity set based on Preferred Direction and Maximum Velocity
    def set_velocity(self):
        """
        Calculate initial Velocity of Pedestrian

        Parameters
        ----------
        none

        Functions
        ----------
        By taking the current position and target point, calculate desired direction of travel
        From the desired direction, with maximum velocity initial velocity of pedestrians are set.
        Math:
            Desired_direction = (target_point - position)/norm(target_point - position)
            velocity = Desired_direction * maximum_velocity
        Returns
        -------
            Sets the self.velocity as calculated velocity [vx,vy]
        """
        direc = np.subtract(self.target_point,self.position)
        desired_dir = direc/np.linalg.norm(direc)
        self.velocity = np.multiply(desired_dir,self.maximum_velocity)

    # Set Forces to Zero after each Timestep
    def set_zeroforce(self):
        """
        Reset the force terms to zero after each iteration to eliminate redundant forces

        Parameters
        ----------
        none

        Returns
        -------
            Sets the forces to zero after each iteration
            Changed forces total_force, repulsive_force, prefered_force
        """
        self.total_force = [0,0]
        self.repulsive_force = [0,0]
        self.prefered_force = [0,0]

    # Calculate Total Force by adding other forces F_prefered, F_wall, and F_repulsive
    def calculate_force(self):
        """
        Calculates the total force acting on each Pedestrian

        Parameters
        ----------
        none

        Returns
        -------
            Sets the total force = sum of all other forces
        """
        self.total_force = self.prefered_force+self.repulsive_force+self.wall_force

    # ERROR CHECK: Usage statistically low: Correct position if the position is out of bound
    def correct_position(self):
        """
        Checks for out of bound pedestrians after each iteration

        Parameters
        ----------
        none

        Concept
        -------
            In case of a vector out of bound i.e. outside the preferred world area,
            the pedestrians are forcefully brought inside the simulation region
            Statistical usage is very low, but proves as a fail safe in case of abnormalities

        Returns
        -------
            Checks and sets the position of pedestrians to acceptable limit
        """
        if self.direction == 1:
            if self.position[1] >= 15 and self.position[1] >= 10:
                if self.position[1] <= 10:
                    self.position[1] = self.wall_point[1]+0.1
                if self.position[1] >= 15:
                    self.position[1] = self.wall_point[1]-0.1
        if self.direction == 2:
            if self.position[0] >= 15 and self.position[1] >= 10:
                if self.position[0] <= 10:
                    self.position[0] = self.wall_point[0]+0.1
                if self.position[0] >= 15:
                    self.position[0] = self.wall_point[0]-0.1

    # Update Velocity of Pedestrian v(t) = v(t-1) + dt*F
    def update_velocity(self,timestep):
        """
        Update velocity of pedestrian for each iteration after calculation of forces

        Parameters
        ----------
        timestep: float
            Simulation time step set by input file

        Concept
        -------
            The equation of motion states that, (dv/dt) = sum of forces
                v(t+1) = v(t) + timestep * total_force
            The velocity is desired velocity but there is a limit to the velocity:
                    A pedestrian can take set as maximum_velocity.
            So, the vector of desired velocity is taken and multiplied by the maximum velocity
                v = v_hat * maximum_velocity
        Returns
        -------
            Sets the velocity of individual pedestrians
        """
        self.calculate_force()
        self.velocity = np.add(self.velocity,np.multiply(self.total_force,timestep))
        norm_value = np.linalg.norm(self.velocity)
        self.velocity = np.multiply(self.velocity/norm_value,min(norm_value,self.maximum_velocity))


    # Update Position of pedestrian x(t) = x(t-1) + dt*v
    def update_position(self,timestep):
        """
        Update position of pedestrian for each iteration

        Parameters
        ----------
        timestep: float
            Simulation time step set by input file

        Concept
        -------
            The equation of motion states that, (dx/dt) = velocity
                x(t+1) = x(t) + timestep * velocity
            After position of pedestrian is calculated:
                    A check is run to identify out of bound pedestrians

        Returns
        -------
            Sets the position of individual pedestrians
        """
        self.position = np.add(self.position,np.multiply(self.velocity, timestep))
        self.correct_position()
