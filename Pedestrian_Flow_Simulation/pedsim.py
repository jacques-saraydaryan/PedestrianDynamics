#*********************************************************************************
# PEDESTRIAN FLOW SIMULATION
# Author    : Arunaachalam Muralidharan
# Version   : 2020_09_23
# MODEL: Social Force Model
# Made for Crossing Flow Geometry with pedestrian Flux(pi) [0.2,2.0]
# Run times: @pi=2.0(660 s), @pi=1.5(370 s), @pi=1.0(220 s), @pi=0.5(55 s) @pi=0.2(20 s)
# Reference Articles: F.Johansson(2013), Hassan(2017), Helbing(1995), Chraibi(2019)
# Some inspirations from JuPedSim, Rex Valkering
#*********************************************************************************

"""
PEDESTRIAN FLOW SIMULATION
MODEL   : Social Force Model
Made for Crossing Flow Geometry with pedestrian Flux(pi) [0.2,2.0]
AUTHOR  : Arunaachalam Muralidharan
VERSION : 2020_09_23

REQUIRED MODULES
----------------
    OS, MATPLOTLIB, NUMPY

USER-DEFINED MODULE
-------------------
    collisionforcemodel

REMARKS
-------
    Simulation set to run for 60 seconds as default
"""

import os
import matplotlib.pyplot as plt
import numpy as np

#Written for Code Performance Testing (Comment out Otherwise)
#from time import time

import collisionforcemodel as cfm


def main(args):
    """
    Main function for the simulation

    Parameters
    ----------
    args : argument parser
        Given as a script with ArgumentParser

    Concept
    -------
    Algorithm for simulation:
        Load input file and set simulation variables
        for i<maximum_iterations:
                Spawn pedestrians and update pedestrian list
                Calculate Average velocity of the world
                for p in [list of pedestrians]:
                    Calculate forces acting on pedestrian p
                    Update velocity for pedestrian p
                    Update position for pedestrian p
                Eliminate pedestrians who exited the world

    Function parameters
    -------------------
    loader : map
        loads the input variables from 'input.yml'
    world : Class
        Class created which stores all the pedestrians at given time
    timer : int
        Total time to run the simulation
    print_png : float
        Time at which a plot of pedestrian positions to be made
    time_step : float
        Time step for simulation satisfying stability
    iter_max : int
        Maximum number of iterations to be performed
    seediter_right, seediter_up : int (Optional)
        Iteration at which a pedestrian should be spawned
    print_iter : int (Optional)
        Iteration at which a plot is to be made
    print_val : int (Optional)
        name of the image file to be printed
    spawn_type : int (Optional)
        Defines the spawn type between point or random

    Additional Functions
    --------------------
    printfile(arg) : prints given string to file 'output.txt'

    arg : string
        This will be printed in the file 'output.txt'

    """

    #Removes a file if present to write required parameters (Here writes time and Average Velocity)
    if os.path.exists("output.txt"):
        os.remove("output.txt")


    #Generate image directory to plot pngs
    image_directory = "img"
    if not os.path.exists(image_directory):
        os.makedirs(image_directory)

    #For Testing purpose only (Comment out Otherwise)
    #start = time()

    #Loads from file
    loader = cfm.FileReader(args.file)
    world = loader.world

    #Check input variables in range and exit if no
    if world.mass <60 or world.mass>90:
        print("Wrong input in parameter 'pedestrian_mass' -- PROGRAM TERMINATED")
        print("The value should be between 60 and 90")
        sys.exit()

    if world.desired_velocity < 1.1 or world.desired_velocity > 1.3:
        print("Wrong input in parameter 'desired_velocity' -- PROGRAM TERMINATED")
        print("The value should be between 1.1 and 1.3")
        sys.exit()

    if world.maximum_velocity < 1.1 or world.maximum_velocity > 1.3:
        print("Wrong input in parameter 'desired_velocity' -- PROGRAM TERMINATED")
        print("The value should be between 1.1 and 1.3")
        sys.exit()

    if world.pedestrian_fluxright < 0.2 or world.pedestrian_fluxright > 2.0:
        print("Wrong input in parameter 'flux_right' -- PROGRAM TERMINATED")
        print("The value should be between 0.2 and 2.0")
        sys.exit()

    if world.pedestrian_fluxup < 0.2 or world.pedestrian_fluxup > 2.0:
        print("Wrong input in parameter 'flux_up' -- PROGRAM TERMINATED")
        print("The value should be between 0.2 and 2.0")
        sys.exit()

    #Constant Simulation Parameters
    timer = 60                          #Maximum Time for Simulation
    print_png = world.print_png         #Time at which a plot to be made
    time_step = world.delta_t
    iter_max = int(timer/time_step)

    #Eliminate repeated calculation in for loop
    seediter_right = round(1.0/(world.pedestrian_fluxright*time_step))
    seediter_up = round(1.0/(world.pedestrian_fluxup*time_step))
    print_iter = int(print_png/time_step)
    print_val = 1
    spawn_type = 0                      #Default spawn type is at point

    if world.spawn_method == 'random':
        spawn_type = 1


    #Simulation Loop for real time 60 seconds and prints png per 1 second(default)
    for i in range(iter_max):
        if i%seediter_right == 0:
            if spawn_type == 0:
                world.add_pedestrian(1,1)
            elif spawn_type == 1:
                world.add_randompedestrian(1,1)
        if i%seediter_up == 0:
            if spawn_type == 0:
                world.add_pedestrian(2,2)
            elif spawn_type == 1:
                world.add_randompedestrian(2,2)

        world.calc_average_velocity()
        world.simulate()
        world.eliminate_exited()

    #For Testing purpose only (Comment out Otherwise)
    #end = time()
    #print(f'It took {end-start} seconds!')

        #write Average Velocity of World
        if i%50 == 0:
            printfile("{}\t{}".format(i*time_step,
                np.linalg.norm(world.pedestrians[0].average_velocity)))

        #Output Images in directory
        if i%print_iter == 0:
            figure = world.plot()
            figure.savefig("{}/{}.png".format(image_directory,print_val),bbox_inches = 'tight')
            figure.clear()
            plt.close(figure)
            print_val += 1


    #Print final stage image outside simulation loop
    figure = world.plot()
    figure.savefig("{}/{}.png".format(image_directory,print_val),bbox_inches = 'tight')
    figure.clear()
    plt.close(figure)

# Method to print any string to file 'output.txt'
def printfile(out_str):
    """
    Prints a string to a file
    Function Call
    -------------
        printfile(output)

    Parameters
    ----------
        output : string
            The string to be printed to file 'output.txt'

    Returns
    -------
        None
    """
    with open('output.txt','a') as file_out:
        sys.stdout = file_out
        print(out_str)


#Script to parse arguments
if __name__ == '__main__':
    import argparse
    import sys
    parser = argparse.ArgumentParser()
    parser.add_argument('-i','--file', default="input.yml",help='YAML-file')
    arguments = parser.parse_args(sys.argv[1:])
    main(arguments)
