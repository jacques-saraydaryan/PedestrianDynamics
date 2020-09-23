#*********************************************************************************
# PEDESTRIAN FLOW SIMULATION
# Author    : Arunaachalam Muralidharan
# Version   : 2020_09_23
# MODEL: Social Force Model (Modified)
# Made for Crossing Flow Geometry with pedestrian Flux(pi) [0.2,2.0]
# Run times: @pi=2.0(660 s), @pi=1.5(370 s), @pi=1.0(220 s), @pi=0.5(55 s) @pi=0.2(20 s)
# Reference Articles: F.Johansson(2013), Hassan(2017), Helbing(1995), Chraibi(2019)
# Some inspirations from JuPedSim, Rex Valkering
#*********************************************************************************

import os
import matplotlib.pyplot as plt
import matplotlib
import numpy as np

#Written for Code Performance Testing (Comment out Otherwise)
#from time import time

import collisionforcemodel as cfm


def main(args):

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

    #Constant Simulation Parameters
    timer = 60                          #Maximum Time for Simulation
    print_png = 1                       #Time at which a plot to be made
    time_step = world.delta_t
    iter_max = int(timer/time_step)

    #Eliminate repeated calculation in for loop
    seediter_right = round(1.0/(world.pedestrian_fluxright*time_step))
    seediter_up = round(1.0/(world.pedestrian_fluxup*time_step))
    print_iter = int(print_png/time_step)
    print_val = 1


    #Simulation Loop for real time 60 seconds and prints png per 1 second(default)
    for i in range(iter_max):
        if i%seediter_right == 0:
            world.add_pedestrian(1,1)
        if i%seediter_up == 0:
            world.add_pedestrian(2,2)

        world.calc_average_velocity()
        world.simulate()
        world.eliminate_exited()

    #For Testing purpose only (Comment out Otherwise)
    #end = time()
    #print(f'It took {end-start} seconds!')

        #write Average Velocity of World
        printfile("{}\t{}".format(i*time_step,np.linalg.norm(world.pedestrians[0].average_velocity)))

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
def printfile(str):
    with open('output.txt','a') as f:
        sys.stdout = f
        print(str)


#Script to parse arguments
if __name__ == '__main__':
    import argparse
    import sys
    parser = argparse.ArgumentParser()
    parser.add_argument('-i','--file', default="input.yml",help='YAML-file')
    args = parser.parse_args(sys.argv[1:])
    main(args)
