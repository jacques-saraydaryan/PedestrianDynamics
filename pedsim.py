# This Python file uses the following encoding: utf-8

# if __name__ == "__main__":
#     pass

import os
import matplotlib.pyplot as plt
import matplotlib
import numpy as np

import collisionforcemodel as cfm


def main(args):
#!OPTIMIZATION NOT REQUIRED
#Generate a file to write required parameters
#    if os.path.exists("output.txt"):
#       os.remove("output.txt")

#!OPTIMIZATION NOT REQUIRED
#Generate image directory to plot pngs
    image_directory = "img"
    if not os.path.exists(image_directory):
        os.makedirs(image_directory)


#Loads from file
    loader = cfm.FileReader(args.file)
    world = loader.world



#Constant Parameters
    timer = 60
    time_step = world.delta_t
    iter_max = int(timer/time_step)

#Eliminate repeated calculation in for loop
    seediter_right = round(1.0/(world.pedestrian_fluxright*time_step))
    seediter_up = round(1.0/(world.pedestrian_fluxup*time_step))
    print_iter = int(0.5/time_step)
    print_val = 1


#Simulation Loop
    for i in range(iter_max):
        if i%seediter_right == 0:
            world.add_pedestrian(1,1)
        if i%seediter_up == 0:
            world.add_pedestrian(2,2)

        world.calc_average_velocity()
        world.simulate()
        world.eliminate_exited()

        #!OPTIMIZATION NOT REQUIRED
        #Output Images in directory
        #if i%print_iter == 0:
        #    figure = world.plot()
        #    figure.savefig("{}/{}.png".format(image_directory,print_val),bbox_inches = 'tight')
        #    figure.clear()
        #    plt.close(figure)
        #    print_val += 1

    #!OPTIMIZATION NOT REQUIRED
    #Print final stage image outside simulation loop
    #figure = world.plot()
    #figure.savefig("{}/{}.png".format(image_directory,print_val),bbox_inches = 'tight')
    #figure.clear()
    #plt.close(figure)


def printfile(str):
    with open('output.txt','a') as f:
        sys.stdout = f
        print(str)



if __name__ == '__main__':
    import argparse
    import sys
    parser = argparse.ArgumentParser()
    parser.add_argument('-i','--file', default="input.yml",help='YAML-file')
    args = parser.parse_args(sys.argv[1:])
    main(args)
