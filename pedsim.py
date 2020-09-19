# Made with Qt Creator
# Work in Progress

"""
Code for simulating pedestrians with social force model and collision control algorithm
--------------------A simplified model with appropriate assumptions--------------------
Parametrs set in the file 'input.yml'
"""


import os
import matplotlib.pyplot as plt

import collisionforcemodel as cfm

def main(args):
    image_directory = "img"
    if not os.path.exists(image_directory):
        os.makedirs(image_directory)


    loader = cfm.FileReader(args.file)
    world = loader.world


    total_time = 60;
    timesteps = int(total_time/world.get_time_step())

    # Simulate Function

    for i in range(timesteps):
        if i%(2/world.get_time_step()) == 0:
            world.set_pedestrian(1,[1,0])
            world.set_pedestrian(2,[0,1])
        world.simulate()
        world.eliminate_exited()
        if i%50 == 0:
            figure = world.plot()
            figure.savefig("{}/{}.png".format(image_directory, int(world.get_time_step()*((i + 1)))),
                       bbox_inches = 'tight',
                       pad_inches = 0.1)
            figure.clear()
            plt.close(figure)

    figure = world.plot()
    figure.savefig("{}/{}.png".format(image_directory, timesteps*world.get_time_step()),
               bbox_inches = 'tight',
               pad_inches = 0.1)
    figure.clear()
    plt.close(figure)





if __name__ == '__main__':
    import argparse
    import sys
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', '--file', default="input.yml", help='YAML-file')
    args = parser.parse_args(sys.argv[1:])
    main(args)
