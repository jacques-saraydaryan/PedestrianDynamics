# This Python file uses the following encoding: utf-8

# if __name__ == "__main__":
#     pass

import os
import matplotlib.pyplot as plt
import matplotlib

import collisionforcemodel as cfm


def main(args):

    image_directory = "img"
    if not os.path.exists(image_directory):
        os.makedirs(image_directory)


    loader = cfm.FileReader(args.file)
    world = loader.world





    timer = 60
    time_step = world.delta_t
    iter_max = int(timer/time_step)
    count_right = 0
    count_up = 0

    #Eliminate repeated calculation
    seediter_right = 1.0/(world.pedestrian_fluxright*time_step)
    seediter_up = 1.0/(world.pedestrian_fluxup*time_step)
    print_iter = int(0.5/time_step)


    for i in range(iter_max):
        if i%seediter_right == 0:
            world.add_pedestrian(1,1,count_right)
            count_right +=1
        if i%seediter_up == 0:
            world.add_pedestrian(2,2,count_up)
            count_up += 1

 #       print('\nat time step',i)


        world.calc_average_velocity()
        world.simulate()
        world.eliminate_exited()


        if i%print_iter == 0:
            figure = world.plot()
            figure.savefig("{}/{}.png".format(image_directory,int(time_step*(i+1))),bbox_inches = 'tight')
            figure.clear()
            plt.close(figure)

    figure = world.plot()
    figure.savefig("{}/{}.png".format(image_directory,int(iter_max*time_step)),bbox_inches = 'tight')
    figure.clear()
    plt.close(figure)






if __name__ == '__main__':
    import argparse
    import sys
    parser = argparse.ArgumentParser()
    parser.add_argument('-i','--file', default="input.yml",help='YAML-file')
    args = parser.parse_args(sys.argv[1:])
    main(args)
