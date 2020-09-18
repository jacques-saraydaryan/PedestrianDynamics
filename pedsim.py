# Made with Qt Creator
# Work in Progress

"""
Code for simulating pedestrians with social force model and collision control algorithm
--------------------A simplified model with appropriate assumptions--------------------
Parametrs set in the file 'input.yml'
"""


import os

import collisionforcemodel as cfm

def main(args):
    print ("Hello World")
    loader = cfm.FileReader(args.file)
    world = loader.world

    print ('Initial: ',world.get_height())
    world.set_height(20)
    print ('Final: ', world.get_height())





if __name__ == '__main__':
    import argparse
    import sys
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', '--file', default="input.yml", help='YAML-file')
    args = parser.parse_args(sys.argv[1:])
    main(args)
