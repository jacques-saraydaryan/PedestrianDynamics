"""
-------------------------FileReader Class---------------------------------
Keywords from file 'input.yml' is matched with appropriate variables of
world, where simulation is run
"""


from yaml import load, dump
from yaml import Loader, Dumper


from .world import World



class FileReader(object):
    def __init__(self,filename=None):
        if filename:
            with open(filename) as file:
                data = load(file, Loader=Loader)
        world = World()
        self.world = world


        standard_parse_functions = {
            'world_height': world.set_height,
            'world_width': world.set_width,
            'time_step': world.set_timestep,
            'spawn_1x' : world.set_spawn_1x,
            'spawn_1y' : world.set_spawn_1y,
            'spawn_2x' : world.set_spawn_2x,
            'spawn_2y' : world.set_spawn_2y
            }

        for key in data:
            if key in standard_parse_functions:
                standard_parse_functions[key](data[key])
