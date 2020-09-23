#*********************************************************************************
# CLASS FileReader - Reads input from file input.yml and sets up World
#*********************************************************************************


from yaml import load,dump
from yaml import Loader,Dumper


from .world import World
from .pedestrian import Pedestrian

class FileReader:
    def __init__(self,filename=None):
        if filename:
            with open(filename) as file:
                data = load(file,Loader=Loader)

        # Create a World Class
        world = World()
        self.world = world

        # Create a Standard Parse Set with Variables
        standard_parse_functions = {
            'pedestrian_mass': world.set_pedestrianmass,
            'desired_velocity': world.set_desiredvelocity,
            'maximum_velocity': world.set_maxvelocity,
            'flux_right': world.set_fluxright,
            'flux_up': world.set_fluxup,
            'spawn_point1x': world.set_point1x,
            'spawn_point1y': world.set_point1y,
            'spawn_point2x': world.set_point2x,
            'spawn_point2y': world.set_point2y,
            'delta_t': world.set_delta_t,
            'spawn_pedflux1': world.set_pedflux1,
            'spawn_pedflux2': world.set_pedflux2,
            'target_point1x': world.set_target1x,
            'target_point1y': world.set_target1y,
            'target_point2x': world.set_target2x,
            'target_point2y': world.set_target2y,
            'print_png': world.set_printpng
            }


        # Set up the Keys in World Class
        for key in data:
            if key in standard_parse_functions:
                standard_parse_functions[key](data[key])



