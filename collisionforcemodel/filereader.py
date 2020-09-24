#*********************************************************************************
# CLASS FileReader - Reads input from file input.yml and sets up World
#*********************************************************************************
"""
CLASS  FileReader
-----------------
    A Class that reads input and sets up the World

MODULES REQUIRED
----------------
    YAML

EXAMPLE
-------
    To call the class pass the input file name to class
        FileReader('filename')
    ONLY USE THE YAML INPUT FILE GIVEN WITH THIS CODE
"""

from yaml import load
from yaml import Loader

from .world import World

class FileReader:
    """
    Class FileReader - Reads and sets the simulation

    Call
    ----
    FileReader(filename)

    filename : string
        Name of the input file to be read

    Parameters
    ----------
    world : class
        Creates a world with inputed variables
    standard_parse_functions : map
        Maps the keywords to values from the input file
    key : map_string
        Keywords in input file
    data : map_datatype
        Values for each keywork in input file

    Returns
    -------
        None
    """
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
            'print_png': world.set_printpng,
            'spawn_method': world.set_spawnmethod
            }


        # Set up the Keys in World Class
        for key in data:
            if key in standard_parse_functions:
                standard_parse_functions[key](data[key])
