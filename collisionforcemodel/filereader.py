# This Python file uses the following encoding: utf-8
from yaml import load,dump
from yaml import Loader,Dumper


from .world import World
from .pedestrian import Pedestrian
from .obstacles import Obstacles

class FileReader:
    def __init__(self,filename=None):
        if filename:
            with open(filename) as file:
                data = load(file,Loader=Loader)

        world = World()
        self.world = world


        standard_parse_functions = {
            'world_height': world.set_height,
            'world_width': world.set_width,
            'pedestrian_mass': world.set_pedestrianmass,
            'pedestrian_radius': world.set_pedestrianradius,
            'desired_velocity': world.set_desiredvelocity,
            'maximum_velocity': world.set_maxvelocity,
            'relaxation_time': world.set_relaxationtime,
            'flux_right': world.set_fluxright,
            'flux_up': world.set_fluxup,
            'spawn_points': world.set_spawnpoints,
            'spawn_point1x': world.set_point1x,
            'spawn_point1y': world.set_point1y,
            'spawn_point2x': world.set_point2x,
            'spawn_point2y': world.set_point2y,
            'spawn_point3x': world.set_point3x,
            'spawn_point3y': world.set_point3y,
            'spawn_point4x': world.set_point4x,
            'spawn_point4y': world.set_point4y,
            'delta_t': world.set_delta_t,
            'spawn_pedflux1': world.set_pedflux1,
            'spawn_pedflux2': world.set_pedflux2,
            'spawn_pedflux3': world.set_pedflux3,
            'spawn_pedflux4': world.set_pedflux4,
            'target_points': world.set_targetpoints,
            'target_point1x': world.set_target1x,
            'target_point1y': world.set_target1y,
            'target_point2x': world.set_target2x,
            'target_point2y': world.set_target2y
            }



        for key in data:
            if key in standard_parse_functions:
                standard_parse_functions[key](data[key])



    def parse_point(self,point):
        return np.array([point['x'], point['y']])

