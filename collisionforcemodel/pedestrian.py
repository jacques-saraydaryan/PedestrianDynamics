# This Python file uses the following encoding: utf-8


class Pedestrian:
    def __init__(self,xposition,yposition,dir):
        self.position = [xposition,yposition]
        self.id = 0
        self.max_velocity = 0.5
        self.desired_direction = dir

    def set_position(self,position,id,dir):
        self.position = position
        self.id = id
        self.desired_direction = dir

    def get_pedpos(self):
        return self.position

    def get_id(self):
        return self.id


