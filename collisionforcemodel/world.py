"""
--------------------------------Class: World----------------------------------
The grid of interest, where the simulation is run
"""

class World(object):
    def __init__(self):
        self.height = 0
        self.width = 0

    def set_height(self,height):
        self.height = height

    def set_width(self,width):
        self.width = width

    def get_height(self):
        return self.height

    def get_width(self):
        return self.width
