from kivy.vector import Vector

class frame:
    def __init__(self, pos=Vector(200,100),tick=0,curr_angle=0):

        self.curr_angle = curr_angle
        self.pos = pos
        self.tick = tick

