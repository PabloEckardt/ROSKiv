from kivy.app import App
from kivy.uix.widget import Widget
from kivy.properties import NumericProperty, ReferenceListProperty,\
    ObjectProperty
from kivy.vector import Vector
from kivy.clock import Clock
from kivy.graphics import Line
from random import randint

from kivy.animation import Animation

import math

from kivy.config import Config
Config.set('graphics', 'width', '1400')
Config.set('graphics', 'height', '720')
Config.set('graphics', 'resizable', False)

x = 0
y = 0
center = 0
ticks = 0
xv = 1
yv = 1

class Wall(Widget):
    pass

class Barrier(Widget):
    pass

class SimCar(Widget):

    velocity_x = NumericProperty(0)
    velocity_y = NumericProperty(0)

    angle = NumericProperty(0)

    velocity = ReferenceListProperty(velocity_x, velocity_y)


    def move(self):
        global x
        global y
        global center
        global ticks
        global xv, xy
        ticks += 1
        if ticks % 100 == 0:
            self.angle -= 10
            pass
        self.velocity_x = math.cos((self.angle * math.pi) / 180)
        self.velocity_y = math.sin((self.angle * math.pi) / 180)

        #self.velocity_y = yv
        #self.velocity_x = xv

        self.pos = Vector(*self.velocity) + self.pos

        x = self.center_x / 1
        y = self.center_y / 1
        center = self.center

class Simulator(Widget): # Root Widget

    global x, xv
    global y, yv
    lidar_angle = 180
    car_x = NumericProperty(0)
    car_y = NumericProperty(0)

    car = ObjectProperty(None) # Get a reference of the car object defined
                              # in the widget rules


    def start_vehicle(self):
        self.car.center = self.center
        with self.canvas:
            self.lidar_beam = Line(points=[0,0,0,0])

    def update(self, dt):


        self.car.move()

        self.car_x = x
        self.car_y = y

        if self.car.collide_widget(self.wall_left) or self.car.collide_widget(self.wall_right):
            self.car.center = self.center

        if self.car.collide_widget(self.wall_top) or self.car.collide_widget(self.wall_down):
            self.car.center = self.center

        self.lidar_beam.points = [self.car.pos[0] + 25, self.car.pos[1] + 25, self.car.pos[0] + 180, self.car.pos[1] + 180]

class SimApp(App):
    def build(self):
        simulator = Simulator()
        simulator.start_vehicle()
        #simulator.gen_beams([90,180,270])
        Clock.schedule_interval(simulator.update, 1.0/60.0)
        return simulator


if __name__ == '__main__':
    SimApp().run()
