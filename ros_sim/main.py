from kivy.app import App
from kivy.uix.widget import Widget
from kivy.properties import NumericProperty, ReferenceListProperty,\
    ObjectProperty
from kivy.vector import Vector
from kivy.clock import Clock
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
            #self.angle -= 10
            pass
        #self.velocity_x = math.cos((self.angle * math.pi) / 180)
        #self.velocity_y = math.sin((self.angle * math.pi) / 180)
        self.velocity_y = yv
        self.velocity_x = xv
        self.pos = Vector(*self.velocity) + self.pos

        x = self.center_x / 1
        y = self.center_y / 1
        center = self.center

class Simulator(Widget): # Root Widget
    locator_x = NumericProperty(0)
    locator_y = NumericProperty(0)

    car = ObjectProperty(None) # Get a reference of the car object defined
                              # in the widget rules
    def start_vehicle(self):
        self.car.center = self.center
        self.car.velocity = Vector(4, 0).rotate(randint(0, 360))

    def update(self, dt):
        global x, xv
        global y, yv
        self.car.move()

        self.locator_x = x
        self.locator_y = y

        if self.car.collide_widget(self.wall_left) or self.car.collide_widget(self.wall_right):
            self.car.center = self.center
            xv *= -1

        if self.car.collide_widget(self.wall_top) or self.car.collide_widget(self.wall_down):
            self.car.center = self.center
            yv *= -1


class SimApp(App):
    def build(self):
        simulator = Simulator()
        simulator.start_vehicle()
        Clock.schedule_interval(simulator.update, 1.0/120.0)
        return simulator


if __name__ == '__main__':
    SimApp().run()
