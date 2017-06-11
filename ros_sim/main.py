from kivy.app import App
from kivy.uix.widget import Widget
from kivy.properties import NumericProperty, ReferenceListProperty,\
    ObjectProperty
from kivy.vector import Vector
from kivy.clock import Clock
from random import randint
from kivy.animation import Animation

class SimCar(Widget):
    velocity_x = NumericProperty(0)
    velocity_y = NumericProperty(0)
    velocity = ReferenceListProperty(velocity_x, velocity_y)

    angle = NumericProperty(0)

    def move(self):
        self.pos = Vector(*self.velocity) + self.pos
        self.angle += 1

        Animation(angle=self.angle)

class Simulator(Widget): # Root Widget
    car = ObjectProperty(None) # Get a reference of the car object defined
                              # in the widget rules
    def start_vehicle(self):
        self.car.center = self.center
        self.car.velocity = Vector(4, 0).rotate(randint(0, 360))

    def update(self, dt):
        self.car.move()


        if (self.car.y < 0) or self.car.top > self.height:
            self.car.velocity_y *= -1


        if (self.car.x < 0) or self.car.right > self.width:
            self.car.velocity_x *= -1


class SimApp(App):
    def build(self):
        simulator = Simulator()
        simulator.start_vehicle()
        Clock.schedule_interval(simulator.update, 1.0/60.0)
        return simulator


if __name__ == '__main__':
    SimApp().run()
