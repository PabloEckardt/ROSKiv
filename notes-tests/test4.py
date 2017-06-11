from kivy.app import App
from kivy.uix.widget import Widget
from kivy.properties import NumericProperty, ReferenceListProperty,\
    ObjectProperty
from kivy.vector import Vector
from kivy.clock import Clock
from random import randint



class square(Widget):
    velocity_x = NumericProperty(0)
    velocity_y = NumericProperty(0)
    velocity = ReferenceListProperty(velocity_x, velocity_y)

    def move(self):
        self.pos = Vector(*self.velocity) + self.pos

class looper(Widget):
    self.sq = ObjectProperty(None)

    def update(selfself, dt):
        self.sq.move()


class testApp(App):

    def build (self):
        loop = looper()
        Clock.schedule_interval(loop.update, 1.0/60.0)
        return loop

if __name__ == '__main__':
    testApp().run()