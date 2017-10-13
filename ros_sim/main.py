from kivy.app import App
from kivy.uix.widget import Widget
from kivy.properties import NumericProperty, ReferenceListProperty,\
    ObjectProperty
from kivy.vector import Vector
from kivy.clock import Clock
from kivy.graphics import Line
from kivy.animation import Animation
from random import randint
from geometry_funcs import find_intersection
from frame import frame
import math
from kivy.uix.textinput import TextInput


from kivy.config import Config
Config.set('graphics', 'width', '1400')
Config.set('graphics', 'height', '720')
Config.set('graphics', 'resizable', False)

RENDERED_FRAMES =[frame()]
CURRENT_FRAME_ID = 0
LATEST_PUB_ANGLE = .5

def on_enter(instance, value):
    print('User pressed enter in', instance)


### BARRIER AND WALL CLASSSES
class Wall(Widget):
    pass

class Barrier(Widget):
    pass

def get_current_frame_id():

    global CURRENT_FRAME_ID
    return CURRENT_FRAME_ID

def set_current_frame_id(num):
    global CURRENT_FRAME_ID
    CURRENT_FRAME_ID= num

def get_next_frame(RENDERED_FRAMES):

    """
    testing moving around frames
    if get_current_frame_id() == 200:
        print ("reseting")
        set_current_frame_id(0)
        return get_next_frame(RENDERED_FRAMES)
    """

    current_frame = RENDERED_FRAMES[get_current_frame_id()]

    if not (current_frame.tick == len(RENDERED_FRAMES)-1):
        # if current is not the latest frame (we are repeating our steps)
        set_current_frame_id(current_frame.tick + 1)
        return RENDERED_FRAMES[get_current_frame_id()]
    else:

        curr_tick = current_frame.tick + 1
        new_angle = current_frame.curr_angle + LATEST_PUB_ANGLE

        velocity_x = math.cos((new_angle * math.pi) / 180)
        velocity_y = math.sin((new_angle * math.pi) / 180)

        new_position = current_frame.pos + Vector(velocity_x,velocity_y)

        new_frame = frame(new_position,curr_tick,new_angle)
        RENDERED_FRAMES.append(new_frame)
        set_current_frame_id(curr_tick)

        return new_frame



class SimCar(Widget):

    def move(self, frame):

        self.angle = frame.curr_angle
        self.pos = frame.pos


class Simulator(Widget): # Root Widget

    global x, y
    global RENDERED_FRAMES

    lidar_angle = 0
    car_x_label = NumericProperty(0)
    car_y_label = NumericProperty(0)

    car = ObjectProperty(None) # Get a reference of the car object defined
                              # in the widget rules

    textinput = TextInput(text='Hello world', multiline=False)
    textinput.bind(on_text_validate=on_enter)

    barrier = ObjectProperty(None)

    def start_vehicle(self):
        with self.canvas:
            self.lidar_beam = Line(points=[0,0,0,0])
            self.barrier = Line(points=[2000,900,800,900])


    def check_border_collision(self):
        if self.car.collide_widget(self.wall_left) or self.car.collide_widget(self.wall_right):
            return True
        if self.car.collide_widget(self.wall_top) or self.car.collide_widget(self.wall_down):
            return True

        return False

    def reset(self):
        set_current_frame_id(0)



    def update(self, dt):

        frame = get_next_frame(RENDERED_FRAMES)
        self.car.move(frame)
        self.car_x_label = frame.pos[0]
        self.car_y_label = frame.pos[1]


        if self.check_border_collision():
            # define behaviour pause, or reset
            # reset for now
            self.reset()
            pass
        else:

            LIDAR_TO_CAR_ANGLE = 45
            LIDAR_RANGE = 250

            car_center_x, car_center_y = self.car.center[0], self.car.center[1]

            # adjust angle so it remains relative to the car
            adj_angle = self.lidar_angle + self.car.angle + LIDAR_TO_CAR_ANGLE

            # define a x for the lidar's end point
            lidar_target_x = (math.cos((adj_angle * math.pi)/180) * LIDAR_RANGE) + car_center_x
            # define a y for the lidar's end point
            lidar_target_y = (math.sin((adj_angle * math.pi)/180) * LIDAR_RANGE) + car_center_y

            # update the lidar
            self.lidar_beam.points = [car_center_x, car_center_y, lidar_target_x, lidar_target_y]

            # lidar collisions with barrier

            p1 = (self.lidar_beam.points[0], self.lidar_beam.points[1])
            p2 = (self.lidar_beam.points[2], self.lidar_beam.points[3])
            p3 = (self.barrier.points[0], self.barrier.points[1])
            p4 = (self.barrier.points[2], self.barrier.points[3])
            distance = find_intersection(p1,p2,p3,p4)

            if distance is not None:
                print ("distance to barrier:", distance)




class SimApp(App):
    def build(self):
        simulator = Simulator()
        simulator.start_vehicle()
        Clock.schedule_interval(simulator.update, 1.0/60.0)
        return simulator

if __name__ == '__main__':
    SimApp().run()
