from kivy.app import App
from kivy.uix.widget import Widget
from kivy.properties import NumericProperty, ReferenceListProperty,\
     ObjectProperty, StringProperty
from kivy.vector import Vector
from kivy.clock import Clock
from kivy.graphics import Line, Color
from kivy.animation import Animation
from kivy.lang import Builder
from random import randint
from kivy.uix.textinput import TextInput
from kivy.config import Config

import rospy
from race.msg import pid_input
from race.msg import pid_input

import  json
import  pickle
import  os
import  math
import  time
from    enum import Enum
from    datetime import datetime
from    numpy import interp
from    frame import frame
from    geometry_funcs import find_intersection, get_bounding_points, dist_two_points, deg_to_rads
import sys


pub = rospy.Publisher('error', pid_input, queue_size=1)
Builder.load_file("Sim.kv")
Config.set('graphics', 'width', '1400')
Config.set('graphics', 'height', '720')
Config.set('graphics', 'resizable', False)


class SAVE_OPTIONS(Enum):
    NO = 1
    FAILURE = 2
    SUCCESS_FAILURE = 3

RENDERED_FRAMES =[frame()]
CURRENT_FRAME_ID = 0
LATEST_PUB_ANGLE = 0
SAVE_TRIGGER = SAVE_OPTIONS.NO
MAP_SELECTION = 1
TARGET_CENTER = []
STATUS_LABEL = "UNDETERMINED"
LIDAR_TO_CAR_ANGLE = 45 # degrees
LIDAR_RANGE = 250 # in cms
SAVE_FILE = False
FRAME_RATE = 60
TRACE_ROUTE = True


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


def update_angle_target():
    global LATEST_PUB_ANGLE
    try:
        line = None
        with open("published_dp.txt", "r") as infile:
            line = infile.read()
            LATEST_PUB_ANGLE = float(line)
    except Exception as e:
        pass


def get_save_trigger(n):
    if n == 1:
        return SAVE_OPTIONS.NO
    elif n == 2:
        return SAVE_OPTIONS.FAILURE
    else:
        return SAVE_OPTIONS.SUCCESS_FAILURE


def get_next_frame(RENDERED_FRAMES):

    try:
        current_frame = RENDERED_FRAMES[get_current_frame_id()]
        if not (current_frame.tick == len(RENDERED_FRAMES)-1):
            # if current is not the latest frame (we are repeating our steps)
            set_current_frame_id(current_frame.tick + 1)
            return RENDERED_FRAMES[get_current_frame_id()]
        else:
            update_angle_target();
            new_angle = LATEST_PUB_ANGLE
            curr_tick = current_frame.tick + 1
            new_frame = frame(tick=curr_tick,target_angle=new_angle)
            RENDERED_FRAMES.append(new_frame)
            set_current_frame_id(curr_tick)
            return new_frame
    except Exception as frame_exception:
        print frame_exception
        sys.exit(1)


class SimCar(Widget):


    relative_angle = 0

    def get_effective_angle(self, relative_angle):
        relative_angle = 75 if relative_angle > 75 else -75 if relative_angle < -75 else relative_angle
        return interp(relative_angle, [-75.0,75.0],[-.4,.4])


    def move(self, frame):
        try:

            self.angle += self.get_effective_angle(frame.target_angle)


            velocity_x = math.cos((self.angle * math.pi) / 180)
            velocity_y = math.sin((self.angle * math.pi) / 180)

            self.pos = Vector(self.pos[0], self.pos[1]) + Vector(velocity_x,velocity_y)

        except Exception as move_exception:
            print move_exception
            sys.exit(1)


class Simulator(Widget): # Root Widget

    global x, y
    global RENDERED_FRAMES

    lidar_angle = 0
    car_x_label = StringProperty("")
    car_y_label = StringProperty("")
    car_angle_label = StringProperty("")

    car = ObjectProperty(None) # Get a reference of the car object defined
                              # in the widget rules

    barriers = None

    def trace_route(self,point1,point2,point3,point4):

        if len(RENDERED_FRAMES) % 10 == 0:
            with self.canvas:
                Color(1,0,0)
                self.mark_lines.append(ObjectProperty(None, allownone=True))
                self.mark_lines[len(self.mark_lines) - 1] = Line(circle = 
                                            (point1[0], point1[1],1), width = 1)

                Color(0,1,0)

                self.mark_lines.append(ObjectProperty(None, allownone=True))
                self.mark_lines[len(self.mark_lines) - 1] = Line(circle = 
                                            (point2[0], point2[1],1), width = 1)

                Color(0,0,1)
                self.mark_lines.append(ObjectProperty(None, allownone=True))
                self.mark_lines[len(self.mark_lines) - 1] = Line(circle = 
                                            (point3[0], point3[1],1), width = 1)
                Color(1,1,1)

                self.mark_lines.append(ObjectProperty(None, allownone=True))
                self.mark_lines[len(self.mark_lines) - 1] = Line(circle = 
                                            (point4[0], point4[1],1), width = 1)


    def load_map(self,map_id):

        self.mark_lines = []
        global TARGET_CENTER
        print "loading the map"
        with self.canvas:
            with open ("maps.json", "r") as mapfile:
                map_data = json.load(mapfile)
                # select a map from the json
                m = map_data[str(map_id)]
                # Draw the lines
                self.barriers = []
                for k in m:
                    if not k == "target":
                        self.barriers.append(ObjectProperty(None,
                                                            allownone=True))
                        _line = map(int,m[k].split(","))
                        points = _line[0:4] 
                        r = _line[4]
                        g = _line[5]
                        b = _line[6]
                        Color(r,g,b)
                        self.barriers[len(self.barriers) - 1] = Line(points=points)
                # Draw the target
                target = map(int,m["target"].split(","))
                TARGET_CENTER = [int(target[0]), int(target[1])]
                self.target = ObjectProperty(None,allownone=True)
                Color (67/255.0,242/255.0,155/255.0)
                self.target = Line(circle=( 
                                            TARGET_CENTER[0],
                                            TARGET_CENTER[1], 
                                            target[2]
                                            ), width = 3)
            assert len (TARGET_CENTER) == 2

    def start_vehicle(self):
        with self.canvas:
            self.lidar_beam = Line(points=[0,0,0,0])
            self.pos = Vector(-500,-200)


    def check_border_collision(self):
        global STATUS_LABEL
        if self.car.collide_widget(self.wall_left)  \
        or self.car.collide_widget(self.wall_right) \
        or self.car.collide_widget(self.wall_top)   \
        or self.car.collide_widget(self.wall_down):
            STATUS_LABEL = "FAILURE"
            return True
        return False



    def check_barrier_collision(self):
        global STATUS_LABEL
        # car's bounding box
        bb = get_bounding_points(self.car.get_center_x(), 
                                        self.car.get_center_y(), 
                                        self.car.angle)
        car_lines = [
                    (bb[0],bb[1]),
                    (bb[1],bb[2]), 
                    (bb[2],bb[3]),
                    (bb[3],bb[0]) 
                    ]

        if TRACE_ROUTE:
            self.trace_route(bb[0],bb[1],bb[2],bb[3])
            pass

        for car_line in car_lines:
            p1,p2 = car_line        
            for b in self.barriers:
                p3 = (b.points[0], b.points[1])
                p4 = (b.points[2], b.points[3])
                _distance = find_intersection(p1,p2,p3,p4)
                if _distance is not None: 
                    # found collision
                    STATUS_LABEL = "FAILURE"
                    return True
        return False

    def get_lidar_measurement(self):

        distance = 1000
        p1 = (self.lidar_beam.points[0], self.lidar_beam.points[1])
        p2 = (self.lidar_beam.points[2], self.lidar_beam.points[3])
        for b in self.barriers:
            p3 = (b.points[0], b.points[1])
            p4 = (b.points[2], b.points[3])
            _distance = find_intersection(p1,p2,p3,p4)
            if _distance is not None: 
                # on many intersecting walls, pick the closest one
                distance = min(distance,_distance)
        return distance

    def check_for_target(self):
        global STATUS_LABEL
        p1 = (self.car.get_center_x(), self.car.get_center_y())
        p2 = TARGET_CENTER
        # 60 is an arbitrary radious to determine if the vehicle is close enough
        if dist_two_points(p1,p2) <= 60:
            STATUS_LABEL = "SUCCESS"
            return True
        return False

    def reset(self):
        global TRACE_ROUTE
        set_current_frame_id(0)
        self.car.pos = [150,135]
        self.car.angle = 0
        TRACE_ROUTE = False

    def draw_labels(self):
        self.car_x_label = "x: " + str(int(self.car.get_center_x()))
        self.car_y_label = "y: " + str(int(self.car.get_center_y()))
        try:
            self.car_angle_label ="angle: " + str(int(self.car.angle))
        except Exception as e:
            print e

    def save_simulation(self):

        # Prevent multiple saves per simulation
        global SAVE_FILE

        if SAVE_TRIGGER == SAVE_OPTIONS.FAILURE \
            or SAVE_TRIGGER == SAVE_OPTIONS.SUCCESS_FAILURE:

            if not SAVE_FILE:
                SAVE_FILE = True
                tag = STATUS_LABEL
                file_name = "simulations/"+tag+str(datetime.now())+"||"+str(MAP_SELECTION)
                with open (file_name, "w") as outfile:
                    try:
                        pickle.dump(RENDERED_FRAMES, outfile)
                        print ("saved log file: " + file_name)
                    except Exception as e:
                        print ("failure to save log") 


    def update(self, dt):

        global LIDAR_RANGE
        global LIDAR_TO_CAR_ANGLE

        print(len(RENDERED_FRAMES))
        if len(RENDERED_FRAMES) > 5:
            pass

        frame = get_next_frame(RENDERED_FRAMES)
        self.car.move(frame)
        self.draw_labels()
        barrier_collision = False
        border_collision = False
        target_reached = False
        try:
            barrier_collision = self.check_barrier_collision()
            border_collision = self.check_border_collision()
            target_reached = self.check_for_target()
        except Exception as e:
            print e
        if barrier_collision or border_collision or target_reached:
            self.save_simulation()
            print ("resetting")
            self.reset()
        else:
            car_center_x, car_center_y = self.car.get_center_x(), self.car.get_center_y()
            # adjust angle so it remains relative to the car
            adj_angle = self.lidar_angle + self.car.angle + LIDAR_TO_CAR_ANGLE

            # define a x for the lidar's beam end point
            lidar_target_x = (math.cos((adj_angle * math.pi)/180) \
                             * LIDAR_RANGE) + car_center_x
            lidar_target_y = (math.sin((adj_angle * math.pi)/180) \
                             * LIDAR_RANGE) + car_center_y

            # update the lidar
            self.lidar_beam.points = [  car_center_x,
                                        car_center_y, 
                                        lidar_target_x, 
                                        lidar_target_y]

            # lidar collisions with barriers
            distance = self.get_lidar_measurement()
            msg = pid_input()
            if distance is not 1000:
                ############ YOUR CODE GOES HERE ################
                # distance = distance to closer barrier if within range
                msg.pid_error = interp(distance, [0,250],[-100,100])
            else:
                msg.pid_error = 1000

            pub.publish(msg)


class SimApp(App):
    global MAP_SELECTION
    def build(self):
        global FRAME_RATE
        simulator = Simulator()
        simulator.start_vehicle()
        simulator.load_map(MAP_SELECTION)
        Clock.schedule_interval(simulator.update, 1.0/FRAME_RATE)
        return simulator


if __name__ == '__main__':

    global MAP_SELECTION

    load_sim = None

    if len(sys.argv)  > 1:
        args = sys.argv
        load_sim = args[1]
        MAP_SELECTION = int(args[2])
        SAVE_TRIGGER = get_save_trigger(int(args[3]))

    else:
        print "Load Simulation?: y/n"
        while not load_sim == "y"   and not load_sim == "yes" \
                                    and not load_sim == "n" \
                                    and not load_sim == "no":
            load_sim = raw_input()

        if load_sim == "y" or load_sim == "yes":
            # if loading dont need to save
            SAVE_TRIGGER = SAVE_OPTIONS.NO
            l = os.listdir("simulations/")
            for index,sim in enumerate(l):
                print ( "[ " + str(index) + " ] " + sim)
            sim = -1
            while sim < 0 or sim > len(l) - 1:
                print ("select a simulation's number: ")
                sim = int(raw_input())

            file = l[sim]
            with open ("simulations/"+file, "rb") as infile:
                RENDERED_FRAMES = pickle.load(infile)

            MAP_SELECTION = int(file.split("||")[1])

        else:
            print ("Enter Map Number: ")
            MAP_SELECTION = int(raw_input())
            print "Save simulation? 1 = no, 2 = on success, 3 = on success or failure"
            _save = int(raw_input())
            assert _save == 1 or _save == 2 or _save == 3
            SAVE_TRIGGER = get_save_trigger(_save)

    rospy.init_node('sim_error', anonymous=True)

    SimApp().run()
