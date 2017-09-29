


class frame:

    #def __init__(self, car_x_y, velocities,last,tick):
    def __init__(self, car_x_y, velocities, frame_id):
        self.car_position = car_x_y
        self.car_velocities = velocities
        self.tick = frame_id
        #self.last_frame = last
        #self.next_frame = None
        #self.tick = tick


    def get_car_position(self):
        return self.car_position
