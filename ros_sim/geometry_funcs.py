import math
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon

def contains(point, polygon):

    p = Point(point[0],point[1])
    pol = Polygon(polygon)
    return pol.contains(p)


def dist_two_points(p1,p2):
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def find_intersection( p0, p1, p2, p3 ) :

    s10_x = p1[0] - p0[0]
    s10_y = p1[1] - p0[1]
    s32_x = p3[0] - p2[0]
    s32_y = p3[1] - p2[1]

    denom = s10_x * s32_y - s32_x * s10_y

    if denom == 0 : return None # collinear

    denom_is_positive = denom > 0

    s02_x = p0[0] - p2[0]
    s02_y = p0[1] - p2[1]

    s_numer = s10_x * s02_y - s10_y * s02_x

    if (s_numer < 0) == denom_is_positive : return None # no collision

    t_numer = s32_x * s02_y - s32_y * s02_x

    if (t_numer < 0) == denom_is_positive : return None # no collision

    if (s_numer > denom) == denom_is_positive or (t_numer > denom) == denom_is_positive : return None # no collision

    # collision detected

    t = t_numer / denom

    intersection_point = [ p0[0] + (t * s10_x), p0[1] + (t * s10_y) ]

    d = dist_two_points (p0, intersection_point)

    return d
    
def deg_to_rads(deg):
    return (deg*math.pi)/180.0

def get_bounding_points(x, y, angle):
    """
    given the center of the car, get the 4 points bounding the vehicle which
    rotate.
    """
    # get the angles relative to the vehicle orientation.

    d_angle_1 = angle + 26.6
    d_angle_2 = (180 - 26) + angle
    d_angle_3 = 180 + d_angle_1
    d_angle_4 = angle - 26.6

    angle_1 = deg_to_rads(d_angle_1)
    angle_2 = deg_to_rads(d_angle_2)
    angle_3 = deg_to_rads(d_angle_3)
    angle_4 = deg_to_rads(d_angle_4)

    # we know the car's dimensions.
    # this is the diagonal size of the car divided by two
    radious = 13.975

    point1 = [radious*math.cos(angle_1) + x, radious*math.sin(angle_1) + y]
    point2 = [radious*math.cos(angle_2) + x, radious*math.sin(angle_2) + y]
    point3 = [radious*math.cos(angle_3) + x, radious*math.sin(angle_3) + y]
    point4 = [radious*math.cos(angle_4) + x, radious*math.sin(angle_4) + y]

    return [point1,point2,point3,point4]

def meters_to_pixels(meters):
    # scale is 25 pixels per meter.
    return meters * 25 