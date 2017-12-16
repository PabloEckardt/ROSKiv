import numpy as np
import cv2
import time

CIRCLE_RADIOUS = 300  # .3 meters
MIN_CIRCLE_RADIOUS = 10  # .3 meters
WIDTH = 512
HEIGHT = 512


def get_x_offset(x_offset):
    return int(np.interp(x_offset, [0, 60], [-120, 120]))


def make_image(distance, count, x_offset, color=(0, 0, 255)):

    #distance = distance - 50
    angle = 0
    radious = np.interp(
        distance, [
            0, 80], [
            float(CIRCLE_RADIOUS), float(MIN_CIRCLE_RADIOUS)])
    img = np.zeros((512, 512, 3), np.uint8)
    x = get_x_offset(x_offset)
    cv2.circle(img, (256 + x, 256), int(radious), color, -1)
    pad_digits = 5 - len(str(count))

    # TODO potentially publish the img object to simplify interfacing
    # If done writing to disk would be optional.

    cv2.imwrite(
        "/home/ubuntu/catkin_ws/src/race/src/camera_feed/" +
        "im" +
        pad_digits *
        "0" +
        str(count) +
        ".png",
        img)
