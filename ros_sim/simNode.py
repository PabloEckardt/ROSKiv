import rospy
from race.msg import drive_param


def update_params(data):
    with open("published_dp.txt", "w") as outfile:
        outfile.write(str(data.angle))


if __name__ == '__main__':

    rospy.init_node('simNode', anonymous=True)
    rospy.Subscriber("drive_parameters", drive_param, update_params)
    rospy.spin()
