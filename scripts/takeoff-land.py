#! /usr/bin/env python
# Import ROS.
import time
import rospy
# Import the API.
from iq_gnc.py_gnc_functions import *
# To print colours (optional).
from iq_gnc.PrintColours import *


def main():
    # Initializing ROS node.
    rospy.init_node("drone_controller", anonymous=True)

    # Create an object for the API.
    drone = gnc_api()
    # Wait for FCU connection.
    drone.wait4connect()
    # Wait for the mode to be switched.
    drone.set_mode("GUIDED")

    # Create local reference frame.
    drone.initialize_local_frame()
    # Request takeoff with an altitude of 3m.
    drone.takeoff(3)
    drone.set_destination(0,0,3,0)
    # Specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish.
    rate = rospy.Rate(3)

    time.sleep(10)
    # Land after all waypoints is reached.
    drone.land()
    rospy.loginfo(CGREEN2 + "landing now." + CEND)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit()
