#!/usr/bin/env python3

"""
ROS node that handles tracking the robot's progress & terminating exploration when a certain threshold of exploration is
reached.
"""

# imports
import rospy
import argparse
from std_msgs.msg import Bool as BoolMSG
from nav_msgs.msg import OccupancyGrid


class ProgressChecker:
    def __init__(self, term_threshold: float = 0.95):
        """
        ROS node that handles tracking the robot's progress & terminating exploration when a certain entropy threshold
        is reached.
        :param term_threshold: The entropy percentage to use as the threshold for terminating exploration
        """
        # make sure termination threshold is a valid percent
        if term_threshold < 0 or term_threshold > 1:
            raise ValueError("Entropy threshold %f is not in range [0,1]" % term_threshold)

        # store termination threshold
        self.term_threshold = term_threshold  # roughly accounts for map cells beyond actual world

        # create node
        rospy.init_node('progress_checker')

        # create termination publisher
        self.term_publisher = rospy.Publisher('/termination', BoolMSG, queue_size=1)

        # create map listener holder
        self.map_listener = None

        # set rate
        self.rate = rospy.Rate(10)  # 10 hz

        # log node creation
        rospy.loginfo("Progress monitoring node created")

    def __del__(self):
        # delete listener
        if self.map_listener is not None:
            self.map_listener.unregister()
            del self.map_listener

        # delete publisher
        self.term_publisher.unregister()
        del self.term_publisher

        # shutdown node
        rospy.signal_shutdown('Node terminated by code')

    def _stop_exploration(self):
        """
        Terminates turtle exploration by continuously publishing a kill signal for nodes associated with it.
        """
        # kill map listener to prevent this being called more than once
        self.map_listener.unregister()

        # loop until node shutdown
        while not rospy.is_shutdown():
            # publish kill signal
            self.term_publisher.publish(BoolMSG(data=True))

            # sleep
            self.rate.sleep()

    def map_callback(self, msg):
        """
        Callback function for receiving occupancy grid updates, performing progress checks each update.
        :param msg: The occupancy grid map received
        """
        # calculate explored percent
        explored_percent = _get_explored_percent(msg)

        # check if threshold has been reached
        if explored_percent >= self.term_threshold:
            rospy.loginfo("Entropy threshold reached, stopping exploration")
            self._stop_exploration()

    def run(self):
        """
        Executes node functionality.
        :return:
        """
        # initialize map listener
        self.map_listener = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)


def _get_explored_percent(msg):
    """
    Helper function that calculates the percentage of the map that has been explored.
    :param msg: The occupancy grid message containing the map to use in calculation
    :return: The percentage (as a float) of the map that has been explored
    """

    # iterate through occupancy grid
    explored_count = 0
    for cell in msg.data:
        # check if cell has been explored
        if cell > -1:
            # increase explored count
            explored_count += 1

    # calculate percentage and return
    return explored_count / 9200  # hard coded estimate since obtaining from grid is unreliable


def main(term_threshold: float):
    """
    Main function. Handles node creation and execution.
    """
    # create & run node
    progress_checker = ProgressChecker(term_threshold)
    progress_checker.run()

    # wait until node is killed
    rospy.spin()


# run on exec
if __name__ == '__main__':
    # parse arguments
    parser = argparse.ArgumentParser(prog="progress_checker", description="ROS node that handles progress monitoring")
    parser.add_argument("termination_threshold", default=0.95,
                        type=float, help="Percentage of map exploration at which to terminate exploration")
    parser.add_argument("__name", default=None, help="Catcher for ROS backend")
    parser.add_argument("__log", default=None, help="Catcher for ROS backend")
    args = parser.parse_args()

    # pass to main
    main(args.termination_threshold)
