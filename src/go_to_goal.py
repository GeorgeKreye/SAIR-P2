#!/usr/bin/env python3

"""
ROS executable that sends a goal to a move_base agent.

If running on its own, this should be run on the command line using ``rosrun sair-p2-GeorgeKreye go_to_goal.py`` after
compiling the ROS package using catkin.
"""

# imports
from typing import Tuple
import argparse
import rospy
import tf
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool as BoolMSG


class GoToGoal:
    """
    ROS action client that moves an agent in 2D space to a given goal.
    """

    def __init__(self, verbose: bool = False):
        """
        ROS action client that moves an agent in 2D space to a given goal.
        :param verbose: Whether to have the client log additional information
        """
        # save verbose flag
        self.verbose = verbose

        # create node & client
        rospy.init_node('go_to_goal')
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        # create readiness publisher
        self.ready_pub = rospy.Publisher('/goal_ready', BoolMSG, queue_size=1)

        # create listeners
        self.tf_listener = tf.TransformListener()
        self.goal_listener = None  # initialized on run
        self.term_listener = rospy.Subscriber('/termination', BoolMSG, self.__del__)

        # set rate
        self.rate = rospy.Rate(1)  # 1 loop/sec

    def __del__(self):
        # kill goal listener
        if self.goal_listener is not None:
            self.goal_listener.unregister()
            del self.goal_listener

        # kill transform listener
        del self.tf_listener

        # kill publisher
        self.ready_pub.publish(False)  # notify listeners that node is no longer ready
        self.ready_pub.unregister()
        del self.ready_pub

        # kill client
        del self.client

        # shutdown node
        rospy.signal_shutdown("Node terminated by code")

    def _go_to_goal(self, goal_pos: Tuple[float, float]):
        """
        Main function that attempts to reach the goal.
        :param goal_pos: The 2D coordinates of the desired goal position
        :return:
        """

        # make sure server is ready
        self.client.wait_for_server()
        if self.verbose:
            rospy.loginfo("Connected to action server")

        # create goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = goal_pos[0]
        goal.target_pose.pose.position.y = goal_pos[1]
        goal.target_pose.pose.orientation.w = 1.0

        # send goal
        if self.verbose:
            rospy.loginfo(f"Going to goal {goal_pos}")
        self.client.send_goal(goal)

    def _get_pose(self) -> bool:
        """
        Gets the pose of the turtle agent and publishes it to the log.
        :return: ``True`` if the turtle's pose is successfully retrieved, ``False`` otherwise
        """
        # retrieve pose & separate into components
        try:
            (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return False  # could not successfully retrieve pose

        # publish pose to log
        pose_msg = (f"Position: ({trans[0]}, {trans[1]}, {trans[2]})\n"
                    f"Orientation: ({rot[0]}, {rot[1]}, {rot[2]}, {rot[3]})")
        rospy.loginfo(pose_msg)

    def goal_callback(self, msg):
        """
        Called when a goal pose is received from the ``/goal`` topic.
        :param msg: The pose message to extract goal data from
        """
        # get goal x and y
        goal_x = msg.position.x
        goal_y = msg.position.y

        # send goal to navigation
        rospy.loginfo("Goal received")
        self._go_to_goal((goal_x, goal_y))

    def run(self):
        """
        Runs node functionality.
        """
        # initialize goal listener
        rospy.loginfo("Listening for goal messages")
        self.goal_listener = rospy.Subscriber("/goal", Pose, self.goal_callback)

        # loop until shutdown
        rospy.loginfo("Starting position tracking")
        while not rospy.is_shutdown():
            # broadcast readiness to receive messages
            self.ready_pub.publish(BoolMSG(data=True))

            # track turtle position
            if not self._get_pose():
                if self.verbose:
                    rospy.logwarn("Could not retrieve turtle's position this loop iteration")
                self.rate.sleep()  # enforce rate
                continue

            # enforce rate
            self.rate.sleep()


def main(verbose: bool):
    """
    Main function, handles ROSPy action client and its execution.
    :param verbose: Whether functions should log additional information
    :return:
    """
    # create client
    client = GoToGoal(verbose)

    # run client
    client.run()


# run on execution
if __name__ == '__main__':
    # get command line arguments
    parser = argparse.ArgumentParser(prog="go_to_goal",
                                     description="ROSPy action client that makes an agent go to a specified goal "
                                                 "position")
    parser.add_argument("__log", default=None, help="catcher for ROS backend")
    parser.add_argument("__name", default=None, help="catcher for ROS backend")
    parser.add_argument("-v", "--verbose", action='store_true',
                        help="Whether to log additional information")
    args = parser.parse_args()

    # pass to main
    main(args.verbose)
