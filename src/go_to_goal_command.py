#!/usr/bin/env python3

"""
Script used to send a goal command to go_to_goal.py manually, or via part1_demo.launch.
"""

# imports
import argparse
import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool as BoolMSG
from typing import Tuple


class GoalCommand:
    def __init__(self):
        """
        Temporary node used to send a goal command.
        """
        # create listener ready flag
        self.listener_ready = False

        # create node & pose publisher
        rospy.init_node('goal_command')
        self.pub = rospy.Publisher('/goal', Pose, queue_size=10)

        # create readiness listener
        self.ready_listener = rospy.Subscriber('/goal_ready', BoolMSG, self.goal_listener_callback)

    def __del__(self):
        # delete publisher
        self.pub.unregister()
        del self.pub

    def goal_listener_callback(self, msg):
        """
        Callback function for receiving goal listener readiness.
        :param msg: The bool message received from the goal listener
        """
        self.listener_ready = msg.data

    def publish(self, goal: Pose):
        """
        Publishes the goal pose to the ``/goal`` topic.
        :param goal: The goal pose to send
        """
        while not self.listener_ready:
            pass
        self.pub.publish(goal)


def main(goal: Tuple[float, float]):
    """
    Main function. Sends a goal command.
    :param goal: The goal position to send=
    """
    # create node
    node = GoalCommand()

    # construct goal pose
    goal_pose = Pose()
    goal_pose.position.x = goal[0]
    goal_pose.position.y = goal[1]
    goal_pose.orientation.w = 1.0

    # publish goal pose
    node.publish(goal_pose)

    # kill node
    del node


# run on exec
if __name__ == '__main__':
    # get goal position arguments
    parser = argparse.ArgumentParser(prog='go_to_goal_command.py',
                                     description='Sends a goal command via the /goal topic')
    parser.add_argument("goal_x", type=float, help="The x position of the desired goal")
    parser.add_argument("goal_y", type=float, help="The y position of the desired goal")
    parser.add_argument("__name", default=None, help="catcher for ROS backend")
    parser.add_argument("__log", default=None, help="catcher for ROS backend")
    args = parser.parse_args()

    # pass to main
    main((args.goal_x, args.goal_y))
