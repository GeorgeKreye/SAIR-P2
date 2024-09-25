#!/usr/bin/env python3

"""
ROS node that ranks and selects centroids for exploration.
"""

# imports
import rospy
import tf
import math
import numpy as np
import argparse
from typing import List, Tuple, Any
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import ColorRGBA
from std_msgs.msg import Bool as BoolMSG
from geometry_msgs.msg import Pose

# selection color constant; random group colors very unlikely to be white
_SELECTION_COLOR = ColorRGBA()
_SELECTION_COLOR.r = 1.0
_SELECTION_COLOR.g = 1.0
_SELECTION_COLOR.b = 1.0
_SELECTION_COLOR.a = 1.0


class FrontierExplorer:
    """
    ROS node that plans robot exploration using frontiers.
    """

    def __init__(self, proximity_weight: float = 1.0, size_weight: float = 1.0, verbose: bool = False):
        """
        ROS node that plans robot exploration using frontiers.
        :param proximity_weight: Weight used for centroid proximity in centroid ranking
        :param size_weight: Weight used for frontier size in centroid ranking
        :param verbose: Whether to log additional information
        """
        # store flags
        self.verbose = verbose
        self.gtg_ready = False

        # store weights
        self.proximity_weight = proximity_weight
        self.size_weight = size_weight

        # initialize node
        rospy.init_node('frontier_explorer')

        # initialize transform listener
        self.position_listener = tf.TransformListener()

        # initialize publishers
        self.frontier_publisher = rospy.Publisher('/frontier_markers', MarkerArray, queue_size=10)
        self.goal_publisher = rospy.Publisher('/goal', Pose, queue_size=10)

        # initialize node readiness listener
        self.gtg_ready_listener = rospy.Subscriber('/goal_ready', BoolMSG, self.gtg_ready_callback)

        # initialize termination listener
        self.term_listener = rospy.Subscriber('/termination', BoolMSG, self.__del__)

        # make empty holders
        self.centroids = None
        self.frontier_groups = None
        self.frontier_listener = None
        self.frontier_sizes = None
        self.map_diagonal = None
        self.map_diagonal_listener = None
        self.pose = None
        self.ranks = None
        self.target_centroid = None

        # initialize flags
        self.centroids_updated = False
        self.groupings_determined = False
        self.should_terminate = False

        # set update rate
        self.rate = rospy.Rate(10)

        # log node creation
        rospy.loginfo("Frontier exploration node initialized")

    def __del__(self):
        # unregister listeners
        if self.frontier_listener is not None:
            self.frontier_listener.unregister()
            del self.frontier_listener
        if self.map_diagonal_listener is not None:
            self.map_diagonal_listener.unregister()
            del self.map_diagonal_listener

        # send shutdown signal
        rospy.signal_shutdown('Node terminated by code')

    def _get_turtle_pos(self) -> bool:
        """
        Gets the current position and orientation of the turtle agent, storing it as a pose object.
        :return: The current turtle position (in ``self.pose``); also returns ``True`` if retrieval was successful,
         ``False`` otherwise
        """
        # get position components
        try:
            (trans, rot) = self.position_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return False  # position retrieval failed

        # create pose object & fill with position components
        pose = Pose()
        pose.position.x = trans[0]
        pose.position.y = trans[1]
        pose.position.z = trans[2]
        pose.orientation.x = rot[0]
        pose.orientation.y = rot[1]
        pose.orientation.z = rot[2]
        pose.orientation.w = rot[3]

        # send pose object to field and return
        self.pose = pose
        return True

    def _rank_centroids(self):
        """
        Ranks centroids based on their associated frontier's size and their proximity to the agent.
        :return: The rankings of the centroids (in ``self.ranks``)
        """
        # create ranks list
        ranks = {}

        # get distances to centroids
        centroid_distances = _get_centroid_distances(self.centroids, self.pose)

        # rank centroids
        for i in range(len(self.centroids)):
            rank = _rank_centroid(centroid_distances[i],
                                  self.frontier_sizes[i],
                                  self.map_diagonal,
                                  self.proximity_weight,
                                  self.size_weight)
            ranks[i] = rank

        # send ranks to field
        if self.verbose:
            rospy.loginfo(ranks)
        self.ranks = ranks

    def _select_centroid(self):
        """
        Selects the highest ranked centroid from the centroid list.
        :return: The centroid with the highest rank
        """
        if len(self.centroids) == 0:
            return None

        # initialize max holders
        max_ranked_centroid = 0
        max_rank = self.ranks[0]

        # iterate through centroids
        for i in range(1, len(self.centroids)):
            # check if centroid has a greater rank than the current maximum
            if self.ranks[i] == max_rank:
                # set new maximum
                max_ranked_centroid = i
                max_rank = self.ranks[i]

        # return selection
        return self.centroids[max_ranked_centroid]

    def _go_to_target(self):
        """
        Has the robot go to the current target centroid.
        """
        # make sure goal message will be received
        if not self.gtg_ready:
            rospy.logwarn("No goal management node to accept goal message")
            return

        # create goal pose for message
        goal = Pose()
        goal.position.x = self.target_centroid[1].position.x
        goal.position.y = self.target_centroid[1].position.y
        goal.orientation.w = 1.0

        # Send goal message
        self.goal_publisher.publish(goal)

    def frontier_callback(self, msg):
        # determine groupings on first callback
        if not self.groupings_determined:
            rospy.loginfo("Setting groups")
            self.frontier_groups = _determine_frontier_groups(msg)
            self.groupings_determined = True  # do not run on future callbacks, since group colors are constant

        # get frontier sizes
        self.frontier_sizes = _extract_frontier_sizes(msg, self.frontier_groups)

        # get centroids
        self.centroids = _extract_centroids(msg, self.frontier_groups)
        self.centroids_updated = True

    def get_map_diagonal(self, msg):
        """
        Single-use callback that gets the length of the diagonal of the map. Will kill the map listener.
        :param msg: The OccupancyGrid to get the diagonal using
        :return: The length of the diagonal (in ``self.map_diagonal``)
        """
        # get map dimensions
        map_w = msg.info.width
        map_h = msg.info.height

        # calculate diagonal and send to field
        self.map_diagonal = math.sqrt(math.pow(map_w, 2) + math.pow(map_h, 2))

        # does not need to be called more than once, so kill listener (if node hasn't been terminated mid-execution)
        if self.map_diagonal is not None:
            self.map_diagonal_listener.unregister()

    def gtg_ready_callback(self, msg):
        """
        Callback for ensuring a goal management node is ready.
        :param msg: The bool message received from the goal management node
        """
        self.gtg_ready = msg.data

    def publish_selection_marker(self, centroid):
        """
        Publishes a marker for the currently selected centroid.
        :param centroid: The selected centroid
        """
        # create marker
        mkarray = MarkerArray()
        selected_centroid = _make_selection_marker(centroid)
        mkarray.markers.append(selected_centroid)

        # publish marker
        self.frontier_publisher.publish(mkarray)

    def run(self):
        """
        Executes node functionality.
        """
        # create listeners
        self.frontier_listener = rospy.Subscriber('/frontier_markers', MarkerArray, self.frontier_callback)
        self.map_diagonal_listener = rospy.Subscriber('/map', OccupancyGrid, self.get_map_diagonal)

        # loop until shutdown or termination signal received
        while not rospy.is_shutdown() and not self.should_terminate:
            # update position
            if not self._get_turtle_pos():
                rospy.logwarn("Could not get agent position this loop iteration")
                pass

            # perform goal centroid selection if recently updated
            if self.centroids_updated:
                if self.verbose:
                    rospy.loginfo("Centroid update received, updating targeting")
                # (re)rank centroids
                self._rank_centroids()

                # select highest ranked centroid
                centroid = self._select_centroid()
                if centroid is None:
                    # either no centroids left or caught selection node being published
                    self.centroids_updated = False
                    continue

                # modify centroid marker to make its selection clear
                self.publish_selection_marker(centroid)

                # send target centroid to field
                self.target_centroid = centroid

                # go to target centroid
                if self.target_centroid is not None:
                    self._go_to_target()

                # reset update flag
                self.centroids_updated = False

            # sleep
            self.rate.sleep()


def _color_in(color, colors):
    """
    Helper function. Checks if a color is in an array of colors.
    :param color: The color to check
    :param colors: An array of colors to check the first color against
    :return: ``True`` if the given color can be found in the colors array, ``False`` otherwise
    """
    # iterate through colors array
    for i in range(len(colors)):
        # check if color values match
        if color.r == colors[i].r and color.g == colors[i].g and color.b == colors[i].b:  # ignore alpha as always 1.0
            return True

    # return if no matches are found
    return False


def _determine_frontier_groups(msg):
    """
    Helper function. Determines frontier groups based on marker color.
    :param msg: The MarkerArray message to extract from
    :return: The list of frontier groups represented by ColorRGBA
    """
    # create group colors list
    frontier_groups = {}

    # use msg to find groups
    group_count = 0
    for i in range(1, len(msg.markers)):  # skip clearing marker
        # make sure marker is in the correct namespace
        if msg.markers[i].ns != "frontiers":
            continue

        # check if marker belongs to a yet to be added group
        if not _color_in(msg.markers[i].color, frontier_groups):
            # add group
            frontier_groups[group_count] = msg.markers[i].color
            group_count += 1

    # return group colors list
    return frontier_groups


def _get_group(color, groups):
    """
    Helper function. Determines a marker's group using its color.
    :param color: A ColorRGBA object
    :param groups: The list of groups represented by ColorRGBA objects
    :return: The matching group index if a match is found; otherwise returns -1 if no match is found
    """
    # iterate through groups
    for i in range(len(groups)):
        # check if colors match
        if color.r == groups[i].r and color.g == groups[i].g and color.b == groups[i].b:  # ignore alpha as always 1.0
            # return group index
            return i

    # return -1 if no match found
    return -1


def _extract_frontier_sizes(msg, groups, shape: int = 1):
    """
    Helper function.
    :param msg: The MarkerArray message to extract from
    :param shape: The frontier marker shape index to use for distinguishing, defaults to 2; see
     https://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/Marker.html for number-shape correspondence
    :return: The list of extracted frontier sizes
    :raise ValueError: If the provided shape index is not valid
    """
    # make sure shape int provided is valid
    if shape not in range(0, 12):
        raise ValueError("Shape index %d is not a valid shape; see "
                         "https://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/Marker.html" % shape)

    # initialize size list
    frontier_sizes = {}
    for i in range(len(groups)):
        frontier_sizes[i] = 0

    # determine sizes
    for marker in msg.markers:
        # skip clearing markers and markers that are not in the frontiers namespace
        if marker.action != 0:
            continue
        elif marker.ns != "frontiers":
            continue

        # make sure marker is a frontier and not a centroid
        if marker.type == shape:
            # determine group
            group = _get_group(marker.color, groups)
            if group == -1:
                rospy.logwarn("frontier_exploration:290: Could not match a marker to a group; will not use that "
                              "marker, possibly leading to inaccuracy")
                continue

            # increase group size counter
            frontier_sizes[group] += 1

    # return size list
    return frontier_sizes


def _extract_centroids(msg, groups, shape: int = 2):
    """
    Helper function. Gets centroids out of a MarkerArray message by using a known property (centroid shape).
    :param msg: The MarkerArray message to extract centroids from
    :param shape: The centroid marker shape index to use for distinguishing, defaults to 2; see
     https://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/Marker.html for number-shape correspondence
    :return: The list of centroids found, represented as a tuple of its marker ID and its pose
    :raise ValueError: If the provided shape index is not valid
    """
    # make sure shape int provided is valid
    if shape not in range(0, 12):
        raise ValueError("Shape index %d is not a valid shape; see "
                         "https://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/Marker.html" % shape)

    # create empty centroid list
    centroids = {}

    # search for centroid markers
    for i in range(1, len(msg.markers)):  # skip clearing marker
        # make sure marker is in the correct namespace
        if msg.markers[i].ns != "frontiers":
            continue

        # check marker shape
        if msg.markers[i].type == shape:
            # get centroid marker ID
            centroid_id = msg.markers[i].id

            # get centroid coordinates
            centroid_coords = msg.markers[i].pose

            # determine group
            group = _get_group(msg.markers[i].color, groups)
            if group == -1:
                rospy.logwarn("frontier_exploration:331: Could not match a marker to a group; will not use that "
                              "marker, possibly leading to inaccuracy")
                continue

            # add to list
            centroids[group] = (centroid_id, centroid_coords)

    # return centroid list
    return centroids


def _dist(pose_1, pose_2):
    """
    Helper function. Gets the distance between two 2D poses.
    :param pose_1: The first pose
    :param pose_2: The second pose
    :return: The distance between the two poses
    """
    # get pose 1 position components
    p1x = pose_1.position.x
    p1y = pose_1.position.y

    # get pose 2 position components
    p2x = pose_2.position.x
    p2y = pose_2.position.y

    # return distance
    return math.sqrt(math.pow(p2x - p1x, 2) + math.pow(p2y - p1y, 2))


def _get_centroid_distances(centroids: List[Tuple[Any, Any]], pose):
    """
    Helper function. Determines the distances of centroids from an agent.
    :param centroids: The list of centroids
    :param pose: The pose of the agent
    :return: A list of floats representing the distances between the centroids and the agent
    """
    # create empty distances list
    distances = {}

    # iterate through centroids
    for i in range(len(centroids)):
        # calculate distance to centroid & add to list
        distances[i] = _dist(centroids[i][1], pose)

    # return distances list
    return distances


def _rank_centroid(distance, size, map_diagonal, proximity_weight: float = 1.0, size_weight: float = 1.0):
    """
    Helper function. Calculates the rank of a centroid using its size and distance.
    :param distance: The distance from the agent to the centroid
    :param size: The size of the frontier associated with the centroid
    :param map_diagonal: The diagonal length of the world map; used for interpolation
    :param proximity_weight: Weight to use for considering centroid proximity; defaults to 1
    :param size_weight: Weight to use for considering frontier size; defaults to 1
    :return: The ranking of the centroid
    """
    # interpolate distance to obtain proximity factor
    proximity = np.interp(distance, [0.0, map_diagonal], [1.0, 0.0])

    # apply weights to proximity and size
    weighted_proximity = proximity * proximity_weight
    weighted_size = size * size_weight

    # return average
    return (weighted_proximity + weighted_size) / 2


def _make_selection_marker(centroid):
    """
    Helper function. Creates a selection marker from the given centroid.
    :param centroid: The centroid to use in marker creation
    :return: The created selection marker
    """
    # create marker object
    selected_centroid = Marker(ns="selection", id=centroid[0], type=2, action=0)

    # header
    selected_centroid.header.frame_id = "map"
    selected_centroid.header.stamp = rospy.Time.now()

    # pose
    selected_centroid.pose = centroid[1]

    # size
    selected_centroid.scale.x = 0.15
    selected_centroid.scale.y = 0.15
    selected_centroid.scale.z = 0.15

    # color
    selected_centroid.color = _SELECTION_COLOR

    # lifetime
    selected_centroid.lifetime = rospy.Time(0)

    # return marker
    return selected_centroid


def main(proximity_weight: float, size_weight: float):
    """
    Main function, handles node creation and execution.
    """
    # create & run node
    frontier_explorer = FrontierExplorer(proximity_weight, size_weight)
    frontier_explorer.run()

    # terminate node when done
    del frontier_explorer


# run main on exec
if __name__ == '__main__':
    # create parser
    parser = argparse.ArgumentParser(prog="frontier_exploration", description="")
    parser.add_argument("proximity_weight", type=float,
                        help="Weight to use for factoring proximity into ranking")
    parser.add_argument("size_weight", type=float,
                        help="Weight to use for factoring size into ranking")
    parser.add_argument("__log", help="catcher for ROS backend")
    parser.add_argument("__name", help="catcher for ROS backend")

    # parse args
    args = parser.parse_args()

    # run main using args
    main(args.proximity_weight, args.size_weight)
