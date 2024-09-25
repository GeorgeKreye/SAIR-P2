#!/usr/bin/env python3

"""
ROS executable that is designed to find frontiers in an occupancy grid.

If running on its own, this should be run on the command line using ``rosrun sair-p2-GeorgeKreye find_frontiers.py``
after compiling the ROS package using catkin.
"""

# imports
import random
import rospy
import argparse
from sklearn.cluster import AgglomerativeClustering
import numpy as np
from typing import Tuple, Union, List
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA
from std_msgs.msg import Bool as BoolMSG


class FrontierFinder:
    """
    ROS node for frontier detection and segmentation.
    """

    def __init__(self, num_clusters: int, expansion_radius: int = 2, verbose: bool = False):
        """
        ROS node for frontier detection and segmentation.
        :param verbose: Whether the node should log additional information; defaults to ``false``
        :param num_clusters: The number of clusters to generate in segmentation of frontiers
        """

        # save verbose setting
        self.verbose = verbose

        # save clustering setting
        self.k = num_clusters

        # save expansion amount setting
        self.expansion_radius = expansion_radius

        # generate marker colors
        self.colors = {}
        for i in range(self.k):
            color = ColorRGBA()
            color.r = random.random()
            color.g = random.random()
            color.b = random.random()
            color.a = 1
            self.colors[i] = color

        # create node
        rospy.init_node('frontier_finder')

        # create termination listener
        self.term_listener = rospy.Subscriber('/termination', BoolMSG, self.__del__)

        # create subscriber holder
        self.listener: rospy.Subscriber | None = None

        # create map holders
        self.og_map = None
        self.expanded_map = None
        self.frontier_map = None

        # create marker holders
        self.frontier_markers = None
        self.centroid_markers = None

        # create frontier marker publisher
        self.marker_publisher = rospy.Publisher('/frontier_markers', MarkerArray, queue_size=10)
        self.frontier_publisher = rospy.Publisher('/frontiers_map', OccupancyGrid, queue_size=10)

        # log node creation
        rospy.loginfo("Frontier finder node initialized")

    def __del__(self):
        rospy.loginfo("Shutting down node")

        # unregister subscriber if created
        if self.listener is not None:
            self.listener.unregister()
            del self.listener

        # unregister publishers
        self.marker_publisher.unregister()
        del self.marker_publisher
        self.frontier_publisher.unregister()
        del self.frontier_publisher

        # terminate node
        rospy.signal_shutdown("Node terminated by code")

    def _grow_obstacles(self, threshold: int = 100):
        """
        Expands obstacles in an occupancy grid as to avoid locations an agent cannot reach.
        :param threshold: The minimum obstacle confidence to use when deciding if there is an obstacle to be expanded;
        must be between 50 and 100 (no confidence threshold below 50% confidence allowed)
        :return: The updated occupancy grid (in ``self.expanded_map``)
        :raise ValueError: if an invalid threshold is given
        """
        # make sure passed threshold is valid
        if not (50 <= threshold <= 100):
            raise ValueError("Threshold %d not in range [50,100]" % threshold)

        # create map copy
        expanded_map = self.og_map[:]

        # expand obstacles in map copy
        for x in range(0, len(self.og_map)):
            for y in range(0, len(self.og_map[0])):
                if self.og_map[x][y] >= threshold:
                    # iterate through adjacent cells
                    for x1 in range(x - self.expansion_radius, x + self.expansion_radius):
                        for y1 in range(y - self.expansion_radius, y + self.expansion_radius):
                            if x1 == x and y1 == y:
                                # skip center
                                continue

                            # expand obstacle to adjacent cell if it exists and isn't already expanded
                            if _not_expanded(expanded_map, x1, y1):
                                expanded_map[x1][y1] = 100

        # pass to field
        self.expanded_map = expanded_map

    def _find_frontiers(self):
        """
        Finds frontiers using the given map.
        :return: The occupancy grid showing frontiers (in ``self.frontier_map``)
        """
        # create empty frontier map
        frontier_map = []

        # create frontier map by finding unoccupied cells
        for x in range(0, len(self.expanded_map)):
            col = []
            for y in range(0, len(self.expanded_map[0])):
                if _is_frontier(self.expanded_map, x, y, verbose=self.verbose):
                    col.append(100)
                else:
                    col.append(0)
            frontier_map.append(col)

        # pass frontier map to field
        self.frontier_map = frontier_map

    def _segment_frontiers(self):
        """
        Segments frontiers using agglomerative clustering.
        :return: A list of marker dictionaries containing segmented frontier information (in ``self.frontier_markers``)
        """
        # convert frontier map to ndarray points list
        fmap = _create_points_list(self.frontier_map)

        # segment frontier map
        clusters = AgglomerativeClustering(n_clusters=self.k)
        clusters.fit(fmap)

        # create segmented frontier dictionary as output
        self.frontier_markers = []
        for point in range(len(clusters.labels_)):
            point_entry = {'pos': fmap[point], 'cluster': clusters.labels_[point]}
            self.frontier_markers.append(point_entry)

    def _find_centroids(self):
        """
        Finds frontier centroids using the frontier map.
        :return: A list containing the centroids of the frontiers (in ``self.centroid_markers``)
        """
        # create centroids list
        centroids = []

        # iterate through clusters
        for i in range(self.k):
            x_avg = 0
            y_avg = 0
            count = 0

            # sum x positions
            for point in self.frontier_markers:
                if point['cluster'] == i:
                    x_avg += point['pos'][0]
                    y_avg += point['pos'][1]
                    count += 1

            # divide by count to get centroid position
            x_avg = x_avg / count
            y_avg = y_avg / count

            # create centroid marker & append to list
            centroid = {'pos': (x_avg, y_avg), 'cluster': i}
            centroids.append(centroid)

        # pass centroid list to field
        self.centroid_markers = centroids

    def _publish_frontiers(self, map_resolution, map_origin):
        """
        Publishes the frontier map to the topic ``/frontiers_map``.
        :param map_resolution: The resolution (m/cell) to use for the map
        :param map_origin: The origin pose for the map
        """
        # make frontier map 1d for publishing
        raw_map, map_dimensions = _1d_map(self.frontier_map)

        # create OccupancyGrid object
        grid = OccupancyGrid()
        grid.header.stamp = rospy.Time.now()
        grid.data = raw_map
        grid.info.width = map_dimensions[0]
        grid.info.height = map_dimensions[1]
        grid.info.resolution = map_resolution
        grid.info.origin = map_origin

        # publish
        self.frontier_publisher.publish(grid)

    def _publish_markers(self, map_resolution, map_origin):
        """
        Publishes segmented frontier markers and their centroids to the topic ``/frontier_markers``.
        :param map_resolution: The map resolution (m/cell) to use to determine non-centroid marker size
         and for coordinate translation
        :param map_origin: The map origin to use for coordinate translation
        """
        # create marker array
        mkarray = MarkerArray()

        # fill marker array
        temp = [Marker(ns="frontiers", id=-1, action=3)]  # clear old markers
        used_ids = []
        for frontier_marker in self.frontier_markers:
            # generate marker (32-bit ID for ensuring it is generalizable)
            marker = Marker(ns="frontiers",
                            id=_generate_unique_id(used_ids, -2147483648, 2147483648),
                            type=1,
                            action=0)
            _shared_gen(frontier_marker, marker, map_resolution, map_origin)
            marker.scale.x = map_resolution
            marker.scale.y = map_resolution
            marker.scale.z = 1
            marker.color = self.colors[frontier_marker['cluster']]

            # add to temp holding space
            temp.append(marker)

        # add centroids
        temp.extend(self._publish_centroids(used_ids, map_resolution, map_origin))

        # set marker array contents to temp
        mkarray.markers = temp

        # publish markers
        self.marker_publisher.publish(mkarray)

    def _publish_centroids(self, used_ids, map_resolution, map_origin):
        """
        Creates markers for publishing segmented frontier centroids.
        :param used_ids: List of used unique marker IDs for unique ID generation.
        :param map_resolution: The resolution of the frontier map
        :param map_origin: The origin of the frontier map
        :return: The list of centroid marker objects generated
        """
        # create empty marker list
        centroid_markers = []

        # fill marker list
        for centroid in self.centroid_markers:
            # create centroid marker
            centroid_marker = Marker(ns="frontiers",
                                     id=_generate_unique_id(used_ids, -2147483648, 2147483648),
                                     type=2,
                                     action=0)
            _shared_gen(centroid, centroid_marker, map_resolution, map_origin)
            centroid_marker.scale.x = 0.15
            centroid_marker.scale.y = 0.15
            centroid_marker.scale.z = 0.15
            centroid_marker.color = self.colors[centroid['cluster']]

            # append to marker list
            centroid_markers.append(centroid_marker)

        # return marker list
        return centroid_markers

    def callback(self, msg):
        """
        Performs node functionality when the map subscriber receives an update.
        :param msg: The subscriber-received message associated with the update
        """
        # extract map
        raw_map = msg.data
        map_dimensions = (msg.info.width, msg.info.height)
        map_resolution = msg.info.resolution
        map_origin = msg.info.origin
        try:
            self.og_map = _2d_map(raw_map, map_dimensions)  # convert map to 2D array
        except ValueError as e:
            # log error and return
            rospy.logerr(e)
            return

        # grow obstacles
        self._grow_obstacles()

        # find frontiers
        self._find_frontiers()

        # segment frontiers
        self._segment_frontiers()

        # find centroids
        self._find_centroids()

        # publish info
        self._publish_frontiers(map_resolution, map_origin)
        self._publish_markers(map_resolution, map_origin)

    def run(self):
        """
        Executes node functionality.
        """
        # create map subscriber
        self.listener = rospy.Subscriber('/map', OccupancyGrid, self.callback)
        rospy.loginfo("Starting listener for /map topic")

        # hold until node is destroyed
        rospy.spin()


def _1d_map(map_2d):
    """
    Helper function. Converts a 2D occupancy grid into 1 dimension alongside its dimensions for publishing.
    :param map_2d: The 2D occupancy grid to process
    :return: The 1D occupancy grid created, and its dimensions
    """
    # create empty 1d grid
    map_1d = []

    # create dimension tuple
    map_dimensions = (len(map_2d), len(map_2d[0]))

    # fill 1d grid
    for x in range(map_dimensions[0]):
        map_1d.extend(map_2d[x])

    # return 1d grid and dimensions
    return map_1d, map_dimensions


def _2d_map(map_1d, dimensions: Tuple[int, int]):
    """
    Helper function. Converts a raw 1D occupancy grid into a 2D array for easier use.
    :param map_1d: The 1D occupancy grid to process
    :param dimensions: The dimensions (width x height) for the map
    :return: The 2D array created
    :raises ValueError: if the given dimensions do not match the given raw map
    """
    # ensure dimensions match raw map data length
    if dimensions[0] * dimensions[1] != len(map_1d):
        raise ValueError("Map dimensions mismatch with actual data")

    # create empty 2D map
    map_2d = []

    # iterate to construct 2D map
    i = 0
    for x in range(dimensions[0]):
        col = []
        for y in range(dimensions[1]):
            # add grid value
            col.append(map_1d[i])

            # iterate counter
            i += 1

        # add to map
        map_2d.append(col)

    # return 2D map as numpy array
    return map_2d


def _not_expanded(og_map, x: int, y: int) -> bool:
    """
    Helper function. Checks whether a given occupancy grid cell has been expanded (set to 100).
    :param og_map: The occupancy grid map to use for checking
    :param x: The X coordinate on the map to check
    :param y: The Y coordinate on the map to check
    :return: ``True`` if the grid cell is valid & not expanded, ``False`` otherwise
    """
    # check for out of bounds
    if 0 > x or x >= len(og_map) or 0 > y or y >= len(og_map[0]):
        return False

    # return whether grid cell is expanded (set to 100)
    return og_map[x][y] < 100


def _is_frontier(og_map, x: int, y: int, obs_threshold: int = 100, verbose: bool = False) -> bool:
    """
    Helper function. Checks whether a given occupancy grid cell represents a frontier (i.e. is an unexplored cell that
    borders at least one known cell)
    :param og_map: The occupancy grid map to use for checking
    :param x: The X coordinate of the map to check
    :param y: The Y coordinate of the map to check
    :param obs_threshold: The threshold for considering a grid cell an obstacle (between 50-100)
    :param verbose: Whether to log additional information
    :return: ``True`` if the grid cell is valid & a frontier, ``False`` otherwise
    :raise ValueError: If the given obstacle threshold is invalid
    """
    # make sure obstacle threshold is valid
    if not (50 <= obs_threshold <= 100):
        raise ValueError("Obstacle threshold %d not in range [50, 100]" % obs_threshold)

    # make sure this cell is valid & unexplored
    if x >= len(og_map) or y >= len(og_map[0]) or x < 0 or y < 0:
        return False
    elif og_map[x][y] != -1:
        return False

    # check if any adjacent cell is known
    for x1 in range(x - 1, x + 1):
        for y1 in range(y - 1, y + 1):
            if x1 == x and y1 == y:
                # skip center
                continue

            # check if this adjacent cell is known & not an obstacle; if yes, this is a frontier
            if obs_threshold > og_map[x1][y1] > -1:
                if verbose:
                    rospy.loginfo("Found frontier cell @ (%d, %d)" % (x, y))
                return True

    # return false if all adjacent cels aren't known
    return False


def _create_points_list(frontier_grid):
    """
    Helper function. Converts a frontier grid into a list of cells considered frontiers.
    :param frontier_grid: The grid to process
    :return: The list of frontier cell coordinates as a ``ndarray``
    """
    # create empty list
    points_list = []

    # fill list with coordinates
    for x in range(len(frontier_grid)):
        for y in range(len(frontier_grid[0])):
            if frontier_grid[x][y] == 100:
                points_list.append((x, y))

    # return list formatted as a ndarray
    return np.array(points_list)


def _generate_unique_id(used_ids: List, id_min: int = 0, id_max: Union[int, None] = None):
    """
    Helper function. Generates a unique ID for usage by a marker, using the range ``[id_min,id_max)``. Defaults
    to ``[0,1)``.
    :param used_ids: List used to store already-generated IDs.
    :param id_min: The minium ID value, defaults to 0. Will be used as id_max instead if ``id_max`` is not set.
    :param id_max: The maximum ID value.
    :return: ``None`` if all IDs have been used, otherwise returns a unique ``int`` ID.
    :raise ValueError: if ``id_max`` is not greater than ``id_min``
    """
    # handle case where id_max is not set
    if id_max is None:
        if id_min > 0:
            id_max = id_min
            id_min = 0
        elif id_min < 0:
            id_max = 0
        else:
            # case where id_min is also not set or set to 0
            id_max = 1
            id_min = 0

    # make sure id_max is greater than id_min
    if id_max <= id_min:
        raise ValueError("id_max must be greater than id_min")

    # check if all IDs have been used
    if len(used_ids) < id_max:
        # generate unique ID
        uid: Union[int, None] = None
        while uid is None or uid in used_ids:
            uid = int(random.random() * (id_max - id_min) + id_min)
        if uid is None:
            # should be impossible to get here
            raise RuntimeError("Impossible state; error in loop at find_frontiers.py:463 likely")

        # add new ID to used list and return
        used_ids.append(uid)
        return uid
    else:
        # all IDs used
        return None


def _shared_gen(obj, marker, map_resolution, map_origin):
    """
    Helper function. Contains shared generation code for markers.
    :param obj: The object being used to create a marker
    :param marker: The marker being created
    :param map_resolution: The map resolution (m/cell) to use for coordinate translation into world space
    :param map_origin: The origin of the map to use for coordinate translation into world space
    """
    # header
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()

    # positional info
    world_pos = _to_world(obj['pos'], map_resolution, map_origin)
    marker.pose.position.x = world_pos[0]
    marker.pose.position.y = world_pos[1]
    marker.pose.orientation.w = 1.0

    # lifetime
    marker.lifetime = rospy.Time(0)


def _to_world(local, map_resolution, map_origin) -> Tuple[float, float]:
    """
    Helper function. Converts local coordinates on a map to world coordinates.
    :param local: The local coordinates to convert
    :param map_resolution: The resolution (m/cell) of the map the local coordinates are contained in
    :param map_origin: The origin of the map the local coordinates are contained in
    :return: The world coordinates corresponding to the passed-in local coordinates
    """
    # create world coordinate holders
    world_x: float = local[0]
    world_y: float = local[1]

    # apply translation
    world_x = world_x * map_resolution + map_origin.position.x + map_resolution / 2
    world_y = world_y * map_resolution + map_origin.position.y + map_resolution / 2

    # correct for world space being in rows instead of columns
    temp = world_y
    world_y = world_x
    world_x = temp

    # return
    return world_x, world_y


def main(num_clusters: int = 5, expansion_radius: int = 2, verbose: bool = False):
    """
    Main function. Handles node creation and execution.
    :param num_clusters: Number of clusters used by the node for frontier segmentation
    :param expansion_radius: The radius used by the node for expanding obstacles
    :param verbose: Whether to have the node log additional information
    """
    # create and run node
    finder = FrontierFinder(num_clusters, expansion_radius, verbose)
    finder.run()


# execute on run
if __name__ == '__main__':
    # parse arguments
    parser = argparse.ArgumentParser(prog="find_frontiers")
    parser.add_argument("num_clusters",
                        type=int, default=5,
                        help="The number of clusters to use in frontier segmentation")
    parser.add_argument("expansion_radius", type=int, default=2,
                        help="The radius in which to expand obstacles")
    parser.add_argument("__log", default=None, help="catcher for ROS backend")
    parser.add_argument("__name", default=None, help="catcher for ROS backend")
    parser.add_argument("-v", "--verbose",
                        action='store_true',
                        help="Whether to log additional information")
    args = parser.parse_args()

    # run main function
    main(args.num_clusters, args.expansion_radius, args.verbose)
