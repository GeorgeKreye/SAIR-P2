# SAIR Project 2

## Part 1
Main functionality for this part of the project is contained within ``go_to_goal.py``, which has the turtle agent attempt to
move towards a goal inputted by the user on the command line. While the node associated with that functionality is
active, it will also print out the current location of the turtle agent at 1-second intervals.
### How to use
Open a Linux command line terminal with ROS commands and do the following:
1. Optional: run `roscore` in its own terminal.
2. Run the following commands, each in separate terminals:<br />`roslaunch turtlebot3_gazebo  turtlebot3_stage_4.launch`
<br />`roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping`<br />`roslaunch sair-p2-GeorgeKreye
turtlebot3_navigation.launch`<br />These will start world simulation, SLAM, and agent navigation, respectively.
3. Once all of the above commands have finished initializing, in one final terminal, run:<br />`roslaunch 
sair-p2-GeorgeKreye part1_demo.launch goal_x:=<x> goal_y:=<y>`<br />Replace `<x>` and `<y>` with the desired goal
coordinates. 
4. To move to a new position, shut down the current goal node (Ctrl-C) and run the command in step 3 with new goal
coordinates.
## Part 2
Main functionality for this part of the project is contained within ``find_frontiers.py``, which listens for map
updates. Upon receiving one, it takes the occupancy grid associated with the map, finds frontiers within that grid,
segments the found frontiers according to a user-inputted number of clusters, and finds the centroids of each segmented
frontier cluster. It will then publish this information for display with RViz.
### How to use
Open a Linux command line terminal with ROS commands and do the following:
1. Option: run `roscore` in its own terminal.
2. Run the following commands, each in separate terminals:<br />`roslaunch turtlebot3_gazebo  turtlebot3_stage_4.launch`
<br />`roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping`<br />`roslaunch sair-p2-GeorgeKreye
turtlebot3_navigation.launch`<br />These will start world simulation, SLAM, and agent navigation, respectively.
3. Once all of the above commands finished initializing , in one final terminal, run: <br /> `roslaunch
sair-p2-GeorgeKreye part2_demo.launch`<br />Optionally, the parameter `num_clusters:=<k>` can be set, replacing `<k>`
with the desired number of clusters to use in frontier segmentation; otherwise, the launch file will default to 5
clusters.
4. Go to the RViz window opened by running ``turtlebot3_slam.launch`` in step 2 and add the following elements:<br />
   1. A Map subscribed to the topic `/frontier_map`
   2. A MarkerArray subscribed to the topic `/frontier_markers`
5. Assert that the frontier markers from the MarkerArray line up with the frontier map.
6. Moving the robot manually in RViz or with `go_to_goal.py` (see Part 1) should cause frontiers to update.
## Part 3
Main functionality is spread between `go_to_goal.py`,`find_frontiers.py`, and `frontier_exploration.py`. Main functionality
added in this part is contained within the latter, with it handling the ranking and selection of centroids to have the
turtle move towards. `progress_checker.py` handles tracking how much the robot has explored the map, and will terminate
the former three nodes and `turtlebot3_navigation` once it deems a high enough percentage of the map has been revealed.
### How to use
For Part 3, all functionality, including backend, can be run with a single launch file from a Linux command line:<br />
`roslaunch sair-p2-GeorgeKreye sair-p2-georgekreye.launch`<br />Parameters are `num_clusters:=<k>`, which as noted in
Part 2 is the number of clusters *k* to use in the segmentation of frontiers; `proximity_weight:=<p>`, which is the 
weight *p* to be applied to proximity in calculating rankings of centroids; `size_weight:=<s>`, which is the weight
*s* to be applied to associated frontier size in calculating rankings of centroids; and `termination_threshold:=<t>`,
which will trip the node to end execution once the ratio of explored squares to squares reaches this percent. Once ran,
the turtle should automatically explore the map with little to no intervention needed, provided the aforementioned
settings are optimal enough. Their default values are *k* = 5, *p* = 1.0, *s* = 1.0, and *t* = 0.95. Frontiers can be
visualized in the same way as part 2, with the ``frontier_explorer`` node highlighting the selected node in white.
### Ranking of centroids
Centroids are ranked using a combination of proximity and associated frontier size, with a weighted average being
employed to add the two together into a final ranking.
#### Proximity
Proximity is calculated by interpolating the distance between the centroid and the turtle onto a range of [1, 0], with
closer centroids being interpolated closer to 1. This interpolation is done using the diagonal of the entire map, as it
is impossible for the robot to exceed this distance due to being boxed in. Proximity weight is then applied after the 
interpolation.
#### Size
Size is simply a count of how many frontier squares are associated with a particular centroid; therefore, with default
parameters, it is given more priority over distance. This was chosen instead of normalizing using the total number of
frontier squares in order to create that priority by default, since larger frontiers generally will reduce map entropy
much quicker when explored. Size weights are applied after this counting is complete.
### Termination
The turtle will terminate its exploration if the number of unexplored squares drops to a very low amount, as to prevent
it from getting itself stuck trying to get to areas it cannot explore. This is done by a percentage representing the
amount of the map that has been explored being compared against a cutoff percentage; if the cutoff is met or exceeded,
the robot will consider its task complete and shut down its main nodes.<br />This behavior is handled by the
`progress_checker` node, which retrieves the map and calculates this entropy percentage whenever the former is updated.

