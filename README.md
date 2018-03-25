## 3D Motion Planning
![alt text][image4]

---


The goals/steps of this project are following:

1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.

### Setup
* Download the [simulator](https://github.com/udacity/FCND-Simulator-Releases/releases).
* Setup [Python environment](https://github.com/udacity/FCND-Term1-Starter-Kit)
* Execute the fly by `python motion_planning.py`
* Refer to the [UdaciDrone API](https://udacity.github.io/udacidrone/) if needed


[//]: # (Image References)

[image1]: ./misc/grid.png "Grid"
[image2]: ./misc/paths_diagonal.png "Path with Diagonal Motion"
[image3]: ./misc/paths_prune.png "Pruned Path"
[image4]: ./misc/flying_drone.gif "Flying Drone"

---

### Starter Code

I started with following files:

* `motion_planning_starter.py`
* `planning_utils_starter.py`
* `backyard_flyer_solution.py`

`motion_planning_starter.py` is a modified version of `backyard_flyer_solution.py` that leverages some extra functions in `planning_utils_starter.py`. The scripts contain a basic planning implementation that makes the quad to fly a jerky path of waypoints to the norhteast for 10 m then land.

A new state PLANNING was added to `motion_planning_starter.py`. Instead of taking off immediately after the quad's armed, the quad switchs to the PLANNING state, and the planner performs path planning in the method `plan_path()` with following steps: 

1. Reads in the file `colliders.csv` containing obstacle data
2. Extracts a grid representation of a 2D configuration space with the method `create_grid()` in `planning-utils_starter.py`
3. Defines the start and goal points
4. Performs A* search to find a path from start to goal with the method `a_star()` in `planning-utils_starter.py`
5. Converts the planned path into waypoints, and send the waypoints to simulator

Here's the grid configuration space created from `colliders.csv`, green color areas are the safety distance from the actual obstacles:

![alt text][image1]

### Planning Algorithm

#### 1. Set global home position
The starter code assumed the home position is where the drone first initializes, but in reality the drone needs to be able to start planning from anywhere. 

The first line of the csv file describes the global home position as `lat0 37.792480, lon0 -122.397450`. I extracted the values of lat0 and lon0 from the csv file, and set it to home position with `self.set_home_position()`. 

I did this in line 129 of `planning_utils.py` and line 157 to 165 of `motion_planning.py`.

#### 2. Set current local position
The starter code assumed the drone takes off from map center, but the drone need to be able to takeoff from anywhere. 

I retreived the drone's current position in geodetic coordinates from `self.global_position`, and the global home position set from last step from `self.global_home`, then used the utility function `global_to_local()` to convert the current global position to local position.

I did this in line 134 of `motion_planning.py`.

#### 3. Set grid start position from local position
The starter code hardcoded the map center as the start point for planning. To further enhance the flexibility to the start location, I changed this to be the current local position in line 144 to 146 of `motion_planning.py`.

#### 4. Set grid goal position from geodetic coords
The starter code hardcoded the goal position as some location 10 m north and 10 m east of map center. To add flexibility to the desired goal location, I modified the code in line 151 to 153 of `motion_planning.py` to accept arbitrary goal postion on the grid given any geodetic coordinates. By defaults I set the coordinates to (longitude, latitude, altitude). 

To assign the goal position, use command line arguments `goal_lon` for longitude, `goal_lat` for latitude, and `goal_alt` for altitude. For example:

```
python motion_planning.py --goal_lat 37.792945 --goal_lon -122.397513 --goal_alt 26
``` 

#### 5. Modify A* to include diagonal motion
I updated the A* implementation to include diagnoal motions on the grid that have a cost of sqrt(2). With diagnoal motions included, the jerky movement disappeared and the trajectories planned for the same goal changed. I did this in lines 58 to 61, 91 to 98 of `planning_utils.py`.

Here's a comparison between paths with and without diagonal motion:

![alt text][image2]

#### 6. Cull waypoints 
To prune the path of unnecessary waypoints, I implemented collinearity test in lines 167 to 188 of `planning_utils.py` and applied it to the path obtained from A* search.

Here's a comparison between paths before and after removing the unnecessary waypoints:

![alt text][image3]

  
### Future Works
* Move to graph search space
* Fly more complex trajectories - rather than simply setting a target altitude, send altitude with each waypoint and set the goal location on top of a building.
* Adjust the size of the deadbands around waypoints, and make deadbands a function of velocity. 
* Add heading commands to waypoints, set the heading points to next waypoint and make crazy paths like double helix to fly.

