## Mission planer pkg
This package purpose is to host all mission handling and creation for the Dubins PathPlanner of the ASVs.

A description of what each file does is provided on inside each file as a comment.

### The missions directory
The missions directory is where all .csv based missions are stored. Each subdirectory stores a single `mission.csv` file. And the subdirectory's name is the mission name used across the package as an identifier when launching it. The structure of the .csv file at its most basic is `lat`, `long` and `head` (in E 0rad, N pi/2 rad). The other parameters, need to be on the file's header definition, but there is no need to specify them on the actual data. An example mission with 2 waypoints is shown below.

```
lat,lon,depth,head,dive_mode,wp_mode
-33.72276582978362,150.6739871096942,0,0
```
- The `nav_goal_to_waypoint.py` file is a sample implementation of the actionClient. It shows how to launch a mission from a csv file taken from a rosparam (see [action_client.launch](launch/action_client.launch)) and also how to send 1-waypoint navigation goals to the PathPlanner for realtime single-waypoint following.

- The files on the `converters` folder are scripts used to convert from (x,y)-defined trajectories, to a GPS trajectory (and viceversa)