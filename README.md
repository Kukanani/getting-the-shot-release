Miscellaneous code and files for ARSO 2017 submitted paper. Includes both viewpoint selection and a map "vectorizer" node, which uses SciPy.

This code includes a "map vectorizer", which takes a ROS map (which is a dense array of occupancy information) and
converts it into a vector-specified occupancy map.

on the turtlebot:

roslaunch turtlebot_multi minimal.launch
roslaunch turtlebot_multi amcl_demo.launch