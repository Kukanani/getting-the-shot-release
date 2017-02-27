Miscellaneous code and files for ARSO 2017 submitted paper. Includes both viewpoint selection and a map "vectorizer" node, which uses SciPy.

I believe that this code is incomplete without also using Andrew Sharps support_utilities package, which unfortunately resides in a private repository for now. Hopefully the map vectorizer and other code snippets can prove useful anyway.

on the turtlebot:

roslaunch turtlebot_multi minimal.launch
roslaunch turtlebot_multi amcl_demo.launch