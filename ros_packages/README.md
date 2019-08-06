This folder contains the packages written for the demos.

The rqt_* packages are the rqt plugins. You just need them in your workspace (compiled with catkin_make) and they will appear under Custom Plugins in the rqt interface. You can run rqt --force-discover if the plugins doesn't appears.
rqt_example_cpp is just a template used to create new plugins
rqt_turtle_teleop is a GUI to control a turtle simulation using a joystick
rqt_joy2dynamixel is a GUI to control a dynamixel using a joystick

learning_joy is the node used to control the turtle simulation using a joystick. Run it using the launch file with :
  roslaunch learning_joy turtle_joy.launch
  
joy_control_dynamixel is the node used to controlthe dynamixel using a joystick. Run it using the launch file with :
  roslaunch joy_control_dynamixel joy_control_dynamixel.launch
  
Note : the dynamixel control needs to configure several .config and .yaml file to work. 
Use dynamixels : https://biorob2.epfl.ch/wiki/scratch:ros_setup#random_notes
