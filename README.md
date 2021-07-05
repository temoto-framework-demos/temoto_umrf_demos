# temoto_umrf_demos
This is a setup package for replicating the ROS + husky simulation-based demonstrations presented the journal paper *"Unified Meaning Representation Format (UMRF) - A Task Description and Execution Formalism for HRI"*

# Installation
```bash
cd <your-catkin-workspace>/src
git clone --recursive https://github.com/temoto-telerobotics-demos/temoto_umrf_demos
cd ..
catkin build
source devel/setup.bash
```
# Install dependencies
```bash
cd ~/<your-catkin-workspace>
rosdep install --from-paths src --ignore-src -r -y

# For ROS Noetic, point the python interpreter to your default python3 with this package:
sudo apt install python-is-python3
```

# Test if the robot simulation is properly set up

Open a terminal and launch the sim_driver:
```bash 
roslaunch robot_temoto_config husky_sim_driver.launch
```
You should see the Gazebo sim opening up. If you are launching it for the first time, then
it might take some time for Gazebo to download the world models. Once the world is loaded, you 
should see the robot. There might be couple of warnings in the terminal about `No p gain specified for pid` 
or `Failed to meet update rate!` but that's nothing to worry about.


Next, open a second terminal and launch the controller:
```bash
roslaunch robot_temoto_config husky_sim_controller.launch 
```
Again, there might initially be couple of warnings about `Timed out waiting for transform from base_link to map` 
and `Parameter max_trans_vel is deprecated`. If any warning is periodically reported, then something is probably not right.


Finally, open a third terminal and launch the visualization:
```bash
roslaunch robot_temoto_config husky_sim_viz.launch
```
RViz should pop up and you should see the map, robot and lidar data. Use the `2D Nav Goal` tool to move the robot 
somewhere within the map. If the robot performs sucessfully, then the simulation is set up properly.