# Shakeit sim  
This package simulates a vibration feeder in CoppeliaSim and provides necessary assets (scenes and models).

## Sim Node
Upon starting the node, CoppeliaSim is started in a separate thread and the `assets/vibrating_feeder.ttt` scene is loaded.  

### Services:
* `/sim/add, {} -> {success: bool}`:  
adds random amount (1 ≤ n ≤ `new_objects_limit`) of new parts to the shaker
* `/sim/purge, {} -> {success: bool}`:  
removes all parts from the shaker
* `/sim/pick, {pose: geometry_msgs/Pose} -> {success: bool}`:  
removes an object under specified coordinates (in the world frame) from the shaker; 
the object has to be within `picking_threshold` from the specified coordinates
* `/sim/flip, {repetitions: int, speed: int} -> {success: bool}`:  
flip objects `repetitions` times, with `speed`-based intensity. 
* `/sim/forward, {repetitions: int, speed: int} -> {success: bool}`:  
move objects forward `repetitions` times, with `speed`-based intensity. 
* `/sim/backward, {repetitions: int, speed: int} -> {success: bool}`:  
move objects backward `repetitions` times, with `speed`-based intensity. 

| ![flip_behavior](docs/sim_behavior_flip.png)| ![flip_behavior](docs/sim_behavior_forward.png) | ![flip_behavior](docs/sim_behavior_backward.png) |
|---|---|---|
### Publishers:
* `/sim/camera/camera_info, {msg: sensor_msgs/CameraInfo}`: provides information about the camera sensor; is published together with `/sim/camera/image_color`
* `/sim/camera/image_color, {msg: sensor_msgs/Image}`, an image of a shaker from God's eye view

## CoppeliaSim Pro installation
1. Install deps

```
sudo apt install build-essential liblua5.3-0 qtcreator qt5-default 
```

1. [Download](http://www.coppeliarobotics.com/downloads) and extract the Pro edition  
2. In the installation folder run CoppeliaSim to instatiate lincesing files
```
./coppeliaSim.sh
```
3. In the installation directory, open file system/usrset.txt and adjust the values of floatingLicenseEnabled and floatingLicenseServer to
```
floatingLicenseEnabled = true
floatingLicenseServer = "YOUR LICENS SERVER"
```
4. Clone [PyRep](https://github.com/stepjam/PyRep)
```
git clone https://github.com/stepjam/PyRep.git
```
5. Add the following to your ~/.bashrc file
```
export COPPELIASIM_ROOT=EDIT/ME/PATH/TO/COPPELIASIM/INSTALL/DIR
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$COPPELIASIM_ROOT
export QT_QPA_PLATFORM_PLUGIN_PATH=$COPPELIASIM_ROOT
```
*  note the 'EDIT ME' in the first line
*  remember to source your bashrc (source ~/.bashrc) or zshrc (source ~/.zshrc) after this
6. Finally install the python library
```
cd PyRep
pip3 install -r requirements.txt
python3 setup.py install --user
```

## CoppeliaSim plugin for ROS2
1. Download [simExtROS2Interface](https://github.com/CoppeliaRobotics/simExtROS2Interface), a ROS2 plugin for Coppelia
```
git clone --recursive https://github.com/CoppeliaRobotics/simExtROS2Interface.git sim_ros2_interface
```
2. Install dependencies
```
sudo apt install xsltproc ros-$ROS_DISTRO-gazebo-ros-pkgs
```
3. Add custom messages and services to the package
```
cd sim_ros2_interface
echo $'\nsensor_msgs/msg/RegionOfInterest\nsensor_msgs/msg/CameraInfo' >> meta/interfaces.txt
```

4. Compile the plugin inside your ROS workspace
```
ulimit -s unlimited
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DLIBPLUGIN_DIR=$COPPELIASIM_ROOT/programming/libPlugin
```
5. Copy **build/sim_ros2_interface/libsimExtROS2Interface.so** from your ROS workspace to the Coppelia installation folder
6. Source ROS2 installation
```
source /opt/ros/$ROS_DISTRO/setup.bash
```
7. If the plugin has been loaded correctly, upon launching CoppeliaSim you should see the following:
```
Plugin 'ROS2Interface': loading...
Plugin 'ROS2Interface': warning: replaced variable 'simROS2'
Plugin 'ROS2Interface': load succeeded.
```

## Frequently Occuring Problems
1. If you get an ImportError with PyRep, start your IDE from a terminal (to export COPPELIASIM_ROOT) 
