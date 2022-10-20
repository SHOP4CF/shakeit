# Shakeit!
![shakeit_overview](docs/shakeit.png)

## Table of Contents
* [Setup and Start](https://github.com/SHOP4CF/shakeit#setup-and-start)
  * [Windows](https://github.com/SHOP4CF/shakeit#windows)
    * [Prerequisites](https://github.com/SHOP4CF/shakeit#prerequisites)
      * [Installing WSL](https://github.com/SHOP4CF/shakeit#installing-wsl)
      * [Installing Docker Desktop WSL Backend](https://github.com/SHOP4CF/shakeit#installing-docker-desktop-wsl-backend)
      * [Installing Xserver](https://github.com/SHOP4CF/shakeit#installing-xserver)
    * [Build](https://github.com/SHOP4CF/shakeit#build)
    * [Run](https://github.com/SHOP4CF/shakeit#run)
* [Requirements/dependencies](https://github.com/SHOP4CF/shakeit#requirementsdependencies)
* [Packages](https://github.com/SHOP4CF/shakeit#packages)
* [Notes](https://github.com/SHOP4CF/shakeit#notes)

# Setup and Start

## Windows

### Prerequisites
There are three main prerequisties to running this project:
* Windows WSL
* Docker Desktop - wsl Backend
* VcXsrv Windows X server

#### Installing WSL
You can install WSL either by following the [documentation](https://learn.microsoft.com/en-us/windows/wsl/install) or the steps written here:

1. Run the following command in an **administrator** PowerShell or Windows Command Prompt
```
wsl --install
```
The first time you launch a newly installed Linux distribution, a console window will open and you'll be asked to wait for files to de-compress and be stored on your machine. All future launches should take less than a second. <br><br>
If you run ```wsl --install``` and see the WSL help text, you already have WSL installed
<br>

2. Restart your machine

#### Installing Docker Desktop WSL Backend
You can install Docker Desktop either by following the [documentation](https://docs.docker.com/desktop/windows/wsl/) or the steps written here:

1. Download and install [Docker Desktop](https://docs.docker.com/desktop/install/windows-install/) (2.3.0.2 or a newer release)

2. Start Docker desktop

3. Go to Setting &rarr; General
![Skærmbillede_dockerDesktopWSL](https://user-images.githubusercontent.com/113982478/196695294-9cc5960c-6b4a-4078-a951-ee68180f2f68.png)

4. Select the _Use the WSL 2 based engine_ checkbox

5. Click _Apply & Restart_

#### Installing Xserver
The Xserver is used to connect to the display.

You can install Windows Xserver either by following this [guide](https://dev.to/darksmile92/run-gui-app-in-linux-docker-container-on-windows-host-4kde) or the steps written here:

1. Install [VcXsrv Windows X Server](https://sourceforge.net/projects/vcxsrv/)

2. run _Xlaunch_ from the start menu

3. Setup the configuration by following the steps shown in the pictures below

![1](https://res.cloudinary.com/practicaldev/image/fetch/s--MCnNoPwj--/c_limit%2Cf_auto%2Cfl_progressive%2Cq_auto%2Cw_880/https://thepracticaldev.s3.amazonaws.com/i/g3roivsrapgy69mqkhpc.png)
![2](https://res.cloudinary.com/practicaldev/image/fetch/s--9T2fJDCh--/c_limit%2Cf_auto%2Cfl_progressive%2Cq_auto%2Cw_880/https://thepracticaldev.s3.amazonaws.com/i/5l5fil0nongqswsc5qx5.png)
![3](https://res.cloudinary.com/practicaldev/image/fetch/s--1fOShFRZ--/c_limit%2Cf_auto%2Cfl_progressive%2Cq_auto%2Cw_880/https://thepracticaldev.s3.amazonaws.com/i/3eh1lry7125modpdj6a2.png)
![4](https://res.cloudinary.com/practicaldev/image/fetch/s--GFylK6hC--/c_limit%2Cf_auto%2Cfl_progressive%2Cq_auto%2Cw_880/https://thepracticaldev.s3.amazonaws.com/i/48tl3o3pv99vbhk06188.png)

4. Save the configuration file before you finish!
   - Save it to one of the following locations:
     - %appdata%\Xming
     - %userprofile%\Desktop
     - %userprofile%

### Build
Clone the repository

Build the docker image using the following command:

```powershell
docker build -t shakeit .
```

### Run
Run VsXrsv server by opening the config file that was saved during installation. When an Xserver is running you can find the logo in the taskbar as shown below.

![Skærmbillede_Xserver](https://user-images.githubusercontent.com/113982478/196703595-1bd388d0-f523-4b2c-8f1c-5be9a5386296.png)

<br>
Run the following line in Windows PowerShell to start the docker container.

```powershell
$shakeip = Get-NetIPAddress -InterfaceAlias "vEthernet (WSL)" | select -exp "IPAddress"; docker run -it --rm --name shakeit -e DISPLAY=${shakeip}:0.0 shakeit ros2 launch shakeit_experiments run_sim_experiment.launch.py
```

<br>

If PowerShell shows the error ```exec /ros_entrypoint.sh: no such file or directory``` then:

1. Open the _ros_entrypoint.sh_ file loacted in root
2. Click on CRLF and change it to LF
![CRLF-to-LF](https://i.stack.imgur.com/GEoYt.png)

3. Rebuild image and run container again

<br>

Run with docker-compose together with moni2, remember to set DISPLAY_IP in .env and to run the command in the same folder as docker-compose.yml
```powershell
docker-compose up
```


# Requirements/dependencies
* ROS2
* [CoppeliaSim, plugin for ROS2 and PyRep](ros_pkg_ws/src/shakeit_sim/README.md)

### Configure ROS
In ~/.bashrc define environment variables for your ROS2 installation, e.g:
```
export ROS_VERSION=2
export ROS_PYTHON_VERSION=3
export ROS_DISTRO=foxy

source /opt/ros/foxy/setup.bash
```
Source ~/.bashrc
```
$ source ~/.bashrc
```

### Install dependencies
```
$ cd ros_pkg_ws
$ sudo apt install python3-wstool
$ sudo apt-get install python3-rosdep2
$ rosdep update
$ wstool init src/ src/src_dependencies.rosinstall --shallow
$ rosdep install --default-yes --ignore-packages-from-source --from-path ./src
``` 

NOTE: If the rosdep update gives errors change the following export in the bashrc file (Solution found here [rosdep error](https://github.com/ros-infrastructure/rosdep/issues/576))

Replace the following line:
```
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$COPPELIASIM_ROOT
```

with the following:
```
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH:$COPPELIASIM_ROOT
```


### Build

Install Colcon
```
sudo apt install python3-colcon-common-extensions
```

Check how to source Corppelia stuff: `ros_pkg_ws/setup.bash_example`

```
$ cd ros_pkg_ws
$ source setup.bash
$ colcon build
```

If you're not using CoppeliaSim skip the sim packages by:

```
colcon build --packages-skip sim_ros2_interface
```

Now go into the launch file and edit the path in [L24](http://robotgit.localdom.net/ai-box/applications/shakeit/-/blob/master/ros_pkg_ws/src/shakeit_experiments/launch/run_experiment.launch.py#L24)
to the path of the src folder in this repo.

### Pycharm
To open PyCharm with code-completion and imports working:
```
$ cd ros_pkg_ws
$ source setup.bash
$ pycharm
```

# Packages
* [shakeit_core](ros_pkg_ws/src/shakeit_core/README.md): contains basic functionality
* [shakeit_interfaces](ros_pkg_ws/src/shakeit_interfaces/README.md): common messages, services, and actions for the shakeit-project
* [shakeit_models](ros_pkg_ws/src/shakeit_models/README.md): models used for training and prediction
* [shakeit_sim](ros_pkg_ws/src/shakeit_sim/README.md): simulation of the vibration feeder

# Notes
Link to sensopart camera pc-software [here](https://www.sensopart.com/en/service/downloads/90-visor-pc-software/)
