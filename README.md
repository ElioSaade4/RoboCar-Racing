- [Outline](#outline)
- [Software Requirements](#software-requirements)
- [Simulation Setup](#simulation-setup)
- [Building the Docker Application (ONCE)](#building-the-docker-application-once)
- [Running the F1Tenth Gym Environment](#running-the-f1tenth-gym-environment)
- [Visualization in RVIZ](#visualization-in-rviz)
- [Controlling the Car with Keyboard Keys](#controlling-the-car-with-keyboard-keys)
- [Running the MPC Node](#running-the-mpc-node)
- [Running the waypoints\_logger Node](#running-the-waypoints_logger-node)
- [Running the waypoints\_marker Node](#running-the-waypoints_marker-node)
- [Visualization with Foxglove](#visualization-with-foxglove)
  - [Live telemetry with Foxglove](#live-telemetry-with-foxglove)
  - [Replay ROS Bag with Foxglove](#replay-ros-bag-with-foxglove)


# Outline
This repo includes the codebase for the final project for EEE 587 - Optimal Control, entitled **Model Predictive Control for F1Tenth Autonomous Racing**. It implements a Model Predictive Controller for an RC car in the context of the F1Tenth competition. The controller follows a reference racing line and therefore allows the car to drive autonomously in qualifying format.

The repository is organized as follows:
- **README Images:** includes all the images of this README file
- **recording:** folder that includes a ROS bag recording of a simulation run, which can be opened and replayed in Foxglove.
- **sim_ws**: folder that includes all the code of the project. It incldues the following 3 folders:
  - **f1tenth_gym_ros:** ROS2 python package that includes the ros bridge to the F1Tenth Gym and the keyboard teleoperations node. **This package is available open-source along with the F1Tenth Gym and was not developed by me.**
  - **mpc:** ROS2 python package that include the code for *mpc_node* which implements the MPC.
  - **waypoints:** ROS2 python package that includes the code for *waypoints_logger* and *waypoints_marker* nodes.
- **Dockerfile:** defines the image of the **sim-1** Docker container.
- **docker-compose.yml:** defines the structure of the multi-container Docker application, along with the containers, images, memory volume and port mappings.

# Software Requirements
- Docker Desktop (Necessary to run simulations)
- Foxglove (Optional: to get live or offline telemetry from the simulations)

# Simulation Setup
<img src="README Images/SimulationSetup.png" width="600"/>

The simulation setup consists of 2 Docker containers (services) running as part of a Docker application. Docker was needed because ROS requires Linux to run and I have a Windows machine, so 2 options were available: use a Linux Virutal Machine (or WSL) or use Docker. Docker was chosen because it seemed more popular in the robotics community and it makes it much easier to share the whole simulation environment.

The first service **(sim-1)** is built based on a custom Docker image which is defined in the **Dockerfile** file. This custom image starts from the **ros:iron** image and adds to it a python installation with the needed dependencies as well as a copy of the F1Tenth Gym environment. This service is where the simulation runs and where ROS2 nodes interact with the F1Tenth Gym. The folder **sim_ws** inside the container running **sim-1** is mapped to the host system folder **sim_ws** where this repo gets cloned; so any change inside the container appears in the host memory and vice versa. This allows file exchange and the development of ROS2 code that runs inside the container. Also, port 8765 of the container is mapped to port 8765 of the computer.

The second service **(novnc-1)** is built from the readily available **theasp/novnc:latest** image. This container provides a remote connection at port 8080 using noVNC to RVIZ which is running in **sim-1**. To make that possible, port 8080 in this container is mapped to port 8080 of the computer. This service is not required to be running all the time: it is only needed to view the visualization in RVIZ. If the goal is to get live telemetry in Foxglove or to record a ROS bag to replay in Foxglove, **(novnc-1)** can be stopped.

# Building the Docker Application (ONCE)
To set up the simulation environment above, the Docker containers for the 2 services need to be built based on their respective images, then need to be combined into one application. The **Dockerfile** file defines the image for **sim-1** and **docker-compose.yml** consists the blueprint for building the muli-container application, including the services, images, memory mappings, and ports mappings.


To build the application, follow the steps below:

1. Clone the repository to a local directory **C:\\...\RoboCar-Racing**

2. Open a command prompt, and navigate to the cloned repository folder
```
cd C:\...\RoboCar-Racing
```

3. Run the following command in the command prompt to build the containers and application. This should take about 10-15 minutes.
```
docker-compose up --build
```
**NOTE: It seems that the command fails to fetch some files online while I am connected to ASU's network. So I suggest running it while connected to a home network or personal hotspot.**

After the command runs successfully, your command prompt should look as follows:

<img src="README Images/DockerCompose.png" width="600"/>

Also, the 2 containers (sim-1 and novnc-1) in the application (robocar-racing) should be up and running as can be seen if you open Docker Desktop.

<img src="README Images/DockerDesktop.png" width="600"/>


**IMPORTANT:** The 3 steps above are only needed once in order to build the containers. After the containers are successfully built, they can simply be started or stopped using the **Start** and **Stop** icons under the **Actions** column (check image above).

Alternatively, the containers can be started and stopped from the command prompt using:
```
docker start robocar-racing-sim-1
```
```
docker start robocar-racing-novnc-1
```
```
docker stop robocar-racing-sim-1
```
```
docker stop robocar-racing-novnc-1
```

# Running the F1Tenth Gym Environment
The first step in performing simulations is to start the F1Tenth Gym environment inside the **sim-1** container. 

To do so, follow the steps below:

1. Start the **sim-1** container from Docker Desktop or from the command promt. (Check IMPORTANT in the previous section)

2. The next step is to open a bash session inside the **sim-1** container. To do that, open a command prompt and run the following command:
```
docker exec -it robocar-racing-sim-1 /bin/bash
```
This is how the command prompt should look like after steps 1 & 2:

<img src="README Images/SimBash.png" width="600"/>

3. In the bash session, source the needed **ros** files, then build the project by running these commands in order:
```
cd .. 
source opt/ros/iron/setup.bash
cd sim_ws 
colcon build
source install/local_setup.bash
```
<img src="README Images/BashBuild.png" width="600"/>

4. Start **tmux** in the bash session with:
```
tmux
```
**tmux** will allow you to open mutliple bash sessions inside the container without repeating steps 2-3. Use **ctrl+B - C** to open a new bash session in tmux. Use **ctrl+B - #** to navigate between the sessions with # being the number of the session. For example, **ctrl-B-1** to go to session 1.

The image below shows how the terminal looks like after starting tmux and opening 2 sessions (session 0 and session 1), both running inside the container:

<img src="README Images/BashTmux.png" width="600"/>

5. In the first bash session, launch the gym environment by typing the command:
```
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

<img src="README Images/BashGym.png" width="600"/>

As such, the F1Tenth Gym environment is up and running. From here, you can choose to visualize it using RVIZ, run ROS2 nodes, get live telemetry in Foxglove, or record a ROS bag to replay offline in Foxglove. These functionalities are detailed in the following sections.

# Visualization in RVIZ
After starting the F1Tenth environment, it can be visualized in RVIZ using the **novnc-1** service. RVIZ is a live visual tool only, it does not provide any telemetry or data logging. 

To view the simulation in RVIZ, follow the steps below:
1. Start the **novnc-1** container from Docker Desktop (or from command promt).

2. In a web browser, open http://localhost:8080/vnc.html and click **Connect**.
For better visualization, change the **Scaling Mode** setting to **Local Scaling**.

<img src="README Images/RVIZ.png" width="600"/>

# Controlling the Car with Keyboard Keys
After starting the Gym environment, the car can be controlled with keyboard commands using the teleoperations ROS node.

In another bash session, run the keyboard teleoperations node to be able to drive the car by pressing keyboard keys:
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

<img src="README Images/Teleop.png" width="600"/>

The car can be moved by clicking on keyboard keys while in this bash session as described in the image.

# Running the MPC Node
To start the MPC controller, open a new bash session and run **mpc_node**:
```
ros2 run mpc mpc_node
```

# Running the waypoints_logger Node
The **waypoints_logger** node can be run in a seperate bash session using the following command:

```
ros2 run waypoints waypoints_logger
```

When the node is stopped using **ctrl-C**, the resulting log file can be found at ***./RoboCar-Racing/sim_ws/src/waypoints/waypoints_log.csv***

# Running the waypoints_marker Node
The **waypoints_marker** node can be run in a seperate bash session using the following command:
```
ros2 run waypoints waypoints_marker
```

# Visualization with Foxglove
An alternative to RVIZ is to use Foxglove to visualize the simulation. Foxglove offers 2 options: live telemetry or offline replay of a ROS bag recording.

**NOTE:** To use Foxglove, the **novnc-1** service does not need to be running. In fact, it is better to stop it in order to free computation resources and improve the performance of Foxglove.

## Live telemetry with Foxglove
In order to connect the F1Tenth Gym environment to Foxglove for live telemetry, follow the steps below:

1. In a new bash session inside **sim-1**, launch the **foxglove_bridge** node:
```
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

<img src="README Images/FoxgloveBridge.png" width="600"/>

2. Open Foxglove, click on **Open connection...**, then make sure that the WebSocket URL is set to **ws://localhost:8765**, and click **Open**.

<img src="README Images/FoxgloveTelemetry.png" width="600"/>

Foxglove should now connect and you should see data flowing into Foxglove. You can then set the panels as desired and save it as a template.

## Replay ROS Bag with Foxglove
The downside of live telemetry is that it is computationally intensive and hinders the performance of the F1Tenth environment and the MPC running in the container. Therefore, ROS bag can be used to record all the topics while the simulation is running; then, the recording can be loaded into Foxglove and replayed/analyzed offline.

To record a session using ROS bag, open a new bash session in the **sim-1** container, then navigate to the **sim_ws/src/** folder. Then start the recording using the following command:
```
ros2 bag record -a -o session_name
```
where **session_name** is the recording folder/file name. The recording folder can be found at ***./RoboCar-Racing/sim_ws/src/session_name***

To open the recoding in Foxglove, click on **Open local file(s)...** then select the recording file.