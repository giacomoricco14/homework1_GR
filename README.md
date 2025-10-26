# Homework 1: Bring Up Your Robot
The goal of this project is to design and simulate a four-degrees-of-freedom robotic manipulator, named **Armando**, within the ROS 2 environment using **Gazebo** and **RViz** simulation software. 
Starting from the provided base package **armando_description**, the robot description was modified and extended to include visualization and physical modeling in RViz and Gazebo, and later the integration of sensors and controllers.
 
## Dockerfile modification
Open the Dockerfile and add the following line:
```sh
RUN echo "export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:/home/user/ros2_ws/src/armando_gazebo/models" >> ${HOME}/.bashrc
```
Then rebuild the docker image through the provided script:
```sh
./docker_build_image.sh <${image_name}>
```
**Warning**: Be sure that this is the only export path for IGN GAZEBO.

## Bring up Armando
Enter in ros2 workspace and build the following packages:
```sh
colcon build --packages-select armando_description armando_gazebo armando_controller
```
Hence, source the code:
```sh
source install/setup.bash
```

### RViz
Spawn Armando in RViz with the correct configuration:
```sh
ros2 launch armando_description armando_display.launch.py
```

### Gazebo
Run the Armando world in Gazebo specifing the controller you want to use through the argument `ctrl`:
* **position controller**:
```sh
ros2 launch armando_gazebo armando_world.launch.py ctrl:=0
```
* **trajectory controller**.
```sh
ros2 launch armando_gazebo armando_world.launch.py ctrl:=1
```
***Note*** that by default is set on the position controller.

Once the robot is in the Gazebo world, run the controller node opening a new terminal:
* **position controller**:
```sh
ros2 run armando_controller armando_controller_node 0
```
* **trajectory controller**:
```sh
ros2 run armando_controller armando_controller_node 1
```
