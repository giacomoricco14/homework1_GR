# :robot: Homework 1: Bring Up Your Robot
The goal of this project is to design and simulate a four-degrees-of-freedom robotic manipulator, named **Armando**, within the ROS 2 environment using **Gazebo** and **Rviz** simulation software. 
Starting from the provided base package **armando_description**, the robot description was modified and extended to include visualization and physical modeling in Rviz and Gazebo, and later the integration of sensors and controllers.
 
## :hammer_and_wrench: Dockerfile modification
This package depends on the following ROS2 Humble packages: ros-humble-ros-ign-bridge, ros-humble-ros-gz, ros-humble-controller-manager, ros-humble-ros2-control, ros-humble-ros2-controllers, and ros-humble-ign-ros2-control, which are already included in the provided Dockerfile. Additional dependencies such as ros-humble-urdf-tutorial, ros-humble-xacro and ros-humble-ros-gz-sim have been manually added.
Therefore, open the Dockerfile and add the following line:
```sh
RUN echo "export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:/home/user/ros2_ws/src/armando_gazebo/models" >> ${HOME}/.bashrc

RUN apt-get install -y ros-humble-ros-ign-bridge && \
apt-get install -y ros-humble-ros-gz && \
apt-get install ros-humble-controller-manager -y && \
apt-get install ros-humble-ros2-control -y && \
apt-get install ros-humble-ros2-controllers -y && \
apt-get install ros-humble-ign-ros2-control -y && \

#adding 
apt-get install ros-humble-urdf-tutorial -y && \
apt-get install ros-humble-xacro -y && \
apt-get install ros-humble-ros-gz-sim -y
```
Then rebuild the docker image through the provided script:
```sh
./docker_build_image.sh <${image_name}>
```
:warning: ***Warning*** Be sure that this is the only export path for IGN GAZEBO.

## :rocket: Bring up Armando
Once the image is updated, download this repository in the image folder:
```sh
git clone https://github.com/giacomoricco14/homework1_GR.git
```
### :hammer: Build the packages
Enter in ros2 workspace and build the following packages using `colcon build` and selecting the required packages:
```sh
colcon build --packages-select armando_description armando_gazebo armando_controller
```
Hence, `source` the code:
```sh
source install/setup.bash
```

### :white_check_mark: Rviz
Spawn Armando in Rviz with the correct configuration using the `ros2 launch` command:
```sh
ros2 launch armando_description armando_display.launch.py
```

### :white_check_mark: Gazebo
Run the `armando_world.launch.py` in Gazebo by specifying the controller you want to use through the argument `ctrl`:
* **position controller**:
```sh
ros2 launch armando_gazebo armando_world.launch.py ctrl:=0
```
* **trajectory controller**.
```sh
ros2 launch armando_gazebo armando_world.launch.py ctrl:=1
```
:mag: ***Note*** by default is set on the position controller.

#### :camera: Armando Camera
Open an other terminal and write the following command to see what the camera shows:
```sh
ros2 run rqt_image_view rqt_image_view
```

### :white_check_mark: Armando Controller
Once the robot is in the Gazebo world, in an other terminal run the `arm_controller_node` by specifying the controller type and setting the argument as follows:
* **position controller**:
```sh
ros2 run armando_controller arm_controller_node 0
```
* **trajectory controller**:
```sh
ros2 run armando_controller arm_controller_node 1
```
:mag: ***Note*** also in this case the default item is the position controller.
