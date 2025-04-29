# ahg-sim

## TODO
- Use the sim to figure out how to use rtabmap.
- The sim publishes the bare minimum topic in order for rtabmap to work
```
/camera/depth/image
/camera/depth/camera_info
/camera/depth/points (shouldn't need to use this)
/camera/color/image/compressed
/camera/color/camera_info
```
NOTE: In order to visualize the rgb topic, you need to uncompress the image. Attached to this repo is a python script that does this. Copy it into your machine and run `python3 decompressor.py`.

- Rtabmap should work with just rgb image, depth, and camera info. The question to answer is how to get it to work.

## Setup instructions:
1) Download the fri-sim.zip file under releases. Unzip it. This is the sim exe.
2) To create a new workspace to try the sim out, run these commands (The ROS-TCP-ENDPOINT is required):
```
mkdir test-sim
cd test-sim
mkdir src
cd src
git clone -b main-ros2 https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
cd ..
```
3) You can now build the workspace:
```
colcon build
source install/setup.bash
```
4) Get the IP address of the linux machine (if there are multiple, pick the first one). This is necessary in order to connect the sim to the workspace:
```
hostname -I
```
5) Launch the ROS-TCP-Connector (Replace <your IP address> with the ip address from step 4)
```
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=<your IP address>
```

6) Open the unzipped folder. Open fri-sim.x86_64. Follow the launch screen instructions below:


## Launch Screen
1) Click the input field
2) Copy the IP address from step 4 into here as well.
3) Click \<enter\> on the keyboard.
4) Click the enter button the the screen.

Controls:

\<K\>: Kill the simulator (use this to exit)

## Simulator Screen
Controls:

\<WASD\>: Hold WASD to move the robot around.

\<Mouse\>: Drag the mouse to look around.

\<Space\>: Locks the mouse to the center of the screen, you can unlock it by clicking \<Space\> again. (may or may not work)

\<K\>: Kill the simulator (use this to exit)
