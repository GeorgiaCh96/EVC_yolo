# How to achieve communication between docker container and the jetson.

These instructions are applicable for Windows machines.

First of all, open the following applications:

* Docker Desktop

* XLaunch:
  1. Run XLaunch from the Windows Taskbar
  2. Select Multiple Windows
  3. Set Display number to 0
  4. Select Start no client
  5. Check Disable access control
  6. Finish

* Also, you have to disable the public network firewall because it will try to block UDP communications between your container and the 
Jetson


In this demo, we work with the YOLO scripts, and we will run the **processing node** on the docker container, while the **publisher** and **subscriber nodes** will run on the jetson.

In a nutshell:

* The **camera calibration_node** is publishing undistoreted images on the topic **/camera/img_undistorted**.

* The **processing_node_YOLO** is subscribing in undistorted images. It publishes the following motion commands:
  1.	STOP if the stop sign is detected, topic name: **/motion/stop_command** 
  2.	SLOW DOWN if a speed limit sign is detected, topic name: **/motion/speed_limit_command**

* The **final_subscriber_node** subscribes to these two topics.



First, in the **processing_node** you have to set fixed ports:

`rospy.init_node('processing_node', xmlrpc_port=49100, tcpros_port=49101)`

You're telling the processing_node (running inside Docker) — to:

* Accept XML-RPC requests (e.g., for parameter server, node registration) on port 49100

* Accept TCPROS connections (e.g., for topic communication -- pub/sub) on port 49101

If you don’t define the ports manually, then ROS chooses random ones — which will not
work across machines.

-----------------------------------------
DOCKER
------

First build the container:

`docker build -t yolo_node_new:latest .`


To interact with the container you have to explicitly set the ROS_MASTER_URI to the jetson's IP, and ROS_IP to your 
PC's IPv4 address. Also you have to expose the XML-RPC and TCPROS ports that you defined in the rospy.init_node.


`docker run -it --rm -e ROS_MASTER_URI=http://192.168.8.2:11311 -e ROS_IP=192.168.8.248 -p 49100:49100 -p 49101:49101 yolo_node_new
`


Inside the docker set these environment variables:

`export ROS_MASTER_URI=http://192.168.8.2:11311`

`export ROS_IP=192.168.8.248 `

`export DISPLAY=192.168.8.248:0`

Then run the following commands: 


#### Step 1: Clean the previous build (optional, but good after big changes)
`rm -rf build/ devel/`

#### Step 2: Source ROS (required for catkin_make)
`source /opt/ros/noetic/setup.bash`


#### Step 3: Rebuild your workspace
`catkin_make`

#### Step 4: Source your workspace (to expose messages, services, etc.)
`source devel/setup.bash`

#### Step 5: Add PYTHONPATH to .bashrc if not already added
`echo 'export PYTHONPATH=$PYTHONPATH:/home/ubuntu/yolo/devel/lib/python3.8/dist-packages' >> ~/.bashrc`

#### Step 6: Add workspace sourcing to .bashrc if not already added
`echo 'source ~/yolo/devel/setup.bash' >> ~/.bashrc`

#### Step 7 (only needed if you modified .bashrc during this session)

`source ~/.bashrc`

Then run the node

`cd src/yolo_package/src/`

`python3 yolov5_det.py`


------------------------------------------
JETSON
-----

First, check if `xeyes` works. 
If it does not work, then run on your local machine:

`ipconfig` and evaluate your PC's IPv4 Address. 

e.g. `IPv4 ...... 192.168.8.248`

Then run:

export `DISPLAY=192.168.8.248:0`

Check again if `xeyes` works. 
Then, navigate in your workspace, e.g.

`cd ~/EVC/workshops/Georgia_docker/workshop4_motion_2074028`

and run:

`catkin_make`

`source ~/.bashrc`

`source ./devel/setup.bash`

Note: Sometimes you need to run again 

`export DISPLAY=192.168.8.248:0
`
with the IP pointing to your PC's IPv4. If you do that, then run again:

`source ~/.bashrc`

and check if `xeyes` works

To launch the ROS publisher and subscriber nodes, run:

`roslaunch jetson_camera camera_publisher.launch`

`roslaunch jetson_camera camera_subscriber.launch`



