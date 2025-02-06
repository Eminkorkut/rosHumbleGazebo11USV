
# 🚢 Ros2 Humble, Gazebo11 and USV

USV control in the gazebo11 simulation environment using ros2 humble code frameworks.

<h2>🛠️ Installation Steps:</h2>

<p1>Download if you don't have git</p1>
```bash
sudo apt install git
```

<p1>Clone the repo</p1>
```bash
git clone https://github.com/Eminkorkut/rosHumbleGazebo11USV.git
```

<p1>Get inside the repo</p1>
```bash
cd rosHumbleGazebo11USV
```

<p1>Ros2 Humble installation</p1>
```bash
cd requirements/
chmod +x ros2Install.bash
./ros2Install.bash
cd ..
```

<p1>Gazebo11 installation</p1>
```bash
cd requirements/
chmod +x gazeboInstall.bash
./gazeboInstall.bash
cd ..
```

<p1>Python and library setup</p1>
```bash
sudo apt install python3
sudo apt update
sudo apt upgrade
sudo apt install python3-pip
pip3 install -r requirements/requirementsPython.txt
```

<p1>Plugin controls</p1>
```bash
locate libgazebo_ros_camera.so && locate libgazebo_ros_planar_move.so
# If the output is blank, use the following command
sudo cp -r rosHumbleGazebo11USV/plugin/* /opt/ros/humble/lib/
```

<h2>🛠️ Stages of Use</h2>

<p1>Create a working environment</p1>
```bash
cd rosHumbleGazebo11USV
```

<p1>Colcon and Ros2 form the foundation</p1>
```bash
colcon build
cd src/
ros2 pkg create myRos2 --build-type ament_python --dependencies rclpy
cd ..
```

<p1>Move codes to required places</p1>
```bash
mv src/*.py src/myRos2/myRos2/
```

<p1>Make the necessary adjustments</p1>
```bash
cd src/myRos2
sed -i '/<depend>rclpy<\/depend>/a \  <depend>geometry_msgs</depend>\n  <depend>threading</depend>\n  <depend>pynput</depend>\n  <depend>cv2</depend>\n  <depend>cv_bridge</depend>\n  <depend>sensor_msgs</depend>\n  <depend>ultralytics</depend>\n  <depend>numpy</depend>' package.xml
sed -i "/'console_scripts': \[/a \ \ \ \ \ \ \ \ \ \ \ \ \"boat_control_with_keyboard = myRos2.boat_control_with_keyboard:main\",\n\ \ \ \ \ \ \ \ \ \ \ \ \"transfer_camera_frame = myRos2.transfer_camera_frame:main\",\n\ \ \ \ \ \ \ \ \ \ \ \ \"autonomous_boat_movement = myRos2.autonomous_boat_movement:main\"," setup.py
cd ../..
```

<p1>Build the project</p1>
```bash
colcon build
source install/setup.bash 
```

<p1>Start and debug gazebo in a terminal</p1>
```bash
cd world/
gazebo 2duba.world --verbose
```

<p1>Launching any topic</p1>
```bash
colcon build
source install/setup.bash
ros2 run myRos2 autonomous_boat_movement
```


## Features

- Transferring Camera Image
- Object Detection
- Live preview
- USV movement with keyboard control
- Autonomous pontoon crossing


## Programs/algorithms Used

**Operating System:** Ubuntu 22.04.5 LTS x86_64 

**Object Detect Algorithm:** YoloV11

**Simulation Environment:** Gazebo11

**Robot Control Framework:** Ros2 Humble

**Programming Language Used:** Python 3.10.12


  
## Authors

- [@Eminkorkut](https://github.com/Eminkorkut)

  


    
