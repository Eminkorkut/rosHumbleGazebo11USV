
# 🚢 Ros2 Humble, Gazebo11 and USV

This guide explains the integration of USV (Unmanned Surface Vehicle) control within the Gazebo 11 simulation environment, utilizing ROS2 Humble for efficient communication and control.


<p align="center">
  <img src="https://github.com/Eminkorkut/rosHumbleGazebo11USV/blob/main/image/gazebo11.png" alt="Gazebo 11" width="800"/>
</p>

<p align="center">
  <img src="https://github.com/Eminkorkut/rosHumbleGazebo11USV/blob/main/image/ros2humble.png" alt="Ros 2 Humble" width="400"/>
</p>


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
sudo apt install curl
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
sudo apt-get install ros-humble-gazebo-ros-pkgs
sudo updatedb
sudo apt install plocate
locate libgazebo_ros_camera.so && locate libgazebo_ros_planar_move.so
# If the output is blank, use the following command
sudo cp -r plugin/* /opt/ros/humble/lib/
```

<h2>🔗 Useful Resources</h2>
<p>For further details on the technologies used in this project, refer to the official documentation:</p>
<ul>
  <li><a href="https://docs.ros.org/en/humble/" target="_blank">ROS2 Humble Documentation</a></li>
  <li><a href="https://classic.gazebosim.org/tutorials?tut=install_ubuntu" target="_blank">Gazebo 11 Documentation</a></li>
  <li><a href="https://github.com/ultralytics/ultralytics" target="_blank">YOLOv11 GitHub Repository</a></li>
</ul>


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
sed -i '/<depend>rclpy<\/depend>/a \  <depend>geometry_msgs</depend>\n  <depend>threading</depend>\n <depend>argparse</depend>\n   <depend>ast</depend>\n <depend>pynput</depend>\n  <depend>cv2</depend>\n  <depend>cv_bridge</depend>\n  <depend>sensor_msgs</depend>\n  <depend>ultralytics</depend>\n  <depend>numpy</depend>' package.xml
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
echo 'export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/opt/ros/humble/lib' >> ~/.bashrc
source ~/.bashrc
gazebo 2duba.world --verbose
```

<p1>Launching any topic</p1>
```bash
colcon build
source install/setup.bash
ros2 run myRos2 autonomous_boat_movement
```


## 🚀 Features

- Transferring Camera Image
- Object Detection
- Live preview
- USV movement with keyboard control
- Autonomous pontoon crossing


## 🧑‍💻 Programs/algorithms Used

**Operating System:** Ubuntu 22.04.5 LTS x86_64 

**Object Detect Algorithm:** YoloV11

**Simulation Environment:** Gazebo11

**Robot Control Framework:** Ros2 Humble

**Programming Language Used:** Python 3.10.12


<h2>🤝 Contribute to the Project</h2>
<p>We welcome contributions to improve this project! If you’d like to contribute, follow these steps:</p>
<ol>
  <li>Fork the repository: <a href="https://github.com/Eminkorkut/rosHumbleGazebo11USV/fork" target="_blank">Fork on GitHub</a></li>
  <li>Clone your forked repository:</li>
  <pre><code>git clone https://github.com/YOUR-NAME/rosHumbleGazebo11USV.git</code></pre>
  <li>Create a new branch:</li>
  <pre><code>git checkout -b feature-branch-name</code></pre>
  <li>Make your changes and commit them:</li>
  <pre><code>git add .
git commit -m "Describe your changes"
git push origin feature-branch-name</code></pre>
  <li>Open a Pull Request: <a href="https://github.com/Eminkorkut/rosHumbleGazebo11USV/pulls" target="_blank">Create a PR</a></li>
</ol>
<p>Thank you for your contributions! 🚀</p>


  
## ✍️ Authors

- [@Eminkorkut](https://github.com/Eminkorkut)

  


    
