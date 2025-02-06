
# Ros2 Humble, Gazebo11 and USV

USV control in the gazebo11 simulation environment using ros2 humble code frameworks.

<h2>🛠️ Installation Steps:</h2>

<p1>Download if you don't have git</p1>
```bash
  sudo apt install git
```

<p1>Clone the Repo</p1>
```bash
  git clone https://github.com/Eminkorkut/rosHumbleGazebo11USV.git
```

<p1>Ros2 Humble Installation</p1>
```bash
  cd requirements/
  chmod +x ros2Install.bash
  ./ros2Install.bash
```

<p1>Gazebo11 Installation</p1>
```bash
  cd requirements/
  chmod +x gazeboInstall.bash
  ./gazeboInstall.bash
```

<p1>Python and Library Setup</p1>
```bash
  sudo apt install python3
  sudo apt update
  sudo apt upgrade
  sudo apt install python3-pip
  cd requirements/
  pip3 -r requirementsPython.txt
```

<p1>Plugin Controls</p1>
```bash
  locate libgazebo_ros_camera.so && locate libgazebo_ros_planar_move.so ## If the output is blank, use the following command
  sudo cp -r rosHumbleGazebo11USV/plugin/* /opt/ros/humble/lib/
```

<h2>🛠️ Stages of Use</h2>

<p1>Plugin Controls</p1>
```bash
  locate libgazebo_ros_camera.so && locate libgazebo_ros_planar_move.so ## If the output is blank, use the following command
  sudo cp -r rosHumbleGazebo11USV/plugin/* /opt/ros/humble/lib/
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

  


    
