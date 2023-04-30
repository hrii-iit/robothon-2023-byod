# ROBOTHON 2023 Bring Your Own Device
Measurement of battery's voltage inside a clicker with a voltmeter manipulated by two Franka Emika robotics arms. The voltage measured by the voltmeter is then captured using a camera mounted on the Franka robotic arm and eventually displayed to the user using a GUI.

![BYOD_setup](/uploads/066a6c604237d5409007eafb031bc9a6/BYOD_setup.PNG)
## Contents 

1. [Installation](https://gitlab.iit.it/hrii/projects/robothon/hrii_robothon_byod#installation)
2. [Repo Structure](https://gitlab.iit.it/hrii/projects/robothon/hrii_robothon_byod#repo-structure)
3. [Contributors](https://gitlab.iit.it/hrii/projects/robothon/hrii_robothon_byod#contributors) 

## Installation
### Prerequisites 
- Ubuntu 20.04
- ROS Noetic

### Clone the repo
1. Create a catkin workspace 
2. Clone the repo using the following commands:

```
cd ~/your-catkin-ws/src/
git clone https://gitlab.iit.it/hrii/projects/robothon/hrii_robothon_byod

```
3. Invoke catkin tool inside ros workspace i.e. `catkin_make`

### Dependencies

- [hrii_board_localization](https://gitlab.iit.it/hrii/projects/robothon/hrii_board_localization)
- [hrii_task_board_description](https://gitlab.iit.it/hrii/projects/robothon/hrii_task_board_description)
- [hrii_robothon_gripper_description](https://gitlab.iit.it/hrii/projects/robothon/hrii_robothon_gripper_description)
- [hrii_robothon_msgs](https://gitlab.iit.it/hrii/projects/robothon/hrii_robothon_msgs)
- [hrii_task_board_fsm](https://gitlab.iit.it/hrii/projects/robothon/hrii_task_board_fsm)


### Launch Franka Emika manipulators
```
roslaunch hrii_robothon_byod main_byod.launch

```

### Launch the camera with the GUI
```
cd ~/your-catkin-ws/src/hrii_board_localization/scripts
python scripts/read_text.py

```
## Repo Structure
```
hrii_robothon_byod
├── config
├── include
├── launch
├── rviz
├── src
└── README.md


```

## Contributors
- Andrea Fortuna, [Andrea Fortuna](https://gitlab.iit.it/Andrea.Fortuna)
- Pietro Balatti, [Pietro Balatti](https://gitlab.iit.it/pietrobalatti)
- Mattia Leonori, [Mattia Leonori](https://gitlab.iit.it/mleonori)
- Hamidreza Raei, [Hamidreza Raei](https://gitlab.iit.it/Hamidreza.Raei)
