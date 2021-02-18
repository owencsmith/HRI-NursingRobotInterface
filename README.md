# HRI-NursingRobotInterface

The following assumes you have ROS installed and have already followed the steps listed in the TRINA2 repository. If not that information can be found here:
https://github.com/hiro-wpi/TRINA-WPI-2.0

To run this project follow the steps below:

**Step 1:**
Switch to the ‘multi_nav_middleman’ branch of the TRINA2 repository.

**Step 2:**
Clone the project repository for this project:
https://github.com/owencsmith/HRI-NursingRobotInterface

**Step 3:**
Install the required Python 3.6 dependences: PyQt5, PyQt5-sip, PyQt5-stubs
**Step 3:**
Use ‘catkin_make’ to build the catkin-workspace

**Step 4:**
Run `roscore`

**Step 5:**
Run the middleman node
`/catkin_ws/src/HRI-NursingRobotInterface$ rosrun middleman Middleman.py`

**Step 6:**
Run the trina2 simulation node
`../..$ roslaunch trina2_gazebo trina2.launch`
This launch file can be edited to include more robots in the simulation.

**Step 7:**
Run the supervisor UI(s)
`/catkin_ws/src/HRI-NursingRobotInterface/supervisorUI/src$ python3 SupervisorUI.py`
This can be called any number of times to achieve the number of desired supervisors.

**Step 8:**
Run the operator UI(s)
`/catkin_ws/src/HRI-NursingRobotInterface/operatorUI/src$ python3 OperatorUI.py`
This can be called any number of times to achieve the number of desired operators.

To see the task priority and reassignment at work, assign more tasks then there are robots. Your tasks will be scheduled and reassigned as necessary.
