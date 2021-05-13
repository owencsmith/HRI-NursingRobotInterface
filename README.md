# HRI-NursingRobotInterface

The following assumes you have ROS installed and have already followed the steps listed in the TRINA2 repository. If not that information can be found here:
https://github.com/hiro-wpi/TRINA-WPI-2.0

To run this project follow the steps below:

**Step 1:**
Switch to the ‘multi-robot-collaborative-search’ branch of the TRINA2 repository.

**Step 2:**
Clone the project repository for this project:
https://github.com/owencsmith/HRI-NursingRobotInterface

**Step 3:**
Install the required Python 3.6 dependences: PyQt5, PyQt5-sip, PyQt5-stubs

**Step 4:**
Use ‘catkin_make’ to build the catkin-workspace

**Step 5:**
Run `roscore`

**Step 6:**
Run the middleman node
`/catkin_ws/src/HRI-NursingRobotInterface$ rosrun middleman Middleman.py`

**Step 7:**
Run the trina2 simulation node
`../..$ roslaunch trina2_gazebo trina2_noplants.launch`
This launch file can be edited to include more robots in the simulation.

**Step 8:**
Run the supervisor UI(s)
`/catkin_ws/src/HRI-NursingRobotInterface/supervisorUI/src$ python3 SupervisorUI.py`
This can be called any number of times to achieve the number of desired supervisors.

**Step 9**
To start the search, click 'Create Task' in the Supervisor UI, and you can assign which robots you want to do the search and which items you want to search for.  Watch the demo video for more information for using the UI: https://www.youtube.com/watch?v=2NaMsUXWYJ0

**Step 10**
To change the searching method, in middleman.py, change the variable search_mode to any of the below searches, but keep in mind that only 0 and 1 work with the UI, since 2 and 3 were implemented for results comparision purposes only.

Parameters for coordinated search:
 0 = guard searching
 1 = guard clustering
 2 = force dispersion
 3 = random walk
