## Study GUI
To start the main study gui, use ``roslaunch hri_preferences_study begin_study``. To specify whether you are using the standalone arm or whole robot body, comment/uncomment the appropriate lines in ``begin_study.launch``.

### Control Method
The user has 3 control methods: Autonomous, Direct Control, and Shared Control. 

### Controller
The user may choose from 3 controllers: xbox, web interface, or keyboard.
- If using the xbox controller, no further action is needed. 
- If using the web interface, connect to the robot via ethernet, enter ``roslaunch rosbridge_server rosbridge_websocket.launch``, and enter the appropriate address into your browser.
- If you are using the keyboard, open a new tab in Terminal and enter ``rosrun hri_preferences_study keyboard_to_joystick.py``. If you are connecting a tobii dynavox, choose this option.

### Goals
You may choose the goal location from the dropdown list in the gui. To create the goal locations, use: ``roslaunch build_profile.launch``. This will let you use the xbox controller to navigate the arm to the goal position, then save all of the joint positions of the robot.