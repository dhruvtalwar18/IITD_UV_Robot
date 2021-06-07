# IITD_UV_Robot
Repository for the development of a UV robot capable of disinfecting indoor environments autonomously 

<h1><b> Overview </b></h1>

The UVBot is a ROS operated robot created to disinfect spaces such as schools, offices, and hospitals to inactivate COVID-19 with UV-C light. The robot can be controlled over Wi-fi using an android app, and the status of disinfection, battery level is displayed on it. It has many safety features such as emergency stop buttons, UV light control, PIR human detection sensors and voice commands to protect humans from UV exposure. Also using the strategically placed 8 IR sensors and 2D lidar, the robot is able to avoid any obstacle acorss its height. 
The UV robot disinfects the indoor environment autonomously, after building a 2D map using a fusion of IR-TOF and 2D lidar sensors, and using the ROS navigation package it achieves so. 

<p align="center"><img src="https://github.com/dhruvtalwar18/IITD_UV_Robot/blob/main/GIFs/Final_Simulation.gif" title="Final Simulation of the UV-Bot"></p>
<p align="center">Fig.1 Final Simulation of the UV-Bot</p>





<h1><b> From CAD to Simulation </b></h1>

After developing the full 3D model the robot(after selecting all the components and sensors), a URDF model of the robot was built so as to simulate it in the gazebo environment.
Once the robot was simulated, gazebo sensor and differential drive plugins were added to the robot. We used Rplidar A3 and 8 Terabee TOF sensors along with a raspi camera.
To control the robot we used a self developed android application using which we could give teleop commands to the bot

<p align="center"><img src="https://github.com/dhruvtalwar18/IITD_UV_Robot/blob/main/GIFs/App_movement_1.gif" title="Movement of the bot using the app"></p>
<p align="center">Fig.2 Movement of the bot using the app</p>



<h1><b> Robot Application</b></h1>

An Android app was created for the easy use and control of the robot. The robot connects to the android application easily via WiFi. Within the application, the user can personalise the robot by giving it a name as well as interact with it. There are three main windows within the application – The manual mode, autonomous mode and the creation of missions mode. Apart from this, the app shall be used to keep track off the on/off cycles of the UV light as well as the amount of hours they have been switched on for.
About the three modes of the app
1. In the manual mode the user can start the creation of a map of a floor in the building. Using the joystick within the app, the robot can be driven manually using the live feed from the camera and point that are to be sanitised can be marked here also.<br>
2. In the create mission mode once the map is created, the user has to merely touch various locations on the map and choose them as sanitisation points. While touching these points, user can see the area that will be sanitised in yellow colour. These saved missions can then be executed from the autonomous mode whenever required <br>
3. In the autonomous mode all important information regarding the current state of the robot and progress of the mission is shown to the user. User may also control the lights of the robot or bring it to an emergency stop. If any safety related event happens during the mission, the robot via the android app can notify the user immediately. <br>

The following scripts need to be run to use all the features in the app:-

Connect the app to the ROS master URL.(They should be over the same network)

$ python throttle.py # This connects the joystick in the app to the cmd_vel topic of the simulation

$ python video.py    # It streams the live video from the camera sensor to the app screen

For the other modes run

$ python status.py\
$ python battery.py\
$ python coordinates_multiple_final.py


The above will get the sanitization status(UV on or off, % completed), battery % and a script to send the coordinates marked on the map to the move_base of the simulated robot

The Apk file of the application along with the souce code is attached in the App_APK folder of the repo. <br><br>

<p><img align ="left" src="https://github.com/dhruvtalwar18/IITD_UV_Robot/blob/main/Images/app_2.PNG" title="Manual Mode" width = "475"  ><img align ="right" src="https://github.com/dhruvtalwar18/IITD_UV_Robot/blob/main/Images/app_3.PNG" title="Create mission mode" width = "300" height = "350" ></p><br><br><br><br><br><br><br><br>
<br><br><br><br><br><br><br><br>
<p align="center">Fig.3 Manual Mode and Create mission mode respectively</p><br>
<br>

<p align="center"><img src="https://github.com/dhruvtalwar18/IITD_UV_Robot/blob/main/Images/app_4.PNG" title="Autonomous Mode"></p>
<p align="center">Fig.4 Autonomous Mode</p> <br><br>


<h1><b> Mapping Simulations</b></h1>

Before the mapping and navigation simulations are run a few scripts need to be run. Because of the lidar positioning, the rays collide with the robot structure and thus those array of rays need to be deleted before mappping is used. 

$ python Laser_robo_obstruction_Laser_new.py

Once the new laser scan is published it should then be fused with the IR sensors data so that incase there is an obstacle which is below the plane of the 2D lidar the robot can still detect it, as there are multiple terabee sensors along the length of the robot with FOV 20 deg. To fuse the data run the following script

$ python Laser_IR_fuse.py

Now we can run the package and start the mapping process

$ roslaunch uvbot_gazebo uvbot_world.launch\
$ roslaunch uvbot_navigation gmapping_demo.launch\
$ roslaunch uvbot_description uvbot_rviz_gmapping.launch

Now we can move the robot around the environement to make an occupancy grid of the environment.

Once we are happy with the map that can we seen on the rviz simulation we can save the map using

$ rosrun map_server map_saver -f ~/uvbot_ws/src/uvbot_navigation


<p align="center"><img src="https://github.com/dhruvtalwar18/IITD_UV_Robot/blob/main/GIFs/Mapping_final.gif" title=" Mapping Simulation"></p>
<p align="center">Fig.5 Mapping Simulation</p> <br><br>


<h1><b> Full Capability implementation </b></h1>

Once the map is made and is saved we can now launch the following along with the ones already running mentioned in the app section.

$ roslaunch uvbot_gazebo  uvbot_world.launch\
$ roslaunch uvbot_navigation amcl_demo.launch map_file:=<file path/map.yaml>\
$ roslaunch uvbot_description uvbot_rviz_amcl.launch\
$ python coordinates_multiple_final.py

Now we can select the points to be sanitised and the robot shall traverse there stop for a preset time(calculted from UV irradation exposure) and then go to the other selected points.
<p align="center"><img src="https://github.com/dhruvtalwar18/IITD_UV_Robot/blob/main/GIFs/Final_Simulation.gif" title="Final Simulation Demo"></p>
<p align="center">Fig.6 Final Simulation Demo</p> <br><br>

We can also see the outcome of the emergency stop button, once pressed the robot shall stop at that moment only, and once the issue is resolved if pressed again the robot shall continue its journey from that point itself, no need for full resetting the bot. 


<p align="center"><img src="https://github.com/dhruvtalwar18/IITD_UV_Robot/blob/main/GIFs/Emergency_Stop_button.gif" title="Emergency Stop"></p>
<p align="center">Fig.7 Emergency Stop</p> <br><br>

The whole control achitecture of the UV bot can be seen as shown in Fig. 8

<p align="center"><img src="https://github.com/dhruvtalwar18/IITD_UV_Robot/blob/main/Images/Hardware%20system%20structure%20and%20diagram%20of%20mobile%20robot.PNG" width = "600" height = "650" title="Control Architecture"></p>
<p align="center">Fig.8 Control Architecture</p> <br><br>

<h1><b> Future Scope</b></h1>

Now that the simulation is complete we will now start to build the actual robot, the codes used in the simulation will also have to be cleaned and be edited accordingly
We have manufactured the robot and have run manual control on the bot.
<p align="center"><img src="https://github.com/dhruvtalwar18/IITD_UV_Robot/blob/main/Real_bot/Test_1_gif.gif" title="Real bot test 1"></p>
<p align="center">Fig.9 Real bot test 1</p> <br><br>


<h1><b> Further Notes</b></h1>

The CAD design of the complete robot can be seen in the presentation and images uploaded.\
The UV-C dose is calculated from the document given to us by Micron Tech, further simulations were done by our team around this, the results can be seen in the UV folder uploaded.\
The functioning of the human detection PIR can be seen in the Images folder
