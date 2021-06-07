# IITD_UV_Robot
Repository for the development of a UV robot capable of disinfecting indoor environments autonomously 

<h1><b> Overview </b></h1>

The UVBot is a ROS operated robot created to disinfect spaces such as schools, offices, and hospitals to inactivate COVID-19 with UV-C light. The robot can be controlled over Wi-fi using an android app, and the status of disinfection, battery level is displayed on it. It has many safety features such as emergency stop buttons, UV light control, PIR human detection sensors and voice commands to protect humans from UV exposure. Also using the strategically placed 8 IR sensors and 2D lidar, the robot is able to avoid any obstacle acorss its height. 
The UV robot disinfects the indoor environment autonomously, after building a 2D map using a fusion of IR-TOF and 2D lidar sensors, and using the ROS navigation package it achieves so. 

<p align="center"><img src="https://github.com/dhruvtalwar18/IITD_UV_Robot/blob/main/GIFs/Final_Simulation.gif" title="Result 1"></p>
<p align="center">Fig.1 Final Simulation of the UV-Bot</p>





<h1><b> From CAD to Simulation </b></h1>

After developing the full 3D model the robot(after selecting all the components and sensors), a URDF model of the robot was built so as to simulate it in the gazebo environment.
Once the robot was simulated, gazebo sensor and differential drive plugins were added to the robot. We used Rplidar A3 and 8 Terabee TOF sensors along with a raspi camera.
To control the robot we used a self developed android application using which we could give teleop commands to the bot

<p align="center"><img src="https://github.com/dhruvtalwar18/IITD_UV_Robot/blob/main/GIFs/App_movement_1.gif" title="Result 2"></p>
<p align="center">Fig.2 Movement of the bot using the app</p>



<h1><b> Robot Application</b></h1>

An Android app was created for the easy use and control of the robot. The robot connects to the android application easily via WiFi. Within the application, the user can personalise the robot by giving it a name as well as interact with it. There are three main windows within the application – The manual mode, autonomous mode and the creation of missions mode. Apart from this, the app shall be used to keep track off the on/off cycles of the UV light as well as the amount of hours they have been switched on for.
<p align="center"><img src="https://github.com/dhruvtalwar18/IITD_UV_Robot/blob/main/Images/app_1.PNG" title="Result 3" height = "400" width = "400" ></p>
<p align="center">Fig.3 Homescreen of the application</p><br><br><br>

In the manual mode the user can start the creation of a map of a floor in the building. Using the joystick within the app, the robot can be driven manually using the live feed from the camera and point that are to be sanitised can be marked here also.
<p align="center"><img src="https://github.com/dhruvtalwar18/IITD_UV_Robot/blob/main/Images/app_2.PNG" title="Result 3"  width = "500" ></p>
<p align="center">Fig.4 Manual Mode</p><br><br><br>


Once the map is created, the user has to merely touch various locations on the map and choose them as sanitisation points. While touching these points, user can see the area that will be sanitised in yellow colour. These saved missions can then be executed from the autonomous mode whenever required 
<p align="center"><img src="https://github.com/dhruvtalwar18/IITD_UV_Robot/blob/main/Images/app_3.PNG" title="Result 3" width = "300" height= "350"></p>
<p align="center">Fig.5 Create Mission Mode</p> <br><br><br>



Once the mission is in progress, this screen is shown to the user. Here all important information regarding the current state of the robot and progress of the mission is shown to the user. User may also control the lights of the robot or bring it to an emergency stop. If any safety related event happens during the mission, the robot via the android app can notify the user immediately.
<p align="center"><img src="https://github.com/dhruvtalwar18/IITD_UV_Robot/blob/main/Images/app_4.PNG" title="Result 3"></p>
<p align="center">Fig.6 Autonomous Mode</p> <br><br><br>









