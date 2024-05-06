# Navigation Module Template

This is the template for the navigation module for Worldskills 2024 Lyon Autonomous Mobile Robotics Skill 23. 

## Projects

There is a java and c++ project that will drive any of the five robots forward at 50% speed. Currently, only the Stack Robot is set to output anything, modify the project as needed for the robot in use. 

**Only the sensors available on the robots have had code added. If there is no code for a sensor it is not wired on the robot. Ex.. Encoders.**

## MockDS

Use of these robots requires MockDS. MockDS is an internal driverstation that will enable or disable the robot. The **Start** and **E-Stop** of each robot has been wired to enable / disable the robot. Once enabled after pressing the **Start** button the code will execute the auto command that is set in RobotContainer. To stop the robot hit the **E-Stop** plunger. 

When MockDS code is included in a project the console station **cannot** be used. Dashboards **can** be used for diagnostic purposes. 

During the acutal run no computer will be connected. 

