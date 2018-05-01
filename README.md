# FinalProject6152 Kinematic simulations using Klampt

### Execution:
```
cd simTests
python MyRRT.py simpleWorld.xml
```

### Robot files

  The folder `mobile_robots` contain the descriptions of the following robots:
1. Unicycle: non holonomic robot with two degrees of freedom.
2. sphero: It is a three degrees of freedom robot. However, it can be forced as
   a six DoF system as done in the samples.

### Simulation files

   The folder `simTests` contains files to perform simple kinematic simulations.
   1. simpleWorld.xml: Contains information on which robot to use and the size
      of the terrain. One should modify this file (or make a copy) to change the type of robot.
   2. MyRRT.py: Creates visualization for kinematic simulations.
   3. RRTUtil.py: This is the main template for simulating trajectories and collision checking.
   4. mathUtils.py: Basic math utility functions.
   5. buildWorldnew.py: To be used for adding trees or fruits to the environment.
    
   The folder `simTests/kinematics` contains wrapper functions for setting up
   the configuration of robots.
   
   simpleWorld.xml contains the information on the robot that is being used and the size of the terrain. I have created my own robot model unicycle.rob by tweaking the already existing sphero.rob file (replacing the sphero.off file with cylinder_y.off file in the sphere.rob) After the robot model is ready we start executing MyRRT.py which is the main file that creates visualization for kinematic simulations. Inside this file RRTplanner method of RRT class is called to implement the algorithm that lies in a separate file called RRTUtil.py. Here in this class for 5000 samples we try to find a path from initial configuration to goal configuration using bidirectional RRT. User defined class called Node serves the purpose of a tree vertex and all the instances that are formed while finding the path are stored in a List data structure.
   
I have used my own collision detection method, as my obstacles are static I have stored their configurations as well as their names in a list so that if the robot collides with the obstacle this method checks if the sum of the radius of the circle or ellipse and the radius of the unicycle is less than the distance between the center of the obstacle and the center of the robot then return true indicating collision, if not less than then it returns false indicating no collision. Also, I should consider the walls (boundaries of the farm) as obstacles so I have checked the robot’s x and y co-ordinates should be less than the x and y co-ordinates of the wall to check if the robot is collision free. 

To check the closeness of two configurations I have used a tolerance of 0.1 for position as well as orientation. To show the simulated motion of the robot from start to goal I have saved all the intermediate configurations in to a list. 
 
