# FinalProject6152 Kinematic simulations using Klampt

### Execution:
```
cd simTests
python MyRRT.py simpleWorld.xml
```

### Robot files

  The folder `mobile_robots` contain the descriptions of the following robots:
1. Unicycle: Differential drive robot with two degrees of freedom.
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
 
