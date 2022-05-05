# Missile Simulator

[Simulation Link](https://share.streamlit.io/nbloem26/missilesimulator/main/app.py)

This is an example simulation of a 2D missile engagment. 
The engagement is configurable with initial conditions and a navigation gain. 
This navigation gain is used in the proportional navigation guidance law used by the missile. 
The time listed is the time of flight of the missle. Miss distance is the final point of closest approach. 
Score is evaluated as 100-(miss distance)+(closing velocity).

# Application Walkthrough 
![image](https://user-images.githubusercontent.com/64054813/166867777-e8ab012c-9eb7-494d-a5e1-2d6d4e37d504.png)

Configurable options are in the sidebar for initial positions for the target and the missile. 
The initial missile velocity is always towards the initial target position. 

A dropdown exists to look further at telemetry of the simulation being shown.
Options inlude:
 - X/Y Position
 - X/Y Veloctiy
 - X/Y Acceleration
 - Lateral Acceleration
 - Thrust
 - Speed

The lateral acceleration is shown below: 
![image](https://user-images.githubusercontent.com/64054813/166868049-9b8d6500-03bc-4627-b7c3-dbb668478196.png)
This illustrates the target maneuver is a sinusoidal weave and the missile is using proportional navigation for closed-loop guidance. 

The speed of each object is shown below:
![image](https://user-images.githubusercontent.com/64054813/166868064-e21e48ec-663c-4a4c-a059-3f2b8893d573.png)
The target in this simulation is moving at a constant velocity. 
The missile displays a large increase in acceleration to simulate the rocket motor firing and then a gradual decrease due to drag. 
