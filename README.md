
# **Redstone Project**

**The University of Texas at Austin**


**Department of Aerospace Engineering**


**Hongseok Kim**


**Last Update: 08/07/24**


-  Please install Satellite Communication Toolbox for running the simulation in MATLAB.  
# **RS\-LL**

**Design Low\-Level Control Mechanism for LEO Satellite**

## Project Description

  In this project, I Designed Low Level control mechanisam for satellite operated in low\-earth orbit. 


  \[ADCS System Design\]

-  To operate satellite ADCS (Attitude Determination and Control System), we need satellite orbit propagator and attitude controller. I used J2 perturbation model for orbit propagation and 3\-axis Reaction Wheels for attitude control (RS\-LL\-1 and RS\-LL\-2). 
-  The simulation using these two algorithms is performed in RS\-LL\-3, simulating 1\-orbit simulation pointing one ground point.  
-  The detailed explanation of satellite Reaction wheel control algorim, which is momentum control by PID algorithm in Direction Cosine Matrix (Euler Angle), is described in RS\-HL\-5. 

  \[Application Algorithm Design\]

-  We considered applications of satellite attitude control system, both for earth observation mission and communication mission. 
-  For earth observation mission, we have simulated GSD (Ground Sample Distance) change by changing the off point roll angle for the satellite (RS\-LL\-4). Not only roll angle, this algorithm is valid for any direction of rotation. 
-  For communciation mission, the defining contact time based on elevation angle from Ground Station to satellite is important factor. In RS\-LL\-6, we have calculated the relationship between satellite's maximum elevation angle and contact time for 500km altitude Low\-Earth\-Orbit. 

## Contents of the Package
1.  RS\-LL\-1: Satellite Orbit Propagator
2. RS\-LL\-2: Satellite Attitude Controller
3. RS\-LL\-3: Simulation for Single Satellite
4. RS\-LL\-4: Tilted GSD
5. RS\-LL\-5: Redstone Control Performance
6. RS\-LL\-6: Contact time Elevation Calculator

# **RS\-HL**

**Satellite Network Routing System Design using MDP with Cooperative Game and Congestion Game**

## **Project Description**

 **\-** 

