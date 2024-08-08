
# **Redstone Project**

**The University of Texas at Austin**


**Department of Aerospace Engineering**


**Hongseok Kim**


**Last Update: 08/07/24**


-  Please install Satellite Communication Toolbox for running the simulation in MATLAB.  
-  Most of the MATLAB code is designed in MATLAB livescript, and exported pdf of livescript is uploaded in most of sub\-directory for convenience. 
# **Project 1: RS\-LL (Low Level)**

**Design Low\-Level Control Mechanism for LEO Satellite**

## Project Description

  In this project, Low Level control mechanisam and its application for satellite operated in low\-earth orbit are described. 


 **\[ADCS System Design\]** 

-  To operate satellite ADCS (Attitude Determination and Control System), we need satellite orbit propagator and attitude controller. I used J2 perturbation model for orbit propagation and 3\-axis Reaction Wheels for attitude control (RS\-LL\-1 and RS\-LL\-2). 
-  The simulation using these two algorithms is performed in RS\-LL\-3, simulating 1\-orbit simulation pointing one ground point.  
-  The detailed explanation of satellite Reaction wheel control algorim, which is momentum control by PID algorithm in Direction Cosine Matrix (Euler Angle), is described in RS\-HL\-5. 

 **\[Application Algorithm Design\]** 

-  We considered applications of satellite attitude control system, both for earth observation mission and communication mission. 
-  For earth observation mission, we have simulated GSD (Ground Sample Distance) change by changing the off point roll angle for the satellite (RS\-LL\-4). Not only roll angle, this algorithm is valid for any direction of rotation. 
-  For communciation mission, the defining contact time based on elevation angle from Ground Station to satellite is important factor. In RS\-LL\-6, we have calculated the relationship between satellite's maximum elevation angle and contact time for 500km altitude Low\-Earth\-Orbit. 

## Contents of RS\-LL
1.  RS\-LL\-1: Satellite Orbit Propagator
2. RS\-LL\-2: Satellite Attitude Controller
3. RS\-LL\-3: Simulation for Single Satellite
4. RS\-LL\-4: Tilted GSD
5. RS\-LL\-5: Redstone Control Performance
6. RS\-LL\-6: Contact time Elevation Calculator

# **Project 2: RS\-HL (High Level)**

**Satellite Network Routing System Design using MDP with Cooperative Game and Congestion Game**

## **Project Description**

  In this project, communication satellite constellation is simulated via MATLAB satellite communication toolbox, and network structure design usign Markov Decision Process (MDP) is described. Also, simulation of multiple packets browsing the satellite network is performed, using 3 different data packet collision avoidance mechanism. 

## Contents of RS\-HL
1.  RS\-HL\-1: Constellation Scenario Formulation
2. RS\-HL\-2: Generate Contact Chart
3. RS\-HL\-3: Network Graph Generation
4. RS\-HL\-4: Data Transmission MDP (Used Q\-Learning)
5. RS\-HL\-5: MDP\-DP (MDP using Dynamic Programming Algorithm)
6. RS\-HL\-6: Ground Station and Satellite Contact Simulation
7. RS\-HL\-7: MDP\-DP Analysis
8. RS\-HL\-8: MDP Functions (Merged to RS\-HL\-10)
9. RS\-HL\-9: Time Variant MDP
10. RS\-HL\-10: Time Varying MDP functions
11. RS\-HL\-11: Multi User Sequential (Sequential Simulation)
12. RS\-HL\-12: Multi User Parallel (Cooperative Game)
13. RS\-HL\-13: Multi User Default (Not using collosion avoidance algorithm)
14. RS\-HL\-14: Multi User Congestion Game
