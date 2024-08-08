
# **Redstone Project**

**Last Update: 08/07/24**


 **\[Open for Graduate Research Assistant or Post\-Bacc Position\]** 

-  I'm graduating UT Austin Aerospace Engineering in December 2024, applying for Graduate Programs starting from Fall 2025 
-  If you are interested in this project and finding aerospace engineer, please contact me to this email: redstone@utexas.edu 


**Note:**

-  Please install Satellite Communication Toolbox for running the simulation in MATLAB.  
-  Most of the MATLAB code is designed in MATLAB livescript, and exported pdf of livescript is uploaded in most of sub\-directory for convenience. 
-  You need minor directory changes for running several mlx files. 
# **Project 1: RS\-LL (Low Level)**

**Design of Low\-Level Control Mechanism for LEO Satellite**

## Project Description

  In this project, Low Level control mechanisam and its application for satellite operated in low\-earth orbit are described. 


 **\[ADCS System Design\]** 

-  To operate satellite ADCS (Attitude Determination and Control System), we need satellite orbit propagator and attitude controller. I used J2 perturbation model for orbit propagation and 3\-axis Reaction Wheels for attitude control (RS\-LL\-1 and RS\-LL\-2). 
-  The simulation using these two algorithms is performed in RS\-LL\-3, simulating 1\-orbit simulation pointing one ground point.  
-  The detailed explanation of satellite Reaction wheel control algorithm, which is momentum control by PID algorithm in Direction Cosine Matrix (Euler Angle), is described in RS\-HL\-5. 

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

**Satellite Network Routing System Design using Markov Decision Process(MDP) with Cooperative Game and Congestion Game**

## **Project Description**

   In this project, communication satellite constellation is simulated via MATLAB satellite communication toolbox, and network routing algorithm design usign Markov Decision Process (MDP) is described.


  Also, simulation of multiple packets browsing the satellite network is performed, using 3 different data packet collision avoidance algorithms. 


**\[Configuration of Constellation Scenario\]**

-  **Orbit Design and Satellite Allocation \[RS\-HL\-1\]**: Designing the satellite orbits and allocating satellites to specific orbits using keplerian 6 elements and SGP4 orbit propagator.   
-  **Ground Stations Configuration \[RS\-HL\-1\]**: Setting up and configuring ground stations (Latitude, Longitude and Altitude) to communicate with the satellite constellation for data transmission and control. 
-  **Generate Contact 3D Chart Genration \[RS\-HL\-2\]**: Creating a three\-dimensional chart to visualize and analyze contact points and times between satellites and ground stations. 
-  **Network Graph Generation \[RS\-HL\-3\]**:  Developing a network graph to represent the connections and communication links between different satellites and ground stations. 
-  **Analysis of the Contact \[RS\-HL\-6\]**: Evaluating the contact data to understand the contact patterns especially oscillating number of available satellite / ground station of communication.   

**\[Network routing algorithm designed by MDP\]**

-   **MDP simulation using matlab** **`createMDP`** **function \[RS\-HL\-4\]** : Simulation of the Markov Decision Process (MDP) using MATLAB's createMDP function to model and solve decision\-making problems. 
-  **MDP\-DP algorithm description \[RS\-HL\-5\]**: Description the MDP\-Dynamic Programming (DP) algorithm used for optimizing decision\-making processes in the satellite network. 
-  **MDP\-DP algorithm analysis \[RS\-HL\-7\]**: Analysis the performance and results of the MDP\-DP algorithm to assess its effectiveness in network routing. The change of state value function is described. 
-  **Time Varying MDP algorithm description \[RS\-HL\-9\]**: Description the algorithm that accounts for time\-varying changes in the MDP model. 
-  **Time Varying MDP functions \[RS\-HL\-10\]**: Functions created from previous analysis for future application. 

**\[Multi\-Agent Collision Avoidance Algorithm\]**

-  **Default Setting Sim \[RS\-HL\-13\]**: Running simulations with default settings to test the basic functionality of the multi\-agent data packet routing algorithm. 
-  **Seqential Algorithm \[RS\-HL\-11\]**: Implementation of a sequential algorithm where agents make decisions one after another to avoid collisions. 
-  **Cooperative Game \[RS\-HL\-12**\]: Application of cooperative game theory to design strategies where agents work together to avoid collisions and achieve common goals. 
-  **Congestion Game \[RS\-HL\-14\]:** Utilization congestion game theory to model scenarios where multiple agents interact and aim to minimize individual delays caused by congestion. 

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
