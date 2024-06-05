function [t,y] = j2propagation_hs(r0,v0,tVec)

mu = 398600.4415;
odeoptions = odeset('RelTol',1e-10,'AbsTo',1e-20);

% Calculate the magnitude of the position and velocity vectors
y0 = [r0;v0];

% Solve the ODE using ode45
[t, y] = ode45(@j2perturb, tVec, y0, odeoptions);

end