% Since the sourcecode for the satellite dynamics and control comes from
% the quadrotor, we have quadParams values, but suited for satellite
% characteristic

% Mass of Satellite
quadParams.m = 0.78;
% Loads Satellite parameters into the structure quadParams
% The Satellite's moment of inertia, expressed in the body frame
quadParams.Jq = diag([200;100;150]);
% taum(i) is the time constant of the ith rotor, in seconds. This governs how
% quickly the rotor responds to input voltage.
quadParams.taum = (1/20)*ones(4,1);
% cm(i) is the factor used to convert motor voltage to motor angular rate
% in steady state for the ith motor, with units of rad/sec/volt
quadParams.cm = 1*ones(4,1);
% Maximum voltage that can be applied to any motor, in volts
quadParams.eamax = 12;
% Reaction Wheel's Momentum of Inertia
quadParams.RWAMOI = diag([1;1;1]);
%-----------------------------------------------------------------------------

