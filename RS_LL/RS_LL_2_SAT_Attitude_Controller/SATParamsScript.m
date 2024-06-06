
% Mass of Satellite
SATParams.m = 0.78;
% Loads Satellite parameters into the structure SATParams
% The Satellite's moment of inertia, expressed in the body frame
SATParams.Jq = diag([200;100;150]);
% taum(i) is the time constant of the ith rotor, in seconds. This governs how
% quickly the rotor responds to input voltage.
SATParams.taum = (1/20)*ones(4,1);
% cm(i) is the factor used to convert motor voltage to motor angular rate
% in steady state for the ith motor, with units of rad/sec/volt
SATParams.cm = 1*ones(4,1);
% Maximum voltage that can be applied to any motor, in volts
SATParams.eamax = 12;
% Reaction Wheel's Momentum of Inertia
SATParams.RWAMOI = diag([1;1;1]);
% Reaction Wheel's PID gain
SATParams.k_p = 20*diag([1 1 1]);
SATParams.k_i = 1*diag([1 1 1]);
SATParams.k_d = 450*diag([1 1 1]);

%-----------------------------------------------------------------------------

