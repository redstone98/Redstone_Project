function [Q,reference_attitude] = SAT_Controller_top(input_params)

operation_params.initial_operation_mode = input_params.initial_operation_mode;
operation_params.operation_mode_vec = input_params.operation_mode_vec;
operation_params.r = input_params.r;
operation_params.v = input_params.v;
operation_params.GS_ECI = input_params.GS_ECI;
operation_params.sun_vector = input_params.sun_vector;

% Ru Satellite Reference Attitude Generator
reference_attitude = SAT_Reference_Attitude(input_params.tVec, operation_params);

R.tVec = input_params.tVec;
R.xIstar = reference_attitude.xIstar;
R.zIstar = reference_attitude.zIstar;

S.oversampFact = input_params.oversampFact;
S.state0.r = input_params.r0;
S.state0.v = input_params.v0;
S.state0.e = reference_attitude.e0;
S.state0.omegaB = zeros(3,1);
S.distMat = zeros(input_params.N-1,3);

SATParamsScript;
constantsScript;
P.SATParams = SATParams;
P.constants = constants;

% Run Satellite Control Simulator
Q = simulateSATControl(R,S,P);