function [eak] = voltageConverter(NBk,omegaVec,P)
% voltageConverter : Generates output voltages appropriate for desired
%                    torque
%
%
% INPUTS
%
%
% NBk -------- Commanded 3x1 torque expressed in the body frame at time tk, in
%              N-m.
%
% P ---------- Structure with the following elements:
%
%    SATParams = Structure containing all relevant parameters for the
%                 SAT, as defined in SATParamsScript.m 
%
%     constants = Structure containing constants used in simulation and
%                 control, as defined in constantsScript.m 
%
%
% OUTPUTS
%
% eak -------- Commanded 4x1 voltage vector to be applied at time tk, in
%              volts. eak(i) is the voltage for the ith motor.
%
%+------------------------------------------------------------------------------+
% References:
%
%
% Author: Hongseok Kim
%+==============================================================================+  

% Assume same specification of RWAS
c_m = P.SATParams.cm(1);
tau_m = P.SATParams.taum(1);

RWAMOI = P.SATParams.RWAMOI;


eak = (-(RWAMOI\NBk)*tau_m+omegaVec)/c_m;







