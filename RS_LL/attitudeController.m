function [NBk, e_E] = attitudeController(R,S,P)
% attitudeController : Controls quadcopter toward a reference attitude
%
%
% INPUTS
%
% R ---------- Structure with the following elements:
%
%       zIstark = 3x1 desired body z-axis direction at time tk, expressed as a
%                 unit vector in the I frame.
%
%       xIstark = 3x1 desired body x-axis direction, expressed as a
%                 unit vector in the I frame.
%
% S ---------- Structure with the following elements:
%
%        statek = State of the quad at tk, expressed as a structure with the
%                 following elements:
%                   
%                  rI = 3x1 position in the I frame, in meters
% 
%                 RBI = 3x3 direction cosine matrix indicating the
%                       attitude
%
%                  vI = 3x1 velocity with respect to the I frame and
%                       expressed in the I frame, in meters per second.
%                 
%              omegaB = 3x1 angular rate vector expressed in the body frame,
%                       in radians per second.
%
% P ---------- Structure with the following elements:
%
%    quadParams = Structure containing all relevant parameters for the
%                 quad, as defined in quadParamsScript.m 
%
%     constants = Structure containing constants used in simulation and
%                 control, as defined in constantsScript.m 
%
%
% OUTPUTS
%
% NBk -------- Commanded 3x1 torque expressed in the body frame at time tk, in
%              N-m.
%
%+------------------------------------------------------------------------------+
% References:
%
%
% Author: Hongseok Kim
%+==============================================================================+  

K = 20*diag([1 1 1]);
K_d = 450*diag([1 1 1]);
K_I = 1*diag([1 1 1]);

zIstar = R.zIstark;
xIstar = R.xIstark;
R_BI = S.statek.RBI;
omegaB = S.statek.omegaB;
e_E_int = S.statek.e_E_int;

quadParams = P.quadParams;



RWAMOI = P.quadParams.RWAMOI;
omegaVec = S.statek.omegaVec;

J = quadParams.Jq;

if cross(zIstar,xIstar) == 0
    b = cross(zIstar,xIstar);
else
    b = cross(zIstar,xIstar)/norm(cross(zIstar,xIstar));
end


a = cross(b,zIstar);

R_BI_star = [a,b,zIstar]';

R_E = R_BI_star * R_BI';




e_E = [R_E(2,3) - R_E(3,2);
       R_E(3,1) - R_E(1,3);
       R_E(1,2) - R_E(2,1)];

% phi = e_E(1);
% theta = e_E(2);
R_E_dot = R_E * crossProductEquivalent(omegaB);
e_E_dot = [R_E_dot(2,3) - R_E_dot(3,2);
           R_E_dot(3,1) - R_E_dot(1,3);
           R_E_dot(1,2) - R_E_dot(2,1)];

% S = [cos(phi)*cos(theta), 0, cos(phi)*sin(theta);
%      sin(phi)*sin(theta), cos(phi), -cos(theta)*sin(phi);
%      -sin(theta), 0, cos(theta)];
% 
% e_E_dot = -omegaB;

%  + crossProductEquivalent(omegaB) * J * omegaB + crossProductEquivalent(omegaB)*RWAMOI* omegaVec
NBk = K * e_E + K_I * e_E_int + K_d * e_E_dot;
%
end


  

