function [NBk] = attitudeController(R,S,P)
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

K = 20*diag([0.15 0.15 0.15]);
K_d = 450*diag([1 1 1]);

zIstar = R.zIstark;
xIstar = R.xIstark;
R_BI = S.statek.RBI;
omegaB = S.statek.omegaB;
quadParams = P.quadParams;


RWAMOI = P.quadParams.RWAMOI;

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


NBk = K * e_E - K_d * omegaB + crossProductEquivalent(omegaB) * J * omegaB+ crossProductEquivalent(omegaB)*RWAMOI* omegaVec;

end


  

