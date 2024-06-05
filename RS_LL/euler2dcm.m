function [R_BW] = euler2dcm(e)
% euler2dcm : Converts Euler angles phi = e(1), theta = e(2), and psi = e(3)
%             (in radians) into a direction cosine matrix for a 3-1-2 rotation.
%
% Let the world (W) and body (B) reference frames be initially aligned.  In a
% 3-1-2 order, rotate B away from W by angles psi (yaw, about the body Z
% axis), phi (roll, about the body X axis), and theta (pitch, about the body Y
% axis).  R_BW can then be used to cast a vector expressed in W coordinates as
% a vector in B coordinates: vB = R_BW * vW
%
% INPUTS
%
% e ---------- 3-by-1 vector containing the Euler angles in radians: phi =
%              e(1), theta = e(2), and psi = e(3)
%
%
% OUTPUTS
%
% R_BW ------- 3-by-3 direction cosine matrix 
% 
%+------------------------------------------------------------------------------+
% References:  Todd Humphreys, Aerial Robotics Course Notes for ASE 379W
%
%
% Author: Hongseok Kim (EID: hk24772)
%+==============================================================================+  

e1 = e(1);
e2 = e(2);
e3 = e(3);

R3_e3 = [cos(e3), sin(e3),  0;
        -sin(e3), cos(e3),  0;
               0,      0,   1];

R1_e1 = [1,       0,         0;
         0, cos(e1),  sin(e1);
         0,-sin(e1),  cos(e1)];

R2_e2 = [cos(e2),0,-sin(e2);
         0      ,1,       0;
         sin(e2),0,cos(e2)];

R_BW = R2_e2 * R1_e1 * R3_e3;


end

  








