function [Xdot] = SATOdeFunctionHF(t,X,eaVec,distVec,e_E,P)
% quadOdeFunctionHF : Ordinary differential equation function that models
%                     Satellite dynamics -- high-fidelity version.  For use
%                     with one of Matlab's ODE solvers (e.g., ode45).
%
%
% INPUTS
%
% t ---------- Scalar time input, as required by Matlab's ODE function
%              format.
%
% X ---------- Nx-by-1 SAT state, arranged as 
%
%              X = [rI',vI',RBI(1,1),RBI(2,1),...,RBI(2,3),RBI(3,3),...
%                   omegaB',omegaVec']'
%
%              rI = 3x1 position vector in I in meters
%              vI = 3x1 velocity vector wrt I and in I, in meters/sec
%             RBI = 3x3 attitude matrix from I to B frame
%          omegaB = 3x1 angular rate vector of body wrt I, expressed in B
%                   in rad/sec
%        omegaVec = 4x1 vector of rotor angular rates, in rad/sec.
%                   omegaVec(i) is the angular rate of the ith rotor.
%        e_E_int
%
%    eaVec --- 4x1 vector of voltages applied to motors, in volts.  eaVec(i)
%              is the constant voltage setpoint for the ith rotor.
%
%  distVec --- 3x1 vector of constant disturbance forces acting on the SAT's
%              center of mass, expressed in Newtons in I.
%
% P ---------- Structure with the following elements:
%
%    quadParams = Structure containing all relevant parameters for the
%                 quad, as defined in quadParamsScript.m 
%
%     constants = Structure containing constants used in simulation and
%                 control, as defined in constantsScript.m 
%
% OUTPUTS
%
% Xdot ------- Nx-by-1 time derivative of the input vector X
%
%+------------------------------------------------------------------------------+
% References:
%
%
% Author: Hongseok Kim
%+==============================================================================+


r_I = [X(1);X(2);X(3)];
v_I = [X(4);X(5);X(6)];



R_BI = zeros(3,3);

R_BI(:) = X(7:15);

w_B = [X(16);X(17);X(18)];

omegaVec = [X(19);X(20);X(21)];

g = P.constants.g;
m = P.quadParams.m;
J = P.quadParams.Jq;
RWAMOI = P.quadParams.RWAMOI;

tau_m = P.quadParams.taum(1);
c_m = P.quadParams.cm(1);


mu = 398600.4415;
J2 = 0.00108248;
R_Earth = 6378.1363;


N_B_vec = - RWAMOI * (-1 * omegaVec ./tau_m + c_m .* eaVec./tau_m);

r_I_dot = v_I;

r_norm = norm(r_I);

a1 = -mu*r_I(1)/(r_norm^3) * (1-J2*3/2*(R_Earth/r_norm)^2*(5*(r_I(3)/r_norm)^2-1)) + distVec(1)/m;
a2 = -mu*r_I(2)/(r_norm^3) * (1-J2*3/2*(R_Earth/r_norm)^2*(5*(r_I(3)/r_norm)^2-1)) + distVec(2)/m;
a3 = -mu*r_I(3)/(r_norm^3) * (1-J2*3/2*(R_Earth/r_norm)^2*(5*(r_I(3)/r_norm)^2-3)) + distVec(3)/m;


v_I_dot = [a1;a2;a3];



w_B_dot = J\(N_B_vec - crossProductEquivalent(w_B) * J * w_B - crossProductEquivalent(w_B) * RWAMOI * omegaVec);

omegaVec_dot = -1 * omegaVec ./tau_m + c_m .* eaVec./tau_m;

R_BI_dot = - crossProductEquivalent(w_B) *R_BI;

Xdot = zeros(24,1);
Xdot(1:3) = r_I_dot;
Xdot(4:6) = v_I_dot;
Xdot(7:15) = R_BI_dot(:);
Xdot(16:18) = w_B_dot;
Xdot(19:21) = omegaVec_dot;
Xdot(22:24) = e_E;

end

