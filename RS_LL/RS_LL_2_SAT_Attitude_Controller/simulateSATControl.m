function [Q] = simulateSATControl(R,S,P)
% simulateSATrotorControl : Simulates closed-loop control of a satellite.
%
%
% INPUTS
%
% R ---------- Structure with the following elements:
%
%          tVec = Nx1 vector of uniformly-sampled time offsets from the
%                 initial time, in seconds, with tVec(1) = 0.
%
%        rIstar = Nx3 matrix of desired CM positions in the I frame, in
%                 meters.  rIstar(k,:)' is the 3x1 position at time tk =
%                 tVec(k).
%
%        vIstar = Nx3 matrix of desired CM velocities with respect to the I
%                 frame and expressed in the I frame, in meters/sec.
%                 vIstar(k,:)' is the 3x1 velocity at time tk = tVec(k).
%
%        aIstar = Nx3 matrix of desired CM accelerations with respect to the I
%                 frame and expressed in the I frame, in meters/sec^2.
%                 aIstar(k,:)' is the 3x1 acceleration at time tk =
%                 tVec(k).
%
%        xIstar = Nx3 matrix of desired body x-axis direction, expressed as a
%                 unit vector in the I frame. xIstar(k,:)' is the 3x1
%                 direction at time tk = tVec(k).
%  
% S ---------- Structure with the following elements:
%
%  oversampFact = Oversampling factor. Let dtIn = R.tVec(2) - R.tVec(1). Then
%                 the output sample interval will be dtOut =
%                 dtIn/oversampFact. Must satisfy oversampFact >= 1.
%
%        state0 = State of the SAT at R.tVec(1) = 0, expressed as a structure
%                 with the following elements:
%                   
%                   r = 3x1 position in the world frame, in meters
% 
%                   e = 3x1 vector of Euler angles, in radians, indicating the
%                       attitude
%
%                   v = 3x1 velocity with respeplay(scenario)ct to the world frame and
%                       expressed in the world frame, in meters per second.
%                 
%              omegaB = 3x1 angular rate vector expressed in the body frame,
%                       in radians per second.
%
%       distMat = (N-1)x3 matrix of disturbance forces acting on the SAT's
%                 center of mass, expressed in Newtons in the world frame.
%                 distMat(k,:)' is the constant (zero-order-hold) 3x1
%                 disturbance vector acting on the SAT from R.tVec(k) to
%                 R.tVec(k+1).
%
% P ---------- Structure with the following elements:
%
%    SATParams = Structure containing all relevant parameters for the
%                 SAT, as defined in SATParamsScript.m 
%
%     constants = Structure containing constants used in simulation and
%                 control, as defined in constantsScript.m 
%
%  sensorParams = Structure containing sensor parameters, as defined in
%                 sensorParamsScript.m
%
%
% OUTPUTS
%
% Q ---------- Structure with the following elements:
%
%          tVec = Mx1 vector of output sample time points, in seconds, where
%                 Q.tVec(1) = R.tVec(1), Q.tVec(M) = R.tVec(N), and M =
%                 (N-1)*oversampFact + 1.
%  
%         state = State of the SAT at times in tVec, expressed as a
%                 structure with the following elements:
%                   
%                rMat = Mx3 matrix composed such that rMat(k,:)' is the 3x1
%                       position at tVec(k) in the I frame, in meters.
% 
%                eMat = Mx3 matrix composed such that eMat(k,:)' is the 3x1
%                       vector of Euler angles at tVec(k), in radians,
%                       indicating the attitude.
%
%                vMat = Mx3 matrix composed such that vMat(k,:)' is the 3x1
%                       velocity at tVec(k) with respect to the I frame
%                       and expressed in the I frame, in meters per
%                       second.
%                 
%           omegaBMat = Mx3 matrix composed such that omegaBMat(k,:)' is the
%                       3x1 angular rate vector expressed in the body frame in
%                       radians, that applies at tVec(k).
%
%+------------------------------------------------------------------------------+
% References:
%
%
% Author: Hongseok Kim,University of Texas at Austin
%+==============================================================================+  

tVecIn = R.tVec;

zIstar = R.zIstar;
xIstar = R.xIstar;

oversampFact = S.oversampFact;
r0 = S.state0.r;
e0 = S.state0.e;
v0 = S.state0.v;
omegaB0 = S.state0.omegaB;

omegaVec0 = zeros(3,1);

distMat = S.distMat;

SATParams = P.SATParams;
constants = P.constants;

dtIn = tVecIn(2) - tVecIn(1);
dtOut = dtIn/oversampFact;

N = length(tVecIn);
M = (N-1)*oversampFact+1;

tVec = zeros(M,1);
rMat = zeros(M,3);
eMat = zeros(M,3);
vMat = zeros(M,3);
hMat = zeros(M,3);
omegaBMat =  zeros(M,3);
omegaVecMat = zeros(M,3);

NBMat = zeros(N,3);
eaMat = zeros(N,3);

xIMat = zeros(M,3);
yIMat = zeros(M,3);
zIMat = zeros(M,3);

rMat(1,:) = r0';
vMat(1,:) = v0';
eMat(1,:) = e0';
omegaBMat(1,:) = omegaB0'; 
omegaVecMat(1,:) = omegaVec0';

e_E_int_Mat = zeros(M,3);

tVec(1) = R.tVec(1);

for k = 1:N-1
    tspan = [tVecIn(k):dtOut:tVecIn(k+1)]';
    

    x_I_star_k = xIstar(k,:)';
    z_I_star_k = zIstar(k,:)';
    distMat_k = distMat(k,:)';

    k_step = (k-1)*oversampFact + 1;

    r_k = rMat(k_step,:)';

    e_k = eMat(k_step,:)';
    RBI_k = euler2dcm(e_k);

    v_k = vMat(k_step,:)';
    omegaB_k = omegaBMat(k_step,:)';
    omegaVec_k = omegaVecMat(k_step,:)';
    e_E_int_k = e_E_int_Mat(k_step,:)';

    R_attitude.zIstark = z_I_star_k;
    R_attitude.xIstark = x_I_star_k;


    S_attitude.statek.RBI = RBI_k;
    S_attitude.statek.omegaB = omegaB_k;
    S_attitude.statek.omegaVec = omegaVec_k;
    S_attitude.statek.e_E_int = e_E_int_k;


    P_attitude.SATParams = SATParams;

    [NBk,e_E] = attitudeController(R_attitude,S_attitude,P_attitude);
    NBMat(k+1,:) = NBk;
    P_voltage.SATParams = SATParams;
    P_voltage.constants = constants;

    eak = voltageConverterSAT(NBk,omegaVec_k,P_voltage);

    eaMat(k+1,:) = eak;

    Parameters.SATParams = SATParams;
    Parameters.constants = constants;

    Xk = [r_k;v_k;RBI_k(:);omegaB_k;omegaVec_k;e_E_int_k];
    
    [tVeck, xMatk] = ode45(@(t,X) SATOdeFunctionHF(t,X,eak,distMat_k,e_E,Parameters),tspan,Xk);
    
    R_BI_data = xMatk(:,7:15);
    eMat_temp = zeros(length(tVeck),3);
    xIMat_temp = zeros(length(tVeck),3);
    yIMat_temp = zeros(length(tVeck),3);
    zIMat_temp = zeros(length(tVeck),3);

    for i = 1:length(tVeck)
    R_BI_temp = zeros(3,3);
    R_BI_temp(:) = R_BI_data(i,:);
    eMat_temp(i,:) = dcm2euler(R_BI_temp);
    xIMat_temp(i,:) = R_BI_temp' * [1;0;0];
    yIMat_temp(i,:) = R_BI_temp' * [0;1;0];
    zIMat_temp(i,:) = R_BI_temp' * [0;0;1]; 
    end


    rMatk = xMatk(:,1:3);
    vMatk = xMatk(:,4:6);
    hMatk = cross(rMatk, vMatk);
    eMatk = eMat_temp;
    xIMatk = xIMat_temp;
    yIMatk = yIMat_temp;
    zIMatk = zIMat_temp;

    omegaBMatk =  xMatk(:,16:18);
    omegaVeck = xMatk(:,19:21);
    e_E_int_Matk = xMatk(:,22:24);

    tVec(k_step:k_step+oversampFact,:) = tVeck;
    rMat(k_step:k_step+oversampFact,:) = rMatk;
    vMat(k_step:k_step+oversampFact,:) = vMatk;
    hMat(k_step:k_step+oversampFact,:) = hMatk;
    eMat(k_step:k_step+oversampFact,:) = eMatk;
    xIMat(k_step:k_step+oversampFact,:) = xIMatk;
    yIMat(k_step:k_step+oversampFact,:) = yIMatk;
    zIMat(k_step:k_step+oversampFact,:) = zIMatk;

    omegaBMat(k_step:k_step+oversampFact,:) = omegaBMatk;      
    omegaVecMat(k_step:k_step+oversampFact,:) = omegaVeck; 
    e_E_int_Mat(k_step:k_step+oversampFact,:) = e_E_int_Matk;

end

Q.tVec = tVec;
Q.state.rMat = rMat;
Q.state.eMat = eMat;
Q.state.vMat = vMat;
Q.state.hMat = hMat;
Q.state.omegaBMat = omegaBMat;
Q.state.omegaVecMat = omegaVecMat;
Q.state.xIMat = xIMat;
Q.state.yIMat = yIMat;
Q.state.zIMat = zIMat;
Q.state.NBMat = NBMat;
Q.state.eaMat = eaMat;
end