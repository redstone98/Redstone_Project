n = length(tVec);
xIstar = zeros(n,3);
yIstar = zeros(n,3);
zIstar = zeros(n,3);
e_star = zeros(n,3);
%% Define Satellite Operation Mode

for i = 1:n

% NADIR Pointing Mode
if operation_mode_vec(i) == 1
      zIstar(i,:) = -r(i,:)/norm(r(i,:));
      xIstar(i,:) = v(i,:)/norm(v(i,:));
      yIstar(i,:) = cross(zIstar(i,:),xIstar(i,:));
      
      %Generate Reference Euler Angle
          if cross( zIstar(i,:),xIstar(i,:))==0
          b_vec = cross(zIstar(i,:),xIstar(i,:));
          else
          b_vec = cross(zIstar(i,:),xIstar(i,:))/norm(cross(zIstar(i,:),xIstar(i,:))); 
          end
          a_vec = cross(b_vec,zIstar(i,:));
      R_BI_star_temp = [a_vec',b_vec',zIstar(i,:)']';
      e_star(i,:) = dcm2euler(R_BI_star_temp)';
end


% Ground Station Contact Mode
if operation_mode_vec(i) == 2
        zIstar(i,:) = (GS_ECI(i,:) - r(i,:))/norm((GS_ECI(i,:) - r(i,:)));
        yIstar(i,:) = cross(zIstar(i,:),v(i,:))/norm( cross(zIstar(i,:),v(i,:)));
        xIstar(i,:) = cross(yIstar(i,:),zIstar(i,:));

     %Generate Reference Euler Angle
          if cross( zIstar(i,:),xIstar(i,:))==0
          b_vec = cross(zIstar(i,:),xIstar(i,:));
          else
          b_vec = cross(zIstar(i,:),xIstar(i,:))/norm(cross(zIstar(i,:),xIstar(i,:))); 
          end
          a_vec = cross(b_vec,zIstar(i,:));
      R_BI_star_temp = [a_vec',b_vec',zIstar(i,:)']';
      e_star(i,:) = dcm2euler(R_BI_star_temp)';
end


% Sun Pointing Mode
if operation_mode_vec(i) == 3
       yIstar(i,:) = -sun_vector;
       zIstar(i,:) = cross(v(i,:),yIstar(i,:))/norm( cross(v(i,:),yIstar(i,:)));
       xIstar(i,:) = cross(yIstar(i,:),zIstar(i,:));


     %Generate Reference Euler Angle
          if cross( zIstar(i,:),xIstar(i,:))==0
          b_vec = cross(zIstar(i,:),xIstar(i,:));
          else
          b_vec = cross(zIstar(i,:),xIstar(i,:))/norm(cross(zIstar(i,:),xIstar(i,:))); 
          end
          a_vec = cross(b_vec,zIstar(i,:));
      R_BI_star_temp = [a_vec',b_vec',zIstar(i,:)']';
      e_star(i,:) = dcm2euler(R_BI_star_temp)';
end

end

%% Define Initial Operation Mode


% Initial NADIR Pointing Mode
if initial_operation_mode == 1
      z_I_star_0 =  -(r(1,:)/norm(r(1,:)))';
      x_I_star_0 = (v(1,:)/norm(v(1,:)))';
      
      %Generate Reference Euler Angle
          if cross(z_I_star_0,x_I_star_0)==0
          b_vec = cross(z_I_star_0,x_I_star_0);
          else
          b_vec = cross(z_I_star_0,x_I_star_0)/norm(cross(z_I_star_0,x_I_star_0)); 
          end
          a_vec = cross(b_vec,z_I_star_0);
      R_BI_0 = [a_vec,b_vec,z_I_star_0]';
      e0 = dcm2euler(R_BI_0)';
end


% Ground Station Contact Mode
slant_range = zeros(n,1);
if initial_operation_mode == 2
        z_I_star_0 = (Austin_ECI(1,:) - r(1,:))/norm((Austin_ECI(1,:) - r(1,:)));
        y_I_star_0  = cross(z_I_star_0 ,v(1,:))/norm(cross(z_I_star_0 ,v(1,:)));
        x_I_star_0 = cross(y_I_star_0 ,z_I_star_0);

     %Generate Reference Euler Angle
          if cross( z_I_star_0,x_I_star_0)==0
          b_vec = cross(z_I_star_0,x_I_star_0);
          else
          b_vec = cross(z_I_star_0,x_I_star_0)/norm(cross(z_I_star_0,x_I_star_0)); 
          end
          a_vec = cross(b_vec,z_I_star_0);
      R_BI_0 = [a_vec',b_vec',z_I_star_0']';
      e0 = dcm2euler(R_BI_0)';
end


% Sun Pointing Mode
if initial_operation_mode == 3
       y_I_star_0 = -sun_vector;
       z_I_star_0 = cross(v(1,:),y_I_star_0)/norm( cross(v(1,:),y_I_star_0));
       x_I_star_0 = cross(y_I_star_0 ,z_I_star_0);


     %Generate Reference Euler Angle
          if cross( z_I_star_0,x_I_star_0)==0
          b_vec = cross(z_I_star_0,x_I_star_0);
          else
          b_vec = cross(z_I_star_0,x_I_star_0)/norm(cross(z_I_star_0,x_I_star_0)); 
          end
          a_vec = cross(b_vec,z_I_star_0);
      R_BI_0 = [a_vec',b_vec',z_I_star_0']';
      e0 = dcm2euler(R_BI_0)';
end



R.tVec = tVec;
R.xIstar = xIstar;
R.zIstar = zIstar;

S.oversampFact = oversampFact;
S.state0.r = r0;
S.state0.v = v0;
S.state0.e = e0;
S.state0.omegaB = zeros(3,1);
S.distMat = zeros(N-1,3);

quadParamsScript;
constantsScript;
P.quadParams = quadParams;
P.constants = constants;

% Run Satellite Control Simulator
Q = simulateSATControl(R,S,P);