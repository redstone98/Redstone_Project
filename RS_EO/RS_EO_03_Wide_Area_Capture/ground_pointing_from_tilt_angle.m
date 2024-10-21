function [center,end1,end2] =  ground_pointing_from_tilt_angle(r_ecef, v_ecef, roll_angle)

    x = r_ecef(1)/1000;
    y = r_ecef(2)/1000;
    z = r_ecef(3)/1000; % Position of Satellite (km)
    
    PS_0 = [v_ecef(1) v_ecef(2) v_ecef(3)]; % Satellite Moving Direction
    phi = roll_angle; % Roll Angle (Degree) -  direction (Left: +, Right: -)
    
    lla_info = ecef2lla(r_ecef);
    

    h = lla_info(3)/1000;
    r = norm(r_ecef)/1000 - h;
    % r = 6378; % Radius of Earth (km)
    % h = sqrt(x^2+y^2+z^2)-r; % Satellite Height (km)
    % 

    

    phi = phi* (pi()/180); % Rolling Angle Direction sign(+/-) modification (Left:-, Right:+)
    PQ = 1/(sqrt(x^2+y^2)) * [-y x 0];
    PR = 1/(sqrt(x^2+y^2+z^2)) * cross([x y z], PQ);
    PS = PS_0/norm(PS_0);
    
    if PS(3)>=0 %Satellite Moves north direction
        alpha = acos(dot(PQ,PS));
    else %Satellite Moves south direction
        alpha = -acos(dot(PQ,PS));
    end
    
    PD = PQ * cos(alpha + pi()/2) + PR * sin((alpha+pi()/2));
    PC = - PD;

    % % Camera Lense Specification
    % IQ = 2.2528/2; % 4096 pixels * 5.5 micro M(cm)
    % f = 58; %Focal Length (cm)
    % theta = atan(IQ/f); %Half of Camera View Angle (rad)
    % 

    fov = 10;
    theta = fov/180*pi/2;

    % Calculating OJ,gamma: Center Point
    if phi >= 0
        delta = pi() - asin((h+r)/r * sin(phi));
        gamma_abs = pi() - phi - delta;
        gamma = gamma_abs; % gamma >= 0
    else
        delta = pi() - asin(sin(-phi)*(h+r)/r);
        gamma_abs = pi() - (-phi) - delta;
        gamma = - gamma_abs; % Gamma < 0
    end
    
    % Calculating OP, alpha: End Point 1
    if phi+theta>=0
        beta = pi() - asin((h+r)/r * sin(phi+theta));
        alpha_abs = pi() - beta - (phi+theta);
        alpha = alpha_abs;
    else
        beta = pi() - asin((h+r)/r* sin(-(phi+theta)));
        alpha_abs = pi() - beta - (-(phi+theta));
        alpha = -alpha_abs;
    end
    
    % Calculating OQ, zeta: End Point 2
    if phi-theta>=0
        epsilon = pi() - asin((h+r)/r * sin(phi-theta));
        zeta_abs = pi() - epsilon - (phi-theta);
        zeta = zeta_abs;
    else
        epsilon = pi() - asin((h+r)/r * sin(-(phi-theta)));
        zeta_abs = pi() - epsilon - (-(phi-theta));
        zeta = -zeta_abs;
    end
    
    
    % SAT = [x y z]; % Location of Satellite
    OH = [x y z]./(h+r)*r; % Location of Land point of Satellite
    FC = PC;
     
    OJ = OH * cos(gamma) + r * FC * sin(gamma); %Middle Point
    OP = OH * cos(alpha) + r * FC * sin(alpha); % End Point 1
    OQ = OH * cos(zeta) + r * FC * sin(zeta); % End Point 2
    
    % SAT % Location of Satellite
    % OH % Location of Land point of Satellite (km)

    center = ecef2lla(OJ*1000);  %Middle Point of the GroundView (km)
    end1 = ecef2lla(OP*1000); % End Point 1 (km)
    end2 = ecef2lla(OQ*1000); % End Point 2 (km)
    % norm(OP-OQ)

end