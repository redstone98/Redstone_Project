function dydt = j2perturb(t, y)


    J2 = 0.00108248;
    % Define the standard gravitational parameter for Earth (km^3/s^2)
    mu = 398600.4415;
    R_Earth = 6378.1363;

    % Extract position and velocity from the input state vector
    r_vec = y(1:3); % Position vector
    v = y(4:6); % Velocity vector
    
    % Calculate the norm of the position vector
    r_norm = norm(r_vec);
    
    % Compute the acceleration based on the gravitational force
    a1 = -mu*r_vec(1)/(r_norm^3) * (1-J2*3/2*(R_Earth/r_norm)^2*(5*(r_vec(3)/r_norm)^2-1));
    a2 = -mu*r_vec(2)/(r_norm^3) * (1-J2*3/2*(R_Earth/r_norm)^2*(5*(r_vec(3)/r_norm)^2-1));
    a3 = -mu*r_vec(3)/(r_norm^3) * (1-J2*3/2*(R_Earth/r_norm)^2*(5*(r_vec(3)/r_norm)^2-3));

    
    % Combine velocity and acceleration to form the derivative of the state vector
    dydt = [v; a1;a2;a3];
    
end
