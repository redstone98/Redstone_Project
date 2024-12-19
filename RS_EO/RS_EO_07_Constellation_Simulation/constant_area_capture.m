function [sat_posvel_info, no_of_sats] = constant_area_capture(scenario, ground_target, elevation_angle_condition)


%% Add Target Ground Point, Add Contact Sequence
gs = groundStation(scenario,"Name",ground_target(:,1), ...
                        "Latitude",str2double(ground_target(:,2)), ...
                        "Longitude", str2double(ground_target(:,3)), ...
                        "Altitude", str2double(ground_target(:,4)), ...
                        "MinElevationAngle", elevation_angle_condition ...
                        );

ground_target_lla = [str2double(ground_target(:,2:4))];
ground_target_ecef = lla2ecef(ground_target_lla);
r_gs = norm(ground_target_ecef);


%% Collect Position/Velocity Information & Calculate Roll Angle

ac = access(scenario.Satellites, gs);

ac.LineColor = 'r';
[satellite_no,timestep] = find(accessStatus(ac) == 1);
contact_info = [satellite_no,timestep];
captured_satellite_no = unique(contact_info(:,1));

accessIntervals(ac);

sat_time_duration = zeros(length(captured_satellite_no),3);
sat_time_duration(:,1) = captured_satellite_no;
satellite_namespace = scenario.Satellites.Name;

% Export Contact End Timing per Satellite

for i = 1:length(captured_satellite_no)
  timestep_index = find(contact_info(:,1) == captured_satellite_no(i));
  timesteps = contact_info(timestep_index,2);
  starting_timestep =  min(timesteps);
  sat_time_duration(i,2) = starting_timestep;
  sat_time_duration(i,3) = (length(timesteps)-1);
end



sat_posvel_info = struct();
no_of_sats = length(captured_satellite_no);




for sat_index = 1:no_of_sats
  sat_posvel_info.(['sat',num2str(sat_index)]).('satellite_name') = satellite_namespace{sat_time_duration(sat_index,1)};
  selected_start_time = scenario.StartTime + seconds(scenario.SampleTime * (sat_time_duration(sat_index,2)-1));
  selected_end_time = selected_start_time + seconds(scenario.SampleTime * sat_time_duration(sat_index,3));
  [r_eci,v_eci] = states(scenario.Satellites(sat_time_duration(sat_index,1)),selected_start_time,'CoordinateFrame','inertial');

  sat_posvel_info.(['sat',num2str(sat_index)]).('start_epoch') = selected_start_time;
  sat_posvel_info.(['sat',num2str(sat_index)]).('end_epoch') = selected_end_time;

  [a,ecc,incl,RAAN,argp,nu] = ijk2keplerian(r_eci,v_eci);

  sat_posvel_info.(['sat',num2str(sat_index)]).('orbit_parameter') = [ecc,incl,RAAN,argp,nu]';

  scenario_temp = satelliteScenario(selected_start_time,selected_end_time,0.1);
  orbit_propagation = satellite(scenario_temp,a,ecc,incl,RAAN,argp,nu,"OrbitPropagator","sgp4");
  [r_ecef_prop, v_ecef_prop, t_prop] = states(orbit_propagation,'CoordinateFrame','ecef');

  r_ecef_prop = r_ecef_prop';
  v_ecef_prop = v_ecef_prop';
  t_prop = t_prop';

  sat_posvel_info.(['sat',num2str(sat_index)]).('r_ecef') = r_ecef_prop;
  sat_posvel_info.(['sat',num2str(sat_index)]).('v_ecef') = v_ecef_prop;
  sat_posvel_info.(['sat',num2str(sat_index)]).('time_vector') = t_prop;

  range_vector = zeros(length(t_prop),1);
  for prop_index = 1:length(t_prop)
  range_vector(prop_index) = norm(r_ecef_prop(prop_index,:) - ground_target_ecef);
  end

  sat_posvel_info.(['sat',num2str(sat_index)]).('range_vector') = range_vector;
  minimum_range = min(range_vector);
  index_for_minimum_range = find(range_vector == minimum_range);
  sat_posvel_info.(['sat',num2str(sat_index)]).('minimum_range_km') =  minimum_range/1000; 
  sat_posvel_info.(['sat',num2str(sat_index)]).('time_of_minimum_range') = t_prop(index_for_minimum_range);
  r_sat = norm(r_ecef_prop(index_for_minimum_range,:));
  roll_angle = acos((r_sat^2+minimum_range^2-r_gs^2)/(2*minimum_range*r_sat))/pi*180;


  lla_of_min_range = ecef2lla(r_ecef_prop(index_for_minimum_range,:));

  sat_posvel_info.(['sat',num2str(sat_index)]).('ground_target_lla') = ground_target_lla;
  sat_posvel_info.(['sat',num2str(sat_index)]).('lla_of_minimum_range') = lla_of_min_range;


  vel_of_min_range = v_ecef_prop(index_for_minimum_range,:);
  sat_posvel_info.(['sat',num2str(sat_index)]).('vel_of_minimum_range') = vel_of_min_range;

  if vel_of_min_range(3) * (lla_of_min_range(2) - ground_target_lla(2)) > 0
    roll_angle = - roll_angle;
  end
  sat_posvel_info.(['sat',num2str(sat_index)]).('roll_angle') = roll_angle;
end


%% Calculate the Center and End Points
for sat_index = 1:no_of_sats
  data_length = length(sat_posvel_info.(['sat',num2str(sat_index)]).('time_vector'));

  lla_center = zeros(data_length,3);
  lla_end1 = zeros(data_length,3);
  lla_end2 = zeros(data_length,3);

  r_ecef_input_vector = sat_posvel_info.(['sat',num2str(sat_index)]).('r_ecef');
  v_ecef_input_vector = sat_posvel_info.(['sat',num2str(sat_index)]).('v_ecef');
  roll_angle_initial =  sat_posvel_info.(['sat',num2str(sat_index)]).('roll_angle');

  for index = 1:data_length

        r_ecef_input = r_ecef_input_vector(index,:);
        v_ecef_input = v_ecef_input_vector(index,:);
       [center,end2,end1] =  ground_pointing_from_tilt_angle(r_ecef_input, v_ecef_input, roll_angle_initial);

       lla_center(index,:) = center;
       lla_end1(index,:) = end1;
       lla_end2(index,:) = end2;

  end

       sat_posvel_info.(['sat',num2str(sat_index)]).('lla_sat') = ecef2lla(r_ecef_input_vector);
       sat_posvel_info.(['sat',num2str(sat_index)]).('lla_center') = lla_center;
       sat_posvel_info.(['sat',num2str(sat_index)]).('lla_end1') = lla_end1;
       sat_posvel_info.(['sat',num2str(sat_index)]).('lla_end2') = lla_end2;

end
end