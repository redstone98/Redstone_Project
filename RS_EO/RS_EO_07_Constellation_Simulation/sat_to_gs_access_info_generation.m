function [time_vector, ...
          sat_to_gs_access_table, ...
          sat_to_gs_access_matrix, ...
          sat_data_generation_matrix, ...
          sat_to_gs_access_info] = sat_to_gs_access_info_generation(scenario,satellites,gs)

 
    number_of_satellites = length(satellites);
    time_vector = scenario.StartTime:seconds(scenario.SampleTime):scenario.StopTime;
    sat_to_gs_access_matrix = zeros(number_of_satellites, length(time_vector));
    sat_data_generation_matrix = zeros(number_of_satellites, length(time_vector));
    sat_to_gs_access_table = [];
    
    for i = 1:number_of_satellites
    sat_to_gs_access_info = access(satellites(i),gs);
    access_single_sat = accessIntervals(sat_to_gs_access_info);
      for j = 1:height(access_single_sat(:,1))
        contact_start_index = (table2array(access_single_sat(j,4)) - scenario.StartTime)/seconds(scenario.SampleTime);
        contact_end_index = (table2array(access_single_sat(j,5)) - scenario.StartTime)/seconds(scenario.SampleTime);
         for k = contact_start_index+1:contact_end_index+1
           sat_to_gs_access_matrix(i,k) = sat_to_gs_access_matrix(i,k) + 1;
         end
          sat_data_generation_matrix(i,contact_end_index+1) = 1;
      end
      sat_to_gs_access_table = [sat_to_gs_access_table; access_single_sat];
    end
    sat_to_gs_access_table;
   end