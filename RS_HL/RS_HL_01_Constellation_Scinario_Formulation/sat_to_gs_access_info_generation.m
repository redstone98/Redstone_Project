function [time_vector, ...
          sat_to_gs_access_table, ...
          sat_to_gs_access_matrix, ...
          sat_to_gs_access_info, ...
          contact_1_gs, ...
          contact_2_gs, ...
          contact_3_gs, ...
          contact_4_plus_gs] = sat_to_gs_access_info_generation(scenario,satellites,gs)

 
    number_of_satellites = length(satellites);
    time_vector = scenario.StartTime:seconds(scenario.SampleTime):scenario.StopTime;
    sat_to_gs_access_matrix = zeros(number_of_satellites, length(time_vector));
    sat_to_gs_access_table = [];
    
    for i = 1:number_of_satellites
    sat_to_gs_access_info = access(satellites(i),gs);
    access_single_sat = accessIntervals(sat_to_gs_access_info);
      for j = 1:height(access_single_sat(:,1))
        contact_start_index = (table2array(access_single_sat(j,4)) - scenario.StartTime)/seconds(scenario.SampleTime);
        contact_end_index = (table2array(access_single_sat(j,5)) - scenario.StartTime)/seconds(scenario.SampleTime);
         for k = contact_start_index+1:contact_end_index+1
           sat_to_gs_access_matrix(i,k) =sat_to_gs_access_matrix(i,k) + 1;
         end
      end
      sat_to_gs_access_table = [sat_to_gs_access_table; access_single_sat];
    end
    
    sat_to_gs_access_table;
    
    contact_1_gs = [];
    contact_2_gs = [];
    contact_3_gs = [];
    contact_4_plus_gs = [];
    
    
    for i = 1:length(sat_to_gs_access_matrix(:,1))
      for j = 1:length(sat_to_gs_access_matrix(1,:))
    
        if(sat_to_gs_access_matrix(i,j) == 1)
        contact_1_gs = [contact_1_gs;j,i];
    
        elseif(sat_to_gs_access_matrix(i,j) == 2)
        contact_2_gs = [contact_2_gs;j,i];
    
        elseif(sat_to_gs_access_matrix(i,j) == 3)
        contact_3_gs = [contact_3_gs;j,i];
    
        elseif(sat_to_gs_access_matrix(i,j) >= 4)
        contact_4_plus_gs = [contact_4_plus_gs;j,i];
        end
      end
    end
    
     end