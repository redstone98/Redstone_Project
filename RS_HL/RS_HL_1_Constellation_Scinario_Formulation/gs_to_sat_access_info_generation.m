function [time_vector, ...
          gs_to_sat_access_table, ...
          gs_to_sat_access_matrix, ...
          gs_to_sat_access_info, ...
          contact_1_sat, ...
          contact_2_sat, ...
          contact_3_sat, ...
          contact_4_plus_sat] = gs_to_sat_access_info_generation(scenario,satellites,gs)

        number_of_gs =length(gs);

        time_vector = scenario.StartTime:seconds(scenario.SampleTime):scenario.StopTime;

        gs_to_sat_access_matrix = zeros(number_of_gs, length(time_vector));

        gs_to_sat_access_table = [];

        for i = 1:number_of_gs
        gs_to_sat_access_info = access(gs(i),satellites);
        access_single_gs = accessIntervals(gs_to_sat_access_info);
          for j = 1:height(access_single_gs(:,1))
            contact_start_index = (table2array(access_single_gs(j,4)) - scenario.StartTime)/seconds(scenario.SampleTime);
            contact_end_index = (table2array(access_single_gs(j,5)) - scenario.StartTime)/seconds(scenario.SampleTime);
             for k = contact_start_index+1:contact_end_index+1
               gs_to_sat_access_matrix(i,k) = gs_to_sat_access_matrix(i,k) + 1;
             end
          end
          gs_to_sat_access_table = [gs_to_sat_access_table; access_single_gs];
        end
 
        contact_1_sat = [];
        contact_2_sat = [];
        contact_3_sat = [];
        contact_4_plus_sat = [];


        for i = 1:length(gs_to_sat_access_matrix(:,1))
          for j = 1:length(gs_to_sat_access_matrix(1,:))

            if(gs_to_sat_access_matrix(i,j) == 1)
            contact_1_sat = [contact_1_sat;j,i];

            elseif(gs_to_sat_access_matrix(i,j) == 2)
            contact_2_sat = [contact_2_sat;j,i];
            elseif(gs_to_sat_access_matrix(i,j) == 3)
            contact_3_sat = [contact_3_sat;j,i];

            elseif(gs_to_sat_access_matrix(i,j) >= 4)
            contact_4_plus_sat = [contact_4_plus_sat;j,i];

            end

          end
        end


    
        end