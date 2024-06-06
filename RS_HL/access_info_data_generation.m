function access_information = access_info_data_generation(satellites,gs)


        number_of_cities =length(gs);
        
        access_table = [];
        
        for i = 1:number_of_cities
        access_info = access(satellites,gs(i));
        access_update = accessIntervals(access_info);
        access_table = [access_table; access_update];
        end

  access_information.access_Intervals = access_table;

end