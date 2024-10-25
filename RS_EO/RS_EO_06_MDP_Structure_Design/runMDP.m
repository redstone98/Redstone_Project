function MDP = runMDP(sat_to_sat_contact_3d_matrix, time_index_vector,destination_state)


%% Configure MDP Structure (Level 1 - 6)
MDP = config_MDP_struct(sat_to_sat_contact_3d_matrix, time_index_vector);

MDP = add_destination_info(MDP, destination_state, time_index_vector, sat_to_sat_contact_3d_matrix);

%% Policy Iteration Process for Last Timestep

number_of_states = length(sat_to_sat_contact_3d_matrix(1,:,1));

pi_distribution = zeros(number_of_states);
policy_iteration_count = 1;
gamma = 1;

while true

    MDP = policy_evaluation_process(MDP, time_index_vector,sat_to_sat_contact_3d_matrix, destination_state,policy_iteration_count,gamma);

   [MDP,pi_distribution_updated] = policy_improvement_process(MDP,time_index_vector, sat_to_sat_contact_3d_matrix, destination_state);

   if pi_distribution == pi_distribution_updated
       break;
   end

   pi_distribution = pi_distribution_updated;
   policy_iteration_count = policy_iteration_count +1;

end

MDP.(['time' num2str(max(time_index_vector))]).policy_distribution = pi_distribution;

%% State and Action value propagation to t_n-1 ... t_1
MDP = value_function_propagation(MDP,time_index_vector,sat_to_sat_contact_3d_matrix, destination_state, gamma);

end