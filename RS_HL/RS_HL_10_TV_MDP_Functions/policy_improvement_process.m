function [MDP,pi_distribution_updated] = policy_improvement_process(MDP,time_index_vector, sat_to_sat_contact_3d_matrix, destination_state)

t_n = max(time_index_vector);

sat_to_sat_contact_matrix = sat_to_sat_contact_3d_matrix(:,:,t_n);

number_of_states = length(sat_to_sat_contact_3d_matrix(1,:,1));

pi_distribution_updated = zeros(number_of_states);

    for state_index = 1:number_of_states
    
        next_state_candidates = find(sat_to_sat_contact_matrix(state_index,:) == 1);
        number_of_actions = length(next_state_candidates);
    
        if state_index == destination_state
          number_of_actions = 1;
        end
    
        action_value_vector = zeros(number_of_actions,1);
    
        for action_index = 1:number_of_actions
        action_value_vector(action_index) = MDP.(['time' num2str(t_n)]).(['state' num2str(state_index)]).(['action' num2str(action_index)]).('action_value');
        end
    
        maximum_action_value = max(action_value_vector);
        argmax_a = find(action_value_vector == maximum_action_value);
        for action_index = 1:number_of_actions
          if any(action_index == argmax_a)
            MDP.(['time' num2str(t_n)]).(['state' num2str(state_index)]).(['action' num2str(action_index)]).('policy_function') = 1/length(argmax_a);
          else
            MDP.(['time' num2str(t_n)]).(['state' num2str(state_index)]).(['action' num2str(action_index)]).('policy_function') = 0; 
          end

        pi_distribution_updated(state_index,action_index) = MDP.(['time' num2str(t_n)]).(['state' num2str(state_index)]).(['action' num2str(action_index)]).('policy_function');  
        end
    end

end