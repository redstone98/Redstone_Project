function MDP = value_function_propagation(MDP,time_index_vector,sat_to_sat_contact_3d_matrix, destination_state,gamma)

number_of_states = length(sat_to_sat_contact_3d_matrix(1,:,1));

inverse_time_index = flip(time_index_vector(1:end-1));

for time_index = inverse_time_index
    pi_distribution_updated = zeros(number_of_states);
    sat_to_sat_contact_matrix =  sat_to_sat_contact_3d_matrix(:,:,time_index);
    for state_index = 1:number_of_states
        % Find the availabe next state candidates for each current state
        next_state_candidates = find(sat_to_sat_contact_matrix(state_index,:) == 1);
        number_of_actions = length(next_state_candidates);

        if state_index == destination_state
            number_of_actions = 1;
        end

        action_value_collector = zeros(number_of_actions,1);

        for action_index = 1:number_of_actions

            T_s = MDP.(['time' num2str(time_index)]).(['state' num2str(state_index)]).(['action' num2str(action_index)]).('success').('transition_probability');
            r_s = MDP.(['time' num2str(time_index)]).(['state' num2str(state_index)]).(['action' num2str(action_index)]).('success').('reward');
            T_f = MDP.(['time' num2str(time_index)]).(['state' num2str(state_index)]).(['action' num2str(action_index)]).('fail').('transition_probability');
            r_f = MDP.(['time' num2str(time_index)]).(['state' num2str(state_index)]).(['action' num2str(action_index)]).('fail').('reward');
            s_s = MDP.(['time' num2str(time_index)]).(['state' num2str(state_index)]).(['action' num2str(action_index)]).('success').('next_state');
            s_f = MDP.(['time' num2str(time_index)]).(['state' num2str(state_index)]).(['action' num2str(action_index)]).('fail').('next_state');
            v_s_s = MDP.(['time' num2str(time_index+1)]).(['state' num2str(s_s)]).('state_value');
            v_s_f = MDP.(['time' num2str(time_index+1)]).(['state' num2str(s_f)]).('state_value');


            q_s_a = T_s * (r_s + gamma* v_s_s) + T_f * (r_f + gamma* v_s_f);
            MDP.(['time' num2str(time_index)]).(['state' num2str(state_index)]).(['action' num2str(action_index)]).('action_value') = q_s_a;
            action_value_collector(action_index) = q_s_a;
        end
        state_value = max(action_value_collector);
        MDP.(['time' num2str(time_index)]).(['state' num2str(state_index)]).('state_value') = state_value;

        argmax_a = find(action_value_collector == state_value);

        for action_index = 1:number_of_actions
            if any(action_index == argmax_a)
                MDP.(['time' num2str(time_index)]).(['state' num2str(state_index)]).(['action' num2str(action_index)]).('policy_function') = 1/length(argmax_a);
            else
                MDP.(['time' num2str(time_index)]).(['state' num2str(state_index)]).(['action' num2str(action_index)]).('policy_function') = 0; 
            end
        pi_distribution_updated(state_index,action_index) = MDP.(['time' num2str(time_index)]).(['state' num2str(state_index)]).(['action' num2str(action_index)]).('policy_function');
        end        
    end

    MDP.(['time' num2str(time_index)]).policy_distribution = pi_distribution_updated;
end

end