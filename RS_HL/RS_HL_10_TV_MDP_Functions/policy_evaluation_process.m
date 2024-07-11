function MDP = policy_evaluation_process(MDP, time_index_vector,sat_to_sat_contact_3d_matrix, destination_state,policy_iteration_count,gamma)

number_of_states = length(sat_to_sat_contact_3d_matrix(1,:,1));

tolerance = 1e-1;
max_iterations = 1000;
Delta = inf; 
value_iteration_count = 0;


t_n = max(time_index_vector);
sat_to_sat_contact_matrix =  sat_to_sat_contact_3d_matrix(:,:,t_n);

while(abs(Delta) > tolerance) && (value_iteration_count < max_iterations)
    Delta_vector = zeros(number_of_states,1);


    state_value_vector = zeros(1,number_of_states);

    for state_index = 1:number_of_states

        next_state_candidates = find(sat_to_sat_contact_matrix(state_index,:) == 1);
        number_of_actions = length(next_state_candidates);

        if state_index == destination_state
        number_of_actions = 1;
        end


        v = MDP.(['time' num2str(t_n)]).(['state' num2str(state_index)]).state_value;
        v_update = 0;    


        for action_index = 1:number_of_actions
            T_s = MDP.(['time' num2str(t_n)]).(['state' num2str(state_index)]).(['action' num2str(action_index)]).('success').('transition_probability');
            r_s = MDP.(['time' num2str(t_n)]).(['state' num2str(state_index)]).(['action' num2str(action_index)]).('success').('reward');
            T_f = MDP.(['time' num2str(t_n)]).(['state' num2str(state_index)]).(['action' num2str(action_index)]).('fail').('transition_probability');
            r_f = MDP.(['time' num2str(t_n)]).(['state' num2str(state_index)]).(['action' num2str(action_index)]).('fail').('reward');
            s_s = MDP.(['time' num2str(t_n)]).(['state' num2str(state_index)]).(['action' num2str(action_index)]).('success').('next_state');
            s_f = MDP.(['time' num2str(t_n)]).(['state' num2str(state_index)]).(['action' num2str(action_index)]).('fail').('next_state');
            v_s_s = MDP.(['time' num2str(t_n)]).(['state' num2str(s_s)]).('state_value');
            v_s_f = MDP.(['time' num2str(t_n)]).(['state' num2str(s_f)]).('state_value');


            q_s_a = T_s * (r_s + gamma* v_s_s) + T_f * (r_f + gamma* v_s_f);
            MDP.(['time' num2str(t_n)]).(['state' num2str(state_index)]).(['action' num2str(action_index)]).('action_value') = q_s_a;

            pi = MDP.(['time' num2str(t_n)]).(['state' num2str(state_index)]).(['action' num2str(action_index)]).('policy_function');

            v_update = v_update + pi*q_s_a;
        end

       MDP.(['time' num2str(t_n)]).(['state' num2str(state_index)]).state_value = v_update;
       Delta_vector(state_index) = abs(v_update-v);
       state_value_vector(state_index) = v_update; 
    end


    Delta = max(Delta_vector);
    value_iteration_count = value_iteration_count + 1;
    if value_iteration_count >= max_iterations
        fprintf('Maximum number of iterations reached for state %d\n', state_index);
    end

    if abs(Delta) < tolerance
        fprintf('Policy: %d -> Value Iteration: %d\n',policy_iteration_count, value_iteration_count)
    end


end

end