function MDP = add_destination_info(MDP, destination_state, time_index_vector, sat_to_sat_contact_3d_matrix)


number_of_states = length(sat_to_sat_contact_3d_matrix(1,:,1));

for time_index = time_index_vector
MDP.(['time' num2str(time_index)]) = rmfield(MDP.(['time' num2str(time_index)]), ['state' num2str(destination_state)]);
MDP.(['time' num2str(time_index)]).(['state' num2str(destination_state)]).('state_value') = 0;
MDP.(['time' num2str(time_index)]).(['state' num2str(destination_state)]).('next_state_vector') = destination_state; 
MDP.(['time' num2str(time_index)]).(['state' num2str(destination_state)]).(['action' num2str(1)]).('action_value') = 0;
MDP.(['time' num2str(time_index)]).(['state' num2str(destination_state)]).(['action' num2str(1)]).('policy_function') = 1;
MDP.(['time' num2str(time_index)]).(['state' num2str(destination_state)]).(['action' num2str(1)]).('success').('next_state') = destination_state;
MDP.(['time' num2str(time_index)]).(['state' num2str(destination_state)]).(['action' num2str(1)]).('success').('transition_probability') = 1;
MDP.(['time' num2str(time_index)]).(['state' num2str(destination_state)]).(['action' num2str(1)]).('success').('reward') = 0;
MDP.(['time' num2str(time_index)]).(['state' num2str(destination_state)]).(['action' num2str(1)]).('fail').('next_state') = destination_state;
MDP.(['time' num2str(time_index)]).(['state' num2str(destination_state)]).(['action' num2str(1)]).('fail').('transition_probability') = 0;
MDP.(['time' num2str(time_index)]).(['state' num2str(destination_state)]).(['action' num2str(1)]).('fail').('reward') = 0;
end


% update corresponding reward information based on destination state
for time_index = time_index_vector

    sat_to_sat_contact_matrix =  sat_to_sat_contact_3d_matrix(:,:,time_index);
    % We don't count the case of destination state: already defined 
    for state_index = 1:number_of_states

        if state_index == destination_state
            continue
        end

        % Find the availabe next state candidates for each current state
        next_state_candidates = find(sat_to_sat_contact_matrix(state_index,:) == 1);
        number_of_actions = length(next_state_candidates);
        for action_index = 1:number_of_actions

        % Define State transition Probability T (Level 5-2 = 4-1-2)
        current_state = state_index;
        next_state = MDP.(['time' num2str(time_index)]).(['state' num2str(state_index)]).(['action' num2str(action_index)]).('success').('next_state');

        %Filter: Next state is destination state
            if next_state == destination_state

            % Positive Reward function for Next state is destination case
            % Same orbit reward = 100, Different orbit reward = 50
                if current_state < 25
                    if next_state < 25
                        MDP.(['time' num2str(time_index)]).(['state' num2str(state_index)]).(['action' num2str(action_index)]).('success').('reward') = 100;
                    end
                    if next_state > 23
                        MDP.(['time' num2str(time_index)]).(['state' num2str(state_index)]).(['action' num2str(action_index)]).('success').('reward') = 75;
                    end
                end

                if current_state > 23
                    if next_state > 23
                        MDP.(['time' num2str(time_index)]).(['state' num2str(state_index)]).(['action' num2str(action_index)]).('success').('reward') = 100;
                    end
                    if next_state < 25
                        MDP.(['time' num2str(time_index)]).(['state' num2str(state_index)]).(['action' num2str(action_index)]).('success').('reward') = 75;
                    end
                end
            end
        end
    end
end

fprintf('simulation set up complete! \n')



end