function MDP = config_MDP_struct(state_transition_matrix,state_reward_matrix, time_index_vector)


% Level 1: Initialize MDP structure
MDP = struct();

% Level 2: Initialize Time Index Vector

for time_index = time_index_vector
    MDP.(['time' num2str(time_index)]) = {};
end

% Level 3: Initialize State and Policy Distribution

number_of_states = length(state_transition_matrix(1,:,1));

for time_index = time_index_vector
    for state_index = 1:number_of_states
    % Level 3.1: Initialize State
    MDP.(['time' num2str(time_index)]).(['state' num2str(state_index)]) = {};
    % Level 3.2: Initialize Policy Distribution
    MDP.(['time' num2str(time_index)]).('policy_distribution') = zeros(number_of_states);
    end
end

% Level 4: Initialize Action, State Value


for time_index = time_index_vector
    sat_to_sat_contact_matrix =  state_transition_matrix(:,:,time_index);
    for state_index = 1:number_of_states

        % Find the availabe next state candidates for each current state
        next_state_candidates = find(sat_to_sat_contact_matrix(state_index,:) == 1);
        number_of_actions = length(next_state_candidates);

        for action_index = 1:number_of_actions
        % Level 4.1: Initialize Action
        MDP.(['time' num2str(time_index)]).(['state' num2str(state_index)]).(['action' num2str(action_index)]) = {};

        % Level 4.2: Initialize State Value with 0 initial value
        MDP.(['time' num2str(time_index)]).(['state' num2str(state_index)]).('state_value') = 0;
    
        % Level 4.3: Initialize Action Value Vector
        MDP.(['time' num2str(time_index)]).(['state' num2str(state_index)]).('action_value_vector')  = {};

        % Level 4.4: Initialize Next State Vector
        MDP.(['time' num2str(time_index)]).(['state' num2str(state_index)]).('next_state_vector')  = next_state_candidates';

        end
    end
end

% Level 5: Initialize Success/Fail, Action Value and Policy Function

for time_index = time_index_vector
    sat_to_sat_contact_matrix =  state_transition_matrix(:,:,time_index);
    for state_index = 1:number_of_states

        % Find the availabe next state candidates for each current state
        next_state_candidates = find(sat_to_sat_contact_matrix(state_index,:) == 1);
        number_of_actions = length(next_state_candidates);        

        for action_index = 1:number_of_actions
            % Level 5.1: Initialize Success / Fail and Next State (Level 6)
            MDP.(['time' num2str(time_index)]).(['state' num2str(state_index)]).(['action' num2str(action_index)]).('success').('next_state') = {};
            MDP.(['time' num2str(time_index)]).(['state' num2str(state_index)]).(['action' num2str(action_index)]).('fail').('next_state') = {};

            % Level 5.2: Initialize Action Value
            MDP.(['time' num2str(time_index)]).(['state' num2str(state_index)]).(['action' num2str(action_index)]).('action_value') = {};

            % Level 5.3: Initialize Policy Function
            MDP.(['time' num2str(time_index)]).(['state' num2str(state_index)]).(['action' num2str(action_index)]).('policy_function') = 1/number_of_actions;

        end
    end
end

% Level 6-1: Define Next State value, State Transition Probability


for time_index = time_index_vector
    sat_to_sat_contact_matrix =  state_transition_matrix(:,:,time_index);
    for state_index = 1:number_of_states    

        % Find the availabe next state candidates for each current state
        next_state_candidates = find(sat_to_sat_contact_matrix(state_index,:) == 1);
        number_of_actions = length(next_state_candidates);        

        for action_index = 1:number_of_actions

         % Level 6.1: Case 1 - Next state value if Action is success
         MDP.(['time' num2str(time_index)]).(['state' num2str(state_index)]).(['action' num2str(action_index)]).('success').('next_state') = next_state_candidates(action_index);

         % Level 6.1: Case 2 - Next state value if Action is failure
         MDP.(['time' num2str(time_index)]).(['state' num2str(state_index)]).(['action' num2str(action_index)]).('fail').('next_state') = state_index;

         % Level 6.2: Define State transition probability T (S - 1 / F - 0)
         MDP.(['time' num2str(time_index)]).(['state' num2str(state_index)]).(['action' num2str(action_index)]).('success').('transition_probability') = 1;
         MDP.(['time' num2str(time_index)]).(['state' num2str(state_index)]).(['action' num2str(action_index)]).('fail').('transition_probability') = 0;

        end
    end
end

% Level 6-2: Define Reward

for time_index = time_index_vector
    sat_to_sat_contact_matrix =  state_transition_matrix(:,:,time_index);
    for state_index = 1:number_of_states

        % Find the availabe next state candidates for each current state
        next_state_candidates = find(sat_to_sat_contact_matrix(state_index,:) == 1);
        number_of_actions = length(next_state_candidates);        

        for action_index = 1:number_of_actions

            MDP.(['time' num2str(time_index)]).(['state' num2str(state_index)]).(['action' num2str(action_index)]).('fail').('reward') = 0;

            next_state = MDP.(['time' num2str(time_index)]).(['state' num2str(state_index)]).(['action' num2str(action_index)]).('success').('next_state');
            MDP.(['time' num2str(time_index)]).(['state' num2str(state_index)]).(['action' num2str(action_index)]).('success').('reward') = state_reward_matrix(state_index,next_state, time_index);


        end
    end
end

fprintf('simulation set up complete! \n')

end