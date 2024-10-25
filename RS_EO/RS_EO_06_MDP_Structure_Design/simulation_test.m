function [time_list, state_list, reward_list, cumulative_reward_list, state_value_list] = simulation_test(start_time,start_state,MDP,time_index_vector, arrival_state)

time_index = start_time:max(time_index_vector);
simulation_time = length(time_index);

state_list = zeros(simulation_time,1);
reward_list = zeros(simulation_time,1);
cumulative_reward_list = zeros(simulation_time,1);
time_list = zeros(simulation_time,1);
state_value_list = zeros(simulation_time,1);
cumulative_reward = 0;

state_list(1) = start_state;
time_list(1) = time_index(1);

% We should consider the possibility of the failure (80% success and 20%
% failure)

for t = 1:simulation_time
    current_state = state_list(t);

    
    state_value_list(t) = MDP.(['time' num2str(time_index(t))]).(['state' num2str(current_state)]).('state_value');

    if ismember(current_state, arrival_state) == true
        break;
    end

    pi_dist = MDP.(['time' num2str(time_index(t))]).('policy_distribution');
    action_number = find(pi_dist(current_state,:));

    if length(action_number) > 1
        action_number = randsample(action_number,1);
    end

    next_state = MDP.(['time' num2str(time_index(t))]).(['state' num2str(current_state)]).(['action' num2str(action_number)]).('success').('next_state');
    reward = MDP.(['time' num2str(time_index(t))]).(['state' num2str(current_state)]).(['action' num2str(action_number)]).('success').('reward');
    cumulative_reward = cumulative_reward + reward;

    reward_list(t+1) = reward;
    cumulative_reward_list(t+1) = cumulative_reward;
    state_list(t+1) = next_state;
    time_list(t+1) = time_index(t+1);

end

state_list = state_list(state_list ~= 0);
cumulative_reward_list = [0; cumulative_reward_list(cumulative_reward_list~=0)];
reward_list = [0;reward_list(reward_list ~= 0)];
state_value_list = [state_value_list(state_value_list ~=0);0;0];
time_list = time_list(time_list ~=0);


end