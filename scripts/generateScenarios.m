function generateScenarios(amount, number_agents, number_tasks, types, discretized)
    % Number of different types of robots
    if nargin < 4
        types = 3;
        discretized = 1;
    elseif nargin < 5
        discretized = 1;
    end

    % Check if amount is a vector
    switch length(amount)
    case 1
        amount_lb   = 1;
        amount_ub   = amount;
    case 2
        amount_lb   = amount(1);
        amount_ub   = amount(2);
    otherwise
        error('amount must be a scalar or a vector of length 2');
    end

    % Create needed directories if they don't exists
    if ~exist('../mat/', 'dir')
        mkdir('../mat/')
    end

    % Check if number_agents and number_tasks are valid
    if any(number_agents < 1) || any(number_tasks < 1) || isempty(number_agents) || isempty(number_tasks)
        error('Invalid number of agents or tasks specified');
    end

    %% Generate random scenarios
    for a = 1:length(number_agents)
        num_agents = number_agents(a);
        for t = 1:length(number_tasks)
            num_tasks = number_tasks(t);
            for random_scenario = amount_lb:amount_ub
                scenario_id = [num2str(num_agents), 'r', num2str(num_tasks), 't', num2str(random_scenario)];
                [Agent, Task] = scenario(num_agents, num_tasks, types, discretized);
                save(['../mat/Agent_', scenario_id, '.mat'], 'Agent');
                save(['../mat/Task_', scenario_id, '.mat'], 'Task');
            end
        end
    end
end