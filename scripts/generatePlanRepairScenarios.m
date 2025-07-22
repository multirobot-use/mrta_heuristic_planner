function generatePlanRepairScenarios(amount, number_agents, number_tasks, short_discrete_delays, long_discrete_delays)
    if nargin < 1
        amount = 100;
    end

    if nargin < 3
        % Scenario size to be loaded (they must be previously generated)
        number_agents = 10;
        number_tasks = 50;
    end

    if nargin < 5
        % Short delay tests
        short_discrete_delays = [0.5 1 2] * 60; % seconds

        % Long delay tests (applied only to recharge tasks)
        long_discrete_delays = [10 15 20] * 60; % seconds
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
    if ~exist('../mat/planRepairTests/', 'dir')
        mkdir('../mat/planRepairTests/')
    end

    % Check if number_agents and number_tasks are valid
    if any(number_agents < 1) || any(number_tasks < 1) || isempty(number_agents) || isempty(number_tasks)
        error('Invalid number of agents or tasks specified');
    end

    %% Generate random delays for each scenario
    disp('Generating random delays for each scenario...');
    for random_scenario = amount_lb:amount_ub
        % Construct the scenario ID
        scenario_id = [num2str(number_agents), 'r', num2str(number_tasks), 't', num2str(random_scenario)];

        % Get the solution to the scenario
        [Agent, ~, ~, ~] = initializer(scenario_id, 'Test', [], 9); % ov9 -> "Heuristic" in T-RO

        % Generate long short delays for this scenario
        for i = 1:2
            % Generate a random delay
            switch i
            case 1 % Short delay tests
                % Generate a random delay
                delay = short_discrete_delays(randi(length(short_discrete_delays)));
                % Get a random robot
                robot = randi(length(Agent));
                % Get a random slot in the robot's queue
                slot = randi(length(Agent(robot).queue) - 1);
                % Suffix to save the results
                suffix = 'short';
            case 2 % Long delay tests
                % Generate a random delay
                delay = long_discrete_delays(randi(length(long_discrete_delays)));
                % Get a random robot
                robot = randi(length(Agent));
                % Get the positions of the recharge tasks in robot's queue
                recharge_task_slots = [Agent(robot).queue == 1].*[0:length(Agent(robot).queue) - 1];
                recharge_task_slots = recharge_task_slots(recharge_task_slots ~= 0);
                % Get a random recharge slot in the robot's queue
                slot = recharge_task_slots(randi(length(recharge_task_slots)));
                % Suffix to save the results
                suffix = 'long';
            end

            % Save the random delay: repair time, result, and the repaired plan
            save(['../mat/planRepairTests/delay_', scenario_id, '_', suffix, '.mat'], 'delay', 'robot', 'slot');
        end
    end

    disp('Random delays generated.');
end