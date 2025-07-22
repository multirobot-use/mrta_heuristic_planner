% Initializer for sparseXATS or small-scale tests: function [Agent, Task, allocation_order, S_R] = heuristicTaskAllocator(arg_1, arg_2, reward_coefficients, v, seed)
% Scalability tests: function [Agent, Task, allocation_order] = heuristicTaskAllocator(arg_1, arg_2, reward_coefficients, v, seed)
% Plan repair: function [Agent, Task, Synchs, Relays] = heuristicTaskAllocator(arg_1, arg_2, reward_coefficients, v, seed)
function [Agent, Task, Synchs, Relays] = heuristicTaskAllocator(arg_1, arg_2, reward_coefficients, v, seed)
    % arg_1 and arg_2 should either be Agent and Task or a valid scenario_id and optionally execution id.
    % reward_coefficients is an integer to select the reward coefficients to use.
    % v is an integer to select the version of the algorithm to use:
    %   - 1:  original version. Task allocation order: first task from group 1 and then from group 2.
    %   - 2:  alternating version. Task allocation order: n tasks from group 1 with tasks from group 2 in between (begin and end with n tasks from group 1).
    %   - 3:  TODO: sand grains algorithm version. Task allocation order: first tasks from group 2, then try to fill waiting time gaps with tasks from group 1.
    %   - 4:  random version. Task allocation order: random.
    %   - 5:  seed version. Task allocation order: input order.
    %   - 6:  random random version. Task allocation order: random. Robot selection criteria: random.
    %   - 7:  reward version. Task allocation order: reward order without group distinction.
    %   - 8:  dynamic reward version. Task allocation order: reward order without group distinction. Reward coefficients are recomputed after each task allocation. Also, the reward terms follows a hierarchy: each term is used to break ties in the previous one.
    %   - 9:  dynamic reward including waiting time. Compute the introduced waiting time for each task as it would be allocated next, and use is as reward criteria
    %   - 10: random sampling version. Robot selection step performs a random sampling of the robots after the deterministic robot selection and keeps the best robots selection in terms of makespan and waiting time.
    %   - 11: versions 9 and 10 combined.
    %   - 12: greedy version. task order according to version 7 and robot selection according to the buffet algorithm.
    %   - 13: re-planning version. The delay is passed through the seed input parameter. Using version 9 (Heuristic version in T-RO). It takes and old plan and a delay and re-plans the plan leaving the tasks that are already executed or in progress, and re-allocating the tasks that haven't started yet or that are affected by the delay.

    % Note: S_R is needed to build the solution array, that is needed to process data in small scale experiments.
    % If we don't need to process data, we can remove S_R, Sta1s1a2s2, Rta1s1a2s2 and all the code related to them.

    % Note: Synchs and Relays outputs are needed in planRepair for the repair experiments.
    % If we are not using planRepair, we can remove Synchs and Relays and all the code related to them.

    %? Note: remember to set the following flags according to the function definition
    heuristicTaskAllocator_flag = false;
    small_scale_tests_flag = false;
    repair_flag = true;

    if nargin < 2
        arg_2 = [];
    end

    % Load Agent and Task
    if ischar(arg_1)
        scenario_id  = arg_1;

        % Try loading scenario
        try
            [Agent, Task] = scenario(scenario_id);
        catch
            error('Either Agent and Task or a valid scenario_id must be provided');
        end
    elseif isempty(arg_1) || isempty(arg_2)
        error('Either Agent and Task or a valid scenario_id must be provided');
    else
        Agent = arg_1;
        Task  = arg_2;
    end

    if nargin < 3 || isempty(reward_coefficients)
        reward_coefficients = 1;
    end

    if nargin < 4 || isempty(v)
        v = 1;
    end

    % Check if the version is 13th: Re-planning version
    if v == 13
        % Check if the delay is already applied and propagated to other affected slots
        if ~isfield(Agent, 'delay_s')
            disp('You need to execute first the Repair Algorithm to generate the delay matrix');
            error('A delay must be provided to use the re-planning version');
        end
    end
    
    % Get  constant scenario values
    [Agent, Task, A, T, S, N, R, Td_a_t_t, Te_t_nf, ~] = getConstantScenarioValues(Agent, Task);

    % Initialize synchronization and relays structures
    % Case 1: we are using the heuristic as heuristicTaskAllocator for XATS or we need them for small scenario tests
    if heuristicTaskAllocator_flag || small_scale_tests_flag
        Sta1s1a2s2 = zeros(T-1,A,S,A,S);
        Rta1s1a2s2 = zeros(T-1,A,S,A,S);

    % Case 2: we need synch and relays constraints for the plan repair tests
    elseif repair_flag
        Synchs = [];
        Relays = [];
    end

    % Tolerance
    tol = 1e-6;

    % Check if the version is not 13th: Re-planning version
    if v ~= 13
        % Initialize the structures fields that stores robots' queue and times
        for a = 1:A
            Agent(a).queue   = 0;                           % Agent(a).queue   = 0;                           % S + 1
            Agent(a).Td_s    = zeros(1,1);                  % Agent(a).Td_s    = zeros(1,S);                  % S
            Agent(a).Tw_s    = zeros(1,1);                  % Agent(a).Tw_s    = zeros(1,S);                  % S
            Agent(a).Te_s    = zeros(1,1);                  % Agent(a).Te_s    = zeros(1,S);                  % S
            Agent(a).tfin_s  = zeros(1,1+1);                % Agent(a).tfin_s  = zeros(1,S+1);                % S + 1
            Agent(a).ac_Ft_s = [Agent(a).Ft_0, zeros(1,1)]; % Agent(a).ac_Ft_s = [Agent(a).Ft_0, zeros(1,S)]; % S + 1
        end
    end

    % Initialize number of robots selected to simultaneously execute each task
    na = [Task.nr];

    % Sort tasks by type and deadline. First non-decomposable, fragmentable and relayable (with Task(t).nf = 1) tasks (group 1), then relayable tasks with Task(t).nf != 1 (group 2).
    % Get indexes of (group 1) the non-decomposable, fragmentable and relayable (with Task(t).nf = 1) tasks (tf_idx ranges from 1 to T-1).
    tf_idx = find([Task(2:end).Relayability] == 0 | ([Task(2:end).Relayability] == 1 & [Task(2:end).nf] == 1));

    % Get indexes of (group 2) the relayable tasks with Task(t).nf != 1 (tr_idx ranges from 1 to T-1).
    tr_idx = find([Task(2:end).Relayability] == 1 & [Task(2:end).nf] ~= 1);

    % Get the indexes of all the tasks without the recharge task
    all_idx = [2:T];

    % Adjust indexes to match the tasks' indexes
    tf_idx = tf_idx + 1;
    tr_idx = tr_idx + 1;

    % Initialize n_samples and best_solution_criteria for the cases when it's not used
    n_samples = 0;
    best_solution_criteria = 0;

    % Compute or initialize the task allocation order
    switch v
    case {1, 2, 4, 6, 5, 7, 12}
        % Compute tasks "reward" to get the allocation order
        % Have into account task's:
        %   - Deadline (tmax)
        %   - Execution time (Te)
        %   - Number of fragments (Task(t).nf)
        %   - Percentage of robots needed (N) with respect to the total (relayable tasks with Task(t).nf > 1 uses all of them)
        %   - Percentage of robots needed (N) with respect to the compatible ones (Hr, Task(t).nc) (relayable tasks with Task(t).nf > 1 uses all of them)
        
        % Set weights: the lower the better
        switch reward_coefficients
        case 2
            alpha   = 1;
            beta    = 0.5;
            gamma   = 0.5;
            delta   = 0.5;
            epsilon = 1;
        case 3
            alpha   = 1;
            beta    = 1;
            gamma   = 1;
            delta   = 1;
            epsilon = 1;
        case 4
            alpha   = 0;
            beta    = 1;
            gamma   = 0;
            delta   = 0;
            epsilon = 0;
        otherwise
            alpha   = 0.5;
            beta    = 1;
            gamma   = 0.5;
            delta   = 0.5;
            epsilon = 1;
        
        end

        % Get maximum task deadline
        max_tmax = max([Task.tmax]);

        % Get maximum execution time
        max_Te = max([Task.Te]);

        % Get the tasks' reward: the lower the better
        % TODO: Include traveling time by using the distance of the N_t nearest robots to the task. In case of a replanning, this should be enough to make task that are being executed to continue being executed unless there is a better option or can't continue being executed right now.
        for t = 2:T
            % Task(t).reward = alpha * (Task(t).tmax / max_tmax) + beta * (Task(t).Te / max_Te) + gamma * (Task(t).nf / N) + delta * (Task(t).N / A) + epsilon * (Task(t).N / Task(t).nc);
            Task(t).reward = alpha * (Task(t).tmax / max_tmax) + beta * (max_Te / Task(t).Te) + gamma * (Task(t).nf / N) + delta * (Task(t).N / A) + epsilon * (Task(t).N / Task(t).nc);
        end

        % Sort (group 1) non-decomposable, fragmentable and relayable (with Task(t).nf = 1) tasks by deadline
        [~, tf_reward_idx] = sort([Task(tf_idx).reward]);

        % Sort (group 2) relayable tasks with Task(t).nf != 1 by deadline
        [~, tr_reward_idx] = sort([Task(tr_idx).reward]);

        % Sort all tasks by reward
        [~, all_reward_idx] = sort([Task(all_idx).reward]);

        % Get groups 1, 2 and all internally ordered
        group_1   =  tf_idx(tf_reward_idx);
        group_2   =  tr_idx(tr_reward_idx);
        group_all = all_idx(all_reward_idx);

        % Repeat each task t in group 1 Task(t).nf times consecutively
        if group_1
            group_1 = repelem(group_1, [Task(group_1).nf]);
        end

        % Join indexed from both groups according to the selected version
        switch v
        case 1
            % Allocate tasks from group 1 and then from group 2
            allocation_order = [group_1, group_2; ones(1, length(group_1)), 2 * ones(1, length(group_2))];
        case 2
            % Initialize allocation order
            allocation_order = [];

            % Get the group ratio, i.e., the number of tasks from group 1 before and after each tasks from group 2
            group_ratio = floor(length(group_1) / (length(group_2) + 1));
            
            % Check if group ratio is zero
            if group_ratio == 0
                group_ratio = 1;
            end

            % Initialize group 1 index
            group_1_idx = 1;

            % Allocate n tasks from group 1 before each task from group 2
            for group_2_idx = length(group_2):-1:1
                % Check if group 1 index is less than the length of group 1
                if group_1_idx <= length(group_1)
                    % Append n tasks from group 1
                    for i = 1:group_ratio
                        % Check if group 1 index is less than the length of group 1
                        if group_1_idx <= length(group_1)
                            % Append group 1 task
                            allocation_order = [allocation_order, [group_1(group_1_idx); 1]];

                            % Update group 1 index
                            group_1_idx = group_1_idx + 1;
                        else
                            % Break the loop
                            break;
                        end
                    end
                end

                % Append group 2 task
                allocation_order = [allocation_order, [group_2(group_2_idx); 2]];
            end

            % Allocate the remaining tasks from group 1 at the end
            for i = group_1_idx:length(group_1)
                % Append group 1 task
                allocation_order = [allocation_order, [group_1(i); 1]];
            end
        case {4, 6}
            % Initialize random number generator seed with the current time
            rng('shuffle');

            % Initialize allocation order
            allocation_order = [group_1, group_2; ones(1, length(group_1)), 2 * ones(1, length(group_2))];
            
            % Allocate tasks in random order
            allocation_order = allocation_order(:,randperm(size(allocation_order, 2)));
        case 5
            % Allocate tasks in input order
            if nargin < 5 || isempty(seed)
                error('A seed must be provided to use the input version');
            end
            allocation_order = seed;
        case {7, 12}
            % Repeat each group 1 task in group all exactly Task(t).nf times consecutively
            allocation_order = [];
            for t = 1:length(group_all)
                if ismember(group_all(t), group_1)
                    allocation_order = [allocation_order, repmat([group_all(t); 1], [1, Task(group_all(t)).nf])];
                else
                    allocation_order = [allocation_order, [group_all(t); 2]];
                end
            end

            % Greedy task allocation
            if v == 12
                % Add the number of robots needed to the allocation_order matrix
                for t = 1:length(allocation_order)
                    if allocation_order(2, t) == 1
                        allocation_order(3, t) = na(allocation_order(1, t));
                    else
                        allocation_order(3, t) = Task(allocation_order(1, t)).nc;
                    end
                end

                % Initialize the robot buffet queue
                robot_buffet_queue = 1:A;

                % Make a copy of the task allocation order
                buffet_stations = allocation_order;

                % Initialize a cell to store which robots have already pick food from each station
                food_stations = cell(1, length(buffet_stations));

                % Iterate until all tasks are allocated
                while sum(buffet_stations(3,:)) > 0
                    % Take the first robot from the buffet queue
                    a = robot_buffet_queue(1);

                    % Get the selected robots total battery time
                    remaining_battery = Agent(a).Ft - Agent(a).Ft_saf;

                    % Initialize the food station index
                    food_station_idx = 1;

                    % Pick food from the buffet until the tray is full
                    % or until there is no more food in the buffet
                    % or until we reach the end of the buffet stations
                    while (remaining_battery > 0) && (sum(buffet_stations(3,:)) > 0) && (food_station_idx <= length(buffet_stations))
                        % Get the task index
                        t = buffet_stations(1, food_station_idx);

                        % Check if the robot a is compatible with task t
                        % if the robot has already pick food from that station
                        % if there is food left in that station
                        if ismember([Agent(a).type], Task(t).Hr) && not(ismember(a, food_stations{food_station_idx})) && buffet_stations(3, food_station_idx) > 0
                            % Check if the robot has enough battery to execute the task
                            if remaining_battery > Te_t_nf(t, Task(t).nf)
                                % Update the remaining battery
                                remaining_battery = remaining_battery - Te_t_nf(t, Task(t).nf);

                                % Update the number of robots needed for the task
                                buffet_stations(3, food_station_idx) = buffet_stations(3, food_station_idx) - 1;

                                % Add the robot to the list of robots that have already pick food from that station
                                food_stations{food_station_idx} = [food_stations{food_station_idx}, a];
                            else
                                % Break the loop
                                break;
                            end
                        end

                        % Increase the food station index
                        food_station_idx = food_station_idx + 1;
                    end

                    % Move the robot to the end of the buffet queue
                    robot_buffet_queue = [robot_buffet_queue(2:end), a];
                end
            end
        end
    case {8, 9, 10, 11, 13}
        % Tasks "rewards" will be sorted in hierarchical order:
        %   - Deadline (tmax): meet / not meet, with a safety gap relative to the task execution time. E.g. Tmax_t < current_makespan + 1'55 * nf(t) * Te_t / n^f_t
        %   - Incremented makespan: first the tasks that increase less makespan
        %   - Introduced waiting time: first the tasks that introduce less waiting time
        %   - Execution time (Te): first task with greater Te (having into account the number remaining fragments)
        %   - n^r_t / n^c_t: number of robots needed with respect to the compatible ones
        %   - Traveling distance (Td): sum of distance to the na_t nearest compatible robots normalized with na_t (nc_t instead of na_t for relayable multi-fragments tasks).
        %   - Random

        % Initialize the number of fragments for each task
        % Note: we don't need to repeat task from group 1 now, as they will have the same reward and we will just keep the record of how many time the have already been allocated using nf.
        nf = [Task.nf];

        % Check if the version is 13th: Re-planning version
        %? Note: this algorithm will only work for scenarios that can be repaired for now. As the repair algorithm now also changes the slots order in each queue to place all executed slots at the beginning and all failed or not started slots at the end. Now the only not repairable case is when the originally delayed slot gets too much delay and the robot fails because of the battery constraints. With no battery enough to reach the charging station. (TODO: we could just say that in this situation, robots automatically go to recharge, so we will only need to place a recharging task at the final moment of the battery)
        % Note: The coordination point is initialized later. That should work for the re-planning case.
        
        % Note: In re-planning we need to relax the "No 2 consecutive recharge tasks" constraint (at least if we are not using an aux task to represent the lost time).
        % TODO: I need to find out where I have to include delay_s. In planRepair was in updateTfin() and updateAcFts().
        if v == 13
            %? TODO Debug: Check if the finish time remains the same for executed slots
            % Save a copy of the current finish times to debug later
            Ft_a_s = zeros(A,S);
            for a = 1:A
                for s = 1:length(Agent(a).queue) - 1
                    Ft_a_s(a, s) = Agent(a).tfin_s(s + 1);
                end
            end

            % Initialize the fragments counter
            fragments_counter = zeros(2,T);
            fragments_counter(1,:) = [1:T];

            % Check which fragments has been executed
            for a = 1:A
                for s = 1:length(Agent(a).queue) - 1
                    % Check if this slot has been executed
                    if Agent(a).executed_flag_s(s)
                        % Increment the fragments counter for the task in this slot
                        % Note: For simplicity, we are also counting the recharges. We can ignore that later
                        fragments_counter(2, Agent(a).queue(s + 1)) = fragments_counter(2, Agent(a).queue(s + 1)) + 1;

                    % Remove not executed slots from the queue (they should be sorted, no more executed slots after the first not executed slot)
                    else
                        Agent(a).queue(s + 1:end) = [];
                        break;
                    end
                end
            end

            % Remove executed tasks from group 1 and set nf value for the rest
            % Note: Group 1 tasks (fragmentable) are completely allocated if nf * na fragments are executed. I need to compute how many nf are missing: nf = n / na (it should be a round result).
            tmp_tf_idx = tf_idx;
            for i = 1:length(tmp_tf_idx)
                t = tmp_tf_idx(i);
                % Compute the number of coalitions missing
                nf(t) = nf(t) - fragments_counter(2, t) / na(t);

                % If there are no more coalitions to allocate, remove this task from the list
                if nf(t) == 0
                    % Find the task in the group 1 list and remove it
                    tf_idx(tf_idx == t) = [];
                end
            end

            % Remove executed tasks from group 2
            % Note: Group 2 tasks (relayable) are completely allocated if any fragment is executed.
            tmp_tr_idx = tr_idx;
            for i = 1:length(tmp_tr_idx)
                t = tmp_tr_idx(i);

                % Check if this task has been allocated
                if fragments_counter(2, t)
                    % Find the task in the group 2 list and remove it
                    tr_idx(tr_idx == t) = [];
                end
            end
        end

        % Initialize the random number generator seed
        rng('shuffle');

        % Initialize the task allocation order to a non-empty value
        allocation_order = [tf_idx, tr_idx; ones(1, length(tf_idx)), 2 * ones(1, length(tr_idx))];

        % Initialize the logs
        % allocation_order_log_file = fopen('../logs/allocation_order.txt', 'a');
        % fprintf(allocation_order_log_file, '\n');
        % allocation_order_log = [];
        % determinant_reward_log = [];

        switch v
        case {10, 11}
            % Initialize the number of samples (including the deterministic one)
            n_samples = 1000;

            % Initialize the best solution criteria
            % 1: waiting time
            % 2: makespan
            best_solution_criteria = 2;
        end
    end

    % Initialize the coordination point for each robot (slot where the waiting time will be applied)
    % 0 if there is no valid recharge task, slot number i.o.c
    % Case 1: slot where the last recharge task with no coordination points in between is
    % Case 2: next slot
    for a = 1:A
        Agent(a).coordination_point = 0;
    end

    % Initialize the pre-selected robots for the cases when it's not used
    pre_selected_robots = [];

    % Allocate tasks in the computed order
    task = 1;
    while task <= size(allocation_order, 2)
        % Get the waiting time that each one of the remaining tasks would introduce and the makespan increment if allocated next
        switch v
        case {8, 10}
            % Initialize the introduced waiting time
            for task_8 = 1:size(allocation_order, 2)
                % Get task's id and group
                t     = allocation_order(1, task_8);
                group = allocation_order(2, task_8);

                Task(t).joint_introduced_waiting_time = 0;
                Task(t).introduced_makespan = 0;
            end
        case {9, 11, 13}
            % Compute the introduced waiting time for each remaining task and save the allocation information for later
            for task_9 = 1:size(allocation_order, 2)
                % Get task's id and group
                t     = allocation_order(1, task_9);
                group = allocation_order(2, task_9);

                % Get the indexes of the compatible robots
                compatible_robots = find(ismember([Agent.type], Task(t).Hr));

                % Get the compatible robots' time information
                [robot_finish_time, robot_pre_recharge_finish_time, robot_remaining_battery, failed_robot_flags] = compatibleRobotsTimeInformation(Agent, Task, t, v, compatible_robots, na, Td_a_t_t, Te_t_nf, A, R);

                % Remove from compatible robots those who have failed (negative battery)
                % Note: robot_remaining_battery includes Td_a_t_t(a, t, last_task + 1) and delay, i.e., 
                compatible_robots_length = length(compatible_robots);
                compatible_robots = compatible_robots(~ismember(compatible_robots, find(failed_robot_flags)));
                if length(compatible_robots) ~= compatible_robots_length
                    % Get the compatible robots' time information again
                    [robot_finish_time, robot_pre_recharge_finish_time, robot_remaining_battery, ~] = compatibleRobotsTimeInformation(Agent, Task, t, v, compatible_robots, na, Td_a_t_t, Te_t_nf, A, R);
                end

                if length(compatible_robots) < na(t)
                    error(['There are not enough compatible robots to meet the requested coalition size for task ', Task(t).name, ' because some robots failed (negative battery)']);
                end

                % Perform the robot selection for the task to get the introduced waiting time and the allocation information
                if group == 1
                    [Task(t).selected_robots, Task(t).pre_recharge, Task(t).coalition_finishing_time, Task(t).joint_introduced_waiting_time, Task(t).introduced_makespan] = robotSelectionGroup1(Agent, Task, t, v, pre_selected_robots, n_samples, best_solution_criteria, compatible_robots, robot_finish_time, robot_pre_recharge_finish_time, robot_remaining_battery, na, Td_a_t_t, Te_t_nf, R, tol);
                else
                    [Task(t).selected_robots,  Task(t).task_plan, Task(t).pre_recharge, Task(t).plan_finish_time, Task(t).joint_introduced_waiting_time, Task(t).introduced_makespan] = robotSelectionGroup2(Agent, Task, t, v, pre_selected_robots, n_samples, best_solution_criteria, compatible_robots, robot_finish_time, robot_pre_recharge_finish_time, robot_remaining_battery, na, Td_a_t_t, Te_t_nf, R, tol);
                end
            end
        end

        switch v
        case {8, 9, 10, 11, 13}
            % To make the last criteria random, we can start by randomizing the tasks. In that way, we can break ties in the previous criteria by using the default (random) order.
            allocation_order = allocation_order(:,randperm(size(allocation_order, 2)));

            % Get the current makespan
            current_makespan = getMakespan(Agent);
            
            % Compute tasks "reward" to get the allocation order
            for task_8 = 1:size(allocation_order, 2)
                % Get task's id and group
                t     = allocation_order(1, task_8);
                group = allocation_order(2, task_8);

                % Compute the first reward term: deadline
                Task(t).reward(1,1) = double(Task(t).tmax < current_makespan + 1.55 * nf(t) * Te_t_nf(t, Task(t).nf));

                % Compute the second reward term: incremented makespan: lower increment first
                Task(t).reward(2,1) = 1/Task(t).introduced_makespan;

                % Compute the third reward term: introduced waiting time: lower waiting time first
                Task(t).reward(3,1) = 1/Task(t).joint_introduced_waiting_time;

                % Compute the fourth reward term: execution time: greater execution time first
                Task(t).reward(4,1) = nf(t) * Te_t_nf(t, Task(t).nf);

                % Compute the fifth reward term: needed robot ratio: higher ratio first
                Task(t).reward(5,1) = na(t) / Task(t).nc;

                % Get the indexes of the compatible robots
                compatible_robots = find(ismember([Agent.type], Task(t).Hr));

                % Get the compatible robots' time information
                [~, ~, ~, failed_robot_flags] = compatibleRobotsTimeInformation(Agent, Task, t, v, compatible_robots, na, Td_a_t_t, Te_t_nf, A, R);

                % Remove from compatible robots those who have failed (negative battery)
                compatible_robots = compatible_robots(~ismember(compatible_robots, find(failed_robot_flags)));

                % Initialize the traveling distance
                Td = [];
                for robot = 1:length(compatible_robots)
                    a = compatible_robots(robot);
                    last_slot = length(Agent(a).queue) - 1;
                    last_task = Agent(a).queue(last_slot + 1);

                    % Compute the traveling distance for robot a
                    Td = [Td, Td_a_t_t(a, t, last_task + 1)];
                end
                % Sort the traveling distance in ascending order
                Td = sort(Td, 'ascend');

                % Compute the sixth reward term: traveling distance: lower distance first
                if group == 1
                    Task(t).reward(6,1) = 1 / (sum(Td(1:na(t))) / na(t));
                else
                    Task(t).reward(6,1) = 1 / (sum(Td) / Task(t).nc);
                end
            end

            % Get the sorting order for all tasks by reward hierarchically: greater first
            [sorted_rewards, all_reward_idx] = sortrows([Task(allocation_order(1, :)).reward]', [1, 2, 3, 4, 5, 6], 'descend');

            % Sort all tasks by reward
            allocation_order = allocation_order(:,all_reward_idx);

            % Add the new allocation order to the log
            % allocation_order_log = [allocation_order_log, num2str(allocation_order(1,1) - 1), ' '];

            % Find out who was the determinant reward criteria to break ties for each task
            % if size(allocation_order, 2) > 1
            %     determinant_reward = zeros(1, size(allocation_order, 2) - 1);
            %     for i = 2:size(sorted_rewards, 1)
            %         for j = 1:size(sorted_rewards, 2)
            %             if abs(sorted_rewards(i, j) - sorted_rewards(i-1, j)) > tol
            %                 determinant_reward(i-1) = j;
            %                 break;
            %             end
            %         end
            %     end
            %     determinant_reward_log =  [determinant_reward_log, num2str(determinant_reward(1)), ' '];
            % end
        end

        % Get task's id and group
        t     = allocation_order(1, task);
        group = allocation_order(2, task);

        % Get the indexes of the compatible robots
        compatible_robots = find(ismember([Agent.type], Task(t).Hr));

        % Get the compatible robots' time information
        [robot_finish_time, robot_pre_recharge_finish_time, robot_remaining_battery, failed_robot_flags] = compatibleRobotsTimeInformation(Agent, Task, t, v, compatible_robots, na, Td_a_t_t, Te_t_nf, A, R);

        % Remove from compatible robots those who have failed (negative battery)
        compatible_robots_length = length(compatible_robots);
        compatible_robots = compatible_robots(~ismember(compatible_robots, find(failed_robot_flags)));
        if length(compatible_robots) ~= compatible_robots_length
            % Get the compatible robots' time information again
            [robot_finish_time, robot_pre_recharge_finish_time, robot_remaining_battery, ~] = compatibleRobotsTimeInformation(Agent, Task, t, v, compatible_robots, na, Td_a_t_t, Te_t_nf, A, R);
        end

        if length(compatible_robots) < na(t)
            error(['There are not enough compatible robots to meet the requested coalition size for task ', Task(t).name, ' because some robots failed (negative battery)']);
        end

        % Allocate task from group 1
        if group == 1
            if v == 9 || v == 13
                % Get the robot selection information from the Task structure
                selected_robots               = Task(t).selected_robots;
                pre_recharge                  = Task(t).pre_recharge;
                coalition_finishing_time      = Task(t).coalition_finishing_time;
                joint_introduced_waiting_time = Task(t).joint_introduced_waiting_time;
                introduced_makespan           = Task(t).introduced_makespan;
            else
                if v == 12
                    % Get the robot selection information from the food stations cell
                    pre_selected_robots = food_stations{task};
                end

                % Select the robots that will execute the task
                [selected_robots, pre_recharge, coalition_finishing_time, joint_introduced_waiting_time, introduced_makespan] = robotSelectionGroup1(Agent, Task, t, v, pre_selected_robots, n_samples, best_solution_criteria, compatible_robots, robot_finish_time, robot_pre_recharge_finish_time, robot_remaining_battery, na, Td_a_t_t, Te_t_nf, R, tol);
            end

            % Allocate the task to the selected robots
            if heuristicTaskAllocator_flag || small_scale_tests_flag
                [Agent, Sta1s1a2s2] = allocateTaskGroup1(Agent, Task, t, v, selected_robots, pre_recharge, coalition_finishing_time, na, Td_a_t_t, Te_t_nf, R, Sta1s1a2s2, tol, heuristicTaskAllocator_flag, small_scale_tests_flag, repair_flag);
            elseif repair_flag
                [Agent, Synchs] = allocateTaskGroup1(Agent, Task, t, v, selected_robots, pre_recharge, coalition_finishing_time, na, Td_a_t_t, Te_t_nf, R, Synchs, tol, heuristicTaskAllocator_flag, small_scale_tests_flag, repair_flag);
            else
                [Agent] = allocateTaskGroup1(Agent, Task, t, v, selected_robots, pre_recharge, coalition_finishing_time, na, Td_a_t_t, Te_t_nf, R, tol, heuristicTaskAllocator_flag, small_scale_tests_flag, repair_flag);
            end
            
        end

        % Allocate task from group 2
        if group == 2
            if v == 9 || v == 13
                % Get the robot selection information from the Task structure
                selected_robots               = Task(t).selected_robots;
                task_plan                     = Task(t).task_plan;
                pre_recharge                  = Task(t).pre_recharge;
                plan_finish_time              = Task(t).plan_finish_time;
                joint_introduced_waiting_time = Task(t).joint_introduced_waiting_time;
                introduced_makespan           = Task(t).introduced_makespan;
            else
                if v == 12
                    % Get the robot selection information from the food stations cell
                    pre_selected_robots = food_stations{task};
                end

                % Select the robots that will execute the task
                [selected_robots,  task_plan, pre_recharge, plan_finish_time, joint_introduced_waiting_time, introduced_makespan] = robotSelectionGroup2(Agent, Task, t, v, pre_selected_robots, n_samples, best_solution_criteria, compatible_robots, robot_finish_time, robot_pre_recharge_finish_time, robot_remaining_battery, na, Td_a_t_t, Te_t_nf, R, tol);
            end
            % Allocate the task to the selected robots
            if heuristicTaskAllocator_flag || small_scale_tests_flag
                [Agent, Sta1s1a2s2, Rta1s1a2s2] = allocateTaskGroup2(Agent, Task, t, v, selected_robots, task_plan, pre_recharge, plan_finish_time, na, Td_a_t_t, Te_t_nf, R, Sta1s1a2s2, Rta1s1a2s2, tol, heuristicTaskAllocator_flag, small_scale_tests_flag, repair_flag);
            elseif repair_flag
                [Agent, Synchs, Relays] = allocateTaskGroup2(Agent, Task, t, v, selected_robots, task_plan, pre_recharge, plan_finish_time, na, Td_a_t_t, Te_t_nf, R, Synchs, Relays, tol, heuristicTaskAllocator_flag, small_scale_tests_flag, repair_flag);
            else
                [Agent] = allocateTaskGroup2(Agent, Task, t, v, selected_robots, task_plan, pre_recharge, plan_finish_time, na, Td_a_t_t, Te_t_nf, R, tol, heuristicTaskAllocator_flag, small_scale_tests_flag, repair_flag);
            end
        end

        % Update task counter or allocation order for the next task
        switch v
        case {1, 2, 4, 6, 5, 7, 12}
            % Update the task index
            task = task + 1;
        case {8, 9, 10, 11, 13}
            % Delete the task from the list
            if group == 1
                nf(t) = nf(t) - 1;
                if nf(t) == 0
                    % Find the task in the group 1 list and remove it
                    tf_idx(tf_idx == t) = [];
                end
            else
                % Find the task in the group 2 list and remove it
                tr_idx(tr_idx == t) = [];
            end

            % Update the list of tasks to be allocated
            allocation_order = [tf_idx, tr_idx; ones(1, length(tf_idx)), 2 * ones(1, length(tr_idx))];
        end
    end

    % Check if heuristicTaskAllocator has used more than S slots
    heuristicTaskAllocator_S = 0;
    for a = 1:A
        if length(Agent(a).queue) - 1 > heuristicTaskAllocator_S
            heuristicTaskAllocator_S = length(Agent(a).queue) - 1;
        end
    end
    
    if heuristicTaskAllocator_flag || small_scale_tests_flag
        % Fill the empty tfin slots with the last tfin value
        for a = 1:A
            for slot = length(Agent(a).queue) + 1:max(S, heuristicTaskAllocator_S) + 1
                Agent(a).tfin_s(slot) = Agent(a).tfin_s(slot - 1);
            end
        end

        if heuristicTaskAllocator_S ~= S
            % Check if the size of Sta1s1a2s2 need to be corrected
            if size(Sta1s1a2s2, 3) < heuristicTaskAllocator_S
                Sta1s1a2s2(1, 1, heuristicTaskAllocator_S, 1, 1) = 0;
            end
            if size(Sta1s1a2s2, 5) < heuristicTaskAllocator_S
                Sta1s1a2s2(1, 1, 1, 1, heuristicTaskAllocator_S) = 0;
            end
        
            % Check if the size of Rta1s1a2s2 need to be corrected
            if size(Rta1s1a2s2, 3) < heuristicTaskAllocator_S
                Rta1s1a2s2(1, 1, heuristicTaskAllocator_S, 1, 1) = 0;
            end
            if size(Rta1s1a2s2, 5) < heuristicTaskAllocator_S
                Rta1s1a2s2(1, 1, 1, 1, heuristicTaskAllocator_S) = 0;
            end
        end

        % Reshape decision variables
        Sta1s1a2s2 = reshape(Sta1s1a2s2,1,[]);
        Rta1s1a2s2 = reshape(Rta1s1a2s2,1,[]);

        % Put all decision variables together
        S_R = [Sta1s1a2s2 Rta1s1a2s2];

    elseif repair_flag && v == 13
        % Fill the empty executed_flag and delay slots
        for a = 1:A
            % Note: Any missing slots before this one (1 outsize the final length of the queue) will be fill with 0
            Agent(a).executed_flag_s(length(Agent(a).queue) + 1) = 0;
            Agent(a).delay_s(length(Agent(a).queue) + 1) = 0;

            % Delete the extra slot
            Agent(a).executed_flag_s(length(Agent(a).queue) + 1) = [];
            Agent(a).delay_s(length(Agent(a).queue) + 1) = [];
        end
    end

    % Save na in Task structure
    for t = 1:length(Task)
        Task(t).na = na(t);
    end

    %? TODO Debug: Check again if the finish time remains the same for executed slots
    if v == 13
        for a = 1:A
            for s = 1:(length(Agent(a).queue) - 1) - 1
                if Agent(a).executed_flag_s(s) && abs(Ft_a_s(a, s) - Agent(a).tfin_s(s + 1)) > tol
                    error(['Error: ', Task(Agent(a).queue(s + 1)).name, ' in slot ', num2str(s), ' of agent ', num2str(a), ' is executed, but its Ft has changed (end).']);
                end
            end
        end
    end
end

% Compute the waiting time for robot a
function [waiting_time] = getWaitingTime(a, t, v, last_slot, Agent, Task, Td_a_t_t, Te_t_nf, R, pre_recharge, coalition_finishing_time, tol)
    % Check if there is a delay in the next slot
    if v == 13
        try
            delay = Agent(a).delay_s(last_slot + 1);
        catch
            delay = 0;
        end
    else
        delay = 0;
    end

    if pre_recharge
        waiting_time = coalition_finishing_time - delay - Te_t_nf(t, Task(t).nf) - Td_a_t_t(a, t, R + 1) - Te_t_nf(R, Task(R).nf) - Td_a_t_t(a, R, Agent(a).queue(last_slot + 1) + 1) - Agent(a).tfin_s(last_slot + 1);
    else
        waiting_time = coalition_finishing_time - delay - Te_t_nf(t, Task(t).nf) - Td_a_t_t(a, t, Agent(a).queue(last_slot + 1) + 1) - Agent(a).tfin_s(last_slot + 1);
    end
    if waiting_time < - tol
        error(['Negative waiting time for robot ', num2str(a), ' and task ', Task(t).name,'. This scenario can''t be solved or this algorithm can''t find a valid solution']);
    end
end

% Update coordination point
function [Agent] = updateCoordinationPoint(Agent, Task, a, t, group, last_slot, na, R)
    % If the task is a recharge task, the new coordination point would be the next task
    if t == R
        Agent(a).coordination_point = last_slot;
    else
        % Check if the task has a coordination point: na(t) > 1 || group == 2
        if na(t) > 1 || group == 2
            Agent(a).coordination_point = 0;
        end
    end
end

% Update tfin and acFt from slot coordination_point(a) to last_slot - 1
function [Agent] = updateTfinAcFt(Agent, a, v, last_slot, R)
    for s = Agent(a).coordination_point : last_slot - 1
        % Check if there is a delay in the next slot
        if v == 13
            try
                delay = Agent(a).delay_s(s);
            catch
                delay = 0;
            end
        else
            delay = 0;
        end

        % Update tfin for the current task
        Agent(a).tfin_s(s + 1)  = Agent(a).tfin_s((s - 1) + 1) + delay + Agent(a).Td_s(s) + Agent(a).Tw_s(s) + Agent(a).Te_s(s);

        % If current task is a recharge, add only the displacement time to the accumulated flight time
        if Agent(a).queue(s + 1) == R
            Agent(a).ac_Ft_s(s + 1) = Agent(a).Td_s(s);
        else
            Agent(a).ac_Ft_s(s + 1) = delay + Agent(a).Td_s(s) + Agent(a).Tw_s(s) + Agent(a).Te_s(s);
        end

        % If the previous task was not a recharge, add also the previous accumulated flight time
        if Agent(a).queue((s - 1) + 1) ~= R
            Agent(a).ac_Ft_s(s + 1) = Agent(a).ac_Ft_s(s + 1) + Agent(a).ac_Ft_s((s - 1) + 1);
        end
    end
end

% Get the makespan from the agents
function [makespan] = getMakespan(Agent)
    makespan = max([Agent.tfin_s]);
end

% Get the compatible robots' time information
function [robot_finish_time, robot_pre_recharge_finish_time, robot_remaining_battery, failed_robot_flags] = compatibleRobotsTimeInformation(Agent, Task, t, v, compatible_robots, na, Td_a_t_t, Te_t_nf, A, R)
    robot_finish_time              = inf * ones(1,A);
    robot_pre_recharge_finish_time = inf * ones(1,A);
    robot_remaining_battery        = zeros(1,A);
    failed_robot_flags             = zeros(1,A);
    for robot = 1:length(compatible_robots)
        a = compatible_robots(robot);
        last_slot = length(Agent(a).queue) - 1;
        last_task = Agent(a).queue(last_slot + 1);

        % Check if there is a delay in the next slot
        if v == 13
            try
                delay = Agent(a).delay_s(last_slot + 1);
            catch
                delay = 0;
            end
        else
            delay = 0;
        end

        % Check if the robot has total flight time enough to execute the task and go back to the recharge station
        if Td_a_t_t(a, t, R + 1) + Te_t_nf(t, Task(t).nf) + Td_a_t_t(a, R, t + 1) < Agent(a).Ft - Agent(a).Ft_saf
            % Compute agents finish time without a previous recharge
            robot_finish_time(a) = Agent(a).tfin_s(last_slot + 1) + Td_a_t_t(a, t, last_task + 1) + delay;

            % Compute agents finish time with a previous recharge
            robot_pre_recharge_finish_time(a) = Agent(a).tfin_s(last_slot + 1) + Td_a_t_t(a, R, last_task + 1) + Te_t_nf(R, Task(R).nf) + Td_a_t_t(a, t, R + 1) + delay;

            % Get the remaining battery of each compatible robot having into account the safety margin, the initial displacement to the task place and the return to the recharge station
            robot_remaining_battery(a) = Agent(a).Ft - Agent(a).Ft_saf - Agent(a).ac_Ft_s(last_slot + 1) - Td_a_t_t(a, t, last_task + 1) - Td_a_t_t(a, R, t + 1) - delay;

            % Check if this robot has battery enough to reach the recharging station
            failed_robot_flags(a) = Agent(a).Ft - Agent(a).Ft_saf - Agent(a).ac_Ft_s(last_slot + 1) - delay - Td_a_t_t(a, R, last_task + 1) < 0;

            %? Future work (TODO): Maybe I can use this place to include the "emergency recharge" in case a delay make this robots run out of battery (if failed_robot_flags(a) == 1). I'd need to be careful and update everything I need after including the automatic recharge. 
        end
    end
end

% Robot selection for group 1
function [selected_robots, pre_recharge, coalition_finishing_time, joint_introduced_waiting_time, introduced_makespan] = robotSelectionGroup1(Agent, Task, t, v, pre_selected_robots, n_samples, best_solution_criteria, compatible_robots, robot_finish_time, robot_pre_recharge_finish_time, robot_remaining_battery, na, Td_a_t_t, Te_t_nf, R, tol)
    % Extract the coordination_point from the Agent structure
    coordination_point = [Agent.coordination_point];

    % Initialize pre-recharge flag
    pre_recharge = [robot_remaining_battery < Te_t_nf(t, Task(t).nf)];

    % Compute robots' finish time with the pre-recharge flags
    finish_time = robot_finish_time .* (1 - pre_recharge) + robot_pre_recharge_finish_time .* pre_recharge + Te_t_nf(t, Task(t).nf);

    % Sort the compatible order in descending order according to the finish time of the last task
    [~, sort_idx] = sort(finish_time, 'ascend');

    % Reorder compatible robots information
    compatible_robots = sort_idx(ismember(sort_idx, compatible_robots));

    % Check if there are at least na(t) compatible robots
    if length(compatible_robots) < na(t)
        error(['There are not enough compatible robots to meet the requested coalition size for task ', Task(t).name, ' because some robots failed (negative battery)']);
    end
    if sum([finish_time ~= Inf]) < na(t)
        error(['There are not enough compatible robots to meet the requested coalition size for task ', Task(t).name, ' .This scenario can''t be solved or this algorithm can''t find a valid solution']);
    end

    % Get the current makespan
    current_makespan = getMakespan(Agent);

    % Initialize the best robot selection and its associated introduced waiting time and incremented makespan
    best_robot_selection               = [];
    best_pre_recharge                  = [];
    best_coalition_finishing_time      = 0;
    best_joint_introduced_waiting_time = inf;
    best_introduced_makespan           = inf;

    % Initialize the number of samples
    switch v
    case {10, 11}
        sample_counter = n_samples;
    otherwise
        sample_counter = 0;
    end

    % Initialize flag to execute the next section in a while loop but only once if version is not 10 nor 11
    execute_next_section = true;
    while execute_next_section
        % Set the flag to false
        if sample_counter == 0
            execute_next_section = false;
        end

        % Select robots randomly from compatible and valid ones
        if v == 6 || ((v == 10 || v == 11) && not(isempty(best_robot_selection)))
            % Pre-select robots with total flight time enough to execute the task and go back to the recharge station
            selected_robots = find(robot_finish_time ~= Inf);

            % Randomize selected robots
            selected_robots = selected_robots(randperm(length(selected_robots), na(t)));
        elseif v == 12
            % Use the pre-selected robots
            selected_robots = pre_selected_robots;
        else
            % Select na(t) robots
            selected_robots = compatible_robots(1:na(t));
        end

        % Initialize the coalition finish time
        coalition_finishing_time = max(robot_finish_time(selected_robots) .* (1 - pre_recharge(selected_robots)) + robot_pre_recharge_finish_time(selected_robots) .* pre_recharge(selected_robots) + Te_t_nf(t, Task(t).nf));

        % Initialize aux new_pre_recharge variable
        new_pre_recharge = pre_recharge;

        % Iterate with pre-recharges until the coalition_finishing_time doesn't change
        changes_in_finish_time = true;

        % Iterate until the estimated finish time doesn't change
        while changes_in_finish_time
            % Set the auxiliary termination flag to false
            changes_in_finish_time = false;

            % Compute pre-recharge flags according to coalition_finishing_time:
            % - if (coalition_finishing_time > robot_finish_time + robot_remaining_battery) then pre_recharge = 1 because of the Tw,
            % - but if coordination_point ~= 0 then pre_recharge = 0 as we can wait in a recharge task, 
            % - unless pre_recharge was already 1, that means that we need a recharge even without waiting.
            new_pre_recharge(selected_robots) = [((coalition_finishing_time > robot_finish_time(selected_robots) + robot_remaining_battery(selected_robots)) & coordination_point(selected_robots) == 0) | pre_recharge(selected_robots)];

            % Check if there are changes in the pre-recharge flags
            if any(new_pre_recharge ~= pre_recharge)
                % Update pre-recharge flags
                pre_recharge = new_pre_recharge;

                % Update the selected robots in case we didn't use the random version
                if not(v == 6 || ((v == 10 || v == 11) && not(isempty(best_robot_selection))) || v == 12)
                    % Compute robots' finish time with the new pre-recharge flags
                    finish_time = robot_finish_time .* (1 - pre_recharge) + robot_pre_recharge_finish_time .* pre_recharge + Te_t_nf(t, Task(t).nf);

                    % Sort the robots by finish time and by used slots
                    [~, sort_idx] = sort(finish_time, 'ascend');

                    % Reorder compatible robots information
                    compatible_robots = sort_idx(ismember(sort_idx, compatible_robots));
                    
                    % Update selected robots
                    selected_robots = compatible_robots(1:na(t));
                end

                % Update the plan finish time
                new_coalition_finishing_time = max(robot_finish_time(selected_robots) .* (1 - pre_recharge(selected_robots)) + robot_pre_recharge_finish_time(selected_robots) .* pre_recharge(selected_robots) + Te_t_nf(t, Task(t).nf));

                % Check if the coalition finish time has changed
                if new_coalition_finishing_time ~= coalition_finishing_time
                    % Update the plan finish time
                    coalition_finishing_time = new_coalition_finishing_time;

                    % Set the auxiliary termination flag back to true
                    changes_in_finish_time = true;
                end
            end
        end

        % Initialize the joint introduced waiting time for the selected robots
        joint_introduced_waiting_time = 0;

        % Compute the joint introduced waiting time for the selected robots
        for robot = 1:length(selected_robots)
            % Get robot's id
            a = selected_robots(robot);

            % Get last used slot
            last_slot = length(Agent(a).queue) - 1;

            % Compute the waiting time for robot a including recharging time as waiting time
            joint_introduced_waiting_time = joint_introduced_waiting_time + getWaitingTime(a, t, v, last_slot, Agent, Task, Td_a_t_t, Te_t_nf, R, pre_recharge(a), coalition_finishing_time, tol) + pre_recharge(a) * Te_t_nf(R, Task(R).nf);
        end

        % Compute the incremented makespan
        if coalition_finishing_time > current_makespan
            introduced_makespan = coalition_finishing_time - current_makespan;
        else
            introduced_makespan = 0;
        end

        % Initialize the flag to update the best solution
        flag_update = false;

        % Check if the current solution is better than the best one
        switch v
        case {10, 11}
            switch best_solution_criteria
            case 1
                % If the joint introduced waiting time is the same as in the best solution
                if abs(joint_introduced_waiting_time - best_joint_introduced_waiting_time) < tol
                    % If the introduced makespan is less than in the best solution
                    if introduced_makespan - best_introduced_makespan < - tol
                        flag_update = true;
                    end
                % If the joint introduced waiting time is less than in the best solution
                elseif joint_introduced_waiting_time - best_joint_introduced_waiting_time < - tol
                    flag_update = true;
                end
            case 2
                % If the introduced makespan is the same as in the best solution
                if abs(introduced_makespan - best_introduced_makespan) < tol
                    % If the joint introduced waiting time is less than in the best solution
                    if joint_introduced_waiting_time - best_joint_introduced_waiting_time < - tol
                        flag_update = true;
                    end
                elseif introduced_makespan - best_introduced_makespan < - tol
                    flag_update = true;
                end
            end

            % Update the number of samples
            sample_counter = sample_counter - 1;
        otherwise
            flag_update = true;
        end

        if flag_update
            % Update the best solution
            best_robot_selection               = selected_robots;
            best_pre_recharge                  = pre_recharge;
            best_coalition_finishing_time      = coalition_finishing_time;
            best_joint_introduced_waiting_time = joint_introduced_waiting_time;
            best_introduced_makespan           = introduced_makespan;
        end

    end

    % Update the selected robots and the coalition finishing time
    selected_robots          = best_robot_selection;
    pre_recharge             = best_pre_recharge;
    coalition_finishing_time = best_coalition_finishing_time;
end

% Robot selection for group 2
function [selected_robots, task_plan, pre_recharge, plan_finish_time, joint_introduced_waiting_time, introduced_makespan] = robotSelectionGroup2(Agent, Task, t, v, pre_selected_robots, n_samples, best_solution_criteria, compatible_robots, robot_finish_time, robot_pre_recharge_finish_time, robot_remaining_battery, na, Td_a_t_t, Te_t_nf, R, tol)
    % Extract the coordination_point from the Agent structure
    coordination_point = [Agent.coordination_point];
    
    % Check if there are at least na(t) compatible robots
    if length(compatible_robots) < na(t)
        error(['There are not enough compatible robots to meet the requested coalition size for task ', Task(t).name, ' because some robots failed (negative battery)']);
    end

    % Check if the number of compatible robots is equal or greater than na
    if Task(t).nc < na(t)
        error(['Invalid scenario. There are no compatible robots enough to meet the requested coalition size for task ', Task(t).name]);
    end

    % Sort the compatible order in descending order according to the finish time of the last task
    [~, robot_order_idx] = sort(robot_finish_time, 'ascend');

    % Reorder compatible robots information
    compatible_robots = robot_order_idx(ismember(robot_order_idx, compatible_robots));

    % Create a 2*Task(t).nf long sequence of tasks alternating task t Task(t).f times and R once.
    sequence = repmat([repmat(t, [1, Task(t).f]), R], [1, Task(t).nc * ceil(Task(t).nf / (Task(t).f + 1))]);

    % Create task's plan matrix (size nc*nf)
    task_plan = [];
    for init_point = 1:Task(t).nc
        task_plan = [task_plan; sequence(init_point : init_point + Task(t).nf - 1)];
    end
    
    % Fix incorrect coalition size at any point of task's plan if needed
    for fragment = 1:Task(t).nf
        % Count the number of exceeding robots
        nx = sum(ismember(task_plan(:, fragment), t)) - na(t);

        % Remove exceeding robots
        for robot = 1:Task(t).nc
            % Check if there is still exceeding robots
            if not(nx)
                break;
            end

            % Check if in current position is task t and if there is a recharge before or after
            if task_plan(robot, fragment) == t
                % Check if this is the first fragment
                if fragment ~= 1
                    % Check if there is a recharge task before
                    if task_plan(robot, fragment - 1) == R
                        % Remove that robot from the coalition
                        task_plan(robot, fragment) = 0;

                        % Decrease the number of exceeding robots
                        nx = nx - 1;

                        continue;
                    end
                end
                
                % Check if this is the last fragment
                if fragment ~= Task(t).nf
                    % Check if there is a recharge task after
                    if task_plan(robot, fragment + 1) == R
                        % Remove that robot from the coalition
                        task_plan(robot, fragment) = 0;

                        % Decrease the number of exceeding robots
                        nx = nx - 1;
                        continue;
                    end
                end
            end
        end
    end

    % Remove recharges from the beginning
    for robot = 1:size(task_plan,1)
        for fragment = 1:Task(t).nf
            % Check if there is a recharge task in this fragment
            if task_plan(robot, fragment) == R
                % Remove that recharge task from the plan
                task_plan(robot, fragment) = 0;
            end

            % Check if there is a t task in this fragment
            if task_plan(robot, fragment) == t
                % Continue with the next robot
                break;
            end
        end
    end

    % Get task_plan row order according to the init moment of each row
    [~, row_order_idx] = sort(sum(task_plan ~= 0, 2), 'descend');

    % Reorder task_plan rows to have the shortest ones first
    task_plan = task_plan(row_order_idx, :);

    % Remove recharges from the end
    for robot = 1:size(task_plan,1)
        for fragment = Task(t).nf:-1:1
            % Check if there is a recharge task in this fragment
            if task_plan(robot, fragment) == R
                % Remove that recharge task from the plan
                task_plan(robot, fragment) = 0;
            end

            % Check if there is a t task in this fragment
            if task_plan(robot, fragment) == t
                % Continue with the next robot
                break;
            end
        end
    end

    % Remove empty rows from task plan (exceeding robots)
    task_plan = task_plan(any(task_plan, 2), :);

    % Initialize auxiliary duration_time and after_recharge_time variables
    duration_time       = zeros(1,size(task_plan,1));
    after_recharge_time = zeros(1,size(task_plan,1));

    % Find, from the end of each row, the time of the first recharge task (or the end of that row)
    for robot = 1:size(task_plan,1)
        % Get the first t task index in robot's row
        first_t_idx = find(task_plan(robot, :) == t, 1);

        % Compute the time from the end of the row until the first t task
        duration_time(robot) = (size(task_plan,2) - first_t_idx + 1) * Te_t_nf(t, Task(t).nf);

        % Get the first recharge or zero task index in robot's row
        first_R_idx = find(task_plan(robot, first_t_idx:end) == R | task_plan(robot, first_t_idx:end) == 0, 1) + first_t_idx - 1;
        
        % Compute the time from the end of the row until the first recharge or zero task
        if isempty(first_R_idx)
            after_recharge_time(robot) = 0;
        else
            after_recharge_time(robot) = (size(task_plan,2) - first_R_idx + 1) * Te_t_nf(t, Task(t).nf);
        end
        % Include the time needed to fly back to the recharge station
        after_recharge_time(robot) = after_recharge_time(robot) - Td_a_t_t(compatible_robots(robot), R, t + 1);
    end

    % Get the current makespan
    current_makespan = getMakespan(Agent);

    % Initialize the best robot selection and its associated introduced waiting time and incremented makespan
    best_robot_selection               = [];
    best_pre_recharge                  = [];
    best_plan_finish_time              = 0;
    best_joint_introduced_waiting_time = inf;
    best_introduced_makespan           = inf;

    switch v
    case {6, 10, 11}
        % Initialize the number of robot samples
        sample_counter_robots = n_samples;

        % Initialize the four parts of the task plan
        task_plan_base        = [];
        task_plan_middle_down = [];
        task_plan_middle_up   = [];
        task_plan_top         = [];

        % Initialize auxiliary duration time and after recharge time variables
        duration_time_base              = [];
        duration_time_middle_down       = [];
        duration_time_middle_up         = [];
        duration_time_top               = [];
        after_recharge_time_base        = [];
        after_recharge_time_middle_down = [];
        after_recharge_time_middle_up   = [];
        after_recharge_time_top         = [];

        % Separate the task plan into four parts according to their init points
        for robot = 1:size(task_plan,1)
            % Get the first t task index in robot's row
            first_t_idx = find(task_plan(robot, :) == t, 1);

            switch first_t_idx
            case 1
                task_plan_base = [task_plan_base; task_plan(robot, :)];
                duration_time_base = [duration_time_base, duration_time(robot)];
                after_recharge_time_base = [after_recharge_time_base, after_recharge_time(robot)];
            case 2
                task_plan_middle_down = [task_plan_middle_down; task_plan(robot, :)];
                duration_time_middle_down = [duration_time_middle_down, duration_time(robot)];
                after_recharge_time_middle_down = [after_recharge_time_middle_down, after_recharge_time(robot)];
            case 3
                task_plan_middle_up = [task_plan_middle_up; task_plan(robot, :)];
                duration_time_middle_up = [duration_time_middle_up, duration_time(robot)];
                after_recharge_time_middle_up = [after_recharge_time_middle_up, after_recharge_time(robot)];
            case 4
                task_plan_top = [task_plan_top; task_plan(robot, :)];
                duration_time_top = [duration_time_top, duration_time(robot)];
                after_recharge_time_top = [after_recharge_time_top, after_recharge_time(robot)];
            end
        end
    otherwise
        sample_counter_robots = 0;
        sample_counter_rows   = 0;
    end

    % Initialize flag to execute the next section in a while loop but only once if version is not 10 nor 11
    execute_next_section_robots = true;
    while execute_next_section_robots
        % Set the flag to false
        if sample_counter_robots == 0
            execute_next_section_robots = false;
        end

        % Select robots randomly from compatible ones (we are not checking if they have total flight time enough to execute the task and go back to the recharge station)
        if v == 6 || ((v == 10 || v == 11) && not(isempty(best_robot_selection)))
            % Randomize selected robots
            selected_robots = compatible_robots(sort(randperm(length(compatible_robots), size(task_plan,1))));
        elseif v == 12
            % Use the pre-selected robots
            selected_robots = pre_selected_robots(1:size(task_plan,1));
        else
            % To find the plan allocation that minimizes the finish time, assign compatible robot number 1 to the first row of the plan, number 2 to the second row, and so on.
            selected_robots = compatible_robots(1:size(task_plan,1));
        end

        % Initialize the number of row samples
        switch v
        case 6
            sample_counter_rows = 0;
        case {10, 11}
            sample_counter_rows = n_samples;
        end
        
        % Initialize flag to execute the next section in a while loop but only once if version is not 10 nor
        execute_next_section_rows = true;
        while execute_next_section_rows
            % Set the flag to false
            if sample_counter_rows == 0
                execute_next_section_rows = false;
            end

            % Sort rows from the task plan randomly
            if v == 6 || ((v == 10 || v == 11) && not(isempty(best_robot_selection)))
                % Sort the rows of each task plan part randomly
                base_random_order        = randperm(size(task_plan_base,        1));
                middle_down_random_order = randperm(size(task_plan_middle_down, 1));
                middle_up_random_order   = randperm(size(task_plan_middle_up,   1));
                top_random_order         = randperm(size(task_plan_top,         1));
                task_plan_base        = task_plan_base(base_random_order, :);
                task_plan_middle_down = task_plan_middle_down(middle_down_random_order, :);
                task_plan_middle_up   = task_plan_middle_up(middle_up_random_order, :);
                task_plan_top         = task_plan_top(top_random_order, :);
                duration_time_base        = duration_time_base(base_random_order);
                duration_time_middle_down = duration_time_middle_down(middle_down_random_order);
                duration_time_middle_up   = duration_time_middle_up(middle_up_random_order);
                duration_time_top         = duration_time_top(top_random_order);
                after_recharge_time_base        = after_recharge_time_base(base_random_order);
                after_recharge_time_middle_down = after_recharge_time_middle_down(middle_down_random_order);
                after_recharge_time_middle_up   = after_recharge_time_middle_up(middle_up_random_order);
                after_recharge_time_top         = after_recharge_time_top(top_random_order);

                % Concatenate the four parts of the task plan again
                task_plan = [task_plan_base; task_plan_middle_down; task_plan_middle_up; task_plan_top];
                duration_time = [duration_time_base, duration_time_middle_down, duration_time_middle_up, duration_time_top];
                after_recharge_time = [after_recharge_time_base, after_recharge_time_middle_down, after_recharge_time_middle_up, after_recharge_time_top];
            end

            % Initialize pre-recharge flag
            pre_recharge = [robot_remaining_battery(selected_robots) < duration_time - after_recharge_time];

            % Initialize the plan finish time as max(row_finish_time) with row_finish_time = (robot_finish_time + duration_time) * (1 - pre_recharge) + (robot_pre_recharge_finish_time + duration_time) * pre_recharge
            plan_finish_time = max(robot_finish_time(selected_robots) .* (1 - pre_recharge) + robot_pre_recharge_finish_time(selected_robots) .* pre_recharge + duration_time);

            % Iterate with pre-recharges until the plan_finish_time doesn't change
            changes_in_finish_time = true;

            % Iterate until the estimated finish time doesn't change
            while changes_in_finish_time
                % Set the auxiliary termination flag to false
                changes_in_finish_time = false;

                % Compute pre-recharge flags according to plan_finish_time:
                % - if (plan_finish_time - after_recharge_time > robot_finish_time + robot_remaining_battery) then pre_recharge = 1 because of the Tw,
                % - but if coordination_point ~= 0 then pre_recharge = 0 as we can wait in a recharge task,
                % - unless pre_recharge was already 1, that means that we need a recharge even without waiting.
                new_pre_recharge = [((plan_finish_time - after_recharge_time > robot_finish_time(selected_robots) + robot_remaining_battery(selected_robots)) & coordination_point(selected_robots) == 0) | pre_recharge];

                % Check if there are changes in the pre-recharge flags
                if any(new_pre_recharge ~= pre_recharge)
                    % Update pre-recharge flags
                    pre_recharge = new_pre_recharge;

                    % Update the plan finish time
                    new_plan_finish_time = max(robot_finish_time(selected_robots) .* (1 - pre_recharge) + robot_pre_recharge_finish_time(selected_robots) .* pre_recharge + duration_time);

                    % Check if the plan finish time has changed
                    if new_plan_finish_time ~= plan_finish_time
                        % Update the plan finish time
                        plan_finish_time = new_plan_finish_time;

                        % Set the auxiliary termination flag back to true
                        changes_in_finish_time = true;
                    end
                end
            end

            % Initialize the joint introduced waiting time for the selected robots
            joint_introduced_waiting_time = 0;

            % Compute the plan's init time
            plan_init_time = plan_finish_time - Task(t).nf * Te_t_nf(t, Task(t).nf);

            % Compute the joint introduced waiting time for the selected robots
            for robot = 1:length(selected_robots)
                % Get robot's id
                a = selected_robots(robot);

                % Get last used slot
                last_slot = length(Agent(a).queue) - 1;

                % Get the first t task index in robot's row
                first_t_idx = find(task_plan(robot, :) == t, 1);

                % Compute the finish time of the first t task
                first_t_finish_time = plan_init_time + first_t_idx * Te_t_nf(t, Task(t).nf);

                % Compute the waiting time for robot a including recharging time as waiting time
                joint_introduced_waiting_time = joint_introduced_waiting_time + getWaitingTime(a, t, v, last_slot, Agent, Task, Td_a_t_t, Te_t_nf, R, pre_recharge(robot), first_t_finish_time, tol) + pre_recharge(robot) * Te_t_nf(R, Task(R).nf);
            end

            % Compute the incremented makespan
            if plan_finish_time > current_makespan
                introduced_makespan = plan_finish_time - current_makespan;
            else
                introduced_makespan = 0;
            end

            % Initialize the flag to update the best solution
            flag_update = false;

            % Check if the current solution is better than the best one
            switch v
            case {10, 11}
                switch best_solution_criteria
                case 1
                    % If the joint introduced waiting time is the same as in the best solution
                    if abs(joint_introduced_waiting_time - best_joint_introduced_waiting_time) < tol
                        % If the introduced makespan is less than in the best solution
                        if introduced_makespan - best_introduced_makespan < - tol
                            flag_update = true;
                        end
                    % If the joint introduced waiting time is less than in the best solution
                    elseif joint_introduced_waiting_time - best_joint_introduced_waiting_time < - tol
                        flag_update = true;
                    end
                case 2
                    % If the introduced makespan is the same as in the best solution
                    if abs(introduced_makespan - best_introduced_makespan) < tol
                        % If the joint introduced waiting time is less than in the best solution
                        if joint_introduced_waiting_time - best_joint_introduced_waiting_time < - tol
                            flag_update = true;
                        end
                    elseif introduced_makespan - best_introduced_makespan < - tol
                        flag_update = true;
                    end
                end

                % Update the number of samples
                sample_counter_rows = sample_counter_rows - 1;
            otherwise
                flag_update = true;
            end

            if flag_update
                % Update the best solution
                best_robot_selection               = selected_robots;
                best_pre_recharge                  = pre_recharge;
                best_plan_finish_time              = plan_finish_time;
                best_joint_introduced_waiting_time = joint_introduced_waiting_time;
                best_introduced_makespan           = introduced_makespan;
            end
        end

        % Update the number of samples
        switch v
        case {10, 11}
            sample_counter_robots = sample_counter_robots - 1;
        end
    end

    % Update the selected robots and the coalition finishing time
    selected_robots  = best_robot_selection;
    pre_recharge     = best_pre_recharge;
    plan_finish_time = best_plan_finish_time;
end

% Allocate task group 1
% Initializer for sparseXATS or small-scale tests: function [Agent, Sta1s1a2s2] = allocateTaskGroup1(Agent, Task, t, v, selected_robots, pre_recharge, coalition_finishing_time, na, Td_a_t_t, Te_t_nf, R, Sta1s1a2s2, tol, heuristicTaskAllocator_flag, small_scale_tests_flag, repair_flag)
% Plan repair: function [Agent, Synchs] = allocateTaskGroup1(Agent, Task, t, v, selected_robots, pre_recharge, coalition_finishing_time, na, Td_a_t_t, Te_t_nf, R, Synchs, tol, heuristicTaskAllocator_flag, small_scale_tests_flag, repair_flag)
% Other: function [Agent] = allocateTaskGroup1(Agent, Task, t, v, selected_robots, pre_recharge, coalition_finishing_time, na, Td_a_t_t, Te_t_nf, R, tol, heuristicTaskAllocator_flag, small_scale_tests_flag, repair_flag)
function [Agent, Synchs] = allocateTaskGroup1(Agent, Task, t, v, selected_robots, pre_recharge, coalition_finishing_time, na, Td_a_t_t, Te_t_nf, R, Synchs, tol, heuristicTaskAllocator_flag, small_scale_tests_flag, repair_flag)
    % Initialize the selected robots and slots lists (synchronization constraints)
    coalition_robot = [];
    coalition_slot  = [];

    % Update selected robots' queue and times
    for robot = 1:length(selected_robots)
        % Get robot's id
        a = selected_robots(robot);

        % Add the robot to the coalition (synchronization constraints)
        coalition_robot = [coalition_robot, a];

        % Get last used slot
        last_slot = length(Agent(a).queue) - 1;

        % Compute the waiting time for robot a
        waiting_time = getWaitingTime(a, t, v, last_slot, Agent, Task, Td_a_t_t, Te_t_nf, R, pre_recharge(a), coalition_finishing_time, tol);

        % Check if robot needs a previous recharge
        if pre_recharge(a)
            % Add recharge task to the robot queue
            Agent(a).queue = [Agent(a).queue, R];

            % Update the last slot
            last_slot = last_slot + 1;

            % Update coordination point
            Agent = updateCoordinationPoint(Agent, Task, a, R, 1, last_slot, na, R);

            % Check if there is a delay in this slot
            if v == 13
                try
                    delay = Agent(a).delay_s(last_slot);
                catch
                    delay = 0;
                end
            else
                delay = 0;
            end

            % Update robot times (here there are a few extra "- 1" in Agent.queue compared with the code above because we already added the recharge task to the queue)
            Agent(a).Td_s(last_slot)    = Td_a_t_t(a, R, Agent(a).queue(last_slot - 1 + 1) + 1);
            Agent(a).Te_s(last_slot)    = Te_t_nf(R, Task(R).nf);
            Agent(a).tfin_s(last_slot + 1)  = Agent(a).tfin_s(last_slot - 1 + 1) + delay + Agent(a).Td_s(last_slot) + Agent(a).Te_s(last_slot);
            Agent(a).ac_Ft_s(last_slot + 1) = Agent(a).Td_s(last_slot);
            
            % If last task was not a recharge, add also the last accumulated flight time
            if Agent(a).queue(last_slot - 1 + 1) ~= R
                Agent(a).ac_Ft_s(last_slot + 1) = Agent(a).ac_Ft_s(last_slot + 1) + Agent(a).ac_Ft_s(last_slot - 1 + 1);
            end

            if Agent(a).ac_Ft_s(last_slot + 1) > Agent(a).Ft - Agent(a).Ft_saf
                error(['Fail while allocating task ', num2str(t - 1),'. Insufficient flight time. This scenario can''t be solved or this algorithm can''t find a valid solution']);
            end
        end

        % Add the task to the robot queue
        Agent(a).queue = [Agent(a).queue, t];

        % Update the last slot
        last_slot = last_slot + 1;

        % Apply waiting time according to the coordination point
        if Agent(a).coordination_point ~= 0
            % Wait in the last recharge task
            Agent(a).Tw_s(Agent(a).coordination_point) = waiting_time;
            waiting_time = 0;

            % Update tfin and acFt from slot coordination_point(a) to last_slot
            Agent = updateTfinAcFt(Agent, a, v, last_slot, R);
        end

        % Update coordination point
        Agent = updateCoordinationPoint(Agent, Task, a, t, 1, last_slot, na, R);

        % Save the slot of the robot (synchronization constraints)
        coalition_slot = [coalition_slot, last_slot];

        % Check if there is a delay in this slot
        if v == 13
            try
                delay = Agent(a).delay_s(last_slot);
            catch
                delay = 0;
            end
        else
            delay = 0;
        end

        % Update robot times (here there are a few extra "- 1" in Agent.queue compared with the code above because we already added the new task to the queue)
        Agent(a).Td_s(last_slot)    = Td_a_t_t(a, t, Agent(a).queue(last_slot - 1 + 1) + 1);
        Agent(a).Tw_s(last_slot)    = waiting_time;
        Agent(a).Te_s(last_slot)    = Te_t_nf(t, Task(t).nf);
        Agent(a).tfin_s(last_slot + 1)  = Agent(a).tfin_s(last_slot - 1 + 1) + delay + Agent(a).Td_s(last_slot) + Agent(a).Tw_s(last_slot) + Agent(a).Te_s(last_slot);
        Agent(a).ac_Ft_s(last_slot + 1) = delay + Agent(a).Td_s(last_slot) + Agent(a).Tw_s(last_slot) + Agent(a).Te_s(last_slot);
        % If last task was not a recharge, add also the last accumulated flight time
        if Agent(a).queue(last_slot - 1 + 1) ~= R
            Agent(a).ac_Ft_s(last_slot + 1) = Agent(a).ac_Ft_s(last_slot + 1) + Agent(a).ac_Ft_s(last_slot - 1 + 1);
        end

        if Agent(a).ac_Ft_s(last_slot + 1) > Agent(a).Ft - Agent(a).Ft_saf
            error(['Fail while allocating task ', num2str(t - 1),'. Insufficient flight time. This scenario can''t be solved or this algorithm can''t find a valid solution']);
        end
    end

    % Synchronization constraints: ns(t,a1,s1) = (na(t) - 1) * x(a1,t,s1)
    for r1 = 1:length(coalition_robot)
        % Data from the first robot
        a1 = coalition_robot(r1);
        s1 = coalition_slot(r1);
    
        % Set synchronization constraint
        for r2 = 1:length(coalition_robot)
            % Data from the second robot
            a2 = coalition_robot(r2);
            s2 = coalition_slot(r2);
    
            if a1 ~= a2
                % Synchronization constraint
                if heuristicTaskAllocator_flag || small_scale_tests_flag
                    Sta1s1a2s2(t - 1, a1, s1, a2, s2) = 1;
                elseif repair_flag
                    Synchs = [Synchs, [a1; s1; a2; s2]];
                end
            end
        end
    end
end

% Allocate task for group 2
% Initializer for sparseXATS or small-scale tests: function [Agent, Sta1s1a2s2, Rta1s1a2s2] = allocateTaskGroup2(Agent, Task, t, v, selected_robots, task_plan, pre_recharge, plan_finish_time, na, Td_a_t_t, Te_t_nf, R, Sta1s1a2s2, Rta1s1a2s2, tol, heuristicTaskAllocator_flag, small_scale_tests_flag, repair_flag)
% Plan repair: function [Agent, Synchs, Relays] = allocateTaskGroup2(Agent, Task, t, v, selected_robots, task_plan, pre_recharge, plan_finish_time, na, Td_a_t_t, Te_t_nf, R, Synchs, Relays, tol, heuristicTaskAllocator_flag, small_scale_tests_flag, repair_flag)
% Other: function [Agent] = allocateTaskGroup2(Agent, Task, t, v, selected_robots, task_plan, pre_recharge, plan_finish_time, na, Td_a_t_t, Te_t_nf, R, tol, heuristicTaskAllocator_flag, small_scale_tests_flag, repair_flag)
function [Agent, Synchs, Relays] = allocateTaskGroup2(Agent, Task, t, v, selected_robots, task_plan, pre_recharge, plan_finish_time, na, Td_a_t_t, Te_t_nf, R, Synchs, Relays, tol, heuristicTaskAllocator_flag, small_scale_tests_flag, repair_flag)
    % Initialize variable to note slots used for task's plan (used to create relays and synchronizations constraints)
    if heuristicTaskAllocator_flag || small_scale_tests_flag || repair_flag
        slot_map = zeros(size(task_plan));
    end

    % Assign the task's plan to the compatible robots
    for robot = 1:size(task_plan,1)
        % Get robot's id
        a = selected_robots(robot);

        % Get last used slot
        last_slot = length(Agent(a).queue) - 1;

        % Get the finish time of the first fragment
        coalition_finishing_time = plan_finish_time - (Task(t).nf - 1) * Te_t_nf(t, Task(t).nf);

        % Assign a robot's fragments
        for fragment = 1:Task(t).nf
            % Check if there is a t or recharge task in this fragment
            if task_plan(robot, fragment) == R
                % Set pre-recharge flag to one
                pre_recharge(robot) = 1;
            elseif task_plan(robot, fragment) ~= 0
                % Compute the waiting time for robot a
                waiting_time = getWaitingTime(a, task_plan(robot, fragment), v, last_slot, Agent, Task, Td_a_t_t, Te_t_nf, R, pre_recharge(robot), coalition_finishing_time, tol);

                % Check if robot needs a previous recharge
                if pre_recharge(robot)
                    % Add recharge task to the robot queue
                    Agent(a).queue = [Agent(a).queue, R];

                    % Set pre-recharge flag to zero
                    pre_recharge(robot) = 0;

                    % Update the last slot
                    last_slot = last_slot + 1;

                    % Update coordination point
                    Agent = updateCoordinationPoint(Agent, Task, a, R, 2, last_slot, na, R);

                    % Check if there is a delay in this slot
                    if v == 13
                        try
                            delay = Agent(a).delay_s(last_slot);
                        catch
                            delay = 0;
                        end
                    else
                        delay = 0;
                    end

                    % Update robot times (here there are a few extra "- 1" in Agent.queue compared with the code above because we already added the recharge task to the queue)
                    Agent(a).Td_s(last_slot)    = Td_a_t_t(a, R, Agent(a).queue(last_slot - 1 + 1) + 1);
                    Agent(a).Te_s(last_slot)    = Te_t_nf(R, Task(R).nf);
                    Agent(a).tfin_s(last_slot + 1)  = Agent(a).tfin_s(last_slot - 1 + 1) + delay + Agent(a).Td_s(last_slot) + Agent(a).Te_s(last_slot);
                    Agent(a).ac_Ft_s(last_slot + 1) = Agent(a).Td_s(last_slot);
                    % If last task was not a recharge, add also the last accumulated flight time
                    if Agent(a).queue(last_slot - 1 + 1) ~= R
                        Agent(a).ac_Ft_s(last_slot + 1) = Agent(a).ac_Ft_s(last_slot + 1) + Agent(a).ac_Ft_s(last_slot - 1 + 1);
                    end

                    if Agent(a).ac_Ft_s(last_slot + 1) > Agent(a).Ft - Agent(a).Ft_saf
                        error(['Fail while allocating task ', num2str(t - 1),'. Insufficient flight time. This scenario can''t be solved or this algorithm can''t find a valid solution']);
                    end
                end

                % Add the task to the robot queue
                Agent(a).queue = [Agent(a).queue, task_plan(robot, fragment)];

                % Update the last slot
                last_slot = last_slot + 1;

                % Apply waiting time according to the coordination point
                if Agent(a).coordination_point ~= 0
                    % Wait in the last recharge task
                    Agent(a).Tw_s(Agent(a).coordination_point) = waiting_time;
                    waiting_time = 0;

                    % Update tfin and acFt from slot coordination_point(a) to last_slot
                    Agent = updateTfinAcFt(Agent, a, v, last_slot, R);
                end

                % Update coordination point
                Agent = updateCoordinationPoint(Agent, Task, a, t, 2, last_slot, na, R);

                % Check if there is a delay in this slot
                if v == 13
                    try
                        delay = Agent(a).delay_s(last_slot);
                    catch
                        delay = 0;
                    end
                else
                    delay = 0;
                end

                % Update robot times (here there are a few extra "- 1" in Agent.queue compared with the code above because we already added the new task to the queue)
                Agent(a).Td_s(last_slot)    = Td_a_t_t(a, task_plan(robot, fragment), Agent(a).queue(last_slot - 1 + 1) + 1);
                Agent(a).Tw_s(last_slot)    = waiting_time;
                Agent(a).Te_s(last_slot)    = Te_t_nf(task_plan(robot, fragment), Task(task_plan(robot, fragment)).nf);
                Agent(a).tfin_s(last_slot + 1)  = Agent(a).tfin_s(last_slot - 1 + 1) + delay + Agent(a).Td_s(last_slot) + Agent(a).Tw_s(last_slot) + Agent(a).Te_s(last_slot);
                Agent(a).ac_Ft_s(last_slot + 1) = delay + Agent(a).Td_s(last_slot) + Agent(a).Tw_s(last_slot) + Agent(a).Te_s(last_slot);
                % If last task was not a recharge, add also the last accumulated flight time
                if Agent(a).queue(last_slot - 1 + 1) ~= R
                    Agent(a).ac_Ft_s(last_slot + 1) = Agent(a).ac_Ft_s(last_slot + 1) + Agent(a).ac_Ft_s(last_slot - 1 + 1);
                end

                if Agent(a).ac_Ft_s(last_slot + 1) > Agent(a).Ft - Agent(a).Ft_saf
                    error(['Fail while allocating task ', num2str(t - 1),'. Insufficient flight time. This scenario can''t be solved or this algorithm can''t find a valid solution']);
                end

                % Add the fragment to the slot_map (used to create relays and synchronizations constraints)
                if heuristicTaskAllocator_flag || small_scale_tests_flag || repair_flag
                    slot_map(robot, fragment) = last_slot;
                end
            end

            % Compute the finish time for the next fragment
            coalition_finishing_time = coalition_finishing_time + Te_t_nf(t, Task(t).nf);
        end
    end

    if heuristicTaskAllocator_flag || small_scale_tests_flag || repair_flag
        % Create relays and synchronizations constraints
        for fragment = 1:Task(t).nf
            % Synchronization constraints
            % Select a robot from fragment column
            for r1 = 1:size(task_plan,1)
                % Check if there is a t task assigned to this robot and fragment
                if task_plan(r1, fragment) == t
                    % Get robot's id
                    a1 = selected_robots(r1);
        
                    % Get fragment's slot
                    s1 = slot_map(r1, fragment);
        
                    % Select another robot from fragment column
                    for r2 = 1:size(task_plan,1)
                        % Check if there is a t task assigned to this robot and fragment
                        if task_plan(r2, fragment) == t
                            % Get robot's id
                            a2 = selected_robots(r2);
        
                            % Get fragment's slot
                            s2 = slot_map(r2, fragment);
        
                            % Check if selected robots are the same
                            if a1 ~= a2
                                % Set synchronization constraint
                                if heuristicTaskAllocator_flag || small_scale_tests_flag
                                    Sta1s1a2s2(t - 1, a1, s1, a2, s2) = 1;
                                elseif repair_flag
                                    Synchs = [Synchs, [a1; s1; a2; s2]];
                                end
                            end
                        end
                    end
                end
            end
        
            % Relays constraints
            % Check if this is the last fragment
            if fragment ~= Task(t).nf
                % Find positions of t tasks in fragment column
                robot_idx = find(ismember([task_plan(:,fragment)], t));
        
                % Find positions of t tasks in next fragment column
                next_robot_idx = find(ismember([task_plan(:,fragment + 1)], t));
        
                for robot = 1:na(t)
                    % Get robot's indexes
                    r1 =      robot_idx(robot);
                    r2 = next_robot_idx(robot);
        
                    % Get robot's ids
                    a1 = selected_robots(r1);
                    a2 = selected_robots(r2);
        
                    % Get fragment's slots
                    s1 = slot_map(r1, fragment);
                    s2 = slot_map(r2, fragment + 1);
        
                    % Set relay constraints
                    if heuristicTaskAllocator_flag || small_scale_tests_flag
                        Rta1s1a2s2(t - 1, a1, s1, a2, s2) = 1;
                    elseif repair_flag
                        Relays = [Relays, [a1; s1; a2; s2]];
                    end
                end
            end
        end
    end
end