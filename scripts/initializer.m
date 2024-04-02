function [Agent, Task, allocation_order, S_R] = initializer(arg_1, arg_2, reward_coefficients, version, seed)
    % arg_1 and arg_2 should either be Agent and Task or a valid scenario_id and optionally execution id.
    % reward_coefficients is an integer to select the reward coefficients to use.
    % version is an integer to select the version of the algorithm to use:
    %   - 1: original version. Task allocation order: first task from group 1 and then from group 2.
    %   - 2: alternating version. Task allocation order: n tasks from group 1 with tasks from group 2 in between (begin and end with n tasks from group 1).
    %   - 3: TODO: sand grains algorithm version. Task allocation order: first tasks from group 2, then try to fill waiting time gaps with tasks from group 1.
    %   - 4: random version. Task allocation order: random.
    %   - 5: seed version. Task allocation order: input order.
    %   - 6: random random version. Task allocation order: random. Robot selection criteria: random.

    if nargin < 2
        arg_2 = [];
    end

    % Load Agent and Task
    if ischar(arg_1)
        scenario_id  = arg_1;
        execution_id = arg_2;

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
        scenario_id  = [];
        execution_id = [];
    end

    if nargin < 3 || isempty(reward_coefficients)
        reward_coefficients = 1;
    end

    if nargin < 4 || isempty(version)
        version = 1;
    end

    % Set Task.nf to 1 to fix old generated scenarios that don't have it
    for t = 1:length(Task)
        Task(t).nf = 1;
    end
    
    % Get  constant scenario values
    [Agent, Task, A, T, S, N, R, Td_a_t_t, Te_t_nf, H_a_t] = getConstantScenarioValues(Agent, Task);

    % Initialize main decision variables
    Sta1s1a2s2 = zeros(T-1,A,S,A,S);
    Rta1s1a2s2 = zeros(T-1,A,S,A,S);

    % Tolerance
    tol = 1e-6;

    % Initialize the structures fields that stores robots' queue and times
    for a = 1:A
        Agent(a).queue   = 0;                           % S + 1
        Agent(a).Td_s    = zeros(1,S);                  % S
        Agent(a).Tw_s    = zeros(1,S);                  % S
        Agent(a).Te_s    = zeros(1,S);                  % S
        Agent(a).tfin_s  = zeros(1,S+1);                % S + 1
        Agent(a).ac_Ft_s = [Agent(a).Ft_0, zeros(1,S)]; % S + 1
    end

    % Initialize number of robots selected to simultaneously execute each task
    na = [Task.N];

    % Update coalition size of the tasks according to their flexibility (not for relayable tasks)
    na(~[Task.N_hardness] & ~[Task.Relayability]) = 1;

    % Sort tasks by type and deadline. First non-decomposable, fragmentable and relayable (with Task(t).nf = 1) tasks (group 1), then relayable tasks with Task(t).nf != 1 (group 2).
    % Get indexes of (group 1) the non-decomposable, fragmentable and relayable (with Task(t).nf = 1) tasks (tf_idx ranges from 1 to T-1).
    tf_idx = find([Task(2:end).Relayability] == 0 | ([Task(2:end).Relayability] == 1 & [Task(2:end).nf] == 1));

    % Get indexes of (group 2) the relayable tasks with Task(t).nf != 1 (tr_idx ranges from 1 to T-1).
    tr_idx = find([Task(2:end).Relayability] == 1 & [Task(2:end).nf] ~= 1);

    % Adjust indexes to match the tasks' indexes
    tf_idx = tf_idx + 1;
    tr_idx = tr_idx + 1;

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
    otherwise
        alpha   = 1;
        beta    = 0;
        gamma   = 0;
        delta   = 0;
        epsilon = 0;
    end

    % Get maximum task deadline
    max_tmax = max([Task.tmax]);

    % Get maximum execution time
    max_Te = max([Task.Te]);

    % Get the tasks' reward: the lower the better
    for t = 2:T
        Task(t).reward = alpha * (Task(t).tmax / max_tmax) + beta * (Task(t).Te / max_Te) + gamma * (Task(t).nf / N) + delta * (Task(t).N / A) + epsilon * (Task(t).N / Task(t).nc);
    end

    % Sort (group 1) non-decomposable, fragmentable and relayable (with Task(t).nf = 1) tasks by deadline
    [~, tf_reward_idx] = sort([Task(tf_idx).reward]);

    % Sort (group 2) relayable tasks with Task(t).nf != 1 by deadline
    [~, tr_reward_idx] = sort([Task(tr_idx).reward]);

    % Get groups 1 and 2 internally ordered
    group_1 = tf_idx(tf_reward_idx);
    group_2 = tr_idx(tr_reward_idx);

    % Repeat each task t in group 1 Task(t).nf times consecutively
    group_1 = repelem(group_1, [Task(group_1).nf]);

    % Join indexed from both groups according to the selected version
    switch version
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
        allocation_order = allocation_order(:,randperm(length(allocation_order)));
    case 5
        % Allocate tasks in input order
        if nargin < 5 || isempty(seed)
            error('A seed must be provided to use the input version');
        end
        allocation_order = seed;
    end

    % Allocate tasks in the computed order
    for task = 1:length(allocation_order)
        % Get task's group
        group = allocation_order(2, task);

        % Allocate task from group 1
        if group == 1
            % Get task's id
            t = allocation_order(1, task);

            % Get the indexes of the compatible robots
            compatible_robots = find(ismember([Agent.type], Task(t).Hr));

            % Get the compatible robots' time information
            robot_finish_time              = inf * ones(1,A);
            robot_pre_recharge_finish_time = inf * ones(1,A);
            robot_remaining_battery        = zeros(1,A);
            for robot = 1:length(compatible_robots)
                a = compatible_robots(robot);
                last_slot = length(Agent(a).queue) - 1;
                last_task = Agent(a).queue(last_slot + 1);

                % Check if the robot has total flight time enough to execute the task and go back to the recharge station
                if Td_a_t_t(a, t, R + 1) + Te_t_nf(t, Task(t).nf) + Td_a_t_t(a, R, t + 1) < Agent(a).Ft - Agent(a).Ft_saf
                    % Compute agents finish time without a previous recharge
                    robot_finish_time(a) = Agent(a).tfin_s(last_slot + 1) + Td_a_t_t(a, t, last_task + 1);

                    % Compute agents finish time with a previous recharge
                    robot_pre_recharge_finish_time(a) = Agent(a).tfin_s(last_slot + 1) + Td_a_t_t(a, R, last_task + 1) + Te_t_nf(R, Task(R).nf) + Td_a_t_t(a, t, R + 1);

                    % Get the remaining battery of each compatible robot having into account the safety margin, the initial displacement to the task place and the return to the recharge station
                    robot_remaining_battery(a) = Agent(a).Ft - Agent(a).Ft_saf - Agent(a).ac_Ft_s(last_slot + 1) - Td_a_t_t(a, t, last_task + 1) - Td_a_t_t(a, R, t + 1);
                end
            end

            % Initialize pre-recharge flag
            pre_recharge = [robot_remaining_battery < Te_t_nf(t, Task(t).nf)];

            % Compute robots' finish time with the pre-recharge flags
            finish_time = robot_finish_time .* (1 - pre_recharge) + robot_pre_recharge_finish_time .* pre_recharge + Te_t_nf(t, Task(t).nf);

            % Sort the compatible order in descending order according to the finish time of the last task
            [~, sort_idx] = sort(finish_time, 'ascend');

            % Reorder compatible robots information
            compatible_robots = sort_idx(ismember(sort_idx, compatible_robots));

            % Check if there are at least na(t) compatible robots
            if sum([finish_time ~= Inf]) < na(t)
                error(['There are not enough compatible robots to meet the requested coalition size for task ', num2str(t)]);
            end

            % Select na(t) robots
            selected_robots = compatible_robots(1:na(t));

            % Select robots randomly from compatible and valid ones
            if version == 6
                % Pre-select robots with total flight time enough to execute the task and go back to the recharge station
                selected_robots = find(robot_finish_time ~= Inf);

                % Randomize selected robots
                selected_robots = selected_robots(randperm(length(selected_robots), na(t)));
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

                % Compute pre-recharge flags according to coalition_finishing_time: if (coalition_finishing_time > robot_finish_time + robot_remaining_battery) then pre_recharge = 1
                new_pre_recharge(selected_robots) = [coalition_finishing_time > robot_finish_time(selected_robots) + robot_remaining_battery(selected_robots)];

                % Check if there are changes in the pre-recharge flags
                if any(new_pre_recharge ~= pre_recharge)
                    % Update pre-recharge flags
                    pre_recharge = new_pre_recharge;

                    % Update the selected robots in case we didn't use the random version
                    if version ~= 6
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
                    % Get the finish time of the last selected robot
                    % coalition_finishing_time = finish_time(sort_idx(na(t)));

                    % Check if the coalition finish time has changed
                    if new_coalition_finishing_time ~= coalition_finishing_time
                        % Update the plan finish time
                        coalition_finishing_time = new_coalition_finishing_time;

                        % Set the auxiliary termination flag back to true
                        changes_in_finish_time = true;
                    end
                end
            end

            % Initialize the selected robots and slots lists
            coalition_robot = [];
            coalition_slot  = [];

            % Update selected robots' queue and times
            for robot = 1:na(t)
                % Get robot's id
                a = selected_robots(robot);

                % Add the robot to the coalition
                coalition_robot = [coalition_robot, a];

                % Get last used slot
                last_slot = length(Agent(a).queue) - 1;

                % Compute the waiting time for robot a
                waiting_time = getWaitingTime(a, t, last_slot, Agent, Task, Td_a_t_t, Te_t_nf, R, pre_recharge(a), coalition_finishing_time);

                % Check if robot needs a previous recharge
                if pre_recharge(a)
                    % Add recharge task to the robot queue
                    Agent(a).queue = [Agent(a).queue, R];

                    % Update the last slot
                    last_slot = last_slot + 1;

                    % Update robot times (here there are a few extra "- 1" in Agent.queue compared with the code above because we already added the recharge task to the queue)
                    Agent(a).Td_s(last_slot)    = Td_a_t_t(a, R, Agent(a).queue(last_slot - 1 + 1) + 1);
                    Agent(a).Tw_s(last_slot)    = waiting_time;
                    Agent(a).Te_s(last_slot)    = Te_t_nf(R, Task(R).nf);
                    Agent(a).tfin_s(last_slot + 1)  = Agent(a).tfin_s(last_slot - 1 + 1) + Agent(a).Td_s(last_slot) + Agent(a).Tw_s(last_slot) + Agent(a).Te_s(last_slot);
                    Agent(a).ac_Ft_s(last_slot + 1) = Agent(a).Td_s(last_slot);
                    % If last task was not a recharge, add also the last accumulated flight time
                    if Agent(a).queue(last_slot - 1 + 1) ~= R
                        Agent(a).ac_Ft_s(last_slot + 1) = Agent(a).ac_Ft_s(last_slot + 1) + Agent(a).ac_Ft_s(last_slot - 1 + 1);
                    end

                    if Agent(a).ac_Ft_s(last_slot + 1) > Agent(a).Ft - Agent(a).Ft_saf
                        error(['Fail while allocating task ', num2str(t),'. This scenario can''t be solved or this algorithm can''t find a valid solution']);
                    end

                    % Set waiting time to zero
                    waiting_time = 0;
                end

                % Add the task to the robot queue
                Agent(a).queue = [Agent(a).queue, t];

                % Update the last slot
                last_slot = last_slot + 1;

                % Save the slot of the robot
                coalition_slot = [coalition_slot, last_slot];

                % Update robot times (here there are a few extra "- 1" in Agent.queue compared with the code above because we already added the new task to the queue)
                Agent(a).Td_s(last_slot)    = Td_a_t_t(a, t, Agent(a).queue(last_slot - 1 + 1) + 1);
                Agent(a).Tw_s(last_slot)    = waiting_time;
                Agent(a).Te_s(last_slot)    = Te_t_nf(t, Task(t).nf);
                Agent(a).tfin_s(last_slot + 1)  = Agent(a).tfin_s(last_slot - 1 + 1) + Agent(a).Td_s(last_slot) + Agent(a).Tw_s(last_slot) + Agent(a).Te_s(last_slot);
                Agent(a).ac_Ft_s(last_slot + 1) = Agent(a).Td_s(last_slot) + Agent(a).Tw_s(last_slot) + Agent(a).Te_s(last_slot);
                % If last task was not a recharge, add also the last accumulated flight time
                if Agent(a).queue(last_slot - 1 + 1) ~= R
                    Agent(a).ac_Ft_s(last_slot + 1) = Agent(a).ac_Ft_s(last_slot + 1) + Agent(a).ac_Ft_s(last_slot - 1 + 1);
                end

                if Agent(a).ac_Ft_s(last_slot + 1) > Agent(a).Ft - Agent(a).Ft_saf
                    error(['Fail while allocating task ', num2str(t),'. This scenario can''t be solved or this algorithm can''t find a valid solution']);
                end
            end

            % Synchronization constraints: ns(t,a1,s1) = (na(t) - 1) * x(a1,t,s1)
            for r1 = 1:na(t)
                % Data from the first robot
                a1 = coalition_robot(r1);
                s1 = coalition_slot(r1);

                % Set synchronization constraint
                for r2 = 1:na(t)
                    % Data from the second robot
                    a2 = coalition_robot(r2);
                    s2 = coalition_slot(r2);

                    if a1 ~= a2
                        % Synchronization constraint
                        Sta1s1a2s2(t - 1, a1, s1, a2, s2) = 1;
                    end
                end
            end
        end

        % Allocate task from group 2
        if group == 2
            % Get task's id
            t = allocation_order(1, task);

            % Get the indexes of the compatible robots
            compatible_robots = find(ismember([Agent.type], Task(t).Hr));

            % Check if the number of compatible robots is equal or greater than na
            if Task(t).nc < na(t)
                error(['There is no compatible robot enough to meet the requested coalition size for task ', num2str(t)]);
            end

            % Get the compatible robots' time information
            robot_finish_time              = inf * ones(1,A);
            robot_pre_recharge_finish_time = inf * ones(1,A);
            robot_remaining_battery = zeros(1,A);
            for robot = 1:length(compatible_robots)
                a = compatible_robots(robot);
                last_slot = length(Agent(a).queue) - 1;
                last_task = Agent(a).queue(last_slot + 1);

                % Check if the robot has total flight time enough to execute the task and go back to the recharge station
                if Td_a_t_t(a, t, R + 1) + Te_t_nf(t, Task(t).nf) + Td_a_t_t(a, R, t + 1) < Agent(a).Ft - Agent(a).Ft_saf
                    % Compute agents finish time without a previous recharge
                    robot_finish_time(a) = Agent(a).tfin_s(last_slot + 1) + Td_a_t_t(a, t, last_task + 1);

                    % Compute agents finish time with a previous recharge
                    robot_pre_recharge_finish_time(a) = Agent(a).tfin_s(last_slot + 1) + Td_a_t_t(a, R, last_task + 1) + Te_t_nf(R, Task(R).nf) + Td_a_t_t(a, t, R + 1);

                    % Get the remaining battery of each compatible robot having into account the safety margin, the initial displacement to the task place and the return to the recharge station
                    robot_remaining_battery(a) = Agent(a).Ft - Agent(a).Ft_saf - Agent(a).ac_Ft_s(last_slot + 1) - Td_a_t_t(a, t, last_task + 1) - Td_a_t_t(a, R, t + 1);
                end
            end

            % Sort the compatible order in descending order according to the finish time of the last task
            [~, robot_order_idx] = sort(robot_finish_time, 'ascend');

            % Reorder compatible robots information
            compatible_robots = robot_order_idx(ismember(robot_order_idx, compatible_robots));
            % robot_finish_time = robot_finish_time(robot_order_idx);
            % robot_pre_recharge_finish_time = robot_pre_recharge_finish_time(robot_order_idx);
            % robot_remaining_battery = robot_remaining_battery(robot_order_idx);

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
                after_recharge_time(robot) = after_recharge_time(robot) - Td_a_t_t(a, R, t + 1);
            end

            % To find the plan allocation that minimizes the finish time, assign compatible robot number 1 to the first row of the plan, number 2 to the second row, and so on.
            % TODO: Algorithm to find the best plan allocation: robot assignment to minimize time loses (waiting times, recharges)
            % TODO: Random Sampling Robot Selection
            selected_robots = compatible_robots(1:size(task_plan,1));

            % Select robots randomly from compatible ones (we are not checking if they have total flight time enough to execute the task and go back to the recharge station)
            if version == 6
                % Randomize selected robots
                selected_robots = compatible_robots(randperm(length(compatible_robots), size(task_plan,1)));
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

                % Compute pre-recharge flags according to plan_finish_time: if (plan_finish_time - after_recharge_time > robot_finish_time + robot_remaining_battery) then pre_recharge = 1
                new_pre_recharge = [plan_finish_time - after_recharge_time > robot_finish_time(selected_robots) + robot_remaining_battery(selected_robots)];

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

            % Initialize variable to note slots used for task's plan
            slot_map = zeros(size(task_plan));

            % Compute the slot_map
            for robot = 1:size(task_plan,1)
                % Get robot's id
                a = selected_robots(robot);

                % Get last used slot
                last_slot = length(Agent(a).queue) - 1;

                % Adjust initial slot with pre-recharge information
                last_slot = last_slot + pre_recharge(robot);

                % Fill the slot map with the corresponding slots
                for fragment = 1:Task(t).nf
                    % Check if there is a t or R task assigned to this robot and fragment
                    if task_plan(robot, fragment) ~= 0
                        % Adjust last slot for this task
                        last_slot = last_slot + 1;

                        % Update slot information
                        slot_map(robot, fragment) = last_slot;
                    end
                end
            end

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
                                    Sta1s1a2s2(t - 1, a1, s1, a2, s2) = 1;
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
                        Rta1s1a2s2(t - 1, a1, s1, a2, s2) = 1;
                    end
                end
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
                        waiting_time = getWaitingTime(a, task_plan(robot, fragment), last_slot, Agent, Task, Td_a_t_t, Te_t_nf, R, pre_recharge(robot), coalition_finishing_time);

                        % Check if robot needs a previous recharge
                        if pre_recharge(robot)
                            % Add recharge task to the robot queue
                            Agent(a).queue = [Agent(a).queue, R];

                            % Set pre-recharge flag to zero
                            pre_recharge(robot) = 0;

                            % Update the last slot
                            last_slot = last_slot + 1;

                            % Update robot times (here there are a few extra "- 1" in Agent.queue compared with the code above because we already added the recharge task to the queue)
                            Agent(a).Td_s(last_slot)    = Td_a_t_t(a, R, Agent(a).queue(last_slot - 1 + 1) + 1);
                            Agent(a).Tw_s(last_slot)    = waiting_time;
                            Agent(a).Te_s(last_slot)    = Te_t_nf(R, Task(R).nf);
                            Agent(a).tfin_s(last_slot + 1)  = Agent(a).tfin_s(last_slot - 1 + 1) + Agent(a).Td_s(last_slot) + Agent(a).Tw_s(last_slot) + Agent(a).Te_s(last_slot);
                            Agent(a).ac_Ft_s(last_slot + 1) = Agent(a).Td_s(last_slot);
                            % If last task was not a recharge, add also the last accumulated flight time
                            if Agent(a).queue(last_slot - 1 + 1) ~= R
                                Agent(a).ac_Ft_s(last_slot + 1) = Agent(a).ac_Ft_s(last_slot + 1) + Agent(a).ac_Ft_s(last_slot - 1 + 1);
                            end

                            if Agent(a).ac_Ft_s(last_slot + 1) > Agent(a).Ft - Agent(a).Ft_saf
                                error(['Fail while allocating task ', num2str(t),'. This scenario can''t be solved or this algorithm can''t find a valid solution']);
                            end

                            % Set waiting time to zero
                            waiting_time = 0;
                        end

                        % Add the task to the robot queue
                        Agent(a).queue = [Agent(a).queue, task_plan(robot, fragment)];

                        % Update the last slot
                        last_slot = last_slot + 1;

                        % Update robot times (here there are a few extra "- 1" in Agent.queue compared with the code above because we already added the new task to the queue)
                        Agent(a).Td_s(last_slot)    = Td_a_t_t(a, task_plan(robot, fragment), Agent(a).queue(last_slot - 1 + 1) + 1);
                        Agent(a).Tw_s(last_slot)    = waiting_time;
                        Agent(a).Te_s(last_slot)    = Te_t_nf(task_plan(robot, fragment), Task(task_plan(robot, fragment)).nf);
                        Agent(a).tfin_s(last_slot + 1)  = Agent(a).tfin_s(last_slot - 1 + 1) + Agent(a).Td_s(last_slot) + Agent(a).Tw_s(last_slot) + Agent(a).Te_s(last_slot);
                        Agent(a).ac_Ft_s(last_slot + 1) = Agent(a).Td_s(last_slot) + Agent(a).Tw_s(last_slot) + Agent(a).Te_s(last_slot);
                        % If last task was not a recharge, add also the last accumulated flight time
                        if Agent(a).queue(last_slot - 1 + 1) ~= R
                            Agent(a).ac_Ft_s(last_slot + 1) = Agent(a).ac_Ft_s(last_slot + 1) + Agent(a).ac_Ft_s(last_slot - 1 + 1);
                        end

                        if Agent(a).ac_Ft_s(last_slot + 1) > Agent(a).Ft - Agent(a).Ft_saf
                            error(['Fail while allocating task ', num2str(t),'. This scenario can''t be solved or this algorithm can''t find a valid solution']);
                        end
                    end

                    % Compute the finish time for the next fragment
                    coalition_finishing_time = coalition_finishing_time + Te_t_nf(t, Task(t).nf);
                end
            end
        end
    end

    % Fill the empty tfin slots with the last tfin value
    for a = 1:A
        for slot = length(Agent(a).queue) + 1:S + 1
            Agent(a).tfin_s(slot) = Agent(a).tfin_s(slot - 1);
        end
    end

    % Check if initializer has used more than S slots
    initializer_S = 0;
    for a = 1:A
        if length(Agent(a).queue) - 1 > initializer_S
            initializer_S = length(Agent(a).queue);
        end
    end

    if initializer_S ~= S
        % Check if the size of Sta1s1a2s2 need to be corrected
        if size(Sta1s1a2s2, 3) < initializer_S
            Sta1s1a2s2(1, 1, initializer_S, 1, 1) = 0;
        end
        if size(Sta1s1a2s2, 5) < initializer_S
            Sta1s1a2s2(1, 1, 1, 1, initializer_S) = 0;
        end

        % Check if the size of Rta1s1a2s2 need to be corrected
        if size(Rta1s1a2s2, 3) < initializer_S
            Rta1s1a2s2(1, 1, initializer_S, 1, 1) = 0;
        end
        if size(Rta1s1a2s2, 5) < initializer_S
            Rta1s1a2s2(1, 1, 1, 1, initializer_S) = 0;
        end
    end

    % Reshape decision variables
    Sta1s1a2s2 = reshape(Sta1s1a2s2,1,[]);
    Rta1s1a2s2 = reshape(Rta1s1a2s2,1,[]);

    % Put all decision variables together
    S_R = [Sta1s1a2s2 Rta1s1a2s2];

    % Save na in Task structure
    for t = 1:length(Task)
        Task(t).na = na(t);
    end
end

% Compute the waiting time for robot a
function [waiting_time] = getWaitingTime(a, t, last_slot, Agent, Task, Td_a_t_t, Te_t_nf, R, pre_recharge, coalition_finishing_time)
    if pre_recharge
        waiting_time = coalition_finishing_time - Te_t_nf(t, Task(t).nf) - Td_a_t_t(a, t, R + 1) - Te_t_nf(R, Task(R).nf) - Td_a_t_t(a, R, Agent(a).queue(last_slot + 1) + 1) - Agent(a).tfin_s(last_slot + 1);
    else
        waiting_time = coalition_finishing_time - Te_t_nf(t, Task(t).nf) - Td_a_t_t(a, t, Agent(a).queue(last_slot + 1) + 1) - Agent(a).tfin_s(last_slot + 1);
    end
    if waiting_time < - 1e-6
        error(['Negative waiting time for robot ', num2str(a), ' and task ', num2str(t),'. This scenario can''t be solved or this algorithm can''t find a valid solution']);
    end
end