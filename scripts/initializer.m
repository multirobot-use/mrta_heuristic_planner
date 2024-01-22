function [sol, Agent, Task] = initializer(arg_1, arg_2, reward_coefficients, check_solution_flag, print_solution_flag)
    % arg_1 and arg_2 should either be Agent and Task or a valid scenario_id and optionally execution id.
    % reward_coefficients is an integer to select the reward coefficients to use.
    % print_solution_flag is a boolean to select if the solution should be printed or not.

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

    if nargin < 3
        reward_coefficients = 1;
    end

    if nargin < 4
        check_solution_flag = false;
    end

    if nargin < 5
        print_solution_flag = false;
    end

    % Set Task.nf to 1 to fix old generated scenarios that don't have it
    for t = 1:length(Task)
        Task(t).nf = 1;
    end
    
    % Get  constant scenario values
    [Agent, Task, A, T, S, N, R, Td_a_t_t, Te_t_nf, H_a_t] = getConstantScenarioValues(Agent, Task);

    % Initialize result
    sol  = [];
    fval = [];

    % Initialize main decision variables
    xats = zeros(A,(T+1),(S+1));
    nf_t = [Task.nf];
    Sta1s1a2s2 = zeros(T-1,A,S,A,S);
    Rta1s1a2s2 = zeros(T-1,A,S,A,S);

    % Tolerance
    tol = 1e-6;

    % Initialize the structures that stores robots' queue and times
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

    % Sort tasks by type and deadline. First non-decomposable, fragmentable and relayable (with Task(t).nf = 1) tasks, then relayable tasks with Task(t).nf != 1.
    % Get indexes of the non-decomposable, fragmentable and relayable (with Task(t).nf = 1) tasks (tf_idx ranges from 1 to T-1).
    tf_idx = find([Task(2:end).Relayability] == 0 | ([Task(2:end).Relayability] == 1 & [Task(2:end).nf] == 1));

    % Get indexes of the relayable tasks with Task(t).nf != 1 (tr_idx ranges from 1 to T-1).
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

    % Sort non-decomposable, fragmentable and relayable (with Task(t).nf = 1) tasks by deadline
    [~, tf_deadline_idx] = sort([Task(tf_idx).reward]);

    % Sort relayable tasks with Task(t).nf != 1 by deadline
    [~, tr_deadline_idx] = sort([Task(tr_idx).reward]);

    % Join indexed from both groups
    deadline_idx = [tf_idx(tf_deadline_idx), tr_idx(tr_deadline_idx)];

    % Assign non-decomposable, fragmentable and relayable (with Task(t).nf = 1) tasks in deadline order
    for task = 1:length(tf_deadline_idx)
        % Get task's id
        t = tf_idx(tf_deadline_idx(task));

        % Create the nf(t) groups
        for fragment = 1:Task(t).nf
            % Initialize the auxiliary finish time of each robot and the previous recharge flag
            finish_time = inf * ones(1,A);
            pre_recharge = zeros(1,A);

            % Get the length of each robot's queue
            used_slots = zeros(1,A);
            for a = 1:A
                used_slots(a) = length(Agent(a).queue) - 1;
            end

            % Compute the finish time for each robot (infinite if robot can't handle the task)
            for a = 1:A
                % Check if the robot is compatible with the task
                if ismember(Agent(a).type, Task(t).Hr)
                    % Get last used slot
                    last_slot = length(Agent(a).queue) - 1;

                    % Check if the robot has total flight time enough to execute the task and go back to the recharge station
                    if Td_a_t_t(a, t, Agent(a).queue(last_slot + 1) + 1) + Te_t_nf(t, Task(t).nf) + Td_a_t_t(a, R, t + 1) < Agent(a).Ft - Agent(a).Ft_saf
                        % Check if there is flight time left enough to execute the task and go back to the recharge station
                        if Agent(a).ac_Ft_s(last_slot + 1) + Td_a_t_t(a, t, Agent(a).queue(last_slot + 1) + 1) + Te_t_nf(t, Task(t).nf) + Td_a_t_t(a, R, t + 1) < Agent(a).Ft - Agent(a).Ft_saf
                            % Compute finish time without previous recharge
                            finish_time(a) = Agent(a).tfin_s(last_slot + 1) + Td_a_t_t(a, t, Agent(a).queue(last_slot + 1) + 1) + Te_t_nf(t, Task(t).nf);
                        % Check if there is flight time left enough to execute a recharge, and if the robot can fly from the recharge station, execute the task and go back
                        elseif Agent(a).ac_Ft_s(last_slot + 1) + Td_a_t_t(a, R, Agent(a).queue(last_slot + 1) + 1) < Agent(a).Ft - Agent(a).Ft_saf && Td_a_t_t(a, t, R + 1) + Te_t_nf(t, Task(t).nf) + Td_a_t_t(a, R, t + 1) < Agent(a).Ft - Agent(a).Ft_saf
                            % Compute finish time with previous recharge
                            finish_time(a) = Agent(a).tfin_s(last_slot + 1) + Td_a_t_t(a, R, Agent(a).queue(last_slot + 1) + 1) + Te_t_nf(R, Task(R).nf) + Td_a_t_t(a, t, R + 1) + Te_t_nf(t, Task(t).nf);
                            pre_recharge(a) = 1;
                        end
                    end
                end
            end

            % Create an auxiliary termination flag to select the na(t) earliest robots
            changes_in_finish_time = true;

            % Iterate until the selected robots don't change
            while changes_in_finish_time
                % Set the auxiliary termination flag to false
                changes_in_finish_time = false;

                % Sort the robots by finish time and by used slots
                [~, sort_idx] = sortrows([finish_time', used_slots'], [1, 2]);

                % Get the finish time of the last selected robot
                coalition_finishing_time = finish_time(sort_idx(na(t)));

                % Check if there are enough robots to execute the task
                % If coalition finishing time is infinite, scenario can't be solved or this algorithm can't find a valid solution.
                if coalition_finishing_time == inf
                    error(['Fail while allocating task ', num2str(t),'. This scenario can''t be solved or this algorithm can''t find a valid solution']);
                end

                % Check if selected robots can be synchronized
                for robot = 1:na(t)
                    % Get robot's id
                    a = sort_idx(robot);

                    % Get last used slot
                    last_slot = length(Agent(a).queue) - 1;

                    % Compute the waiting time for robot a
                    waiting_time = getWaitingTime(a, t, last_slot, Agent, Task, Td_a_t_t, Te_t_nf, R, pre_recharge, coalition_finishing_time);

                    % If the waiting time is negative, that means that the robot can't be used as a relay for the previous group
                    if waiting_time < - tol
                        % Update finish time for robot
                        finish_time(a) = inf;

                        % Set the auxiliary termination flag back to true
                        changes_in_finish_time = true;

                        continue;
                    end

                    % Check if robot does not contemplates a previous recharge (waiting time doesn't affect flight time because it will be made landed in recharge station)
                    if not(pre_recharge(a))
                        % Check if robot doesn't have flight time enough contemplating the waiting time
                        if not(Agent(a).ac_Ft_s(last_slot + 1) + Td_a_t_t(a, t, Agent(a).queue(last_slot + 1) + 1) + waiting_time + Te_t_nf(t, Task(t).nf) + Td_a_t_t(a, R, t + 1) < Agent(a).Ft - Agent(a).Ft_saf)
                            % Check if there is flight time left enough to execute a recharge, and if the robot can fly from the recharge station, execute the task and go back
                            if Agent(a).ac_Ft_s(last_slot + 1) + Td_a_t_t(a, R, Agent(a).queue(last_slot + 1) + 1) < Agent(a).Ft - Agent(a).Ft_saf && Td_a_t_t(a, t, R + 1) + Te_t_nf(t, Task(t).nf) + Td_a_t_t(a, R, t + 1) < Agent(a).Ft - Agent(a).Ft_saf
                                % Add a previous recharge to robot
                                pre_recharge(a) = 1;

                                % Update finish time for robot
                                finish_time(a) = Agent(a).tfin_s(last_slot + 1) + Td_a_t_t(a, R, Agent(a).queue(last_slot + 1) + 1) + Te_t_nf(R, Task(R).nf) + Td_a_t_t(a, t, R + 1) + Te_t_nf(t, Task(t).nf);
                            else
                                % Update finish time for robot
                                finish_time(a) = inf;
                            end

                            % Set the auxiliary termination flag back to true
                            changes_in_finish_time = true;

                            % Restart the robot selection
                            break;
                        end
                    end
                end
            end

            % Initialize the selected robots and slots lists
            coalition_robot = [];
            coalition_slot  = [];

            % Update selected robots' queue and times
            for robot = 1:na(t)
                % Get robot's id
                a = sort_idx(robot);

                % Add the robot to the coalition
                coalition_robot = [coalition_robot, a];

                % Get last used slot
                last_slot = length(Agent(a).queue) - 1;

                % Compute the waiting time for robot a
                waiting_time = getWaitingTime(a, t, last_slot, Agent, Task, Td_a_t_t, Te_t_nf, R, pre_recharge, coalition_finishing_time);

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
    end

    % Assign relayable tasks in deadline order
    for task = 1:length(tr_deadline_idx)
        % Get task's id
        t = tr_idx(tr_deadline_idx(task));

        % Create a 2*Task(t).nf long sequence of tasks alternating task t Task(t).f times and R once.
        sequence = repmat([repmat(t, [1, Task(t).f]), R], [1, Task(t).nc * ceil(Task(t).nf / (Task(t).f + 1))]);

        % Create task's plan matrix (size nc*nf)
        task_plan = [];
        for init_point = 1:Task(t).nc
            task_plan = [task_plan; sequence(init_point : init_point + Task(t).nf - 1)];
        end

        % Get the indexes of the compatible robots
        compatible_robots = find(ismember([Agent.type], Task(t).Hr));

        % Check if the number of compatible robots is equal or greater than na
        if Task(t).nc < na(t)
            error(['There is no compatible robot enough to meet the requested coalition size for task ', num2str(t)]);
        end
        
        % TODO: check if there's a way to avoid this step by adjusting the numbers and the rounding
        % TODO: check if there's a way to anticipate empty rows before this and the following steps
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

        % Create an auxiliary termination flag to get rid of empty rows
        changes_in_task_plan = true;

        % Remove initial and final recharge tasks and split workload with empty rows
        while changes_in_task_plan
            % Set the auxiliary termination flag to false
            changes_in_task_plan = false;

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

            % Check if this function is being called from sparseXATS or solver. If so, we need to fit queues in S slots.
            if any(strcmp({dbstack().name}, 'sparseXATS') | strcmp({dbstack().name}, 'solver'))
                % Check if there are empty rows in the task's plan
                if not(all(any(task_plan, 2)))
                    % Get the index of the first empty row
                    empty_row_idx = find(not(any(task_plan, 2)), 1);

                    % Get the index of the row with the most non-zero elements
                    [max_row_slots, max_row_idx] = max(sum(task_plan ~= 0, 2));

                    % Make sure that there are more than one non-zero element in that row
                    if max_row_slots < 2
                        break;
                    end

                    % Find the first non-zero element in the "max_row_idx" row
                    first_non_zero = find(task_plan(max_row_idx, :), 1);

                    % Split the workload of the row with the most non-zero elements
                    task_plan(empty_row_idx, first_non_zero + ceil(max_row_slots / 2) : Task(t).nf) = task_plan(max_row_idx, first_non_zero + ceil(max_row_slots / 2) : Task(t).nf);
                    task_plan(max_row_idx, first_non_zero + ceil(max_row_slots / 2) : Task(t).nf) = 0;

                    % Set the auxiliary termination flag back to true
                    changes_in_task_plan = true;
                end
            else
                % Remove empty rows from task plan (exceeding robots)
                task_plan = task_plan(any(task_plan, 2), :);
            end
        end

        % Get the length of each robot's queue
        used_slots = zeros(1, length(compatible_robots));
        for robot = 1:length(compatible_robots)
            % Get robot's id
            a = compatible_robots(robot);

            % Get number of used slots
            used_slots(robot) = length(Agent(a).queue) - 1;
        end

        % From compatible robots, select "size(task_plan,1)" robots with the least used slots
        % Note: to have into account also the finish time in the selection, we'll need to compute all possible combinations of task_plan to robot assignment, and so the algorithm wont be polynomial anymore.
        % Note: for similar reasons, we can't have into account the pre-recharge slot in the selection.
        [~, sort_idx] = sortrows(used_slots');
        selected_robots = compatible_robots(sort_idx(1:size(task_plan,1)));

        % Get task_plan row order according to the length of each row
        [~, row_order_idx] = sort(sum(task_plan ~= 0, 2), 'descend');

        % Reorder task_plan rows to have the longest ones first
        task_plan = task_plan(row_order_idx, :);

        % Initialize pre-recharge flag
        pre_recharge = zeros(1,A);

        % Initialize auxiliary finish time variable
        finish_time = zeros(1,size(task_plan,1));

        % Create an auxiliary termination flag to select the needed robots according to the plan
        changes_in_finish_time = true;

        % Iterate until the estimated finish time doesn't change
        while changes_in_finish_time
            % Set the auxiliary termination flag to false
            changes_in_finish_time = false;

            % Get task finish time having into account pre-recharges
            for robot = 1:size(task_plan,1)
                % Get robot's id
                a = selected_robots(robot);

                % Get last used slot
                last_slot = length(Agent(a).queue) - 1;

                % Compute the time at which each robot could be done executing its task_plan having into account last but not beginning 0 tasks: pre-recharge if needed, displacement time, and then "nf - find(task_plan(robot,:), 1) - 1" time Te(t,nf) to synch the end of the plan but save the time with empty steps of the plan at the beginning
                % This allows the plan to start before some of the selected robots end executing their previous task if those robots' plans start with some zeros.
                finish_time(robot) = Agent(a).tfin_s(last_slot + 1) + pre_recharge(a) * (Td_a_t_t(a, R, Agent(a).queue(last_slot + 1) + 1) + Te_t_nf(R, Task(R).nf) + Td_a_t_t(a, t, R + 1)) + not(pre_recharge(a)) * (Td_a_t_t(a, t, Agent(a).queue(last_slot + 1) + 1)) + (Task(t).nf - (find(task_plan(robot,:), 1) - 1)) * Te_t_nf(t, Task(t).nf);
            end

            % Get the init time of the first fragment
            plan_init_time = max(finish_time) - Task(t).nf * Te_t_nf(t, Task(t).nf);

            % Compute pre_recharge and finish_time for the compatible robots
            for robot = 1:size(task_plan,1)
                % Get robot's id
                a = selected_robots(robot);

                % Check if robot does not contemplates a previous recharge (waiting time doesn't affect flight time because it will be made landed in recharge station)
                if not(pre_recharge(a))
                    % Get last used slot
                    last_slot = length(Agent(a).queue) - 1;

                    % Compute the synch time for robot a. Note: as there may be some zeros at the beginning of this robot's plan, synch_time should the the end time of its first non-zero task in task_plan
                    synch_time = getWaitingTime(a, t, last_slot, Agent, Task, Td_a_t_t, Te_t_nf, R, pre_recharge, plan_init_time + find(task_plan(robot,:), 1) * Te_t_nf(t, Task(t).nf));

                    % Get index of the first recharge task in robot's task plan
                    recharge_idx = find(task_plan(robot, :) == R, 1);

                    % Check if robot has battery enough to execute its queue until the first recharge task included
                    if isempty(recharge_idx)
                        % Get the idx of the last t task in robot's task plan
                        last_t_idx = find(task_plan(robot, :) == t, 1, 'last');

                        consecutive_fragments = last_t_idx;
                    else
                        consecutive_fragments = recharge_idx - 1;
                    end

                    % Check if robot doesn't have flight time enough contemplating the waiting time
                    if not(Agent(a).ac_Ft_s(last_slot + 1) + Td_a_t_t(a, t, Agent(a).queue(last_slot + 1) + 1) + synch_time + consecutive_fragments * Te_t_nf(t, Task(t).nf) + Td_a_t_t(a, R, t + 1) < Agent(a).Ft - Agent(a).Ft_saf)
                        % Check if there is flight time left enough to execute a recharge, and if the robot can fly from the recharge station, execute the task and go back
                        if Agent(a).ac_Ft_s(last_slot + 1) + Td_a_t_t(a, R, Agent(a).queue(last_slot + 1) + 1) < Agent(a).Ft - Agent(a).Ft_saf && Td_a_t_t(a, t, R + 1) + Te_t_nf(t, Task(t).nf) + Td_a_t_t(a, R, t + 1) < Agent(a).Ft - Agent(a).Ft_saf
                            % Add a previous recharge to robot
                            pre_recharge(a) = 1;
                        else
                            disp('Error: this robot has not enough flight time to execute the last task and go back to the recharge station');
                        end

                        % Set the auxiliary termination flag back to true
                        changes_in_finish_time = true;

                        % Restart the robot selection
                        break;
                    end
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
            last_slot = last_slot + pre_recharge(a);

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
            coalition_finishing_time = max(finish_time) - (Task(t).nf - 1) * Te_t_nf(t, Task(t).nf);

            % Assign a robot's fragments
            for fragment = 1:Task(t).nf
                % Check if there is a t or recharge task in this fragment
                if task_plan(robot, fragment) == R
                    % Set pre-recharge flag to one
                    pre_recharge(a) = 1;
                elseif task_plan(robot, fragment) ~= 0
                    % Compute the waiting time for robot a
                    waiting_time = getWaitingTime(a, task_plan(robot, fragment), last_slot, Agent, Task, Td_a_t_t, Te_t_nf, R, pre_recharge, coalition_finishing_time);

                    % Check if robot needs a previous recharge
                    if pre_recharge(a)
                        % Add recharge task to the robot queue
                        Agent(a).queue = [Agent(a).queue, R];

                        % Set pre-recharge flag to zero
                        pre_recharge(a) = 0;

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
                end

                % Compute the finish time for the next fragment
                coalition_finishing_time = coalition_finishing_time + Te_t_nf(t, Task(t).nf);
            end
        end
    end

    %% Set xats decision variable
    % Initial tasks
    t = 0;
    s = 0;
    xats(:, t + 1, s + 1) = 1;

    % Move robot's queue to xats decision variable
    for a = 1:A
        s = 0;
        for task = 1:length(Agent(a).queue) - 1
            t = Agent(a).queue(task + 1);
            s = s + 1;
            xats(a, t + 1, s + 1) = 1;
        end
    end

    % Fill the empty tfin slots with the last tfin value
    for a = 1:A
        for slot = length(Agent(a).queue) + 1:S + 1
            Agent(a).tfin_s(slot) = Agent(a).tfin_s(slot - 1);
        end
    end

    % Check if initializer has used more than S slots
    initializer_S = size(xats, 3) - 1;
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
    xats = reshape(xats,1,[]);
    Sta1s1a2s2 = reshape(Sta1s1a2s2,1,[]);
    Rta1s1a2s2 = reshape(Rta1s1a2s2,1,[]);

    % Put all decision variables together
    xats_nf_S_R = [xats nf_t Sta1s1a2s2 Rta1s1a2s2];

    % Compute the auxiliary decision variables
    if check_solution_flag
        [sol_chk, fval_chk, result_chk] = checkSolution(xats_nf_S_R, Agent, Task, 0, scenario_id, 'Initial solution', false);
        
        % Check if the scenario is feasible
        if result_chk
            sol  = sol_chk;
            fval = fval_chk;
        end
    end

    % Print solution
    if print_solution_flag
        printSolution([], Agent, Task, false, scenario_id, 'Initial solution', 0);
    end

    % Save the result
    if not(isempty(execution_id))
        save(['../mat/Agent_', execution_id, '.mat'], 'Agent');
        save(['../mat/Task_',  execution_id, '.mat'], 'Task');
        save(['../mat/sol_',   execution_id, '.mat'], 'sol');
        save(['../mat/fval_',  execution_id, '.mat'], 'fval');
    end
end

% Compute the waiting time for robot a
function [waiting_time] = getWaitingTime(a, t, last_slot, Agent, Task, Td_a_t_t, Te_t_nf, R, pre_recharge, coalition_finishing_time)
    if pre_recharge(a)
        waiting_time = coalition_finishing_time - Te_t_nf(t, Task(t).nf) - Td_a_t_t(a, t, R + 1) - Te_t_nf(R, Task(R).nf) - Td_a_t_t(a, R, Agent(a).queue(last_slot + 1) + 1) - Agent(a).tfin_s(last_slot + 1);
    else
        waiting_time = coalition_finishing_time - Te_t_nf(t, Task(t).nf) - Td_a_t_t(a, t, Agent(a).queue(last_slot + 1) + 1) - Agent(a).tfin_s(last_slot + 1);
    end
end