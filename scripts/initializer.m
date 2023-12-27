function [dv] = initializer(arg_1, arg_2, print_solution_flag)
    switch nargin
    case 1
        try
            scenario_id = arg_1;
            print_solution_flag = false;
            [Agent, Task] = scenario(scenario_id);
        catch
            error('Either Agent and Task or a valid scenario_id must be provided');
        end
    case 2
        if ischar(arg_1)
            try
                scenario_id = arg_1;
                print_solution_flag = arg_2;
                [Agent, Task] = scenario(scenario_id);
            catch
                error('Either Agent and Task or a valid scenario_id must be provided');
            end
        elseif isempty(arg_1) || isempty(arg_2)
            error('Either Agent and Task or a valid scenario_id must be provided');
        else
            Agent = arg_1;
            Task  = arg_2;
            scenario_id = [];
            print_solution_flag = false;
        end
    case 3
        if isempty(arg_1) || isempty(arg_2)
            error('Either Agent and Task or a valid scenario_id must be provided');
        end
        Agent = arg_1;
        Task  = arg_2;
        scenario_id = [];
    otherwise
        error('Either Agent and Task or a valid scenario_id must be provided');
    end

    % Set Task.nf to 1 to fix old generated scenarios that don't have it
    for t = 1:length(Task)
        Task(t).nf = 1;
    end
    
    [Agent, Task, A, T, S, N, R, Td_a_t_t, Te_t_nf, H_a_t] = getConstantScenarioValues(Agent, Task);

    % Initialize result
    dv = [];

    % Initialize main decision variables
    xats = zeros(A,(T+1),(S+1));
    nf_t = [Task.nf] .* ones(1,T);
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

    % Sort non-decomposable, fragmentable and relayable (with Task(t).nf = 1) tasks by deadline
    [~, tf_deadline_idx] = sort([Task(tf_idx).tmax]);

    % Sort relayable tasks with Task(t).nf != 1 by deadline
    [~, tr_deadline_idx] = sort([Task(tr_idx).tmax]);

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

        % Remove recharges from the first column
        task_plan(task_plan(:, 1) == R, 1) = 0;

        % Remove recharges from the last column
        task_plan(task_plan(:, Task(t).nf) == R, Task(t).nf) = 0;

        % Check if there are at least two fragments in te task's plan
        if (Task(t).nf > 1)
            % Remove recharges from the second column if the first task of that row is 0
            task_plan(task_plan(:, 1) == 0 & task_plan(:, 2) == R, 2) = 0;
            
            % Remove recharges from the second last column if the last task of that row is 0
            task_plan(task_plan(:, Task(t).nf - 1) == R & task_plan(:, Task(t).nf) == 0, Task(t).nf - 1) = 0;
        end

        % Initialize pre-recharge flag
        pre_recharge = zeros(1,A);

        % Initialize auxiliary finish time variable
        finish_time = zeros(1,Task(t).nc);

        % Create an auxiliary termination flag to select the na(t) earliest robots
        changes_in_finish_time = true;

        % Iterate until the selected robots don't change
        while changes_in_finish_time
            % Set the auxiliary termination flag to false
            changes_in_finish_time = false;

            % Get task init time having into account pre-recharges
            for robot = 1:Task(t).nc
                % Get robot's id
                a = compatible_robots(robot);

                % Get last used slot
                last_slot = length(Agent(a).queue) - 1;

                % Compute the finish time of each compatible robot for the previous task, the pre-recharge if needed, and the current task displacement time
                % Note: we use t and not task_plan(robot, 1) because recharge tasks are deleted from the beginning of the plan, so the first task will be t always, with a pre-recharge if needed
                finish_time(robot) = Agent(a).tfin_s(last_slot + 1) + pre_recharge(a) * (Td_a_t_t(a, R, Agent(a).queue(last_slot + 1) + 1) + Te_t_nf(R, Task(R).nf) + Td_a_t_t(a, t, R + 1)) + not(pre_recharge(a)) * (Td_a_t_t(a, t, Agent(a).queue(last_slot + 1) + 1));
            end

            % Get the finish time of the first fragment
            coalition_finishing_time = max(finish_time) + Te_t_nf(t, Task(t).nf);

            % Compute pre_recharge and finish_time for the compatible robots
            for robot = 1:Task(t).nc
                % Get robot's id
                a = compatible_robots(robot);

                % Check if robot does not contemplates a previous recharge (waiting time doesn't affect flight time because it will be made landed in recharge station)
                if not(pre_recharge(a))
                    % Get last used slot
                    last_slot = length(Agent(a).queue) - 1;

                    % Compute the synch time for robot a. Note that the first task is always t as we have removed initial tasks.
                    synch_time = getWaitingTime(a, t, last_slot, Agent, Task, Td_a_t_t, Te_t_nf, R, pre_recharge, coalition_finishing_time);

                    % Get index of the first recharge task in robot's task plan
                    recharge_idx = find(task_plan(robot, :) == R, 1);

                    % Check if robot has battery enough to execute its queue until the first recharge task included
                    if isempty(recharge_idx)
                        consecutive_fragments = Task(t).nf;
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
                            disp('Error: robot has not enough flight time to execute the last task and go back to the recharge station');
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
        slot_map = zeros(Task(t).nc, Task(t).nf);

        % Compute the slot_map
        for robot = 1:Task(t).nc
            % Get robot's id
            a = compatible_robots(robot);

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
            for r1 = 1:Task(t).nc
                % Check if there is a t task assigned to this robot and fragment
                if task_plan(r1, fragment) == t
                    % Get robot's id
                    a1 = compatible_robots(r1);

                    % Get fragment's slot
                    s1 = slot_map(r1, fragment);

                    % Select another robot from fragment column
                    for r2 = 1:Task(t).nc
                        % Check if there is a t task assigned to this robot and fragment
                        if task_plan(r2, fragment) == t
                            % Get robot's id
                            a2 = compatible_robots(r2);

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
                    a1 = compatible_robots(r1);
                    a2 = compatible_robots(r2);

                    % Get fragment's slots
                    s1 = slot_map(r1, fragment);
                    s2 = slot_map(r2, fragment + 1);

                    % Set relay constraints
                    Rta1s1a2s2(t - 1, a1, s1, a2, s2) = 1;
                end
            end
        end

        % Get the finish time of the first fragment
        coalition_finishing_time = max(finish_time) + Te_t_nf(t, Task(t).nf);

        % Assign the task's plan to the compatible robots
        for robot = 1:Task(t).nc
            % Get robot's id
            a = compatible_robots(robot);

            % Get last used slot
            last_slot = length(Agent(a).queue) - 1;

            % Assign a robot's fragments
            for fragment = 1:Task(t).nf
                % Check if there is a t or recharge task in this fragment
                if task_plan(robot, fragment) ~= 0
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

    % Reshape decision variables
    xats = reshape(xats,1,[]);
    Sta1s1a2s2 = reshape(Sta1s1a2s2,1,[]);
    Rta1s1a2s2 = reshape(Rta1s1a2s2,1,[]);

    % Put all decision variables together
    xats_nf_S_R = [xats nf_t Sta1s1a2s2 Rta1s1a2s2];

    % Compute the auxiliary decision variables
    [dv_chk, ~, result_chk] = checkSolution(xats_nf_S_R, Agent, Task, 0, scenario_id, 'Initial solution', print_solution_flag);

    % Check if the scenario is feasible and return
    if result_chk
        dv = dv_chk;
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