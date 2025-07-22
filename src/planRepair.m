function [Agent, Task, result] = planRepair(Agent, Task, Synchs, Relays, delay, print_middle_step)
    % Inputs:
        % Agent:  Robot structure array
        % Task:   Task structure array
        % synch:  List of slots to be synchronized
        % relays: List of relays to be coordinated
        % delay:  Array containing the amount of delay in second and the slot delayed ([d r s])
    % Result is false if the plan can't be repaired, true otherwise
    result = true;

    % Check if the number of inputs is correct
    if nargin < 5
        error('Not enough input arguments');
    end

    if nargin < 6
        print_middle_step = false;
    end

    % Check if the delay input is correct
    if length(delay) ~= 3
        error('Delay input is incorrect');
    else
        % Separate the delay into its components
        delayed_time = delay(1);
        delayed_robot = delay(2);
        delayed_slot = delay(3);
    end

    % Check if the scenario is solved by the heuristic
    if not(isfield(Agent, 'queue'))
        error('Input scenario must be solved by the heuristic');
    end

    % Numerical tolerance
    tol = 1e-6;

    % Get scenario information
    [Agent, Task, A, ~, S, ~, R, ~, ~, ~] = getConstantScenarioValues(Agent, Task);

    % Put constant scenario values into a structure
    constant_scenario_values = struct('A', A, 'S', S, 'R', R);

    % Determine which tasks has already been executed and move their waiting time information to an auxiliary variable
    % Find out the finish time of the task that got delayed, i.e. the current execution time
    delayed_task_finish_time = Agent(delayed_robot).tfin_s(delayed_slot + 1) + delayed_time;

    % Initialize the delayed slots list
    delayed_slots = [delayed_robot; delayed_slot];

    % Find any other slot affected by delayed slots
    new_delayed_slots = true;
    while new_delayed_slots
        % Initialize the flags
        delayed_amount = width(delayed_slots);
        new_delayed_slots = false;

        % Check for every slot currently in the list
        for slot = 1:width(delayed_slots)
            % Find any slot coordinated with the delayed slots
            delayed_slots = [delayed_slots, Synchs(1:2, all(Synchs(3:4,:) == delayed_slots(:,slot)))];
            delayed_slots = [delayed_slots, Synchs(3:4, all(Synchs(1:2,:) == delayed_slots(:,slot)))];
            delayed_slots = [delayed_slots, Relays(1:2, all(Relays(3:4,:) == delayed_slots(:,slot)))];
            delayed_slots = [delayed_slots, Relays(3:4, all(Relays(1:2,:) == delayed_slots(:,slot)))];
        end

        % Remove duplicates from the list
        delayed_slots = unique(delayed_slots', 'rows')';

        % Add to the list every slot that is behind the delayed slots
        for slot = 1:width(delayed_slots)
            % Loop until the end of this robots queue
            for s = delayed_slots(2, slot) + 1:length(Agent(delayed_slots(1, slot)).queue) - 1
                delayed_slots = [delayed_slots, [delayed_slots(1, slot); s]];
            end
        end

        % Remove duplicates from the list
        delayed_slots = unique(delayed_slots', 'rows')';

        % Check if there are any new slots in the list
        if width(delayed_slots) ~= delayed_amount
            new_delayed_slots = true;
        end
    end

    % Initialize the executed flag and the delayed time variable for each slot
    for a = 1:A
        Agent(a).executed_flag_s = false(1, length(Agent(a).queue) - 1);
        Agent(a).delay_s = zeros(1,length(Agent(a).queue) - 1);

        % Step 1 of the delay_s algorithm. 
        % Set the delayed time for the first delayed slot of this robot if any.
        % Note: Initially, we will only set the delay to "delayed_time" for the first delayed slot of each agent if any. This keeps the plan coordinated for now (ignoring battery constraints). Doing this we can correctly set the executed flag value for all slots.
        % Once we have the value for the executed flags, we can assign the correct value of delay_s for all slots.
        % This algorithm works for every possible combination of executed (finished and in progress) and not executed (failed or not started) slots (all the details in One Note TODO: place here the link to the full explanation in the final thesis).
        if any(delayed_slots(1,:) == a)
            % Get the first delayed slot in this agent.
            s = min(delayed_slots(2, delayed_slots(1,:) == a));

            % Note: any coordinated slot affected by the delay, to keep being coordinated, must be delayed exactly a "delayed_time" amount of time (one delayed slot in total per robot, i.e. its first delayed slot).
            % To compute the executed_flag, we need to set initially delay_s = delayed_time. This value will be change later.
            Agent(a).delay_s(s) = delayed_time;
        end
    end

    % Re-compute the accumulated flight time and finish time to get which slots are executed and which are not.
    Agent = updateTfin(Agent, constant_scenario_values);
    Agent = updateAcFts(Agent, constant_scenario_values);

    % Create a list to save the slots whose executed flags are already set (add an initial slot to give dimensions to the matrix)
    processed_slots = [0; 0];

    % Step 2 of the delay_s algorithm: identify executed and not executed slots
    % Set the executed flags for executed slots and slots that are in progress and have battery enough to finish
    % The executed flag indicates that a slot's waiting time can not be modified.
    for a = 1:A
        for s = 1:length(Agent(a).queue) - 1
            % Check if this slot is not in the processed slots list
            if ~any(all(processed_slots == [a; s]))
                % Note: this was originally thought for delayed relayed slots (allocated using a matrix pattern), but later was reused and now the same code applies for any kind of slots.
                % Create a list with all the coordinated slots
                coordinated_slots = [a; s];
                new_coordinated_slots = true;
                while new_coordinated_slots
                    % Initialize the flags
                    coordinated_amount = width(coordinated_slots);
                    new_coordinated_slots = false;

                    % Check for every slot currently in the list
                    for slot = 1:width(coordinated_slots)
                        % Find any slot coordinated with the delayed slots
                        coordinated_slots = [coordinated_slots, Synchs(1:2, all(Synchs(3:4,:) == coordinated_slots(:,slot)))];
                        coordinated_slots = [coordinated_slots, Synchs(3:4, all(Synchs(1:2,:) == coordinated_slots(:,slot)))];
                        coordinated_slots = [coordinated_slots, Relays(1:2, all(Relays(3:4,:) == coordinated_slots(:,slot)))];
                        coordinated_slots = [coordinated_slots, Relays(3:4, all(Relays(1:2,:) == coordinated_slots(:,slot)))];
                    end

                    % Remove duplicates from the list
                    coordinated_slots = unique(coordinated_slots', 'rows')';

                    % Check if there are any new slots in the list
                    if width(coordinated_slots) ~= coordinated_amount
                        new_coordinated_slots = true;
                    end
                end
                                
                % Aux vars to get the init and finish time of the coordinated slots
                coordinated_init_time = inf;
                coordinated_finish_time = 0;

                % Get the join init and finish time of the robot coalition
                for i = 1:width(coordinated_slots)
                    robot = coordinated_slots(1, i);
                    slot = coordinated_slots(2, i);

                    % Compare the init time
                    if coordinated_init_time > Agent(robot).tfin_s((slot - 1) + 1)
                        coordinated_init_time = Agent(robot).tfin_s((slot - 1) + 1);
                    end

                    % Compare the finish time
                    if coordinated_finish_time < Agent(robot).tfin_s(slot + 1)
                        coordinated_finish_time = Agent(robot).tfin_s(slot + 1);
                    end
                end

                % Init the battery constraint flag
                battery_flag = true;

                % Get the list of robots involved in this coordination
                involved_robots = unique(coordinated_slots(1, :));

                % Check if the battery fails at any time of those robots up to the coordinated slot
                for i = 1:length(involved_robots)
                    robot = involved_robots(i);

                    % Iterate from the first slot of each involved robot to its last slot belonging to the coordinated slots list
                    for slot = 1:max(coordinated_slots(2, coordinated_slots(1,:) == robot))
                        % Check if the battery constraint is not met for this slot. Only delayed slots need its battery checked
                        if any(all(delayed_slots == [a; s])) && Agent(robot).ac_Ft_s(slot + 1) > Agent(robot).Ft - Agent(robot).Ft_saf
                            battery_flag = false;

                            % Add this slot and the ones behind to the processed slots list. They need to be re-allocated.
                            for aux_s = slot:length(Agent(robot).queue) - 1
                                processed_slots = [processed_slots, [robot; aux_s]];
                            end

                            % Move to the next involved robot
                            break;
                        end
                    end
                end

                if battery_flag
                    % Check if the coordinated slots are in execution or have been executed
                    if coordinated_init_time < delayed_task_finish_time
                        % Set all slots from the coordinated slots list to executed
                        for slot = 1:width(coordinated_slots)
                            Agent(coordinated_slots(1, slot)).executed_flag_s(coordinated_slots(2, slot)) = true;
                        end
                    end
                end

                % Add all coordinated slots to the processed slots list
                for slot = 1:width(coordinated_slots)
                    processed_slots = [processed_slots, [coordinated_slots(1, slot); coordinated_slots(2, slot)]];
                end
            end
        end
    end

    % Step 3 of the delay_s algorithm: loop through every queue to reorganize them and set the final delay_s value.
    for a = 1:A
        % Initialize the auxiliary slot counter, i.e. s'
        sp = 1;

        % This part will take approx S steps, more precisely, one step per slot in this queue
        for s = 1:length(Agent(a).queue) - 1
            % Check if the s' slot is executed
            if Agent(a).executed_flag_s(sp)
                % Compute the delay in s' as Tfin_{s'} - Te_{s'} - Tw_{s'} - Td_{s'} - Tfin_{s'-1}
                Agent(a).delay_s(sp) = Agent(a).tfin_s(sp + 1) - Agent(a).Te_s(sp) - Agent(a).Tw_s(sp) - Agent(a).Td_s(sp) - Agent(a).tfin_s((sp - 1) + 1);

                % Increase s'
                sp = sp + 1;
            else
                % If this is a recharge task followed by an also executed task, set it also as executed and not move it.
                pre_relay_recharge_flag = false;
                if Agent(a).queue(sp + 1) == R
                    % Check if there is any executed task after this slot
                    for sp_aux = sp + 1:length(Agent(a).queue) - 1
                        if Agent(a).executed_flag_s(sp_aux)
                            % Set recharge task also as executed
                            Agent(a).executed_flag_s(sp) = true;

                            % Compute the delay in s' as Tfin_{s'} - Te_{s'} - Tw_{s'} - Td_{s'} - Tfin_{s'-1}
                            Agent(a).delay_s(sp) = Agent(a).tfin_s(sp + 1) - Agent(a).Te_s(sp) - Agent(a).Tw_s(sp) - Agent(a).Td_s(sp) - Agent(a).tfin_s((sp - 1) + 1);

                            % Increase s'
                            sp = sp + 1;

                            pre_relay_recharge_flag = true;
                            break;
                        end
                    end
                end

                % Check if this wasn't a recharge task previous to a relay
                if ~pre_relay_recharge_flag
                    % Move the fragment in s' to the end of this queue
                    [Agent, Synchs, Relays] = moveSToEnd(a, sp, Agent, Synchs, Relays);
                    % disp(['Moved ', Task(Agent(a).queue(end)).name,' [', num2str(a), ', ', num2str(sp), '] to the end']);
                end
            end
        end

        % Step 4 of the delay_s algorithm: compute the delay in s' as max(0, current execution time - Tfin_{s'-1})
        Agent(a).delay_s(sp) = max(0, delayed_task_finish_time - Agent(a).tfin_s((sp - 1) + 1));

        % Set to 0 the delay in slots after sp
        for s = sp + 1:length(Agent(a).queue) - 1
            Agent(a).delay_s(s) = 0;
        end

        % Substitute virtual zeros
        Agent(a).delay_s(abs(Agent(a).delay_s) < tol) = 0;
    end

    % Print middle step
    if print_middle_step
        % Re-compute the accumulated flight time and finish time
        Agent = updateTfin(Agent, constant_scenario_values);
        Agent = updateAcFts(Agent, constant_scenario_values);

        % Print the middle step
        printSolution([], Agent, Task, false, '', 'Delayed Plan');
        fig = gcf;
        fig.WindowState = 'maximized';

        return;
    end

    % Remove all waiting times from the not executed slots
    for a = 1:A
        for s = 1:length(Agent(a).queue) - 1
            % Check if the s' slot is not executed
            if ~Agent(a).executed_flag_s(s)
                % Set this slot waiting time to 0
                Agent(a).Tw_s(s) = 0;
            end
        end
    end

    % Step 5 of the delay_s algorithm: re-compute the accumulated flight time and finish time
    Agent = updateTfin(Agent, constant_scenario_values);
    Agent = updateAcFts(Agent, constant_scenario_values);

    % Check if at this point, before repair/replanning, if any robot has failed, i.e., run out of battery.
    for a = 1:A
        for s = 1:length(Agent(a).queue) - 1
            % Check if the accumulated flight time is grater than the total flight time capacity minus de safety time
            if Agent(a).ac_Ft_s(s + 1) > Agent(a).Ft - Agent(a).Ft_saf
                result = false;
                return;
            end
        end
    end

    % Step 6 of the delay_s algorithm: repair the plan (or re-plan if this fails)
    % Note: in order to repair, we can only change the waiting times of not executed slots, i.e. finish time of executed slots must stay the same.
    %? Tw_a_s
    %* Depends on: t_fin_a_s 
    % Auxiliary copy of S and R to coordinate the slots one by one
    Synchs_copy = Synchs;
    Relays_copy = Relays;
    
    % Auxiliary variable to store the already coordinated slots
    Synchs_coordinated = [];
    Relays_coordinated = [];

    % While there are still slots to be coordinated
    coordination_remaining_flag = not(isempty(Synchs_copy)) || not(isempty(Relays_copy));
    last_update_flag = false;
    infinite_loop_flag = false;

    while coordination_remaining_flag || not(last_update_flag)
        if infinite_loop_flag
            disp('Invalid solution: infinite task coordination loop found.');
            result = false;
            break;
        end

        coordination_remaining_flag = not(isempty(Synchs_copy)) || not(isempty(Relays_copy));
        if not(coordination_remaining_flag)
            last_update_flag = true;
        end

        % Compute all variables that depends on Tw_a_s directly or indirectly with Tw_a_s' actual value
        Agent = updateTfin(Agent, constant_scenario_values);

        if last_update_flag
            break;
        end

        % Search the first slot to be coordinated from the remaining ones
        for s = 1:S
            for a = 1:A
                % If S(:, a, s, :, :) || S(:, :, :, a, s) || R(:, a, s, :, :) || R(:, :, :, a, s) -> (a1, s1) is a slot to be coordinated
                % Note that, as synchronizations are bidirectional, if any(S(:, a, s, :, :)), there will be also any(S(:, :, :, a, s))
                % Note that it is possible to have at the same time any(R(:, a, s, :, :)) and any(R(:, :, :, a, s)), but not to the same other slot
                coord_type_ind = 0;
                if not(isempty(Synchs_copy))
                    coord_type_ind = coord_type_ind + 1 * not(isempty(Synchs_copy(:, Synchs_copy(1,:) == a & Synchs_copy(2,:) == s)));
                end
                if not(isempty(Relays_copy))
                    coord_type_ind = coord_type_ind + 10 * not(isempty(Relays_copy(:, Relays_copy(1,:) == a & Relays_copy(2,:) == s))) + ...
                                                     100 * not(isempty(Relays_copy(:, Relays_copy(3,:) == a & Relays_copy(4,:) == s)));
                end
                if coord_type_ind
                    [Agent, Synchs_copy, Relays_copy, Synchs_coordinated, Relays_coordinated, infinite_loop_flag] = coordinateTwoSlots(Agent, a, s, Synchs, Relays, Synchs_copy, Relays_copy, Synchs_coordinated, Relays_coordinated, tol, constant_scenario_values);
                end
                if coord_type_ind
                    break;
                end
            end
            if coord_type_ind
                break;
            end
        end
    end

    % Update tfin_s and ac_Ft_s values
    Agent = updateTfin(Agent, constant_scenario_values);
    Agent = updateAcFts(Agent, constant_scenario_values);

    % Check if the new plan fits the battery constraints
    for a = 1:A
        for s = 1:length(Agent(a).queue) - 1
            if Agent(a).ac_Ft_s(s + 1) > Agent(a).Ft - Agent(a).Ft_saf
                % disp(['Insufficient battery to finish task ', Task(Agent(a).queue(s + 1)).name, ' in slot ', num2str(s), ' agent ', num2str(a)]);
                result = false;
            end
        end
    end
end

%% Update tfin_a_s
function [Agent] = updateTfin(Agent, constant_scenario_values)
    A = constant_scenario_values.A;
    S = constant_scenario_values.S;

    %* Depends on: Td_a_s, Tw_a_s, Te_a_s
    % tfin_a(s) value
    % tfin(a,s) = tfin(a,s-1) + Td(a,s) + Tw(a,s) + Te(a,s), for all a = 1 to A and s = 1 to S
    for a = 1:A
        % tfin(a,0) = 0
        Agent(a).tfin_s(0 + 1) = 0;
        for s = 1:length(Agent(a).queue) - 1
            % tfin_a(s) = tfin(a,s-1) + Td(a,s) + Tw(a,s) + Te(a,s)
            Agent(a).tfin_s(s + 1) = Agent(a).tfin_s((s - 1) + 1) + Agent(a).delay_s(s) + Agent(a).Td_s(s) + Agent(a).Tw_s(s) + Agent(a).Te_s(s);
        end
    end
end

%% Update ac_Ft_s
function [Agent] = updateAcFts(Agent, constant_scenario_values)
    A = constant_scenario_values.A;
    S = constant_scenario_values.S;
    R = constant_scenario_values.R;

    for a = 1:A
        for s = 1:length(Agent(a).queue) - 1
            % Check if the task is a recharge task
            if Agent(a).queue(s + 1) == R
                Agent(a).ac_Ft_s(s + 1) = Agent(a).Td_s(s);
            else
                Agent(a).ac_Ft_s(s + 1) = Agent(a).delay_s(s) + Agent(a).Td_s(s) + Agent(a).Tw_s(s) + Agent(a).Te_s(s);
            end
            
            % If last task was not a recharge, add also the last accumulated flight time
            if Agent(a).queue((s - 1) + 1) ~= R
                Agent(a).ac_Ft_s(s + 1) = Agent(a).ac_Ft_s(s + 1) + Agent(a).ac_Ft_s((s - 1) + 1);
            end
        end
    end
end

%% Move the fragment in slot sp to the end of the queue
function [Agent, Synchs, Relays] = moveSToEnd(a, sp, Agent, Synchs, Relays)
    % Copy sp information to aux variables
    aux_fragment = Agent(a).queue(sp + 1);
    aux_Td = Agent(a).Td_s(sp);
    aux_Tw = Agent(a).Tw_s(sp);
    aux_Te = Agent(a).Te_s(sp);
    aux_executed_flag = Agent(a).executed_flag_s(sp);

    % Replace [a; sp] by [a; inf] in Synchs and Relays
    Synchs(1:2, all(Synchs(1:2, :) == [a; sp])) = repmat([a; inf], 1, sum(all(Synchs(1:2, :) == [a; sp])));
    Synchs(3:4, all(Synchs(3:4, :) == [a; sp])) = repmat([a; inf], 1, sum(all(Synchs(3:4, :) == [a; sp])));
    Relays(1:2, all(Relays(1:2, :) == [a; sp])) = repmat([a; inf], 1, sum(all(Relays(1:2, :) == [a; sp])));
    Relays(3:4, all(Relays(3:4, :) == [a; sp])) = repmat([a; inf], 1, sum(all(Relays(3:4, :) == [a; sp])));

    % Move the information in the slots after sp one slot forward
    for s = sp:(length(Agent(a).queue) - 1) - 1
        Agent(a).queue(s + 1)       = Agent(a).queue((s + 1) + 1);
        Agent(a).Td_s(s)            = Agent(a).Td_s((s + 1));
        Agent(a).Tw_s(s)            = Agent(a).Tw_s((s + 1));
        Agent(a).Te_s(s)            = Agent(a).Te_s((s + 1));
        Agent(a).executed_flag_s(s) = Agent(a).executed_flag_s((s + 1));

        % Replace [a; s + 1] by [a; s] in Synchs and Relays
        Synchs(1:2, all(Synchs(1:2, :) == [a; s + 1])) = repmat([a; s], 1, sum(all(Synchs(1:2, :) == [a; s + 1])));
        Synchs(3:4, all(Synchs(3:4, :) == [a; s + 1])) = repmat([a; s], 1, sum(all(Synchs(3:4, :) == [a; s + 1])));
        Relays(1:2, all(Relays(1:2, :) == [a; s + 1])) = repmat([a; s], 1, sum(all(Relays(1:2, :) == [a; s + 1])));
        Relays(3:4, all(Relays(3:4, :) == [a; s + 1])) = repmat([a; s], 1, sum(all(Relays(3:4, :) == [a; s + 1])));
    end

    % Place the information initially in sp at the end of the queue
    Agent(a).queue((length(Agent(a).queue) - 1) + 1)       = aux_fragment;
    Agent(a).Td_s((length(Agent(a).queue) - 1))            = aux_Td;
    Agent(a).Tw_s((length(Agent(a).queue) - 1))            = aux_Tw;
    Agent(a).Te_s((length(Agent(a).queue) - 1))            = aux_Te;
    Agent(a).executed_flag_s((length(Agent(a).queue) - 1)) = aux_executed_flag;

    % Replace [a; inf] by [a; length(Agent(a).queue) - 1] in Synchs and Relays
    Synchs(1:2, all(Synchs(1:2, :) == [a; inf])) = repmat([a; length(Agent(a).queue) - 1], 1, sum(all(Synchs(1:2, :) == [a; inf])));
    Synchs(3:4, all(Synchs(3:4, :) == [a; inf])) = repmat([a; length(Agent(a).queue) - 1], 1, sum(all(Synchs(3:4, :) == [a; inf])));
    Relays(1:2, all(Relays(1:2, :) == [a; inf])) = repmat([a; length(Agent(a).queue) - 1], 1, sum(all(Relays(1:2, :) == [a; inf])));
    Relays(3:4, all(Relays(3:4, :) == [a; inf])) = repmat([a; length(Agent(a).queue) - 1], 1, sum(all(Relays(3:4, :) == [a; inf])));
end

%% Get (a2,s2)
function [a2, s2] = geta2s2(a, s, coord_type_ind, Synchs_copy, Relays_copy, constant_scenario_values)
    A = constant_scenario_values.A;
    S = constant_scenario_values.S;

    %? Note: this nested for loop was originally in the opposite order and I don't remember if there was a reason for that
    %? I think the reason is because I wanna avoid extra iterations by finding slots to be coordinated starting from lower slots
    for s2 = 1:S
        for a2 = 1:A
            switch coord_type_ind
            case {1, 11, 101, 111}
                if not(isempty(Synchs_copy)) && not(isempty(Synchs_copy(:, Synchs_copy(1,:) == a & Synchs_copy(2,:) == s & Synchs_copy(3,:) == a2 & Synchs_copy(4,:) == s2)))
                    return;
                end
            case 10
                if not(isempty(Relays_copy)) && not(isempty(Relays_copy(:, Relays_copy(1,:) == a & Relays_copy(2,:) == s & Relays_copy(3,:) == a2 & Relays_copy(4,:) == s2)))
                    return;
                end
            case {100, 110}
                if not(isempty(Relays_copy)) && not(isempty(Relays_copy(:, Relays_copy(1,:) == a2 & Relays_copy(2,:) == s2 & Relays_copy(3,:) == a & Relays_copy(4,:) == s)))
                    return;
                end
            otherwise
                error('Missing case in switch statement.');
            end
        end
    end
end

%% Coordinate two slots
function [Agent, Synchs_copy, Relays_copy, Synchs_coordinated, Relays_coordinated, infinite_loop_flag] = coordinateTwoSlots(Agent, a, s, Synchs, Relays, Synchs_copy, Relays_copy, Synchs_coordinated, Relays_coordinated, tol, constant_scenario_values)
    A = constant_scenario_values.A;
    S = constant_scenario_values.S;
    R = constant_scenario_values.R;
    
    a0 = a;
    s0 = s;
    infinite_loop_flag = false;
    coord_break_flag = false;
    while not(coord_break_flag)
        % Note that, as synchronizations are bidirectional, if any(S(:, a, s, :, :)), there will be also any(S(:, :, :, a, s))
        % Note that it is possible to have at the same time any(R(:, a, s, :, :)) and any(R(:, :, :, a, s)), but not to the same other slot
        coord_type_ind = 0;
        if not(isempty(Synchs_copy))
            coord_type_ind = coord_type_ind + 1 * not(isempty(Synchs_copy(:, Synchs_copy(1,:) == a & Synchs_copy(2,:) == s)));
        end
        if not(isempty(Relays_copy))
            coord_type_ind = coord_type_ind + 10 * not(isempty(Relays_copy(:, Relays_copy(1,:) == a & Relays_copy(2,:) == s))) + ...
                                             100 * not(isempty(Relays_copy(:, Relays_copy(3,:) == a & Relays_copy(4,:) == s)));
        end
        
        % Find out the (a2, s2) that (a,s) must be coordinated with
        [a2, s2] = geta2s2(a, s, coord_type_ind, Synchs_copy, Relays_copy, constant_scenario_values);
        
        % Find the first slot to be coordinated in a2 to check if there is a previous slot to be coordinated in a2.
        if a == a2
            first_slot_coord_a2 = s2;
        else
            for first_slot_coord_a2 = 1:s2
                if not(isempty(Synchs_copy)) && (not(isempty(Synchs_copy(:, Synchs_copy(1,:) == a2 & Synchs_copy(2,:) == first_slot_coord_a2)))  || ...
                                                 not(isempty(Synchs_copy(:, Synchs_copy(3,:) == a2 & Synchs_copy(4,:) == first_slot_coord_a2)))) || ...
                   not(isempty(Relays_copy)) && (not(isempty(Relays_copy(:, Relays_copy(1,:) == a2 & Relays_copy(2,:) == first_slot_coord_a2)))  || ...
                                                 not(isempty(Relays_copy(:, Relays_copy(3,:) == a2 & Relays_copy(4,:) == first_slot_coord_a2))))
                    break;
                end
            end
        end

        % If (a2, s2) was the first slot to be coordinated in a2
        if first_slot_coord_a2 == s2
            % Check if (a1, s1) and (a2, s2) are already coordinated
            switch coord_type_ind
            case {1, 11, 101, 111}
                time_to_wait = Agent(a).tfin_s(s + 1) - Agent(a2).tfin_s(s2 + 1);
            case 10
                time_to_wait = Agent(a).tfin_s(s + 1) - (Agent(a2).tfin_s(s2 + 1) - Agent(a2).Te_s(s2));
            case {100, 110}
                time_to_wait = Agent(a2).tfin_s(s2 + 1) - (Agent(a).tfin_s(s + 1) - Agent(a).Te_s(s));
            otherwise
                error('Missing case in switch statement.');
            end

            Tw = abs(time_to_wait);
            if Tw < tol
                % None has to wait, they are already coordinated
                aw = a;
                sw = s;
                coord_break_flag = true;
            elseif time_to_wait < 0
                switch coord_type_ind
                case {1, 11, 101, 111, 10}
                    % "a" has to wait
                    aw = a;
                    sw = s;
                case {100, 110}
                    % "a2" has to wait
                    aw = a2;
                    sw = s2;
                otherwise
                    error('Missing case in switch statement.');
                end
            else
                switch coord_type_ind
                case {1, 11, 101, 111, 10}
                    % "a2" has to wait
                    aw = a2;
                    sw = s2;
                case {100, 110}
                    % "a" has to wait
                    aw = a;
                    sw = s;
                otherwise
                    error('Missing case in switch statement.');
                end
            end

            % If (a, s) and (a2, s2) are not already coordinated
            flag_any_recharge = false;
            flag_any_coordination = false;
            if not(coord_break_flag)
                % Find, if any, the last recharge task before the slot to coordinate in the agent that waits
                for sr = sw-1:-1:1
                    if Agent(aw).queue(sr + 1) == R % Recharge task
                        flag_any_recharge = true;
                        break;
                    end
                end

                % If there is not any recharge task before the slot to coordinate in the agent that waits, then the waiting is done in that slot
                if flag_any_recharge
                    % Check if there is any already synchronized or relayed task between the last recharge task and the task to coordinate in the agent that waits
                    for ssw = sr+1:sw-1
                        for as = 1:A
                            for ss = 1:S
                                if not(isempty(Synchs)) && (not(isempty(Synchs(:, Synchs(1,:) == aw & Synchs(2,:) == ssw & Synchs(3,:) == as & Synchs(4,:) == ss)))  || ...
                                                            not(isempty(Synchs(:, Synchs(1,:) == as & Synchs(2,:) == ss & Synchs(3,:) == aw & Synchs(4,:) == ssw)))) || ...
                                   not(isempty(Relays)) && (not(isempty(Relays(:, Relays(1,:) == aw & Relays(2,:) == ssw & Relays(3,:) == as & Relays(4,:) == ss)))  || ...
                                                            not(isempty(Relays(:, Relays(1,:) == as & Relays(2,:) == ss & Relays(3,:) == aw & Relays(4,:) == ssw))))
                                    flag_any_coordination = 1;
                                end
                            end
                        end
                    end
                    % If there is not any coordination in between, then the waiting is done in the last recharge, else, in the task to coordinate
                end

                % Update waiting time
                if flag_any_recharge && not(flag_any_coordination) && ~Agent(aw).executed_flag_s(sr)
                    Agent(aw).Tw_s(sr) = Agent(aw).Tw_s(sr) + Tw;
                else
                    Agent(aw).Tw_s(sw) = Agent(aw).Tw_s(sw) + Tw;
                end
            end

            Agent = updateTfin(Agent, constant_scenario_values);

            % Remove this coordination before checking if the past ones still coordinated because we may need to call again this function
            switch coord_type_ind
            case {1, 11, 101, 111}
                Synchs_copy(:, Synchs_copy(1,:) == a & Synchs_copy(2,:) == s & Synchs_copy(3,:) == a2 & Synchs_copy(4,:) == s2) = [];
                Synchs_copy(:, Synchs_copy(1,:) == a2 & Synchs_copy(2,:) == s2 & Synchs_copy(3,:) == a & Synchs_copy(4,:) == s) = [];
            case 10
                Relays_copy(:, Relays_copy(1,:) == a & Relays_copy(2,:) == s & Relays_copy(3,:) == a2 & Relays_copy(4,:) == s2) = [];
            case {100, 110}
                Relays_copy(:, Relays_copy(1,:) == a2 & Relays_copy(2,:) == s2 & Relays_copy(3,:) == a & Relays_copy(4,:) == s) = [];
            otherwise
                error('Missing case in switch statement.');
            end

            % Check if this slot was already coordinated with a previous one
            % For every time (aw,sw) has been coordinated before with other slot
            for sb = 1:S
                for ab = 1:A
                    % Note that, as synchronizations are bidirectional, if any(S(:, a, s, :, :)), there will be also any(S(:, :, :, a, s))
                    % Note that it is possible to have at the same time any(R(:, a, s, :, :)) and any(R(:, :, :, a, s)), but not to the same other slot
                    coord_type_ind_before = 0;
                    if not(isempty(Synchs_coordinated))
                        coord_type_ind_before = coord_type_ind_before + 1 * not(isempty(Synchs_coordinated(:, Synchs_coordinated(1,:) == aw & Synchs_coordinated(2,:) == sw & Synchs_coordinated(3,:) == ab & Synchs_coordinated(4,:) == sb)));
                    end
                    if not(isempty(Relays_coordinated))
                        coord_type_ind_before = coord_type_ind_before + 10 * not(isempty(Relays_coordinated(:, Relays_coordinated(1,:) == aw & Relays_coordinated(2,:) == sw & Relays_coordinated(3,:) == ab & Relays_coordinated(4,:) == sb))) + ...
                                                                       100 * not(isempty(Relays_coordinated(:, Relays_coordinated(1,:) == ab & Relays_coordinated(2,:) == sb & Relays_coordinated(3,:) == aw & Relays_coordinated(4,:) == sw)));
                    end
                    
                    if coord_type_ind_before
                        [Agent, Synchs_copy, Relays_copy, Synchs_coordinated, Relays_coordinated, infinite_loop_flag] = coordinateAgain(Agent, aw, sw, ab, sb, coord_type_ind_before, Synchs, Relays, Synchs_copy, Relays_copy, Synchs_coordinated, Relays_coordinated, tol, constant_scenario_values);
                    end
                end
            end

            % Check if there are any other coordinations already done in a future slot in the agent that waited
            % For every coordination already done in aw for a slot greater than sw (note: if the waiting time was applied in a previous recharge, we would find this coordination again in the following loop except for the fact that it hasn't been included in the coordinated list yet)
            for sd = sw+1:S
                for sb = 1:S
                    for ab = 1:A
                        %! Next is the (1) most time consuming line in this code right now
                        coord_type_ind_before = 0;
                        if not(isempty(Synchs_coordinated))
                            coord_type_ind_before = coord_type_ind_before + 1 * not(isempty(Synchs_coordinated(:, Synchs_coordinated(1,:) == aw & Synchs_coordinated(2,:) == sd & Synchs_coordinated(3,:) == ab & Synchs_coordinated(4,:) == sb)));
                        end
                        if not(isempty(Relays_coordinated))
                            coord_type_ind_before = coord_type_ind_before + 10 * not(isempty(Relays_coordinated(:, Relays_coordinated(1,:) == aw & Relays_coordinated(2,:) == sd & Relays_coordinated(3,:) == ab & Relays_coordinated(4,:) == sb))) + ...
                                                                           100 * not(isempty(Relays_coordinated(:, Relays_coordinated(1,:) == ab & Relays_coordinated(2,:) == sb & Relays_coordinated(3,:) == aw & Relays_coordinated(4,:) == sd)));
                        end
                        if coord_type_ind_before
                            [Agent, Synchs_copy, Relays_copy, Synchs_coordinated, Relays_coordinated, infinite_loop_flag] = coordinateAgain(Agent, aw, sd, ab, sb, coord_type_ind_before, Synchs, Relays, Synchs_copy, Relays_copy, Synchs_coordinated, Relays_coordinated, tol, constant_scenario_values);
                        end
                    end
                end
            end

            % After checking and tuning past coordinations, check if the slots of the current coordination still coordinated
            switch coord_type_ind
            case {1, 11, 101, 111}
                tmp_time_to_wait = Agent(a).tfin_s(s + 1) - Agent(a2).tfin_s(s2 + 1);
            case 10
                tmp_time_to_wait = Agent(a).tfin_s(s + 1) - (Agent(a2).tfin_s(s2 + 1) - Agent(a2).Te_s(s2));
            case {100, 110}
                tmp_time_to_wait = Agent(a2).tfin_s(s2 + 1) - (Agent(a).tfin_s(s + 1) - Agent(a).Te_s(s));
            otherwise
                error('Missing case in switch statement.');
            end

            if abs(tmp_time_to_wait) > tol
                infinite_loop_flag = true;
                break;
            end

            % Add the coordination to the coordinated list
            switch coord_type_ind
            case {1, 11, 101, 111}
                Synchs_coordinated = [Synchs_coordinated, [a; s; a2; s2]];
                Synchs_coordinated = [Synchs_coordinated, [a2; s2; a; s]];
            case 10
                Relays_coordinated = [Relays_coordinated, [a; s; a2; s2]];
            case {100, 110}
                Relays_coordinated = [Relays_coordinated, [a2; s2; a; s]];
            otherwise
                error('Missing case in switch statement.');
            end

            coord_break_flag = true;
        else
            % Check if an infinite loop has been found
            if a2 == a0 && first_slot_coord_a2 == s0
                infinite_loop_flag = true;
                break;
            end

            % Change the target slot to coordinate, (a,s), and continue
            a = a2;
            s = first_slot_coord_a2;
        end
    end
end

%% Coordinate again
function [Agent, Synchs_copy, Relays_copy, Synchs_coordinated, Relays_coordinated, infinite_loop_flag] = coordinateAgain(Agent, aw, sw, ab, sb, coord_type_ind_before, Synchs, Relays, Synchs_copy, Relays_copy, Synchs_coordinated, Relays_coordinated, tol, constant_scenario_values)
    infinite_loop_flag = false;

    switch coord_type_ind_before
    case {1, 11, 101, 111}
        time_to_wait = Agent(aw).tfin_s(sw + 1) - Agent(ab).tfin_s(sb + 1);
    case 10
        time_to_wait = Agent(aw).tfin_s(sw + 1) - (Agent(ab).tfin_s(sb + 1) - Agent(ab).Te_s(sb));
    case {100, 110}
        time_to_wait = Agent(ab).tfin_s(sb + 1) - (Agent(aw).tfin_s(sw + 1) - Agent(aw).Te_s(sw));
    otherwise
        error('Missing case in switch statement.');
    end

    if abs(time_to_wait) > tol
        % Add that coordination back to the pending list and get the slot where the waiting time was applied in the old coordination
        switch coord_type_ind_before
        case {1, 11, 101, 111}
            Synchs_copy = [Synchs_copy, [aw; sw; ab; sb]];
            Synchs_copy = [Synchs_copy, [ab; sb; aw; sw]];
            Synchs_coordinated(:, Synchs_coordinated(1,:) == aw & Synchs_coordinated(2,:) == sw & Synchs_coordinated(3,:) == ab & Synchs_coordinated(4,:) == sb) = [];
            Synchs_coordinated(:, Synchs_coordinated(1,:) == ab & Synchs_coordinated(2,:) == sb & Synchs_coordinated(3,:) == aw & Synchs_coordinated(4,:) == sw) = [];
        case 10
            Relays_copy = [Relays_copy, [aw; sw; ab; sb]];
            Relays_coordinated(:, Relays_coordinated(1,:) == aw & Relays_coordinated(2,:) == sw & Relays_coordinated(3,:) == ab & Relays_coordinated(4,:) == sb) = [];
        case {100, 110}
            Relays_copy = [Relays_copy, [ab; sb; aw; sw]];
            Relays_coordinated(:, Relays_coordinated(1,:) == ab & Relays_coordinated(2,:) == sb & Relays_coordinated(3,:) == aw & Relays_coordinated(4,:) == sw) = [];
        otherwise
            error('Missing case in switch statement.');
        end

        Agent = updateTfin(Agent, constant_scenario_values);

        [Agent, Synchs_copy, Relays_copy, Synchs_coordinated, Relays_coordinated, infinite_loop_flag] = coordinateTwoSlots(Agent, aw, sw, Synchs, Relays, Synchs_copy, Relays_copy, Synchs_coordinated, Relays_coordinated, tol, constant_scenario_values);

        Agent = updateTfin(Agent, constant_scenario_values);
    end
end