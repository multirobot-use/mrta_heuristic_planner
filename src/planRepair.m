function [Agent, Task, result] = planRepair(Agent, Task, Synchs, Relays, delay)
    % Inputs:
        % Agent:  Robot structure array
        % Task:   Task structure array
        % synch:  List of slots to be synchronized
        % relays: List of relays to be coordinated
        % delay:  Array containing the amount of delay in second and the slot delayed ([d r s])
    % Result is false if the plan can't be repaired, true otherwise
    result = true;

    % Check if the number of inputs is correct
    if nargin ~= 5
        error('Not enough input arguments');
    end

    % Check if the delay input is correct
    if length(delay) ~= 3
        error('Delay input is incorrect');
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
    % Find out the finish time of the task that got delayed
    delayed_task_finish_time = Agent(delay(2)).tfin_s(delay(3) + 1) + delay(1);

    % Fill an auxiliary variable with the waiting times of the tasks that have already been executed
    for a = 1:A
        % Create the auxiliary variable to store the already executed waiting times
        Agent(a).exTw_s = zeros(1,S);
        
        % Copy the already executed waiting times into exTw_s
        for s = 1:S
            % Case 1: The task has already been executed
            if Agent(a).tfin_s(s + 1) <= delayed_task_finish_time
                Agent(a).exTw_s(s) = Agent(a).Tw_s(s);
            % Case 2: The task hasn't started yet
            elseif Agent(a).tfin_s((s - 1) + 1) + Agent(a).Td_s(s) >= delayed_task_finish_time
                break;
            % Case 3: The task is being executed, the waiting time has already been applied
            elseif Agent(a).tfin_s((s - 1) + 1) + Agent(a).Td_s(s) + Agent(a).Tw_s(s) <= delayed_task_finish_time
                Agent(a).exTw_s(s) = Agent(a).Tw_s(s);
            % Case 4: The task is being executed, the waiting time was not (totally) executed
            else
                Agent(a).exTw_s(s) = delayed_task_finish_time - Agent(a).tfin_s((s - 1) + 1) - Agent(a).Td_s(s);
            end
        end

        % Set the waiting times back to 0
        Agent(a).Tw_s = zeros(1,S);
    end

    % Add the delay time to the auxiliary variable
    Agent(delay(2)).exTw_s(delay(3)) = Agent(delay(2)).exTw_s(delay(3)) + delay(1);

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
                    % Get (a2,s2) to be able to call coordinateTwoSlots for the first time (note: inside coordinateTwoSlots, (a2,s2) is computed again because at the end of the main while loop, (a,s) can change to the old (a2,s2))
                    [a2, s2] = geta2s2(Agent, a, s, coord_type_ind, Synchs_copy, Relays_copy, constant_scenario_values);
                    [Agent, Synchs_copy, Relays_copy, Synchs_coordinated, Relays_coordinated, infinite_loop_flag] = coordinateTwoSlots(Agent, a, s, a2, s2, Synchs, Relays, Synchs_copy, Relays_copy, Synchs_coordinated, Relays_coordinated, tol, constant_scenario_values);
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
    for a = 1:A
        for s = 1:length(Agent(a).queue) - 1
            Agent(a).tfin_s(s + 1)  = Agent(a).tfin_s((s - 1) + 1) + Agent(a).Td_s(s) + Agent(a).Tw_s(s) + Agent(a).Te_s(s) + Agent(a).exTw_s(s);
            % Check if the task is a recharge task
            if Agent(a).queue(s + 1) == R
                Agent(a).ac_Ft_s(s + 1) = Agent(a).Td_s(s);
            else
                Agent(a).ac_Ft_s(s + 1) = Agent(a).Td_s(s) + Agent(a).Tw_s(s) + Agent(a).Te_s(s);
            end
            
            % If last task was not a recharge, add also the last accumulated flight time
            if Agent(a).queue((s - 1) + 1) ~= R
                Agent(a).ac_Ft_s(s + 1) = Agent(a).ac_Ft_s(s + 1) + Agent(a).ac_Ft_s((s - 1) + 1);
            end

            % Check if there is enough battery to finish the task
            if Agent(a).ac_Ft_s(s + 1) > Agent(a).Ft - Agent(a).Ft_saf
                disp(['Insufficient battery to finish task ', num2str(Agent(a).queue(s + 1) - 1), ' in slot ', num2str(s), ' agent ', num2str(a)]);
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
        for s = 1:S
            % tfin_a(s) = tfin(a,s-1) + Td(a,s) + Tw(a,s) + Te(a,s)
            Agent(a).tfin_s(s + 1) = Agent(a).tfin_s((s - 1) + 1) + Agent(a).Td_s(s) + Agent(a).Tw_s(s) + Agent(a).Te_s(s) + Agent(a).exTw_s(s);
        end
    end
end

%% Get (a2,s2)
function [a2, s2] = geta2s2(Agent, a, s, coord_type_ind, Synchs_copy, Relays_copy, constant_scenario_values)
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
function [Agent, Synchs_copy, Relays_copy, Synchs_coordinated, Relays_coordinated, infinite_loop_flag] = coordinateTwoSlots(Agent, a, s, a2, s2, Synchs, Relays, Synchs_copy, Relays_copy, Synchs_coordinated, Relays_coordinated, tol, constant_scenario_values)
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
        [a2, s2] = geta2s2(Agent, a, s, coord_type_ind, Synchs_copy, Relays_copy, constant_scenario_values);
        
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
                if flag_any_recharge && not(flag_any_coordination)
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

        [Agent, Synchs_copy, Relays_copy, Synchs_coordinated, Relays_coordinated, infinite_loop_flag] = coordinateTwoSlots(Agent, aw, sw, ab, sb, Synchs, Relays, Synchs_copy, Relays_copy, Synchs_coordinated, Relays_coordinated, tol, constant_scenario_values);

        Agent = updateTfin(Agent, constant_scenario_values);
    end
end