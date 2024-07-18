function [Agent, Task, A, T, S, N, R, Td_a_t_t, Te_t_nf, H_a_t] = getConstantScenarioValues(Agent, Task)
    % Number of Agents
    A = length(Agent);
    if A < 1
        error('There must be at least one agent');
    end

    % Number of Tasks
    T = length(Task);
    if T < 2
        error('There must be at least two tasks: Recharge and another one');
    end

    % Index of the Recharge task: R
    R = find(strcmp({Task.name}, 't_R'), 1);

    % Compute the minimum safety flight time for each robot depending on its traveling speed and the distance to the recharge station
    % Maximum distance to the recharge station from any task waypoint
    max_dist = 0;
    for t = 1:T
        if t ~= R
            dist = norm([Task(t).wp.x, Task(t).wp.y, Task(t).wp.z] - [Task(R).wp.x, Task(R).wp.y, Task(R).wp.z]);
            if dist > max_dist
                max_dist = dist;
            end
        end
    end

    % Safety flight time for each robot depending on its traveling speed and its distance to the recharge station
    for a = 1:A
        dist = norm([Agent(a).P0.x, Agent(a).P0.y, Agent(a).P0.z] - [Task(R).wp.x, Task(R).wp.y, Task(R).wp.z]);
        if dist > max_dist
            Agent(a).Ft_saf = dist / Agent(a).ts;
        else
            Agent(a).Ft_saf = max_dist / Agent(a).ts;
        end
    end

    % Calculate Td(t1,t2) matrix
    % Due to initial positions, Td(t1,t2) will depend also on the agent. So it's renamed as Td(a,t1,t2)
    Td_a_t_t = zeros(A,T,T+1);
    for a = 1:A
        for t = 1:T
            for t2 = 0:T
                if t2 == 0
                    Td_a_t_t(a,t,t2 + 1) = norm([Task(t).wp.x, Task(t).wp.y, Task(t).wp.z] - [Agent(a).P0.x, Agent(a).P0.y, Agent(a).P0.z])/Agent(a).ts;
                else
                    Td_a_t_t(a,t,t2 + 1) = norm([Task(t).wp.x, Task(t).wp.y, Task(t).wp.z] - [Task(t2).wp.x, Task(t2).wp.y, Task(t2).wp.z])/Agent(a).ts;
                end
            end
        end
    end

    % Estimate the upper bound for the number of fragments a task is divided into
    N = 0;
    for t = 1:T
        if t ~= R
            % Task execution time corrected with the fragmentation loses
            aux_Te = Task(t).Te * (1 + Task(t).Fl/100);

            % Set heuristically the number of robots needed for the task: nr(t)
            if Task(t).N_hardness == 1
                % If the coalition size flexibility is fixed, nr(t) = N(t)
                Task(t).nr = Task(t).N;
            else
                % If the coalition size flexibility is variable or unspecified, nr(t) = 1
                Task(t).nr = 1;
            end

            % Get the flight time of the compatible robots corrected with the displacement time and the safety flight time and sort them from the highest to the lowest
            aux_Ft = sort(nonzeros(([Agent.Ft] - [Agent.Ft_saf] - max(Td_a_t_t(:))).*ismember([Agent.type], Task(t).Hr)), 'descend');

            % Check if there are enough robots to perform the task
            if Task(t).nr > length(aux_Ft)
                error('There are no robots to perform the task');
            end

            % Non-decomposable tasks:
            if Task(t).Fragmentability == 0 && Task(t).Relayability == 0
                % Take the flight time of the Task(t).nr-th compatible robot with highest flight time
                aux_Ft = aux_Ft(Task(t).nr);
            % Fragmentable or decomposable tasks:
            else
                % Take the flight time of the compatible robot that has the least flight
                aux_Ft = min(aux_Ft);
            end

            % Maximum number of robots that can perform task t: compatible robots
            nc = sum(ismember([Agent.type], Task(t).Hr));

            % Number of robots that can perform task t and are not needed and so can be used as relays: exceeding robots
            ne = nc - Task(t).nr;

            % Correction for taking into account the task decomposability and coalition flexibility
            correction_factor = 1;

            % Only relayable tasks need correction, mostly N-hard ones
            if Task(t).Relayability == 1
                % Check if the task need relays depending on its execution time
                if aux_Te > aux_Ft
                    % If the task is hard and no extra robots are available, there wont be a solution
                    % If the task is soft but there is only one robot available, there wont be a solution
                    % Otherwise, a correction factor is computed
                    if ne == 0
                        error('There are no robots to perform relays');
                    else
                        correction_factor = ceil(Task(t).nr / ne);
                    end
                end
            end

            % Number of fragments needed for task t
            if Task(t).Fragmentability == 0 && Task(t).Relayability == 0
                if aux_Te > aux_Ft
                    error('There are no robots to perform the task');
                end
                nf = 1;
            else
                nf = ceil(correction_factor * aux_Te / (aux_Ft - Task(R).Te));
            end
            
            % Update the maximum number of fragments
            if nf > N
                N = nf;
            end

            % Add information to the task structure
            % Number of compatible robots
            Task(t).nc = nc;

            % Number of exceeding robots (nc - Task(t).nr)
            Task(t).ex = ne;

            % Correction factor
            Task(t).cf = correction_factor;

            % Number of fragments needed
            Task(t).nf = nf;

            % Recharge frequency (each f consecutive fragments in a robot, 1 recharge is needed)
            Task(t).f = floor((aux_Ft - Task(R).Te) / (aux_Te / nf));
        else
            Task(t).nr = 0;
            Task(t).nc = A;
            Task(t).ex = A;
            Task(t).cf = 1;
            Task(t).nf = 1;
            Task(t).f  = 0;
        end
    end
    % Add some extra fragments for slack purposes
    slack_N = 0;
    N = N + slack_N;

    % Calculate Te(t,nf) matrix
    Te_t_nf = ones(T,N);
    Te_t_nf(R,:) = Te_t_nf(R,:) * Task(R).Te;
    for t = 1:T
        if t ~= R
            Te_t_nf(t,1) = Task(t).Te;
            for nf = 2:N
                % Fragmentable tasks have fragmentation loses
                if Task(t).Fragmentability == 1
                    Te_t_nf(t,nf) = Task(t).Te * (1 / nf + Task(t).Fl / 100);
                % Non-fragmentable tasks don't have fragmentation loses when divided if allowed
                else
                    Te_t_nf(t,nf) = Task(t).Te / nf;
                end
            end
        end
    end

    % Agent - Task compatibility
    H_a_t = zeros(A, T);
    for a = 1:A
        for t = 1:T
            if (ismember(Agent(a).type, Task(t).Hr))
                H_a_t(a,t) = 1;
            end
        end
    end

    % The maximum possible slots to be needed is 2*(T-1)*max(nf). As relays are allowed, tasks may be assigned to more than a single Agent and more than once. Also, a recharge may be needed before each task. However, sometimes a smaller value for S could be set safely.
    % A slightly more realistic value for S would be 2*sum from t = 2 to T of (nf(t))
    % nf(t) can be estimated as ceil((max(Td(t) + Te(t)*(1 + Task(t).Fl)) / min((Ft(a) - Ft_saf(a))|Hr))
    % To simplify, est_nf(t) = ceil((Te(t) * (1 + Task(t).Fl)) / (min((Ft(a) - Ft_saf(a)) - Td_max)*ismember([Agent.type], Task(t).Hr)))
    S = 0;
    for t = 1:T
        if t ~= R
            N_needed = Task(t).N;
            if N_needed == 0
                N_needed = 1;
            end
            % Task execution time corrected with the fragmentation loses and weighted by the number of robots the task needs
            aux_Te = Task(t).Te * (1 + Task(t).Fl/100);
            % Flight time of the compatible robot that has the least flight time corrected with the displacement time and the safety flight time
            aux_Ft = min(nonzeros(([Agent.Ft] - [Agent.Ft_saf] - max(Td_a_t_t(:))).*ismember([Agent.type], Task(t).Hr)));
            % Correction for taking into account the task decomposability and coalition flexibility
            % Only relayable tasks need correction, mostly N-hard ones
            if Task(t).Relayability == 1
                % Maximum number of robots that can perform task t
                nc = sum(ismember([Agent.type], Task(t).Hr));

                % Number of robots that can perform task t and are not needed and so can be used as relays
                ne = nc - N_needed;

                % Check if the task need relays depending on its execution time
                if aux_Te > aux_Ft
                    % If the task is hard and no extra robots are available, there wont be a solution
                    % If the task is soft but there is only one robot available, there wont be a solution
                    % Otherwise, a correction factor is computed
                    if (ne == 0 && Task(t).N_hardness == 1) || (Task(t).N_hardness == 0 && nc - 1 == 0)
                        display('WARNING: There wont be a solution because there is no available robot to perform the needed relays');
                        correction_factor = 1;
                    elseif ne > 0
                        correction_factor = ceil(N_needed / ne);
                    elseif Task(t).N_hardness == 0
                        correction_factor = ceil(N_needed / (nc - 1));
                    end
                end
            else
                correction_factor = 1;
            end

            % Number of slots needed for task t
            S = S + ceil(correction_factor * aux_Te * N_needed / aux_Ft);
        end
    end

    % Add some extra slots for slack purposes
    slack_S = 0; %(T - 1);
    S = S + slack_S;

    % Check if initializer has been run to make sure S get the correct value
    if isfield(Agent, 'queue')
        % Get the number of used slots per robot
        used_slots = zeros(1,A);
        for a = 1:A
            used_slots(a) = length(Agent(a).queue) - 1;
        end

        % Check if initializer uses more than S
        if max(used_slots) > S
            S = max(used_slots);
        end
    end
end
