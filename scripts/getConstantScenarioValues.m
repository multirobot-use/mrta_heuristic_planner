function [A, T, S, N, R, Td_a_t_t, Te_t_nf, H_a_t] = getConstantScenarioValues(Agent, Task)
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
    for t = 1:T
        if strcmp(Task(t).name, 't_R')
            R = t;
            break;
        end
    end

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
    N = 2 + ceil((max([Task.Te]) * (1 + max([Task.Fl]) / 100)) / (min([Agent.Ft]) - max([Agent.Ft_saf]) - max(max(max(Td_a_t_t)))));

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

    % The maximum possible slots to be needed is 2*(T-1)*max(nf). As relays are allowed, tasks may be assigned to more than a single Agent and more than once. Also, a recharge may be needed before each task. However, sometimes a smaller value for S could be set safely.
    % A slightly more realistic value for S would be 2*sum from t = 2 to T of (nf(t))
    % nf(t) can be estimated as ceil((max(Td(t) + Te(t)*(1 + Task(t).Fl)) / min((Ft(a) - Ft_saf(a))|Hr))
    % To simplify, est_nf(t) = ceil((Td_max + Te(t) * (1 + Task(t).Fl)) / min((Ft(a) - Ft_saf(a))*ismember([Agent.type], Task(t).Hr)))
    S = 0;
    est_nf = 0;
    for t = 1:T
        if t ~= R
            est_nf = ceil((max(max(max(Td_a_t_t))) + Te_t_nf(t,1) * (1 + Task(t).Fl/100)) / min(nonzeros(([Agent.Ft] - [Agent.Ft_saf]).*ismember([Agent.type], Task(t).Hr))));
            S = S + est_nf;
        end
    end
    S = 2 * S;

    % Agent - Task compatibility
    H_a_t = zeros(A, T);
    for a = 1:A
        for t = 1:T
            if (ismember(Agent(a).type, Task(t).Hr))
                H_a_t(a,t) = 1;
            end
        end
    end
end