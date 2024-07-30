%% Function to build the decision variable array from a heuristic solution
%! ----------------------------------------------------------
function [fval] = computeFval(Agent, Task, objective_function)
    % Minimum required inputs: Agent and Task

    if not(isfield(Agent, 'queue'))
        error('Incorrect solution input array');
    end
    
    % Objective function (options):
    %   - 1. Minimize the longest queue's execution time: min(max(tfin(a,S))).
    %   - 2. Minimize the total joint flight time: min(sum(tfin(a,S))).
    %   - 3. Same as 1. but without used slots term.
    %   - 4. Combination of 1 and 2. Use parameters alpha and betta to give more importance to one over another.
    %   - 5. Makespan, total joint flight time, total joint consumed battery time, coalition size deviation, deadline delays.
    if nargin < 3 || isempty(objective_function)
        objective_function = 1;
    end
    alpha = 0.8;
    betta = 1 - alpha;

    % Get scenario information
    [Agent, Task, A, T, S, ~, R, Td_a_t_t, Te_t_nf, ~] = getConstantScenarioValues(Agent, Task);

    % z is normalized by scaling its value between the theoretical maximum and minimum value for the total execution time.
    % The minimum value is not 0, but for simplicity, it will be considered as 0.
    % The maximum value can be computed as the sum of all tasks assigned to the slowest agent with a recharge task between them. 
    % This would be Td_a_t_t(a,0,R) + sum from t = 2 to T of (2 * Td_a_t_t(a,R,t) + Te_t_nf(t,1)) + T*Te_t_nf(R,1), but for simplicity, it will be considered as sum from t = 1 to T of (Task(t).Te)
    z_max = 0;
    for t = 1:T
        if t ~= R
            z_max = z_max + Task(t).Te;
        end
    end
    if z_max == 0
        z_max = 1;
    end

    % Maximum time of the mission (s)
    Task_Te = [Task.Te];
    tmax_m = T * (Task_Te(R) + max(Task_Te([1:R-1 R+1:T])) + max(max(max(Td_a_t_t))));

    % Tw is normalized by scaling its value between zero and the maximum Agent.Ft value.
    Tw_max = max([Agent.Ft]);

    % V is normalized by scaling its value between the theoretical maximum and minimum value for the total vacancies.
    % Task.N = 0 -> not specified, it doesn't penalize in the objective function.
    % Task.N ~= 0 -> specified, so the number of selected Agents should be equal to Task.N, if not, it would penalize in the objective function.
    % The vacancies minimum value is 0, while the maximum value can be computed as sum from t = 2 to T of (Task(t).N - 1)
    V_max = 0;
    for t = 1:T
        if t ~= R
            if Task(t).N ~= 0
                V_max = V_max + Task(t).N - 1;
            end
        end
    end
    if V_max == 0
        V_max = 1;
    end

    % d_tmax_tfin_a_s is normalized by scaling its value between zero and the maximum tmax(t) value.
    d_tmax_max = max([Task.tmax]);

    % s_used is normalized by scaling its value between zero and the total number of slots.
    s_used_max = S*A;

    % Update Recharge maximum time
    Task(R).tmax = tmax_m;

    % Extract tfin_a_s from Agent.tfin
    tfin_a_s = zeros(A, S+1);
    for a = 1:A
        for s = 1:S
            tfin_a_s(a, s + 1) = Agent(a).tfin_s(s + 1);
        end
    end

    % Extract consumed battery time (Bc_a_s) from Agent.queue, Agent.Td_s, Agent.Tw_s and Agent.Te_S
    Bc_a_s = zeros(A, S);
    for a = 1:A
        for s = 1:length(Agent(a).queue) - 1
            t = Agent(a).queue(s + 1);
            if t == R
                Bc_a_s(a, s) = Agent(a).Td_s(s);
            else
                Bc_a_s(a, s) = Agent(a).Td_s(s) + Agent(a).Tw_s(s) + Task(t).Te;
            end
        end
    end
    
    % Objective function coefficients
    % Compute makespan
    makespan = max(max(tfin_a_s));

    % Compute the total joint flight time
    total_Ft = sum(tfin_a_s(:, S + 1));

    % Compute the total waiting time
    total_Tw = 0;
    for a = 1:A
        total_Tw = total_Tw + sum(Agent(a).Tw_s);
        
        % Check if this plan has been repaired
        if isfield(Agent, 'exTw_s')
            total_Tw = total_Tw + sum(Agent(a).exTw_s);
        end
    end

    % Compute the number of used slots
    s_used = 0;
    for a = 1:A
        s_used = s_used + length(Agent(a).queue) - 1;
    end

    % Compute the coalition size deviation
    task_N  = [Task.N];
    task_na = [Task.na];
    task_na(task_N == 0) = [];
    task_N(task_N == 0)  = [];
    CSD = sum(task_N - task_na);

    % Compute the deviation from the tmax of a task (Deadline Deviation (DD))
    DD = 0;
    for a = 1:A
        s = 0;
        for task = 1:length(Agent(a).queue) - 1
            t = Agent(a).queue(task + 1);
            s = s + 1;
            d_tmax_tfin = tfin_a_s(a, s + 1) - Task(t).tmax;
            if d_tmax_tfin > 0
                DD = DD + d_tmax_tfin;
            end
        end
    end

    % Compute the total joint consumed battery time
    CBT = sum(sum(Bc_a_s));

    switch objective_function
    case 2
        % Minimize the total joint flight time: min(sum(tfin(a,S))), for all a = 1 to A.
        % CSD: Coefficients of the term penalizing the use of a different number of Agents. V(2 to T) to exclude Recharge task.
        % d_tmax_tfin_a_s: Coefficients of the term penalizing exceeding the tmax of a task.
        % Note: no need to minimize Tw as its included in the total joint flight time.
        fval = total_Ft/tmax_m + CSD/V_max + DD/d_tmax_max;
    case 3
        % Minimize the longest queue's execution time: min(max(tfin(a,S))) -> min(z).
        % Tw: Coefficients of the term minimizing the waiting time to avoid unnecessary waitings.
        % CSD: Coefficients of the term penalizing the use of a different number of Agents. V(2 to T) to exclude Recharge task.
        % d_tmax_tfin_a_s: Coefficients of the term penalizing exceeding the tmax of a task.
        fval = makespan/z_max + total_Tw/Tw_max + CSD/V_max + DD/d_tmax_max;
    case 4
        % Minimize the longest queue's execution time: min(max(tfin(a,S))) -> min(z).
        % Minimize the total joint flight time: min(sum(tfin(a,S))), for all a = 1 to A.
        % Tw: Coefficients of the term minimizing the waiting time to avoid unnecessary waitings.
        % CSD: Coefficients of the term penalizing the use of a different number of Agents. V(2 to T) to exclude Recharge task.
        % d_tmax_tfin_a_s: Coefficients of the term penalizing exceeding the tmax of a task.
        % Note: here Tw penalises twice, first in the total joint flight time term, then in the Tw term
        fval = makespan*alpha/z_max + total_Ft*betta/tmax_m + total_Tw/Tw_max + CSD/V_max + DD/d_tmax_max;
    case 5
        % Minimize the longest queue's execution time: min(max(tfin(a,S))) -> min(z).
        % Minimize the total joint flight time: min(sum(tfin(a,S))), for all a = 1 to A.
        % Minimize the total joint consumed battery time
        % CSD: Coefficients of the term penalizing the use of a different number of Agents. V(2 to T) to exclude Recharge task.
        % d_tmax_tfin_a_s: Coefficients of the term penalizing exceeding the tmax of a task.
        fval = makespan/z_max + total_Ft/tmax_m + CSD/V_max + DD/d_tmax_max + CBT/(A*Tw_max);
    otherwise
        % Minimize the longest queue's execution time: min(max(tfin(a,S))) -> min(z).
        % Tw: Coefficients of the term minimizing the waiting time to avoid unnecessary waitings.
        % s_used: Coefficients of the term minimizing the number of used slots.
        % CSD: Coefficients of the term penalizing the use of a different number of Agents. V(2 to T) to exclude Recharge task.
        % d_tmax_tfin_a_s: Coefficients of the term penalizing exceeding the tmax of a task.
        fval = makespan/z_max + total_Tw/Tw_max + s_used/s_used_max + CSD/V_max + DD/d_tmax_max;
    end
end