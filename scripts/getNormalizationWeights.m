function [z_max, tmax_m, Tw_max, V_max, d_tmax_max, s_used_max] = getNormalizationWeights(Agent, Task)
    % Get constant scenario information values
    [Agent, Task, A, T, S, ~, R, Td_a_t_t, ~, ~] = getConstantScenarioValues(Agent, Task);
    
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
end