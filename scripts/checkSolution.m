%% Function to check if a handmade matrix solution is correct
%! ----------------------------------------------------------
function [dv, fval, result] = checkSolution(sol, Agent, Task, execution_id, print_coord_steps_flag)
    % Minimum required inputs: sol, Agent, Task

    % Numerical tolerance
    tol = 1e-6;

    % Objective function (options):
    %   - 1. Minimize the longest queue's execution time: min(max(tfin(a,S))).
    %   - 2. Minimize the total joint flight time: min(sum(tfin(a,S))).
    objective_function = 0;

    % Get scenario information
    [A, T, S, N, R, Td_a_t_t, Te_t_nf, H_a_t] = getConstantScenarioValues(Agent, Task);
    [z_max, tmax_m, Tw_max, U_max, d_tmax_max, s_used_max] = getNormalizationWeights(Agent, Task);

    % Get start length structure information
    [dv_start_length, length_dv] = getStartLengthInformation(Agent, Task);

    %% Get all dv start and length information
    [start_z                   length_z                   ...
     start_x_a_t_s             length_x_a_t_s             ...
     start_xx_a_t_t_s          length_xx_a_t_t_s          ...
     start_V_t                 length_V_t                 ...
     start_U_t                 length_U_t                 ...
     start_n_t                 length_n_t                 ...
     start_na_t                length_na_t                ...
     start_nq_t                length_nq_t                ...
     start_nq_a_t              length_nq_a_t              ...
     start_nf_t                length_nf_t                ...
     start_nf_t_nf             length_nf_t_nf             ...
     start_nfx_a_nf_t_s        length_nfx_a_nf_t_s        ...
     start_naf_t_nf            length_naf_t_nf            ...
     start_nax_a_t_s           length_nax_a_t_s           ...
     start_Ft_a_s              length_Ft_a_s              ...
     start_Ftx_a_s1            length_Ftx_a_s1            ...
     start_tfin_a_s            length_tfin_a_s            ...
     start_tfinx_a_t_s         length_tfinx_a_t_s         ...
     start_d_tmax_tfin_a_s     length_d_tmax_tfin_a_s     ...
     start_s_used              length_s_used              ...
     start_Td_a_s              length_Td_a_s              ...
     start_Tw_a_s              length_Tw_a_s              ...
     start_Twx_a_s             length_Twx_a_s             ...
     start_Te_a_s              length_Te_a_s              ...
     start_S_t_a1_s1_a2_s2     length_S_t_a1_s1_a2_s2     ...
     start_tfinS_t_a1_s1_a2_s2 length_tfinS_t_a1_s1_a2_s2 ...
     start_R_t_a1_s1_a2_s2     length_R_t_a1_s1_a2_s2     ...
     start_tfinR_t_a1_s1_a2_s2 length_tfinR_t_a1_s1_a2_s2 ...
     start_TeR_t_a1_s1_a2_s2   length_TeR_t_a1_s1_a2_s2      ] = extractStartLengthInformation(dv_start_length);
    
    % Create an array for the decision variables
    dv = zeros(length_dv, 1);

    %% Index of the Recharge task
    for t = 1:T
        if strcmp(Task(t).name, 't_R')
            R = t;
            break;
        end
    end

    % Put the values of x_a_t_s, nf_t, S_t_a1_s1_a2_s2 and R_t_a1_s1_a2_s2 in their corresponding position in the decision variables array
    % Check if the first input if the whole solution array or just the x_a_t_s part
    if length(sol) == length_dv
        x_a_t_s = sol(start_x_a_t_s : start_x_a_t_s + length_x_a_t_s - 1);
        nf_t = sol(start_nf_t : start_nf_t + length_nf_t - 1);
        S_t_a1_s1_a2_s2 = sol(start_S_t_a1_s1_a2_s2 : start_S_t_a1_s1_a2_s2 + length_S_t_a1_s1_a2_s2 - 1);
        R_t_a1_s1_a2_s2 = sol(start_R_t_a1_s1_a2_s2 : start_R_t_a1_s1_a2_s2 + length_R_t_a1_s1_a2_s2 - 1);
    elseif numel(sol) == length_x_a_t_s + length_nf_t + length_S_t_a1_s1_a2_s2 + length_R_t_a1_s1_a2_s2
        prev_end = 0;
        x_a_t_s = sol(prev_end + 1:length_x_a_t_s);
        prev_end = prev_end + length_x_a_t_s; 
        nf_t = sol(prev_end + 1 : prev_end + length_nf_t);
        prev_end = prev_end + length_nf_t;
        S_t_a1_s1_a2_s2 = sol(prev_end + 1 : prev_end + length_S_t_a1_s1_a2_s2);
        prev_end = prev_end + length_S_t_a1_s1_a2_s2;
        R_t_a1_s1_a2_s2 = sol(prev_end + 1 : prev_end + length_R_t_a1_s1_a2_s2);
        prev_end = prev_end + length_R_t_a1_s1_a2_s2;
    else
        error('Incorrect solution input array');
    end
    dv(start_x_a_t_s : start_x_a_t_s + length_x_a_t_s - 1) = reshape(x_a_t_s,1,[]);
    dv(start_nf_t : start_nf_t + length_nf_t - 1) = reshape(nf_t,1,[]);
    dv(start_S_t_a1_s1_a2_s2 : start_S_t_a1_s1_a2_s2 + length_S_t_a1_s1_a2_s2 - 1) = reshape(S_t_a1_s1_a2_s2,1,[]);
    dv(start_R_t_a1_s1_a2_s2 : start_R_t_a1_s1_a2_s2 + length_R_t_a1_s1_a2_s2 - 1) = reshape(R_t_a1_s1_a2_s2,1,[]);

    % Auxiliary copy of S and R to coordinate the slots one by one
    S_t_a1_s1_a2_s2 = reshape(S_t_a1_s1_a2_s2, T-1, A, S, A, S);
    R_t_a1_s1_a2_s2 = reshape(R_t_a1_s1_a2_s2, T-1, A, S, A, S);

    % Auxiliary variable to store the already coordinated slots
    S_t_a1_s1_a2_s2_coordinated = zeros(T-1, A, S, A, S);
    R_t_a1_s1_a2_s2_coordinated = zeros(T-1, A, S, A, S);

    % Auxiliary variable to store where was the Tw applied and its value. key: strcat(num2str(t),',',num2str(a1),',',num2str(s1),',',num2str(a2),',',num2str(s2)), value: [Tw, aw, sw]
    slot_Tw_coordinated = containers.Map();

    % Objective function coefficients
    f = zeros(1,length_dv);
    switch objective_function
        case 2
            % Minimize the total joint flight time: min(sum(tfin(a,S))), for all a = 1 to A.
            f((start_tfin_a_s - 1) + (1 + ((S + 1) - 1)*A):(start_tfin_a_s - 1) + (A + ((S + 1) - 1)*A)) = 1/tmax_m;
        otherwise
            % Minimize the longest queue's execution time: min(max(tfin(a,S))) -> min(z).
            f((start_z - 1) + 1) = 1/z_max;
            
            % Tw: Coefficients of the term minimizing the waiting time to avoid unnecessary waitings.
            f((start_Tw_a_s - 1) + 1 : (start_Tw_a_s - 1) + length_Tw_a_s) = 1/Tw_max;
    end

    % U: Coefficients of the term penalizing the use of a different number of Agents. U(2 to T) to exclude Recharge task.
    f((start_U_t - 1) + 2 : (start_U_t - 1) + length_U_t) = 1/U_max;

    % d_tmax_tfin_a_s: Coefficients of the term penalizing exceeding the tmax of a task.
    f((start_d_tmax_tfin_a_s - 1) + 1 : (start_d_tmax_tfin_a_s - 1) + length_d_tmax_tfin_a_s) = 1/d_tmax_max;

    % s_used: Coefficients of the term minimizing the number of used slots.
    f((start_s_used - 1) + 1 : (start_s_used - 1) + length_s_used) = 1/s_used_max;

    %% Calculate the values of the decision variables using the equality constraints (equations)
    %? xx_a_t_t_s
    %* Depends on: x_a_t_s
    for a = 1:A
        for s = 1:S
            for t = 1:T
                for t2 = 0:T
                    % xx_a_t_t_s = xa(t,s) * xa(t2,s-1)
                    dv((start_xx_a_t_t_s - 1) + (a + ((t2 + 1) - 1)*A + (t - 1)*A*(T+1) + (s - 1)*A*(T+1)*T)) = dv((start_x_a_t_s - 1) + (a + ((t + 1) - 1)*A + ((s + 1) - 1)*A*(T+1))) * dv((start_x_a_t_s - 1) + (a + ((t2 + 1) - 1)*A + (((s-1) + 1) - 1)*A*(T+1)));
                end
            end
        end
    end

    %? n_t
    %* Depends on: x_a_t_s
    % n(t): Number of Agents that are selected to carry out task t.
    % n(t) = sum from a = 1 to A of (sum from s = 1 to S of (xa(t,s))), for all t = 1 to T
    for t = 1:T
        aux_n_t_a = 0;
        for a = 1:A
            aux_n_t_a_s = 0;
            for s = 1:S
                aux_n_t_a_s = aux_n_t_a_s + dv((start_x_a_t_s - 1) + (a + ((t + 1) - 1)*A + ((s + 1) - 1)*A*(T+1)));
            end
            aux_n_t_a = aux_n_t_a + aux_n_t_a_s;
        end
        dv((start_n_t - 1) + (t)) = aux_n_t_a;
    end

    %? nf_t_nf
    %* Depends on: nf_t
    % Set nf_t_nf aux decision variable value depending on nf_t
    % sum from nf = 1 to N of (nf * nf(t,nf)) = nf(t), for all t = 1 to T
    % nf_t_nf value shouldn't be a combination of several nf values
    % sum from nf = 1 to N of (nf(t,nf)) = 1, for all t = 1 to T
    for t = 1:T
        for nf = 1:N
            if abs(dv((start_nf_t - 1) + t) - nf) < tol
                dv((start_nf_t_nf - 1) + (t + (nf - 1)*T)) = 1;
            else
                dv((start_nf_t_nf - 1) + (t + (nf - 1)*T)) = 0;
            end
        end
    end

    %? nq_a_t
    %* Depends on: x_a_t_s
    % nq(a,t) = OR(xat(s)), for all t = 1 to T, for all a = 1 to A (S input logic OR)
    for t = 1:T
        for a = 1:A
            dv((start_nq_a_t - 1) + (a + (t - 1)*A)) = any(dv((start_x_a_t_s - 1) + (a + ((t + 1) - 1)*A + (1 - 1)*A*(T+1)) : A*(T+1) : (start_x_a_t_s - 1) + (a + ((t + 1) - 1)*A + (S - 1)*A*(T+1))));
        end
    end

    %? nq_t
    %* Depends on: nq_a_t
    % nq(t): number of queues that task t appears in.
    % nq(t) = sum from a = 1 to A of (nq(a,t)), for all t = 1 to T
    for t = 1:t
        aux_nq_t_a = 0;
        for a = 1:A
            aux_nq_t_a = aux_nq_t_a + dv((start_nq_a_t - 1) + (a + (t - 1)*A));
        end
        dv((start_nq_t - 1) + (t)) = aux_nq_t_a;
    end

    %? na_t
    %* Depends on: n_t, nf_t
    % na(t) = n(t)/nf(t), for all t = 2 to T
    for t = 1:T
        if t ~= R
            dv((start_na_t - 1) + (t)) = dv((start_n_t - 1) + (t)) / dv((start_nf_t - 1) + (t));
        end
    end

    %? naf_t_nf
    %* Depends on: na_t, nf_t_nf
    for t = 1:T
        for nf = 1:N
            dv((start_naf_t_nf - 1) + (t + (nf - 1)*T)) = dv((start_na_t - 1) + (t)) * dv((start_nf_t_nf - 1) + (t + (nf - 1)*T));
        end
    end

    %? nfx_a_nf_t_s
    %* Depends on: x_a_t_s, nf_t_nf
    for a = 1:A
        for s = 1:S
            for t = 1:T
                for nf = 1:N
                    % nfx_a_nf_t_s = xa(t,s) * nf(t, nf)
                    dv((start_nfx_a_nf_t_s - 1) + (a + (nf - 1)*A + (t - 1)*A*N + (s - 1)*A*N*T)) = dv((start_x_a_t_s - 1) + (a + ((t + 1) - 1)*A + ((s + 1) - 1)*A*(T+1))) * dv((start_nf_t_nf - 1) + (t + (nf - 1)*T));
                end
            end
        end
    end

    %? nax_a_t_s
    %* Depends on: na(t), x(a1,t,s1)
    for a = 1:A
        for s = 1:S
            for t = 1:T
                % na(t) * x(a,t,s) -> nax_a_t_s
                dv((start_nax_a_t_s - 1) + (a + (t - 1)*A + (s - 1)*A*T)) = dv((start_na_t - 1) + (t)) * dv((start_x_a_t_s - 1) + (a + ((t + 1) - 1)*A + ((s + 1) - 1)*A*(T+1)));
            end
        end
    end

    %? V_t
    %* Depends on: n_t
    % "Task.N simultaneous Agents" constraint
    % V(t) + na(t) = N(t), for all t = 2 to T if N(t) > 0
    for t = 1:T
        if t ~= R
            if Task(t).N > 0
                dv((start_V_t - 1) + (t)) = Task(t).N - dv((start_na_t - 1) + (t));
            end
        end
    end

    %? U_t
    %* Depends on: V_t
    % U(t) = abs(V(t))
    for t = 1:T
        if t ~= R
            dv((start_U_t - 1) + (t)) = abs(dv((start_V_t - 1) + (t)));
        end
    end

    %? s_used value
    %* Depends on: x_a_t_s
    % s_used = sum from a = 1 to A of (sum from t = 0 to T of (sum from s = 0 to S of (xa(t,s)))) - A
    dv((start_s_used - 1) + 1) = sum(dv(start_x_a_t_s : start_x_a_t_s + length_x_a_t_s - 1)) - A;

    %? Td_a_s
    %* Depends on xx_a_t_t_s
    % Td(a,s) = sum from t = 1 to T of (sum from t2 = 0 to T of (Td_a(t,t2) * xa(t2,s-1)) * xa(t,s)) -> sum from t = 1 to T of (sum from t2 = 0 to T of (Td_a(t,t2) * xa(t2,s-1) * xa(t,s)))
    for a = 1:A
        for s = 1:S
            aux_Td = 0;
            for t = 1:T
                % Td(a,t,s)
                aux_Td_a_t_s = 0;
                for t2 = 0:T
                    % xa(t2,s-1)*xa(t,s) -> xx_a_t_t_s
                    aux_Td_a_t_s = aux_Td_a_t_s + Td_a_t_t(a,t,(t2 + 1))*dv((start_xx_a_t_t_s - 1) + (a + ((t2 + 1) - 1)*A + (t - 1)*A*(T+1) + (s - 1)*A*(T+1)*T));
                end
                % Td(a,s)
                aux_Td = aux_Td + aux_Td_a_t_s;
            end
            dv((start_Td_a_s - 1) + (a + (s - 1)*A)) = aux_Td;
        end
    end

    %? Te_a_s
    %* Depends on nfx_a_nf_t_s
    % Te(a,s) = sum from t = 1 to T of (sum from nf = 1 to N of (Tf(t,nf) * nf(t,nf)) * xa(t,s)) -> sum from t = 2 to T of (sum from nf = 1 to N of (Tf(t,nf) * nf(t,nf) * xa(t,s)))
    for a = 1:A
        for s = 1:S
            aux_Te = 0;
            for t = 1:T
                % Te(t)
                aux_Te_t = 0;
                for nf = 1:N
                    % nf(t,nf)*xa(t,s) -> nfx_a_nf_t_s
                    aux_Te_t = aux_Te_t + Te_t_nf(t,nf)*dv((start_nfx_a_nf_t_s - 1) + (a + (nf - 1)*A + (t - 1)*A*N + (s - 1)*A*N*T));
                end
                % Te(a,s)
                aux_Te = aux_Te + aux_Te_t;
            end
            dv((start_Te_a_s - 1) + (a + (s - 1)*A)) = aux_Te;
        end
    end
 
    %? Tw_a_s
    %* Depends on: t_fin_a_s 
    % While there are still slots to be coordinated
    coordination_remaining_flag = any(S_t_a1_s1_a2_s2(:)) || any(R_t_a1_s1_a2_s2(:));
    last_update_flag = false;
    infinite_loop_flag = false;

    iteration_idx = 0;
    while coordination_remaining_flag || not(last_update_flag)
        if infinite_loop_flag
            disp('Invalid solution: infinite task coordination loop found.');
            result = false;
            break;
        end

        coordination_remaining_flag = any(S_t_a1_s1_a2_s2(:)) || any(R_t_a1_s1_a2_s2(:));
        if not(coordination_remaining_flag)
            last_update_flag = true;
        end
        %% Compute all variables that depends on Tw_a_s directly or indirectly with Tw_a_s' actual value
        dv = updateTwDependentVariables(dv, dv_start_length, Agent, Task, A, T, S, N, R, Te_t_nf);

        if last_update_flag
            break;
        end

        iteration_idx = iteration_idx + 1;
        if nargin > 4 && print_coord_steps_flag
            printSolution(dv, Agent, Task, [], strcat('coordination iteration ', num2str(iteration_idx)));
        end

        %% Search the first slot to be coordinated from the remaining ones
        for s = 1:S
            for a = 1:A
                % If S(:, a, s, :, :) || S(:, :, :, a, s) || R(:, a, s, :, :) || R(:, :, :, a, s) -> (a1, s1) is a slot to be coordinated
                coord_type_ind = 1 * any(any(any(any(any(S_t_a1_s1_a2_s2(:, a, s, :, :)))))) + 10 * any(any(any(any(any(S_t_a1_s1_a2_s2(:, :, :, a, s)))))) + 100 * any(any(any(any(any(R_t_a1_s1_a2_s2(:, a, s, :, :)))))) + 1000 * any(any(any(any(any(R_t_a1_s1_a2_s2(:, :, :, a, s))))));
                if coord_type_ind
                    % Get (a2,s2) to be able to call coordinateTwoSlots for the first time (note: inside coordinateTwoSlots, (a2,s2) is computed again because at the end of the main while loop, (a,s) can change to the old (a2,s2))
                    [a2, s2] = geta2s2(a, s, A, S, coord_type_ind, S_t_a1_s1_a2_s2, R_t_a1_s1_a2_s2);
                    [dv, S_t_a1_s1_a2_s2, R_t_a1_s1_a2_s2, S_t_a1_s1_a2_s2_coordinated, R_t_a1_s1_a2_s2_coordinated, slot_Tw_coordinated, infinite_loop_flag] = coordinateTwoSlots(a, s, a2, s2, dv, S_t_a1_s1_a2_s2, R_t_a1_s1_a2_s2, S_t_a1_s1_a2_s2_coordinated, R_t_a1_s1_a2_s2_coordinated, slot_Tw_coordinated, dv_start_length, Agent, Task, A, T, S, N, R, Te_t_nf, tol);
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

    % Get the objective function value
    fval = f*dv;

    %%? Check if the solution meets the constraints
    result = true;

    % N-hardness
    for t = 1:T
        if t ~= R
            if Task(t).N_hardness
                % na(t) = N(t) -> V(t) = 0
                if not(abs(dv((start_V_t - 1) + t)) <= tol)
                    disp(strcat('N-hard constraint not met for task t = ', num2str(t)));
                    result = false;
                end
            end
        end
    end

    %% Non-decomposable
    for t = 1:T
        if t ~= R
            if Task(t).Relayability == 0 && Task(t).Fragmentability == 0
                % nf(t) = 1
                if not(abs(dv((start_nf_t - 1) + t) - 1) <= tol)
                    disp(strcat('N-hard constraint not met for task t = ', num2str(t)));
                    result = false;
                end
            end
        end
    end

    %* Depends on: na_t, nq_r
    % na(t) <= nq(t), for all t = 2 to T
    for t = 1:T
        if t ~= R
            % na(t) - nq(t) <= 0
            if not(dv((start_na_t - 1) + t) - dv((start_nq_t - 1) + t) <= tol)
                disp(strcat('na(t) <= nq(t) constraint not met for task t = ', num2str(t)));
                result = false;
            end
        end
    end

    %* Depends on: Ft_a_s
    % Flight time constraint
    % Ft_a(s) <= Ft_a - Ft_saf_a, for all a = 1 to A and s = 1 to S
    for a = 1:A
        for s = 1:S
            if not(dv((start_Ft_a_s - 1) + (a + ((s + 1) - 1)*A)) <= (Agent(a).Ft - Agent(a).Ft_saf) + tol)
                disp(strcat('Flight time constraint not met for agent a = ', num2str(a), ' and s = ', num2str(s)));
                result = false;
            end
        end
    end

    %* Depends on: d_tmax_tfin_a_s
    % tmax_a(s) constraint (this is no longer a hard constraint, but a term to minimize in the objective function)
    % tfin_a(s) <= tmax_a(s), for all a = 1 to A and s = 1 to S
    for a = 1:A
        for s = 1:S
            if not(dv((start_d_tmax_tfin_a_s - 1) + (a + (s - 1)*A)) <= tol)
                disp(strcat('t_max_a(s) exceeded for agent a = ', num2str(a), ' and s = ', num2str(s)));
            end
        end
    end

    %* Depends on: x_a_t_s
    % "Recharge tasks shouldn't be assigned to the same Agent in two consecutive slots" constraint
    % xa(t,s) + xa(t,s+1) <= 1, for all a = 1 to A, t = 1 and s = 1 to S-1
    for a = 1:A
        for t = 1:1
            for s = 1:S-1
                if not(dv((start_x_a_t_s - 1) + (a + ((t + 1) - 1)*A + ((s + 1) - 1)*A*(T+1))) + dv((start_x_a_t_s - 1) + (a + ((t + 1) - 1)*A + (((s+1) + 1) - 1)*A*(T+1))) <= 1 + tol)
                    disp(strcat('"Tasks shouldn''t be assigned to the same Agent in two consecutive slots" constraint not met for agent a = ', num2str(a), ', t = ', num2str(t), ' and s = ', num2str(s)));
                    result = false;
                end
            end
        end
    end

    %* Depends on: x_a_t_s
    % Hardware constraint
    % xa(t,s) <= Ha(t), for all a = 1 to A, t = 1 to T and s = 1 to S
    for a = 1:A
        for t = 1:T
            for s = 1:S
                if not(dv((start_x_a_t_s - 1) + (a + ((t + 1) - 1)*A + ((s + 1) - 1)*A*(T+1))) <= H_a_t(a,t) + tol)
                    disp(strcat('Hardware constraint not met for agent a = ', num2str(a), ', t = ', num2str(t), ' and s = ', num2str(s)));
                    result = false;
                end
            end
        end
    end

    %* Depends on: x_a_t_s
    % "Max of 1 task per slot" constraint
    % sum from t = 0 to T of (xa(t,s)) <= 1, for all a = 1 to A and s = 1 to S
    for a = 1:A
        for s = 1:S
            if not(sum(dv((start_x_a_t_s - 1) + (a + ((0 + 1) - 1)*A + ((s + 1) - 1)*A*(T+1)):A:(start_x_a_t_s - 1) + (a + ((T + 1) - 1)*A + ((s + 1) - 1)*A*(T+1)))) <= 1 + tol)
                disp(strcat('"Max of 1 task per slot" constraint not met for agent a = ', num2str(a), ' and s = ', num2str(s)));
                result = false;
            end
        end
    end

    %* Depends on: x_a_t_s
    % Continuity constraint: make it impossible for there to be empty slots in between. But there can be empty slots at the end.
    % sum from t = 1 to T of (xa(t,s)) <= sum from t = 1 to T of (xa(t,s-1)). for all s = 2 to S and a = 1 to A
    for a = 1:A
        for s = 2:S
            if not(sum(dv((start_x_a_t_s - 1) + (a + ((1 + 1) - 1)*A + ((s + 1) - 1)*A*(T+1)):A:(start_x_a_t_s - 1) + (a + ((T + 1) - 1)*A + ((s + 1) - 1)*A*(T+1)))) <= sum(dv((start_x_a_t_s - 1) + (a + ((1 + 1) - 1)*A + (((s-1) + 1) - 1)*A*(T+1)):A:(start_x_a_t_s - 1) + (a + ((T + 1) - 1)*A + (((s-1) + 1) - 1)*A*(T+1)))) + tol)
                disp(strcat('Continuity constraint not met for agent a = ', num2str(a), ' and s = ', num2str(s)));
                result = false;
            end
        end
    end

    %% Synchronization constraints
    % "Synchronized tasks must be the same" constraint
    % S(t,a1,s1,a2,s2) < x(a1,t,s1), for all a1,a2 in A, s1,s2 in S and t in [2,T]
    % S(t,a1,s1,a2,s2) < x(a2,t,s2), for all a1,a2 in A, s1,s2 in S and t in [2,T] %? Intuition says me that one of them is enough
    for t = 1:T
        if t ~= R
            for a1 = 1:A
                for s1 = 1:S
                    for a2 = 1:A
                        for s2 = 1:S
                            if not(dv((start_S_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A)) <= dv((start_x_a_t_s - 1) + (a1 + ((t + 1) - 1)*A + ((s1 + 1) - 1)*A*(T+1))) + tol && dv((start_S_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A)) <= dv((start_x_a_t_s - 1) + (a2 + ((t + 1) - 1)*A + ((s2 + 1) - 1)*A*(T+1))) + tol)
                                disp(strcat('Synchronized tasks are not the same for t = ', num2str(t), ', agent a1 = ', num2str(a1), ', s1 = ', num2str(s1), ', a2 = ', num2str(a2), ' and s2 = ', num2str(s2)));
                                result = false;
                            end
                        end
                    end
                end
            end
        end
    end

    % All synchronizations must be bidirectional
    % S(t, a1, s1, a2, s2) = S(t, a2, s2, a1, s1)
    for t = 1:T
        if t ~= R
            for a1 = 1:A
                for s1 = 1:S
                    for a2 = 1:A
                        for s2 = 1:S
                            if not(abs(dv((start_S_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A)) - dv((start_S_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a2 - 1)*(T-1) + (s2 - 1)*(T-1)*A + (a1 - 1)*(T-1)*A*S + (s1 - 1)*(T-1)*A*S*A))) < tol)
                                disp(strcat('Tasks in slots (', num2str(a1), ', ', num2str(s1), ') and (', num2str(a1), ', ', num2str(s1), ') are not bidirectionally synchronized'));
                                result = false;
                            end
                        end
                    end
                end
            end
        end
    end


    % Number of Synchronizations
    % ns(t,a1,s1) = sum from a2 = 1 to A of (sum from s2 = 1 to S of (S(t,a1,s1,a2,s2))), for all t = 2 to T, a1 in A, s1 in S, a1 != a2
    % ns(t,a1,s1) = (na(t) - 1) * x(a1,t,s1) = na(t) * x(a1,t,s1) - x(a1,t,s1)
    for t = 1:T
        if t ~= R
            for a1 = 1:A
                for s1 = 1:S
                    S_t = 0;
                    for a2 = 1:A
                        if a1 ~= a2
                            for s2 = 1:S
                                S_t = S_t + dv((start_S_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A));
                            end
                        end
                    end
                    if not(S_t <= (dv((start_na_t - 1) + t) - 1) * dv((start_x_a_t_s - 1) + (a + ((t + 1) - 1)*A + ((s + 1) - 1)*A*(T+1))) + tol && (dv((start_na_t - 1) + t) - 1) * dv((start_x_a_t_s - 1) + (a + ((t + 1) - 1)*A + ((s + 1) - 1)*A*(T+1))) - tol <= S_t)
                        disp(strcat('Incorrect number of synchronizations for t = ', num2str(t)));
                        result = false;
                    end
                end
            end
        end
    end

    % Synchronizations time coordination
    % tfin(a1,s1)*S(t,a1,s1,a2,s2) == tfin(a2,s2)*S(t,a1,s1,a2,s2), for all t in [2,T], a1,a2 in A, s1,s2 in S, a1 != a2
    %* Depends on: tfinS_t_a1_s1_a2_s2
    for t = 1:T
        if t ~= R
            for a1 = 1:A-1
                for s1 = 1:S
                    for a2 = a1+1:A
                        for s2 = 1:S
                            if abs(dv((start_tfinS_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A)) - dv((start_tfinS_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a2 - 1)*(T-1) + (s2 - 1)*(T-1)*A + (a1 - 1)*(T-1)*A*S + (s1 - 1)*(T-1)*A*S*A))) > tol
                                disp(strcat('Synchronization time coordination is not correct for t = ', num2str(t), ', agent a1 = ', num2str(a1), ', s1 = ', num2str(s1), ', a2 = ', num2str(a2), ' and s2 = ', num2str(s2)));
                                result = false;
                            end
                        end
                    end
                end
            end
        end
    end

    %% Relays constraints
    % "Relayed tasks must be the same" constraint
    % R(t,a1,s1,a2,s2) < x(a1,t,s1), for all a1,a2 in A, s1,s2 in S and t in [2,T]
    % R(t,a1,s1,a2,s2) < x(a2,t,s2), for all a1,a2 in A, s1,s2 in S and t in [2,T]
    for t = 1:T
        if t ~= R
            for a1 = 1:A
                for s1 = 1:S
                    for a2 = 1:A
                        for s2 = 1:S
                            if not (dv((start_R_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A)) <= dv((start_x_a_t_s - 1) + (a1 + ((t + 1) - 1)*A + ((s1 + 1) - 1)*A*(T+1))) + tol && dv((start_R_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A)) <= dv((start_x_a_t_s - 1) + (a2 + ((t + 1) - 1)*A + ((s2 + 1) - 1)*A*(T+1))) + tol)
                                disp(strcat('Relayed tasks are not the same for t = ', num2str(t), ', agent a1 = ', num2str(a1), ', s1 = ', num2str(s1), ', a2 = ', num2str(a2), ' and s2 = ', num2str(s2)));
                                result = false;
                            end
                        end
                    end
                end
            end
        end
    end

    % "Each task can only be relayed once" constraint
    % sum from a2 = 1 to A of (sum from s2 = 1 to S of (R(t,a1,s1,a2,s2))) <= 1, for all a1 in A, s1 in S and t in [2,T]
    for t = 1:T
        if t ~= R
            for a1 = 1:A
                for s1 = 1:S
                    R_t_a1_s1 = 0;
                    for a2 = 1:A
                        for s2 = 1:S
                            R_t_a1_s1 = R_t_a1_s1 + dv((start_R_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A));
                        end
                    end
                    if not (R_t_a1_s1 <= 1)
                        disp(strcat('"Each task can only be relayed once" constraint not met for t = ', num2str(t), ', agent a1 = ', num2str(a1), ' and s1 = ', num2str(s1)));
                        result = false;
                    end
                end
            end
        end
    end

    % "Each task can only rely one task" constraint
    % sum from a1 = 1 to A of (sum from s1 = 1 to S of (R(t,a1,s1,a2,s2))) <= 1, for all a2 in A, s2 in S and t in [2,T]
    for t = 1:T
        if t ~= R
            for a2 = 1:A
                for s2 = 1:S
                    R_t_a2_s2 = 0;
                    for a1 = 1:A
                        for s1 = 1:S
                            R_t_a2_s2 = R_t_a2_s2 + dv((start_R_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A));
                        end
                    end
                    if not (R_t_a2_s2 <= 1)
                        disp(strcat('"Each task can only rely one task" constraint not met for t = ', num2str(t), ', agent a2 = ', num2str(a2), ' and s2 = ', num2str(s2)));
                        result = false;
                    end
                end
            end
        end
    end

    % Number of relays
    % nr(t) = sum from a1 = 1 to A of (sum from s1 = 1 to S of (sum from a2 = 1 to A of (sum from s2 = 1 to S of (R(t,a1,s1,a2,s2))))), for all t = 2 to T
    % nr(t) = n(t) - na(t), for all t = 2 to T
    for t = 1:T
        if t ~= R
            if Task(t).Relayability == 1
                R_t = 0;
                for a1 = 1:A
                    for s1 = 1:S
                        for a2 = 1:A
                            for s2 = 1:S
                                if not(a1 == a2 && s1 == s2)
                                    R_t = R_t + dv((start_R_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A));
                                end
                            end
                        end
                    end
                end
                if not (abs(R_t - (dv((start_n_t - 1) + t) - dv((start_na_t - 1) + t))) < tol)
                    disp(strcat('Incorrect number of relays for t = ', num2str(t)));
                    result = false;
                end
            end
        end
    end

    % Relays time coordination
    % tfin(a1,s1)*R(t,a1,s1,a2,s2) == (tfin(a2,s2) - Te(a2,s2))*R(t,a1,s1,a2,s2), for all t in [2,T], a1,a2 in A, s1,s2 in S, not(a1 == a2 && s1 == s2)
    %* Depends on: tfinR_t_a1_s1_a2_s2, TeR_t_a1_s1_a2_s2
    for t = 1:T
        if t ~= R
            for a1 = 1:A
                for s1 = 1:S
                    for a2 = 1:A
                        for s2 = 1:S
                            if not(a1 == a2 && s1 == s2)
                                if abs(dv((start_tfinR_t_a1_s1_a2_s2 - 1) + (1 + ((t - 1) - 1)*2 + (a1 - 1)*2*(T-1) + (s1 - 1)*2*(T-1)*A + (a2 - 1)*2*(T-1)*A*S + (s2 - 1)*2*(T-1)*A*S*A)) - dv((start_tfinR_t_a1_s1_a2_s2 - 1) + (2 + ((t - 1) - 1)*2 + (a1 - 1)*2*(T-1) + (s1 - 1)*2*(T-1)*A + (a2 - 1)*2*(T-1)*A*S + (s2 - 1)*2*(T-1)*A*S*A)) + dv((start_TeR_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A))) > tol
                                    disp(strcat('Relays time coordination constraint not met for t = ', num2str(t), ', agent a1 = ', num2str(a1), ', sensor s1 = ', num2str(s1), ', agent a2 = ', num2str(a2), ' and sensor s2 = ', num2str(s2)));
                                    result = false;
                                end
                            end
                        end
                    end
                end
            end
        end
    end

    %% Check if the computed solution is equal to the input solution
    if length(sol) == length_dv
        disp('Computed dv that aren''t equal to the solution:');
        for i = 1:length_dv
            if  abs(sol(i) - dv(i)) > tol
                [var_name, var_index] = getVarName(i, dv_start_length);
                disp(strcat(var_name, ' - ', num2str(var_index)));
            end
        end
    end

    %% Print solution
    if nargin > 3
        printSolution(dv, Agent, Task, [], execution_id, fval);
    else
        printSolution(dv, Agent, Task, [], [], fval);
    end
end

%% Update Tw dependent variables
function [dv] = updateTwDependentVariables(dv, dv_start_length, Agent, Task, A, T, S, N, R, Te_t_nf)
    % Get all dv start and length information
    [start_z                   length_z                   ...
     start_x_a_t_s             length_x_a_t_s             ...
     start_xx_a_t_t_s          length_xx_a_t_t_s          ...
     start_V_t                 length_V_t                 ...
     start_U_t                 length_U_t                 ...
     start_n_t                 length_n_t                 ...
     start_na_t                length_na_t                ...
     start_nq_t                length_nq_t                ...
     start_nq_a_t              length_nq_a_t              ...
     start_nf_t                length_nf_t                ...
     start_nf_t_nf             length_nf_t_nf             ...
     start_nfx_a_nf_t_s        length_nfx_a_nf_t_s        ...
     start_naf_t_nf            length_naf_t_nf            ...
     start_nax_a_t_s           length_nax_a_t_s           ...
     start_Ft_a_s              length_Ft_a_s              ...
     start_Ftx_a_s1            length_Ftx_a_s1            ...
     start_tfin_a_s            length_tfin_a_s            ...
     start_tfinx_a_t_s         length_tfinx_a_t_s         ...
     start_d_tmax_tfin_a_s     length_d_tmax_tfin_a_s     ...
     start_s_used              length_s_used              ...
     start_Td_a_s              length_Td_a_s              ...
     start_Tw_a_s              length_Tw_a_s              ...
     start_Twx_a_s             length_Twx_a_s             ...
     start_Te_a_s              length_Te_a_s              ...
     start_S_t_a1_s1_a2_s2     length_S_t_a1_s1_a2_s2     ...
     start_tfinS_t_a1_s1_a2_s2 length_tfinS_t_a1_s1_a2_s2 ...
     start_R_t_a1_s1_a2_s2     length_R_t_a1_s1_a2_s2     ...
     start_tfinR_t_a1_s1_a2_s2 length_tfinR_t_a1_s1_a2_s2 ...
     start_TeR_t_a1_s1_a2_s2   length_TeR_t_a1_s1_a2_s2      ] = extractStartLengthInformation(dv_start_length);

    %? tfin_a_s
    %* Depends on: Td_a_s, Tw_a_s, Te_a_s
    % tfin_a(s) value
    % tfin(a,s) = tfin(a,s-1) + Td(a,s) + Tw(a,s) + Te(a,s), for all a = 1 to A and s = 1 to S
    for a = 1:A
        % tfin(a,0) = 0
        dv((start_tfin_a_s - 1) + (a + ((0 + 1) - 1)*A)) = 0;
        for s = 1:S
            % tfin_a(s) = tfin(a,s-1) + Td(a,s) + Tw(a,s) + Te(a,s)
            dv((start_tfin_a_s - 1) + (a + ((s + 1) - 1)*A)) = dv((start_tfin_a_s - 1) + (a + (((s-1) + 1) - 1)*A)) + dv((start_Td_a_s - 1) + (a + (s - 1)*A)) + dv((start_Tw_a_s - 1) + (a + (s - 1)*A)) + dv((start_Te_a_s - 1) + (a + (s - 1)*A));
        end
    end

    %? d_tmax_tfin_a_s
    %* Depends on: tfin_a_s, tfinx_a_t_s
    %? tfinx_a_t_s
    %* Depends on: tfin_a_s, x_a_t_s
    % d_tmax_tfin_a_s >= tfin(a,s) * sum from t = 1 to T of (x(a,t,s)) - tmax(a,s), for all a = 1 to A and s = 1 to S
    % d_tmax_tfin_a_s >= sum from t = 1 to T of (tfin(a,s) * x(a,t,s)) - sum from t = 1 to T of (tmax(t)*x(a,t,s)), for all a = 1 to A and s = 1 to S
    % d_tmax_tfin_a_s >= sum from t = 1 to T of ((tfin(a,s) - tmax(t))*x(a,t,s)), for all a = 1 to A and s = 1 to S
    for a = 1:A
        for s = 1:S
            aux_tfinx_a_t_s = 0;
            for t = 1:T
                % (tfin(a,s) - tmax(t)) * x(a,t,s)
                aux_tfinx_a_t_s = aux_tfinx_a_t_s + (dv((start_tfin_a_s - 1) + (a + ((s + 1) - 1)*A)) - Task(t).tmax) * dv((start_x_a_t_s - 1) + (a + ((t + 1) - 1)*A + ((s + 1) - 1)*A*(T+1)));

                % tfinx_a_t_s = tfin(a,s)*x(a,t,s)
                dv((start_tfinx_a_t_s - 1) + (a + (t - 1)*A + (s - 1)*A*T)) = dv((start_tfin_a_s - 1) + (a + ((s + 1) - 1)*A)) * dv((start_x_a_t_s - 1) + (a + ((t + 1) - 1)*A + ((s + 1) - 1)*A*(T+1)));
            end
            % d_tmax_tfin_a_s = max(aux_tfinx_a_t_s - tmax_a(s), 0)
            dv((start_d_tmax_tfin_a_s - 1) + (a + (s - 1)*A)) = max(aux_tfinx_a_t_s, 0);
        end
    end

    %? z
    %* Depends on: tfinx
    % z = max(tfin(a,S)) -> tfin(a,S) - z <= 0
    for a = 1:A
        % z >= tfin(a,S)
        if dv((start_z - 1) + 1) < dv((start_tfin_a_s - 1) + (a + ((S + 1) - 1)*A))
            dv((start_z - 1) + 1) = dv((start_tfin_a_s - 1) + (a + ((S + 1) - 1)*A));
        end
    end

    %? Ftx_a_s1
    %* Depends on: x_a_t_s, Ft_a_s
    %? Ft_a_s
    %* Depends on: nfx_a_nf_t_s, Ft_a_s, Ftx_a_s1, Td_a_s, Tw_a_s, Te_a_s
    % Flight time value
    % Ft(a,s) = Ft(a,s-1) * (1 - x(a,R,s-1)) + Td(a,s) + (Tw(a,s) + Te(a,s)) * (1 - x(a,R,s)), for all a = 1 to A and s = 1 to S
    for a = 1:A
        % Ft(a,0) = Ft_0(a)
        dv((start_Ft_a_s - 1) + (a + ((0 + 1) - 1)*A)) = Agent(a).Ft_0;
        for s = 1:S
            % Ftx_a_s1 = xa(R,s-1) * Ft_a(s-1)
            dv((start_Ftx_a_s1 - 1) + (a + (s - 1)*A)) = dv((start_x_a_t_s - 1) + (a + ((R + 1) - 1)*A + (((s-1) + 1) - 1)*A*(T+1))) * dv((start_Ft_a_s - 1) + (a + (((s-1) + 1) - 1)*A));

            % Twx_a_s = Tw(a,s) * x(a,R,s)
            dv((start_Twx_a_s - 1) + (a + (s - 1)*A)) = dv((start_Tw_a_s - 1) + (a + (s - 1)*A)) * dv((start_x_a_t_s - 1) + (a + ((R + 1) - 1)*A + ((s + 1) - 1)*A*(T+1)));

             % Ft(a,s) = Ft(a,s-1) * (1 - x(a,R,s-1)) + Td(a,s) + (Tw(a,s) + Te(a,s)) * (1 - x(a,R,s))
            dv((start_Ft_a_s - 1) + (a + ((s + 1) - 1)*A)) = dv((start_Ft_a_s - 1) + (a + (((s-1) + 1) - 1)*A)) * (1 - dv((start_x_a_t_s - 1) + (a + ((R + 1) - 1)*A + (((s-1) + 1) - 1)*A*(T+1)))) + dv((start_Td_a_s - 1) + (a + (s - 1)*A)) + (dv((start_Tw_a_s - 1) + (a + (s - 1)*A)) + dv((start_Te_a_s - 1) + (a + (s - 1)*A))) * (1 - dv((start_x_a_t_s - 1) + (a + ((R + 1) - 1)*A + ((s + 1) - 1)*A*(T+1))));
        end
    end

    %? tfinS_t_a1_s1_a2_s2
    %* Depends on: tfin_a_s, S_t_a1_s1_a2_s2
    for t = 1:T
        if t ~= R
            for a1 = 1:A-1
                for s1 = 1:S
                    for a2 = a1+1:A
                        for s2 = 1:S
                            % tfin(a1,s1)*S(t,a1,s1,a2,s2) -> tfinS(t,a1,s1,a2,s2)
                            dv((start_tfinS_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A)) = dv((start_tfin_a_s - 1) + (a1 + ((s1 + 1) - 1)*A)) * dv((start_S_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A));

                            % tfin(a2,s2)*S(t,a1,s1,a2,s2) -> tfinS(t,a2,s2,a1,s1)
                            dv((start_tfinS_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a2 - 1)*(T-1) + (s2 - 1)*(T-1)*A + (a1 - 1)*(T-1)*A*S + (s1 - 1)*(T-1)*A*S*A)) = dv((start_tfin_a_s - 1) + (a2 + ((s2 + 1) - 1)*A)) * dv((start_S_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A));
                        end
                    end
                end
            end
        end
    end

    %? tfinR_t_a1_s1_a2_s2
    %* Depends on: tfin_a_s, R_t_a1_s1_a2_s2
    %? TeR_t_a1_s1_a2_s2
    %* Depends on: Te_a_s, R_t_a1_s1_a2_s2
    for t = 1:T
        if t ~= R
            for a1 = 1:A
                for s1 = 1:S
                    for a2 = 1:A
                        for s2 = 1:S
                            if not(a1 == a2 && s1 == s2)
                                % tfin(a1,s1)*R(t,a1,s1,a2,s2) -> tfinR(1,t,a1,s1,a2,s2)
                                dv((start_tfinR_t_a1_s1_a2_s2 - 1) + (1 + ((t - 1) - 1)*2 + (a1 - 1)*2*(T-1) + (s1 - 1)*2*(T-1)*A + (a2 - 1)*2*(T-1)*A*S + (s2 - 1)*2*(T-1)*A*S*A)) = dv((start_tfin_a_s - 1) + (a1 + ((s1 + 1) - 1)*A)) * dv((start_R_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A));

                                % tfin(a2,s2)*R(t,a1,s1,a2,s2) -> tfinR(2,t,a1,s1,a2,s2)
                                dv((start_tfinR_t_a1_s1_a2_s2 - 1) + (2 + ((t - 1) - 1)*2 + (a1 - 1)*2*(T-1) + (s1 - 1)*2*(T-1)*A + (a2 - 1)*2*(T-1)*A*S + (s2 - 1)*2*(T-1)*A*S*A)) = dv((start_tfin_a_s - 1) + (a2 + ((s2 + 1) - 1)*A)) * dv((start_R_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A));

                                % Te(a2,s2)*R(t,a1,s1,a2,s2)   -> TeR(t,a1,s1,a2,s2)
                                dv((start_TeR_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A)) = dv((start_Te_a_s - 1) + (a2 + (s2 - 1)*A)) * dv((start_R_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A));
                            end
                        end
                    end
                end
            end
        end
    end
end

%% Get (a2,s2)
function [a2, s2] = geta2s2(a, s, A, S, coord_type_ind, S_t_a1_s1_a2_s2, R_t_a1_s1_a2_s2)
    for s2 = 1:S
        for a2 = 1:A
            switch coord_type_ind
                case {1, 11, 101, 111, 1001, 1011, 1101, 1111}
                    if any(any(S_t_a1_s1_a2_s2(:, a, s, a2, s2)))
                        return;
                    end
                case {10, 110, 1010, 1110}
                    if any(any(S_t_a1_s1_a2_s2(:, a2, s2, a, s)))
                        return;
                    end
                case 100
                    if any(any(R_t_a1_s1_a2_s2(:, a, s, a2, s2)))
                        return;
                    end
                case {1000, 1100}
                    if any(any(R_t_a1_s1_a2_s2(:, a2, s2, a, s)))
                        return;
                    end
                otherwise
                    error('Missing case in switch statement.');
            end
        end
    end
end

%% Coordinate two slots
function [dv, S_t_a1_s1_a2_s2, R_t_a1_s1_a2_s2, S_t_a1_s1_a2_s2_coordinated, R_t_a1_s1_a2_s2_coordinated, slot_Tw_coordinated, infinite_loop_flag] = coordinateTwoSlots(a, s, a2, s2, dv, S_t_a1_s1_a2_s2, R_t_a1_s1_a2_s2, S_t_a1_s1_a2_s2_coordinated, R_t_a1_s1_a2_s2_coordinated, slot_Tw_coordinated, dv_start_length, Agent, Task, A, T, S, N, R, Te_t_nf, tol)
    % Get all dv start and length information
    [start_z                   length_z                   ...
     start_x_a_t_s             length_x_a_t_s             ...
     start_xx_a_t_t_s          length_xx_a_t_t_s          ...
     start_V_t                 length_V_t                 ...
     start_U_t                 length_U_t                 ...
     start_n_t                 length_n_t                 ...
     start_na_t                length_na_t                ...
     start_nq_t                length_nq_t                ...
     start_nq_a_t              length_nq_a_t              ...
     start_nf_t                length_nf_t                ...
     start_nf_t_nf             length_nf_t_nf             ...
     start_nfx_a_nf_t_s        length_nfx_a_nf_t_s        ...
     start_naf_t_nf            length_naf_t_nf            ...
     start_nax_a_t_s           length_nax_a_t_s           ...
     start_Ft_a_s              length_Ft_a_s              ...
     start_Ftx_a_s1            length_Ftx_a_s1            ...
     start_tfin_a_s            length_tfin_a_s            ...
     start_tfinx_a_t_s         length_tfinx_a_t_s         ...
     start_d_tmax_tfin_a_s     length_d_tmax_tfin_a_s     ...
     start_s_used              length_s_used              ...
     start_Td_a_s              length_Td_a_s              ...
     start_Tw_a_s              length_Tw_a_s              ...
     start_Twx_a_s             length_Twx_a_s             ...
     start_Te_a_s              length_Te_a_s              ...
     start_S_t_a1_s1_a2_s2     length_S_t_a1_s1_a2_s2     ...
     start_tfinS_t_a1_s1_a2_s2 length_tfinS_t_a1_s1_a2_s2 ...
     start_R_t_a1_s1_a2_s2     length_R_t_a1_s1_a2_s2     ...
     start_tfinR_t_a1_s1_a2_s2 length_tfinR_t_a1_s1_a2_s2 ...
     start_TeR_t_a1_s1_a2_s2   length_TeR_t_a1_s1_a2_s2      ] = extractStartLengthInformation(dv_start_length);

    a0 = a;
    s0 = s;
    infinite_loop_flag = false;
    coord_break_flag = false;
    while not(coord_break_flag)
        coord_type_ind = 1 * any(any(any(any(any(S_t_a1_s1_a2_s2(:, a, s, :, :)))))) + 10 * any(any(any(any(any(S_t_a1_s1_a2_s2(:, :, :, a, s)))))) + 100 * any(any(any(any(any(R_t_a1_s1_a2_s2(:, a, s, :, :)))))) + 1000 * any(any(any(any(any(R_t_a1_s1_a2_s2(:, :, :, a, s))))));
        % Find out the (a2, s2) that (a,s) must be coordinated with
        [a2, s2] = geta2s2(a, s, A, S, coord_type_ind, S_t_a1_s1_a2_s2, R_t_a1_s1_a2_s2);
        
        % Find the first slot to be coordinated in a2  to check if there is a previous slot to be coordinated in a2.
        if a == a2
            first_slot_coord_a2 = s2;
        else
            for first_slot_coord_a2 = 1:s2
                if any(any(any(any(any(S_t_a1_s1_a2_s2(:, a2, first_slot_coord_a2, :, :)))))) || any(any(any(any(any(S_t_a1_s1_a2_s2(:, :, :, a2, first_slot_coord_a2)))))) || any(any(any(any(any(R_t_a1_s1_a2_s2(:, a2, first_slot_coord_a2, :, :)))))) || any(any(any(any(any(R_t_a1_s1_a2_s2(:, :, :, a2, first_slot_coord_a2))))))
                    break;
                end
            end
        end

        % If (a2, s2) was the first slot to be coordinated in a2
        if first_slot_coord_a2 == s2
            % Check if (a1, s1) and (a2, s2) are already coordinated
            switch coord_type_ind
                case {1, 11, 101, 111, 1001, 1011, 1101, 1111, 10, 110, 1010, 1110}
                    time_to_wait = dv((start_tfin_a_s - 1) + (a + ((s + 1) - 1)*A)) - dv((start_tfin_a_s - 1) + (a2 + ((s2 + 1) - 1)*A));
                case 100
                    time_to_wait = dv((start_tfin_a_s - 1) + (a + ((s + 1) - 1)*A)) - (dv((start_tfin_a_s - 1) + (a2 + ((s2 + 1) - 1)*A)) - dv((start_Te_a_s - 1) + (a2 + (s2 - 1)*A)));
                case {1000, 1100}
                    time_to_wait = dv((start_tfin_a_s - 1) + (a2 + ((s2 + 1) - 1)*A)) - (dv((start_tfin_a_s - 1) + (a + ((s + 1) - 1)*A)) - dv((start_Te_a_s - 1) + (a + (s - 1)*A)));
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
                    case {1, 11, 101, 111, 1001, 1011, 1101, 1111, 10, 110, 1010, 1110, 100}
                        % "a" has to wait
                        aw = a;
                        sw = s;
                    case {1000, 1100}
                        % "a2" has to wait
                        aw = a2;
                        sw = s2;
                    otherwise
                        error('Missing case in switch statement.');
                    end
            else
                switch coord_type_ind
                    case {1, 11, 101, 111, 1001, 1011, 1101, 1111, 10, 110, 1010, 1110, 100}
                        % "a2" has to wait
                        aw = a2;
                        sw = s2;
                    case {1000, 1100}
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
                    if dv((start_x_a_t_s - 1) + (aw + ((R + 1) - 1)*A + ((sr + 1) - 1)*A*(T+1))) > 1 - tol
                        flag_any_recharge = true;
                        break;
                    end
                end
                % If there is not any recharge task before the slot to coordinate in the agent that waits, then the waiting is done in that slot
                if flag_any_recharge
                    % Check if there is any already synchronized or relayed task between the last recharge task and the task to coordinate in the agent that waits
                    for ssw = sr+1:s-1
                        for as = 1:A
                            for ss = 1:S
                                for ts = 2:T
                                    if dv((start_S_t_a1_s1_a2_s2 - 1) + ((ts - 1) + (aw - 1)*(T-1) + (ssw - 1)*(T-1)*A + (as - 1)*(T-1)*A*S + (ss - 1)*(T-1)*A*S*A)) || dv((start_S_t_a1_s1_a2_s2 - 1) + ((ts - 1) + (as - 1)*(T-1) + (ss - 1)*(T-1)*A + (aw - 1)*(T-1)*A*S + (ssw - 1)*(T-1)*A*S*A)) || dv((start_R_t_a1_s1_a2_s2 - 1) + ((ts - 1) + (aw - 1)*(T-1) + (ssw - 1)*(T-1)*A + (as - 1)*(T-1)*A*S + (ss - 1)*(T-1)*A*S*A)) || dv((start_R_t_a1_s1_a2_s2 - 1) + ((ts - 1) + (as - 1)*(T-1) + (ss - 1)*(T-1)*A + (aw - 1)*(T-1)*A*S + (ssw - 1)*(T-1)*A*S*A))
                                        flag_any_coordination = 1;
                                    end
                                end
                            end
                        end
                    end
                    % If there is not any coordination in between, then the waiting is done in the last recharge, else, in the task to coordinate
                end
                if flag_any_recharge && not(flag_any_coordination)
                    dv((start_Tw_a_s - 1) + (aw + (sr - 1)*A)) = Tw;
                else
                    dv((start_Tw_a_s - 1) + (aw + (sw - 1)*A)) = Tw;
                end
            end

            dv = updateTwDependentVariables(dv, dv_start_length, Agent, Task, A, T, S, N, R, Te_t_nf);

            switch coord_type_ind
                case {1, 11, 101, 111, 1001, 1011, 1101, 1111, 10, 110, 1010, 1110}
                    S_t_a1_s1_a2_s2(:, a, s, a2, s2) = 0;
                    S_t_a1_s1_a2_s2(:, a2, s2, a, s) = 0;
                case 100
                    R_t_a1_s1_a2_s2(:, a, s, a2, s2) = 0;
                case {1000, 1100}
                    R_t_a1_s1_a2_s2(:, a2, s2, a, s) = 0;
                otherwise
                    error('Missing case in switch statement.');
            end

            % For every time (aw,sw) has been coordinated before with other slot
             for sb = 1:S
                 for ab = 1:A
                     coord_type_ind_before = 1 * any(any(any(any(any(S_t_a1_s1_a2_s2_coordinated(:, aw, sw, ab, sb)))))) + 10 * any(any(any(any(any(S_t_a1_s1_a2_s2_coordinated(:, ab, sb, aw, sw)))))) + 100 * any(any(any(any(any(R_t_a1_s1_a2_s2_coordinated(:, aw, sw, ab, sb)))))) + 1000 * any(any(any(any(any(R_t_a1_s1_a2_s2_coordinated(:, ab, sb, aw, sw))))));
                     if coord_type_ind_before
                        [dv, S_t_a1_s1_a2_s2, R_t_a1_s1_a2_s2, S_t_a1_s1_a2_s2_coordinated, R_t_a1_s1_a2_s2_coordinated, slot_Tw_coordinated] = coordinateAgain(aw, sw, ab, sb, coord_type_ind_before, dv, S_t_a1_s1_a2_s2, R_t_a1_s1_a2_s2, S_t_a1_s1_a2_s2_coordinated, R_t_a1_s1_a2_s2_coordinated, slot_Tw_coordinated, dv_start_length, Agent, Task, A, T, S, N, R, Te_t_nf, tol);
                     end
                 end
             end
             % For every coordination already done in aw for a slot greater than sw (note: if the waiting time was applied in a previous recharge, we would find this coordination again in the following loop except for the fact that it hasn't been included in the coordinated list yet)
             for sd = sw+1:S
                 for sb = 1:S
                     for ab = 1:A
                         coord_type_ind_before = 1 * any(any(any(any(any(S_t_a1_s1_a2_s2_coordinated(:, aw, sd, ab, sb)))))) + 10 * any(any(any(any(any(S_t_a1_s1_a2_s2_coordinated(:, ab, sb, aw, sd)))))) + 100 * any(any(any(any(any(R_t_a1_s1_a2_s2_coordinated(:, aw, sd, ab, sb)))))) + 1000 * any(any(any(any(any(R_t_a1_s1_a2_s2_coordinated(:, ab, sb, aw, sd))))));
                         if coord_type_ind_before
                            [dv, S_t_a1_s1_a2_s2, R_t_a1_s1_a2_s2, S_t_a1_s1_a2_s2_coordinated, R_t_a1_s1_a2_s2_coordinated, slot_Tw_coordinated] = coordinateAgain(aw, sd, ab, sb, coord_type_ind_before, dv, S_t_a1_s1_a2_s2, R_t_a1_s1_a2_s2, S_t_a1_s1_a2_s2_coordinated, R_t_a1_s1_a2_s2_coordinated, slot_Tw_coordinated, dv_start_length, Agent, Task, A, T, S, N, R, Te_t_nf, tol);
                         end
                     end
                 end
             end

            switch coord_type_ind
                case {1, 11, 101, 111, 1001, 1011, 1101, 1111, 10, 110, 1010, 1110}
                    S_t_a1_s1_a2_s2_coordinated(:, a, s, a2, s2) = 1;
                case 100
                    R_t_a1_s1_a2_s2_coordinated(:, a, s, a2, s2) = 1;
                case {1000, 1100}
                    R_t_a1_s1_a2_s2_coordinated(:, a2, s2, a, s) = 1;
                otherwise
                    error('Missing case in switch statement.');
            end
            % Add the info about where was the Tw applied to the map
            switch coord_type_ind
                case {1, 11, 101, 111, 1001, 1011, 1101, 1111, 10, 110, 1010, 1110, 100}
                    if flag_any_recharge && not(flag_any_coordination)
                        slot_Tw_coordinated(strcat(num2str(a),',',num2str(s),',',num2str(a2),',',num2str(s2))) = [Tw, aw, sr];
                    else
                        slot_Tw_coordinated(strcat(num2str(a),',',num2str(s),',',num2str(a2),',',num2str(s2))) = [Tw, aw, sw];
                    end
                case {1000, 1100}
                    if flag_any_recharge && not(flag_any_coordination)
                        slot_Tw_coordinated(strcat(num2str(a2),',',num2str(s2),',',num2str(a),',',num2str(s))) = [Tw, aw, sr];
                    else
                        slot_Tw_coordinated(strcat(num2str(a2),',',num2str(s2),',',num2str(a),',',num2str(s))) = [Tw, aw, sw];
                    end
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
function [dv, S_t_a1_s1_a2_s2, R_t_a1_s1_a2_s2, S_t_a1_s1_a2_s2_coordinated, R_t_a1_s1_a2_s2_coordinated, slot_Tw_coordinated] = coordinateAgain(aw, sw, ab, sb, coord_type_ind_before, dv, S_t_a1_s1_a2_s2, R_t_a1_s1_a2_s2, S_t_a1_s1_a2_s2_coordinated, R_t_a1_s1_a2_s2_coordinated, slot_Tw_coordinated, dv_start_length, Agent, Task, A, T, S, N, R, Te_t_nf, tol)
    % Get all dv start and length information
    [start_z                   length_z                   ...
     start_x_a_t_s             length_x_a_t_s             ...
     start_xx_a_t_t_s          length_xx_a_t_t_s          ...
     start_V_t                 length_V_t                 ...
     start_U_t                 length_U_t                 ...
     start_n_t                 length_n_t                 ...
     start_na_t                length_na_t                ...
     start_nq_t                length_nq_t                ...
     start_nq_a_t              length_nq_a_t              ...
     start_nf_t                length_nf_t                ...
     start_nf_t_nf             length_nf_t_nf             ...
     start_nfx_a_nf_t_s        length_nfx_a_nf_t_s        ...
     start_naf_t_nf            length_naf_t_nf            ...
     start_nax_a_t_s           length_nax_a_t_s           ...
     start_Ft_a_s              length_Ft_a_s              ...
     start_Ftx_a_s1            length_Ftx_a_s1            ...
     start_tfin_a_s            length_tfin_a_s            ...
     start_tfinx_a_t_s         length_tfinx_a_t_s         ...
     start_d_tmax_tfin_a_s     length_d_tmax_tfin_a_s     ...
     start_s_used              length_s_used              ...
     start_Td_a_s              length_Td_a_s              ...
     start_Tw_a_s              length_Tw_a_s              ...
     start_Twx_a_s             length_Twx_a_s             ...
     start_Te_a_s              length_Te_a_s              ...
     start_S_t_a1_s1_a2_s2     length_S_t_a1_s1_a2_s2     ...
     start_tfinS_t_a1_s1_a2_s2 length_tfinS_t_a1_s1_a2_s2 ...
     start_R_t_a1_s1_a2_s2     length_R_t_a1_s1_a2_s2     ...
     start_tfinR_t_a1_s1_a2_s2 length_tfinR_t_a1_s1_a2_s2 ...
     start_TeR_t_a1_s1_a2_s2   length_TeR_t_a1_s1_a2_s2      ] = extractStartLengthInformation(dv_start_length);

    switch coord_type_ind_before
        case {1, 11, 101, 111, 1001, 1011, 1101, 1111, 10, 110, 1010, 1110}
            time_to_wait = dv((start_tfin_a_s - 1) + (aw + ((sw + 1) - 1)*A)) - dv((start_tfin_a_s - 1) + (ab + ((sb + 1) - 1)*A));
        case 100
            time_to_wait = dv((start_tfin_a_s - 1) + (aw + ((sw + 1) - 1)*A)) - (dv((start_tfin_a_s - 1) + (ab + ((sb + 1) - 1)*A)) - dv((start_Te_a_s - 1) + (ab + (sb - 1)*A)));
        case {1000, 1100}
            time_to_wait = dv((start_tfin_a_s - 1) + (ab + ((sb + 1) - 1)*A)) - (dv((start_tfin_a_s - 1) + (aw + ((sw + 1) - 1)*A)) - dv((start_Te_a_s - 1) + (aw + (sw - 1)*A)));
        otherwise
            error('Missing case in switch statement.');
    end
    if abs(time_to_wait) > tol
        % Add that coordination back to the pending list and Get the slot where the waiting time was applied in the old coordination
        switch coord_type_ind_before
            case {1, 11, 101, 111, 1001, 1011, 1101, 1111, 10, 110, 1010, 1110}
                S_t_a1_s1_a2_s2(:, aw, sw, ab, sb) = 1;
                S_t_a1_s1_a2_s2_coordinated(:, aw, sw, ab, sb) = 0;
                Tw_aw_sw = slot_Tw_coordinated(strcat(num2str(aw),',',num2str(sw),',',num2str(ab),',',num2str(sb)));
            case 100
                R_t_a1_s1_a2_s2(:, aw, sw, ab, sb) = 1;
                R_t_a1_s1_a2_s2_coordinated(:, aw, sw, ab, sb) = 0;
                Tw_aw_sw = slot_Tw_coordinated(strcat(num2str(aw),',',num2str(sw),',',num2str(ab),',',num2str(sb)));
            case {1000, 1100}
                R_t_a1_s1_a2_s2(:, ab, sb, aw, sw) = 1;
                R_t_a1_s1_a2_s2_coordinated(:, ab, sb, aw, sw) = 0;
                Tw_aw_sw = slot_Tw_coordinated(strcat(num2str(ab),',',num2str(sb),',',num2str(aw),',',num2str(sw)));
            otherwise
                error('Missing case in switch statement.');
        end
        Twb = Tw_aw_sw(1);
        awb = Tw_aw_sw(2);
        swb = Tw_aw_sw(3);

        % Make the old Tw applied to that coordination equal to 0
        dv((start_Tw_a_s - 1) + (awb + (swb - 1)*A)) = 0;

        dv = updateTwDependentVariables(dv, dv_start_length, Agent, Task, A, T, S, N, R, Te_t_nf);

        [dv, S_t_a1_s1_a2_s2, R_t_a1_s1_a2_s2, S_t_a1_s1_a2_s2_coordinated, R_t_a1_s1_a2_s2_coordinated, slot_Tw_coordinated, infinite_loop_flag] = coordinateTwoSlots(aw, sw, ab, sb, dv, S_t_a1_s1_a2_s2, R_t_a1_s1_a2_s2, S_t_a1_s1_a2_s2_coordinated, R_t_a1_s1_a2_s2_coordinated, slot_Tw_coordinated, dv_start_length, Agent, Task, A, T, S, N, R, Te_t_nf, tol);

        dv = updateTwDependentVariables(dv, dv_start_length, Agent, Task, A, T, S, N, R, Te_t_nf);
    end
end