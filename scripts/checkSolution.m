%% Function to check if an array solution fits constraints
%! ----------------------------------------------------------
function [result] = checkSolution(sol, Agent, Task)
    % Result is true if the solution is correct, false otherwise
    result = true;

    % Numerical tolerance
    tol = 1e-6;

    % Get start length structure information
    [dv_start_length, length_dv] = getStartLengthInformation(Agent, Task);

    % Check if sol has the correct length
    if length(sol) ~= length_dv
        disp('Incorrect solution input array');
        result = false;
        return;
    end

    % Get all dv start and length information
    [start_z                   length_z                   ...
     start_x_a_t_s             length_x_a_t_s             ...
     start_xx_a_t_t_s          length_xx_a_t_t_s          ...
     start_V_t                 length_V_t                 ...
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
    
    % Get scenario information
    [Agent, Task, A, T, S, N, R, Td_a_t_t, Te_t_nf, H_a_t] = getConstantScenarioValues(Agent, Task);
    [z_max, tmax_m, Tw_max, V_max, d_tmax_max, s_used_max] = getNormalizationWeights(Agent, Task);

    % Update Recharge maximum time
    Task(R).tmax = tmax_m;

    % N-hardness
    for t = 1:T
        if t ~= R
            if Task(t).N_hardness
                % na(t) = N(t) -> V(t) = 0
                if not(abs(sol((start_V_t - 1) + t)) <= tol)
                    disp(['N-hard constraint not met for task t = ', num2str(t)]);
                    result = false;
                    return;
                end
            end
        end
    end

    % Non-decomposable
    for t = 1:T
        if t ~= R
            if Task(t).Relayability == 0 && Task(t).Fragmentability == 0
                % nf(t) = 1
                if not(abs(sol((start_nf_t - 1) + t) - 1) <= tol)
                    disp(['N-hard constraint not met for task t = ', num2str(t)]);
                    result = false;
                    return;
                end
            end
        end
    end

    % na(t) <= nq(t), for all t = 2 to T
    for t = 1:T
        if t ~= R
            % na(t) - nq(t) <= 0
            if not(sol((start_na_t - 1) + t) - sol((start_nq_t - 1) + t) <= tol)
                disp(['na(t) <= nq(t) constraint not met for task t = ', num2str(t)]);
                result = false;
                return;
            end
        end
    end

    % Flight time constraint
    % Ft_a(s) <= Ft_a - Ft_saf_a, for all a = 1 to A and s = 1 to S
    for a = 1:A
        for s = 1:S
            if not(sol((start_Ft_a_s - 1) + (a + ((s + 1) - 1)*A)) <= (Agent(a).Ft - Agent(a).Ft_saf) + tol)
                disp(['Flight time constraint not met for agent a = ', num2str(a), ' and s = ', num2str(s)]);
                result = false;
                return;
            end
        end
    end

    % tmax_a(s) constraint (this is no longer a hard constraint, but a term to minimize in the objective function)
    % tfin_a(s) <= tmax_a(s), for all a = 1 to A and s = 1 to S
    for a = 1:A
        for s = 1:S
            if not(sol((start_d_tmax_tfin_a_s - 1) + (a + (s - 1)*A)) <= tol)
                disp(['t_max_a(s) exceeded for agent a = ', num2str(a), ' and s = ', num2str(s)]);
            end
        end
    end

    % "Recharge tasks shouldn't be assigned to the same Agent in two consecutive slots" constraint
    % xa(t,s) + xa(t,s+1) <= 1, for all a = 1 to A, t = 1 and s = 1 to S-1
    for a = 1:A
        for t = 1:1
            for s = 1:S-1
                if not(sol((start_x_a_t_s - 1) + (a + ((t + 1) - 1)*A + ((s + 1) - 1)*A*(T+1))) + sol((start_x_a_t_s - 1) + (a + ((t + 1) - 1)*A + (((s+1) + 1) - 1)*A*(T+1))) <= 1 + tol)
                    disp(['"Tasks shouldn''t be assigned to the same Agent in two consecutive slots" constraint not met for agent a = ', num2str(a), ', t = ', num2str(t), ' and s = ', num2str(s)]);
                    result = false;
                    return;
                end
            end
        end
    end

    % Hardware constraint
    % xa(t,s) <= Ha(t), for all a = 1 to A, t = 1 to T and s = 1 to S
    for a = 1:A
        for t = 1:T
            for s = 1:S
                if not(sol((start_x_a_t_s - 1) + (a + ((t + 1) - 1)*A + ((s + 1) - 1)*A*(T+1))) <= H_a_t(a,t) + tol)
                    disp(['Hardware constraint not met for agent a = ', num2str(a), ', t = ', num2str(t), ' and s = ', num2str(s)]);
                    result = false;
                    return;
                end
            end
        end
    end

    % "Max of 1 task per slot" constraint
    % sum from t = 0 to T of (xa(t,s)) <= 1, for all a = 1 to A and s = 1 to S
    for a = 1:A
        for s = 1:S
            if not(sum(sol((start_x_a_t_s - 1) + (a + ((0 + 1) - 1)*A + ((s + 1) - 1)*A*(T+1)):A:(start_x_a_t_s - 1) + (a + ((T + 1) - 1)*A + ((s + 1) - 1)*A*(T+1)))) <= 1 + tol)
                disp(['"Max of 1 task per slot" constraint not met for agent a = ', num2str(a), ' and s = ', num2str(s)]);
                result = false;
                return;
            end
        end
    end

    % Continuity constraint: make it impossible for there to be empty slots in between. But there can be empty slots at the end.
    % sum from t = 1 to T of (xa(t,s)) <= sum from t = 1 to T of (xa(t,s-1)). for all s = 2 to S and a = 1 to A
    for a = 1:A
        for s = 2:S
            if not(sum(sol((start_x_a_t_s - 1) + (a + ((1 + 1) - 1)*A + ((s + 1) - 1)*A*(T+1)):A:(start_x_a_t_s - 1) + (a + ((T + 1) - 1)*A + ((s + 1) - 1)*A*(T+1)))) <= sum(sol((start_x_a_t_s - 1) + (a + ((1 + 1) - 1)*A + (((s-1) + 1) - 1)*A*(T+1)):A:(start_x_a_t_s - 1) + (a + ((T + 1) - 1)*A + (((s-1) + 1) - 1)*A*(T+1)))) + tol)
                disp(['Continuity constraint not met for agent a = ', num2str(a), ' and s = ', num2str(s)]);
                result = false;
                return;
            end
        end
    end

    % Synchronization constraints
    % Synchronization constraints could be applied only to tasks that have a specified number of robots
    for t = 1:T
        if t ~= R
            if Task(t).N == 0
                for a1 = 1:A
                    for s1 = 1:S
                        for a2 = 1:A
                            for s2 = 1:S
                                if abs(sol((start_S_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A))) > tol
                                    disp(['Synchronization constraints incorrectly applied to t = ', num2str(t), ', (a1, s1) = (', num2str(a1), ', ', num2str(s1), ') and (a2, s2) = (', num2str(a2), ', ', num2str(s2), ')']);
                                    result = false;
                                    return;
                                end
                            end
                        end
                    end
                end
            end
        end
    end

    % "Synchronized tasks must be the same" constraint
    % S(t,a1,s1,a2,s2) < x(a1,t,s1), for all a1,a2 in A, s1,s2 in S and t in [2,T]
    % S(t,a1,s1,a2,s2) < x(a2,t,s2), for all a1,a2 in A, s1,s2 in S and t in [2,T] %? Intuition says me that one of them is enough
    for t = 1:T
        if t ~= R
            for a1 = 1:A
                for s1 = 1:S
                    for a2 = 1:A
                        for s2 = 1:S
                            if not(sol((start_S_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A)) <= sol((start_x_a_t_s - 1) + (a1 + ((t + 1) - 1)*A + ((s1 + 1) - 1)*A*(T+1))) + tol && sol((start_S_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A)) <= sol((start_x_a_t_s - 1) + (a2 + ((t + 1) - 1)*A + ((s2 + 1) - 1)*A*(T+1))) + tol)
                                disp(['Synchronized tasks are not the same for t = ', num2str(t), ', (a1, s1) = (', num2str(a1), ', ', num2str(s1), ') and (a2, s2) = (', num2str(a2), ', ', num2str(s2), ')']);
                                result = false;
                                return;
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
                            if not(abs(sol((start_S_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A)) - sol((start_S_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a2 - 1)*(T-1) + (s2 - 1)*(T-1)*A + (a1 - 1)*(T-1)*A*S + (s1 - 1)*(T-1)*A*S*A))) < tol)
                                disp(['Tasks in slots (', num2str(a1), ', ', num2str(s1), ') and (', num2str(a1), ', ', num2str(s1), ') are not bidirectionally synchronized']);
                                result = false;
                                return;
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
            if Task(t).N ~= 0
                for a1 = 1:A
                    for s1 = 1:S
                        S_t = 0;
                        for a2 = 1:A
                            if a1 ~= a2
                                for s2 = 1:S
                                    S_t = S_t + sol((start_S_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A));
                                end
                            end
                        end
                        if not(S_t <= (sol((start_na_t - 1) + t) - 1) * sol((start_x_a_t_s - 1) + (a1 + ((t + 1) - 1)*A + ((s1 + 1) - 1)*A*(T+1))) + tol && (sol((start_na_t - 1) + t) - 1) * sol((start_x_a_t_s - 1) + (a1 + ((t + 1) - 1)*A + ((s1 + 1) - 1)*A*(T+1))) - tol <= S_t)
                            disp(['Incorrect number of synchronizations for t = ', num2str(t)]);
                            result = false;
                            return;
                        end
                    end
                end
            end
        end
    end

    % Synchronizations time coordination
    % tfin(a1,s1)*S(t,a1,s1,a2,s2) == tfin(a2,s2)*S(t,a1,s1,a2,s2), for all t in [2,T], a1,a2 in A, s1,s2 in S, a1 != a2
    % tfinS(1,t,a1,s1,a2,s2) == tfinS(2,t,a1,s1,a2,s2)
    for t = 1:T
        if t ~= R
            for a1 = 1:A-1
                for s1 = 1:S
                    for a2 = a1+1:A
                        for s2 = 1:S
                            if abs(sol((start_tfinS_t_a1_s1_a2_s2 - 1) + (1 + ((t - 1) - 1)*2 + (a1 - 1)*2*(T-1) + (s1 - 1)*2*(T-1)*A + (a2 - 1)*2*(T-1)*A*S + (s2 - 1)*2*(T-1)*A*S*A)) - sol((start_tfinS_t_a1_s1_a2_s2 - 1) + (2 + ((t - 1) - 1)*2 + (a1 - 1)*2*(T-1) + (s1 - 1)*2*(T-1)*A + (a2 - 1)*2*(T-1)*A*S + (s2 - 1)*2*(T-1)*A*S*A))) > tol
                                disp(['Synchronization time coordination is not correct for t = ', num2str(t), ', (a1, s1) = (', num2str(a1), ', ', num2str(s1), ') and (a2, s2) = (', num2str(a2), ', ', num2str(s2), ')']);
                                result = false;
                                return;
                            end
                        end
                    end
                end
            end
        end
    end

    % Relays constraints
    % Relay constraints could be applied only to relayable tasks
    for t = 1:T
        if t ~= R
            if Task(t).Relayability == 0
                for a1 = 1:A
                    for s1 = 1:S
                        for a2 = 1:A
                            for s2 = 1:S
                                if abs(sol((start_R_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A))) > tol
                                    disp(['Relay constraint incorrectly applied to task t = ', num2str(t), ', (a1, s1) = (', num2str(a1), ', ', num2str(s1), ') and (a2, s2) = (', num2str(a2), ', ', num2str(s2), ')']);
                                    result = false;
                                    return;
                                end
                            end
                        end
                    end
                end
            end
        end
    end


    % "Relayed tasks must be the same" constraint
    % R(t,a1,s1,a2,s2) < x(a1,t,s1), for all a1,a2 in A, s1,s2 in S and t in [2,T]
    % R(t,a1,s1,a2,s2) < x(a2,t,s2), for all a1,a2 in A, s1,s2 in S and t in [2,T]
    for t = 1:T
        if t ~= R
            for a1 = 1:A
                for s1 = 1:S
                    for a2 = 1:A
                        for s2 = 1:S
                            if not (sol((start_R_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A)) <= sol((start_x_a_t_s - 1) + (a1 + ((t + 1) - 1)*A + ((s1 + 1) - 1)*A*(T+1))) + tol && sol((start_R_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A)) <= sol((start_x_a_t_s - 1) + (a2 + ((t + 1) - 1)*A + ((s2 + 1) - 1)*A*(T+1))) + tol)
                                disp(['Relayed tasks are not the same for t = ', num2str(t), ', (a1, s1) = (', num2str(a1), ', ', num2str(s1), ') and (a2, s2) = (', num2str(a2), ', ', num2str(s2), ')']);
                                result = false;
                                return;
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
                            R_t_a1_s1 = R_t_a1_s1 + sol((start_R_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A));
                        end
                    end
                    if R_t_a1_s1 > 1 + tol
                        disp(['"Each task can only be relayed once" constraint not met for t = ', num2str(t), ', (a1, s1) = (', num2str(a1), ', ', num2str(s1), ')']);
                        result = false;
                        return;
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
                            R_t_a2_s2 = R_t_a2_s2 + sol((start_R_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A));
                        end
                    end
                    if R_t_a2_s2 > 1 + tol
                        disp(['"Each task can only rely one task" constraint not met for t = ', num2str(t), ', (a2, s2) = (', num2str(a2), ', ', num2str(s2), ')']);
                        result = false;
                        return;
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
                                    R_t = R_t + sol((start_R_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A));
                                end
                            end
                        end
                    end
                end
                if not(abs(R_t - (sol((start_n_t - 1) + t) - sol((start_na_t - 1) + t))) < tol)
                    disp(['Incorrect number of relays for t = ', num2str(t)]);
                    result = false;
                    return;
                end
            end
        end
    end

    % Relays time coordination
    % tfin(a1,s1)*R(t,a1,s1,a2,s2) == (tfin(a2,s2) - Te(a2,s2))*R(t,a1,s1,a2,s2), for all t in [2,T], a1,a2 in A, s1,s2 in S, not(a1 == a2 && s1 == s2)
    % tfinR(1,t,a1,s1,a2,s2) == tfinR(2,t,a1,s1,a2,s2)
    for t = 1:T
        if t ~= R
            for a1 = 1:A
                for s1 = 1:S
                    for a2 = 1:A
                        for s2 = 1:S
                            if not(a1 == a2 && s1 == s2)
                                if abs(sol((start_tfinR_t_a1_s1_a2_s2 - 1) + (1 + ((t - 1) - 1)*2 + (a1 - 1)*2*(T-1) + (s1 - 1)*2*(T-1)*A + (a2 - 1)*2*(T-1)*A*S + (s2 - 1)*2*(T-1)*A*S*A)) - sol((start_tfinR_t_a1_s1_a2_s2 - 1) + (2 + ((t - 1) - 1)*2 + (a1 - 1)*2*(T-1) + (s1 - 1)*2*(T-1)*A + (a2 - 1)*2*(T-1)*A*S + (s2 - 1)*2*(T-1)*A*S*A)) + sol((start_TeR_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A))) > tol
                                    disp(['Relays time coordination constraint not met for t = ', num2str(t), ', (a1, s1) = (', num2str(a1), ', ', num2str(s1), ') and (a2, s2) = (', num2str(a2), ', ', num2str(s2), ')']);
                                    result = false;
                                    return;
                                end
                            end
                        end
                    end
                end
            end
        end
    end
end