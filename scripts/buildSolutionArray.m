%% Function to build the decision variable array from a handmade or heuristic solution
%! ----------------------------------------------------------
function [dv, fval, result] = buildSolutionArray(handmade_solution, Agent, Task, objective_function, scenario_id, print_coord_steps_flag)
    % Minimum required inputs: handmade_solution and/or Agent and Task
    % Result is false if the handmade solution is incorrect, true otherwise
    result = true;

    % Objective function (options):
    %   - 1. Minimize the longest queue's execution time: min(max(tfin(a,S))).
    %   - 2. Minimize the total joint flight time: min(sum(tfin(a,S))).
    %   - 3. Same as 1. but without used slots term.
    %   - 4. Combination of 1 and 2. Use parameters alpha and betta to give more importance to one over another.
    %   - 5. TODO: Makespan, total joint flight time, total joint consumed battery time, coalition size deviation, deadline delays.
    if nargin < 4 || isempty(objective_function)
        objective_function = 1;
    end
    alpha = 0.8;
    betta = 1 - alpha;

    if nargin < 5
        scenario_id = '';
    end

    if nargin < 6
        print_coord_steps_flag = false;
    end

    % Numerical tolerance
    tol = 1e-6;

    % Get scenario information
    [Agent, Task, A, T, S, N, R, Td_a_t_t, Te_t_nf, H_a_t] = getConstantScenarioValues(Agent, Task);
    [z_max, tmax_m, Tw_max, V_max, d_tmax_max, s_used_max] = getNormalizationWeights(Agent, Task);

    % Put constant scenario values into a structure
    constant_scenario_values = struct('A', A, 'T', T, 'S', S, 'N', N, 'R', R);

    % Update Recharge maximum time
    Task(R).tmax = tmax_m;

    % Get start length structure information
    [dv_start_length, length_dv] = getStartLengthInformation(Agent, Task);

    %% Get all dv start and length information
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
    
    % Create an array for the decision variables
    dv = zeros(length_dv, 1);

    % Create a flag to know if the input came from the heuristic
    heuristic_flag = false;

    % Put the values of x_a_t_s, nf_t, S_t_a1_s1_a2_s2 and R_t_a1_s1_a2_s2 in their corresponding position in the decision variables array
    % Check if the first input if the whole solution array or just the x_a_t_s part
    if length(handmade_solution) == length_dv
        dv(start_x_a_t_s : start_x_a_t_s + length_x_a_t_s - 1) = handmade_solution(start_x_a_t_s : start_x_a_t_s + length_x_a_t_s - 1);
        dv(start_nf_t : start_nf_t + length_nf_t - 1) = handmade_solution(start_nf_t : start_nf_t + length_nf_t - 1);
        dv(start_S_t_a1_s1_a2_s2 : start_S_t_a1_s1_a2_s2 + length_S_t_a1_s1_a2_s2 - 1) = handmade_solution(start_S_t_a1_s1_a2_s2 : start_S_t_a1_s1_a2_s2 + length_S_t_a1_s1_a2_s2 - 1);
        dv(start_R_t_a1_s1_a2_s2 : start_R_t_a1_s1_a2_s2 + length_R_t_a1_s1_a2_s2 - 1) = handmade_solution(start_R_t_a1_s1_a2_s2 : start_R_t_a1_s1_a2_s2 + length_R_t_a1_s1_a2_s2 - 1);
    elseif numel(handmade_solution) == length_x_a_t_s + length_nf_t + length_S_t_a1_s1_a2_s2 + length_R_t_a1_s1_a2_s2
        prev_end = 0;
        dv(start_x_a_t_s : start_x_a_t_s + length_x_a_t_s - 1) = handmade_solution(prev_end + 1:length_x_a_t_s);
        prev_end = prev_end + length_x_a_t_s; 
        dv(start_nf_t : start_nf_t + length_nf_t - 1) = handmade_solution(prev_end + 1 : prev_end + length_nf_t);
        prev_end = prev_end + length_nf_t;
        dv(start_S_t_a1_s1_a2_s2 : start_S_t_a1_s1_a2_s2 + length_S_t_a1_s1_a2_s2 - 1) = handmade_solution(prev_end + 1 : prev_end + length_S_t_a1_s1_a2_s2);
        prev_end = prev_end + length_S_t_a1_s1_a2_s2;
        dv(start_R_t_a1_s1_a2_s2 : start_R_t_a1_s1_a2_s2 + length_R_t_a1_s1_a2_s2 - 1) = handmade_solution(prev_end + 1 : prev_end + length_R_t_a1_s1_a2_s2);
    elseif isfield(Agent, 'queue') && length(handmade_solution) == length_S_t_a1_s1_a2_s2 + length_R_t_a1_s1_a2_s2
        % Set heuristic flag to true
        heuristic_flag = true;

        % Extract x_a_t_s from Agent.queue
        x_a_t_s = zeros(A, T+1, S+1);
        % Initial tasks
        t = 0;
        s = 0;
        x_a_t_s(:, t + 1, s + 1) = 1;

        % Move robot's queue to x_a_t_s decision variable
        for a = 1:A
            s = 0;
            for task = 1:length(Agent(a).queue) - 1
                t = Agent(a).queue(task + 1);
                s = s + 1;
                x_a_t_s(a, t + 1, s + 1) = 1;
            end
        end
        dv(start_x_a_t_s : start_x_a_t_s + length_x_a_t_s - 1) = reshape(x_a_t_s,1,[]);

        % Extract na_t form Task.na
        dv(start_na_t : start_na_t + length_na_t - 1) = [Task.na];

        % Extract nf_t from Task.nf
        dv(start_nf_t : start_nf_t + length_nf_t - 1) = [Task.nf];

        % Extract tfin_a_s from Agent.tfin
        tfin_a_s = zeros(A, S+1);
        for a = 1:A
            for s = 1:S
                tfin_a_s(a, s + 1) = Agent(a).tfin_s(s + 1);
            end
        end
        dv(start_tfin_a_s : start_tfin_a_s + length_tfin_a_s - 1) = reshape(tfin_a_s,1,[]);

        % Extract Ft_a_s from Agent.tfin
        % Ft_a_s = [Agent(a).Ft_0, zeros(1,S)];
        % for a = 1:A
        %     for s = 1:S
        %         Ft_a_s(a, s + 1) = Agent(a).ac_Ft_s(s + 1);
        %     end
        % end
        % dv(start_Ft_a_s : start_Ft_a_s + length_Ft_a_s - 1) = reshape(Ft_a_s,1,[]);

        % Extract makespan from tfin_a_s
        dv((start_z - 1) + 1) = max(max(tfin_a_s));

        % Extract Td_a_s from Agent.Td_s
        Td = zeros(A,S);
        for a = 1:A
            for s = 1:length(Agent(a).queue) - 1
                Td(a,s) = Agent(a).Td_s(s);
            end
        end
        dv(start_Td_a_s : start_Td_a_s + length_Td_a_s - 1) = reshape(Td,1,[]);

        % Extract Tw_a_s from Agent.Tw_s
        Tw = zeros(A,S);
        for a = 1:A
            for s = 1:length(Agent(a).queue) - 1
                Tw(a,s) = Agent(a).Tw_s(s);
            end
        end
        dv(start_Tw_a_s : start_Tw_a_s + length_Tw_a_s - 1) = reshape(Tw,1,[]);

        % Extract Te_a_s from Agent.Te_s
        Te = zeros(A,S);
        for a = 1:A
            for s = 1:length(Agent(a).queue) - 1
                Te(a,s) = Agent(a).Te_s(s);
            end
        end
        dv(start_Te_a_s : start_Te_a_s + length_Te_a_s - 1) = reshape(Te,1,[]);

        % Extract S_t_a1_s1_a2_s2 and R_t_a1_s1_a2_s2 from handmade_solution
        prev_end = 0;
        dv(start_S_t_a1_s1_a2_s2 : start_S_t_a1_s1_a2_s2 + length_S_t_a1_s1_a2_s2 - 1) = handmade_solution(prev_end + 1 : prev_end + length_S_t_a1_s1_a2_s2);
        prev_end = prev_end + length_S_t_a1_s1_a2_s2;
        dv(start_R_t_a1_s1_a2_s2 : start_R_t_a1_s1_a2_s2 + length_R_t_a1_s1_a2_s2 - 1) = handmade_solution(prev_end + 1 : prev_end + length_R_t_a1_s1_a2_s2);
    else
        error('Incorrect solution input array');
    end

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
            dv((start_nq_a_t - 1) + (a + (t - 1)*A)) = any(dv((start_x_a_t_s - 1) + (a + ((t + 1) - 1)*A + ((1 + 1) - 1)*A*(T+1)) : A*(T+1) : (start_x_a_t_s - 1) + (a + ((t + 1) - 1)*A + ((S + 1) - 1)*A*(T+1))));
        end
    end

    %? nq_t
    %* Depends on: nq_a_t
    % nq(t): number of queues that task t appears in.
    % nq(t) = sum from a = 1 to A of (nq(a,t)), for all t = 1 to T
    for t = 1:T
        aux_nq_t_a = 0;
        for a = 1:A
            aux_nq_t_a = aux_nq_t_a + dv((start_nq_a_t - 1) + (a + (t - 1)*A));
        end
        dv((start_nq_t - 1) + (t)) = aux_nq_t_a;
    end

    if not(heuristic_flag)
        %? na_t
        %* Depends on: n_t, nf_t
        % na(t) = n(t)/nf(t), for all t = 2 to T
        for t = 1:T
            if t ~= R
                dv((start_na_t - 1) + (t)) = dv((start_n_t - 1) + (t)) / dv((start_nf_t - 1) + (t));
            end
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

    %? s_used value
    %* Depends on: x_a_t_s
    % s_used = sum from a = 1 to A of (sum from t = 0 to T of (sum from s = 0 to S of (xa(t,s)))) - A
    dv((start_s_used - 1) + 1) = sum(dv(start_x_a_t_s : start_x_a_t_s + length_x_a_t_s - 1)) - A;

    if not(heuristic_flag)
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
        % Auxiliary copy of S and R to coordinate the slots one by one
        S_t_a1_s1_a2_s2 = reshape(dv(start_S_t_a1_s1_a2_s2 : start_S_t_a1_s1_a2_s2 + length_S_t_a1_s1_a2_s2 - 1), T-1, A, S, A, S);
        R_t_a1_s1_a2_s2 = reshape(dv(start_R_t_a1_s1_a2_s2 : start_R_t_a1_s1_a2_s2 + length_R_t_a1_s1_a2_s2 - 1), T-1, A, S, A, S);

        % Auxiliary variable to store the already coordinated slots
        S_t_a1_s1_a2_s2_coordinated = zeros(T-1, A, S, A, S);
        R_t_a1_s1_a2_s2_coordinated = zeros(T-1, A, S, A, S);

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
            dv = updateTfin(dv, dv_start_length, constant_scenario_values);

            if print_coord_steps_flag && iteration_idx == 0
                printSolution(dv, Agent, Task, 0, scenario_id, ['coordination iteration ', num2str(iteration_idx)]);
            end

            if last_update_flag
                break;
            end

            %% Search the first slot to be coordinated from the remaining ones
            for s = 1:S
                for a = 1:A
                    % If S(:, a, s, :, :) || S(:, :, :, a, s) || R(:, a, s, :, :) || R(:, :, :, a, s) -> (a1, s1) is a slot to be coordinated
                    % Note that, as synchronizations are bidirectional, if any(S(:, a, s, :, :)), there will be also any(S(:, :, :, a, s))
                    % Note that it is possible to have at the same time any(R(:, a, s, :, :)) and any(R(:, :, :, a, s)), but not to the same other slot
                    coord_type_ind = 1 * any(any(any(S_t_a1_s1_a2_s2(:, a, s, :, :)))) + 10 * any(any(any(R_t_a1_s1_a2_s2(:, a, s, :, :)))) + 100 * any(any(any(R_t_a1_s1_a2_s2(:, :, :, a, s))));
                    if coord_type_ind
                        % Get (a2,s2) to be able to call coordinateTwoSlots for the first time (note: inside coordinateTwoSlots, (a2,s2) is computed again because at the end of the main while loop, (a,s) can change to the old (a2,s2))
                        [a2, s2] = geta2s2(a, s, A, S, coord_type_ind, S_t_a1_s1_a2_s2, R_t_a1_s1_a2_s2);
                        [dv, S_t_a1_s1_a2_s2, R_t_a1_s1_a2_s2, S_t_a1_s1_a2_s2_coordinated, R_t_a1_s1_a2_s2_coordinated, infinite_loop_flag, iteration_idx] = coordinateTwoSlots(a, s, a2, s2, dv, S_t_a1_s1_a2_s2, R_t_a1_s1_a2_s2, S_t_a1_s1_a2_s2_coordinated, R_t_a1_s1_a2_s2_coordinated, dv_start_length, constant_scenario_values, tol, iteration_idx, print_coord_steps_flag);
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
    end

    % Update Tw_a_s dependent variables
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

    if not(heuristic_flag)
        %? z
        %* Depends on: tfinx
        % z = max(tfin(a,S)) -> tfin(a,S) - z <= 0
        for a = 1:A
            % z >= tfin(a,S)
            if dv((start_z - 1) + 1) < dv((start_tfin_a_s - 1) + (a + ((S + 1) - 1)*A))
                dv((start_z - 1) + 1) = dv((start_tfin_a_s - 1) + (a + ((S + 1) - 1)*A));
            end
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
            % Synchronization constraints could be applied only to tasks that have a specified number of robots
            if Task(t).N ~= 0
                for a1 = 1:A
                    for s1 = 1:S
                        for a2 = 1:A
                            if a1 ~= a2
                                for s2 = 1:S
                                    % tfin(a1,s1)*S(t,a1,s1,a2,s2) -> tfinS(1,t,a1,s1,a2,s2)
                                    %! Next is the (9) most time consuming line in this code right now
                                    dv((start_tfinS_t_a1_s1_a2_s2 - 1) + (1 + ((t - 1) - 1)*2 + (a1 - 1)*2*(T-1) + (s1 - 1)*2*(T-1)*A + (a2 - 1)*2*(T-1)*A*S + (s2 - 1)*2*(T-1)*A*S*A)) = dv((start_tfin_a_s - 1) + (a1 + ((s1 + 1) - 1)*A)) * dv((start_S_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A));

                                    % tfin(a2,s2)*S(t,a1,s1,a2,s2) -> tfinS(2,t,a1,s1,a2,s2)
                                    dv((start_tfinS_t_a1_s1_a2_s2 - 1) + (2 + ((t - 1) - 1)*2 + (a1 - 1)*2*(T-1) + (s1 - 1)*2*(T-1)*A + (a2 - 1)*2*(T-1)*A*S + (s2 - 1)*2*(T-1)*A*S*A)) = dv((start_tfin_a_s - 1) + (a2 + ((s2 + 1) - 1)*A)) * dv((start_S_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A));
                                end
                            end
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
            % Relay constraints could be applied only to relayable tasks
            if Task(t).Relayability == 1
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

    % Objective function coefficients
    f = zeros(1,length_dv);
    switch objective_function
    case 2
        % Minimize the total joint flight time: min(sum(tfin(a,S))), for all a = 1 to A.
        % Note: no need to minimize Tw as its included in the total joint flight time.
        f((start_tfin_a_s - 1) + (1 + ((S + 1) - 1)*A):(start_tfin_a_s - 1) + (A + ((S + 1) - 1)*A)) = 1/tmax_m;
    case 3
        % Minimize the longest queue's execution time: min(max(tfin(a,S))) -> min(z).
        f((start_z - 1) + 1) = 1/z_max;
        
        % Tw: Coefficients of the term minimizing the waiting time to avoid unnecessary waitings.
        f((start_Tw_a_s - 1) + 1 : (start_Tw_a_s - 1) + length_Tw_a_s) = 1/Tw_max;
    case 4
        % Minimize the longest queue's execution time: min(max(tfin(a,S))) -> min(z).
        f((start_z - 1) + 1) = alpha/z_max;

        % Minimize the total joint flight time: min(sum(tfin(a,S))), for all a = 1 to A.
        f((start_tfin_a_s - 1) + (1 + ((S + 1) - 1)*A):(start_tfin_a_s - 1) + (A + ((S + 1) - 1)*A)) = betta/tmax_m;
        
        % Tw: Coefficients of the term minimizing the waiting time to avoid unnecessary waitings.
        % Note: here Tw penalises twice, first in the total joint flight time term, then in the Tw term
        f((start_Tw_a_s - 1) + 1 : (start_Tw_a_s - 1) + length_Tw_a_s) = 1/Tw_max;
    case 5
        % Minimize the longest queue's execution time: min(max(tfin(a,S))) -> min(z).
        f((start_z - 1) + 1) = 1/z_max;

        % Minimize the total joint flight time: min(sum(tfin(a,S))), for all a = 1 to A.
        f((start_tfin_a_s - 1) + (1 + ((S + 1) - 1)*A):(start_tfin_a_s - 1) + (A + ((S + 1) - 1)*A)) = 1/tmax_m;

        % Minimize the total joint consumed battery time
        % f() = 1/Tw_max;
    otherwise
        % Minimize the longest queue's execution time: min(max(tfin(a,S))) -> min(z).
        f((start_z - 1) + 1) = 1/z_max;
        
        % Tw: Coefficients of the term minimizing the waiting time to avoid unnecessary waitings.
        f((start_Tw_a_s - 1) + 1 : (start_Tw_a_s - 1) + length_Tw_a_s) = 1/Tw_max;

        % s_used: Coefficients of the term minimizing the number of used slots.
        f((start_s_used - 1) + 1 : (start_s_used - 1) + length_s_used) = 1/s_used_max;
    end

    % V: Coefficients of the term penalizing the use of a different number of Agents. V(2 to T) to exclude Recharge task.
    f((start_V_t - 1) + 2 : (start_V_t - 1) + length_V_t) = 1/V_max;

    % d_tmax_tfin_a_s: Coefficients of the term penalizing exceeding the tmax of a task.
    f((start_d_tmax_tfin_a_s - 1) + 1 : (start_d_tmax_tfin_a_s - 1) + length_d_tmax_tfin_a_s) = 1/d_tmax_max;

    % Get the objective function value
    fval = f*dv;
end

%% Update tfin_a_s
function [dv] = updateTfin(dv, dv_start_length, constant_scenario_values)
    A = constant_scenario_values.A;
    T = constant_scenario_values.T;
    S = constant_scenario_values.S;
    N = constant_scenario_values.N;
    R = constant_scenario_values.R;
    
    % Get needed dv start and length information
    [~,              ~, ...
     ~,              ~, ...
     ~,              ~, ...
     ~,              ~, ...
     ~,              ~, ...
     ~,              ~, ...
     ~,              ~, ...
     ~,              ~, ...
     ~,              ~, ...
     ~,              ~, ...
     ~,              ~, ...
     ~,              ~, ...
     ~,              ~, ...
     ~,              ~, ...
     ~,              ~, ...
     start_tfin_a_s, ~, ...
     ~,              ~, ...
     ~,              ~, ...
     ~,              ~, ...
     start_Td_a_s,   ~, ...
     start_Tw_a_s,   ~, ...
     ~,              ~, ...
     start_Te_a_s,   ~, ...
     ~,              ~, ...
     ~,              ~, ...
     ~,              ~, ...
     ~,              ~, ...
     ~,              ~] = extractStartLengthInformation(dv_start_length);

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
end

%% Get (a2,s2)
function [a2, s2] = geta2s2(a, s, A, S, coord_type_ind, S_t_a1_s1_a2_s2, R_t_a1_s1_a2_s2)
    for s2 = 1:S
        for a2 = 1:A
            switch coord_type_ind
            case {1, 11, 101, 111}
                if any(S_t_a1_s1_a2_s2(:, a, s, a2, s2))
                    return;
                end
            case 10
                if any(R_t_a1_s1_a2_s2(:, a, s, a2, s2))
                    return;
                end
            case {100, 110}
                if any(R_t_a1_s1_a2_s2(:, a2, s2, a, s))
                    return;
                end
            otherwise
                error('Missing case in switch statement.');
            end
        end
    end
end

%% Coordinate two slots
function [dv, S_t_a1_s1_a2_s2, R_t_a1_s1_a2_s2, S_t_a1_s1_a2_s2_coordinated, R_t_a1_s1_a2_s2_coordinated, infinite_loop_flag, iteration_idx] = coordinateTwoSlots(a, s, a2, s2, dv, S_t_a1_s1_a2_s2, R_t_a1_s1_a2_s2, S_t_a1_s1_a2_s2_coordinated, R_t_a1_s1_a2_s2_coordinated, dv_start_length, constant_scenario_values, tol, iteration_idx, print_coord_steps_flag)
    A = constant_scenario_values.A;
    T = constant_scenario_values.T;
    S = constant_scenario_values.S;
    N = constant_scenario_values.N;
    R = constant_scenario_values.R;

    % Get all dv start and length information
    [~,                     ~, ...
     start_x_a_t_s,         ~, ...
     ~,                     ~, ...
     ~,                     ~, ...
     ~,                     ~, ...
     ~,                     ~, ...
     ~,                     ~, ...
     ~,                     ~, ...
     ~,                     ~, ...
     ~,                     ~, ...
     ~,                     ~, ...
     ~,                     ~, ...
     ~,                     ~, ...
     ~,                     ~, ...
     ~,                     ~, ...
     start_tfin_a_s,        ~, ...
     ~,                     ~, ...
     ~,                     ~, ...
     ~,                     ~, ...
     ~,                     ~, ...
     start_Tw_a_s,          ~, ...
     ~,                     ~, ...
     start_Te_a_s,          ~, ...
     start_S_t_a1_s1_a2_s2, ~, ...
     ~,                     ~, ...
     start_R_t_a1_s1_a2_s2, ~, ...
     ~,                     ~, ...
     ~,                     ~] = extractStartLengthInformation(dv_start_length);

    a0 = a;
    s0 = s;
    infinite_loop_flag = false;
    coord_break_flag = false;
    while not(coord_break_flag)
        % Note that, as synchronizations are bidirectional, if any(S(:, a, s, :, :)), there will be also any(S(:, :, :, a, s))
        % Note that it is possible to have at the same time any(R(:, a, s, :, :)) and any(R(:, :, :, a, s)), but not to the same other slot
        coord_type_ind = 1 * any(any(any(S_t_a1_s1_a2_s2(:, a, s, :, :)))) + 10 * any(any(any(R_t_a1_s1_a2_s2(:, a, s, :, :)))) + 100 * any(any(any(R_t_a1_s1_a2_s2(:, :, :, a, s))));
        % Find out the (a2, s2) that (a,s) must be coordinated with
        [a2, s2] = geta2s2(a, s, A, S, coord_type_ind, S_t_a1_s1_a2_s2, R_t_a1_s1_a2_s2);
        
        % Find the first slot to be coordinated in a2 to check if there is a previous slot to be coordinated in a2.
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
            case {1, 11, 101, 111}
                time_to_wait = dv((start_tfin_a_s - 1) + (a + ((s + 1) - 1)*A)) - dv((start_tfin_a_s - 1) + (a2 + ((s2 + 1) - 1)*A));
            case 10
                time_to_wait = dv((start_tfin_a_s - 1) + (a + ((s + 1) - 1)*A)) - (dv((start_tfin_a_s - 1) + (a2 + ((s2 + 1) - 1)*A)) - dv((start_Te_a_s - 1) + (a2 + (s2 - 1)*A)));
            case {100, 110}
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
                    if dv((start_x_a_t_s - 1) + (aw + ((R + 1) - 1)*A + ((sr + 1) - 1)*A*(T+1))) > 1 - tol
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

                % Update waiting time
                if flag_any_recharge && not(flag_any_coordination)
                    dv((start_Tw_a_s - 1) + (aw + (sr - 1)*A)) = dv((start_Tw_a_s - 1) + (aw + (sr - 1)*A)) + Tw;
                else
                    dv((start_Tw_a_s - 1) + (aw + (sw - 1)*A)) = dv((start_Tw_a_s - 1) + (aw + (sw - 1)*A)) + Tw;
                end
            end

            dv = updateTfin(dv, dv_start_length, constant_scenario_values);

            iteration_idx = iteration_idx + 1;
            if print_coord_steps_flag
                disp(['Coordinated slots (', num2str(a), ',', num2str(s), ') and (', num2str(a2), ',', num2str(s2), ')']);
                printSolution(dv, Agent, Task, 1, '', ['coordination iteration ', num2str(iteration_idx)]);
            end

            % Remove this coordination before checking if the past ones still coordinated because we may need to call again this function
            switch coord_type_ind
            case {1, 11, 101, 111}
                S_t_a1_s1_a2_s2(:, a, s, a2, s2) = 0;
                S_t_a1_s1_a2_s2(:, a2, s2, a, s) = 0;
            case 10
                R_t_a1_s1_a2_s2(:, a, s, a2, s2) = 0;
            case {100, 110}
                R_t_a1_s1_a2_s2(:, a2, s2, a, s) = 0;
            otherwise
                error('Missing case in switch statement.');
            end

            % Check if this slot was already coordinated with a previous one
            % For every time (aw,sw) has been coordinated before with other slot
            for sb = 1:S
                for ab = 1:A
                    % Note that, as synchronizations are bidirectional, if any(S(:, a, s, :, :)), there will be also any(S(:, :, :, a, s))
                    % Note that it is possible to have at the same time any(R(:, a, s, :, :)) and any(R(:, :, :, a, s)), but not to the same other slot
                    coord_type_ind_before = 1 * any(S_t_a1_s1_a2_s2_coordinated(:, aw, sw, ab, sb)) + 10 * any(R_t_a1_s1_a2_s2_coordinated(:, aw, sw, ab, sb)) + 100 * any(R_t_a1_s1_a2_s2_coordinated(:, ab, sb, aw, sw));
                    if coord_type_ind_before
                        [dv, S_t_a1_s1_a2_s2, R_t_a1_s1_a2_s2, S_t_a1_s1_a2_s2_coordinated, R_t_a1_s1_a2_s2_coordinated, infinite_loop_flag, iteration_idx] = coordinateAgain(aw, sw, ab, sb, coord_type_ind_before, dv, S_t_a1_s1_a2_s2, R_t_a1_s1_a2_s2, S_t_a1_s1_a2_s2_coordinated, R_t_a1_s1_a2_s2_coordinated, dv_start_length, constant_scenario_values, tol, iteration_idx, print_coord_steps_flag);
                    end
                end
            end

            % Check if there are any other coordinations already done in a future slot in the agent that waited
            % For every coordination already done in aw for a slot greater than sw (note: if the waiting time was applied in a previous recharge, we would find this coordination again in the following loop except for the fact that it hasn't been included in the coordinated list yet)
            for sd = sw+1:S
                for sb = 1:S
                    for ab = 1:A
                        %! Next is the (1) most time consuming line in this code right now
                        coord_type_ind_before = 1 * any(S_t_a1_s1_a2_s2_coordinated(:, aw, sd, ab, sb)) + 10 * any(R_t_a1_s1_a2_s2_coordinated(:, aw, sd, ab, sb)) + 100 * any(R_t_a1_s1_a2_s2_coordinated(:, ab, sb, aw, sd));
                        if coord_type_ind_before
                            [dv, S_t_a1_s1_a2_s2, R_t_a1_s1_a2_s2, S_t_a1_s1_a2_s2_coordinated, R_t_a1_s1_a2_s2_coordinated, infinite_loop_flag, iteration_idx] = coordinateAgain(aw, sd, ab, sb, coord_type_ind_before, dv, S_t_a1_s1_a2_s2, R_t_a1_s1_a2_s2, S_t_a1_s1_a2_s2_coordinated, R_t_a1_s1_a2_s2_coordinated, dv_start_length, constant_scenario_values, tol, iteration_idx, print_coord_steps_flag);
                        end
                    end
                end
            end

            % After checking and tuning past coordinations, check if the slots of the current coordination still coordinated
            switch coord_type_ind
            case {1, 11, 101, 111}
                tmp_time_to_wait = dv((start_tfin_a_s - 1) + (a + ((s + 1) - 1)*A)) - dv((start_tfin_a_s - 1) + (a2 + ((s2 + 1) - 1)*A));
            case 10
                tmp_time_to_wait = dv((start_tfin_a_s - 1) + (a + ((s + 1) - 1)*A)) - (dv((start_tfin_a_s - 1) + (a2 + ((s2 + 1) - 1)*A)) - dv((start_Te_a_s - 1) + (a2 + (s2 - 1)*A)));
            case {100, 110}
                tmp_time_to_wait = dv((start_tfin_a_s - 1) + (a2 + ((s2 + 1) - 1)*A)) - (dv((start_tfin_a_s - 1) + (a + ((s + 1) - 1)*A)) - dv((start_Te_a_s - 1) + (a + (s - 1)*A)));
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
                S_t_a1_s1_a2_s2_coordinated(:, a, s, a2, s2) = 1;
                S_t_a1_s1_a2_s2_coordinated(:, a2, s2, a, s) = 1;
            case 10
                R_t_a1_s1_a2_s2_coordinated(:, a, s, a2, s2) = 1;
            case {100, 110}
                R_t_a1_s1_a2_s2_coordinated(:, a2, s2, a, s) = 1;
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
function [dv, S_t_a1_s1_a2_s2, R_t_a1_s1_a2_s2, S_t_a1_s1_a2_s2_coordinated, R_t_a1_s1_a2_s2_coordinated, infinite_loop_flag, iteration_idx] = coordinateAgain(aw, sw, ab, sb, coord_type_ind_before, dv, S_t_a1_s1_a2_s2, R_t_a1_s1_a2_s2, S_t_a1_s1_a2_s2_coordinated, R_t_a1_s1_a2_s2_coordinated, dv_start_length, constant_scenario_values, tol, iteration_idx, print_coord_steps_flag)
    A = constant_scenario_values.A;
    T = constant_scenario_values.T;
    S = constant_scenario_values.S;
    N = constant_scenario_values.N;
    R = constant_scenario_values.R;
    
    % Get all dv start and length information
    [~,              ~, ...
     ~,              ~, ...
     ~,              ~, ...
     ~,              ~, ...
     ~,              ~, ...
     ~,              ~, ...
     ~,              ~, ...
     ~,              ~, ...
     ~,              ~, ...
     ~,              ~, ...
     ~,              ~, ...
     ~,              ~, ...
     ~,              ~, ...
     ~,              ~, ...
     ~,              ~, ...
     start_tfin_a_s, ~, ...
     ~,              ~, ...
     ~,              ~, ...
     ~,              ~, ...
     ~,              ~, ...
     ~,              ~, ...
     ~,              ~, ...
     start_Te_a_s,   ~, ...
     ~,              ~, ...
     ~,              ~, ...
     ~,              ~, ...
     ~,              ~, ...
     ~,              ~] = extractStartLengthInformation(dv_start_length);

    infinite_loop_flag = false;

    switch coord_type_ind_before
    case {1, 11, 101, 111}
        time_to_wait = dv((start_tfin_a_s - 1) + (aw + ((sw + 1) - 1)*A)) - dv((start_tfin_a_s - 1) + (ab + ((sb + 1) - 1)*A));
    case 10
        time_to_wait = dv((start_tfin_a_s - 1) + (aw + ((sw + 1) - 1)*A)) - (dv((start_tfin_a_s - 1) + (ab + ((sb + 1) - 1)*A)) - dv((start_Te_a_s - 1) + (ab + (sb - 1)*A)));
    case {100, 110}
        time_to_wait = dv((start_tfin_a_s - 1) + (ab + ((sb + 1) - 1)*A)) - (dv((start_tfin_a_s - 1) + (aw + ((sw + 1) - 1)*A)) - dv((start_Te_a_s - 1) + (aw + (sw - 1)*A)));
    otherwise
        error('Missing case in switch statement.');
    end

    if abs(time_to_wait) > tol
        % Add that coordination back to the pending list and get the slot where the waiting time was applied in the old coordination
        switch coord_type_ind_before
        case {1, 11, 101, 111}
            S_t_a1_s1_a2_s2(:, aw, sw, ab, sb) = 1;
            S_t_a1_s1_a2_s2(:, ab, sb, aw, sw) = 1;
            S_t_a1_s1_a2_s2_coordinated(:, aw, sw, ab, sb) = 0;
            S_t_a1_s1_a2_s2_coordinated(:, ab, sb, aw, sw) = 0;
        case 10
            R_t_a1_s1_a2_s2(:, aw, sw, ab, sb) = 1;
            R_t_a1_s1_a2_s2_coordinated(:, aw, sw, ab, sb) = 0;
        case {100, 110}
            R_t_a1_s1_a2_s2(:, ab, sb, aw, sw) = 1;
            R_t_a1_s1_a2_s2_coordinated(:, ab, sb, aw, sw) = 0;
        otherwise
            error('Missing case in switch statement.');
        end

        dv = updateTfin(dv, dv_start_length, constant_scenario_values);

        [dv, S_t_a1_s1_a2_s2, R_t_a1_s1_a2_s2, S_t_a1_s1_a2_s2_coordinated, R_t_a1_s1_a2_s2_coordinated, infinite_loop_flag, iteration_idx] = coordinateTwoSlots(aw, sw, ab, sb, dv, S_t_a1_s1_a2_s2, R_t_a1_s1_a2_s2, S_t_a1_s1_a2_s2_coordinated, R_t_a1_s1_a2_s2_coordinated, dv_start_length, constant_scenario_values, tol, iteration_idx, print_coord_steps_flag);

        dv = updateTfin(dv, dv_start_length, constant_scenario_values);
    end
end