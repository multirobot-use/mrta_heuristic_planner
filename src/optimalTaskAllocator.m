function [sol, fval, population, scores] = optimalTaskAllocator(scenario_id, execution_id, scenario_size, objective_function, formulation_variants_flags, config_flags, solver_config)
    % Minimum required input arguments: scenario_id, execution_id

    % scenario_id: numeric if predefined scenario, 0 if random, not numeric if saved scenario (saved scenario ID name)
    % execution_id: used in log file and as file sufix to save data in case we want to save the results
    % scenario size: 1x3 vector with the number of robots (A), number of tasks (T) (without counting the recharge task) and number of different types of robots (types). Used for random generated scenarios (scenario_id == 0)
    % objective_function: integer parameter to choose between different predefined objective functions.
    % formulation_variants_flags: 1x4 logic vector (empty to use default config: complete formulation): [recharges_allowed_flag, relays_allowed_flag, fragmentation_allowed_flag, variable_number_of_robots_allowed_flag]
    % config_flags: 1x9 logic vector (empty to use default config): [solve_flag, initialize_flag, genetic_algorithm_solver, print_solution_flag, display_flag, log_file_flag, save_flag, test_flag, recovery_flag]
    % solver_config: integer to choose between different solver configurations (empty or 0 to use default)

    last_toc = toc;
    
    if isnumeric(scenario_id)
        predefined = scenario_id;
        old_executed_random_scenario_id = '';
    else
        predefined = 0;
        old_executed_random_scenario_id = scenario_id;
    end

    % Objective function (options):
    %   - 1. Minimize the longest queue's execution time: min(max(tfin(a,S))).
    %   - 2. Minimize the total joint flight time: min(sum(tfin(a,S))).
    %   - 3. Same as 1. but without used slots term.
    %   - 4. Combination of 1 and 2. Use parameters alpha and betta to give more importance to one over another.
    if nargin < 4 || isempty(objective_function)
        objective_function = 1;
    end
    alpha = 0.8;
    betta = 1 - alpha;

    % Solver configuration (options): 
    %   - 0. No output function, display iter.
    %   - 1. Save best so far, display off.
    %   - 2. Save best so far, display iter.
    %   - 3. Stop at first valid solution, display off.
    %   - 4. No output function, display off
    %   - 5. Save all found integer solutions, with iter
    if nargin < 7 || isempty(solver_config)
        solver_config = 0;
    end

    % Tolerance
    tol = 1e-6;

    % Formulation variants flags
    if nargin < 5 || isempty(formulation_variants_flags)
        % Use complete formulation
        recharges_allowed_flag                 = true;
        relays_allowed_flag                    = true;
        fragmentation_allowed_flag             = true;
        variable_number_of_robots_allowed_flag = true;
    else
        recharges_allowed_flag                 = formulation_variants_flags(1);
        relays_allowed_flag                    = formulation_variants_flags(2);
        fragmentation_allowed_flag             = formulation_variants_flags(3);
        variable_number_of_robots_allowed_flag = formulation_variants_flags(4);
    end

    % Configuration flags
    if nargin < 6 || isempty(config_flags)
        % Use default values
        solve_flag                             = true;
        initialize_flag                        = true;
        genetic_algorithm_solver               = false;
        print_solution_flag                    = false;
        display_flag                           = false;
        log_file_flag                          = false;
        save_flag                              = false;
        test_flag                              = false;
        recovery_flag                          = false;
    else
        solve_flag                             = config_flags(1);
        initialize_flag                        = config_flags(2);
        genetic_algorithm_solver               = config_flags(3);
        print_solution_flag                    = config_flags(4);
        display_flag                           = config_flags(5);
        log_file_flag                          = config_flags(6);
        save_flag                              = config_flags(7);
        test_flag                              = config_flags(8);
        recovery_flag                          = config_flags(9);
    end

    % Set save_flag to 1 if scenario is gonna be randomly generated and print_solution_flag is off to be able later to load the scenario information to print the solution manually
    if predefined == 0 && not(print_solution_flag)
        save_flag = true;
    end

    % Create needed directories if they don't exists
    if ~exist('../mat/', 'dir')
        mkdir('../mat/')
    end
    if ~exist('../fig/', 'dir')
        mkdir('../fig/')
    end
    if ~exist('../logs/', 'dir')
        mkdir('../logs/')
    end

    % Open a log file to write computing times
    if not(test_flag) && log_file_flag
        logFile = fopen('../logs/log.txt', 'a');
        assert(logFile > 0, 'Unable to open log file');
    end

    % Write the date and time of the execution in the log file
    if test_flag
        execution_id = 'test';
    else
        if isempty(execution_id)
            execution_id = char(datetime('now','Format','yyyy_MM_dd_HH_mm'));
        end
        if log_file_flag
            fprintf(logFile, '%s\n\n', ['Execution ID: ', execution_id]);
        end
    end

    %% Load or create scenario
    % Scenario: predefined scenario (0 if random), number of agents, number of tasks, types of agents
    if isempty(old_executed_random_scenario_id)
        if predefined == 0
            if nargin < 3
                error('No random scenario size specified');
            else
                [Agent, Task] = scenario(scenario_size(1), scenario_size(2), scenario_size(3));
            end
        else
            [Agent, Task] = scenario(predefined);
        end
    else
        [Agent, Task] = scenario(old_executed_random_scenario_id);
    end

    % Save Agent and Task
    if save_flag
        save(['../mat/Agent_', execution_id, '.mat'], 'Agent');
        save(['../mat/Task_', execution_id, '.mat'], 'Task');
    end

    % N == 0 are unspecified type tasks. They are treated as fragmentable tasks:
    for t = 1:length(Task)
        if not(strcmp(Task(t).name, 't_R'))
            if Task(t).N == 0
                Task(t).Relayability = 0;
                Task(t).Fragmentability = 1;
            end
        end
    end

    % Get constant scenario information values
    [Agent, Task, A, T, S, N, R, Td_a_t_t, Te_t_nf, H_a_t] = getConstantScenarioValues(Agent, Task);

    % Get objective function normalization weights
    [z_max, tmax_m, Tw_max, V_max, d_tmax_max, s_used_max] = getNormalizationWeights(Agent, Task);

    % Update Recharge maximum time
    Task(R).tmax = tmax_m;

    % Set up linearization maximum and minimum values
    % Get maximum an minimum values from Agents and Tasks
    Te_min = 0;
    Te_max = max([Task.Te]);

    % Minimum and maximum number of agents. Needed for later linearizations.
    na_min = 0;
    na_max = A;

    % Minimum and maximum task execution time. Needed for later linearizations.
    tfin_min = 0;
    tfin_max = tmax_m;

    % Minimum and maximum waiting time. Needed for later linearizations.
    Tw_min = 0;

    % Minimum and maximum task flight time. Needed for later linearizations.
    Ft_min_a = 0;
    Ft_max_a = max([Agent.Ft] - [Agent.Ft_saf]);

    %% Decision Variables
    % -------------------------------------------------------------------------------------------------------------------------------------------------------------
    % z:                   (Real)    first part of the objective function (option 1). Minimize z with z >= tfin(a,S) for all a in A. i.e. tfin(a,S) - z <= 0.
    % -------------------- ----------------------------------------------------------------------------------------------------------------------------------------
    % x_a_t_s:             (Binary)  main decision variable. It's the solution itself. 1 if task t is assigned to slot s of Agent a. 0 otherwise.
    % -------------------- ----------------------------------------------------------------------------------------------------------------------------------------
    % xx_a_t_t_s:          (Binary)  aux decision variable to linearize the product of x_a_t_s and x_a_t_s
    % -------------------- ----------------------------------------------------------------------------------------------------------------------------------------
    % V_t:                 (Real)    vacancies. difference between the number of recruiters and the number of recruiters required for task t.
    % -------------------- ----------------------------------------------------------------------------------------------------------------------------------------
    % n_t:                 (Natural) total number of times that task t (is queued) appears among all queues.
    % -------------------- ----------------------------------------------------------------------------------------------------------------------------------------
    % na_t:                (Natural) number of Agents that are selected to carry out task t. Number of agents that are executing task t simultaneously.
    % -------------------- ----------------------------------------------------------------------------------------------------------------------------------------
    % nq_t:                (Natural) number of different queues that task $t$ is queued in.
    % -------------------- ----------------------------------------------------------------------------------------------------------------------------------------
    % nq_a_t:              (Binary)  aux binary variable. 1 if task t appears in Agent a's queue. 0 otherwise.
    % -------------------- ----------------------------------------------------------------------------------------------------------------------------------------
    % nf_t:                (Natural) decision variable, solver's DoF. Number of fragments that task t is divided into. Used to get the Te_t_nf of a task.
    % -------------------- ----------------------------------------------------------------------------------------------------------------------------------------
    % nf_t_nf:             (Binary)  aux decision variable to have a binary codification of nf_t.
    % -------------------- ----------------------------------------------------------------------------------------------------------------------------------------
    % nfx_a_nf_t_s:        (Binary)  aux decision variable to linearize the product of nf_t_nf and x_a_t_s.
    % -------------------- ----------------------------------------------------------------------------------------------------------------------------------------
    % naf_t_nf:            (Natural) aux decision variable to linearize the product of na_t and nf_t_nf.
    % -------------------- ----------------------------------------------------------------------------------------------------------------------------------------
    % nax_a_t_s            (Natural) aux decision variable to linearize the product of na_t and x_a_t_s.
    % -------------------- ----------------------------------------------------------------------------------------------------------------------------------------
    % Ft_a_s:              (Real)    accumulated flight time consumed by agent a until slot s.
    % -------------------- ----------------------------------------------------------------------------------------------------------------------------------------
    % Ftx_a_s1:            (Real)    aux decision variable to linearize the product of x_a_1_s1 and Ft_a_s1.
    % -------------------- ----------------------------------------------------------------------------------------------------------------------------------------
    % tfin_a_s:            (Real)    time at with slot s of agent a ends it execution.
    % -------------------- ----------------------------------------------------------------------------------------------------------------------------------------
    % tfinx_a_t_s:         (Real)    aux decision variable to linearize the product of tfin_a_s and x_a_t_s.
    % -------------------- ----------------------------------------------------------------------------------------------------------------------------------------
    % d_tmax_tfin_a_s:     (Real)    difference between tmax_a_s and tfin_a_s. Third part of the objective function. Has a lower limit equal to 0.
    % -------------------- ----------------------------------------------------------------------------------------------------------------------------------------
    % s_used:              (Natural) number of slots used in total.
    % -------------------- ----------------------------------------------------------------------------------------------------------------------------------------
    % Td_a_s:              (Real)    time that agent a spends traveling from its previous location to the slot s task location.
    % -------------------- ----------------------------------------------------------------------------------------------------------------------------------------
    % Tw_a_s:              (Real)    time that the task in slot s of agent a must wait after Td and before Te to synch its execution with the other Agent's tasks.
    % -------------------- ----------------------------------------------------------------------------------------------------------------------------------------
    % Twx_a_s:             (Real)    aux decision variable to linearize the product of Tw_a_s and x_a_t_s.
    % -------------------- ----------------------------------------------------------------------------------------------------------------------------------------
    % Te_a_s:              (Real)    time that the task in slot s of agent a takes to execute.
    % -------------------- ----------------------------------------------------------------------------------------------------------------------------------------
    % S_t_a1_s1_a2_s2:     (Binary)  Synch DV. Indicates whether there's a task t synchronization between (a1,s1) and (a2,s2) or not. 1 if there is. 0 otherwise.
    % -------------------- ----------------------------------------------------------------------------------------------------------------------------------------
    % tfinS_t_a1_s1_a2_s2: (Real)    aux decision variable to linearize de product of S_t_a1_s1_a2_s2 and tfin_a_s. Note that S_t_a1_s1_a2_s2 = S_t_a2_s2_a1_s1.
    % -------------------- ----------------------------------------------------------------------------------------------------------------------------------------
    % R_t_a1_s1_a2_s2:     (Binary)  Relays' DV. Indicates whether there's a task t relay between (a1,s1) and (a2,s2) or not. 1 if there is. 0 otherwise.
    % -------------------- ----------------------------------------------------------------------------------------------------------------------------------------
    % tfinR_t_a1_s1_a2_s2: (Real)    aux decision variable to linearize de product of R_t_a1_s1_a2_s2 and tfin_a1_s1. Note that R_t_a1_s1_a2_s2 != R_t_a2_s2_a1_s1.
    % -------------------- ----------------------------------------------------------------------------------------------------------------------------------------
    % TeR_t_a1_s1_a2_s2:   (Real)    aux decision variable to linearize de product of R_t_a1_s1_a2_s2 and Te_a2_s2. Note that R_t_a1_s1_a2_s2 != R_t_a2_s2_a1_s1.
    % -------------------------------------------------------------------------------------------------------------------------------------------------------------

    % Get start length structure information
    [dv_start_length, length_dv] = getStartLengthInformation(Agent, Task);

    % Get all dv start and length information
    [start_z,                   ~,                      ...
     start_x_a_t_s,             length_x_a_t_s,         ...
     start_xx_a_t_t_s,          length_xx_a_t_t_s,      ...
     start_V_t,                 length_V_t,             ...
     start_n_t,                 length_n_t,             ...
     start_na_t,                length_na_t,            ...
     start_nq_t,                length_nq_t,            ...
     start_nq_a_t,              length_nq_a_t,          ...
     start_nf_t,                length_nf_t,            ...
     start_nf_t_nf,             length_nf_t_nf,         ...
     start_nfx_a_nf_t_s,        length_nfx_a_nf_t_s,    ...
     start_naf_t_nf,            length_naf_t_nf,        ...
     start_nax_a_t_s,           length_nax_a_t_s,       ...
     start_Ft_a_s,              ~,                      ...
     start_Ftx_a_s1,            length_Ftx_a_s1,        ...
     start_tfin_a_s,            ~,                      ...
     start_tfinx_a_t_s,         ~,                      ...
     start_d_tmax_tfin_a_s,     length_d_tmax_tfin_a_s, ...
     start_s_used,              length_s_used,          ...
     start_Td_a_s,              length_Td_a_s,          ...
     start_Tw_a_s,              length_Tw_a_s,          ...
     start_Twx_a_s,             length_Twx_a_s,         ...
     start_Te_a_s,              length_Te_a_s,          ...
     start_S_t_a1_s1_a2_s2,     length_S_t_a1_s1_a2_s2, ...
     start_tfinS_t_a1_s1_a2_s2, ~,                      ...
     start_R_t_a1_s1_a2_s2,     length_R_t_a1_s1_a2_s2, ...
     start_tfinR_t_a1_s1_a2_s2, ~,                      ...
     start_TeR_t_a1_s1_a2_s2,   length_TeR_t_a1_s1_a2_s2] = extractStartLengthInformation(dv_start_length);

    % Get sparse matrix information
    [~, ...
     max_eq,   max_eq_sparse_terms,   max_eq_independent_sparse_terms,   ...
     max_ineq, max_ineq_sparse_terms, max_ineq_independent_sparse_terms, ...
     start_N_Hard_eq,            ...
     start_Non_decomposable_eq,  ...
     start_Td_a_s_eq,            ...
     start_Te_a_s_eq,            ...
     start_n_t_eq,               ...
     start_nf_t_nf_eq,           ...
     start_nq_t_eq,              ...
     start_na_nf_n_t_eq,         ...
     start_t_fin_a_s_eq,         ...
     start_Ft_a_s_eq,            ...
     start_V_t_eq,               ...
     start_s_used_eq,            ...
     start_synch_eq,             ...
     start_relays_eq,            ...
     start_z_ineq,               ...
     start_nq_a_t_ineq,          ...
     start_na_nq_t_ineq,         ...
     start_naf_t_nf_ineq,        ...
     start_d_tmax_tfin_a_s_ineq, ...
     start_Ft_a_s_ineq,          ...
     start_recharge_ineq,        ...
     start_hardware_ineq,        ...
     start_max1task_ineq,        ...
     start_continuity_ineq,      ...
     start_synch_ineq,           ...
     start_relays_ineq,          ...
     start_linearizations_ineq] = getSparseMatrixInformation(Agent, Task);

    % Objective function equations: memory is reserved for the maximum expected number of sparse elements
    A_double_sparse   = spalloc(max_ineq, length_dv, max_ineq_sparse_terms);
    b_double_sparse   = spalloc(max_ineq,         1, max_ineq_independent_sparse_terms);
    Aeq_double_sparse = spalloc(max_eq,   length_dv, max_eq_sparse_terms);
    beq_double_sparse = spalloc(max_eq,           1, max_eq_independent_sparse_terms);

    % dv = [z,      x_a_t_s,   xx_a_t_t_s,   V_t,    n_t,       na_t,      nq_t,      nq_a_t,   nf_t,      nf_t_nf,   nfx_a_nf_t_s,   naf_t_nf,   nax_a_t_s,   Ft_a_s,   Ftx_a_s1,   tfin_a_s,   tfinx_a_t_s,   d_tmax_tfin_a_s,   s_used,    Td_a_s,    Tw_a_s,    Twx_a_s,    Te_a_s,    S_t_a1_s1_a2_s2,    tfinS_t_a1_s1_a2_s2,    R_t_a1_s1_a2_s2,    tfinR_t_a1_s1_a2_s2,    TeR_t_a1_s1_a2_s2];
    %      [Real,   Binary,    Binary,       Real,   Natural,   Natural,   Natural,   Binary,   Natural,   Binary,    Binary,         Natural,    Natural,     Real,     Real,       Real,       Real,          Real,              Natural,   Real,      Real,      Real,       Real,      Binary,             Real,                   Binary,             Real               ,    Real             ];

    % Specify optimization variables that are integers/natural (or binary). Indexed of the variables that must be integers/natural (or binary with bounds).
    intcon = [  start_x_a_t_s         : start_x_a_t_s         + length_x_a_t_s         - 1, ...
                start_xx_a_t_t_s      : start_xx_a_t_t_s      + length_xx_a_t_t_s      - 1, ...
                start_n_t             : start_n_t             + length_n_t             - 1, ...
                start_na_t            : start_na_t            + length_na_t            - 1, ...
                start_nq_t            : start_nq_t            + length_nq_t            - 1, ...
                start_nq_a_t          : start_nq_a_t          + length_nq_a_t          - 1, ...
                start_nf_t            : start_nf_t            + length_nf_t            - 1, ...
                start_nf_t_nf         : start_nf_t_nf         + length_nf_t_nf         - 1, ...
                start_nfx_a_nf_t_s    : start_nfx_a_nf_t_s    + length_nfx_a_nf_t_s    - 1, ...
                start_naf_t_nf        : start_naf_t_nf        + length_naf_t_nf        - 1, ...
                start_nax_a_t_s       : start_nax_a_t_s       + length_nax_a_t_s       - 1, ...
                start_s_used          : start_s_used          + length_s_used          - 1, ...
                start_S_t_a1_s1_a2_s2 : start_S_t_a1_s1_a2_s2 + length_S_t_a1_s1_a2_s2 - 1, ...
                start_R_t_a1_s1_a2_s2 : start_R_t_a1_s1_a2_s2 + length_R_t_a1_s1_a2_s2 - 1]';

    % Set the bounds for the integer variables: -) Natural variables are lower bounded by 0 and upper bounded by inf. -) Binary variables are lower bounded by 0 and upper bounded by 1. -) Integer variables are lower bounded by -inf and upper bounded by inf.
    % Lower bounds for the value of the variables
    aux_lb_n_t = ones(T,1);
    aux_lb_n_t(R) = 0;
    lb = -inf*ones(length_dv,1);
    lb(start_x_a_t_s           : start_x_a_t_s           + length_x_a_t_s           - 1) = 0;
    lb(start_xx_a_t_t_s        : start_xx_a_t_t_s        + length_xx_a_t_t_s        - 1) = 0;
    lb(start_V_t               : start_V_t               + length_V_t               - 1) = 0;
    lb(start_n_t               : start_n_t               + length_n_t               - 1) = aux_lb_n_t;
    lb(start_na_t              : start_na_t              + length_na_t              - 1) = aux_lb_n_t;
    lb(start_nq_t              : start_nq_t              + length_nq_t              - 1) = aux_lb_n_t;
    lb(start_nq_a_t            : start_nq_a_t            + length_nq_a_t            - 1) = 0;
    lb(start_nf_t              : start_nf_t              + length_nf_t              - 1) = 1;
    lb(start_nf_t_nf           : start_nf_t_nf           + length_nf_t_nf           - 1) = 0;
    lb(start_nfx_a_nf_t_s      : start_nfx_a_nf_t_s      + length_nfx_a_nf_t_s      - 1) = 0;
    lb(start_naf_t_nf          : start_naf_t_nf          + length_naf_t_nf          - 1) = 0;
    lb(start_nax_a_t_s         : start_nax_a_t_s         + length_nax_a_t_s         - 1) = 0;
    lb(start_Ftx_a_s1          : start_Ftx_a_s1          + length_Ftx_a_s1          - 1) = 0;
    lb(start_d_tmax_tfin_a_s   : start_d_tmax_tfin_a_s   + length_d_tmax_tfin_a_s   - 1) = 0;
    lb(start_s_used            : start_s_used            + length_s_used            - 1) = 0;
    lb(start_Td_a_s            : start_Td_a_s            + length_Td_a_s            - 1) = 0;
    lb(start_Tw_a_s            : start_Tw_a_s            + length_Tw_a_s            - 1) = 0;
    lb(start_Twx_a_s           : start_Twx_a_s           + length_Twx_a_s           - 1) = 0;
    lb(start_Te_a_s            : start_Te_a_s            + length_Te_a_s            - 1) = 0;
    lb(start_S_t_a1_s1_a2_s2   : start_S_t_a1_s1_a2_s2   + length_S_t_a1_s1_a2_s2   - 1) = 0;
    lb(start_R_t_a1_s1_a2_s2   : start_R_t_a1_s1_a2_s2   + length_R_t_a1_s1_a2_s2   - 1) = 0;
    lb(start_TeR_t_a1_s1_a2_s2 : start_TeR_t_a1_s1_a2_s2 + length_TeR_t_a1_s1_a2_s2 - 1) = 0;

    % Upper bounds for the value of the variables
    ub = inf*ones(length_dv,1);
    ub(start_x_a_t_s         : start_x_a_t_s         + length_x_a_t_s         - 1) = 1;
    ub(start_xx_a_t_t_s      : start_xx_a_t_t_s      + length_xx_a_t_t_s      - 1) = 1;
    ub(start_nq_a_t          : start_nq_a_t          + length_nq_a_t          - 1) = 1;
    ub(start_nf_t_nf         : start_nf_t_nf         + length_nf_t_nf         - 1) = 1;
    ub(start_nfx_a_nf_t_s    : start_nfx_a_nf_t_s    + length_nfx_a_nf_t_s    - 1) = 1;
    ub(start_Ftx_a_s1        : start_Ftx_a_s1        + length_Ftx_a_s1        - 1) = repmat([Agent.Ft] - [Agent.Ft_saf], 1, S)';
    ub(start_S_t_a1_s1_a2_s2 : start_S_t_a1_s1_a2_s2 + length_S_t_a1_s1_a2_s2 - 1) = 1;
    ub(start_R_t_a1_s1_a2_s2 : start_R_t_a1_s1_a2_s2 + length_R_t_a1_s1_a2_s2 - 1) = 1;

    % Fix nf(R) value to 1
    ub((start_nf_t - 1) + R) = 1;

    % Force initial task (task in slot 0) to be Task(0) (initial positions of the Agents, used to compute Td for the task in the first slot). It becomes Task(1) and Slot(1) because matlab starts counting from 1.
    s = 0;
    for a = 1:A
        for t = 0:T
            if t == 0
                lb((start_x_a_t_s - 1) + (a + ((t + 1) - 1)*A + ((s + 1) - 1)*A*(T+1))) = 1;
                ub((start_x_a_t_s - 1) + (a + ((t + 1) - 1)*A + ((s + 1) - 1)*A*(T+1))) = 1;
            else
                lb((start_x_a_t_s - 1) + (a + ((t + 1) - 1)*A + ((s + 1) - 1)*A*(T+1))) = 0;
                ub((start_x_a_t_s - 1) + (a + ((t + 1) - 1)*A + ((s + 1) - 1)*A*(T+1))) = 0; 
            end
        end
    end

    % Recharges allowed or not
    % If recharges_allowed_flag == 0: x(a,R,s) = 0, for all a = 1 to A, s = 1 to S
    if not(recharges_allowed_flag)
        for a = 1:A
            for s = 1:S
                lb((start_x_a_t_s - 1) + (a + ((R + 1) - 1)*A + ((s + 1) - 1)*A*(T+1))) = 0;
                ub((start_x_a_t_s - 1) + (a + ((R + 1) - 1)*A + ((s + 1) - 1)*A*(T+1))) = 0; 
            end
        end
    end

    % Relays allowed or not
    if not(relays_allowed_flag)
        for t = 1:T
            if t ~= R
                Task(t).Relayability = 0;
            end
        end
    end

    % Fragmentation allowed or not
    if not(fragmentation_allowed_flag)
        for t = 1:T
            if t ~= R
                Task(t).Fragmentability = 0;
            end
        end
    end

    % Variable or fixed number of robots
    if not(variable_number_of_robots_allowed_flag)
        for t = 1:T
            if t ~= R
                Task(t).N_hardness = 1;
                if Task(t).N == 0
                    Task(t).N = 1;
                end
            end
        end
    end

    %% Initialize parallel pool
    if not(test_flag)
        pool = gcp;
        if isempty(pool)
            error('No parallel pool found. Automatic pool start may be disabled.');
        end

        if display_flag
            disp(['Initialization time: ', num2str(toc - last_toc), ' s']);
        end
        if log_file_flag
            fprintf(logFile, 'Initialization time: %f s\n', toc - last_toc);
        end
    end

    %% Equations and constraints
    last_toc = toc;

    % Initial guess
    x0 = [];

    % Call initializer
    if initialize_flag
        if isempty(old_executed_random_scenario_id)
            [init_Agent, init_Task, allocation_order, init_S_R] = initializer(Agent, Task, 2, 2);
        else
            [init_Agent, init_Task, allocation_order, init_S_R] = initializer(old_executed_random_scenario_id, execution_id, 2, 2);
        end

        % Compute the solution array
        [init_sol, ~, ~] = buildSolutionArray(init_S_R, init_Agent, init_Task, objective_function, scenario_id, false);

        if checkSolution(init_sol, init_Agent, init_Task)
            % Check if initializer solution can be used as initial solution
            if length(init_sol) == length_dv
                x0 = init_sol;
            else
                disp('Initializer''s solution use more than S slots');
            end
        else
            disp('Initializer wasn''t able to find a solution');
        end
    end

    % Use a solver finded solution
    if recovery_flag
        try
            x0 = load('../mat/bestSolutionSoFar.mat');
            x0 = x0.x;
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

    %% N-hardness
    parfor t = 1:T
        if t ~= R
            if Task(t).N_hardness
                A_tmp = spalloc(max_eq, length_dv, 1);
                % na(t) = N(t) -> V(t) = 0
                A_tmp((start_N_Hard_eq - 1) + (t - 1), (start_V_t - 1) + t) = 1;
                Aeq_double_sparse = Aeq_double_sparse + A_tmp;

                % Check whether the initial value meets this constraint
                if not(isempty(x0))
                    error = abs(A_tmp((start_N_Hard_eq - 1) + (t - 1),:) * x0 - 0);
                    if error > tol
                        disp(['N_Hard eq not met for t = ', num2str(t), ' by ', num2str(error)]);
                    end
                end
            end
        end
    end

    %% Non-decomposable
    parfor t = 1:T
        if t ~= R
            if Task(t).Relayability == 0 && Task(t).Fragmentability == 0
                A_tmp = spalloc(max_eq, length_dv, 1);
                b_tmp = spalloc(max_eq, 1, 1);
                % nf(t) = 1
                A_tmp((start_Non_decomposable_eq - 1) + (t - 1), (start_nf_t - 1) + t) = 1;
                b_tmp((start_Non_decomposable_eq - 1) + (t - 1)) = 1;
                Aeq_double_sparse = Aeq_double_sparse + A_tmp;
                beq_double_sparse = beq_double_sparse + b_tmp;

                % Check whether the initial value meets this constraint
                if not(isempty(x0))
                    error = abs(A_tmp((start_Non_decomposable_eq - 1) + (t - 1),:) * x0 - b_tmp((start_Non_decomposable_eq - 1) + (t - 1),:));
                    if error > tol
                        disp(['Non_decomposable eq not met for t = ', num2str(t), ' by ', num2str(error)]);
                    end
                end
            end
        end
    end

    %% Td(a,s)
    % Td(a,s) = sum from t = 1 to T of (sum from t2 = 0 to T of (Td_a(t,t2) * x(a,t2,s-1)) * x(a,t,s))
    % Td(a,s) = sum from t = 1 to T of (sum from t2 = 0 to T of (Td_a(t,t2) * x(a,t2,s-1) * x(a,t,s)))
    % Td(a,s) = sum from t = 1 to T of (sum from t2 = 0 to T of (Td_a(t,t2) * x(a,t,t2,s)))
    parfor a = 1:A
        for s = 1:S
            A_tmp = spalloc(max_eq, length_dv, (1 + (T * (T + 1)) * 1));
            % - Td(a,s) + sum from t = 1 to T of (sum from t2 = 0 to T of (Td_a(t,t2) * x(a,t2,s-1) * x(a,t,s))) = 0
            A_tmp((start_Td_a_s_eq - 1) + (a + (s - 1)*A), (start_Td_a_s - 1) + (a + (s - 1)*A)) = -1;
            for t = 1:T
                for t2 = 0:T
                    % Td_a(t,t2) * x(a,t2,s-1)*x(a,t,s) -> Td_a(t,t2) * xx_a_t_t_s
                    A_tmp((start_Td_a_s_eq - 1) + (a + (s - 1)*A), (start_xx_a_t_t_s - 1) + (a + ((t2 + 1) - 1)*A + (t - 1)*A*(T+1) + (s - 1)*A*(T+1)*T)) = Td_a_t_t(a,t,(t2 + 1));
                end
            end
            Aeq_double_sparse = Aeq_double_sparse + A_tmp;

            % Check whether the initial value meets this constraint
            if not(isempty(x0))
                error = abs(A_tmp((start_Td_a_s_eq - 1) + (a + (s - 1)*A),:) * x0 - 0);
                if error > tol
                    disp(['Td_a_s eq not met for (a,s) = (', num2str(a), ', ', num2str(s), ') by ', num2str(error)]);
                end
            end
        end
    end

    %% Te(a,s)
    % Te(a,s) = sum from t = 1 to T of (sum from nf = 1 to N of (Te(t,nf) * nf(t,nf)) * x(a,t,s))
    % Te(a,s) = sum from t = 1 to T of (sum from nf = 1 to N of (Te(t,nf) * nf(t,nf) * x(a,t,s)))
    % Te(a,s) = sum from t = 1 to T of (sum from nf = 1 to N of (Te(t,nf) * nfx(a,nf,t,s)))
    parfor a = 1:A
        for s = 1:S
            A_tmp = spalloc(max_eq, length_dv, (1 + (T * N) * 1));
            % - Te(a,s) + sum from t = 2 to T of (sum from nf = 1 to N of (Te(t,nf) * nf(t,nf) * x(a,t,s))) = 0
            A_tmp((start_Te_a_s_eq - 1) + (a + (s - 1)*A), (start_Te_a_s - 1) + (a + (s - 1)*A)) = -1;
            for t = 1:T
                % Te(t)
                for nf = 1:N
                    % Te(t,nf) * nf(t,nf)*x(a,t,s) -> Te(t,nf) * nfx_a_nf_t_s
                    A_tmp((start_Te_a_s_eq - 1) + (a + (s - 1)*A), (start_nfx_a_nf_t_s - 1) + (a + (nf - 1)*A + (t - 1)*A*N + (s - 1)*A*N*T)) = Te_t_nf(t,nf);
                end
            end
            Aeq_double_sparse = Aeq_double_sparse + A_tmp;

            % Check whether the initial value meets this constraint
            if not(isempty(x0))
                error = abs(A_tmp((start_Te_a_s_eq - 1) + (a + (s - 1)*A),:) * x0 - 0);
                if error > tol
                    disp(['Te_a_s eq not met for (a,s) = (', num2str(a), ', ', num2str(s), ') by ', num2str(error)]);
                end
            end
        end
    end

    %% z: minimize the maximum time from all agents
    parfor a = 1:A
        A_tmp = spalloc(max_ineq, length_dv, 2);
        % z = max(tfin(a,S)) -> tfin(a,S) - z <= 0
        A_tmp((start_z_ineq - 1) + a, (start_tfin_a_s - 1) + (a + ((S + 1) - 1)*A)) = 1;
        A_tmp((start_z_ineq - 1) + a, (start_z - 1) + 1) = -1;
        A_double_sparse = A_double_sparse + A_tmp;

        % Check whether the initial value meets this constraint
        if not(isempty(x0))
            error = A_tmp((start_z_ineq - 1) + a,:) * x0 - 0;
            if error > tol
                disp(['z ineq not met for a = ', num2str(a), ' by ', num2str(error)]);
            end
        end
    end

    %% n(t): Total number of times that task t appears among all queues.
    % n(t) = sum from a = 1 to A of (sum from s = 1 to S of (x(a,t,s))), for all t = 1 to T
    parfor t = 1:T
        A_tmp = spalloc(max_eq, length_dv, (1 + (A * S) * 1));
        % - n(t) + sum from a = 1 to A of (sum from s = 1 to S of (x(a,t,s))) = 0
        A_tmp((start_n_t_eq - 1) + t, (start_n_t - 1) + t) = -1;
        for a = 1:A
            for s = 1:S
                % x(a,t,s)
                A_tmp((start_n_t_eq - 1) + t, (start_x_a_t_s - 1) + (a + ((t + 1) - 1)*A + ((s + 1) - 1)*A*(T+1))) = 1;
            end
        end
        Aeq_double_sparse = Aeq_double_sparse + A_tmp;

        % Check whether the initial value meets this constraint
        if not(isempty(x0))
            error = abs(A_tmp((start_n_t_eq - 1) + t,:) * x0 - 0);
            if error > tol
                disp(['n_t eq not met for t = ', num2str(t), ' by ', num2str(error)]);
            end
        end
    end

    % Set nf_t_nf aux decision variable value depending on nf_t
    % sum from nf = 1 to N of (nf * nf(t,nf)) = nf(t), for all t = 1 to T
    parfor t = 1:T
        A_tmp = spalloc(max_eq, length_dv, (1 + N * 1));
        % - nf(t) + sum from nf = 1 to N of (nf * nf(t,nf)) = 0
        A_tmp((start_nf_t_nf_eq - 1) + t, (start_nf_t - 1) + t) = -1;
        for nf = 1:N
            % nf * nf(t,nf)
            A_tmp((start_nf_t_nf_eq - 1) + t, (start_nf_t_nf - 1) + (t + (nf - 1)*T)) = nf;
        end
        Aeq_double_sparse = Aeq_double_sparse + A_tmp;

        % Check whether the initial value meets this constraint
        if not(isempty(x0))
            error = abs(A_tmp((start_nf_t_nf_eq - 1) + t,:) * x0 - 0);
            if error > tol
                disp(['nf_t_nf eq not met for t = ', num2str(t), ' by ', num2str(error)]);
            end
        end
    end

    % nf_t_nf value shouldn't be a combination of several nf values
    % sum from nf = 1 to N of (nf(t,nf)) = 1, for all t = 1 to T
    parfor t = 1:T
        A_tmp = spalloc(max_eq, length_dv, (N * 1));
        b_tmp = spalloc(max_eq, 1, 1);
        % sum from nf = 1 to N of (nf(t,nf)) = 1
        for nf = 1:N
            % nf(t,nf)
            A_tmp((start_nf_t_nf_eq - 1 + T) + t, (start_nf_t_nf - 1) + (t + (nf - 1)*T)) = 1;
        end
        b_tmp((start_nf_t_nf_eq - 1 + T) + t) = 1;
        Aeq_double_sparse = Aeq_double_sparse + A_tmp;
        beq_double_sparse = beq_double_sparse + b_tmp;

        % Check whether the initial value meets this constraint
        if not(isempty(x0))
            error = abs(A_tmp((start_nf_t_nf_eq - 1 + T) + t,:) * x0 - b_tmp((start_nf_t_nf_eq - 1 + T) + t,:));
            if error > tol
                disp(['nf_t_nf eq not met for t = ', num2str(t), ' by ', num2str(error)]);
            end
        end
    end

    % nq(t): number of queues that task t appears in.
    % nq(t) = sum from a = 1 to A of (nq(a,t)), for all t = 1 to T
    parfor t = 1:T
        A_tmp = spalloc(max_eq, length_dv, (1 + A * 1));
        % - nq(t) + sum from a = 1 to A of (nq(a,t)) = 0
        A_tmp((start_nq_t_eq - 1) + t, (start_nq_t - 1) + (t)) = -1;
        for a = 1:A
            % nq(a,t)
            A_tmp((start_nq_t_eq - 1) + t, (start_nq_a_t - 1) + (a + (t - 1)*A)) = 1;
        end
        Aeq_double_sparse = Aeq_double_sparse + A_tmp;

        % Check whether the initial value meets this constraint
        if not(isempty(x0))
            error = abs(A_tmp((start_nq_t_eq - 1) + t,:) * x0 - 0);
            if error > tol
                disp(['nq_t eq not met for t = ', num2str(t), ' by ', num2str(error)]);
            end
        end
    end

    % nq(a,t): aux binary variable. 1 if task t appears in Agent a's queue. 0 otherwise.
    % nq(a,t) = OR(x(a,t,s)), for all t = 1 to T, for all a = 1 to A (S input logic OR)
    % To linearize the S input logic OR gate we need to add the following constraints:
    % nq_a_t >= xa_t_s, for all t = 1 to T, for all a = 1 to A, for all s = 1 to S
    parfor a = 1:A
        for t = 1:T
            for s = 1:S
                A_tmp = spalloc(max_ineq, length_dv, 2);
                % x(a,t,s) - nq(a,t) <= 0
                A_tmp((start_nq_a_t_ineq - 1) + (a + (t - 1)*A + (s - 1)*A*T), (start_x_a_t_s - 1) + (a + ((t + 1) - 1)*A + ((s + 1) - 1)*A*(T+1))) = 1;
                A_tmp((start_nq_a_t_ineq - 1) + (a + (t - 1)*A + (s - 1)*A*T), (start_nq_a_t - 1) + (a + (t - 1)*A)) = -1;
                A_double_sparse = A_double_sparse + A_tmp;

                % Check whether the initial value meets this constraint
                if not(isempty(x0))
                    error = A_tmp((start_nq_a_t_ineq - 1) + (a + (t - 1)*A + (s - 1)*A*T),:) * x0 - 0;
                    if error > tol
                        disp(['nq_a_t ineq not met for (a,t,s) = (', num2str(a), ', ', num2str(t), ', ', num2str(s), ') by ', num2str(error)]);
                    end
                end
            end
        end
    end

    % nq_a_t <= sum from s = 1 to S of (x(a,t,s)), for all t = 1 to T, for all a = 1 to A
    parfor a = 1:A
        for t = 1:T
            A_tmp = spalloc(max_ineq, length_dv, (1 + S * 1));
            % nq(a,t) - sum from s = 1 to S of (x(a,t,s)) <= 0
            A_tmp((start_nq_a_t_ineq - 1 + A*T*S) + (a + (t - 1)*A), (start_nq_a_t - 1) + (a + (t - 1)*A)) = 1;
            for s = 1:S
                % - x(a,t,s)
                A_tmp((start_nq_a_t_ineq - 1 + A*T*S) + (a + (t - 1)*A), (start_x_a_t_s - 1) + (a + ((t + 1) - 1)*A + ((s + 1) - 1)*A*(T+1))) = -1;
            end
            A_double_sparse = A_double_sparse + A_tmp;

            % Check whether the initial value meets this constraint
            if not(isempty(x0))
                error = A_tmp((start_nq_a_t_ineq - 1 + A*T*S) + (a + (t - 1)*A),:) * x0 - 0;
                if error > tol
                    disp(['nq_a_t ineq not met for (a,t) = (', num2str(a), ', ', num2str(t), ') by ', num2str(error)]);
                end
            end
        end
    end

    % na(t) <= nq(t), for all t = 2 to T
    parfor t = 1:T
        if t ~= R
            A_tmp = spalloc(max_ineq, length_dv, 2);
            % na(t) - nq(t) <= 0
            A_tmp((start_na_nq_t_ineq - 1) + t, (start_na_t - 1) + t) = 1;
            A_tmp((start_na_nq_t_ineq - 1) + t, (start_nq_t - 1) + t) = -1;
            A_double_sparse = A_double_sparse + A_tmp;

            % Check whether the initial value meets this constraint
            if not(isempty(x0))
                error = A_tmp((start_na_nq_t_ineq - 1) + t,:) * x0 - 0;
                if error > tol
                    disp(['na_nq_t ineq not met for t = ', num2str(t), ' by ', num2str(error)]);
                end
            end
        end
    end

    % na(t) * nf(t) = n(t), for all t = 1 to T -> ...
    % na(t) * sum from nf = 1 to N of (nf * nf(t,nf)) = n(t), for all t = 1 to T -> ...
    % sum form nf = 1 to N of (nf * naf(t,nf)) = n(t), for all t = 1 to T
    parfor t = 1:T
        if t ~= R
            A_tmp = spalloc(max_eq, length_dv, (1 + N * 1));
            % - n(t) + sum form nf = 1 to N of (nf * naf(t,nf)) = 0
            A_tmp((start_na_nf_n_t_eq - 1) + t, (start_n_t - 1) + t) = -1;
            for nf = 1:N
                % nf * naf(t,nf)
                A_tmp((start_na_nf_n_t_eq - 1) + t, (start_naf_t_nf - 1) + (t + (nf - 1)*T)) = nf;
            end
            Aeq_double_sparse = Aeq_double_sparse + A_tmp;

            % Check whether the initial value meets this constraint
            if not(isempty(x0))
                error = abs(A_tmp((start_na_nf_n_t_eq - 1) + t,:) * x0 - 0);
                if error > tol
                    disp(['na_nf_n_t eq not met for t = ', num2str(t), ' by ', num2str(error)]);
                end
            end
        end
    end

    % Add constraints to force the value of the naf_t_nf aux variable to be the same as its original hight order terms.
    parfor t = 1:T
        for nf = 1:N
            A_tmp = spalloc(max_ineq, length_dv, 2);
            % nf(t,nf)*na_min - naf_t_nf <= 0
            A_tmp((start_naf_t_nf_ineq - 1) + (t + (nf - 1)*T), (start_nf_t_nf - 1)  + (t + (nf - 1)*T)) = na_min;
            A_tmp((start_naf_t_nf_ineq - 1) + (t + (nf - 1)*T), (start_naf_t_nf - 1) + (t + (nf - 1)*T)) = -1;
            A_double_sparse = A_double_sparse + A_tmp;

            % Check whether the initial value meets this constraint
            if not(isempty(x0))
                error = A_tmp((start_naf_t_nf_ineq - 1) + (t + (nf - 1)*T),:) * x0 - 0;
                if error > tol
                    disp(['naf_t_nf ineq not met for (t,nf) = (', num2str(t), ', ', num2str(nf), ') by ', num2str(error)]);
                end
            end
            
            A_tmp = spalloc(max_ineq, length_dv, 2);
            % naf_t_nf - nf(t,nf)*na_max <= 0
            A_tmp((start_naf_t_nf_ineq - 1 + 1*T*N) + (t + (nf - 1)*T), (start_naf_t_nf - 1) + (t + (nf - 1)*T)) = 1;
            A_tmp((start_naf_t_nf_ineq - 1 + 1*T*N) + (t + (nf - 1)*T), (start_nf_t_nf - 1)  + (t + (nf - 1)*T)) = - na_max;
            A_double_sparse = A_double_sparse + A_tmp;

            % Check whether the initial value meets this constraint
            if not(isempty(x0))
                error = A_tmp((start_naf_t_nf_ineq - 1 + 1*T*N) + (t + (nf - 1)*T),:) * x0 - 0;
                if error > tol
                    disp(['naf_t_nf ineq 1 not met for (t,nf) = (', num2str(t), ', ', num2str(nf), ') by ', num2str(error)]);
                end
            end
            
            A_tmp = spalloc(max_ineq, length_dv, 3);
            b_tmp = spalloc(max_ineq, 1, 1);
            % na(t) + na_max*nf(t,nf) - naf_t_nf <= na_max
            A_tmp((start_naf_t_nf_ineq - 1 + 2*T*N) + (t + (nf - 1)*T), (start_na_t - 1) + t) = 1;
            A_tmp((start_naf_t_nf_ineq - 1 + 2*T*N) + (t + (nf - 1)*T), (start_nf_t_nf - 1)  + (t + (nf - 1)*T)) = na_max;
            A_tmp((start_naf_t_nf_ineq - 1 + 2*T*N) + (t + (nf - 1)*T), (start_naf_t_nf - 1) + (t + (nf - 1)*T)) = -1;
            b_tmp((start_naf_t_nf_ineq - 1 + 2*T*N) + (t + (nf - 1)*T)) = na_max;
            A_double_sparse = A_double_sparse + A_tmp;
            b_double_sparse = b_double_sparse + b_tmp;

            % Check whether the initial value meets this constraint
            if not(isempty(x0))
                error = A_tmp((start_naf_t_nf_ineq - 1 + 2*T*N) + (t + (nf - 1)*T),:) * x0 - b_tmp((start_naf_t_nf_ineq - 1 + 2*T*N) + (t + (nf - 1)*T),:);
                if error > tol
                    disp(['naf_t_nf ineq 2 not met for (t,nf) = (', num2str(t), ', ', num2str(nf), ') by ', num2str(error)]);
                end
            end
            
            A_tmp = spalloc(max_ineq, length_dv, 3);
            b_tmp = spalloc(max_ineq, 1, 1);
            % naf_t_nf - na(t) - na_min*nf(t,nf) <= - na_min
            A_tmp((start_naf_t_nf_ineq - 1 + 3*T*N) + (t + (nf - 1)*T), (start_naf_t_nf - 1) + (t + (nf - 1)*T)) = 1;
            A_tmp((start_naf_t_nf_ineq - 1 + 3*T*N) + (t + (nf - 1)*T), (start_na_t - 1) + t) = -1;
            A_tmp((start_naf_t_nf_ineq - 1 + 3*T*N) + (t + (nf - 1)*T), (start_nf_t_nf - 1)  + (t + (nf - 1)*T)) = - na_min;
            b_tmp((start_naf_t_nf_ineq - 1 + 3*T*N) + (t + (nf - 1)*T)) = - na_min;
            A_double_sparse = A_double_sparse + A_tmp;
            b_double_sparse = b_double_sparse + b_tmp;

            % Check whether the initial value meets this constraint
            if not(isempty(x0))
                error = A_tmp((start_naf_t_nf_ineq - 1 + 3*T*N) + (t + (nf - 1)*T),:) * x0 - b_tmp((start_naf_t_nf_ineq - 1 + 3*T*N) + (t + (nf - 1)*T),:);
                if error > tol
                    disp(['naf_t_nf ineq 3 not met for (t,nf) = (', num2str(t), ', ', num2str(nf), ') by ', num2str(error)]);
                end
            end
        end
    end

    %% d_tmax_tfin_a_s
    % It's only penalized when tfin(a,s) > tmax(a,s), so we force d_tmax_tfin_a_s >= 0 and then we only have to compute the following eq. avoiding to compute the absolute value.
    % d_tmax_tfin_a_s >= tfin(a,s) * sum from t = 1 to T of (x(a,t,s)) - tmax(a,s), for all a = 1 to A and s = 1 to S
    % d_tmax_tfin_a_s >= sum from t = 1 to T of (tfin(a,s) * x(a,t,s)) - sum from t = 1 to T of (tmax(t)*x(a,t,s)), for all a = 1 to A and s = 1 to S
    % d_tmax_tfin_a_s >= sum from t = 1 to T of (tfinx(a,t,s) - tmax(t)*x(a,t,s)), for all a = 1 to A and s = 1 to S
    parfor a = 1:A
        for s = 1:S
            A_tmp = spalloc(max_ineq, length_dv, 3);
            % tfin(a,s)*x(a,t,s) -> tfinx_a_t_s
            % tfinx_(a,t,s) - tmax(a,s) - d_tmax_tfin_a_s <= 0
            for t = 1:T
                A_tmp((start_d_tmax_tfin_a_s_ineq - 1) + (a + (s - 1)*A), (start_tfinx_a_t_s - 1) + (a + (t - 1)*A + (s - 1)*A*T)) = 1;
                % - tmax(t) * x(a,t,s)
                A_tmp((start_d_tmax_tfin_a_s_ineq - 1) + (a + (s - 1)*A), (start_x_a_t_s - 1) + (a + ((t + 1) - 1)*A + ((s + 1) - 1)*A*(T+1))) = - Task(t).tmax;
            end
            A_tmp((start_d_tmax_tfin_a_s_ineq - 1) + (a + (s - 1)*A), (start_d_tmax_tfin_a_s - 1) + (a + (s - 1)*A)) = -1;
            % Add the new row and independent term to the corresponding matrix
            A_double_sparse = A_double_sparse + A_tmp;

            % Check whether the initial value meets this constraint
            if not(isempty(x0))
                error = A_tmp((start_d_tmax_tfin_a_s_ineq - 1) + (a + (s - 1)*A),:) * x0 - 0;
                if error > tol
                    disp(['d_tmax_tfin_a_s_nf ineq not met for (a,s) = (', num2str(a), ', ', num2str(s), ') by ', num2str(error)]);
                end
            end
        end
    end

    % tfin(a,s) value
    % tfin(a,s) = tfin(a,s-1) + Td(a,s) + Tw(a,s) + Te(a,s), for all a = 1 to A and s = 1 to S
    parfor a = 1:A
        A_tmp = spalloc(max_eq, length_dv, 1);
        % tfin(a,0) = 0
        A_tmp((start_t_fin_a_s_eq - 1) + a, (start_tfin_a_s - 1) + (a + ((0 + 1) - 1)*A)) = 1;
        Aeq_double_sparse = Aeq_double_sparse + A_tmp;
        for s = 1:S
            A_tmp = spalloc(max_eq, length_dv, (4 + T * 1));
            % - tfin(a,s) + tfin(a,s-1) + Td(a,s) + Tw(a,s) + Te(a,s) = 0
            A_tmp((start_t_fin_a_s_eq - 1 + A) + (a + (s - 1)*A), (start_tfin_a_s - 1) + (a + ((s + 1) - 1)*A)) = -1;
            A_tmp((start_t_fin_a_s_eq - 1 + A) + (a + (s - 1)*A), (start_tfin_a_s - 1) + (a + (((s-1) + 1) - 1)*A)) = 1;
            A_tmp((start_t_fin_a_s_eq - 1 + A) + (a + (s - 1)*A), (start_Td_a_s - 1) + (a + (s - 1)*A)) = 1;
            A_tmp((start_t_fin_a_s_eq - 1 + A) + (a + (s - 1)*A), (start_Tw_a_s - 1) + (a + (s - 1)*A)) = 1;
            A_tmp((start_t_fin_a_s_eq - 1 + A) + (a + (s - 1)*A), (start_Te_a_s - 1) + (a + (s - 1)*A)) = 1;
            Aeq_double_sparse = Aeq_double_sparse + A_tmp;

            % Check whether the initial value meets this constraint
            if not(isempty(x0))
                error = abs(A_tmp((start_t_fin_a_s_eq - 1 + A) + (a + (s - 1)*A),:) * x0 - 0);
                if error > tol
                    disp(['t_fin_a_s eq not met for (a,s) = (', num2str(a), ', ', num2str(s), ') by ', num2str(error)]);
                end
            end
        end
    end

    %% Flight time constraint
    % Ft(a,s) <= Ft(a) - Ft_saf(a), for all a = 1 to A and s = 1 to S
    parfor a = 1:A
        for s = 1:S
            A_tmp = spalloc(max_ineq, length_dv, 1);
            b_tmp = spalloc(max_ineq, 1, 1);
            % Ft(a,s) <= Ft_a - Ft_saf_a
            A_tmp((start_Ft_a_s_ineq - 1) + (a + (s - 1)*A), (start_Ft_a_s - 1) + (a + ((s + 1) - 1)*A)) = 1;
            b_tmp((start_Ft_a_s_ineq - 1) + (a + (s - 1)*A)) = Agent(a).Ft - Agent(a).Ft_saf;
            % Add the new row and independent term to the corresponding matrix
            A_double_sparse = A_double_sparse + A_tmp;
            b_double_sparse = b_double_sparse + b_tmp;

            % Check whether the initial value meets this constraint
            if not(isempty(x0))
                error = A_tmp((start_Ft_a_s_ineq - 1) + (a + (s - 1)*A),:) * x0 - b_tmp((start_Ft_a_s_ineq - 1) + (a + (s - 1)*A),:);
                if error > tol
                    disp(['Ft_a_s ineq not met for (a,s) = (', num2str(a), ', ', num2str(s), ') by ', num2str(error)]);
                end
            end
        end
    end

    %% Flight time value
    % If x(a,R,s-1) == 1 (just recharged):     Ft(a,s) = Td(a,s) + Tw(a,s) + Te(a,s)
    % If x(a,R,s-1) == 0 (not just recharged):
    %       If x(a,R,s) == 1 (recharging):     Ft(a,s) = Ft(a,s-1) + Td(a,s)
    %       If x(a,R,s) == 0 (not recharging): Ft(a,s) = Ft(a,s-1) + Td(a,s) + Tw(a,s) + Te(a,s)
    % The first conditional (if x(a,R,s-1)) affects only to Ft(a,s-1).
    %       In order to add Ft(a,s-1) or not, - Ft(a,s-1)*x(a,R,s-1) is added to the expression.
    % The second conditional (if x(a,R,s)) affects to Tw(a,s) and Te(a,s).
    %       In order to add Tw(a,s) or not, - Tw(a,s)*x(a,R,s) is added to the expression.
    %       In order to add Te(a,s) or not, we just iterate from t = 2 to T, excluding R, which equals 1.
    % Ft(a,s) = Ft(a,s-1) * (1 - x(a,R,s-1)) + Td(a,s) + (Tw(a,s) + Te(a,s)) * (1 - x(a,R,s)), for all a = 1 to A and s = 1 to S
    % Ft(a,s) = Ft(a,s-1) - Ft(a,s-1)*x(a,R,s-1) + Td(a,s) + Tw(a,s) - Tw(a,s)*x(a,R,s) + sum from t = 2 to T of (sum from nf = 1 to N of (Te(t,nf)*nf(t,nf))*x(a,t,s)), for all a = 1 to A and s = 1 to S
    % Note: Te(a,s) variable is not used here instead of its full expression because it is necessary not to add Te(a,s) when a Recharge task is in slot s of agent a.
    parfor a = 1:A
        A_tmp = spalloc(max_eq, length_dv, 1);
        b_tmp = spalloc(max_eq, 1, 1);
        % Ft(a,0) = Ft_0(a)
        A_tmp((start_Ft_a_s_eq - 1) + a, (start_Ft_a_s - 1) + (a + ((0 + 1) - 1)*A)) = 1;
        b_tmp((start_Ft_a_s_eq - 1) + a) = Agent(a).Ft_0;
        Aeq_double_sparse = Aeq_double_sparse + A_tmp;
        beq_double_sparse = beq_double_sparse + b_tmp;

        % Check whether the initial value meets this constraint
        if not(isempty(x0))
            error = abs(A_tmp((start_Ft_a_s_eq - 1) + a,:) * x0 - b_tmp((start_Ft_a_s_eq - 1) + a,:));
            if error > tol
                disp(['Ft_a_s eq not met for a = ', num2str(a), ' by ', num2str(error)]);
            end
        end

        for s = 1:S
            A_tmp = spalloc(max_eq, length_dv, (6 + (T - 1) * N * 1));
            % Ft(a,s) - Ft(a,s-1) + Ft(a,s-1)*x(a,R,s-1) - Td(a,s) - Tw(a,s) + Tw(a,s)*x(a,R,s) - sum from t = 2 to T of (sum from nf = 1 to N of (Te(t,nf)*nf(t,nf))*x(a,t,s)) = 0
            % Ft(a,s-1)*x(a,R,s-1) -> Ftx_a_s1
            A_tmp((start_Ft_a_s_eq - 1 + A) + (a + (s - 1)*A), (start_Ft_a_s - 1) + (a + ((s + 1) - 1)*A)) = 1;
            A_tmp((start_Ft_a_s_eq - 1 + A) + (a + (s - 1)*A), (start_Ft_a_s - 1) + (a + (((s-1) + 1) - 1)*A)) = -1;
            A_tmp((start_Ft_a_s_eq - 1 + A) + (a + (s - 1)*A), (start_Ftx_a_s1 - 1) + (a + (s - 1)*A)) = 1;
            A_tmp((start_Ft_a_s_eq - 1 + A) + (a + (s - 1)*A), (start_Td_a_s - 1) + (a + (s - 1)*A)) = -1;
            A_tmp((start_Ft_a_s_eq - 1 + A) + (a + (s - 1)*A), (start_Tw_a_s - 1) + (a + (s - 1)*A)) = -1;
            A_tmp((start_Ft_a_s_eq - 1 + A) + (a + (s - 1)*A), (start_Twx_a_s - 1) + (a + (s - 1)*A)) = 1;
            for t = 1:T
                if t ~= R
                    for nf = 1:N
                        % sum from t = 2 to T of (sum from nf = 1 to N of (Te(t,nf)*nf(t,nf))*x(a,t,s)) -> sum from t = 2 to T of (sum from nf = 1 to N of (Te(t,nf)*nf(t,nf)*x(a,t,s)))
                        % nf(t,nf)*x(a,t,s) -> nfx_a_nf_t_s
                        A_tmp((start_Ft_a_s_eq - 1 + A) + (a + (s - 1)*A), (start_nfx_a_nf_t_s - 1) + (a + (nf - 1)*A + (t - 1)*A*N + (s - 1)*A*N*T)) = - Te_t_nf(t,nf);
                    end
                end
            end
            Aeq_double_sparse = Aeq_double_sparse + A_tmp;

            % Check whether the initial value meets this constraint
            if not(isempty(x0))
                error = abs(A_tmp((start_Ft_a_s_eq - 1 + A) + (a + (s - 1)*A),:) * x0 - 0);
                if error > tol
                    disp(['Ft_a_s eq not met for (a,s) = (', num2str(a), ', ', num2str(s), ') by ', num2str(error)]);
                end
            end
        end
    end

    %% "Task.N simultaneous Agents" constraint
    % V(t) + na(t) = N(t), for all t = 2 to T if N(t) > 0
    parfor t = 1:T
        if t ~= R
            if Task(t).N > 0
                A_tmp = spalloc(max_eq, length_dv, 2);
                b_tmp = spalloc(max_eq, 1, 1);
                % V(t) + na(t) = N(t)
                A_tmp((start_V_t_eq - 1) + (t - 1), (start_V_t - 1) + t) = 1;
                A_tmp((start_V_t_eq - 1) + (t - 1), (start_na_t - 1) + t) = 1;
                b_tmp((start_V_t_eq - 1) + (t - 1)) = Task(t).N;
                Aeq_double_sparse = Aeq_double_sparse + A_tmp;
                beq_double_sparse = beq_double_sparse + b_tmp;

                % Check whether the initial value meets this constraint
                if not(isempty(x0))
                    error = abs(A_tmp((start_V_t_eq - 1) + (t - 1),:) * x0 - b_tmp((start_V_t_eq - 1) + (t - 1),:));
                    if error > tol
                        disp(['V_t eq not met for t = ', num2str(t), ' by ', num2str(error)]);
                    end
                end
            end
        end
    end

    %% "Recharge tasks shouldn't be assigned to the same Agent in two consecutive slots" constraint
    % x(a,R,s) + x(a,R,s+1) <= 1, for all a = 1 to A and s = 1 to S-1
    parfor a = 1:A
        for s = 1:S-1
            A_tmp = spalloc(max_ineq, length_dv, 2);
            b_tmp = spalloc(max_ineq, 1, 1);
            % x(a,t,s) + x(a,t,s+1) <= 1
            A_tmp((start_recharge_ineq - 1) + (a + (s - 1)*A), (start_x_a_t_s - 1) + (a + ((R + 1) - 1)*A + ((s + 1) - 1)*A*(T+1))) = 1;
            A_tmp((start_recharge_ineq - 1) + (a + (s - 1)*A), (start_x_a_t_s - 1) + (a + ((R + 1) - 1)*A + (((s+1) + 1) - 1)*A*(T+1))) = 1;
            b_tmp((start_recharge_ineq - 1) + (a + (s - 1)*A)) = 1;
            A_double_sparse = A_double_sparse + A_tmp;
            b_double_sparse = b_double_sparse + b_tmp;

            % Check whether the initial value meets this constraint
            if not(isempty(x0))
                error = A_tmp((start_recharge_ineq - 1) + (a + (s - 1)*A),:) * x0 - b_tmp((start_recharge_ineq - 1) + (a + (s - 1)*A),:);
                if error > tol
                    disp(['recharge ineq not met for (a,s) = (', num2str(a), ', ', num2str(s), ') by ', num2str(error)]);
                end
            end
        end
    end

    %% Hardware constraint
    % x(a,t,s) <= H(a,t), for all a = 1 to A, t = 1 to T and s = 1 to S
    parfor a = 1:A
        for t = 1:T
            for s = 1:S
                A_tmp = spalloc(max_ineq, length_dv, 1);
                b_tmp = spalloc(max_ineq, 1, 1);
                % x(a,t,s) <= H(a,t)
                A_tmp((start_hardware_ineq - 1) + (a + (t - 1)*A + (s - 1)*A*T), (start_x_a_t_s - 1) + (a + ((t + 1) - 1)*A + ((s + 1) - 1)*A*(T+1))) = 1;
                b_tmp((start_hardware_ineq - 1) + (a + (t - 1)*A + (s - 1)*A*T)) = H_a_t(a,t);
                A_double_sparse = A_double_sparse + A_tmp;
                b_double_sparse = b_double_sparse + b_tmp;

                % Check whether the initial value meets this constraint
                if not(isempty(x0))
                    error = A_tmp((start_hardware_ineq - 1) + (a + (t - 1)*A + (s - 1)*A*T),:) * x0 - b_tmp((start_hardware_ineq - 1) + (a + (t - 1)*A + (s - 1)*A*T),:);
                    if error > tol
                        disp(['hardware ineq not met for (a,t,s) = (', num2str(a), ', ', num2str(t), ', ', num2str(s), ') by ', num2str(error)]);
                    end
                end
            end
        end
    end

    %% "Max of 1 task per slot" constraint
    % sum from t = 0 to T of (x(a,t,s)) <= 1, for all a = 1 to A and s = 1 to S
    parfor a = 1:A
        for s = 1:S
            A_tmp = spalloc(max_ineq, length_dv, 1 * (T+1));
            b_tmp = spalloc(max_ineq, 1, 1);
            % sum from t = 0 to T of (x(a,t,s)) <= 1
            A_tmp((start_max1task_ineq - 1) + (a + (s - 1)*A), (start_x_a_t_s - 1) + (a + ((0 + 1) - 1)*A + ((s + 1) - 1)*A*(T+1)):A:(start_x_a_t_s - 1) + (a + ((T + 1) - 1)*A + ((s + 1) - 1)*A*(T+1))) = 1;
            b_tmp((start_max1task_ineq - 1) + (a + (s - 1)*A)) = 1;
            A_double_sparse = A_double_sparse + A_tmp;
            b_double_sparse = b_double_sparse + b_tmp;

            % Check whether the initial value meets this constraint
            if not(isempty(x0))
                error = A_tmp((start_max1task_ineq - 1) + (a + (s - 1)*A),:) * x0 - b_tmp((start_max1task_ineq - 1) + (a + (s - 1)*A),:);
                if error > tol
                    disp(['max1task ineq not met for (a,s) = (', num2str(a), ', ', num2str(s), ') by ', num2str(error)]);
                end
            end
        end
    end

    %% Continuity constraint: make it impossible for there to be empty slots in between. But there can be empty slots at the end.
    % sum from t = 1 to T of (x(a,t,s)) - sum from t = 1 to T of (x(a,t,s-1)) <= 0. for all s = 2 to S and a = 1 to A
    parfor a = 1:A
        for s = 2:S
            A_tmp = spalloc(max_ineq, length_dv, 2 * T);
            % sum from t = 1 to T of (x(a,t,s)) - sum from t = 1 to T of (x(a,t,s-1)) <= 0
            A_tmp((start_continuity_ineq - 1) + (a + ((s-1) - 1)*A), (start_x_a_t_s - 1) + (a + ((1 + 1) - 1)*A + ((s + 1) - 1)*A*(T+1)):A:(start_x_a_t_s - 1) + (a + ((T + 1) - 1)*A + ((s + 1) - 1)*A*(T+1))) = 1;
            A_tmp((start_continuity_ineq - 1) + (a + ((s-1) - 1)*A), (start_x_a_t_s - 1) + (a + ((1 + 1) - 1)*A + (((s-1) + 1) - 1)*A*(T+1)):A:(start_x_a_t_s - 1) + (a + ((T + 1) - 1)*A + (((s-1) + 1) - 1)*A*(T+1))) = -1;
            A_double_sparse = A_double_sparse + A_tmp;

            % Check whether the initial value meets this constraint
            if not(isempty(x0))
                error = A_tmp((start_continuity_ineq - 1) + (a + ((s-1) - 1)*A),:) * x0 - 0;
                if error > tol
                    disp(['continuity ineq not met for (a,s) = (', num2str(a), ', ', num2str(s), ') by ', num2str(error)]);
                end
            end
        end
    end

    %% s_used value
    A_tmp = spalloc(max_eq, length_dv, (1 + A * (T+1) * (S+1)));
    b_tmp = spalloc(max_eq, 1, 1);
    % s_used = sum from a = 1 to A of (sum from t = 1 to T of (sum from s = 1 to S of (x(a,t,s))))
    % In order to be able to use Matlab's matrix notation and save us 3 for loops, we can just iterate also on slots 0 and tasks 0 and then subtract A.
    % s_used = sum from a = 1 to A of (sum from t = 0 to T of (sum from s = 0 to S of (x(a,t,s)))) - A
    % - s_used + sum from a = 1 to A of (sum from t = 0 to T of (sum from s = 0 to S of (x(a,t,s)))) = A
    A_tmp((start_s_used_eq - 1) + 1, (start_s_used - 1) + 1) = -1;
    A_tmp((start_s_used_eq - 1) + 1, start_x_a_t_s : start_x_a_t_s + length_x_a_t_s - 1) = 1;
    b_tmp((start_s_used_eq - 1) + 1) = A;
    Aeq_double_sparse = Aeq_double_sparse + A_tmp;
    beq_double_sparse = beq_double_sparse + b_tmp;

    % Check whether the initial value meets this constraint
    if not(isempty(x0))
        error = abs(A_tmp((start_s_used_eq - 1) + 1,:) * x0 - b_tmp((start_s_used_eq - 1) + 1,:));
        if error > tol
            disp('s_used eq not met by ', num2str(error)');
        end
    end

    %% Synchronization constraints
    parfor t = 1:T
        if t ~= R
            % Synchronization constraints could be applied only to tasks that have a specified number of robots
            if Task(t).N ~= 0
                for a1 = 1:A
                    for s1 = 1:S
                        for a2 = 1:A
                            if a1 ~= a2
                                for s2 = 1:S
                                    % "Synchronized tasks must be the same" constraint
                                    % S(t,a1,s1,a2,s2) <= x(a1,t,s1), for all a1,a2 in A, s1,s2 in S and t in [2,T]
                                    % S(t,a1,s1,a2,s2) <= x(a2,t,s2), for all a1,a2 in A, s1,s2 in S and t in [2,T]
                                    A_tmp = spalloc(max_ineq, length_dv, 2);
                                    % S(t,a1,s1,a2,s2) - x(a1,t,s1) <= 0
                                    A_tmp((start_synch_ineq - 1) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_S_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A)) = 1;
                                    A_tmp((start_synch_ineq - 1) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_x_a_t_s - 1) + (a1 + ((t + 1) - 1)*A + ((s1 + 1) - 1)*A*(T+1))) = -1;
                                    A_double_sparse = A_double_sparse + A_tmp;

                                    % Check whether the initial value meets this constraint
                                    if not(isempty(x0))
                                        error = A_tmp((start_synch_ineq - 1) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A),:) * x0 - 0;
                                        if error > tol
                                            disp(['synch ineq not met for (t,a1,s1,a2,s2) = (', num2str(t), ', ', num2str(a1), ', ', num2str(s1), ', ', num2str(a2), ', ', num2str(s2), ') by ', num2str(error)]);
                                        end
                                    end
                                    
                                    A_tmp = spalloc(max_ineq, length_dv, 2);
                                    % S(t,a1,s1,a2,s2) - x(a2,t,s2) <= 0
                                    A_tmp((start_synch_ineq - 1 + 1*((T - 1) * A * S * A * S)) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_S_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A)) = 1;
                                    A_tmp((start_synch_ineq - 1 + 1*((T - 1) * A * S * A * S)) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_x_a_t_s - 1) + (a2 + ((t + 1) - 1)*A + ((s2 + 1) - 1)*A*(T+1))) = -1;
                                    A_double_sparse = A_double_sparse + A_tmp;

                                    % Check whether the initial value meets this constraint
                                    if not(isempty(x0))
                                        error = A_tmp((start_synch_ineq - 1 + 1*((T - 1) * A * S * A * S)) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A),:) * x0 - 0;
                                        if error > tol
                                            disp(['synch ineq 1 not met for (t,a1,s1,a2,s2) = (', num2str(t), ', ', num2str(a1), ', ', num2str(s1), ', ', num2str(a2), ', ', num2str(s2), ') by ', num2str(error)]);
                                        end
                                    end

                                    % Synchronizations time coordination
                                    % tfin(a1,s1)*S(t,a1,s1,a2,s2) = tfin(a2,s2)*S(t,a1,s1,a2,s2), for all t in [2,T], a1,a2 in A, s1,s2 in S, a1 != a2
                                    % tfin(a1,s1)*S(t,a1,s1,a2,s2) -> tfinS(1,t,a1,s1,a2,s2)
                                    % tfin(a2,s2)*S(t,a1,s1,a2,s2) -> tfinS(2,t,a1,s1,a2,s2)
                                    A_tmp = spalloc(max_eq, length_dv, 2);
                                    % tfinS(1,t,a1,s1,a2,s2) - tfinS(2,t,a1,s1,a2,s2) = 0
                                    A_tmp((start_synch_eq - 1 + ((T - 1) * A * S)) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_tfinS_t_a1_s1_a2_s2 - 1) + (1 + ((t - 1) - 1)*2 + (a1 - 1)*2*(T-1) + (s1 - 1)*2*(T-1)*A + (a2 - 1)*2*(T-1)*A*S + (s2 - 1)*2*(T-1)*A*S*A)) = 1;
                                    A_tmp((start_synch_eq - 1 + ((T - 1) * A * S)) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_tfinS_t_a1_s1_a2_s2 - 1) + (2 + ((t - 1) - 1)*2 + (a1 - 1)*2*(T-1) + (s1 - 1)*2*(T-1)*A + (a2 - 1)*2*(T-1)*A*S + (s2 - 1)*2*(T-1)*A*S*A)) = -1;
                                    Aeq_double_sparse = Aeq_double_sparse + A_tmp;
                                    
                                    % Check whether the initial value meets this constraint
                                    if not(isempty(x0))
                                        error = abs(A_tmp((start_synch_eq - 1 + ((T - 1) * A * S)) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A),:) * x0 - 0);
                                        if error > tol
                                            disp(['synch eq not met for (t,a1,s1,a2,s2) = (', num2str(t), ', ', num2str(a1), ', ', num2str(s1), ', ', num2str(a2), ', ', num2str(s2), ') by ', num2str(error)]);
                                        end
                                    end

                                    % Add constraints to force the value of the tfinS(i,t,a1,s1,a2,s2) aux variable to be the same as its original hight order terms.
                                    % tfin(a1,s1)*S(t,a1,s1,a2,s2) -> tfinS(1,t,a1,s1,a2,s2)
                                    A_tmp = spalloc(max_ineq, length_dv, 2);
                                    % S(t,a1,s1,a2,s2)*tfin_min - tfinS(1,t,a1,s1,a2,s2) <= 0
                                    A_tmp((start_synch_ineq - 1 + 2*((T - 1) * A * S * A * S)) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_S_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A)) = tfin_min;
                                    A_tmp((start_synch_ineq - 1 + 2*((T - 1) * A * S * A * S)) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_tfinS_t_a1_s1_a2_s2 - 1) + (1 + ((t - 1) - 1)*2 + (a1 - 1)*2*(T-1) + (s1 - 1)*2*(T-1)*A + (a2 - 1)*2*(T-1)*A*S + (s2 - 1)*2*(T-1)*A*S*A)) = -1;
                                    A_double_sparse = A_double_sparse + A_tmp;

                                    % Check whether the initial value meets this constraint
                                    if not(isempty(x0))
                                        error = A_tmp((start_synch_eq - 1 + ((T - 1) * A * S)) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A),:) * x0 - 0;
                                        if error > tol
                                            disp(['synch ineq 2 not met for (t,a1,s1,a2,s2) = (', num2str(t), ', ', num2str(a1), ', ', num2str(s1), ', ', num2str(a2), ', ', num2str(s2), ') by ', num2str(error)]);
                                        end
                                    end
                                    
                                    A_tmp = spalloc(max_ineq, length_dv, 2);
                                    % tfinS(1,t,a1,s1,a2,s2) - S(t,a1,s1,a2,s2)*tfin_max <= 0
                                    A_tmp((start_synch_ineq - 1 + 3*((T - 1) * A * S * A * S)) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_tfinS_t_a1_s1_a2_s2 - 1) + (1 + ((t - 1) - 1)*2 + (a1 - 1)*2*(T-1) + (s1 - 1)*2*(T-1)*A + (a2 - 1)*2*(T-1)*A*S + (s2 - 1)*2*(T-1)*A*S*A)) = 1;
                                    A_tmp((start_synch_ineq - 1 + 3*((T - 1) * A * S * A * S)) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_S_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A)) = - tfin_max;
                                    A_double_sparse = A_double_sparse + A_tmp;

                                    % Check whether the initial value meets this constraint
                                    if not(isempty(x0))
                                        error = A_tmp((start_synch_ineq - 1 + 3*((T - 1) * A * S * A * S)) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A),:) * x0 - 0;
                                        if error > tol
                                            disp(['synch ineq 3 not met for (t,a1,s1,a2,s2) = (', num2str(t), ', ', num2str(a1), ', ', num2str(s1), ', ', num2str(a2), ', ', num2str(s2), ') by ', num2str(error)]);
                                        end
                                    end
                                    
                                    A_tmp = spalloc(max_ineq, length_dv, 3);
                                    b_tmp = spalloc(max_ineq, 1, 1);
                                    % tfin(a1,s1) + tfin_max*S(t,a1,s1,a2,s2) - tfinS(1,t,a1,s1,a2,s2) <= tfin_max
                                    A_tmp((start_synch_ineq - 1 + 4*((T - 1) * A * S * A * S)) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_tfin_a_s - 1) + (a1 + ((s1 + 1) - 1)*A)) = 1;
                                    A_tmp((start_synch_ineq - 1 + 4*((T - 1) * A * S * A * S)) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_S_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A)) = tfin_max;
                                    A_tmp((start_synch_ineq - 1 + 4*((T - 1) * A * S * A * S)) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_tfinS_t_a1_s1_a2_s2 - 1) + (1 + ((t - 1) - 1)*2 + (a1 - 1)*2*(T-1) + (s1 - 1)*2*(T-1)*A + (a2 - 1)*2*(T-1)*A*S + (s2 - 1)*2*(T-1)*A*S*A)) = -1;
                                    b_tmp((start_synch_ineq - 1 + 4*((T - 1) * A * S * A * S)) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A)) = tfin_max;
                                    A_double_sparse = A_double_sparse + A_tmp;
                                    b_double_sparse = b_double_sparse + b_tmp;

                                    % Check whether the initial value meets this constraint
                                    if not(isempty(x0))
                                        error = A_tmp((start_synch_ineq - 1 + 4*((T - 1) * A * S * A * S)) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A),:) * x0 - b_tmp((start_synch_ineq - 1 + 4*((T - 1) * A * S * A * S)) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A),:);
                                        if error > tol
                                            disp(['synch ineq 4 not met for (t,a1,s1,a2,s2) = (', num2str(t), ', ', num2str(a1), ', ', num2str(s1), ', ', num2str(a2), ', ', num2str(s2), ') by ', num2str(error)]);
                                        end
                                    end

                                    A_tmp = spalloc(max_ineq, length_dv, 3);
                                    b_tmp = spalloc(max_ineq, 1, 1);
                                    % tfinS(1,t,a1,s1,a2,s2) - tfin(a1,s1) - tfin_min*S(t,a1,s1,a2,s2) <= - tfin_min
                                    A_tmp((start_synch_ineq - 1 + 5*((T - 1) * A * S * A * S)) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_tfinS_t_a1_s1_a2_s2 - 1) + (1 + ((t - 1) - 1)*2 + (a1 - 1)*2*(T-1) + (s1 - 1)*2*(T-1)*A + (a2 - 1)*2*(T-1)*A*S + (s2 - 1)*2*(T-1)*A*S*A)) = 1;
                                    A_tmp((start_synch_ineq - 1 + 5*((T - 1) * A * S * A * S)) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_tfin_a_s - 1) + (a1 + ((s1 + 1) - 1)*A)) = -1;
                                    A_tmp((start_synch_ineq - 1 + 5*((T - 1) * A * S * A * S)) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_S_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A)) = - tfin_min;
                                    b_tmp((start_synch_ineq - 1 + 5*((T - 1) * A * S * A * S)) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A)) = - tfin_min;
                                    A_double_sparse = A_double_sparse + A_tmp;
                                    b_double_sparse = b_double_sparse + b_tmp;

                                    % Check whether the initial value meets this constraint
                                    if not(isempty(x0))
                                        error = A_tmp((start_synch_ineq - 1 + 5*((T - 1) * A * S * A * S)) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A),:) * x0 - b_tmp((start_synch_ineq - 1 + 5*((T - 1) * A * S * A * S)) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A),:);
                                        if error > tol
                                            disp(['synch ineq 5 not met for (t,a1,s1,a2,s2) = (', num2str(t), ', ', num2str(a1), ', ', num2str(s1), ', ', num2str(a2), ', ', num2str(s2), ') by ', num2str(error)]);
                                        end
                                    end
                                    
                                    % tfin(a2,s2)*S(t,a1,s1,a2,s2) -> tfinS(2,t,a1,s1,a2,s2)
                                    A_tmp = spalloc(max_ineq, length_dv, 2);
                                    % S(t,a1,s1,a2,s2)*tfin_min - tfinS(2,t,a1,s1,a2,s2) <= 0
                                    A_tmp((start_synch_ineq - 1 + 6*((T - 1) * A * S * A * S)) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_S_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A)) = tfin_min;
                                    A_tmp((start_synch_ineq - 1 + 6*((T - 1) * A * S * A * S)) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_tfinS_t_a1_s1_a2_s2 - 1) + (2 + ((t - 1) - 1)*2 + (a1 - 1)*2*(T-1) + (s1 - 1)*2*(T-1)*A + (a2 - 1)*2*(T-1)*A*S + (s2 - 1)*2*(T-1)*A*S*A)) = -1;
                                    A_double_sparse = A_double_sparse + A_tmp;

                                    % Check whether the initial value meets this constraint
                                    if not(isempty(x0))
                                        error = A_tmp((start_synch_ineq - 1 + 6*((T - 1) * A * S * A * S)) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A),:) * x0 - 0;
                                        if error > tol
                                            disp(['synch ineq 6 not met for (t,a1,s1,a2,s2) = (', num2str(t), ', ', num2str(a1), ', ', num2str(s1), ', ', num2str(a2), ', ', num2str(s2), ') by ', num2str(error)]);
                                        end
                                    end
                                    
                                    A_tmp = spalloc(max_ineq, length_dv, 2);
                                    % tfinS(2,t,a1,s1,a2,s2) - S(t,a1,s1,a2,s2)*tfin_max <= 0
                                    A_tmp((start_synch_ineq - 1 + 7*((T - 1) * A * S * A * S)) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_tfinS_t_a1_s1_a2_s2 - 1) + (2 + ((t - 1) - 1)*2 + (a1 - 1)*2*(T-1) + (s1 - 1)*2*(T-1)*A + (a2 - 1)*2*(T-1)*A*S + (s2 - 1)*2*(T-1)*A*S*A)) = 1;
                                    A_tmp((start_synch_ineq - 1 + 7*((T - 1) * A * S * A * S)) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_S_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A)) = - tfin_max;
                                    A_double_sparse = A_double_sparse + A_tmp;

                                    % Check whether the initial value meets this constraint
                                    if not(isempty(x0))
                                    error = A_tmp((start_synch_ineq - 1 + 7*((T - 1) * A * S * A * S)) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A),:) * x0 - 0;
                                        if error > tol
                                            disp(['synch ineq 7 not met for (t,a1,s1,a2,s2) = (', num2str(t), ', ', num2str(a1), ', ', num2str(s1), ', ', num2str(a2), ', ', num2str(s2), ') by ', num2str(error)]);
                                        end
                                    end
                                    
                                    A_tmp = spalloc(max_ineq, length_dv, 3);
                                    b_tmp = spalloc(max_ineq, 1, 1);
                                    % tfin(a2,s2) + tfin_max*S(t,a1,s1,a2,s2) - tfinS(2,t,a1,s1,a2,s2) <= tfin_max
                                    A_tmp((start_synch_ineq - 1 + 8*((T - 1) * A * S * A * S)) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_tfin_a_s - 1) + (a2 + ((s2 + 1) - 1)*A)) = 1;
                                    A_tmp((start_synch_ineq - 1 + 8*((T - 1) * A * S * A * S)) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_S_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A)) = tfin_max;
                                    A_tmp((start_synch_ineq - 1 + 8*((T - 1) * A * S * A * S)) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_tfinS_t_a1_s1_a2_s2 - 1) + (2 + ((t - 1) - 1)*2 + (a1 - 1)*2*(T-1) + (s1 - 1)*2*(T-1)*A + (a2 - 1)*2*(T-1)*A*S + (s2 - 1)*2*(T-1)*A*S*A)) = -1;
                                    b_tmp((start_synch_ineq - 1 + 8*((T - 1) * A * S * A * S)) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A)) = tfin_max;
                                    A_double_sparse = A_double_sparse + A_tmp;
                                    b_double_sparse = b_double_sparse + b_tmp;

                                    % Check whether the initial value meets this constraint
                                    if not(isempty(x0))
                                    error = A_tmp((start_synch_ineq - 1 + 8*((T - 1) * A * S * A * S)) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A),:) * x0 - b_tmp((start_synch_ineq - 1 + 8*((T - 1) * A * S * A * S)) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A),:);
                                        if error > tol
                                            disp(['synch ineq 8 not met for (t,a1,s1,a2,s2) = (', num2str(t), ', ', num2str(a1), ', ', num2str(s1), ', ', num2str(a2), ', ', num2str(s2), ') by ', num2str(error)]);
                                        end
                                    end
                                    
                                    A_tmp = spalloc(max_ineq, length_dv, 3);
                                    b_tmp = spalloc(max_ineq, 1, 1);
                                    % tfinS(2,t,a1,s1,a2,s2) - tfin(a2,s2) - tfin_min*S(t,a1,s1,a2,s2) <= - tfin_min
                                    A_tmp((start_synch_ineq - 1 + 9*((T - 1) * A * S * A * S)) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_tfinS_t_a1_s1_a2_s2 - 1) + (2 + ((t - 1) - 1)*2 + (a1 - 1)*2*(T-1) + (s1 - 1)*2*(T-1)*A + (a2 - 1)*2*(T-1)*A*S + (s2 - 1)*2*(T-1)*A*S*A)) = 1;
                                    A_tmp((start_synch_ineq - 1 + 9*((T - 1) * A * S * A * S)) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_tfin_a_s - 1) + (a2 + ((s2 + 1) - 1)*A)) = -1;
                                    A_tmp((start_synch_ineq - 1 + 9*((T - 1) * A * S * A * S)) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_S_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A)) = - tfin_min;
                                    b_tmp((start_synch_ineq - 1 + 9*((T - 1) * A * S * A * S)) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A)) = - tfin_min;
                                    A_double_sparse = A_double_sparse + A_tmp;
                                    b_double_sparse = b_double_sparse + b_tmp;

                                    % Check whether the initial value meets this constraint
                                    if not(isempty(x0))
                                    error = A_tmp((start_synch_ineq - 1 + 9*((T - 1) * A * S * A * S)) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A),:) * x0 - b_tmp((start_synch_ineq - 1 + 9*((T - 1) * A * S * A * S)) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A),:);
                                        if error > tol
                                            disp(['synch ineq 9 not met for (t,a1,s1,a2,s2) = (', num2str(t), ', ', num2str(a1), ', ', num2str(s1), ', ', num2str(a2), ', ', num2str(s2), ') by ', num2str(error)]);
                                        end
                                    end

                                    % All synchronizations must be bidirectional
                                    % S(t, a1, s1, a2, s2) = S(t, a2, s2, a1, s1)
                                    % S(t, a1, s1, a2, s2) - S(t, a2, s2, a1, s1) = 0
                                    A_tmp = spalloc(max_eq, length_dv, 2);
                                    A_tmp((start_synch_eq - 1 + ((T - 1) * A * S) + ((T - 1) * A * S * A * S)) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_S_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A)) = 1;
                                    A_tmp((start_synch_eq - 1 + ((T - 1) * A * S) + ((T - 1) * A * S * A * S)) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_S_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a2 - 1)*(T-1) + (s2 - 1)*(T-1)*A + (a1 - 1)*(T-1)*A*S + (s1 - 1)*(T-1)*A*S*A)) = -1;
                                    Aeq_double_sparse = Aeq_double_sparse + A_tmp;

                                    % Check whether the initial value meets this constraint
                                    if not(isempty(x0))
                                    error = abs(A_tmp((start_synch_eq - 1 + ((T - 1) * A * S) + ((T - 1) * A * S * A * S)) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A),:) * x0 - 0);
                                        if error > tol
                                            disp(['synch eq 1 not met for (t,a1,s1,a2,s2) = (', num2str(t), ', ', num2str(a1), ', ', num2str(s1), ', ', num2str(a2), ', ', num2str(s2), ') by ', num2str(error)]);
                                        end
                                    end
                                end
                            end
                        end
                        
                        % Number of Synchronizations
                        % ns(t,a1,s1) = sum from a2 = 1 to A of (sum from s2 = 1 to S of (S(t,a1,s1,a2,s2))), for all t = 2 to T, a1 in A, s1 in S, a1 != a2
                        % ns(t,a1,s1) = (na(t) - 1) * x(a1,t,s1) = na(t) * x(a1,t,s1) - x(a1,t,s1)
                        A_tmp = spalloc(max_eq, length_dv, (2 + (A * S) * 1));
                        % - na(t) * x(a1,t,s1) + x(a1,t,s1) + sum from a2 = 1 to A of (sum from s2 = 1 to S of (S(t,a1,s1,a2,s2))) = 0
                        % na(t) * x(a1,t,s1) -> nax_a_t_s
                        A_tmp((start_synch_eq - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A), (start_nax_a_t_s - 1) + (a1 + (t - 1)*A + (s1 - 1)*A*T)) = -1;
                        A_tmp((start_synch_eq - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A), (start_x_a_t_s - 1) + (a1 + ((t + 1) - 1)*A + ((s1 + 1) - 1)*A*(T+1))) = 1;
                        for a2 = 1:A
                            if a1 ~= a2
                                for s2 = 1:S
                                    A_tmp((start_synch_eq - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A), (start_S_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A)) = 1;
                                end
                            end
                        end
                        Aeq_double_sparse = Aeq_double_sparse + A_tmp;

                        % Check whether the initial value meets this constraint
                        if not(isempty(x0))
                        error = abs(A_tmp((start_synch_eq - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A),:) * x0 - 0);
                            if error > tol
                                disp(['synch eq not met for (t,a1,s1) = (', num2str(t), ', ', num2str(a1), ', ', num2str(s1), ') by ', num2str(error)]);
                            end
                        end
                    end
                end
            end
        end
    end

    %% Relays constraints
    parfor t = 1:T
        if t ~= R
            % Relay constraints could be applied only to relayable tasks
            if Task(t).Relayability == 1
                for a1 = 1:A
                    for s1 = 1:S
                        for a2 = 1:A
                            for s2 = 1:S
                                if not(a1 == a2 && s1 == s2)
                                    % "Relayed tasks must be the same" constraint
                                    % R(t,a1,s1,a2,s2) <= x(a1,t,s1), for all a1,a2 in A, s1,s2 in S and t in [2,T]
                                    % R(t,a1,s1,a2,s2) <= x(a2,t,s2), for all a1,a2 in A, s1,s2 in S and t in [2,T]
                                    A_tmp = spalloc(max_ineq, length_dv, 2);
                                    % R(t,a1,s1,a2,s2) - x(a1,t,s1) <= 0
                                    A_tmp((start_relays_ineq - 1 + 2*(T-1)*A*S) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_R_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A)) = 1;
                                    A_tmp((start_relays_ineq - 1 + 2*(T-1)*A*S) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_x_a_t_s - 1) + (a1 + ((t + 1) - 1)*A + ((s1 + 1) - 1)*A*(T+1))) = -1;
                                    A_double_sparse = A_double_sparse + A_tmp;

                                    % Check whether the initial value meets this constraint
                                    if not(isempty(x0))
                                    error = A_tmp((start_relays_ineq - 1 + 2*(T-1)*A*S) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A),:) * x0 - 0;
                                        if error > tol
                                            disp(['relays ineq not met for (t,a1,s1,a2,s2) = (', num2str(t), ', ', num2str(a1), ', ', num2str(s1), ', ', num2str(a2), ', ', num2str(s2), ') by ', num2str(error)]);
                                        end
                                    end
                                    
                                    A_tmp = spalloc(max_ineq, length_dv, 2);
                                    % R(t,a1,s1,a2,s2) - x(a2,t,s2) <= 0
                                    A_tmp((start_relays_ineq - 1 + 2*(T-1)*A*S + 1*(T-1)*A*S*A*S) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_R_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A)) = 1;
                                    A_tmp((start_relays_ineq - 1 + 2*(T-1)*A*S + 1*(T-1)*A*S*A*S) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_x_a_t_s - 1) + (a2 + ((t + 1) - 1)*A + ((s2 + 1) - 1)*A*(T+1))) = -1;
                                    A_double_sparse = A_double_sparse + A_tmp;
                                    
                                    % Check whether the initial value meets this constraint
                                    if not(isempty(x0))
                                    error = A_tmp((start_relays_ineq - 1 + 2*(T-1)*A*S + 1*(T-1)*A*S*A*S) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A),:) * x0 - 0;
                                        if error > tol
                                            disp(['relays ineq 1 not met for (t,a1,s1,a2,s2) = (', num2str(t), ', ', num2str(a1), ', ', num2str(s1), ', ', num2str(a2), ', ', num2str(s2), ') by ', num2str(error)]);
                                        end
                                    end

                                    % Relays time coordination
                                    % tfin(a1,s1)*R(t,a1,s1,a2,s2) = (tfin(a2,s2) - Te(a2,s2))*R(t,a1,s1,a2,s2), for all t in [2,T], a1,a2 in A, s1,s2 in S, not(a1 == a2 && s1 == s2)
                                    % tfin(a1,s1)*R(t,a1,s1,a2,s2) -> tfinR(1,t,a1,s1,a2,s2)
                                    % tfin(a2,s2)*R(t,a1,s1,a2,s2) -> tfinR(2,t,a1,s1,a2,s2)
                                    % Te(a2,s2)*R(t,a1,s1,a2,s2)   -> TeR(t,a1,s1,a2,s2)
                                    A_tmp = spalloc(max_eq, length_dv, 3);
                                    % tfinR(1,t,a1,s1,a2,s2) - tfinR(2,t,a1,s1,a2,s2) + TeR(t,a1,s1,a2,s2) = 0
                                    A_tmp((start_relays_eq - 1 + (T - 1)) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_tfinR_t_a1_s1_a2_s2 - 1) + (1 + ((t - 1) - 1)*2 + (a1 - 1)*2*(T-1) + (s1 - 1)*2*(T-1)*A + (a2 - 1)*2*(T-1)*A*S + (s2 - 1)*2*(T-1)*A*S*A)) = 1;
                                    A_tmp((start_relays_eq - 1 + (T - 1)) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_tfinR_t_a1_s1_a2_s2 - 1) + (2 + ((t - 1) - 1)*2 + (a1 - 1)*2*(T-1) + (s1 - 1)*2*(T-1)*A + (a2 - 1)*2*(T-1)*A*S + (s2 - 1)*2*(T-1)*A*S*A)) = -1;
                                    A_tmp((start_relays_eq - 1 + (T - 1)) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_TeR_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A)) = 1;
                                    Aeq_double_sparse = Aeq_double_sparse + A_tmp;

                                    % Check whether the initial value meets this constraint
                                    if not(isempty(x0))
                                    error = abs(A_tmp((start_relays_eq - 1 + (T - 1)) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A),:) * x0 - 0);
                                        if error > tol
                                            disp(['relays eq not met for (t,a1,s1,a2,s2) = (', num2str(t), ', ', num2str(a1), ', ', num2str(s1), ', ', num2str(a2), ', ', num2str(s2), ') by ', num2str(error)]);
                                        end
                                    end

                                    % Add constraints to force the value of the tfinR(i,t,a1,s1,a2,s2) and TeR(t,a1,s1,a2,s2) aux variables to be the same as their original hight order terms.
                                    % tfin(a1,s1)*R(t,a1,s1,a2,s2) -> tfinR(1,t,a1,s1,a2,s2)
                                    A_tmp = spalloc(max_ineq, length_dv, 2);
                                    % R(t,a1,s1,a2,s2)*tfin_min - tfinR(t,a1,s1,a2,s2) <= 0
                                    A_tmp((start_relays_ineq - 1 + 2*(T-1)*A*S + 2*(T-1)*A*S*A*S) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_R_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A)) = tfin_min;
                                    A_tmp((start_relays_ineq - 1 + 2*(T-1)*A*S + 2*(T-1)*A*S*A*S) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_tfinR_t_a1_s1_a2_s2 - 1) + (1 + ((t - 1) - 1)*2 + (a1 - 1)*2*(T-1) + (s1 - 1)*2*(T-1)*A + (a2 - 1)*2*(T-1)*A*S + (s2 - 1)*2*(T-1)*A*S*A)) = -1;
                                    % Add the new row and independent term to the corresponding matrix
                                    A_double_sparse = A_double_sparse + A_tmp;

                                    % Check whether the initial value meets this constraint
                                    if not(isempty(x0))
                                    error = A_tmp((start_relays_ineq - 1 + 2*(T-1)*A*S + 2*(T-1)*A*S*A*S) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A),:) * x0 - 0;
                                        if error > tol
                                            disp(['relays ineq 2 not met for (t,a1,s1,a2,s2) = (', num2str(t), ', ', num2str(a1), ', ', num2str(s1), ', ', num2str(a2), ', ', num2str(s2), ') by ', num2str(error)]);
                                        end
                                    end
                                    
                                    A_tmp = spalloc(max_ineq, length_dv, 2);
                                    % tfinR(t,a1,s1,a2,s2) - R(t,a1,s1,a2,s2)*tfin_max <= 0
                                    A_tmp((start_relays_ineq - 1 + 2*(T-1)*A*S + 3*(T-1)*A*S*A*S) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_tfinR_t_a1_s1_a2_s2 - 1) + (1 + ((t - 1) - 1)*2 + (a1 - 1)*2*(T-1) + (s1 - 1)*2*(T-1)*A + (a2 - 1)*2*(T-1)*A*S + (s2 - 1)*2*(T-1)*A*S*A)) = 1;
                                    A_tmp((start_relays_ineq - 1 + 2*(T-1)*A*S + 3*(T-1)*A*S*A*S) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_R_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A)) = - tfin_max;
                                    A_double_sparse = A_double_sparse + A_tmp;

                                    % Check whether the initial value meets this constraint
                                    if not(isempty(x0))
                                    error = A_tmp((start_relays_ineq - 1 + 2*(T-1)*A*S + 3*(T-1)*A*S*A*S) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A),:) * x0 - 0;
                                        if error > tol
                                            disp(['relays ineq 3 not met for (t,a1,s1,a2,s2) = (', num2str(t), ', ', num2str(a1), ', ', num2str(s1), ', ', num2str(a2), ', ', num2str(s2), ') by ', num2str(error)]);
                                        end
                                    end
                                    
                                    A_tmp = spalloc(max_ineq, length_dv, 3);
                                    b_tmp = spalloc(max_ineq, 1, 1);
                                    % tfin(a1,s1) + tfin_max*R(t,a1,s1,a2,s2) - tfinR(t,a1,s1,a2,s2) <= tfin_max
                                    A_tmp((start_relays_ineq - 1 + 2*(T-1)*A*S + 4*(T-1)*A*S*A*S) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_tfin_a_s - 1) + (a1 + ((s1 + 1) - 1)*A)) = 1;
                                    A_tmp((start_relays_ineq - 1 + 2*(T-1)*A*S + 4*(T-1)*A*S*A*S) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_R_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A)) =  tfin_max;
                                    A_tmp((start_relays_ineq - 1 + 2*(T-1)*A*S + 4*(T-1)*A*S*A*S) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_tfinR_t_a1_s1_a2_s2 - 1) + (1 + ((t - 1) - 1)*2 + (a1 - 1)*2*(T-1) + (s1 - 1)*2*(T-1)*A + (a2 - 1)*2*(T-1)*A*S + (s2 - 1)*2*(T-1)*A*S*A)) = -1;
                                    b_tmp((start_relays_ineq - 1 + 2*(T-1)*A*S + 4*(T-1)*A*S*A*S) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A)) = tfin_max;
                                    A_double_sparse = A_double_sparse + A_tmp;
                                    b_double_sparse = b_double_sparse + b_tmp;

                                    % Check whether the initial value meets this constraint
                                    if not(isempty(x0))
                                    error = A_tmp((start_relays_ineq - 1 + 2*(T-1)*A*S + 4*(T-1)*A*S*A*S) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A),:) * x0 - b_tmp((start_relays_ineq - 1 + 2*(T-1)*A*S + 4*(T-1)*A*S*A*S) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A),:);
                                        if error > tol
                                            disp(['relays ineq 4 not met for (t,a1,s1,a2,s2) = (', num2str(t), ', ', num2str(a1), ', ', num2str(s1), ', ', num2str(a2), ', ', num2str(s2), ') by ', num2str(error)]);
                                        end
                                    end
                                    
                                    A_tmp = spalloc(max_ineq, length_dv, 3);
                                    b_tmp = spalloc(max_ineq, 1, 1);
                                    % tfinR(t,a1,s1,a2,s2) - tfin(a1,s1) - tfin_min*R(t,a1,s1,a2,s2) <= - tfin_min
                                    A_tmp((start_relays_ineq - 1 + 2*(T-1)*A*S + 5*(T-1)*A*S*A*S) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_tfinR_t_a1_s1_a2_s2 - 1) + (1 + ((t - 1) - 1)*2 + (a1 - 1)*2*(T-1) + (s1 - 1)*2*(T-1)*A + (a2 - 1)*2*(T-1)*A*S + (s2 - 1)*2*(T-1)*A*S*A)) = 1;
                                    A_tmp((start_relays_ineq - 1 + 2*(T-1)*A*S + 5*(T-1)*A*S*A*S) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_tfin_a_s - 1) + (a1 + ((s1 + 1) - 1)*A)) = -1;
                                    A_tmp((start_relays_ineq - 1 + 2*(T-1)*A*S + 5*(T-1)*A*S*A*S) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_R_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A)) = - tfin_min;
                                    b_tmp((start_relays_ineq - 1 + 2*(T-1)*A*S + 5*(T-1)*A*S*A*S) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A)) = - tfin_min;
                                    A_double_sparse = A_double_sparse + A_tmp;
                                    b_double_sparse = b_double_sparse + b_tmp;

                                    % Check whether the initial value meets this constraint
                                    if not(isempty(x0))
                                    error = A_tmp((start_relays_ineq - 1 + 2*(T-1)*A*S + 5*(T-1)*A*S*A*S) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A),:) * x0 - b_tmp((start_relays_ineq - 1 + 2*(T-1)*A*S + 5*(T-1)*A*S*A*S) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A),:);
                                        if error > tol
                                            disp(['relays ineq 5 not met for (t,a1,s1,a2,s2) = (', num2str(t), ', ', num2str(a1), ', ', num2str(s1), ', ', num2str(a2), ', ', num2str(s2), ') by ', num2str(error)]);
                                        end
                                    end

                                    % tfin(a2,s2)*R(t,a1,s1,a2,s2) -> tfinR(2,t,a1,s1,a2,s2)
                                    A_tmp = spalloc(max_ineq, length_dv, 2);
                                    % R(t,a1,s1,a2,s2)*tfin_min - tfinR(2,t,a1,s1,a2,s2) <= 0
                                    A_tmp((start_relays_ineq - 1 + 2*(T-1)*A*S + 6*(T-1)*A*S*A*S) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_R_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A)) = tfin_min;
                                    A_tmp((start_relays_ineq - 1 + 2*(T-1)*A*S + 6*(T-1)*A*S*A*S) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_tfinR_t_a1_s1_a2_s2 - 1) + (2 + ((t - 1) - 1)*2 + (a1 - 1)*2*(T-1) + (s1 - 1)*2*(T-1)*A + (a2 - 1)*2*(T-1)*A*S + (s2 - 1)*2*(T-1)*A*S*A)) = -1;
                                    A_double_sparse = A_double_sparse + A_tmp;

                                    % Check whether the initial value meets this constraint
                                    if not(isempty(x0))
                                    error = A_tmp((start_relays_ineq - 1 + 2*(T-1)*A*S + 6*(T-1)*A*S*A*S) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A),:) * x0 - 0;
                                        if error > tol
                                            disp(['relays ineq 6 not met for (t,a1,s1,a2,s2) = (', num2str(t), ', ', num2str(a1), ', ', num2str(s1), ', ', num2str(a2), ', ', num2str(s2), ') by ', num2str(error)]);
                                        end
                                    end
                                    
                                    A_tmp = spalloc(max_ineq, length_dv, 2);
                                    % tfinR(2,t,a1,s1,a2,s2) - R(t,a1,s1,a2,s2)*tfin_max <= 0
                                    A_tmp((start_relays_ineq - 1 + 2*(T-1)*A*S + 7*(T-1)*A*S*A*S) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_tfinR_t_a1_s1_a2_s2 - 1) + (2 + ((t - 1) - 1)*2 + (a1 - 1)*2*(T-1) + (s1 - 1)*2*(T-1)*A + (a2 - 1)*2*(T-1)*A*S + (s2 - 1)*2*(T-1)*A*S*A)) = 1;
                                    A_tmp((start_relays_ineq - 1 + 2*(T-1)*A*S + 7*(T-1)*A*S*A*S) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_R_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A)) = - tfin_max;
                                    A_double_sparse = A_double_sparse + A_tmp;

                                    % Check whether the initial value meets this constraint
                                    if not(isempty(x0))
                                    error = A_tmp((start_relays_ineq - 1 + 2*(T-1)*A*S + 7*(T-1)*A*S*A*S) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A),:) * x0 - 0;
                                        if error > tol
                                            disp(['relays ineq 7 not met for (t,a1,s1,a2,s2) = (', num2str(t), ', ', num2str(a1), ', ', num2str(s1), ', ', num2str(a2), ', ', num2str(s2), ') by ', num2str(error)]);
                                        end
                                    end
                                    
                                    A_tmp = spalloc(max_ineq, length_dv, 3);
                                    b_tmp = spalloc(max_ineq, 1, 1);
                                    % tfin(a2,s2) + tfin_max*R(t,a1,s1,a2,s2) - tfinR(2,t,a1,s1,a2,s2) <= tfin_max
                                    A_tmp((start_relays_ineq - 1 + 2*(T-1)*A*S + 8*(T-1)*A*S*A*S) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_tfin_a_s - 1) + (a2 + ((s2 + 1) - 1)*A)) = 1;
                                    A_tmp((start_relays_ineq - 1 + 2*(T-1)*A*S + 8*(T-1)*A*S*A*S) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_R_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A)) = tfin_max;
                                    A_tmp((start_relays_ineq - 1 + 2*(T-1)*A*S + 8*(T-1)*A*S*A*S) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_tfinR_t_a1_s1_a2_s2 - 1) + (2 + ((t - 1) - 1)*2 + (a1 - 1)*2*(T-1) + (s1 - 1)*2*(T-1)*A + (a2 - 1)*2*(T-1)*A*S + (s2 - 1)*2*(T-1)*A*S*A)) = -1;
                                    b_tmp((start_relays_ineq - 1 + 2*(T-1)*A*S + 8*(T-1)*A*S*A*S) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A)) = tfin_max;
                                    A_double_sparse = A_double_sparse + A_tmp;
                                    b_double_sparse = b_double_sparse + b_tmp;

                                    % Check whether the initial value meets this constraint
                                    if not(isempty(x0))
                                    error = A_tmp((start_relays_ineq - 1 + 2*(T-1)*A*S + 8*(T-1)*A*S*A*S) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A),:) * x0 - b_tmp((start_relays_ineq - 1 + 2*(T-1)*A*S + 8*(T-1)*A*S*A*S) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A),:);
                                        if error > tol
                                            disp(['relays ineq 8 not met for (t,a1,s1,a2,s2) = (', num2str(t), ', ', num2str(a1), ', ', num2str(s1), ', ', num2str(a2), ', ', num2str(s2), ') by ', num2str(error)]);
                                        end
                                    end
                                    
                                    A_tmp = spalloc(max_ineq, length_dv, 3);
                                    b_tmp = spalloc(max_ineq, 1, 1);
                                    % tfinR(2,t,a1,s1,a2,s2) - tfin(a2,s2) - tfin_min*R(t,a1,s1,a2,s2) <= - tfin_min
                                    A_tmp((start_relays_ineq - 1 + 2*(T-1)*A*S + 9*(T-1)*A*S*A*S) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_tfinR_t_a1_s1_a2_s2 - 1) + (2 + ((t - 1) - 1)*2 + (a1 - 1)*2*(T-1) + (s1 - 1)*2*(T-1)*A + (a2 - 1)*2*(T-1)*A*S + (s2 - 1)*2*(T-1)*A*S*A)) = 1;
                                    A_tmp((start_relays_ineq - 1 + 2*(T-1)*A*S + 9*(T-1)*A*S*A*S) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_tfin_a_s - 1) + (a2 + ((s2 + 1) - 1)*A)) = -1;
                                    A_tmp((start_relays_ineq - 1 + 2*(T-1)*A*S + 9*(T-1)*A*S*A*S) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_R_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A)) = - tfin_min;
                                    b_tmp((start_relays_ineq - 1 + 2*(T-1)*A*S + 9*(T-1)*A*S*A*S) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A)) = - tfin_min;
                                    A_double_sparse = A_double_sparse + A_tmp;
                                    b_double_sparse = b_double_sparse + b_tmp;

                                    % Check whether the initial value meets this constraint
                                    if not(isempty(x0))
                                    error = A_tmp((start_relays_ineq - 1 + 2*(T-1)*A*S + 9*(T-1)*A*S*A*S) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A),:) * x0 - b_tmp((start_relays_ineq - 1 + 2*(T-1)*A*S + 9*(T-1)*A*S*A*S) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A),:);
                                        if error > tol
                                            disp(['relays ineq 9 not met for (t,a1,s1,a2,s2) = (', num2str(t), ', ', num2str(a1), ', ', num2str(s1), ', ', num2str(a2), ', ', num2str(s2), ') by ', num2str(error)]);
                                        end
                                    end
                                    
                                    % Te(a2,s2)*R(t,a1,s1,a2,s2)   -> TeR(t,a1,s1,a2,s2)
                                    A_tmp = spalloc(max_ineq, length_dv, 2);
                                    % R(t,a1,s1,a2,s2)*Te_min - TeR(t,a1,s1,a2,s2) <= 0
                                    A_tmp((start_relays_ineq - 1 + 2*(T-1)*A*S + 10*(T-1)*A*S*A*S) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_R_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A)) = Te_min;
                                    A_tmp((start_relays_ineq - 1 + 2*(T-1)*A*S + 10*(T-1)*A*S*A*S) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_TeR_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A)) = -1;
                                    A_double_sparse = A_double_sparse + A_tmp;

                                    % Check whether the initial value meets this constraint
                                    if not(isempty(x0))
                                    error = A_tmp((start_relays_ineq - 1 + 2*(T-1)*A*S + 10*(T-1)*A*S*A*S) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A),:) * x0 - 0;
                                        if error > tol
                                            disp(['relays ineq 10 not met for (t,a1,s1,a2,s2) = (', num2str(t), ', ', num2str(a1), ', ', num2str(s1), ', ', num2str(a2), ', ', num2str(s2), ') by ', num2str(error)]);
                                        end
                                    end
                                    
                                    A_tmp = spalloc(max_ineq, length_dv, 2);
                                    % TeR(t,a1,s1,a2,s2) - R(t,a1,s1,a2,s2)*Te_max <= 0
                                    A_tmp((start_relays_ineq - 1 + 2*(T-1)*A*S + 11*(T-1)*A*S*A*S) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_TeR_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A)) = 1;
                                    A_tmp((start_relays_ineq - 1 + 2*(T-1)*A*S + 11*(T-1)*A*S*A*S) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_R_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A)) = - Te_max;
                                    A_double_sparse = A_double_sparse + A_tmp;

                                    % Check whether the initial value meets this constraint
                                    if not(isempty(x0))
                                    error = A_tmp((start_relays_ineq - 1 + 2*(T-1)*A*S + 11*(T-1)*A*S*A*S) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A),:) * x0 - 0;
                                        if error > tol
                                            disp(['relays ineq 11 not met for (t,a1,s1,a2,s2) = (', num2str(t), ', ', num2str(a1), ', ', num2str(s1), ', ', num2str(a2), ', ', num2str(s2), ') by ', num2str(error)]);
                                        end
                                    end
                                    
                                    A_tmp = spalloc(max_ineq, length_dv, 3);
                                    b_tmp = spalloc(max_ineq, 1, 1);
                                    % Te(a2,s2) + Te_max*R(t,a1,s1,a2,s2) - TeR(t,a1,s1,a2,s2) <= Te_max
                                    A_tmp((start_relays_ineq - 1 + 2*(T-1)*A*S + 12*(T-1)*A*S*A*S) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_Te_a_s - 1) + (a2 + (s2 - 1)*A)) = 1;
                                    A_tmp((start_relays_ineq - 1 + 2*(T-1)*A*S + 12*(T-1)*A*S*A*S) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_R_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A)) = Te_max;
                                    A_tmp((start_relays_ineq - 1 + 2*(T-1)*A*S + 12*(T-1)*A*S*A*S) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_TeR_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A)) = -1;
                                    b_tmp((start_relays_ineq - 1 + 2*(T-1)*A*S + 12*(T-1)*A*S*A*S) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A)) = Te_max;
                                    A_double_sparse = A_double_sparse + A_tmp;
                                    b_double_sparse = b_double_sparse + b_tmp;

                                    % Check whether the initial value meets this constraint
                                    if not(isempty(x0))
                                    error = A_tmp((start_relays_ineq - 1 + 2*(T-1)*A*S + 12*(T-1)*A*S*A*S) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A),:) * x0 - b_tmp((start_relays_ineq - 1 + 2*(T-1)*A*S + 12*(T-1)*A*S*A*S) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A),:);
                                        if error > tol
                                            disp(['relays ineq 12 not met for (t,a1,s1,a2,s2) = (', num2str(t), ', ', num2str(a1), ', ', num2str(s1), ', ', num2str(a2), ', ', num2str(s2), ') by ', num2str(error)]);
                                        end
                                    end
                                    
                                    A_tmp = spalloc(max_ineq, length_dv, 3);
                                    b_tmp = spalloc(max_ineq, 1, 1);
                                    % TeR(t,a1,s1,a2,s2) - Te(a2,s2) - Te_min*R(t,a1,s1,a2,s2) <= - Te_min
                                    A_tmp((start_relays_ineq - 1 + 2*(T-1)*A*S + 13*(T-1)*A*S*A*S) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_TeR_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A)) = 1;
                                    A_tmp((start_relays_ineq - 1 + 2*(T-1)*A*S + 13*(T-1)*A*S*A*S) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_Te_a_s - 1) + (a2 + (s2 - 1)*A)) = -1;
                                    A_tmp((start_relays_ineq - 1 + 2*(T-1)*A*S + 13*(T-1)*A*S*A*S) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A), (start_R_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A)) = - Te_min;
                                    b_tmp((start_relays_ineq - 1 + 2*(T-1)*A*S + 13*(T-1)*A*S*A*S) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A)) = - Te_min;
                                    A_double_sparse = A_double_sparse + A_tmp;
                                    b_double_sparse = b_double_sparse + b_tmp;

                                    % Check whether the initial value meets this constraint
                                    if not(isempty(x0))
                                    error = A_tmp((start_relays_ineq - 1 + 2*(T-1)*A*S + 13*(T-1)*A*S*A*S) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A),:) * x0 - b_tmp((start_relays_ineq - 1 + 2*(T-1)*A*S + 13*(T-1)*A*S*A*S) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A),:);
                                        if error > tol
                                            disp(['relays ineq 13 not met for (t,a1,s1,a2,s2) = (', num2str(t), ', ', num2str(a1), ', ', num2str(s1), ', ', num2str(a2), ', ', num2str(s2), ') by ', num2str(error)]);
                                        end
                                    end
                                end
                            end
                        end
                    end
                end

                % "Each task can only be relayed once" constraint
                % sum from a2 = 1 to A of (sum from s2 = 1 to S of (R(t,a1,s1,a2,s2))) <= 1, for all a1 in A, s1 in S and t in [2,T]
                for a1 = 1:A
                    for s1 = 1:S
                        A_tmp = spalloc(max_ineq, length_dv, (A * S));
                        b_tmp = spalloc(max_ineq, 1, 1);
                        % sum from a2 = 1 to A of (sum from s2 = 1 to S of (R(t,a1,s1,a2,s2))) <= 1
                        for a2 = 1:A
                            for s2 = 1:S
                                if not(a1 == a2 && s1 == s2)
                                    A_tmp((start_relays_ineq - 1) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A), (start_R_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A)) = 1;
                                end
                            end
                        end
                        b_tmp((start_relays_ineq - 1) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A)) = 1;
                        A_double_sparse = A_double_sparse + A_tmp;
                        b_double_sparse = b_double_sparse + b_tmp;

                        % Check whether the initial value meets this constraint
                        if not(isempty(x0))
                        error = A_tmp((start_relays_ineq - 1) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A),:) * x0 - b_tmp((start_relays_ineq - 1) + ((t-1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A),:);
                            if error > tol
                                disp(['relays ineq not met for (t,a1,s1) = (', num2str(t), ', ', num2str(a1), ', ', num2str(s1), ') by ', num2str(error)]);
                            end
                        end
                    end
                end

                % "Each task can only rely one task" constraint
                % sum from a1 = 1 to A of (sum from s1 = 1 to S of (R(t,a1,s1,a2,s2))) <= 1, for all a2 in A, s2 in S and t in [2,T]
                for a2 = 1:A
                    for s2 = 1:S
                        A_tmp = spalloc(max_ineq, length_dv, (A * S));
                        b_tmp = spalloc(max_ineq, 1, 1);
                        % sum from a1 = 1 to A of (sum from s1 = 1 to S of (R(t,a1,s1,a2,s2))) <= 1
                        for a1 = 1:A
                            for s1 = 1:S
                                if not(a1 == a2 && s1 == s2)
                                    A_tmp((start_relays_ineq - 1 + (T-1)*A*S) + ((t-1) + (a2 - 1)*(T-1) + (s2 - 1)*(T-1)*A), (start_R_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A)) = 1;
                                end
                            end
                        end
                        b_tmp((start_relays_ineq - 1 + (T-1)*A*S) + ((t-1) + (a2 - 1)*(T-1) + (s2 - 1)*(T-1)*A)) = 1;
                        A_double_sparse = A_double_sparse + A_tmp;
                        b_double_sparse = b_double_sparse + b_tmp;

                        % Check whether the initial value meets this constraint
                        if not(isempty(x0))
                        error = A_tmp((start_relays_ineq - 1 + (T-1)*A*S) + ((t-1) + (a2 - 1)*(T-1) + (s2 - 1)*(T-1)*A),:) * x0 - b_tmp((start_relays_ineq - 1 + (T-1)*A*S) + ((t-1) + (a2 - 1)*(T-1) + (s2 - 1)*(T-1)*A),:);
                            if error > tol
                                disp(['relays ineq not met for (t,a2,s2) = (', num2str(t), ', ', num2str(a2), ', ', num2str(s2), ') by ', num2str(error)]);
                            end
                        end
                    end
                end

                % Number of relays
                % nr(t) = sum from a1 = 1 to A of (sum from s1 = 1 to S of (sum from a2 = 1 to A of (sum from s2 = 1 to S of (R(t,a1,s1,a2,s2))))), for all t = 2 to T
                % nr(t) = n(t) - na(t), for all t = 2 to T
                A_tmp = spalloc(max_eq, length_dv, (2 + (A * S * A * S)));
                % - n(t) + na(t) + sum from a1 = 1 to A of (sum from s1 = 1 to S of (sum from a2 = 1 to A of (sum from s2 = 1 to S of (R(t,a1,s1,a2,s2))))) = 0
                A_tmp((start_relays_eq - 1) + (t - 1), (start_n_t - 1) + t) = -1;
                A_tmp((start_relays_eq - 1) + (t - 1), (start_na_t - 1) + t) = 1;
                for a1 = 1:A
                    for s1 = 1:S
                        for a2 = 1:A
                            for s2 = 1:S
                                if not(a1 == a2 && s1 == s2)
                                    A_tmp((start_relays_eq - 1) + (t - 1), (start_R_t_a1_s1_a2_s2 - 1) + ((t - 1) + (a1 - 1)*(T-1) + (s1 - 1)*(T-1)*A + (a2 - 1)*(T-1)*A*S + (s2 - 1)*(T-1)*A*S*A)) = 1;
                                end
                            end
                        end
                    end
                end
                Aeq_double_sparse = Aeq_double_sparse + A_tmp;

                % Check whether the initial value meets this constraint
                if not(isempty(x0))
                error = abs(A_tmp((start_relays_eq - 1) + (t - 1),:) * x0 - 0);
                    if error > tol
                        disp(['relays eq not met for t = ', num2str(t), ' by ', num2str(error)]);
                    end
                end
            end
        end
    end

    %% Add constraints to force the value of the xx_a_t_t_s, nfx_a_nf_t_s, nax_a_t_s, tfinx_a_t_s, Ftx_a_s1 and Twx_a_s aux variables to be the same as their original hight order terms.
    parfor a = 1:A
        for s = 1:S
            for t = 1:T
                % x(a,t2,s-1)*x(a,t,s) -> xx_a_t_t_s
                for t2 = 0:T
                    A_tmp = spalloc(max_ineq, length_dv, 2);
                    % xx_a_t_t_s - x(a,t,s) <= 0
                    A_tmp((start_linearizations_ineq - 1) + (a + (s - 1)*A + (t - 1)*A*S + ((t2+1) - 1)*A*S*T), (start_xx_a_t_t_s - 1) + (a + ((t2 + 1) - 1)*A + (t - 1)*A*(T+1) + (s - 1)*A*(T+1)*T)) = 1;
                    A_tmp((start_linearizations_ineq - 1) + (a + (s - 1)*A + (t - 1)*A*S + ((t2+1) - 1)*A*S*T), (start_x_a_t_s - 1) + (a + ((t + 1) - 1)*A + ((s + 1) - 1)*A*(T+1))) = -1;
                    A_double_sparse = A_double_sparse + A_tmp;

                    % Check whether the initial value meets this constraint
                    if not(isempty(x0))
                    error = A_tmp((start_linearizations_ineq - 1) + (a + (s - 1)*A + (t - 1)*A*S + ((t2+1) - 1)*A*S*T),:) * x0 - 0;
                        if error > tol
                            disp(['linearizations ineq not met for (a,s,t,t2) = (', num2str(a), ', ', num2str(s), ', ', num2str(t), ', ', num2str(t2), ') by ', num2str(error)]);
                        end
                    end
                    
                    A_tmp = spalloc(max_ineq, length_dv, 2);
                    % xx_a_t_t_s - x(a,t2,s-1) <= 0
                    A_tmp((start_linearizations_ineq - 1 + 1*(A*S*T*(T+1))) + (a + (s - 1)*A + (t - 1)*A*S + ((t2+1) - 1)*A*S*T), (start_xx_a_t_t_s - 1) + (a + ((t2 + 1) - 1)*A + (t - 1)*A*(T+1) + (s - 1)*A*(T+1)*T)) = 1;
                    A_tmp((start_linearizations_ineq - 1 + 1*(A*S*T*(T+1))) + (a + (s - 1)*A + (t - 1)*A*S + ((t2+1) - 1)*A*S*T), (start_x_a_t_s - 1) + (a + ((t2 + 1) - 1)*A + (((s-1) + 1) - 1)*A*(T+1))) = -1;
                    A_double_sparse = A_double_sparse + A_tmp;

                    % Check whether the initial value meets this constraint
                    if not(isempty(x0))
                    error = A_tmp((start_linearizations_ineq - 1 + 1*(A*S*T*(T+1))) + (a + (s - 1)*A + (t - 1)*A*S + ((t2+1) - 1)*A*S*T),:) * x0 - 0;
                        if error > tol
                            disp(['linearizations ineq 1 not met for (a,s,t,t2) = (', num2str(a), ', ', num2str(s), ', ', num2str(t), ', ', num2str(t2), ') by ', num2str(error)]);
                        end
                    end
                    
                    A_tmp = spalloc(max_ineq, length_dv, 3);
                    b_tmp = spalloc(max_ineq, 1, 1);
                    % x(a,t2,s-1) + x(a,t,s) - xx_a_t_t_s <= 1
                    A_tmp((start_linearizations_ineq - 1 + 2*(A*S*T*(T+1))) + (a + (s - 1)*A + (t - 1)*A*S + ((t2+1) - 1)*A*S*T), (start_x_a_t_s - 1) + (a + ((t2 + 1) - 1)*A + (((s-1) + 1) - 1)*A*(T+1))) = 1;
                    A_tmp((start_linearizations_ineq - 1 + 2*(A*S*T*(T+1))) + (a + (s - 1)*A + (t - 1)*A*S + ((t2+1) - 1)*A*S*T), (start_x_a_t_s - 1) + (a + ((t + 1) - 1)*A + ((s + 1) - 1)*A*(T+1))) = 1;
                    A_tmp((start_linearizations_ineq - 1 + 2*(A*S*T*(T+1))) + (a + (s - 1)*A + (t - 1)*A*S + ((t2+1) - 1)*A*S*T), (start_xx_a_t_t_s - 1) + (a + ((t2 + 1) - 1)*A + (t - 1)*A*(T+1) + (s - 1)*A*(T+1)*T)) = -1;
                    b_tmp((start_linearizations_ineq - 1 + 2*(A*S*T*(T+1))) + (a + (s - 1)*A + (t - 1)*A*S + ((t2+1) - 1)*A*S*T)) = 1;
                    A_double_sparse = A_double_sparse + A_tmp;
                    b_double_sparse = b_double_sparse + b_tmp;

                    % Check whether the initial value meets this constraint
                    if not(isempty(x0))
                    error = A_tmp((start_linearizations_ineq - 1 + 2*(A*S*T*(T+1))) + (a + (s - 1)*A + (t - 1)*A*S + ((t2+1) - 1)*A*S*T),:) * x0 - b_tmp((start_linearizations_ineq - 1 + 2*(A*S*T*(T+1))) + (a + (s - 1)*A + (t - 1)*A*S + ((t2+1) - 1)*A*S*T),:);
                        if error > tol
                            disp(['linearizations ineq 2 not met for (a,s,t,t2) = (', num2str(a), ', ', num2str(s), ', ', num2str(t), ', ', num2str(t2), ') by ', num2str(error)]);
                        end
                    end
                end

                % nf(t,nf)*x(a,t,s) -> nfx_a_nf_t_s
                for nf = 1:N
                    A_tmp = spalloc(max_ineq, length_dv, 2);
                    % nfx_a_nf_t_s - x(a,t,s) <= 0
                    A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1))) + (a + (s - 1)*A + (t - 1)*A*S + (nf - 1)*A*S*T), (start_nfx_a_nf_t_s - 1) + (a + (nf - 1)*A + (t - 1)*A*N + (s - 1)*A*N*T)) = 1;
                    A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1))) + (a + (s - 1)*A + (t - 1)*A*S + (nf - 1)*A*S*T), (start_x_a_t_s - 1) + (a + ((t + 1) - 1)*A + ((s + 1) - 1)*A*(T+1))) = -1;
                    A_double_sparse = A_double_sparse + A_tmp;

                    % Check whether the initial value meets this constraint
                    if not(isempty(x0))
                    error = A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1))) + (a + (s - 1)*A + (t - 1)*A*S + (nf - 1)*A*S*T),:) * x0 - 0;
                        if error > tol
                            disp(['linearizations ineq not met for (a,s,t,nf) = (', num2str(a), ', ', num2str(s), ', ', num2str(t), ', ', num2str(nf), ') by ', num2str(error)]);
                        end
                    end
                    
                    A_tmp = spalloc(max_ineq, length_dv, 2);
                    % nfx_a_nf_t_s - nf(t,nf) <= 0
                    A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 1*(A*S*T*N)) + (a + (s - 1)*A + (t - 1)*A*S + (nf - 1)*A*S*T), (start_nfx_a_nf_t_s - 1) + (a + (nf - 1)*A + (t - 1)*A*N + (s - 1)*A*N*T)) = 1;
                    A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 1*(A*S*T*N)) + (a + (s - 1)*A + (t - 1)*A*S + (nf - 1)*A*S*T), (start_nf_t_nf - 1) + (t + (nf - 1)*T)) = -1;
                    A_double_sparse = A_double_sparse + A_tmp;

                    % Check whether the initial value meets this constraint
                    if not(isempty(x0))
                    error = A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 1*(A*S*T*N)) + (a + (s - 1)*A + (t - 1)*A*S + (nf - 1)*A*S*T),:) * x0 - 0;
                        if error > tol
                            disp(['linearizations ineq 1 not met for (a,s,t,nf) = (', num2str(a), ', ', num2str(s), ', ', num2str(t), ', ', num2str(nf), ') by ', num2str(error)]);
                        end
                    end
                    
                    A_tmp = spalloc(max_ineq, length_dv, 3);
                    b_tmp = spalloc(max_ineq, 1, 1);
                    % nf(t,nf) + x(a,t,s) - nfx_a_nf_t_s <= 1
                    A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 2*(A*S*T*N)) + (a + (s - 1)*A + (t - 1)*A*S + (nf - 1)*A*S*T), (start_nf_t_nf - 1) + (t + (nf - 1)*T)) = 1;
                    A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 2*(A*S*T*N)) + (a + (s - 1)*A + (t - 1)*A*S + (nf - 1)*A*S*T), (start_x_a_t_s - 1) + (a + ((t + 1) - 1)*A + ((s + 1) - 1)*A*(T+1))) = 1;
                    A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 2*(A*S*T*N)) + (a + (s - 1)*A + (t - 1)*A*S + (nf - 1)*A*S*T), (start_nfx_a_nf_t_s - 1) + (a + (nf - 1)*A + (t - 1)*A*N + (s - 1)*A*N*T)) = -1;
                    b_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 2*(A*S*T*N)) + (a + (s - 1)*A + (t - 1)*A*S + (nf - 1)*A*S*T)) = 1;
                    A_double_sparse = A_double_sparse + A_tmp;
                    b_double_sparse = b_double_sparse + b_tmp;

                    % Check whether the initial value meets this constraint
                    if not(isempty(x0))
                    error = A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 2*(A*S*T*N)) + (a + (s - 1)*A + (t - 1)*A*S + (nf - 1)*A*S*T),:) * x0 - b_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 2*(A*S*T*N)) + (a + (s - 1)*A + (t - 1)*A*S + (nf - 1)*A*S*T),:);
                        if error > tol
                            disp(['linearizations ineq 2 not met for (a,s,t,nf) = (', num2str(a), ', ', num2str(s), ', ', num2str(t), ', ', num2str(nf), ') by ', num2str(error)]);
                        end
                    end
                end

                % na(t)*x(a,t,s) -> nax_a_t_s
                A_tmp = spalloc(max_ineq, length_dv, 2);
                % x(a,t,s)*na_min - nax_a_t_s <= 0
                A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N)) + (a + (s - 1)*A + (t - 1)*A*S), (start_x_a_t_s - 1) + (a + ((t + 1) - 1)*A + ((s + 1) - 1)*A*(T+1))) = na_min;
                A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N)) + (a + (s - 1)*A + (t - 1)*A*S), (start_nax_a_t_s - 1) + (a + (t - 1)*A + (s - 1)*A*T)) = -1;
                A_double_sparse = A_double_sparse + A_tmp;

                % Check whether the initial value meets this constraint
                if not(isempty(x0))
                error = A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N)) + (a + (s - 1)*A + (t - 1)*A*S),:) * x0 - 0;
                    if error > tol
                        disp(['linearizations ineq not met for (a,s,t) = (', num2str(a), ', ', num2str(s), ', ', num2str(t), ') by ', num2str(error)]);
                    end
                end
                
                A_tmp = spalloc(max_ineq, length_dv, 2);
                % nax_a_t_s - x(a,t,s)*na_max <= 0
                A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 1*(A*S*T)) + (a + (s - 1)*A + (t - 1)*A*S), (start_nax_a_t_s - 1) + (a + (t - 1)*A + (s - 1)*A*T)) = 1;
                A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 1*(A*S*T)) + (a + (s - 1)*A + (t - 1)*A*S), (start_x_a_t_s - 1) + (a + ((t + 1) - 1)*A + ((s + 1) - 1)*A*(T+1))) = - na_max;
                A_double_sparse = A_double_sparse + A_tmp;

                % Check whether the initial value meets this constraint
                if not(isempty(x0))
                error = A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 1*(A*S*T)) + (a + (s - 1)*A + (t - 1)*A*S),:) * x0 - 0;
                    if error > tol
                        disp(['linearizations ineq 1 not met for (a,s,t) = (', num2str(a), ', ', num2str(s), ', ', num2str(t), ') by ', num2str(error)]);
                    end
                end
                
                A_tmp = spalloc(max_ineq, length_dv, 3);
                b_tmp = spalloc(max_ineq, 1, 1);
                % na(t) + na_max*x(a,t,s) - nax_a_t_s <= na_max
                A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 2*(A*S*T)) + (a + (s - 1)*A + (t - 1)*A*S), (start_na_t - 1) + t) = 1;
                A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 2*(A*S*T)) + (a + (s - 1)*A + (t - 1)*A*S), (start_x_a_t_s - 1) + (a + ((t + 1) - 1)*A + ((s + 1) - 1)*A*(T+1))) = na_max;
                A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 2*(A*S*T)) + (a + (s - 1)*A + (t - 1)*A*S), (start_nax_a_t_s - 1) + (a + (t - 1)*A + (s - 1)*A*T)) = -1;
                b_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 2*(A*S*T)) + (a + (s - 1)*A + (t - 1)*A*S)) = na_max;
                A_double_sparse = A_double_sparse + A_tmp;
                b_double_sparse = b_double_sparse + b_tmp;

                % Check whether the initial value meets this constraint
                if not(isempty(x0))
                error = A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 2*(A*S*T)) + (a + (s - 1)*A + (t - 1)*A*S),:) * x0 - b_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 2*(A*S*T)) + (a + (s - 1)*A + (t - 1)*A*S),:);
                    if error > tol
                        disp(['linearizations ineq 2 not met for (a,s,t) = (', num2str(a), ', ', num2str(s), ', ', num2str(t), ') by ', num2str(error)]);
                    end
                end
                
                A_tmp = spalloc(max_ineq, length_dv, 3);
                b_tmp = spalloc(max_ineq, 1, 1);
                % nax_a_t_s - na(t) - na_min*x(a,t,s) <= - na_min
                A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 3*(A*S*T)) + (a + (s - 1)*A + (t - 1)*A*S), (start_nax_a_t_s - 1) + (a + (t - 1)*A + (s - 1)*A*T)) = 1;
                A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 3*(A*S*T)) + (a + (s - 1)*A + (t - 1)*A*S), (start_na_t - 1) + t) = -1;
                A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 3*(A*S*T)) + (a + (s - 1)*A + (t - 1)*A*S), (start_x_a_t_s - 1) + (a + ((t + 1) - 1)*A + ((s + 1) - 1)*A*(T+1))) = - na_min;
                b_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 3*(A*S*T)) + (a + (s - 1)*A + (t - 1)*A*S)) = - na_min;
                A_double_sparse = A_double_sparse + A_tmp;
                b_double_sparse = b_double_sparse + b_tmp;

                % Check whether the initial value meets this constraint
                if not(isempty(x0))
                error = A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 3*(A*S*T)) + (a + (s - 1)*A + (t - 1)*A*S),:) * x0 - b_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 3*(A*S*T)) + (a + (s - 1)*A + (t - 1)*A*S),:);
                    if error > tol
                        disp(['linearizations ineq 3 not met for (a,s,t) = (', num2str(a), ', ', num2str(s), ', ', num2str(t), ') by ', num2str(error)]);
                    end
                end

                % tfin(a,s)*x(a,t,s) -> tfinx_a_t_s
                A_tmp = spalloc(max_ineq, length_dv, 2);
                % x(a,t,s)*tfin_min - tfinx_a_t_s <= 0
                A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 4*(A*S*T)) + (a + (s - 1)*A + (t - 1)*A*S), (start_x_a_t_s - 1) + (a + ((t + 1) - 1)*A + ((s + 1) - 1)*A*(T+1))) = tfin_min;
                A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 4*(A*S*T)) + (a + (s - 1)*A + (t - 1)*A*S), (start_tfinx_a_t_s - 1) + (a + (t - 1)*A + (s - 1)*A*T)) = -1;
                A_double_sparse = A_double_sparse + A_tmp;

                % Check whether the initial value meets this constraint
                if not(isempty(x0))
                error = A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 4*(A*S*T)) + (a + (s - 1)*A + (t - 1)*A*S),:) * x0 - 0;
                    if error > tol
                        disp(['linearizations ineq 4 not met for (a,s,t) = (', num2str(a), ', ', num2str(s), ', ', num2str(t), ') by ', num2str(error)]);
                    end
                end
                
                A_tmp = spalloc(max_ineq, length_dv, 2);
                % tfinx_a_t_s - x(a,t,s)*tfin_max <= 0
                A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 5*(A*S*T)) + (a + (s - 1)*A + (t - 1)*A*S), (start_tfinx_a_t_s - 1) + (a + (t - 1)*A + (s - 1)*A*T)) = 1;
                A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 5*(A*S*T)) + (a + (s - 1)*A + (t - 1)*A*S), (start_x_a_t_s - 1) + (a + ((t + 1) - 1)*A + ((s + 1) - 1)*A*(T+1))) = - tfin_max;
                A_double_sparse = A_double_sparse + A_tmp;

                % Check whether the initial value meets this constraint
                if not(isempty(x0))
                error = A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 5*(A*S*T)) + (a + (s - 1)*A + (t - 1)*A*S),:) * x0 - 0;
                    if error > tol
                        disp(['linearizations ineq 5 not met for (a,s,t) = (', num2str(a), ', ', num2str(s), ', ', num2str(t), ') by ', num2str(error)]);
                    end
                end
                
                A_tmp = spalloc(max_ineq, length_dv, 3);
                b_tmp = spalloc(max_ineq, 1, 1);
                % tfin(a,s) + tfin_max*x(a,t,s) - tfinx_a_t_s <= tfin_max
                A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 6*(A*S*T)) + (a + (s - 1)*A + (t - 1)*A*S), (start_tfin_a_s - 1) + (a + ((s + 1) - 1)*A)) = 1;
                A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 6*(A*S*T)) + (a + (s - 1)*A + (t - 1)*A*S), (start_x_a_t_s - 1) + (a + ((t + 1) - 1)*A + ((s + 1) - 1)*A*(T+1))) = tfin_max;
                A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 6*(A*S*T)) + (a + (s - 1)*A + (t - 1)*A*S), (start_tfinx_a_t_s - 1) + (a + (t - 1)*A + (s - 1)*A*T)) = -1;
                b_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 6*(A*S*T)) + (a + (s - 1)*A + (t - 1)*A*S)) = tfin_max;
                A_double_sparse = A_double_sparse + A_tmp;
                b_double_sparse = b_double_sparse + b_tmp;

                % Check whether the initial value meets this constraint
                if not(isempty(x0))
                error = A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 6*(A*S*T)) + (a + (s - 1)*A + (t - 1)*A*S),:) * x0 - b_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 6*(A*S*T)) + (a + (s - 1)*A + (t - 1)*A*S),:);
                    if error > tol
                        disp(['linearizations ineq 6 not met for (a,s,t) = (', num2str(a), ', ', num2str(s), ', ', num2str(t), ') by ', num2str(error)]);
                    end
                end
                
                A_tmp = spalloc(max_ineq, length_dv, 3);
                b_tmp = spalloc(max_ineq, 1, 1);
                % tfinx_a_t_s - tfin(a,s) - tfin_min*x(a,t,s) <= - tfin_min
                A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 7*(A*S*T)) + (a + (s - 1)*A + (t - 1)*A*S), (start_tfinx_a_t_s - 1) + (a + (t - 1)*A + (s - 1)*A*T)) = 1;
                A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 7*(A*S*T)) + (a + (s - 1)*A + (t - 1)*A*S), (start_tfin_a_s - 1) + (a + ((s + 1) - 1)*A)) = -1;
                A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 7*(A*S*T)) + (a + (s - 1)*A + (t - 1)*A*S), (start_x_a_t_s - 1) + (a + ((t + 1) - 1)*A + ((s + 1) - 1)*A*(T+1))) = - tfin_min;
                b_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 7*(A*S*T)) + (a + (s - 1)*A + (t - 1)*A*S)) = - tfin_min;
                A_double_sparse = A_double_sparse + A_tmp;
                b_double_sparse = b_double_sparse + b_tmp;

                % Check whether the initial value meets this constraint
                if not(isempty(x0))
                error = A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 7*(A*S*T)) + (a + (s - 1)*A + (t - 1)*A*S),:) * x0 - b_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 7*(A*S*T)) + (a + (s - 1)*A + (t - 1)*A*S),:);
                    if error > tol
                        disp(['linearizations ineq 7 not met for (a,s,t) = (', num2str(a), ', ', num2str(s), ', ', num2str(t), ') by ', num2str(error)]);
                    end
                end
            end
            
            A_tmp = spalloc(max_ineq, length_dv, 2);
            % x(a,1,s-1)*Ft_min_a - Ftx_a_s1 <= 0
            A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 8*(A*S*T)) + (a + (s - 1)*A), (start_x_a_t_s - 1) + (a + ((R + 1) - 1)*A + (((s-1) + 1) - 1)*A*(T+1))) = Ft_min_a;
            A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 8*(A*S*T)) + (a + (s - 1)*A), (start_Ftx_a_s1 - 1) + (a + (s - 1)*A)) = -1;
            A_double_sparse = A_double_sparse + A_tmp;

            % Check whether the initial value meets this constraint
            if not(isempty(x0))
            error = A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 8*(A*S*T)) + (a + (s - 1)*A),:) * x0 - 0;
                if error > tol
                    disp(['linearizations ineq not met for (a,s) = (', num2str(a), ', ', num2str(s), ') by ', num2str(error)]);
                end
            end
            
            A_tmp = spalloc(max_ineq, length_dv, 2);
            % Ftx_a_s1 - x(a,1,s-1)*Ft_max_a <= 0
            A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 8*(A*S*T) + 1*(A*S)) + (a + (s - 1)*A), (start_Ftx_a_s1 - 1) + (a + (s - 1)*A)) = 1;
            A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 8*(A*S*T) + 1*(A*S)) + (a + (s - 1)*A), (start_x_a_t_s - 1) + (a + ((R + 1) - 1)*A + (((s-1) + 1) - 1)*A*(T+1))) = - Ft_max_a;
            A_double_sparse = A_double_sparse + A_tmp;

            % Check whether the initial value meets this constraint
            if not(isempty(x0))
            error = A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 8*(A*S*T) + 1*(A*S)) + (a + (s - 1)*A),:) * x0 - 0;
                if error > tol
                    disp(['linearizations ineq 1 not met for (a,s) = (', num2str(a), ', ', num2str(s), ') by ', num2str(error)]);
                end
            end
            
            A_tmp = spalloc(max_ineq, length_dv, 3);
            b_tmp = spalloc(max_ineq, 1, 1);
            % Ft(a,s-1) + Ft_max_a*x(a,1,s-1) - Ftx_a_s1 <= Ft_max_a
            A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 8*(A*S*T) + 2*(A*S)) + (a + (s - 1)*A), (start_Ft_a_s - 1) + (a + (((s-1) + 1) - 1)*A)) = 1;
            A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 8*(A*S*T) + 2*(A*S)) + (a + (s - 1)*A), (start_x_a_t_s - 1) + (a + ((R + 1) - 1)*A + (((s-1) + 1) - 1)*A*(T+1))) = Ft_max_a;
            A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 8*(A*S*T) + 2*(A*S)) + (a + (s - 1)*A), (start_Ftx_a_s1 - 1) + (a + (s - 1)*A)) = -1;
            b_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 8*(A*S*T) + 2*(A*S)) + (a + (s - 1)*A)) = Ft_max_a;
            A_double_sparse = A_double_sparse + A_tmp;
            b_double_sparse = b_double_sparse + b_tmp;

            % Check whether the initial value meets this constraint
            if not(isempty(x0))
            error = A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 8*(A*S*T) + 2*(A*S)) + (a + (s - 1)*A),:) * x0 - b_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 8*(A*S*T) + 2*(A*S)) + (a + (s - 1)*A),:);
                if error > tol
                    disp(['linearizations ineq 2 not met for (a,s) = (', num2str(a), ', ', num2str(s), ') by ', num2str(error)]);
                end
            end
            
            A_tmp = spalloc(max_ineq, length_dv, 3);
            b_tmp = spalloc(max_ineq, 1, 1);
            % Ftx_a_s1 - Ft(a,s-1) - Ft_min_a*x(a,1,s-1) <= - Ft_min_a
            A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 8*(A*S*T) + 3*(A*S)) + (a + (s - 1)*A), (start_Ftx_a_s1 - 1) + (a + (s - 1)*A)) = 1;
            A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 8*(A*S*T) + 3*(A*S)) + (a + (s - 1)*A), (start_Ft_a_s - 1) + (a + (((s-1) + 1) - 1)*A)) = -1;
            A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 8*(A*S*T) + 3*(A*S)) + (a + (s - 1)*A), (start_x_a_t_s - 1) + (a + ((R + 1) - 1)*A + (((s-1) + 1) - 1)*A*(T+1))) = - Ft_min_a;
            b_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 8*(A*S*T) + 3*(A*S)) + (a + (s - 1)*A)) = - Ft_min_a;
            A_double_sparse = A_double_sparse + A_tmp;
            b_double_sparse = b_double_sparse + b_tmp;

            % Check whether the initial value meets this constraint
            if not(isempty(x0))
            error = A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 8*(A*S*T) + 3*(A*S)) + (a + (s - 1)*A),:) * x0 - b_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 8*(A*S*T) + 3*(A*S)) + (a + (s - 1)*A),:);
                if error > tol
                    disp(['linearizations ineq 3 not met for (a,s) = (', num2str(a), ', ', num2str(s), ') by ', num2str(error)]);
                end
            end

            % Tw(a, s)*x(a,1,s) -> Twx_a_s
            A_tmp = spalloc(max_ineq, length_dv, 2);
            % x(a,1,s)*Tw_min - Twx_a_s <= 0
            A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 8*(A*S*T) + 4*(A*S)) + (a + (s - 1)*A), (start_x_a_t_s - 1) + (a + ((R + 1) - 1)*A + ((s + 1) - 1)*A*(T+1))) = Tw_min;
            A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 8*(A*S*T) + 4*(A*S)) + (a + (s - 1)*A), (start_Twx_a_s - 1) + (a + (s - 1)*A)) = -1;
            A_double_sparse = A_double_sparse + A_tmp;

            % Check whether the initial value meets this constraint
            if not(isempty(x0))
            error = A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 8*(A*S*T) + 4*(A*S)) + (a + (s - 1)*A),:) * x0 - 0;
                if error > tol
                    disp(['linearizations ineq 4 not met for (a,s) = (', num2str(a), ', ', num2str(s), ') by ', num2str(error)]);
                end
            end
            
            A_tmp = spalloc(max_ineq, length_dv, 2);
            % Twx_a_s - x(a,1,s)*Tw_max <= 0
            A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 8*(A*S*T) + 5*(A*S)) + (a + (s - 1)*A), (start_Twx_a_s - 1) + (a + (s - 1)*A)) = 1;
            A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 8*(A*S*T) + 5*(A*S)) + (a + (s - 1)*A), (start_x_a_t_s - 1) + (a + ((R + 1) - 1)*A + ((s + 1) - 1)*A*(T+1))) = - Tw_max;
            A_double_sparse = A_double_sparse + A_tmp;

            % Check whether the initial value meets this constraint
            if not(isempty(x0))
            error = A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 8*(A*S*T) + 5*(A*S)) + (a + (s - 1)*A),:) * x0 - 0;
                if error > tol
                    disp(['linearizations ineq 5 not met for (a,s) = (', num2str(a), ', ', num2str(s), ') by ', num2str(error)]);
                end
            end
            
            A_tmp = spalloc(max_ineq, length_dv, 3);
            b_tmp = spalloc(max_ineq, 1, 1);
            % Tw(a, s) + Tw_max*x(a,1,s) - Twx_a_s <= Tw_max
            A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 8*(A*S*T) + 6*(A*S)) + (a + (s - 1)*A), (start_Tw_a_s - 1) + (a + (s - 1)*A)) = 1;
            A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 8*(A*S*T) + 6*(A*S)) + (a + (s - 1)*A), (start_x_a_t_s - 1) + (a + ((R + 1) - 1)*A + ((s + 1) - 1)*A*(T+1))) = Tw_max;
            A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 8*(A*S*T) + 6*(A*S)) + (a + (s - 1)*A), (start_Twx_a_s - 1) + (a + (s - 1)*A)) = -1;
            b_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 8*(A*S*T) + 6*(A*S)) + (a + (s - 1)*A)) = Tw_max;
            A_double_sparse = A_double_sparse + A_tmp;
            b_double_sparse = b_double_sparse + b_tmp;

            % Check whether the initial value meets this constraint
            if not(isempty(x0))
            error = A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 8*(A*S*T) + 6*(A*S)) + (a + (s - 1)*A),:) * x0 - b_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 8*(A*S*T) + 6*(A*S)) + (a + (s - 1)*A),:);
                if error > tol
                    disp(['linearizations ineq 6 not met for (a,s) = (', num2str(a), ', ', num2str(s), ') by ', num2str(error)]);
                end
            end
            
            A_tmp = spalloc(max_ineq, length_dv, 3);
            b_tmp = spalloc(max_ineq, 1, 1);
            % Twx_a_s - Tw(a, s) - Tw_min*x(a,1,s) <= - Tw_min
            A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 8*(A*S*T) + 7*(A*S)) + (a + (s - 1)*A), (start_Twx_a_s - 1) + (a + (s - 1)*A)) = 1;
            A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 8*(A*S*T) + 7*(A*S)) + (a + (s - 1)*A), (start_Tw_a_s - 1) + (a + (s - 1)*A)) = -1;
            A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 8*(A*S*T) + 7*(A*S)) + (a + (s - 1)*A), (start_x_a_t_s - 1) + (a + ((R + 1) - 1)*A + ((s + 1) - 1)*A*(T+1))) = - Tw_min;
            b_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 8*(A*S*T) + 7*(A*S)) + (a + (s - 1)*A)) = - Tw_min;
            A_double_sparse = A_double_sparse + A_tmp;
            b_double_sparse = b_double_sparse + b_tmp;

            % Check whether the initial value meets this constraint
            if not(isempty(x0))
            error = A_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 8*(A*S*T) + 7*(A*S)) + (a + (s - 1)*A),:) * x0 - b_tmp((start_linearizations_ineq - 1 + 3*(A*S*T*(T+1)) + 3*(A*S*T*N) + 8*(A*S*T) + 7*(A*S)) + (a + (s - 1)*A),:);
                if error > tol
                    disp(['linearizations ineq 7 not met for (a,s) = (', num2str(a), ', ', num2str(s), ') by ', num2str(error)]);
                end
            end
        end
    end

    %% Optimization
    if display_flag
        disp(['Time to build A, b, Aeq and beq matrices: ', num2str(toc - last_toc), 's']);
    end
    if log_file_flag
        fprintf(logFile, 'Time to build A, b, Aeq and beq matrices: %f s\n', toc - last_toc);
    end

    %% Solver
    last_toc = toc;

    % Solver settings
    if genetic_algorithm_solver
        % GA solver options: see https://es.mathworks.com/help/gads/genetic-algorithm-options.html
        options = optimoptions('ga', 'UseVectorized', true, 'UseParallel', true, 'PlotFcn', @gaplotbestf);
    else
        % Matlab/Gurobi MILP solver options: : see https://es.mathworks.com/help/optim/ug/intlinprog.html#btv2x05
        switch solver_config
        case 1
            % Save best solution so far while solving
            options = optimoptions('intlinprog', 'Display', 'off', 'MaxTime', inf, 'MaxNodes', inf, 'OutputFcn', @saveBestSolutionSoFar);%, 'BranchRule', 'maxfun', 'CutGeneration', 'advanced', 'Heuristics', 'advanced' 'RelativeGapTolerance', 0, 'AbsoluteGapTolerance', 0, 'ConstraintTolerance', 1e-9, 'IntegerTolerance', 1e-6, 'LPOptimalityTolerance', 1e-10);
        case 2
            % Save best solution so far while solving, with iter info
            options = optimoptions('intlinprog', 'Display', 'iter', 'MaxTime', inf, 'MaxNodes', inf, 'OutputFcn', @saveBestSolutionSoFar);%, 'BranchRule', 'maxfun', 'CutGeneration', 'advanced', 'Heuristics', 'advanced' 'RelativeGapTolerance', 0, 'AbsoluteGapTolerance', 0, 'ConstraintTolerance', 1e-9, 'IntegerTolerance', 1e-6, 'LPOptimalityTolerance', 1e-10);
        case 3
            % Stop when the first valid solution is found
            options = optimoptions('intlinprog', 'Display', 'off', 'MaxTime', inf, 'MaxNodes', inf, 'OutputFcn', @stopAtFirstValidSolution);%, 'BranchRule', 'maxfun', 'CutGeneration', 'advanced', 'Heuristics', 'advanced' 'RelativeGapTolerance', 0, 'AbsoluteGapTolerance', 0, 'ConstraintTolerance', 1e-9, 'IntegerTolerance', 1e-6, 'LPOptimalityTolerance', 1e-10);
        case 4
            % No output function, display off
            options = optimoptions('intlinprog', 'Display', 'off', 'MaxTime', inf, 'MaxNodes', inf);%, 'BranchRule', 'maxfun', 'CutGeneration', 'advanced', 'Heuristics', 'advanced' 'RelativeGapTolerance', 0, 'AbsoluteGapTolerance', 0, 'ConstraintTolerance', 1e-9, 'IntegerTolerance', 1e-6, 'LPOptimalityTolerance', 1e-10);
        case 5
            % Save all found integer solutions, with iter
            options = optimoptions('intlinprog', 'Display', 'iter', 'MaxTime', 1*24*60*60, 'MaxNodes', inf, 'OutputFcn', @savemilpsolutions);%, 'BranchRule', 'maxfun', 'CutGeneration', 'advanced', 'Heuristics', 'advanced' 'RelativeGapTolerance', 0, 'AbsoluteGapTolerance', 0, 'ConstraintTolerance', 1e-9, 'IntegerTolerance', 1e-6, 'LPOptimalityTolerance', 1e-10);
        otherwise
            % No output function
            options = optimoptions('intlinprog', 'Display', 'iter', 'MaxTime', inf, 'MaxNodes', inf);%, 'BranchRule', 'maxfun', 'CutGeneration', 'advanced', 'Heuristics', 'advanced' 'RelativeGapTolerance', 0, 'AbsoluteGapTolerance', 0, 'ConstraintTolerance', 1e-9, 'IntegerTolerance', 1e-6, 'LPOptimalityTolerance', 1e-10);
        end
    end

    if not(solve_flag)
        return;
    end

    % If a handmade or previous solution is loaded, check if it's valid
    if recovery_flag && not(isempty(x0))
        try
            result_eq = ismembertol(Aeq_double_sparse * x0, full(beq_double_sparse), tol);
            result_ineq = A_double_sparse * x0 <= full(b_double_sparse) + tol;

            eq_not_met = find(~result_eq);
            ineq_not_met = find(~result_ineq);

            if not(isempty(eq_not_met)) || not(isempty(ineq_not_met))
                warning('x0 is infeasible');

                % disp(eq_not_met);
                % disp(ineq_not_met);

                sol = [];
                fval = 0;
                population = [];
                scores = [];
                return;
            end
        end
    end

    if test_flag
        sol = [];
        fval = 0;
        population = [];
        scores = [];
        return;
    end
    
    % Solver call
    if genetic_algorithm_solver
        % Define the function to compute the fitness values
        fun = @(dv)(f*dv');
        % Call Matlab GA solver
        [sol, fval, exitflag, output, population, scores] = ga(fun, length_dv, A_double_sparse, b_double_sparse, Aeq_double_sparse, beq_double_sparse, lb, ub, [], intcon, options);
    else
        % Call Matlab/Gurobi MILP solver
        [sol, fval, exitflag, output] = intlinprog(f, intcon, A_double_sparse, b_double_sparse, Aeq_double_sparse, beq_double_sparse, lb, ub, x0, options);

        % Set GA extra output variables value
        population = [];
        scores = [];
    end

    solving_time = toc - last_toc;
    if display_flag
        disp(['Solving time: ', num2str(solving_time), 's']);
    end
    if log_file_flag
        fprintf(logFile, 'Solving time: %f s\n', solving_time);
    end

    if not(isempty(sol))
        % Round integer variables to eliminate tolerance decimals
        sol(intcon) = round(sol(intcon));

        % Substitute virtual zeros by exact zeros in the sol vector
        sol(abs(sol) < tol) = 0;

        % Decompose fval into each objective function term associated fval
        if log_file_flag
            f_term_1  = sparse(1,length_dv);
            f_term_2  = sparse(1,length_dv);
            f_term_3  = sparse(1,length_dv);
            f_term_4  = sparse(1,length_dv);

            % d_tmax_tfin_a_s: Coefficients of the term penalizing exceeding the tmax of a task.
            f_term_2((start_d_tmax_tfin_a_s - 1) + 1 : (start_d_tmax_tfin_a_s - 1) + length_d_tmax_tfin_a_s) = 1/d_tmax_max;

            % V: Coefficients of the term penalizing the use of a different number of Agents. V(2 to T) to exclude Recharge task.
            f_term_4((start_V_t - 1) + 2 : (start_V_t - 1) + length_V_t) = 1/V_max;

            fval_f2 = f_term_2*sol;
            fval_f4 = f_term_4*sol;

            switch objective_function
            case 2
                % Minimize the total joint flight time: min(sum(tfin(a,S))), for all a = 1 to A.
                f_term_1((start_tfin_a_s - 1) + (1 + ((S + 1) - 1)*A):(start_tfin_a_s - 1) + (A + ((S + 1) - 1)*A)) = 1/tmax_m;

                fval_f1 = f_term_1*sol;
                
                fprintf(logFile, 'fval f   = %f s\n', fval);
                fprintf(logFile, 'fval f_1 = %f s\n', fval_f1);
                fprintf(logFile, 'fval f_2 = %f s\n', fval_f2);
                fprintf(logFile, 'fval f_4 = %f s\n', fval_f4);

                disp('fval fval_f1 fval_f2 fval_f4');
                disp([fval fval_f1 fval_f2 fval_f4]);
            case 3
                % Minimize the longest queue's execution time: min(max(tfin(a,S))) -> min(z).
                f_term_1((start_z - 1) + 1) = 1/z_max;
                
                % Tw: Coefficients of the term minimizing the waiting time to avoid unnecessary waitings.
                f_term_3((start_Tw_a_s - 1) + 1 : (start_Tw_a_s - 1) + length_Tw_a_s) = 1/Tw_max;

                fval_f1 = f_term_1*sol;
                fval_f3 = f_term_3*sol;

                fprintf(logFile, 'fval f   = %f s\n', fval);
                fprintf(logFile, 'fval f_1 = %f s\n', fval_f1);
                fprintf(logFile, 'fval f_2 = %f s\n', fval_f2);
                fprintf(logFile, 'fval f_3 = %f s\n', fval_f3);
                fprintf(logFile, 'fval f_4 = %f s\n', fval_f4);

                disp('fval fval_f1 fval_f2 fval_f3 fval_f4');
                disp([fval fval_f1 fval_f2 fval_f3 fval_f4]);
            case 4
                f_term_1a = sparse(1,length_dv);
                f_term_1b = sparse(1,length_dv);

                % Minimize the longest queue's execution time: min(max(tfin(a,S))) -> min(z).
                f_term_1a((start_z - 1) + 1) = alpha/z_max;

                % Minimize the total joint flight time: min(sum(tfin(a,S))), for all a = 1 to A.
                f_term_1b((start_tfin_a_s - 1) + (1 + ((S + 1) - 1)*A):(start_tfin_a_s - 1) + (A + ((S + 1) - 1)*A)) = betta/tmax_m;
                
                % Tw: Coefficients of the term minimizing the waiting time to avoid unnecessary waitings.
                f_term_3((start_Tw_a_s - 1) + 1 : (start_Tw_a_s - 1) + length_Tw_a_s) = 1/Tw_max;

                fval_f1a = f_term_1a*sol;
                fval_f1b = f_term_1b*sol;
                fval_f3  = f_term_3*sol;

                fprintf(logFile, 'fval f    = %f s\n', fval);
                fprintf(logFile, 'fval f_1a = %f s\n', fval_f1a);
                fprintf(logFile, 'fval f_1b = %f s\n', fval_f1b);
                fprintf(logFile, 'fval f_2  = %f s\n', fval_f2);
                fprintf(logFile, 'fval f_3  = %f s\n', fval_f3);
                fprintf(logFile, 'fval f_4  = %f s\n', fval_f4);

                disp('fval fval_f1a fval_f1b fval_f2 fval_f3 fval_f4');
                disp([fval fval_f1a fval_f1b fval_f2 fval_f3 fval_f4]);
            otherwise
                f_term_5  = sparse(1,length_dv);

                % Minimize the longest queue's execution time: min(max(tfin(a,S))) -> min(z).
                f_term_1((start_z - 1) + 1) = 1/z_max;
                
                % Tw: Coefficients of the term minimizing the waiting time to avoid unnecessary waitings.
                f_term_3((start_Tw_a_s - 1) + 1 : (start_Tw_a_s - 1) + length_Tw_a_s) = 1/Tw_max;

                % s_used: Coefficients of the term minimizing the number of used slots.
                f_term_5((start_s_used - 1) + 1 : (start_s_used - 1) + length_s_used) = 1/s_used_max;

                fval_f1 = f_term_1*sol;
                fval_f3 = f_term_3*sol;
                fval_f5 = f_term_5*sol;

                fprintf(logFile, 'fval f   = %f s\n', fval);
                fprintf(logFile, 'fval f_1 = %f s\n', fval_f1);
                fprintf(logFile, 'fval f_2 = %f s\n', fval_f2);
                fprintf(logFile, 'fval f_3 = %f s\n', fval_f3);
                fprintf(logFile, 'fval f_4 = %f s\n', fval_f4);
                fprintf(logFile, 'fval f_5 = %f s\n', fval_f5);

                disp('fval fval_f1 fval_f2 fval_f3 fval_f4 fval_f5');
                disp([fval fval_f1 fval_f2 fval_f3 fval_f4 fval_f5]);
            end
            % Add a spacing line at the end of each run in the log file
            fprintf(logFile, '\n--------------------\n');
        end
    end

    save(['../mat/sol_', execution_id, '.mat'], 'sol');
    save(['../mat/fval_', execution_id, '.mat'], 'fval');
    if save_flag
        save(['../mat/exitflag_', execution_id, '.mat'], 'exitflag');
        save(['../mat/output_', execution_id, '.mat'], 'output');
    end

    %% Print solution
    if print_solution_flag
        printSolution(sol, Agent, Task, 1, scenario_id, execution_id, fval);
    end

    if isempty(sol) && predefined == 0 && not(save_flag)
        previewScenario(Agent, Task, scenario_id);
        save(['../mat/Agent_', execution_id, '.mat'], 'Agent');
        save(['../mat/Task_', execution_id, '.mat'], 'Task');
    end
end
