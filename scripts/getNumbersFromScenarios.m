function getNumbersFromScenarios(amount, number_agents, number_tasks)
    if nargin < 1
        number_agents = [1, 1, 2, 2, 2, 3, 3, 2, 5, 5,  2, 10, 10];
        number_tasks  = [1, 2, 1, 2, 3, 2, 3, 5, 2, 5, 10,  2, 10];
        amount = 100;
    end

    min_values = [];
    av_values = [];
    max_values = [];
    biggest_values = [];

    %% Run each variant of the heuristic for each scenario
    for s = 1:length(number_agents)
        num_agents = number_agents(s);
        num_tasks = number_tasks(s);

        disp(['Processing scenarios with ', num2str(num_agents), ' agents and ', num2str(num_tasks), ' tasks...']);

        % Initialize the biggest scenario values
        biggest_basic_constraints_variables = 0;
        biggest_time_related_variables = 0;
        biggest_time_related_linearization_variables = 0;
        biggest_time_coordination_variables = 0;
        biggest_time_coordination_linearization_variables = 0;
        biggest_binary_variables = 0;
        biggest_integer_variables = 0;
        biggest_real_variables = 0;
        biggest_total_variables = 0;
        biggest_basic_constraints = 0;
        biggest_time_related_constraints = 0;
        biggest_time_related_linearization_constraints = 0;
        biggest_time_coordination_constraints = 0;
        biggest_time_coordination_linearization_constraints = 0;
        biggest_total_constraints = 0;

        % Initialize the variables to store the results
        all_basic_constraints_variables = [];
        all_time_related_variables = [];
        all_time_related_linearization_variables = [];
        all_time_coordination_variables = [];
        all_time_coordination_linearization_variables = [];
        all_binary_variables = [];
        all_integer_variables = [];
        all_real_variables = [];
        all_total_variables = [];
        all_basic_constraints = [];
        all_time_related_constraints = [];
        all_time_related_linearization_constraints = [];
        all_time_coordination_constraints = [];
        all_time_coordination_linearization_constraints = [];
        all_total_constraints = [];

        for random_scenario = 1:amount
            % Generate scenario id
            scenario_id = [num2str(num_agents), 'r', num2str(num_tasks), 't', num2str(random_scenario)];

            % Try loading scenario
            try
                [Agent, Task] = scenario(scenario_id);
            catch
                error('Either Agent and Task or a valid scenario_id must be provided');
            end

            [~, ~, A, T, S, N, ~, ~, ~, ~] = getConstantScenarioValues(Agent, Task);

            [basic_constraints_variables, ...
            time_related_variables, ...
            time_related_linearization_variables, ...
            time_coordination_variables, ...
            time_coordination_linearization_variables, ...
            binary_variables, ...
            integer_variables, ...
            real_variables, ...
            total_variables] = getVariablesNumbersPerCategory(A, T, S, N);

            [basic_constraints, ...
            time_related_constraints, ...
            time_related_linearization_constraints, ...
            time_coordination_constraints, ...
            time_coordination_linearization_constraints, ...
            total_constraints] = getConstraintsNumbersPerCategory(A, T, S, N);

            % Check if this scenario is the biggest so far
            if total_variables > biggest_total_variables
                biggest_basic_constraints_variables = basic_constraints_variables;
                biggest_time_related_variables = time_related_variables;
                biggest_time_related_linearization_variables = time_related_linearization_variables;
                biggest_time_coordination_variables = time_coordination_variables;
                biggest_time_coordination_linearization_variables = time_coordination_linearization_variables;
                biggest_binary_variables = binary_variables;
                biggest_integer_variables = integer_variables;
                biggest_real_variables = real_variables;
                biggest_total_variables = total_variables;
                biggest_basic_constraints = basic_constraints;
                biggest_time_related_constraints = time_related_constraints;
                biggest_time_related_linearization_constraints = time_related_linearization_constraints;
                biggest_time_coordination_constraints = time_coordination_constraints;
                biggest_time_coordination_linearization_constraints = time_coordination_linearization_constraints;
                biggest_total_constraints = total_constraints;
            end

            % Add this scenario's numbers
            all_basic_constraints_variables = [all_basic_constraints_variables, basic_constraints_variables];
            all_time_related_variables = [all_time_related_variables, time_related_variables];
            all_time_related_linearization_variables = [all_time_related_linearization_variables, time_related_linearization_variables];
            all_time_coordination_variables = [all_time_coordination_variables, time_coordination_variables];
            all_time_coordination_linearization_variables = [all_time_coordination_linearization_variables, time_coordination_linearization_variables];
            all_binary_variables = [all_binary_variables, binary_variables];
            all_integer_variables = [all_integer_variables, integer_variables];
            all_real_variables = [all_real_variables, real_variables];
            all_total_variables = [all_total_variables, total_variables];
            all_basic_constraints = [all_basic_constraints, basic_constraints];
            all_time_related_constraints = [all_time_related_constraints, time_related_constraints];
            all_time_related_linearization_constraints = [all_time_related_linearization_constraints, time_related_linearization_constraints];
            all_time_coordination_constraints = [all_time_coordination_constraints, time_coordination_constraints];
            all_time_coordination_linearization_constraints = [all_time_coordination_linearization_constraints, time_coordination_linearization_constraints];
            all_total_constraints = [all_total_constraints, total_constraints];
        end
        % TODO: Join Binary and Integer in 1, then group them with Real variables into a single row in a tuple in % units
        % TODO: Sort scenario sizes by total number of variables

        % Add the min values for this scenario size to the data
        min_values = [min_values, [num_agents, num_tasks, min(all_basic_constraints_variables), min(all_time_related_variables + all_time_related_linearization_variables), min(all_time_coordination_variables + all_time_coordination_linearization_variables), round(100 * min(all_time_related_linearization_variables + all_time_coordination_linearization_variables) / min(all_total_variables), 2), round(100 * min(all_binary_variables + all_integer_variables) / min(all_total_variables), 2), round(100 * min(all_real_variables) / min(all_total_variables), 2), min(all_total_variables), min(all_basic_constraints), min(all_time_related_constraints + all_time_related_linearization_constraints), min(all_time_coordination_constraints + all_time_coordination_linearization_constraints), round(100 * min(all_time_related_linearization_constraints + all_time_coordination_linearization_constraints) / min(all_total_constraints), 2), min(all_total_constraints)]'];

        % Add the average values for this scenario size to the data
        av_values = [av_values, [num_agents, num_tasks, round(mean(all_basic_constraints_variables), 2), round(mean(all_time_related_variables + all_time_related_linearization_variables), 2), round(mean(all_time_coordination_variables + all_time_coordination_linearization_variables), 2), round(100 * mean(all_time_related_linearization_variables + all_time_coordination_linearization_variables) / mean(all_total_variables), 2), round(100 * mean(all_binary_variables + all_integer_variables) / mean(all_total_variables), 2), round(100 * mean(all_real_variables) / mean(all_total_variables), 2), round(mean(all_total_variables), 2), round(mean(all_basic_constraints), 2), round(mean(all_time_related_constraints + all_time_related_linearization_constraints), 2), round(mean(all_time_coordination_constraints + all_time_coordination_linearization_constraints), 2), round(100 * mean(all_time_related_linearization_constraints + all_time_coordination_linearization_constraints) / mean(all_total_constraints), 2), round(mean(all_total_constraints), 2)]'];

        % Add the max values for this scenario size to the data
        max_values = [max_values, [num_agents, num_tasks, max(all_basic_constraints_variables), max(all_time_related_variables + all_time_related_linearization_variables), max(all_time_coordination_variables + all_time_coordination_linearization_variables), round(100 * max(all_time_related_linearization_variables + all_time_coordination_linearization_variables) / max(all_total_variables), 2), round(100 * max(all_binary_variables + all_integer_variables) / max(all_total_variables), 2), round(100 * max(all_real_variables) / max(all_total_variables), 2), max(all_total_variables), max(all_basic_constraints), max(all_time_related_constraints + all_time_related_linearization_constraints), max(all_time_coordination_constraints + all_time_coordination_linearization_constraints), round(100 * max(all_time_related_linearization_constraints + all_time_coordination_linearization_constraints) / max(all_total_constraints), 2), max(all_total_constraints)]'];

        % Add the biggest values for this scenario size to the data
        biggest_values = [biggest_values, [num_agents, num_tasks, biggest_basic_constraints_variables, biggest_time_related_variables + biggest_time_related_linearization_variables, biggest_time_coordination_variables + biggest_time_coordination_linearization_variables, round(100 * (biggest_time_related_linearization_variables + biggest_time_coordination_linearization_variables) / biggest_total_variables, 2), round(100 * (biggest_binary_variables + biggest_integer_variables) / biggest_total_variables, 2), round(100 * biggest_real_variables / biggest_total_variables, 2), biggest_total_variables, biggest_basic_constraints, biggest_time_related_constraints + biggest_time_related_linearization_constraints, biggest_time_coordination_constraints + biggest_time_coordination_linearization_constraints, round(100 * (biggest_time_related_linearization_constraints + biggest_time_coordination_linearization_constraints) / biggest_total_constraints, 2), biggest_total_constraints]'];
    end

    % Save the results to a .csv file
    rowNames = {'Scenario size n'; 'Scenario size m'; 'Basic variables'; 'Time related variables'; 'Time coordination variables'; 'Linearization variables (%)'; 'Integer variables (%)'; 'Real variables (%)'; 'Total variables'; 'Basic constraints'; 'Time related constraints'; 'Time coordination constraints'; 'Linearization constraints (%)'; 'Total constraints'};
    
    min_table = array2table(min_values);
    av_table = array2table(av_values);
    max_table = array2table(max_values);
    biggest_table = array2table(biggest_values);

    min_table.Properties.RowNames = rowNames;
    av_table.Properties.RowNames = rowNames;
    max_table.Properties.RowNames = rowNames;
    biggest_table.Properties.RowNames = rowNames;

    writetable(min_table, '../mat/min_table.csv', 'WriteRowNames', true);
    writetable(av_table, '../mat/av_table.csv', 'WriteRowNames', true);
    writetable(max_table, '../mat/max_table.csv', 'WriteRowNames', true);
    writetable(biggest_table, '../mat/biggest_table.csv', 'WriteRowNames', true);

    disp('Results saved to CSV files.')
end

% Get variables numbers
function [basic_constraints_variables, ...
          time_related_variables, ...
          time_related_linearization_variables, ...
          time_coordination_variables, ...
          time_coordination_linearization_variables, ...
          binary_variables, ...
          integer_variables, ...
          real_variables, ...
          total_variables] = getVariablesNumbersPerCategory(A, T, S, N)

    % Length of each decision variable
    length_z                   = 1;               % (Real)    first part of the objective function (option 1). Minimize z with z >= tfin(a,S) for all a in A. i.e. tfin(a,S) - z <= 0.
    length_x_a_t_s             = A*(T+1)*(S+1);   % (Binary)  main decision variable. It's the solution itself. 1 if task t is assigned to slot s of Agent a. 0 otherwise.
    length_xx_a_t_t_s          = A*(T+1)*T*S;     % (Binary)  aux decision variable to linearize the product of x_a_t_s and x_a_t_s
    length_V_t                 = T;               % (Real)    vacancies. difference between the number of recruiters and the number of recruiters required for task t.
    length_n_t                 = T;               % (Natural) total number of times that task t (is queued) appears among all queues.
    length_na_t                = T;               % (Natural) number of Agents that are selected to carry out task t. Number of agents that are executing task t simultaneously.
    length_nq_t                = T;               % (Natural) number of different queues that task $t$ is queued in.
    length_nq_a_t              = A*T;             % (Binary)  aux binary variable. 1 if task t appears in Agent a's queue. 0 otherwise.
    length_nf_t                = T;               % (Natural) decision variable, solver's DoF. Number of fragments that task t is divided into. Used to get the Te_t_nf of a task.
    length_nf_t_nf             = T*N;             % (Binary)  aux decision variable to have a binary codification of nf_t.
    length_nfx_a_nf_t_s        = A*N*T*S;         % (Binary)  aux decision variable to linearize the product of nf_t_nf and x_a_t_s.
    length_naf_t_nf            = T*N;             % (Natural) aux decision variable to linearize the product of na_t and nf_t_nf.
    length_nax_a_t_s           = A*T*S;           % (Natural) aux decision variable to linearize the product of na_t and x_a_t_s.
    length_Ft_a_s              = A*(S+1);         % (Real)    accumulated flight time consumed by agent a until slot s.
    length_Ftx_a_s1            = A*S;             % (Real)    aux decision variable to linearize the product of x_a_1_s1 and Ft_a_s1.
    length_tfin_a_s            = A*(S+1);         % (Real)    time at with slot s of agent a ends it execution.
    length_tfinx_a_t_s         = A*T*S;           % (Real)    aux decision variable to linearize the product of tfin_a_s and x_a_t_s.
    length_d_tmax_tfin_a_s     = A*S;             % (Real)    difference between tmax_a_s and tfin_a_s. Third part of the objective function. Has a lower limit equal to 0.
    length_s_used              = 1;               % (Natural) number of slots used in total.
    length_Td_a_s              = A*S;             % (Real)    time that agent a spends traveling from its previous location to the slot s task location.
    length_Tw_a_s              = A*S;             % (Real)    time that the task in slot s of agent a must wait after Td and before Te to synch its execution with the other Agent's tasks.
    length_Twx_a_s             = A*S;             % (Real)    aux decision variable to linearize the product of Tw_a_s and x_a_t_s.
    length_Te_a_s              = A*S;             % (Real)    time that the task in slot s of agent a takes to execute.
    length_S_t_a1_s1_a2_s2     = (T-1)*A*S*A*S;   % (Binary)  Synch DV. Indicates whether there's a task t synchronization between (a1,s1) and (a2,s2) or not. 1 if there is. 0 otherwise.
    length_tfinS_t_a1_s1_a2_s2 = 2*(T-1)*A*S*A*S; % (Real)    aux decision variable to linearize de product of S_t_a1_s1_a2_s2 and tfin_a_s. Note that S_t_a1_s1_a2_s2 = S_t_a2_s2_a1_s1.
    length_R_t_a1_s1_a2_s2     = (T-1)*A*S*A*S;   % (Binary)  Relays' DV. Indicates whether there's a task t relay between (a1,s1) and (a2,s2) or not. 1 if there is. 0 otherwise.
    length_tfinR_t_a1_s1_a2_s2 = 2*(T-1)*A*S*A*S; % (Real)    aux decision variable to linearize de product of R_t_a1_s1_a2_s2 and tfin_a1_s1. Note that R_t_a1_s1_a2_s2 != R_t_a2_s2_a1_s1.
    length_TeR_t_a1_s1_a2_s2   = (T-1)*A*S*A*S;   % (Real)    aux decision variable to linearize de product of R_t_a1_s1_a2_s2 and Te_a2_s2. Note that R_t_a1_s1_a2_s2 != R_t_a2_s2_a1_s1.

    % Group the variables by category
    basic_constraints_variables = length_z + length_x_a_t_s + length_V_t + length_n_t + length_na_t + length_nq_t + length_nq_a_t + length_nf_t + length_nf_t_nf + length_s_used;
    time_related_variables = length_Ft_a_s + length_tfin_a_s + length_d_tmax_tfin_a_s + length_Td_a_s + length_Tw_a_s + length_Te_a_s;
    time_related_linearization_variables = length_xx_a_t_t_s + length_nfx_a_nf_t_s + length_Ftx_a_s1 + length_tfinx_a_t_s + length_Twx_a_s;
    time_coordination_variables = length_S_t_a1_s1_a2_s2 + length_R_t_a1_s1_a2_s2;
    time_coordination_linearization_variables = length_naf_t_nf + length_nax_a_t_s + length_tfinS_t_a1_s1_a2_s2 + length_tfinR_t_a1_s1_a2_s2 + length_TeR_t_a1_s1_a2_s2;
    binary_variables = length_x_a_t_s + length_xx_a_t_t_s + length_nq_a_t + length_nf_t_nf + length_nfx_a_nf_t_s + length_S_t_a1_s1_a2_s2 + length_R_t_a1_s1_a2_s2;
    integer_variables = length_n_t + length_na_t + length_nq_t + length_nf_t + length_naf_t_nf + length_nax_a_t_s + length_s_used;
    real_variables = length_z + length_V_t + length_Ft_a_s + length_Ftx_a_s1 + length_tfin_a_s + length_tfinx_a_t_s + length_d_tmax_tfin_a_s + length_Td_a_s + length_Tw_a_s + length_Twx_a_s + length_Te_a_s + length_tfinS_t_a1_s1_a2_s2 + length_tfinR_t_a1_s1_a2_s2 + length_TeR_t_a1_s1_a2_s2;
    total_variables = length_z + length_x_a_t_s + length_xx_a_t_t_s + length_V_t + length_n_t + length_na_t + length_nq_t + length_nq_a_t + length_nf_t + length_nf_t_nf + length_nfx_a_nf_t_s + length_naf_t_nf + length_nax_a_t_s + length_Ft_a_s + length_Ftx_a_s1 + length_tfin_a_s + length_tfinx_a_t_s + length_d_tmax_tfin_a_s + length_s_used + length_Td_a_s + length_Tw_a_s + length_Twx_a_s + length_Te_a_s + length_S_t_a1_s1_a2_s2 + length_tfinS_t_a1_s1_a2_s2 + length_R_t_a1_s1_a2_s2 + length_tfinR_t_a1_s1_a2_s2 + length_TeR_t_a1_s1_a2_s2;
end

% Get constraints numbers
function [basic_constraints, ...
          time_related_constraints, ...
          time_related_linearization_constraints, ...
          time_coordination_constraints, ...
          time_coordination_linearization_constraints, ...
          total_constraints] = getConstraintsNumbersPerCategory(A, T, S, N)

    % Maximum number of equations per decision variable
    N_Hard_eq            = (T - 1) * 1;
    Non_decomposable_eq  = (T - 1) * 1;
    Td_a_s_eq            = (A * S) * 1;
    Te_a_s_eq            = (A * S) * 1;
    z_ineq               = A * 1;
    n_t_eq               = T * 1;
    nf_t_nf_eq           = T * (1 + 1);
    nq_t_eq              = T * 1;
    nq_a_t_ineq          = (A * T) * (S * 1 + 1);
    na_nq_t_ineq         = T * 1;
    na_nf_n_t_eq         = T * 1;
    naf_t_nf_ineq        = (T * N) * 4;
    d_tmax_tfin_a_s_ineq = (A * S) * 1;
    t_fin_a_s_eq         = A * (1 + S * 1);
    Ft_a_s_ineq          = (A * S) * 1;
    Ft_a_s_eq            = A * (1 + S * 1);
    V_t_eq               = (T - 1) * 1;
    recharge_ineq        = A * 1 * (S - 1) * 1;
    hardware_ineq        = A * T * S * 1;
    max1task_ineq        = A * S * 1;
    continuity_ineq      = A * (S - 1) * 1;
    s_used_eq            = 1;
    synch_eq             = ((T - 1) * A * S) * ((A * S) * (1 + 1) + 1);
    synch_ineq           = ((T - 1) * A * S * A * S) * (1 + 1 + 8);
    relays_eq            = (T - 1) * ((A * S * A * S) * 1 + 1);
    relays_ineq          = ((T - 1) * A * S) * ((A * S) * (1 + 1 + 12) + 1 + 1);
    linearizations_ineq  = (A * S * (T * ((T + 1) * 3 + N * 3 + 8) + 8));

    % Group the constraints by category
    basic_constraints = N_Hard_eq + Non_decomposable_eq + z_ineq + n_t_eq + nf_t_nf_eq + nq_t_eq + nq_a_t_ineq + na_nq_t_ineq + na_nf_n_t_eq + V_t_eq + recharge_ineq + hardware_ineq + max1task_ineq + continuity_ineq + s_used_eq;
    time_related_constraints = Td_a_s_eq + Te_a_s_eq + d_tmax_tfin_a_s_ineq + t_fin_a_s_eq + Ft_a_s_ineq + Ft_a_s_eq;
    time_related_linearization_constraints = linearizations_ineq;
    time_coordination_constraints = synch_eq + relays_eq;
    time_coordination_linearization_constraints = naf_t_nf_ineq + synch_ineq + relays_ineq;
    total_constraints = N_Hard_eq + Non_decomposable_eq + Td_a_s_eq + Te_a_s_eq + z_ineq + n_t_eq + nf_t_nf_eq + nq_t_eq + nq_a_t_ineq + na_nq_t_ineq + na_nf_n_t_eq + naf_t_nf_ineq + d_tmax_tfin_a_s_ineq + t_fin_a_s_eq + Ft_a_s_ineq + Ft_a_s_eq + V_t_eq + recharge_ineq + hardware_ineq + max1task_ineq + continuity_ineq + s_used_eq + synch_eq + synch_ineq + relays_eq + relays_ineq + linearizations_ineq;
end