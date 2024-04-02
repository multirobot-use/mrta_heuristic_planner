function [dv_start_length, length_dv] = getStartLengthInformation(Agent, Task)
    [Agent, Task, A, T, S, N, R, Td_a_t_t, Te_t_nf, H_a_t] = getConstantScenarioValues(Agent, Task);
    
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

    % Starting position of each decision variable in the decision variable vector:
    start_z                   = 1;
    start_x_a_t_s             = start_z                   + length_z;
    start_xx_a_t_t_s          = start_x_a_t_s             + length_x_a_t_s;
    start_V_t                 = start_xx_a_t_t_s          + length_xx_a_t_t_s;  
    start_n_t                 = start_V_t                 + length_V_t;
    start_na_t                = start_n_t                 + length_n_t;
    start_nq_t                = start_na_t                + length_na_t;
    start_nq_a_t              = start_nq_t                + length_nq_t;
    start_nf_t                = start_nq_a_t              + length_nq_a_t;
    start_nf_t_nf             = start_nf_t                + length_nf_t;
    start_nfx_a_nf_t_s        = start_nf_t_nf             + length_nf_t_nf;
    start_naf_t_nf            = start_nfx_a_nf_t_s        + length_nfx_a_nf_t_s;
    start_nax_a_t_s           = start_naf_t_nf            + length_naf_t_nf;
    start_Ft_a_s              = start_nax_a_t_s           + length_nax_a_t_s;
    start_Ftx_a_s1            = start_Ft_a_s              + length_Ft_a_s;
    start_tfin_a_s            = start_Ftx_a_s1            + length_Ftx_a_s1;
    start_tfinx_a_t_s         = start_tfin_a_s            + length_tfin_a_s;
    start_d_tmax_tfin_a_s     = start_tfinx_a_t_s         + length_tfinx_a_t_s;
    start_s_used              = start_d_tmax_tfin_a_s     + length_d_tmax_tfin_a_s;
    start_Td_a_s              = start_s_used              + length_s_used;
    start_Tw_a_s              = start_Td_a_s              + length_Td_a_s;
    start_Twx_a_s             = start_Tw_a_s              + length_Tw_a_s;
    start_Te_a_s              = start_Twx_a_s             + length_Twx_a_s;
    start_S_t_a1_s1_a2_s2     = start_Te_a_s              + length_Te_a_s;
    start_tfinS_t_a1_s1_a2_s2 = start_S_t_a1_s1_a2_s2     + length_S_t_a1_s1_a2_s2;
    start_R_t_a1_s1_a2_s2     = start_tfinS_t_a1_s1_a2_s2 + length_tfinS_t_a1_s1_a2_s2;
    start_tfinR_t_a1_s1_a2_s2 = start_R_t_a1_s1_a2_s2     + length_R_t_a1_s1_a2_s2;
    start_TeR_t_a1_s1_a2_s2   = start_tfinR_t_a1_s1_a2_s2 + length_tfinR_t_a1_s1_a2_s2;

    % Put all decision variable starting positions and lengths together in a dictionary using the var names as keys and the starting positions as values:
    dv_start_length = containers.Map({'z', 'x_a_t_s', 'xx_a_t_t_s', 'V_t', 'n_t', 'na_t', 'nq_t', 'nq_a_t', 'nf_t', 'nf_t_nf', 'nfx_a_nf_t_s', 'naf_t_nf', 'nax_a_t_s', 'Ft_a_s', 'Ftx_a_s1', 'tfin_a_s', 'tfinx_a_t_s', 'd_tmax_tfin_a_s', 's_used', 'Td_a_s', 'Tw_a_s', 'Twx_a_s', 'Te_a_s','S_t_a1_s1_a2_s2', 'tfinS_t_a1_s1_a2_s2', 'R_t_a1_s1_a2_s2', 'tfinR_t_a1_s1_a2_s2', 'TeR_t_a1_s1_a2_s2'}, {[start_z length_z], [start_x_a_t_s length_x_a_t_s], [start_xx_a_t_t_s length_xx_a_t_t_s], [start_V_t length_V_t], [start_n_t length_n_t], [start_na_t length_na_t], [start_nq_t length_nq_t], [start_nq_a_t length_nq_a_t], [start_nf_t length_nf_t], [start_nf_t_nf length_nf_t_nf], [start_nfx_a_nf_t_s length_nfx_a_nf_t_s], [start_naf_t_nf length_naf_t_nf], [start_nax_a_t_s length_nax_a_t_s], [start_Ft_a_s length_Ft_a_s], [start_Ftx_a_s1 length_Ftx_a_s1], [start_tfin_a_s length_tfin_a_s], [start_tfinx_a_t_s length_tfinx_a_t_s], [start_d_tmax_tfin_a_s length_d_tmax_tfin_a_s], [start_s_used length_s_used], [start_Td_a_s length_Td_a_s], [start_Tw_a_s length_Tw_a_s], [start_Twx_a_s length_Twx_a_s], [start_Te_a_s length_Te_a_s], [start_S_t_a1_s1_a2_s2 length_S_t_a1_s1_a2_s2], [start_tfinS_t_a1_s1_a2_s2 length_tfinS_t_a1_s1_a2_s2], [start_R_t_a1_s1_a2_s2 length_R_t_a1_s1_a2_s2], [start_tfinR_t_a1_s1_a2_s2 length_tfinR_t_a1_s1_a2_s2], [start_TeR_t_a1_s1_a2_s2 length_TeR_t_a1_s1_a2_s2]});

    % Compute the total length of the decision variable vector:
    length_dv = length_z + length_x_a_t_s + length_xx_a_t_t_s + length_V_t + length_n_t + length_na_t + length_nq_t + length_nq_a_t + length_nf_t + length_nf_t_nf + length_nfx_a_nf_t_s + length_naf_t_nf + length_nax_a_t_s + length_Ft_a_s + length_Ftx_a_s1 + length_tfin_a_s + length_tfinx_a_t_s + length_d_tmax_tfin_a_s + length_s_used + length_Td_a_s + length_Tw_a_s + length_Twx_a_s + length_Te_a_s + length_S_t_a1_s1_a2_s2 + length_tfinS_t_a1_s1_a2_s2 + length_R_t_a1_s1_a2_s2 + length_tfinR_t_a1_s1_a2_s2 + length_TeR_t_a1_s1_a2_s2;
end