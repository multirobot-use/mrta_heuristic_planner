function [max_dv_sparse_terms ...
          max_eq   max_eq_sparse_terms   max_eq_independent_sparse_terms   ...
          max_ineq max_ineq_sparse_terms max_ineq_independent_sparse_terms ...
          start_N_Hard_eq            ...
          start_Non_decomposable_eq  ...
          start_Td_a_s_eq            ...
          start_Te_a_s_eq            ...
          start_n_t_eq               ...
          start_nf_t_nf_eq           ...
          start_nq_t_eq              ...
          start_na_nf_n_t_eq         ...
          start_t_fin_a_s_eq         ...
          start_Ft_a_s_eq            ...
          start_V_t_eq               ...
          start_s_used_eq            ...
          start_synch_eq             ...
          start_relays_eq            ...
          start_z_ineq               ...
          start_nq_a_t_ineq          ...
          start_na_nq_t_ineq         ...
          start_naf_t_nf_ineq        ...
          start_d_tmax_tfin_a_s_ineq ...
          start_Ft_a_s_ineq          ...
          start_recharge_ineq        ...
          start_hardware_ineq        ...
          start_max1task_ineq        ...
          start_continuity_ineq      ...
          start_synch_ineq           ...
          start_relays_ineq          ...
          start_linearizations_ineq] = getSparseMatrixInformation(Agent, Task)
    
    % Get constant scenario values
    [Agent, Task, A, T, S, N, R, Td_a_t_t, Te_t_nf, H_a_t] = getConstantScenarioValues(Agent, Task);

    % Maximum number of sparse terms in the decision variable array per decision variable
    z_sparse_terms                   = 1;
    x_a_t_s_sparse_terms             = A*(S+1);
    xx_a_t_t_s_sparse_terms          = A*S;
    V_t_sparse_terms                 = T;
    n_t_sparse_terms                 = T;
    na_t_sparse_terms                = T;
    nq_t_sparse_terms                = T;
    nq_a_t_sparse_terms              = A*T;
    nf_t_sparse_terms                = T;
    nf_t_nf_sparse_terms             = T;
    nfx_a_nf_t_s_sparse_terms        = A*S;
    naf_t_nf_sparse_terms            = T;
    nax_a_t_s_sparse_terms           = A*S;
    Ft_a_s_sparse_terms              = A*(S+1);
    Ftx_a_s1_sparse_terms            = A*S;
    tfin_a_s_sparse_terms            = A*(S+1);
    tfinx_a_t_s_sparse_terms         = A*S;
    d_tmax_tfin_a_s_sparse_terms     = A*S;
    s_used_sparse_terms              = 1;
    Td_a_s_sparse_terms              = A*S;
    Tw_a_s_sparse_terms              = A*S;
    Twx_a_s_sparse_terms             = A*S;
    Te_a_s_sparse_terms              = A*S;
    S_t_a1_s1_a2_s2_sparse_terms     = (T-1)*A*S*(A-1);
    tfinS_t_a1_s1_a2_s2_sparse_terms = 2*(T-1)*A*S*(A-1);
    R_t_a1_s1_a2_s2_sparse_terms     = (T-1)*(A*N-1);
    tfinR_t_a1_s1_a2_s2_sparse_terms = 2*(T-1)*(A*N-1);
    TeR_t_a1_s1_a2_s2_sparse_terms   = (T-1)*(A*N-1);

    % Maximum number of sparse terms in the decision variable array
    max_dv_sparse_terms = z_sparse_terms + x_a_t_s_sparse_terms + xx_a_t_t_s_sparse_terms + V_t_sparse_terms + n_t_sparse_terms + na_t_sparse_terms + nq_t_sparse_terms + nq_a_t_sparse_terms + nf_t_sparse_terms + nf_t_nf_sparse_terms + nfx_a_nf_t_s_sparse_terms + naf_t_nf_sparse_terms + nax_a_t_s_sparse_terms + Ft_a_s_sparse_terms + Ftx_a_s1_sparse_terms + tfin_a_s_sparse_terms + tfinx_a_t_s_sparse_terms + d_tmax_tfin_a_s_sparse_terms + s_used_sparse_terms + Td_a_s_sparse_terms + Tw_a_s_sparse_terms + Twx_a_s_sparse_terms + Te_a_s_sparse_terms + S_t_a1_s1_a2_s2_sparse_terms + tfinS_t_a1_s1_a2_s2_sparse_terms + R_t_a1_s1_a2_s2_sparse_terms + tfinR_t_a1_s1_a2_s2_sparse_terms + TeR_t_a1_s1_a2_s2_sparse_terms;

    % Maximum number of sparse terms in the matrixes per decision variable
    N_Hard_eq_sparse_terms            = (T - 1) * 1;
    Non_decomposable_eq_sparse_terms  = (T - 1) * 1;
    Td_a_s_eq_sparse_terms            = (A * S) * (1 + (T * (T + 1)) * 1);
    Te_a_s_eq_sparse_terms            = (A * S) * (1 + (T * N) * 1);
    z_ineq_sparse_terms               = A * 2;
    n_t_eq_sparse_terms               = T * (1 + (A * S) * 1);
    nf_t_nf_eq_sparse_terms           = T * ((1 + N * 1) + N * 1);
    nq_t_eq_sparse_terms              = T * (1 + A * 1);
    nq_a_t_ineq_sparse_terms          = (A * T) * (S * 2 + (1 + S * 1));
    na_nq_t_ineq_sparse_terms         = T * 2;
    na_nf_n_t_eq_sparse_terms         = T * (1 + N * 1);
    naf_t_nf_ineq_sparse_terms        = (T * N) * 10;
    d_tmax_tfin_a_s_ineq_sparse_terms = (A * S) * (T * 2 + 1);
    t_fin_a_s_eq_sparse_terms         = A * (1 + S * (4 + T * 1));
    Ft_a_s_ineq_sparse_terms          = (A * S) * 1;
    Ft_a_s_eq_sparse_terms            = A * (1 + S * (6 + (T - 1) * N * 1));
    V_t_eq_sparse_terms               = (T - 1) * 2;
    recharge_ineq_sparse_terms        = A * 1 * (S - 1) * 2;
    hardware_ineq_sparse_terms        = A * T * S * 1;
    max_1_ineq_sparse_terms           = A * S * 1 * (T+1);
    continuity_ineq_sparse_terms      = A * (S - 1) * 2 * T;
    s_used_eq_sparse_terms            = 1 + A * (T+1) * (S+1);
    synch_eq_sparse_terms             = ((T - 1) * A * S) * ((A * S) * (2 + 2) + (2 + (A * S) * 1));
    synch_ineq_sparse_terms           = ((T - 1) * A * S * A * S) * (2 + 2 + 20);
    relays_eq_sparse_terms            = (T - 1) * (2 + (A * S * A * S) * (3 + 1));
    relays_ineq_sparse_terms          = ((T - 1) * A * S * A * S) * (2 + 2 + 30 + 1 + 1);
    linearizations_ineq_sparse_terms  = (A * S * (T * ((T + 1) * 7 + N * 7 + 20) + 20));

    % Max number of sparse terms in the matrixes
    max_eq_sparse_terms = N_Hard_eq_sparse_terms + Non_decomposable_eq_sparse_terms + Td_a_s_eq_sparse_terms + Te_a_s_eq_sparse_terms + n_t_eq_sparse_terms + nf_t_nf_eq_sparse_terms + nq_t_eq_sparse_terms + na_nf_n_t_eq_sparse_terms + t_fin_a_s_eq_sparse_terms + Ft_a_s_eq_sparse_terms + V_t_eq_sparse_terms + s_used_eq_sparse_terms + synch_eq_sparse_terms + relays_eq_sparse_terms;
    max_ineq_sparse_terms = z_ineq_sparse_terms + nq_a_t_ineq_sparse_terms + na_nq_t_ineq_sparse_terms + naf_t_nf_ineq_sparse_terms + d_tmax_tfin_a_s_ineq_sparse_terms + Ft_a_s_ineq_sparse_terms + recharge_ineq_sparse_terms + hardware_ineq_sparse_terms + max_1_ineq_sparse_terms + continuity_ineq_sparse_terms + synch_ineq_sparse_terms + relays_ineq_sparse_terms + linearizations_ineq_sparse_terms;

    % Maximum number of independent terms in the matrixes per decision variable
    N_Hard_eq_independent_sparse_terms            = 0;
    Non_decomposable_eq_independent_sparse_terms  = (T - 1) * 1;
    Td_a_s_eq_independent_sparse_terms            = 0;
    Te_a_s_eq_independent_sparse_terms            = 0;
    z_ineq_independent_sparse_terms               = 0;
    n_t_eq_independent_sparse_terms               = 0;
    nf_t_nf_eq_independent_sparse_terms           = T * 1;
    nq_t_eq_independent_sparse_terms              = 0;
    nq_a_t_ineq_independent_sparse_terms          = 0;
    na_nq_t_ineq_independent_sparse_terms         = 0;
    na_nf_n_t_eq_independent_sparse_terms         = 0;
    naf_t_nf_ineq_independent_sparse_terms        = (T * N) * 2;
    d_tmax_tfin_a_s_ineq_independent_sparse_terms = 0;
    t_fin_a_s_eq_independent_sparse_terms         = 0;
    Ft_a_s_ineq_independent_sparse_terms          = (A * S) * 1;
    Ft_a_s_eq_independent_sparse_terms            = A * 1;
    V_t_eq_independent_sparse_terms               = (T - 1) * 1;
    recharge_ineq_independent_sparse_terms        = A * 1 * (S - 1) * 1;
    hardware_ineq_independent_sparse_terms        = A * T * S * 1;
    max_ineq_independent_sparse_terms             = A * S * 1;
    continuity_ineq_independent_sparse_terms      = 0;
    s_used_eq_independent_sparse_terms            = 1;
    synch_eq_independent_sparse_terms             = 0;
    synch_ineq_independent_sparse_terms           = ((T - 1) * A * S * A * S) * 4;
    relays_eq_independent_sparse_terms            = 0;
    relays_ineq_independent_sparse_terms          = ((T - 1) * A * S) * ((A * S) * 6 + 2);
    linearizations_ineq_independent_sparse_terms  = (A * S * (T * ((T + 1) * 1 + N * 1 + 4) + 4));

    % Max number of sparse terms in the independent terms
    max_eq_independent_sparse_terms = N_Hard_eq_independent_sparse_terms + Non_decomposable_eq_independent_sparse_terms + Td_a_s_eq_independent_sparse_terms + Te_a_s_eq_independent_sparse_terms + n_t_eq_independent_sparse_terms + nf_t_nf_eq_independent_sparse_terms + nq_t_eq_independent_sparse_terms + na_nf_n_t_eq_independent_sparse_terms + t_fin_a_s_eq_independent_sparse_terms + Ft_a_s_eq_independent_sparse_terms + V_t_eq_independent_sparse_terms + s_used_eq_independent_sparse_terms + synch_eq_independent_sparse_terms + relays_eq_independent_sparse_terms;
    max_ineq_independent_sparse_terms = z_ineq_independent_sparse_terms + nq_a_t_ineq_independent_sparse_terms + na_nq_t_ineq_independent_sparse_terms + naf_t_nf_ineq_independent_sparse_terms + d_tmax_tfin_a_s_ineq_independent_sparse_terms + Ft_a_s_ineq_independent_sparse_terms + recharge_ineq_independent_sparse_terms + hardware_ineq_independent_sparse_terms + max_ineq_independent_sparse_terms + continuity_ineq_independent_sparse_terms + synch_ineq_independent_sparse_terms + relays_ineq_independent_sparse_terms + linearizations_ineq_independent_sparse_terms;

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

    % Max number of equations (rows in A and Aeq)
    max_eq = N_Hard_eq + Non_decomposable_eq + Td_a_s_eq + Te_a_s_eq + n_t_eq + nf_t_nf_eq + nq_t_eq + na_nf_n_t_eq + t_fin_a_s_eq + Ft_a_s_eq + V_t_eq + s_used_eq + synch_eq + relays_eq;
    max_ineq = z_ineq + nq_a_t_ineq + na_nq_t_ineq + naf_t_nf_ineq + d_tmax_tfin_a_s_ineq + Ft_a_s_ineq + recharge_ineq + hardware_ineq + max1task_ineq + continuity_ineq + synch_ineq + relays_ineq + linearizations_ineq;

    % Start index for each equation
    start_N_Hard_eq            = 1;
    start_Non_decomposable_eq  = start_N_Hard_eq            + N_Hard_eq;
    start_Td_a_s_eq            = start_Non_decomposable_eq  + Non_decomposable_eq;
    start_Te_a_s_eq            = start_Td_a_s_eq            + Td_a_s_eq;
    start_n_t_eq               = start_Te_a_s_eq            + Te_a_s_eq;
    start_nf_t_nf_eq           = start_n_t_eq               + n_t_eq;
    start_nq_t_eq              = start_nf_t_nf_eq           + nf_t_nf_eq;
    start_na_nf_n_t_eq         = start_nq_t_eq              + nq_t_eq;
    start_t_fin_a_s_eq         = start_na_nf_n_t_eq         + na_nf_n_t_eq;
    start_Ft_a_s_eq            = start_t_fin_a_s_eq         + t_fin_a_s_eq;
    start_V_t_eq               = start_Ft_a_s_eq            + Ft_a_s_eq;
    start_s_used_eq            = start_V_t_eq               + V_t_eq;
    start_synch_eq             = start_s_used_eq            + s_used_eq;
    start_relays_eq            = start_synch_eq             + synch_eq;

    start_z_ineq               = 1;
    start_nq_a_t_ineq          = start_z_ineq               + z_ineq;
    start_na_nq_t_ineq         = start_nq_a_t_ineq          + nq_a_t_ineq;
    start_naf_t_nf_ineq        = start_na_nq_t_ineq         + na_nq_t_ineq;
    start_d_tmax_tfin_a_s_ineq = start_naf_t_nf_ineq        + naf_t_nf_ineq;
    start_Ft_a_s_ineq          = start_d_tmax_tfin_a_s_ineq + d_tmax_tfin_a_s_ineq;
    start_recharge_ineq        = start_Ft_a_s_ineq          + Ft_a_s_ineq;
    start_hardware_ineq        = start_recharge_ineq        + recharge_ineq;
    start_max1task_ineq        = start_hardware_ineq        + hardware_ineq;
    start_continuity_ineq      = start_max1task_ineq        + max1task_ineq;
    start_synch_ineq           = start_continuity_ineq      + continuity_ineq;
    start_relays_ineq          = start_synch_ineq           + synch_ineq;
    start_linearizations_ineq  = start_relays_ineq          + relays_ineq;
end