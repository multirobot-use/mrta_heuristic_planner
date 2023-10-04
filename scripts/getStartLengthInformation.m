function [dv_start_length, length_dv] = getStartLengthInformation(Agent, Task)
    [A, T, S, N, R, Td_a_t_t, Te_t_nf, H_a_t] = getConstantScenarioValues(Agent, Task);
    
    % Length of each decision variable
    length_z                   = 1;
    length_x_a_t_s             = A*(T+1)*(S+1);
    length_xx_a_t_t_s          = A*(T+1)*T*S;
    length_V_t                 = T;
    length_U_t                 = T;
    length_n_t                 = T;
    length_na_t                = T;
    length_nq_t                = T;
    length_nq_a_t              = A*T;
    length_nf_t                = T;
    length_nf_t_nf             = T*N;
    length_nfx_a_nf_t_s        = A*N*T*S;
    length_naf_t_nf            = T*N;
    length_nax_a_t_s           = A*T*S;
    length_Ft_a_s              = A*(S+1);
    length_Ftx_a_s1            = A*S;
    length_tfin_a_s            = A*(S+1);
    length_tfinx_a_t_s         = A*T*S;
    length_d_tmax_tfin_a_s     = A*S;
    length_s_used              = 1;
    length_Td_a_s              = A*S;
    length_Tw_a_s              = A*S;
    length_Twx_a_s             = A*S;
    length_Te_a_s              = A*S;
    length_S_t_a1_s1_a2_s2     = (T-1)*A*S*A*S;
    length_tfinS_t_a1_s1_a2_s2 = 2*(T-1)*A*S*A*S;
    length_R_t_a1_s1_a2_s2     = (T-1)*A*S*A*S;
    length_tfinR_t_a1_s1_a2_s2 = 2*(T-1)*A*S*A*S;
    length_TeR_t_a1_s1_a2_s2   = (T-1)*A*S*A*S;

    % Starting position of each decision variable in the decision variable vector:
    start_z                   = 1;
    start_x_a_t_s             = start_z                   + length_z;
    start_xx_a_t_t_s          = start_x_a_t_s             + length_x_a_t_s;
    start_V_t                 = start_xx_a_t_t_s          + length_xx_a_t_t_s;
    start_U_t                 = start_V_t                 + length_V_t;
    start_n_t                 = start_U_t                 + length_U_t;
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
    dv_start_length = containers.Map({'z', 'x_a_t_s', 'xx_a_t_t_s', 'V_t', 'U_t', 'n_t', 'na_t', 'nq_t', 'nq_a_t', 'nf_t', 'nf_t_nf', 'nfx_a_nf_t_s', 'naf_t_nf', 'nax_a_t_s', 'Ft_a_s', 'Ftx_a_s1', 'tfin_a_s', 'tfinx_a_t_s', 'd_tmax_tfin_a_s', 's_used', 'Td_a_s', 'Tw_a_s', 'Twx_a_s', 'Te_a_s','S_t_a1_s1_a2_s2', 'tfinS_t_a1_s1_a2_s2', 'R_t_a1_s1_a2_s2', 'tfinR_t_a1_s1_a2_s2', 'TeR_t_a1_s1_a2_s2'}, {[start_z length_z], [start_x_a_t_s length_x_a_t_s], [start_xx_a_t_t_s length_xx_a_t_t_s], [start_V_t length_V_t], [start_U_t length_U_t], [start_n_t length_n_t], [start_na_t length_na_t], [start_nq_t length_nq_t], [start_nq_a_t length_nq_a_t], [start_nf_t length_nf_t], [start_nf_t_nf length_nf_t_nf], [start_nfx_a_nf_t_s length_nfx_a_nf_t_s], [start_naf_t_nf length_naf_t_nf], [start_nax_a_t_s length_nax_a_t_s], [start_Ft_a_s length_Ft_a_s], [start_Ftx_a_s1 length_Ftx_a_s1], [start_tfin_a_s length_tfin_a_s], [start_tfinx_a_t_s length_tfinx_a_t_s], [start_d_tmax_tfin_a_s length_d_tmax_tfin_a_s], [start_s_used length_s_used], [start_Td_a_s length_Td_a_s], [start_Tw_a_s length_Tw_a_s], [start_Twx_a_s length_Twx_a_s], [start_Te_a_s length_Te_a_s], [start_S_t_a1_s1_a2_s2 length_S_t_a1_s1_a2_s2], [start_tfinS_t_a1_s1_a2_s2 length_tfinS_t_a1_s1_a2_s2], [start_R_t_a1_s1_a2_s2 length_R_t_a1_s1_a2_s2], [start_tfinR_t_a1_s1_a2_s2 length_tfinR_t_a1_s1_a2_s2], [start_TeR_t_a1_s1_a2_s2 length_TeR_t_a1_s1_a2_s2]});

    % Compute the total length of the decision variable vector:
    length_dv = length_z + length_x_a_t_s + length_xx_a_t_t_s + length_V_t + length_U_t + length_n_t + length_na_t + length_nq_t + length_nq_a_t + length_nf_t + length_nf_t_nf + length_nfx_a_nf_t_s + length_naf_t_nf + length_nax_a_t_s + length_Ft_a_s + length_Ftx_a_s1 + length_tfin_a_s + length_tfinx_a_t_s + length_d_tmax_tfin_a_s + length_s_used + length_Td_a_s + length_Tw_a_s + length_Twx_a_s + length_Te_a_s + length_S_t_a1_s1_a2_s2 + length_tfinS_t_a1_s1_a2_s2 + length_R_t_a1_s1_a2_s2 + length_tfinR_t_a1_s1_a2_s2 + length_TeR_t_a1_s1_a2_s2;
end