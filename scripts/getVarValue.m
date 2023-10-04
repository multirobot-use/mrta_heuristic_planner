function [var_value] = getVarValue(sol, Agent, Task, var_name)
    % Get scenario information
    [A, T, S, N, R, Td_a_t_t, Te_t_nf, H_a_t] = getConstantScenarioValues(Agent, Task);
    [dv_start_length, length_dv] = getStartLengthInformation(Agent, Task);

    start_length = dv_start_length(var_name);
    start = start_length(1);
    length = start_length(2);

    var_value = sol(start:start+length-1)';

    switch var_name
        case 'z'
        case 'x_a_t_s'
            var_value = reshape(var_value, A, (T+1), (S+1));
        case 'xx_a_t_t_s'
            var_value = reshape(var_value, A, (T+1), T, S);
        case 'V_t'
        case 'U_t'
        case 'n_t'
        case 'na_t'
        case 'nq_t'
        case 'nq_a_t'
            var_value = reshape(var_value, A, T);
        case 'nf_t'
        case 'nf_t_nf'
            var_value = reshape(var_value, T, N);
        case 'nfx_a_nf_t_s'
            var_value = reshape(var_value, A, N, T, S);
        case 'naf_t_nf'
            var_value = reshape(var_value, T, N);
        case 'nax_a_t_s'
            var_value = reshape(var_value, A, T, S);
        case 'Ft_a_s'
            var_value = reshape(var_value, A, (S+1));
        case 'Ftx_a_s1'
            var_value = reshape(var_value, A, S);
        case 'tfin_a_s'
            var_value = reshape(var_value, A, (S+1));
        case 'tfinx_a_t_s'
            var_value = reshape(var_value, A, T, S);
        case 'd_tmax_tfin_a_s'
            var_value = reshape(var_value, A, S);
        case 's_used'
        case 'Td_a_s'
            var_value = reshape(var_value, A, S);
        case 'Tw_a_s'
            var_value = reshape(var_value, A, S);
        case 'Twx_a_s'
            var_value = reshape(var_value, A, S);
        case 'Te_a_s'
            var_value = reshape(var_value, A, S);
        case 'S_t_a1_s1_a2_s2'
            var_value = reshape(var_value, (T-1), A, S, A, S);
        case 'tfinS_t_a1_s1_a2_s2'
            var_value = reshape(var_value, 2, (T-1), A, S, A, S);
        case 'R_t_a1_s1_a2_s2'
            var_value = reshape(var_value, (T-1), A, S, A, S);
        case 'tfinR_t_a1_s1_a2_s2'
            var_value = reshape(var_value, 2, (T-1), A, S, A, S);
        case 'TeR_t_a1_s1_a2_s2'
            var_value = reshape(var_value, (T-1), A, S, A, S);
        otherwise
            error('var_name not recognized');
    end
end