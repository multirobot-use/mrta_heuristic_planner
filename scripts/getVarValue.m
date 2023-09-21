function [var_value] = getVarValue(display_flag, sol, A, N, T, S, dv_start_length, var_name, idx)
    switch nargin
        case 8
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
        case 9
            start_length = dv_start_length(var_name);
            start = start_length(1);
            length = start_length(2);

            switch var_name
                case 'z'
                    var_value = sol(start);
                case 't_total_a'
                    var_value = sol((start - 1) + idx(1));
                case 'x_a_t_s'
                    var_value = sol((start - 1) + sub2ind([A, (T+1), (S+1)], idx(1), idx(2) + 1, idx(3) + 1));
                case 'xx_a_t_t_s'
                    var_value = sol((start - 1) + sub2ind([A, (T+1), T, S], idx(1), idx(2) + 1, idx(3), idx(4)));
                case 'V_t'
                    var_value = sol((start - 1) + idx(1));
                case 'U_t'
                    var_value = sol((start - 1) + idx(1));
                case 'n_t'
                    var_value = sol((start - 1) + idx(1));
                case 'na_t'
                    var_value = sol((start - 1) + idx(1));
                case 'nq_t'
                    var_value = sol((start - 1) + idx(1));
                case 'nq_a_t'
                    var_value = sol((start - 1) + sub2ind([A, T], idx(1), idx(2)));
                case 'nf_t'
                    var_value = sol((start - 1) + idx(1));
                case 'nf_t_nf'
                    var_value = sol((start - 1) + sub2ind([T, N], idx(1), idx(2)));
                case 'nfx_a_nf_t_s'
                    var_value = sol((start - 1) + sub2ind([A, N, T, S], idx(1), idx(2), idx(3), idx(4)));
                case 'naf_t_nf'
                    var_value = sol((start - 1) + sub2ind([T, N], idx(1), idx(2)));
                case 'nax_a_t_s'
                    var_value = sol((start - 1) + sub2ind([A, T, S], idx(1), idx(2), idx(3)));
                case 'Ft_a_s'
                    var_value = sol((start - 1) + sub2ind([A, (S+1)], idx(1), idx(2) + 1));
                case 'Ftx_a_s1'
                    var_value = sol((start - 1) + sub2ind([A, S], idx(1), idx(2)));
                case 'tfin_a_s'
                    var_value = sol((start - 1) + sub2ind([A, (S+1)], idx(1), idx(2) + 1));
                case 'tfinx_a_t_s'
                    var_value = sol((start - 1) + sub2ind([A, T, S], idx(1), idx(2), idx(3)));
                case 'd_tmax_tfin_a_s'
                    var_value = sol((start - 1) + sub2ind([A, S], idx(1), idx(2)));
                case 's_used'
                    var_value = sol(start);
                case 'Td_a_s'
                    var_value = sol((start - 1) + sub2ind([A, S], idx(1), idx(2)));
                case 'Tw_a_s'
                    var_value = sol((start - 1) + sub2ind([A, S], idx(1), idx(2)));
                case 'Te_a_s'
                    var_value = sol((start - 1) + sub2ind([A, S], idx(1), idx(2)));
                case 'S_t_a1_s1_a2_s2'
                    var_value = sol((start - 1) + sub2ind([(T-1), A, S, A, S], idx(1) - 1, idx(2), idx(3), idx(4), idx(5)))
                case 'tfinS_t_a1_s1_a2_s2'
                    var_value = sol((start - 1) + sub2ind([2, (T-1), A, S, A, S], idx(1), idx(2) - 1, idx(3), idx(4), idx(5), idx(6)));
                case 'R_t_a1_s1_a2_s2'
                    var_value = sol((start - 1) + sub2ind([(T-1), A, S, A, S], idx(1) - 1, idx(2), idx(3), idx(4), idx(5)));
                case 'tfinR_t_a1_s1_a2_s2'
                    var_value = sol((start - 1) + sub2ind([2, (T-1), A, S, A, S], idx(1), idx(2) - 1, idx(3), idx(4), idx(5), idx(6)));
                case 'TeR_t_a1_s1_a2_s2'
                    var_value = sol((start - 1) + sub2ind([(T-1), A, S, A, S], idx(1) - 1, idx(2), idx(3), idx(4), idx(5)));
                otherwise
                    error('var_name not recognized');
            end
        otherwise
            error('Incorrect num_args');
    end
    if display_flag
        display(num2str(var_value));
    end
end