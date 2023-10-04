function [start_z                   length_z                   ...
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
          start_TeR_t_a1_s1_a2_s2   length_TeR_t_a1_s1_a2_s2      ] = extractStartLengthInformation(dv_start_length)

    %? z
    start_length = dv_start_length('z');
    start_z = start_length(1);
    length_z = start_length(2);
    %? x_a_t_s
    start_length = dv_start_length('x_a_t_s');
    start_x_a_t_s = start_length(1);
    length_x_a_t_s = start_length(2);
    %? xx_a_t_t_s
    start_length = dv_start_length('xx_a_t_t_s');
    start_xx_a_t_t_s = start_length(1);
    length_xx_a_t_t_s = start_length(2);
    %? V_t
    start_length = dv_start_length('V_t');
    start_V_t = start_length(1);
    length_V_t = start_length(2);
    %? U_t
    start_length = dv_start_length('U_t');
    start_U_t = start_length(1);
    length_U_t = start_length(2);
    %? n_t
    start_length = dv_start_length('n_t');
    start_n_t = start_length(1);
    length_n_t = start_length(2);
    %? na_t
    start_length = dv_start_length('na_t');
    start_na_t = start_length(1);
    length_na_t = start_length(2);
    %? nq_t
    start_length = dv_start_length('nq_t');
    start_nq_t = start_length(1);
    length_nq_t = start_length(2);
    %? nq_a_t
    start_length = dv_start_length('nq_a_t');
    start_nq_a_t = start_length(1);
    length_nq_a_t = start_length(2);
    %? nf_t
    start_length = dv_start_length('nf_t');
    start_nf_t = start_length(1);
    length_nf_t = start_length(2);
    %? nf_t_nf
    start_length = dv_start_length('nf_t_nf');
    start_nf_t_nf = start_length(1);
    length_nf_t_nf = start_length(2);
    %? nfx_a_nf_t_s
    start_length = dv_start_length('nfx_a_nf_t_s');
    start_nfx_a_nf_t_s = start_length(1);
    length_nfx_a_nf_t_s = start_length(2);
    %? naf_t_nf
    start_length = dv_start_length('naf_t_nf');
    start_naf_t_nf = start_length(1);
    length_naf_t_nf = start_length(2);
    %? nax_a_t_s
    start_length = dv_start_length('nax_a_t_s');
    start_nax_a_t_s = start_length(1);
    length_nax_a_t_s = start_length(2);
    %? Ft_a_s
    start_length = dv_start_length('Ft_a_s');
    start_Ft_a_s = start_length(1);
    length_Ft_a_s = start_length(2);
    %? Ftx_a_s1
    start_length = dv_start_length('Ftx_a_s1');
    start_Ftx_a_s1 = start_length(1);
    length_Ftx_a_s1 = start_length(2);
    %? tfin_a_s
    start_length = dv_start_length('tfin_a_s');
    start_tfin_a_s = start_length(1);
    length_tfin_a_s = start_length(2);
    %? tfinx_a_t_s
    start_length = dv_start_length('tfinx_a_t_s');
    start_tfinx_a_t_s = start_length(1);
    length_tfinx_a_t_s = start_length(2);
    %? d_tmax_tfin_a_s
    start_length = dv_start_length('d_tmax_tfin_a_s');
    start_d_tmax_tfin_a_s = start_length(1);
    length_d_tmax_tfin_a_s = start_length(2);
    %? s_used
    start_length = dv_start_length('s_used');
    start_s_used = start_length(1);
    length_s_used = start_length(2);
    %? Td_a_s
    start_length = dv_start_length('Td_a_s');
    start_Td_a_s = start_length(1);
    length_Td_a_s = start_length(2);
    %? Tw_a_s
    start_length = dv_start_length('Tw_a_s');
    start_Tw_a_s = start_length(1);
    length_Tw_a_s = start_length(2);
    %? Twx_a_s
    start_length = dv_start_length('Twx_a_s');
    start_Twx_a_s = start_length(1);
    length_Twx_a_s = start_length(2);
    %? Te_a_s
    start_length = dv_start_length('Te_a_s');
    start_Te_a_s = start_length(1);
    length_Te_a_s = start_length(2);
    %? S_t_a1_s1_a2_s2
    start_length = dv_start_length('S_t_a1_s1_a2_s2');
    start_S_t_a1_s1_a2_s2 = start_length(1);
    length_S_t_a1_s1_a2_s2 = start_length(2);
    %? tfinS_t_a1_s1_a2_s2
    start_length = dv_start_length('tfinS_t_a1_s1_a2_s2');
    start_tfinS_t_a1_s1_a2_s2 = start_length(1);
    length_tfinS_t_a1_s1_a2_s2 = start_length(2);
    %? R_t_a1_s1_a2_s2
    start_length = dv_start_length('R_t_a1_s1_a2_s2');
    start_R_t_a1_s1_a2_s2 = start_length(1);
    length_R_t_a1_s1_a2_s2 = start_length(2);
    %? tfinR_t_a1_s1_a2_s2
    start_length = dv_start_length('tfinR_t_a1_s1_a2_s2');
    start_tfinR_t_a1_s1_a2_s2 = start_length(1);
    length_tfinR_t_a1_s1_a2_s2 = start_length(2);
    %? TeR_t_a1_s1_a2_s2
    start_length = dv_start_length('TeR_t_a1_s1_a2_s2');
    start_TeR_t_a1_s1_a2_s2 = start_length(1);
    length_TeR_t_a1_s1_a2_s2 = start_length(2);
end