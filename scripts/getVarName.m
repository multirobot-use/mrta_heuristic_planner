%% Function to know wich variable is stored in each position of the solution vector
function [var_name, var_index] = getVarName(index, dv_start_length)
    [values, sortIdx] = sort(cellfun(@(x) x(1), dv_start_length.values));
    unsortedkeys = dv_start_length.keys;
    keys = unsortedkeys(sortIdx);

    for i = 1:length(values)
        if index >= values(i);
            var_name = keys{i};
            var_index = index - (values(i) - 1);
        end
    end
end