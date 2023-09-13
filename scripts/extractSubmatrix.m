function [submatrix] = extractSubmatrix(matrix, fixed_coordinates, fixed_coordinates_values)
    if length(fixed_coordinates) ~= length(fixed_coordinates_values) || length(size(matrix)) ~= length(fixed_coordinates) + 2
        error('Incorrect number of fixed coordinates or fixed coordinates values');
    end
    
    sizes = size(matrix);
    submatrix = zeros(sizes(setdiff(1:length(sizes), fixed_coordinates)));

    size_submatrix = size(submatrix);

    % Construct prefix, middle and suffix strings so later we can do: prefix + num2str(i) + middle + num2str(j) + suffix
    prefix = 'matrix(';
    middle = '';
    suffix = '';
    row_flag = 0;
    col_flag = 0;
    counter = 1;
    for i = 1:length(size(matrix))
        if ismember(i, fixed_coordinates)
            if row_flag == 0
                prefix = [prefix, num2str(fixed_coordinates_values(counter))];
                counter = counter + 1;
            elseif col_flag == 0
                if isequal(middle, '')
                    middle = ',';
                end
                middle = [middle, num2str(fixed_coordinates_values(counter))];
                counter = counter + 1;
            else
                if isequal(suffix, '')
                    suffix = ',';
                end
                suffix = [suffix, num2str(fixed_coordinates_values(counter))];
                counter = counter + 1;
            end
            if i ~= length(size(matrix))
                if row_flag == 0
                    prefix = [prefix, ','];
                elseif col_flag == 0
                    middle = [middle, ','];
                else
                    suffix = [suffix, ','];
                end
            end	
        else
            if row_flag == 0
                row_flag = 1;
            elseif col_flag == 0
                col_flag = 1;
            end
        end
    end
    suffix = [suffix, ')'];
    if isequal(middle, '')
        middle = ',';
    end

    for i = 1:size_submatrix(1)
        for j = 1:size_submatrix(2)
            submatrix(i, j) = eval([prefix, num2str(i), middle, num2str(j), suffix]);
        end
    end
end