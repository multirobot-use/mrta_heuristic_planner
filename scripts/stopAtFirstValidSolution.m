function stop = stopAtFirstValidSolution(x, optimValues, state)
    % Function to use as OutputFcn to inlinprog(). It control the stop criterion.
    % It stop solving when the first valid solution is found.
    % At this way we can test if the MILP formulation is ok with not so small scenarios without waiting a lot of time.
    % See https://es.mathworks.com/help/optim/ug/intlinprog-output-functions-and-plot-functions.html

    if not(isempty(x))
        stop = true;
    else
        stop = false;
    end
end