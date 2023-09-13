function stop = saveBestSolutionSoFar(x, optimValues, state)
    % Function to use as OutputFcn to inlinprog(). It control the stop criterion.
    % Here we save a copy of the best solution found so far in case the computer crashes power off.
    % See https://es.mathworks.com/help/optim/ug/intlinprog-output-functions-and-plot-functions.html

    if not(isempty(x))
        save("../mat/bestSolutionSoFar.mat", 'x');
    end
    if state == "done"
        stop = true;
    else
        stop = false;
    end
end