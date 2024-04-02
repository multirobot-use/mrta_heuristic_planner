%% Print solution
function printSolution(sol, Agent, Task, join_flag, scenario_id, execution_id, fval)
    % Minimum required inputs: sol, Agent, Task
    if nargin < 3
        error('Not enough input arguments.');
    end

    % Set the numerical precision
    tol = 1e-6;

    % Get scenario information
    [Agent, Task, A, T, S, N, R, Td_a_t_t, Te_t_nf, H_a_t] = getConstantScenarioValues(Agent, Task);

    % Get start length structure information
    [dv_start_length, length_dv] = getStartLengthInformation(Agent, Task);
    
    % Check the printing mode: from a solution vector or from an initializer solution structure
    if length(sol) == length_dv
        % Substitute virtual zeros by exact zeros in the sol vector
        sol(abs(sol) < tol) = 0;

        % Extract necessary decision variables values from solution vector
        % Extract makespan from sol
        makespan = sol(1);
        
        % Extract x_a_t_s and reshape back solution vector to matrix x(start_x_a_t_s : start_x_a_t_s + length_x_a_t_s - 1) -> x(a,t,s)
        start_length = dv_start_length('x_a_t_s');
        start_x_a_t_s = start_length(1);
        length_x_a_t_s = start_length(2);
        x_a_t_s = reshape(full(sol(start_x_a_t_s : start_x_a_t_s + length_x_a_t_s - 1)), A, T+1, S+1);

        % Extract tfin_a_s and reshape back solution vector to matrix tfin_a_s(start_tfin_a_s : start_tfin_a_s + length_tfin_a_s - 1) -> tfin_a_s(a,s)
        start_length = dv_start_length('tfin_a_s');
        start_tfin_a_s = start_length(1);
        length_tfin_a_s = start_length(2);
        tfin_a_s = reshape(sol(start_tfin_a_s : start_tfin_a_s + length_tfin_a_s - 1), A, S+1);

        % Extract Td_a_s and reshape back solution vector to matrix Td_a_s(start_Td_a_s : start_Td_a_s + length_Td_a_s - 1) -> Td(a,s)
        start_length = dv_start_length('Td_a_s');
        start_Td_a_s = start_length(1);
        length_Td_a_s = start_length(2);
        Td = reshape(sol(start_Td_a_s : start_Td_a_s + length_Td_a_s - 1), A, S);

        % Extract Tw_a_s and reshape back solution vector to matrix Tw_a_s(start_Tw_a_s : start_Tw_a_s + length_Tw_a_s - 1) -> Tw(a,s)
        start_length = dv_start_length('Tw_a_s');
        start_Tw_a_s = start_length(1);
        length_Tw_a_s = start_length(2);
        Tw = reshape(sol(start_Tw_a_s : start_Tw_a_s + length_Tw_a_s - 1), A, S);

        % Extract Te_a_s and reshape back solution vector to matrix Te_a_s(start_Te_a_s : start_Te_a_s + length_Te_a_s - 1) -> Te(a,s)
        start_length = dv_start_length('Te_a_s');
        start_Te_a_s = start_length(1);
        length_Te_a_s = start_length(2);
        Te = reshape(sol(start_Te_a_s : start_Te_a_s + length_Te_a_s - 1), A, S);
    elseif isfield(Agent, 'queue')
        % Get the number of used slots per robot
        used_slots = zeros(1,A);
        for a = 1:A
            used_slots(a) = length(Agent(a).queue) - 1;
        end

        % Get the exact number of used slots
        S = max(used_slots);

        % Extract necessary decision variables values from Agent structure
        % Extract x_a_t_s from Agent.queue
        x_a_t_s = zeros(A, T+1, S+1);
        % Initial tasks
        t = 0;
        s = 0;
        x_a_t_s(:, t + 1, s + 1) = 1;

        % Move robot's queue to x_a_t_s decision variable
        for a = 1:A
            s = 0;
            for task = 1:length(Agent(a).queue) - 1
                t = Agent(a).queue(task + 1);
                s = s + 1;
                x_a_t_s(a, t + 1, s + 1) = 1;
            end
        end

        % Substitute virtual zeros by exact zeros in x_a_t_s
        x_a_t_s(abs(x_a_t_s) < tol) = 0;

        % Extract tfin_a_s from Agent.tfin
        tfin_a_s = zeros(A, S+1);
        for a = 1:A
            for s = 1:S
                tfin_a_s(a, s + 1) = Agent(a).tfin_s(s + 1);
            end
        end

        % Substitute virtual zeros by exact zeros in tfin_a_s
        tfin_a_s(abs(tfin_a_s) < tol) = 0;

        % Extract makespan from tfin_a_s
        makespan = max(max(tfin_a_s));

        % Extract Td_a_s from Agent.Td_s
        Td = zeros(A,S);
        for a = 1:A
            for s = 1:S
                Td(a,s) = Agent(a).Td_s(s);
            end
        end

        % Substitute virtual zeros by exact zeros in Td
        Td(abs(Td) < tol) = 0;

        % Extract Tw_a_s from Agent.Tw_s
        Tw = zeros(A,S);
        for a = 1:A
            for s = 1:S
                Tw(a,s) = Agent(a).Tw_s(s);
            end
        end

        % Substitute virtual zeros by exact zeros in Tw
        Tw(abs(Tw) < tol) = 0;

        % Extract Te_a_s from Agent.Te_s
        Te = zeros(A,S);
        for a = 1:A
            for s = 1:S
                Te(a,s) = Agent(a).Te_s(s);
            end
        end

        % Substitute virtual zeros by exact zeros in Te
        Te(abs(Te) < tol) = 0;
    else
        return;
    end

    % Join consecutive tasks of the same type
    if nargin < 4 || isempty(join_flag) || join_flag
        for a = 1:A
            for s = 1:S-1
                for t = 1:T
                    repeat_flag = 1;
                    while repeat_flag
                        repeat_flag = 0;
                        if x_a_t_s(a,t + 1,s + 1) && x_a_t_s(a,t + 1,(s+1) + 1)
                            % Join displacement, waiting and execution times
                            Td(a,s) = Td(a,s) + Td(a,s + 1);
                            Tw(a,s) = Tw(a,s) + Tw(a,s + 1);
                            Te(a,s) = Te(a,s) + Te(a,s + 1);
                            % Update displacement, waiting and execution times for the slots that have been moved
                            Td(a,s + 1:S) = [Td(a,(s+1) + 1:S), 0];
                            Tw(a,s + 1:S) = [Tw(a,(s+1) + 1:S), 0];
                            Te(a,s + 1:S) = [Te(a,(s+1) + 1:S), 0];
                            % Move the empty slot to the end
                            for slot = s+1:S-1
                                for task = 1:T
                                    x_a_t_s(a,task + 1,slot + 1) = x_a_t_s(a,task + 1,(slot+1) + 1);
                                end
                            end
                            % Empty the last slot
                            x_a_t_s(a,t + 1,S + 1) = 0;
                            % Update tfin for the slots that have been moved
                            tfin_a_s(a,s + 1:S + 1) = [tfin_a_s(a,(s+1) + 1:S + 1), tfin_a_s(a,S + 1)];
                            % Set the flag to repeat the process
                            repeat_flag = 1;
                        end
                    end
                end
            end
        end
    end

    % Open a new figure
    figure();
    hold on;
    barWidth = 0.8;
    times_bar_proportion = 1/3;

    % Plot Agents' queue
    for a = 1:A
        % Make a copy of tfin_a_s
        slot_duration = tfin_a_s(a,:);

        % Compute slot duration
        slot_duration = diff(slot_duration);

        % Substitute virtual zeros by exact zeros
        slot_duration(abs(slot_duration) < tol) = 0;

        % Remove the zeros from the end
        slot_duration = [slot_duration(slot_duration ~= 0)];

        % Compute task phase slot duration
        slot_phase_duration = zeros(1,3*S);
        for s = 1:S
                slot_phase_duration(3*(s-1) + 1) = Td(a,s);
                slot_phase_duration(3*(s-1) + 2) = Tw(a,s);
                slot_phase_duration(3*(s-1) + 3) = Te(a,s);
        end

        % Plot a's queue
        if not(isempty(slot_duration))
            gantt = barh(a + barWidth * (1/2 - (1 - times_bar_proportion)/2), slot_duration, barWidth * (1 - times_bar_proportion), 'stacked', 'LineWidth', 1.5, 'FaceAlpha', 0.75);
            times = barh(a + barWidth * (times_bar_proportion/2 - 1/2), slot_phase_duration, barWidth * times_bar_proportion, 'stacked', 'LineWidth', 1.5, 'FaceAlpha', 0.75, 'EdgeAlpha', 0);
            box   = barh(a, slot_duration, barWidth, 'stacked', 'LineWidth', 1.5, 'FaceAlpha', 0);

            % Change tasks colors and add task name
            for s = 1:S
                for t = 1:T
                    if x_a_t_s(a,t + 1,s + 1)
                        gantt(s).FaceColor = Task(t).color;
                        text(gantt(s).YEndPoints - gantt(s).YData/2, gantt(s).XEndPoints, Task(t).name, 'FontSize', 16, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
                        if slot_phase_duration(3*(s-1) + 1) > tol
                            times((s-1)*3+1).FaceColor = [0 0 1];
                            % text(times((s-1)*3+1).YEndPoints - times((s-1)*3+1).YData/2, times((s-1)*3+1).XEndPoints, 'T^d', 'FontSize', 14, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
                        end
                        if slot_phase_duration(3*(s-1) + 2) > tol
                            times((s-1)*3+2).FaceColor = [1 1 0];
                            % text(times((s-1)*3+2).YEndPoints - times((s-1)*3+2).YData/2, times((s-1)*3+2).XEndPoints, 'T^w', 'FontSize', 14, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
                        end
                        if slot_phase_duration(3*(s-1) + 3) > tol
                            times((s-1)*3+3).FaceColor = [0 1 0];
                            % text(times((s-1)*3+3).YEndPoints - times((s-1)*3+3).YData/2, times((s-1)*3+3).XEndPoints, 'T^e', 'FontSize', 14, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
                        end
                    end
                end
            end
        end
    end
    % Set axis labels
    xlabel('Time (s)', 'FontSize', 16);
    ylabel('Robot','FontSize', 16);
    xticks('auto');
    xlim([0 makespan]);
    yticks([1:A]);
    ylim([0.5 A+0.5]);

    % Set axis font size
    set(gca,'FontSize', 16, 'FontWeight', 'bold', 'XGrid', 'off', 'YGrid', 'off','xminorgrid','off', 'TickDir', 'out');

    % Set title
    switch nargin
    case 3
        title('Mission plan.');
    case 4
        title('Mission plan.');
    case 5
        if not(isempty(scenario_id))
            title(['Mission plan for scenario: ', strrep(num2str(scenario_id),'_','\_'), '.']);
        else
            title('Mission plan.');
        end
    case 6
        if isempty(execution_id) && isempty(scenario_id)
            title('Mission plan.');
        elseif isempty(execution_id) && not(isempty(scenario_id))
            title(['Mission plan for scenario: ', strrep(num2str(scenario_id),'_','\_'), '.']);
        elseif not(isempty(execution_id)) && isempty(scenario_id)
            title(['Mission plan (', strrep(execution_id,'_','\_'), ').']);
        else
            title(['Mission plan for scenario: ', strrep(num2str(scenario_id),'_','\_'), ' (', strrep(execution_id,'_','\_'), ').']);
        end
    case 7
        if isempty(execution_id) && isempty(scenario_id)
            title(['Mission plan. fval: ', num2str(fval), '.']);
        elseif isempty(execution_id) && not(isempty(scenario_id))
            title(['Mission plan for scenario: ', strrep(num2str(scenario_id),'_','\_'), '. fval: ', num2str(fval), '.']);
        elseif not(isempty(execution_id)) && isempty(scenario_id)
            title(['Mission plan (', strrep(execution_id,'_','\_'), '). fval: ', num2str(fval), '.']);
        else
            title(['Mission plan for scenario: ', strrep(num2str(scenario_id),'_','\_'), ' (', strrep(execution_id,'_','\_'), '). fval: ', num2str(fval)]);
        end
    end
    
    if nargin > 5 && not(isempty(execution_id))
        saveas(gcf, ['../fig/', execution_id], 'fig');
    end
end