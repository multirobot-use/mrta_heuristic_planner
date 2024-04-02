%% Print solution
function previewScenario(Agent, Task, scenario_id)
    % Get scenario information
    [Agent, Task, A, T, S, N, R, Td_a_t_t, Te_t_nf, H_a_t] = getConstantScenarioValues(Agent, Task);

    barWidth = 0.8;

    % Get longest axis: max(max(Ft), max(Te))
    axis_limit = 3000; % seconds % max(max([Task.Te]),max(([Agent.Ft])));
    word_length = 0.4 * axis_limit;

    % Open a new figure
    figure();
    subplot(2,1,1);
    hold on;

    % Plot Agents' information: type, Ft, Ft_0
    for a = 1:A
        % Color
        color = [randi([0 255]) randi([0 255]) randi([0 255])]/255;
        % Make sure that the color isn't too dark to read black text over it
        while mean(color) < 0.6
            color = [randi([0 255]) randi([0 255]) randi([0 255])]/255;
        end

        % Plot agent's flight time
        flight_time = barh(a, [Agent(a).Ft], barWidth, 'stacked', 'LineWidth', 1.5, 'FaceColor', color, 'FaceAlpha', 0.75);

        % Plot agent's initially consumed flight time
        if Agent(a).Ft_0 > 1e-6
            initial_flight_time = barh(a - barWidth/4, Agent(a).Ft_0, barWidth/2, 'stacked', 'LineWidth', 1.5, 'FaceColor', [0.2 0.2 0.2], 'FaceAlpha', 0.5, 'EdgeColor', color, 'EdgeAlpha', 0);
        end

        % Add agent type
        text(axis_limit - word_length/2, flight_time(1).XEndPoints + flight_time(1).BarWidth/4, ['Type: ', num2str(Agent(a).type)], 'FontSize', 12, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
    end

    subplot(2,1,2);
    hold on;

    % Plot Tasks' information: Hr, Te, N, N_hardness, Relayability, Fragmentability
    for t = 1:T
        execution_time = barh(t - 1, [Task(t).Te], barWidth, 'stacked', 'LineWidth', 1.5, 'FaceAlpha', 0.75);

        % Change tasks colors and add task porperties
        if t == R
            execution_time(1).FaceColor = Task(t).color;
        else
            if Task(t). Fragmentability == 0 && Task(t).Relayability == 0
                color = [0.93 0.00 0.13];
            elseif Task(t). Fragmentability == 0 && Task(t).Relayability == 1
                color = [0.93 0.69 0.13];
            elseif Task(t). Fragmentability == 1 && Task(t).Relayability == 0
                color = [0.47 0.67 0.19];
            end
            execution_time(1).FaceColor = color;

            if Task(t).N_hardness == 0
                soft_hard = 'soft';
            else
                soft_hard = 'hard';
            end
            % text(axis_limit - word_length/2, execution_time(1).XEndPoints + execution_time(1).BarWidth/4, ['Hr: ', num2str(Task(t).Hr)], 'FontSize', 12, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
            % text(axis_limit - word_length/2, execution_time(1).XEndPoints - execution_time(1).BarWidth/4, ['N ', soft_hard, ': ', num2str(Task(t).N)], 'FontSize', 12, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
            text(axis_limit - word_length/2, execution_time(1).XEndPoints, ['Hr: ', num2str(Task(t).Hr), ', N ', soft_hard, ': ', num2str(Task(t).N)], 'FontSize', 12, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
        end
    end

    % Set axis labels
    subplot(2,1,1);
    ylabel('Robot','FontSize', 12);

    subplot(2,1,2);
    xlabel('Time (s)', 'FontSize', 12);
    ylabel('Task','FontSize', 12);
    
    % Set axis limits and font size    
    subplot(2,1,1);
    xticks('auto');
    xlim([0, axis_limit]);
    
    yticks([1:A]);
    ylim([0.5 A+0.5]);

    set(gca,'FontSize', 12, 'FontWeight', 'bold', 'XGrid', 'off', 'YGrid', 'off', 'xminorgrid', 'off', 'yminorgrid', 'off', 'TickDir', 'out');

    subplot(2,1,2);
    xticks('auto');
    xlim([0, axis_limit]);
    
    yticks([0:T-1]);
    ylim([-0.5 T-1+0.5]);

    set(gca,'FontSize', 12, 'FontWeight', 'bold', 'XGrid', 'off', 'YGrid', 'off', 'xminorgrid', 'off', 'yminorgrid', 'off', 'TickDir', 'out');


    % Set title
    subplot(2,1,1);
    if nargin > 2 && not(isempty(scenario_id))
        title(['Preview of scenario "', strrep(scenario_id,'_','\_'), '". S= ', num2str(S), ', N= ', num2str(N),'.']);
        saveas(gcf, ['../fig/preview_scenario_', scenario_id], 'fig');
    else
        % title(['Scenario preview. S= ', num2str(S), ', N= ', num2str(N),'.']);
    end
end
