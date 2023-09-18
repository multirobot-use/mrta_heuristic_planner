%% Print solution
function printSolution(sol, fval, Agent, Task, A, N, T, S, Td_a_t_t, Te_t_nf, dv_start_length, execution_id, objective_function, predefined)
    if not(isempty(sol))
        tol = 1e-6;

        % Extract necessary decision variables values from solution vector
        % Extract x_a_t_s and reshape back solution vector to matrix x(start_x_a_t_s : start_x_a_t_s + length_x_a_t_s - 1) -> x(a,t,s)
        start_length = dv_start_length('x_a_t_s');
        start_x_a_t_s = start_length(1);
        length_x_a_t_s = start_length(2);
        x_a_t_s = reshape(sol(start_x_a_t_s : start_x_a_t_s + length_x_a_t_s - 1), A, T+1, S+1);

        % Extract tfin_a_s and reshape back solution vector to matrix tfin_a_s(start_tfin_a_s : start_tfin_a_s + length_tfin_a_s - 1) -> tfin_a_s(a,s)
        start_length = dv_start_length('tfin_a_s');
        start_tfin_a_s = start_length(1);
        length_tfin_a_s = start_length(2);
        tfin_a_s = reshape(sol(start_tfin_a_s : start_tfin_a_s + length_tfin_a_s - 1), A, S+1);

        % Extract nf_t_nf and reshape back solution vector to matrix nf_t_nf(start_nf_t_nf : start_nf_t_nf + length_nf_t_nf - 1) -> nf_t_nf(t,nf)
        start_length = dv_start_length('nf_t_nf');
        start_nf_t_nf = start_length(1);
        length_nf_t_nf = start_length(2);
        nf_t_nf = reshape(sol(start_nf_t_nf : start_nf_t_nf + length_nf_t_nf - 1), T, N);

        % Compute Td(a,s)
        % Td(a,s) = sum from t = 1 to T of ((sum from t2 = 0 to T of (Td_a(t,t2) * xa(t2,s-1))) * xa(t,s)), for all a = 1 to A and s = 1 to S
        Td = zeros(A,S);
        for a = 1:A
            for s = 1:S
                for t = 1:T
                    aux_Td_t2 = 0;
                    for t2 = 0:T
                        aux_Td_t2 = aux_Td_t2 + Td_a_t_t(a,t,(t2 + 1)) * x_a_t_s(a,(t2 + 1),((s-1) + 1));
                    end
                    Td(a,s) = Td(a,s) + aux_Td_t2 * x_a_t_s(a,(t + 1),(s + 1));
                end
            end
        end

        % Extract Tw_a_s and reshape back solution vector to matrix Tw_a_s(start_Tw_a_s : start_Tw_a_s + length_Tw_a_s - 1) -> Tw(a,s)
        start_length = dv_start_length('Tw_a_s');
        start_Tw_a_s = start_length(1);
        length_Tw_a_s = start_length(2);
        Tw = reshape(sol(start_Tw_a_s : start_Tw_a_s + length_Tw_a_s - 1), A, S);

        % Compute Te(a,s)
        % Te(a,s) = sum from t = 1 to T of ((sum from nf = 1 to N of (Te(t,nf) * nf(t,nf))) * xa(t,s)), for all a = 1 to A and s = 1 to S
        Te = zeros(A,S);
        for a = 1:A
            for s = 1:S
                for t = 1:T
                    aux_Te_nf = 0;
                    for nf = 1:N
                        aux_Te_nf = aux_Te_nf + Te_t_nf(t,nf) * nf_t_nf(t,nf);
                    end
                    Te(a,s) = Te(a,s) + aux_Te_nf * x_a_t_s(a,(t + 1),(s + 1));
                end
            end
        end

        % Open a new figure
        figure();
        hold on;
        barWidth = 0.8;

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
                gantt = barh(a, slot_duration, barWidth,'stacked', 'LineWidth', 1.5);
                times = barh(a - 0.2, slot_phase_duration, barWidth/2,'stacked', 'LineWidth', 1.5, 'FaceAlpha', 0);

                % Change tasks colors and add task name
                for s = 1:S
                    for t = 1:T
                        if x_a_t_s(a,t + 1,s + 1)
                            gantt(s).FaceColor = Task(t).color;
                            text(gantt(s).YEndPoints - gantt(s).YData/2, gantt(s).XEndPoints + gantt(s).BarWidth/4, Task(t).name, 'FontSize', 16, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
                            if slot_phase_duration(3*(s-1) + 1)
                                text(times((s-1)*3+1).YEndPoints - times((s-1)*3+1).YData/2, times((s-1)*3+1).XEndPoints, 'T^d', 'FontSize', 14, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
                            end
                            if slot_phase_duration(3*(s-1) + 2)
                                text(times((s-1)*3+2).YEndPoints - times((s-1)*3+2).YData/2, times((s-1)*3+2).XEndPoints, 'T^w', 'FontSize', 14, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
                            end
                            if slot_phase_duration(3*(s-1) + 3)
                                text(times((s-1)*3+3).YEndPoints - times((s-1)*3+3).YData/2, times((s-1)*3+3).XEndPoints, 'T^e', 'FontSize', 14, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
                            end
                        end
                    end
                end
            end
        end
        % Set axis labels
        xlabel('Time (s)', 'FontSize', 16);
        ylabel('Robot','FontSize', 16);
        yticks([1:A]);
        ylim([0.5 A+0.5]);

        % Set axis font size
        set(gca,'FontSize', 16, 'XGrid', 'on', 'YGrid', 'off','xminorgrid','on');

        % Set title
        title(strcat('Mission plan (', strrep(execution_id,'_','\_'), ') - Scenario: ', num2str(predefined), ' - fval: ', num2str(fval)));
        
        saveas(gcf, strcat("../fig/", execution_id), 'fig');
    end
end