function plot_test(sys1, sys2, timeEnd)
% For report (lateral-directional)

    nOutputs = min(size(sys1, 1), size(sys2, 1));
    if nOutputs > 4
        nOutputs = 4;  % exclude washed out yaw rate from output plot
    end
    nInputs = size(sys1, 2);

    t = 0:0.01:timeEnd;
    amplitudeStep = deg2rad(5);

    %% Step
    y1 = step(amplitudeStep * sys1, t);
    y1 = rad2deg(y1);

    y2 = step(amplitudeStep * sys2, t);
    y2 = rad2deg(y2);

    unitsConverted = regexprep(sys1.OutputUnit, "rad", "deg");

    figsArray = get(groot, 'Children');
    if class(figsArray) == "matlab.graphics.GraphicsPlaceholder"
        currentMaxFigIdNumber = 0;
    else
        currentMaxFigIdNumber = max([figsArray.Number]);
    end
    for iInput = 1:nInputs
        figure(currentMaxFigIdNumber + iInput);
        for iOutput = 1:nOutputs
            subplot(nOutputs, 1, iOutput); hold on;
            plot(t, y1(:, iOutput, iInput), 'LineWidth', 1.0, ...
                'Color', 'b', 'LineStyle', '-');
            plot(t, y2(:, iOutput, iInput), 'LineWidth', 1.0, ...
                'Color', 'r', 'LineStyle', '-');
            grid on
            ylabel(sys1.OutputName{iOutput} + ", " + ...
                unitsConverted{iOutput}, 'FontSize', 12);
        end
        xlabel('Time, s', 'FontSize', 12);
        %legend('system 1', 'system 2', 'Location', 'best');
        legend('without turn coordinator', 'with turn coordinator', 'Location', 'southwest');
        subplot(nOutputs, 1, 1);
        % title("Step Response (" + sys1.InputName{iInput} + ", +" + ...
        %     rad2deg(amplitudeStep) + " deg)", 'FontSize', 14);
    end

end