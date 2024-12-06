function [resp, tOut] = plot_step_response(system, amplitudeStep, tEnd, boolSavePlot)
%plot_step_response Plot step response of aircraft model.
%   [resp, tOut] = plot_step_response(system, amplitudeStep, tEnd, boolSavePlot)
%   plots the step response of a dynamical system
%   represented in state space form.
%
%   - amplitudeStep in radian
%   - tEnd in second
%
%   Note: The property fields
%       - InputName
%       - OutputName
%       - StateName
%       - InputUnit
%       - OutputUnit
%   must be specified.
%
%   Author: H. N. Tang

    if nargin == 3
        boolSavePlot = false;
    end

    nOutputs = size(system, 1);
    if nOutputs > 4
        nOutputs = 4;  % exclude washed out yaw rate from output plot
    end
    nInputs = size(system, 2);

    [resp, tOut] = step(amplitudeStep * system(1:nOutputs, :), tEnd);

    % Conver rad, rad/s to deg, deg/s
    respConverted = resp;
    indexRad = strncmp(system.OutputUnit(1:nOutputs), "rad", 3);  % find index of
                                                                  % outputs with unit rad
    respConverted(:, indexRad, :) = rad2deg(respConverted(:, indexRad, :));
    unitsConverted = regexprep(system.OutputUnit, "rad", "deg");

    figsArray = get(groot, 'Children');
    if class(figsArray) == "matlab.graphics.GraphicsPlaceholder"
        currentMaxFigIdNumber = 0;
    else
        currentMaxFigIdNumber = max([figsArray.Number]);
    end
    for iInput = 1:nInputs
        figure(currentMaxFigIdNumber + iInput);
        for iOutput = 1:nOutputs
            subplot(nOutputs, 1, iOutput);
            plot(tOut, respConverted(:, iOutput, iInput), 'LineWidth', 1.5);
            grid on
            ylabel(system.OutputName{iOutput} + ", " + ...
                unitsConverted{iOutput}, 'FontSize', 12);
        end
        xlabel('t, s', 'FontSize', 12);
        subplot(nOutputs, 1, 1);
        figTitle = "Step Response (" + system.InputName{iInput} + ...
            ", +-" + rad2deg(amplitudeStep) + " deg)";
        title(figTitle, 'FontSize', 14);

        % Save plot
        if boolSavePlot
            systemName = inputname(1);
            exportgraphics(gcf, ...
                systemName + "_" + join(strsplit(figTitle), '_') + ".png", ...
                'Resolution', 600);
        end
    end

end