function [resp, tOut] = plot_step_response(system, amplitudeStep, tEnd)
%plot_step_response Plot step response of aircraft model.
%   [resp, tOut] = plot_step_response(system_name, amplitudeStep, tEnd)
%   plots the step response of a dynamical system
%   represented in state space form.
%
%   - amplitudeStep in radian
%   - tEnd in second
%
%   Author: H. N. Tang
%
%   Note: The property fields
%       - InputName
%       - OutputName
%       - StateName
%       - InputUnit
%       - OutputUnit
%   must be specified.

    nOutputs = size(system, 1);
    nInputs = size(system, 2);

    [resp, tOut] = step(amplitudeStep * system, tEnd);

    % Conver rad, rad/s to deg, deg/s
    respConverted = resp;
    indexRad = strncmp(system.OutputUnit, "rad", 3);  % find index of
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
        title("Step Response (" + system.InputName{iInput} + ", +" + ...
            rad2deg(amplitudeStep) + " deg)", 'FontSize', 14);
    end

end