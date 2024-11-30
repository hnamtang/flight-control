function [resp, tOut] = plot_doublet_response(system, amplitudeDoublet, tEnd)
%plot_doublet_response Plot response for doublet input of aircraft model.
%   [resp, tOut] = plot_doublet_response(system_name, amplitudeDoublet, tEnd)
%   plots the response for double input of a dynamical system
%   represented in state space form.
%
%   - amplitudeDoublet in radian
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

    tOut = 0:0.01:tEnd;
    input = amplitudeDoublet * ((tOut > tEnd/10) - 2 * (tOut > tEnd/10 + 1) + (tOut > tEnd/10 + 2));
    u = zeros(nInputs, nInputs, length(tOut));
    if nInputs == 1
        u(1, 1, :) = input;
    else
        u(1, :, :) = [input; zeros(size(tOut))];
        u(2, :, :) = [zeros(size(tOut)); input];
    end

    resp = zeros(length(tOut), nOutputs, nInputs);
    for iInput = 1:nInputs
        resp(:, :, iInput) = lsim(system, reshape(u(iInput, :, :), nInputs, length(tOut)), tOut);
    end

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
            hold on;
            plot(tOut, rad2deg(input), 'LineWidth', 1, ...
                'LineStyle', '--', 'Color', 'k');
            grid on
            ylabel(system.OutputName{iOutput} + ", " + ...
                unitsConverted{iOutput}, 'FontSize', 12);
        end
        xlabel('t, s', 'FontSize', 12);
        subplot(nOutputs, 1, 1);
        title("Doublet Response (" + system.InputName{iInput} + ...
            ", +-" + rad2deg(amplitudeDoublet) + " deg)", 'FontSize', 14);
    end

end