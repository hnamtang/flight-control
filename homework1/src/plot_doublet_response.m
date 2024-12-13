function plot_doublet_response(system1, system2, amplitudeDoublet, tEnd, boolSavePlot)
%plot_doublet_response Plot response for doublet input of aircraft model.
%   plot_doublet_response(system1, system2, amplitudeDoublet, tEnd, boolSavePlot)
%   plots the responses for double input of two dynamical systems
%   represented in state space form.
%
%   - amplitudeDoublet in radian
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

    if nargin == 4
        boolSavePlot = false;
    end

    nOutputs = size(system1, 1);
    if nOutputs > 4
        nOutputs = 4;  % exclude washed out yaw rate from output plot
    end
    nInputs = size(system1, 2);

    tOut = 0:0.01:tEnd;
    input = amplitudeDoublet*((tOut > tEnd/10) - 2*(tOut > tEnd/10 + 1) + (tOut > tEnd/10 + 2));
    u = zeros(nInputs, nInputs, length(tOut));
    if nInputs == 1
        u(1, 1, :) = input;
    else
        u(1, :, :) = [input; zeros(size(tOut))];
        u(2, :, :) = [zeros(size(tOut)); input];
    end

    resp1 = zeros(length(tOut), nOutputs, nInputs);
    resp2 = zeros(length(tOut), nOutputs, nInputs);
    for iInput = 1:nInputs
        resp1(:, :, iInput) = lsim(system1(1:nOutputs, :), reshape(u(iInput, :, :), nInputs, length(tOut)), tOut);
        resp2(:, :, iInput) = lsim(system2(1:nOutputs, :), reshape(u(iInput, :, :), nInputs, length(tOut)), tOut);
    end

    % Conver rad, rad/s to deg, deg/s
    respConverted1 = resp1;
    indexRad1 = strncmp(system1.OutputUnit(1:nOutputs), "rad", 3);  % find index of
                                                           % outputs with unit rad
    respConverted1(:, indexRad1, :) = rad2deg(respConverted1(:, indexRad1, :));
    unitsConverted1 = regexprep(system1.OutputUnit, "rad", "deg");

    respConverted2 = resp2;
    indexRad2 = strncmp(system2.OutputUnit(1:nOutputs), "rad", 3);  % find index of
                                                           % outputs with unit rad
    respConverted2(:, indexRad2, :) = rad2deg(respConverted2(:, indexRad2, :));

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
            plot(tOut, respConverted1(:, iOutput, iInput), 'LineWidth', 1.0, ...
                'Color', 'b', 'LineStyle', '-');
            hold on;
            plot(tOut, respConverted2(:, iOutput, iInput), 'LineWidth', 1.0, ...
                'Color', 'r', 'LineStyle', '-');
            plot(tOut, rad2deg(input), 'LineWidth', 0.8, ...
                'Color', 'k', 'LineStyle', '--');
            grid on
            ylabel(system1.OutputName{iOutput} + ", " + ...
                unitsConverted1{iOutput}, 'FontSize', 12);
        end
        xlabel('Time, s', 'FontSize', 12);
        subplot(nOutputs, 1, nOutputs);
        %legend('without turn coordinator', 'with turn coordinator', 'input doublet', 'Location', 'southeast');
        legend('SAS off', 'SAS on', 'input doublet', 'Location', 'southeast');

        % Save plot as image
        if boolSavePlot
            systemName1 = inputname(1);
            systemName2 = inputname(2);
            figTitle = "Doublet Response (" + system1.InputName{iInput} + ...
                ", +-" + rad2deg(amplitudeDoublet) + " deg)";
            exportgraphics(gcf, ...
                systemName1 + "_" + systemName2 + join(strsplit(figTitle), '_') + ".png", ...
                'Resolution', 600);
        end
    end

end