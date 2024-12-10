function plot_doublet_controller(system1, amplitudeDoublet, tEnd)
%   Author: H. N. Tang

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
    for iInput = 1:nInputs
        resp1(:, :, iInput) = lsim(system1(1:nOutputs, :), reshape(u(iInput, :, :), nInputs, length(tOut)), tOut);
    end

    % Conver rad, rad/s to deg, deg/s
    respConverted1 = resp1;
    indexRad1 = strncmp(system1.OutputUnit(1:nOutputs), "rad", 3);  % find index of
                                                           % outputs with unit rad
    respConverted1(:, indexRad1, :) = rad2deg(respConverted1(:, indexRad1, :));
    unitsConverted1 = regexprep(system1.OutputUnit, "rad", "deg");

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
            uTemp = rad2deg(u(iInput, :, :));
            uPlot = reshape(uTemp(:, iOutput, :), 1, length(tOut));
            plot(tOut, uPlot, 'LineWidth', 0.8, ...
                'Color', 'k', 'LineStyle', '--');
            grid on
            ylabel(system1.OutputName{iOutput} + " defl, " + ...
                unitsConverted1{iOutput}, 'FontSize', 12);
        end
        xlabel('Time, s', 'FontSize', 12);
        subplot(nOutputs, 1, nOutputs);
        %legend('without turn coordinator', 'with turn coordinator', 'input doublet', 'Location', 'southeast');
        legend('controller output', 'input doublet', 'Location', 'southeast');
    end

end