function [resp, tOut] = plot_step_response(system, tEnd)
%plot_step_response Plot step response of aircraft model.
%   [resp, tOut] = plot_step_response(system_name) plots the step response
%   of a dynamical system represented in state space form.
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

    [resp, tOut] = step(system, tEnd);

    % Conver rad, rad/s to deg, deg/s
    indexOutput = strncmp(system.OutputUnit, "rad", 3);  % find index of
                                                         % outputs with unit rad
    resp(:, indexOutput, :) = rad2deg(resp(:, indexOutput, :));
    unitsConverted = regexprep(system.OutputUnit, "rad", "deg");


    % currentFigNumber = length(findobj('Type', 'Figure'));
    % for iInput = 1:nInputs
    %     figure(currentFigNumber + iInput);
    %     for iOutput = 1:nOutputs
    %         subplot(nOutputs, 1, iOutput);
    %         plot(tOut, resp(:, iOutput, iInput), 'LineWidth', 1.5);
    %         grid on
    %         ylabel("$" + system.OutputName{iOutput} + "$" + " [" + ...
    %             system.OutputUnit{iOutput} + "]", ...
    %             'Interpreter', 'latex', 'FontSize', 14);
    %     end
    %     xlabel('t [s]', 'Interpreter', 'latex', 'FontSize', 14);
    %     subplot(nOutputs, 1, 1);
    %     title("Step Response ($" + system.InputName{iInput} + "$)", ...
    %         'Interpreter', 'latex', 'FontSize', 16);
    % end

    currentFigNumber = length(findobj('Type', 'Figure'));
    for iInput = 1:nInputs
        figure(currentFigNumber + iInput);
        for iOutput = 1:nOutputs
            subplot(nOutputs, 1, iOutput);
            plot(tOut, resp(:, iOutput, iInput), 'LineWidth', 1.5);
            grid on
            ylabel(system.OutputName{iOutput} + " [" + ...
                unitsConverted{iOutput} + "]", 'FontSize', 14);
        end
        xlabel('t [s]', 'FontSize', 14);
        subplot(nOutputs, 1, 1);
        title("Step Response (" + system.InputName{iInput} + ")", 'FontSize', 16);
    end

end