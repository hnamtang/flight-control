function step_response(system)
%step_response Plot step response of aircraft model.
%   step_response(system_name) plot the step response of a dynamical system
%   represented in state space form.
%
%   Note: The property fields
%       - InputName
%       - OutputName
%       - StateName
%       - InputUnit
%       - OutputUnit
%   must be specified.

    [y, tout] = step(system);

    figure(1);
    for i = 1:4
        subplot(4, 1, i); plot(tout, y(:, i, 1), 'LineWidth', 1.5);
        grid on
        ylabel("$" + system.OutputName{i} + "$" + "[" + system.OutputUnit{i} + "]", ...
            'Interpreter', 'latex', 'FontSize', 14);
    end
    xlabel('t [s]', 'Interpreter', 'latex', 'FontSize', 14);
    subplot(4, 1, 1); title('Step Response ($\delta_t$)', 'Interpreter', 'latex', ...
        'FontSize', 16);

    figure(2);
    for i = 1:4
        subplot(4, 1, i); plot(tout, y(:, i, 2), 'LineWidth', 1.5);
        grid on
        ylabel("$" + system.OutputName{i} + "$" + "[" + system.OutputUnit{i} + "]", ...
            'Interpreter', 'latex', 'FontSize', 14);
    end
    xlabel('t [s]', 'Interpreter', 'latex', 'FontSize', 14);
    subplot(4, 1, 1); title('Step Response ($\delta_e$)', 'Interpreter', 'latex', ...
        'FontSize', 16);

end