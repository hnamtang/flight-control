function plot_step_response(sys, t)
% Outputs are: r, beta, p, Phi, by

[nOutput, nInput] = size(sys);

if nOutput > 5
    warning('Number of output exceeds 5.');
    nOutput = 5;
elseif nInput > 5
    warning('Number of input exceeds 5.');
    nInput = 5;
end

stepinput = deg2rad(1);  % 1° deflection
if nargin == 1
    [y, t] = step(stepinput * sys);
else
    y = step(stepinput * sys, t);
end

outputunits = {'°', '°', '°', '°', ''};
figure();
iFig = 1;
for iOutput = 1:nOutput
    if any(iOutput == [1, 2, 3, 4])
        conv_factor = 180/pi;
    else
        conv_factor = 1;
    end

    for iInput = 1:nInput
        subplot(nOutput, nInput, iFig);
        plot(t, squeeze(y(:, iOutput, iInput))*conv_factor, 'LineWidth', 1.5);
        grid on
        if iOutput == 1
            title([sys.InputName{iInput}, ', °'])
        end
        iFig = iFig + 1;
        if iInput == 1
            if iOutput ~= nOutput
                ylabel([sys.OutputName{iOutput}, ', ', outputunits{iOutput}])
            else
                ylabel(sys.OutputName{iOutput})
            end
        end
    end
end
end