function plot_step_response(sys, inputMag, tEnd, boolSavePlot)
% Inputs:
%   - sys: state space form
%   - inputMag: input magnitude in degree
%   - tEnd (optional): end time for simulation in second
%
% Author: H. N. Tang

if nargin < 4
    boolSavePlot = false;
end

[nOutput, nInput] = size(sys);

% Determine whether plot of actuators is necessary
if any(strncmp('\delta \xi', sys.OutputName, 10))
    bool_actua = 1;
else
    bool_actua = 0;
end


if nOutput > 5
    warning('Number of output exceeds 5.');
    nOutput = 5;
elseif nInput > 5
    warning('Number of input exceeds 2.');
    nInput = 2;
end

stepinput = deg2rad(inputMag);
if nargin == 2
    [y, t] = step(stepinput * sys(1:nOutput,1:nInput));

    if bool_actua
        y_actua = step(stepinput * sys({'\delta \xi', '\delta \zeta'},1:nInput), t);
    end
elseif nargin >= 3
    t = 0:0.01:tEnd;
    y = step(stepinput * sys(1:nOutput,1:nInput), t);
    if bool_actua
        y_actua = step(stepinput * sys({'\delta \xi', '\delta \zeta'},1:nInput), t);
    end
end

outputunits = {'°/s', '°', '°/s', '°', ''};

% Plot responses
fig_y = figure();
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
        elseif iOutput == nOutput
            xlabel('Time, s');
        end
        
        if iInput == 1
            if iOutput ~= nOutput
                ylabel([sys.OutputName{iOutput}, ', ', outputunits{iOutput}]);
            else
                ylabel(sys.OutputName{iOutput});
            end
        end
        iFig = iFig + 1;
    end
end


% Plot actuator responses
if bool_actua
    actua_name = {'\delta \xi', '\delta \zeta'};
    nActua = 2;
    fig_actua = figure();
    iFig = 1;
    for iActua = 1:nActua
        for iInput = 1:nInput
            subplot(nActua, nInput, iFig);
            plot(t, squeeze(y_actua(:,iActua,iInput)) * 180/pi, 'LineWidth', 1.5);
            grid on

            if iActua == 1
                title([sys.InputName{iInput}, ', °'])
            elseif iActua == nActua
                xlabel('Time, s');
            end

            if iInput == 1
                ylabel([actua_name{iActua}, ', °']);
            end
            iFig = iFig + 1;
        end
    end
end

% Save plots as image
if boolSavePlot
    exportgraphics(fig_y, "y_step_response.png", 'Resolution', 600);
    exportgraphics(fig_actua, "actua_step_response.png", 'Resolution', 600);
end


end