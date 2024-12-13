function plot_sim_data(dataOL, dataCL, dt, motionType)
%   Plot comparison between OL and CL in flight simulator
%   Inputs: open loop, closed-loop, time adjust, motion type.
%
%   Author: H. N. Tang

    samplingFrequency = 50;  % Hz
    font_size = 10;

    % Plot
    if contains(motionType, "lon", "IgnoreCase", true)
        figure();  % state variables
        ax1 = subplot(6, 1, 1); hold on;
        % ALTITUDE
        plot(dataOL.duration, dataOL.altitude, 'Color', 'b', ...
            'LineWidth', 1.0, 'LineStyle', '-');
        plot(dataCL.duration + dt.altitude/samplingFrequency, ...
           dataCL.altitude, 'Color', 'r', 'LineWidth', 1.0, 'LineStyle', '-');
        grid on
        ylabel('ALT, ft', 'FontSize', font_size);
        
        % RATE OF CLIMB
        ax2 = subplot(6, 1, 2); hold on;
        plot(dataOL.duration, dataOL.rateOfClimb, 'Color', 'b', ...
            'LineWidth', 1.0, 'LineStyle', '-');
        plot(dataCL.duration + dt.rateOfClimb/samplingFrequency, ...
            dataCL.rateOfClimb, 'Color', 'r', 'LineWidth', 1.0, 'LineStyle', '-');
        grid on
        ylabel('ROC, ft/min', 'FontSize', font_size);
        
        % V_TAS
        ax3 = subplot(6, 1, 3); hold on;
        plot(dataOL.duration, dataOL.trueAirspeed, 'Color', 'b', ...
            'LineWidth', 1.0, 'LineStyle', '-');
        plot(dataCL.duration + dt.trueAirspeed/samplingFrequency, ...
            convvel(dataCL.trueAirspeed, 'm/s', 'kts'), 'Color', 'r', ...
            'LineWidth', 1.0, 'LineStyle', '-');
        grid on
        ylabel('V_{TAS}, kts', 'FontSize', font_size);
        
        % AOA
        ax4 = subplot(6, 1, 4); hold on;
        plot(dataOL.duration, dataOL.AOA, 'Color', 'b', ...
            'LineWidth', 1.0, 'LineStyle', '-');
        plot(dataCL.duration + dt.AOA/samplingFrequency, ...
            dataCL.AOA, 'Color', 'r', 'LineWidth', 1.0, 'LineStyle', '-');
        grid on
        ylabel('\alpha, deg', 'FontSize', font_size);

        % THRUST
        ax5 = subplot(6, 1, 5); hold on;
        plot(dataOL.duration, dataOL.thrust, 'Color', 'b', ...
            'LineWidth', 1.0, 'LineStyle', '-');
        plot(dataCL.duration + dt.thrust/samplingFrequency, ...
            dataCL.thrust, 'Color', 'r', 'LineWidth', 1.0, 'LineStyle', '-');
        grid on
        ylabel('\delta_t, %', 'FontSize', font_size);

        % ELEVATOR
        ax6 = subplot(6, 1, 6); hold on;
        plot(dataOL.duration, dataOL.elevatorDefl, 'Color', 'b', ...
            'LineWidth', 1.0, 'LineStyle', '-');
        plot(dataCL.duration + dt.elevatorDefl/samplingFrequency, ...
            dataCL.elevatorDefl, 'Color', 'r', 'LineWidth', 1.0, 'LineStyle', '-');
        grid on
        ylabel('\delta_e, deg', 'FontSize', font_size);
        xlabel('Flight duration, s', 'FontSize', font_size);
        legend('without controller', 'with controller', 'Location', 'southeast');

    elseif contains(motionType, "lat", "IgnoreCase", true) || ...
            contains(motionType, "dir", "IgnoreCase", true)
        figure();  % state variables
        ax1 = subplot(4, 1, 1); hold on;
        plot(dataOL.duration, dataOL.yawRateBody, 'Color', 'b', ...
            'LineWidth', 1.0, 'LineStyle', '-');
        plot(dataCL.duration + dt.yawRateBody/samplingFrequency, ...
           dataCL.yawRateBody, 'Color', 'r', 'LineWidth', 1.0, 'LineStyle', '-');
        grid on
        xlim([0, 200]);
        ylabel('Yaw rate, deg/s', 'FontSize', 12);

        ax2 = subplot(4, 1, 2); hold on;
        plot(dataOL.duration, dataOL.sideslip, 'Color', 'b', ...
            'LineWidth', 1.0, 'LineStyle', '-');
        plot(dataCL.duration + dt.sideslip/samplingFrequency, ...
            dataCL.sideslip, 'Color', 'r', 'LineWidth', 1.0, 'LineStyle', '-');
        grid on
        xlim([0, 200]);
        ylabel('Sideslip, deg', 'FontSize', 12);

        ax3 = subplot(4, 1, 3); hold on;
        plot(dataOL.duration, dataOL.rollRateBody, 'Color', 'b', ...
            'LineWidth', 1.0, 'LineStyle', '-');
        plot(dataCL.duration + dt.rollRateBody/samplingFrequency, ...
            dataCL.rollRateBody, 'Color', 'r', 'LineWidth', 1.0, 'LineStyle', '-');
        grid on
        xlim([0, 200]);
        ylabel('Roll rate, deg/s', 'FontSize', 12);

        ax4 = subplot(4, 1, 4); hold on;
        plot(dataOL.duration, dataOL.rollAngleEuler, 'Color', 'b', ...
            'LineWidth', 1.0, 'LineStyle', '-');
        plot(dataCL.duration + dt.rollAngleEuler/samplingFrequency, ...
            dataCL.rollAngleEuler, 'Color', 'r', 'LineWidth', 1.0, 'LineStyle', '-');
        yline(26.3, 'Color', 'k', 'LineWidth', 1.0, 'LineStyle', '-');
        grid on
        xlim([0, 200]);
        ylabel('Bank angle, deg', 'FontSize', 12);
        xlabel('Flight duration, s', 'FontSize', 12);
        legend('without controller', 'with controller', 'required bank angle', ...
            'Location', 'southeast');

        figure(); hold on;  % Altitude
        plot(dataOL.duration, convlength(dataOL.altitude, 'm', 'ft'), ...
            'Color', 'b', 'LineWidth', 1.0, 'LineStyle', '-');
        % plot(dataCL.duration + dt.altitude/samplingFrequency, ...
        %     convlength(dataCL.altitude, 'm', 'ft'), 'Color', 'r', ...
        %     'LineWidth', 1.0, 'LineStyle', '-');
        plot(dataCL.duration, convlength(dataCL.altitude, 'm', 'ft'), ...
            'Color', 'r', 'LineWidth', 1.0, 'LineStyle', '-');
        yline(4000, 'Color', 'k', 'LineWidth', 1.0, 'LineStyle', '-');
        yline(4050, 'Color', 'k', 'LineWidth', 1.0, 'LineStyle', '--');
        yline(3950, 'Color', 'k', 'LineWidth', 1.0, 'LineStyle', '--');
        grid on
        xlim([0, 200]);
        ylabel('Altitude, ft', 'FontSize', 12);
        xlabel('Flight duration, s', 'FontSize', 12);
        legend('without controller', 'with controller', ...
            'trim altitude', 'altitude tolerance', ...
            'Location', 'best');

        figure(); hold on;  % Speed
        plot(dataOL.duration, convvel(dataOL.trueAirspeed, 'm/s', 'kts'), ...
            'Color', 'b', 'LineWidth', 1.0, 'LineStyle', '-');
        % plot(dataCL.duration + dt.trueAirspeed/samplingFrequency, ...
        %     convvel(dataCL.trueAirspeed, 'm/s', 'kts'), 'Color', 'r', ...
        %     'LineWidth', 1.0, 'LineStyle', '-');
        plot(dataCL.duration, convvel(dataCL.trueAirspeed, 'm/s', 'kts'), ...
            'Color', 'r', 'LineWidth', 1.0, 'LineStyle', '-');
        yline(180, 'Color', 'k', 'LineWidth', 1.0, 'LineStyle', '-');
        yline(200, 'Color', 'k', 'LineWidth', 1.0, 'LineStyle', '--');
        yline(175, 'Color', 'k', 'LineWidth', 1.0, 'LineStyle', '--');
        grid on
        ylim([170, 215]);
        xlim([0, 200]);
        ylabel('True airspeed, kts', 'FontSize', 12);
        xlabel('Flight duration, s', 'FontSize', 12);
        legend('without controller', 'with controller', ...
            'trim speed', 'speed tolerance', ...
            'Location', 'northwest');

        figure(); hold on;  % Yaw angle
        plot(dataOL.duration, dataOL.yawAngleEuler, ...
            'Color', 'b', 'LineWidth', 1.0, 'LineStyle', '-');
        % plot(dataCL.duration - dt.yawAngleEuler/samplingFrequency, ...
        %     dataCL.yawAngleEuler, 'Color', 'r', ...
        %     'LineWidth', 1.0, 'LineStyle', '-');
        plot(dataCL.duration, dataCL.yawAngleEuler, ...
            'Color', 'r', 'LineWidth', 1.0, 'LineStyle', '-');
        xline(120, 'Color', 'k', 'LineWidth', 1.0, 'LineStyle', '-');
        xline(110, 'Color', 'k', 'LineWidth', 1.0, 'LineStyle', '--');
        xline(130, 'Color', 'k', 'LineWidth', 1.0, 'LineStyle', '--');
        yline(dataCL.yawAngleEuler(1), 'Color', 'k', ...
            'LineWidth', 1.0, 'LineStyle', '-');
        yline(dataCL.yawAngleEuler(1) + 360, 'Color', 'k', ...
            'LineWidth', 1.0, 'LineStyle', '-');
        grid on
        xlim([0, 200]);
        text(165, 260, 'start heading', 'FontSize', 12);
        text(161.5, 600, 'target heading', 'FontSize', 12);
        ylabel('Yaw angle/Azimuth, deg', 'FontSize', 12);
        xlabel('Flight duration, s', 'FontSize', 12);
        legend('without controller', 'with controller', ...
            'required turn duration', 'duration tolerance', ...
            'Location', 'northwest');

        figure(); hold on;  % aileron deflection
        plot(dataOL.duration, dataOL.aileronDefl, ...
            'Color', 'b', 'LineWidth', 1.0, 'LineStyle', '-');
        plot(dataCL.duration + dt.aileronDefl/samplingFrequency, ...
            dataCL.aileronDefl, 'Color', 'r', ...
            'LineWidth', 1.0, 'LineStyle', '-');
        grid on
        xlim([0, 200]);
        ylabel('Aileron deflection, deg', 'FontSize', 12);
        xlabel('Flight duration, s', 'FontSize', 12);
        legend('without controller', 'with controller', 'Location', 'best');

        figure(); hold on;  % rudder deflection
        plot(dataOL.duration, dataOL.rudderDefl, ...
            'Color', 'b', 'LineWidth', 1.0, 'LineStyle', '-');
        plot(dataCL.duration, dataCL.rudderDefl, 'Color', 'r', ...
            'LineWidth', 1.0, 'LineStyle', '-');
        grid on
        xlim([0, 200]);
        ylabel('Rudder deflection, deg', 'FontSize', 12);
        xlabel('Flight duration, s', 'FontSize', 12);
        legend('without controller', 'with controller', 'Location', 'best');
    else
        error("invalid motion type.");
    end
    linkaxes([ax1, ax2, ax3, ax4, ax5, ax6],'x')

end