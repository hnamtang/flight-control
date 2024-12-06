function plot_sim_data(dataNoCtrl, dataWithCtrl, motion)

    dataNoCtrl = dataNoCtrl.data;
    dataWithCtrl = dataWithCtrl.data;

    timeNoCtrl = dataNoCtrl.wd_time;
    timeNoCtrl = timeNoCtrl - timeNoCtrl(1);  % shift the timestamp to zero
    timeWithCtrl = dataWithCtrl.wd_time;
    timeWithCtrl = timeWithCtrl - timeWithCtrl(1);  % shift the timestamp to zero

    figure();
    if motion == "lon"
        subplot(4, 1, 1); hold on;
        plot(timeNoCtrl, dataNoCtrl.wd_qb, 'Color', 'b', 'LineWidth', 1.5);
        plot(timeWithCtrl, dataWithCtrl.wd_qb, 'Color', 'r', 'LineWidth', 1.5);
        grid on
        ylabel('Body pitch rate (deg/s)')
        legend('no controller', 'with controller', 'Location', 'northeast');
        xlim([0, 130]);  % Duration of 130 s

        subplot(4, 1, 2); hold on;
        plot(timeNoCtrl, dataNoCtrl.wd_al, 'Color', 'b', 'LineWidth', 1.5);
        plot(timeWithCtrl, dataWithCtrl.wd_al, 'Color', 'r', 'LineWidth', 1.5);
        grid on
        ylabel('AoA (deg)');
        xlim([0, 130]);  % Duration of 130 s

        subplot(4, 1, 3); hold on;
        plot(timeNoCtrl, dataNoCtrl.wd_vfpath, 'Color', 'b', 'LineWidth', 1.5);
        plot(timeWithCtrl, dataWithCtrl.wd_vfpath, 'Color', 'r', 'LineWidth', 1.5);
        grid on
        ylabel('Flight path velocity (kts)');
        xlim([0, 130]);  % Duration of 130 s

        subplot(4, 1, 4); hold on;
        plot(timeNoCtrl, dataNoCtrl.wd_theta, 'Color', 'b', 'LineWidth', 1.5);
        plot(timeWithCtrl, dataWithCtrl.wd_theta, 'Color', 'r', 'LineWidth', 1.5);
        grid on
        ylabel('Pitch angle (deg)');
        xlim([0, 130]);  % Duration of 130 s

    elseif motion == "lat"
        subplot(4, 1, 1); hold on;
        plot(timeNoCtrl, dataNoCtrl.wd_rb, 'Color', 'b', 'LineWidth', 1.5);
        plot(timeWithCtrl, dataWithCtrl.wd_rb, 'Color', 'r', 'LineWidth', 1.5);
        grid on
        ylabel('Body yaw rate (deg/s)')
        legend('no controller', 'with controller', 'Location', 'northeast');
        xlim([0, 130]);  % Duration of 130 s

        subplot(4, 1, 2); hold on;
        plot(timeNoCtrl, dataNoCtrl.wd_beta, 'Color', 'b', 'LineWidth', 1.5);
        plot(timeWithCtrl, dataWithCtrl.wd_beta, 'Color', 'r', 'LineWidth', 1.5);
        grid on
        ylabel('Sideslip angle (deg)');
        xlim([0, 130]);  % Duration of 130 s

        subplot(4, 1, 3); hold on;
        plot(timeNoCtrl, dataNoCtrl.wd_pb, 'Color', 'b', 'LineWidth', 1.5);
        plot(timeWithCtrl, dataWithCtrl.wd_pb, 'Color', 'r', 'LineWidth', 1.5);
        grid on
        ylabel('Body roll rate (deg/s)');
        xlim([0, 130]);  % Duration of 130 s

        subplot(4, 1, 4); hold on;
        plot(timeNoCtrl, dataNoCtrl.wd_phi, 'Color', 'b', 'LineWidth', 1.5);
        plot(timeWithCtrl, dataWithCtrl.wd_phi, 'Color', 'r', 'LineWidth', 1.5);
        grid on
        ylabel('Bank angle (deg)');
        xlim([0, 130]);  % Duration of 130 s

    else

        error('invalid motion.');

    end

end