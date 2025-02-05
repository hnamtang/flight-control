% Flight Control - TU Berlin
% WS 24/25
% HW 2 - Analysis of dynamic aircraft behavior
% and design of control systems for manual and automatic flight
%
% Task 2 - Design of a Rate Command/Attitude Hold Controller
% Evaluation of Flight Data
%
% Author: H. N. Tang
clear all; close all; clc

% Import controller
run('./task2');

% References
H0 = 20;  % ALtitude in m
Phi0 = 15;  % Bank angle in degrees
V0 = 17;  % True airspeed in m/s

% Tolerances
dH = 2;  % Altitude in m
dPhi = 5;  % Bank angle in degrees
dV = 2.5;  % True airspeed in m/sclos

% Relevant variables for import
columns = [2, 3, 6, 8, 15, 25, 26, 31, 32, 33, 38, 39];


%% Part a) Open loop Aircraft - Import Data

path = './flight_data/Flugversuch_ol.xlsx';
opts = detectImportOptions(path);
opts.SelectedVariableNames = columns;
variable_names = opts.VariableNames(columns);
data_ol = readmatrix(path, opts);  % 10184 x 12

t_start = 52;
[idx_start, idx_end] = find_index(data_ol(:,1), t_start, data_ol(:,7));

t_sim = data_ol(idx_start:idx_end,1);  % Relevant time
t_sim = t_sim - t_sim(1);

disp(variable_names);


%% Part a) Open loop Aircraft - Evaluation

% Plots
% Altitude
H = data_ol(idx_start:idx_end,6);
% fig_H_ol = plot_flight_data(t_sim, H, 'H', H0, dH);
% exportgraphics(fig_H_ol, "task2.7.H_ol.eps", 'Resolution', 600);

% Bank angle
Phi = data_ol(idx_start:idx_end,2);
% fig_Phi_ol = plot_flight_data(t_sim, Phi, 'Phi', Phi0, dPhi);
% exportgraphics(fig_Phi_ol, "task2.7.Phi_ol.eps", 'Resolution', 600);

% Airspeed
V = data_ol(idx_start:idx_end,5);
% fig_V_ol = plot_flight_data(t_sim, V, 'V', V0, dV);
% exportgraphics(fig_V_ol, "task2.7.V_ol.eps", 'Resolution', 600);

% fig_all_ol = figure();
% subplot(3, 1, 1); hold on;
% plot(t_sim, rad2deg(Phi), 'LineWidth', 1.5);
% yline(Phi0 + dPhi, 'LineWidth', 1.5, 'LineStyle', '--', 'Color', 'k');
% yline(Phi0 - dPhi, 'LineWidth', 1.5, 'LineStyle', '--', 'Color', 'k');
% hold off;
% grid on;
% ylabel('Phi, deg');
% axis padded;
% subplot(3, 1, 2); hold on;
% plot(t_sim, H, 'LineWidth', 1.5);
% yline(H0 + dH, 'LineWidth', 1.5, 'LineStyle', '--', 'Color', 'k');
% yline(H0 - dH, 'LineWidth', 1.5, 'LineStyle', '--', 'Color', 'k');
% hold off;
% grid on;
% ylabel('Altitude, m');
% axis padded
% subplot(3, 1, 3); hold on;
% plot(t_sim, V, 'LineWidth', 1.5);
% yline(V0 + dV, 'LineWidth', 1.5, 'LineStyle', '--', 'Color', 'k');
% yline(V0 - dV, 'LineWidth', 1.5, 'LineStyle', '--', 'Color', 'k');
% hold off;
% grid on;
% ylabel('Indicated airspeed, m/s');
% xlabel('Time, s');
% axis padded
% 
% exportgraphics(fig_all_ol, "task2.7.all_ol.eps", 'Resolution', 600);



%% Part b) Closed-loop Aircraft
path = './flight_data/Flugversuch_cl.xlsx';
opts = detectImportOptions(path);
opts.SelectedVariableNames = columns;
data_cl = readmatrix(path, opts);

t_start = 10;
[idx_start, idx_end] = find_index(data_cl(:,1), t_start, data_cl(:,7));

t_sim = data_cl(idx_start:idx_end,1);  % Relevant time
t_sim = t_sim - t_sim(1);


%% Part b) Closed-loop Aircraft - Evaluation

% Plots
% Altitude
H = data_cl(idx_start:idx_end,6);
% plot_flight_data(t_sim, H, 'H', H0, dH);

% Bank angle
Phi = data_cl(idx_start:idx_end,2);
% plot_flight_data(t_sim, Phi, 'Phi', Phi0, dPhi);

% Airspeed
V = data_cl(idx_start:idx_end,5);
% plot_flight_data(t_sim, V, 'V', V0, dV);

% fig_all_cl = figure();
% subplot(3, 1, 1); hold on;
% plot(t_sim, rad2deg(Phi), 'LineWidth', 1.5);
% yline(Phi0 + dPhi, 'LineWidth', 1.5, 'LineStyle', '--', 'Color', 'k');
% yline(Phi0 - dPhi, 'LineWidth', 1.5, 'LineStyle', '--', 'Color', 'k');
% hold off;
% grid on;
% ylabel('Phi, deg');
% axis padded;
% subplot(3, 1, 2); hold on;
% plot(t_sim, H, 'LineWidth', 1.5);
% yline(H0 + dH, 'LineWidth', 1.5, 'LineStyle', '--', 'Color', 'k');
% yline(H0 - dH, 'LineWidth', 1.5, 'LineStyle', '--', 'Color', 'k');
% hold off;
% grid on;
% ylabel('Altitude, m');
% axis padded
% subplot(3, 1, 3); hold on;
% plot(t_sim, V, 'LineWidth', 1.5);
% yline(V0 + dV, 'LineWidth', 1.5, 'LineStyle', '--', 'Color', 'k');
% yline(V0 - dV, 'LineWidth', 1.5, 'LineStyle', '--', 'Color', 'k');
% hold off;
% grid on;
% ylabel('Indicated airspeed, m/s');
% xlabel('Time, s');
% axis padded
% 
% exportgraphics(fig_all_cl, "task2.7.all_cl.eps", 'Resolution', 600);

% % Comparison linear vs. nonlinear
% roll_rate_command = data_cl(idx_start:idx_end,11);
% Phi_linear = lsim(sys4('\delta \Phi',1), roll_rate_command, t_sim);
% 
% fig_compare = figure();
% subplot(2, 1, 1); hold on;
% plot(t_sim, rad2deg(Phi_linear), 'LineWidth', 1.5);
% plot(t_sim, rad2deg(Phi), 'LineWidth', 1.5);
% grid on;
% hold off;
% ylabel('Bank angle, deg');
% legend('linear', 'nonlinear', 'FontSize', 11, 'Location', 'best');
% axis padded
% 
% subplot(2, 1, 2);
% plot(t_sim, rad2deg(roll_rate_command), ...
%     'LineWidth', 1.5, 'LineStyle', '--', 'Color', 'k');
% grid on;
% ylabel('Roll rate command, deg/s');
% xlabel('Time, s');
% axis padded
% 
% exportgraphics(fig_compare, "task2.7.compare.eps", 'Resolution', 600);


%% Define functions
function fig = plot_flight_data(t, data, variable_name, reference, tolerance)
% plot flight data of altitude H, bank angle Phi, and indicated airspeed V.

    if variable_name == "Phi"
        data = data * (180)/pi;
        lgd = {'Bank angle', 'Tolerance'};
        ylbl = 'Phi, deg';
    elseif variable_name == "V"  % indicated airspeed V
        lgd = {'Indicated airspeed', 'Tolerance'};
        ylbl = 'IAS, m/s';
    elseif variable_name == "H"  % altitude H
        lgd = {'Altitude', 'Tolerance'};
        ylbl = 'Altitude (MSL), m';
    else
        error("Invalid index.");
    end

    fig = figure(); plot(t, data, 'LineWidth', 1.5);
    hold on
    yline(reference + tolerance, 'LineWidth', 1.5, 'LineStyle', '--', 'Color', 'k');
    yline(reference - tolerance, 'LineWidth', 1.5, 'LineStyle', '--', 'Color', 'k');
    legend(lgd, 'FontSize', 11, 'Location', 'best');
    grid on
    xlabel('Time, s');
    ylabel(ylbl);
    %title(lgd(1))
    axis padded

end