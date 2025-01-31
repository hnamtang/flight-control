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

% References
H0 = 20;  % ALtitude in m
Phi0 = 15;  % Bank angle in degrees
V0 = 17;  % True airspeed in m/s

% Tolerances
dH = 2;  % Altitude in m
dPhi = 5;  % Bank angle in degrees
dV = 2.5;  % True airspeed in m/s

% Relevant variables for import
columns = [2, 3, 6, 8, 16, 25, 26, 31, 32, 33, 37, 38, 39];


%% Part a) Open loop Aircraft - Import Data

path = './flight_data/Flugversuch_ol.xlsx';
% path = './flight_data/Flugversuch_ol1.xlsx';
opts = detectImportOptions(path);
opts.SelectedVariableNames = columns;
variable_names = opts.VariableNames(columns);
data_ol = readmatrix(path, opts);  % 10184 x 13

% t_start = 27;  % file: ol1
t_start = 52;  % file: ol2_stabil
[idx_start, idx_end] = find_index(data_ol(:,1), t_start, data_ol(:,7));

t_sim = data_ol(idx_start:idx_end,1);  % Relevant time
t_sim = t_sim - t_sim(1);


%% Part a) Open loop Aircraft - Evaluation

% Plots

% Altitude
H = data_ol(idx_start:idx_end,6);
plot_flight_data(t_sim, H, 'H', H0, dH);

% Bank angle
Phi = data_ol(idx_start:idx_end,2);
plot_flight_data(t_sim, Phi, 'Phi', Phi0, dPhi);

% Airspeed
V = data_ol(idx_start:idx_end,5);
plot_flight_data(t_sim, V, 'V', V0, dV);


%% Part b) Closed-loop Aircraft
path = './flight_data/Flugversuch_cl.xlsx';
opts = detectImportOptions(path);
opts.SelectedVariableNames = columns;
data_cl = readmatrix(path, opts);

t_start = 10;  % file: cl_final2
[idx_start, idx_end] = find_index(data_cl(:,1), t_start, data_cl(:,7));

t_sim = data_cl(idx_start:idx_end,1);  % Relevant time
t_sim = t_sim - t_sim(1);


%% Part b) Closed-loop Aircraft - Evaluation

% Plots

% Altitude
H = data_cl(idx_start:idx_end,6);
plot_flight_data(t_sim, H, 'H', H0, dH);

% Bank angle
Phi = data_cl(idx_start:idx_end,2);
plot_flight_data(t_sim, Phi, 'Phi', Phi0, dPhi);

% Airspeed
V = data_cl(idx_start:idx_end,5);
plot_flight_data(t_sim, V, 'V', V0, dV);







%% Define function
function plot_flight_data(t, data, variable_name, reference, tolerance)
% plot flight data of altitude H, bank angle Phi, and true airspeed V.

    if variable_name == "Phi"
        data = data * (180)/pi;
        lgd = {'Bank angle', 'Tolerance'};
        ylbl = 'Phi, °';
    elseif variable_name == "V"  % True airspeed V
        lgd = {'True airspeed', 'Tolerance'};
        ylbl = 'TAS, m/s';
    elseif variable_name == "H"  % Altitude H
        lgd = {'Altitude', 'Tolerance'};
        ylbl = 'Altitude (MSL), m';
    else
        error("Invalid index.");
    end

    figure(); plot(t, data, 'LineWidth', 1.5);
    hold on
    yline(reference + tolerance, 'k--');
    yline(reference - tolerance, 'k--');
    legend(lgd, 'Location', 'best');
    grid on
    xlabel('Time, s');
    ylabel(ylbl);
    title(lgd(1))
    axis padded

end