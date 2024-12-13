% Flight Control - TU Berlin
% WS 24/25
% HW 1 - Analysis of dynamic aircraft behavior
% and design of stability augmentation system.
%
% Task 4 - Testing the controllers in the nonlinear simulation
%
% Author: H. N. Tang

clear all; close all; clc


%% Part I - Longitudinal Motion


%% Part II - Lateral-directional Motion
% Nauval
% dataOLLat = read_sim_data("0669", "lateral");
% dataCLLat = read_sim_data("0670", "lateral");

% Nam
dataOLLat = read_sim_data("0674", "lateral");
dataCLLat = read_sim_data("0675", "lateral");

dtLat = sim_data_delay(dataOLLat, dataCLLat, "lateral");
plot_sim_data(dataOLLat, dataCLLat, dtLat, "lateral")
