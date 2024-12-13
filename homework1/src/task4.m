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
% % Nauval
% dataOLLon = read_sim_data("0666", "longitudinal");
% dataCLLon = read_sim_data("0667", "longitudinal");
% 
% % Nam
% % dataOLLon = read_sim_data("0672", "longitudinal");
% % dataCLLon = read_sim_data("0673", "longitudinal");
% 
% dtLon = sim_data_delay(dataOLLon, dataCLLon, "longitudinal");
% plot_sim_data(dataOLLon, dataCLLon, dtLon, "longitudinal")

%% Part II - Lateral-directional Motion
% Nauval
% dataOLLat = read_sim_data("0669", "lateral");
% dataCLLat = read_sim_data("0670", "lateral");

% Nam
dataOLLat = read_sim_data("0674", "lateral");
dataCLLat = read_sim_data("0675", "lateral");

dtLat = sim_data_delay(dataOLLat, dataCLLat, "lateral");
plot_sim_data(dataOLLat, dataCLLat, dtLat, "lateral")
