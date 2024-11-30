% Flight Control - TU Berlin
% WS 24/25
% HW 1 - Analysis of dynamic aircraft behavior and
% design of stability augmentation system.
%
% Task 1 - Analysis of eigenbehavior
clear all; close all; clc

run("model_lon");

delta_e = deg2rad(20);  % maximum deflection of elevator

%% 3.2.1.2. Phugoid stability
  
% % Complete longitudinal state-space model
% pzplot(sysLon);
% set(findall(gcf,'Type','Line'),'LineWidth',1.5,'MarkerSize',9)
% 
% % % Approximated phugoid model
% % Aphu = Alon(3:4, 3:4);
% % Bphu = Blon(3:4, 2);
% % Cphu = eye(2);
% % Dphu = zeros(2, 1);
% % sysPhu = ss(Aphu, Bphu, Cphu, Dphu);
% % sysPhu.StateName = sysLon.StateName(3:4);
% % sysPhu.InputName = sysLon.InputName(2);
% % sysPhu.OutputName = sysPhu.StateName;
% % sysPhu.InputUnit = sysLon.InputUnit(2);
% % sysPhu.OutputUnit = sysLon.OutputUnit(3:4);
% % 
% % plot_step_response(sysPhu, 200);
% 
% 
% % Approximated short-period model
% Asp = Alon(1:2, 1:2);
% Bsp = Blon(1:2, 2);
% Csp = eye(2);
% Dsp = zeros(2, 1);
% sysSP = ss(Asp, Bsp, Csp, Dsp);
% sysSP.StateName = sysLon.StateName(3:4);
% sysSP.InputName = sysLon.InputName(2);
% sysSP.OutputName = sysSP.StateName;
% sysSP.InputUnit = sysLon.InputUnit(2);
% sysSP.OutputUnit = sysLon.OutputUnit(3:4);
% 
% plot_step_response(delta_e*sysSP, 20);
% 
% % [y, tOut] = step(sysLon, 20);
% % subplot(2, 1, 1); hold on; plot(tOut, y(:, 1, 2), 'LineWidth', 1.5);
% % subplot(2, 1, 2); hold on; plot(tOut, y(:, 2, 2), 'LineWidth', 1.5);

%% 3.2.1.3 Flight-path stability
[y, tOut] = plot_step_response(delta_e * sysLon(:, 2), 100);
flightPath = y(:, 4) - y(:, 2);  % flight path = pitch angle - angle of attack

figure();
plot(convvel(y(:, 3), 'm/s', 'kts'), rad2deg(flightPath), 'LineWidth', 1.5);
grid on
xlabel("$V$ [kts]", "Interpreter", "latex", 'FontSize', 14);
ylabel("$\gamma$ $[^\circ]$", "Interpreter", "latex", 'FontSize', 14)
title("Flight-path Stability", "Interpreter", "latex", "FontSize", 16)