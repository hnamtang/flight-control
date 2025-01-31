% Flight Control - TU Berlin
% WS 24/25
% HW 2 - Analysis of dynamic aircraft behavior
% and design of control systems for manual and automatic flight
%
% Task 2 - Design of a Rate Command/Attitude Hold Controller
% Author: H. N. Tang
clear all; close all; clc

% Import state-space model of Talon UAV (source: AlphaLink Engineering)
run('../VFTE_state_space');

s = tf('s');

% Extract relevant variables
sysLat = ss(G.A(5:8,5:8), G.B(5:8,3:4), G.C([5:8,11],5:8), G.D([5:8,11],3:4));
sysLat.StateName = G.StateName(5:8);
sysLat.InputName = G.InputName(3:4);
sysLat.OutputName = G.OutputName([5:8,11]);

% sysLat = G([5:8, 11],3:4);


% Filters
T_p = 0.1;
GF_p = 1 / (T_p*s+1);
GF_p.InputName = "\delta p";
GF_p.OutputName = "\delta pf";  % f stands for filtered

T_rTP = 0.1;
LP_r = 1 / (T_rTP*s+1);
T_rHP = 2;
HP_r = T_rHP*s / (T_rHP*s+1);
GF_r = LP_r * HP_r;
GF_r.InputName = "\delta r";
GF_r.OutputName = "\delta rf";


% Lateral dynamics with all pre-defined filters
sysLatF = connect(sysLat, GF_r, GF_p, ...
    sysLat.InputName, [sysLat.OutputName; GF_r.OutputName; GF_p.OutputName]);


%% Part 2) Coordinated Turn
K_zetaxi = tf(-dcgain(sysLatF('\delta \beta','\delta \xi')) / dcgain(sysLatF('\delta \beta','\delta \zeta')));
K_zetaxi.InputName = "\delta \xi cmd";
K_zetaxi.OutputName = "yK \zeta \xi";

K_xizeta = 1 / K_zetaxi;
K_xizeta.InputName = "\delta \zeta cmd";
K_xizeta.OutputName = "yK \xi \zeta";

S1 = sumblk("\delta \xi = \delta \xi cmd + yK \xi \zeta");
S2 = sumblk("\delta \zeta = \delta \zeta cmd + yK \zeta \xi");

sys1 = connect(K_zetaxi, K_xizeta, sysLatF, S1, S2, ...
    {'\delta \xi cmd', '\delta \zeta cmd'}, ...
    [sysLatF.OutputName; '\delta \xi'; '\delta \zeta']);

% DONE.


%% Part 3) Yaw Damper
 
% figure(); rlocus(-sys1('\delta rf','\delta \zeta cmd')); grid on
% set(findall(gcf, 'Type', 'Line'), 'LineWidth', 1.5, 'MarkerSize', 9)

% Find k_zetar to achieve max. Dutch Roll damping
Krlocus = linspace(0.13, 0.3, 1000);
R = rlocus(-sys1('\delta rf','\delta \zeta cmd'), Krlocus);
zeta_DR = -cos(angle(R(find(imag(R(:,1)) ~= 0, 1),:)));
[~, idx] = max(zeta_DR);
% k_zetar = Krlocus(idx);
k_zetar = 0.08;  % test1: damping 0.3 -> max k_xip 0.0078
K_zetar = tf(-k_zetar);
K_zetar.InputName = GF_r.OutputName;
K_zetar.OutputName = "yK zetar";

S3 = sumblk("\delta \zeta cmd = \delta \zeta input - yK zetar");

sys1cl = connect(sys1, K_zetar, S3, ...
    [sys1.InputName(1); S3.InputName(1)], sys1.OutputName);

% DONE.


%% Part 4) Roll Damper

% figure(); rlocus(-sys1cl('\delta pf','\delta \xi cmd')); grid on
% set(findall(gcf, 'Type', 'Line'), 'LineWidth', 1.5, 'MarkerSize', 9)

k_xip = 0.003;  % origin, also test1
K_xip = tf(-k_xip);
K_xip.InputName = GF_p.OutputName;
K_xip.OutputName = "yK xip";

S4 = sumblk("\delta \xi cmd = \delta \xi input - yK xip");

sys2cl = connect(sys1cl, K_xip, S4, ...
    [S4.InputName(1); sys1cl.InputName(2)], sys1cl.OutputName);

% DONE.


%% Part 5) Outer Loop - Bank Angle Command

S5 = sumblk("e \Phi dot = \delta \Phi cmd - \delta \Phi");

% PI compensator
G_Phixiinput = sys2cl('\delta \Phi','\delta \xi input');

% opts = pidtuneOptions('PhaseMargin', 45, 'DesignFocus', 'reference-tracking');
% [PI_comp, info_PI] = pidtune(G_Phixiinput, 'PI', opts);
% [PI_comp, info_PI] = pidtune(G_Phixiinput, 'PI', 2);  % good
[PI_comp, info_PI] = pidtune(G_Phixiinput, 'PI', 2.2);
PI_comp.InputName = "e \Phi dot";
PI_comp.OutputName = "\delta \xi input";

% -> Gm = 6.23 dB, Pm = 60°

sys3_PIcl = connect(sys2cl, PI_comp, S5, ...
    [S5.InputName(1); sys2cl.InputName(2)], sys2cl.OutputName);

% DONE.


%% Part 6) Feedforward controller

G_PhiPhicmd = sys3_PIcl('\delta \Phi','\delta \Phi cmd');

% opts = pidtuneOptions('PhaseMargin', 45, 'DesignFocus', 'reference-tracking');
% [PI_FF, info_PIFF] = pidtune(G_PhiPhicmd, 'PI', opts)
zPI_FF = -2.2;
PI_FF = ((-1/zPI_FF)*s + 1) / s;
PI_FF.InputName = "p cmd";
PI_FF.OutputName = "\delta \Phi cmd";

% -> Gm = 6.27 dB, Pm = 84.7°

figure(); bode(PI_FF * G_PhiPhicmd); grid on;
set(findall(gcf, 'Type', 'Line'), 'LineWidth', 1.5, 'MarkerSize', 9)

% Open loop
sys4 = connect(sys3_PIcl, PI_FF, ...
    [PI_FF.InputName; sys3_PIcl.InputName(2)], ...
    [sys3_PIcl.OutputName; PI_FF.OutputName]);

% DONE.


%%
% Sim
t = 0:0.01:20;
u = [deg2rad(50); 0] * ((t>1) - (t>10));
y = lsim(sys4([1:5, 8, 9, 10],:), u, t);
convFactor = (180/pi) * ones(1, 8); convFactor(5) = 1;
y = y * diag(convFactor);

% figure();
% subplot(5, 1, 1); plot(t, y(:,1), 'LineWidth', 1.5); grid on;
% ylabel('r, °/s');
% title('Output');
% subplot(5, 1, 2); plot(t, y(:,2), 'LineWidth', 1.5); grid on;
% ylabel('beta, °');
% subplot(5, 1, 3); plot(t, y(:,3), 'LineWidth', 1.5); grid on;
% ylabel('p, °/s');
% subplot(5, 1, 4); plot(t, y(:,4), 'LineWidth', 1.5); grid on;
% hold on; plot(t, y(:,8), 'LineWidth', 1.5);
% ylabel('Phi, °');
% subplot(5, 1, 5); plot(t, y(:,5), 'LineWidth', 1.5); grid on;
% ylabel('b_y');
% xlabel('Time, s');

% figure();
% subplot(2, 1, 1); plot(t, y(:,6), 'LineWidth', 1.5); grid on;
% ylabel('xi, °');
% subplot(2, 1, 2); plot(t, y(:,7), 'LineWidth', 1.5); grid on;
% ylabel('zeta, °');
% xlabel('Time, s');


figure(); hold on
plot(t, y(:,4), 'LineWidth', 1.5);
plot(t, y(:,8), 'LineWidth', 1.5);
yline(60, 'k--');
grid on
legend('Phi', 'Phi cmd', '60°', 'Location', 'best');
ylabel('Phi, °');
xlabel('Time, s');


disp([num2str((max(y(:,4)) - y(end,4))*100 / y(end,4)), '%']);


%% Display all gain values

% Part 2
disp(['K_zetaxi = ', num2str(dcgain(K_zetaxi))]);
disp(['K_xizeta = ', num2str(dcgain(K_xizeta))]);

% Part 3
disp(['K_zetar = ', num2str(-k_zetar)]);

% Part 4
disp(['K_xip = ', num2str(-k_xip)]);

% Part 5
[num_PI, ~] = tfdata(PI_comp, 'v');
disp(['K_PhiP = ', num2str(num_PI(1))]);
disp(['K_PhiI = ', num2str(num_PI(2))]);

% Part 6
[num_FF, ~] = tfdata(PI_FF, 'v');
disp(['K_Phi = ', num2str(num_FF(1))]);
