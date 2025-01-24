% Flight Control - TU Berlin
% WS 24/25
% HW 2 - Analysis of dynamic aircraft behavior
% and design of control systems for manual and automatic flight
%
% Task 2 - Design of a Rate Command/Attitude Hold Controller
% Author: H. N. Tang
clear all; close all; clc

% Import state-space model of Talon UAV (source: AlphaLink Engineering)
run('VFTE_state_space');

s = tf('s');

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


sysLat = G([5:8, 11], 3:4);


%% Part 2) Coordinated turn
K_zetaxi = tf(-dcgain(sysLat('\delta \beta','\delta \xi')) / dcgain(sysLat('\delta \beta','\delta \zeta')));
K_zetaxi.InputName = "\delta \xi cmd";
K_zetaxi.OutputName = "yK \zeta \xi";

K_xizeta = 1 / K_zetaxi;
K_xizeta.InputName = "\delta \zeta cmd";
K_xizeta.OutputName = "yK \xi \zeta";

S1 = sumblk("\delta \xi = \delta \xi cmd + yK \xi \zeta");
S2 = sumblk("\delta \zeta = \delta \zeta cmd + yK \zeta \xi");

sys1 = connect(K_zetaxi, K_xizeta, sysLat, S1, S2, ...
    {'\delta \xi cmd', '\delta \zeta cmd'}, ...
    [sysLat.OutputName; "\delta \xi"; "\delta \zeta"]);

% DONE.


%% Part 3) Yaw rate feedback
sys2 = connect(sys1, GF_r, ...
    sys1.InputName, [sys1.OutputName; GF_r.OutputName]);

% rlocus(-sys2('\delta rf', '\delta \zeta cmd'));
%K_zetar = tf(-0.25);
%K_zetar = tf(-0.15);
%K_zetar = tf(-0.078);
%K_zetar = tf(-0.17);
K_zetar = tf(-0.172);
K_zetar.InputName = GF_r.OutputName;
K_zetar.OutputName = "yK \zeta r";

S3 = sumblk("\delta \zeta cmd = \delta \zeta input - yK \zeta r");
sys3 = connect(sys2, K_zetar, S3, ...
    [sys1.InputName(1); "\delta \zeta input"], sys2.OutputName);

% DONE.


%% Part 4) Roll rate feedback

% max aileron deflection of 20°
u = deg2rad(20);

sys4 = connect(sys3, GF_p, ...
    sys3.InputName, [sys3.OutputName; GF_p.OutputName]);

%rlocus(-sys4('\delta pf', '\delta \xi cmd'));

%K_xip = tf(-0.0061);
%K_xip = tf(-0.0024);
%K_xip = tf(-0.0071);
K_xip = tf(-0.00724);
K_xip.InputName = GF_p.OutputName;
K_xip.OutputName = "yK \xi p";

S4 = sumblk("\delta \xi cmd = \delta \xi input - yK \xi p");
sys5 = connect(sys4, K_xip, S4, ...
    {"\delta \xi input", "\delta \zeta input"}, sys4.OutputName);



% Problem: roll is too sensitive. What to do?


% TODO: redesign K_xip according to MIL-F-8785C





%% Part 5) Outer loop - Bank angle command

% Design 1:
% % Only P-controller
% % First, calculate gain margin
% [Gm, Pm, Wcg, Wcp] = margin(-sys5('\delta \Phi', '\delta \xi input'));
% K_PhiP = tf(-Gm / db2mag(6));  % gain margin of S (requirement: 6 dB gain margin)
% %K_PhiP = tf(0.002);  % from root locus
% %K_PhiP = tf(-0.025);  % from root locus
% %K_PhiP = tf(-0.049);  % from root locus
% K_PhiP.InputName = "e \Phi dot";
% K_PhiP.OutputName = "yK \Phi P";
% 
% S5 = tf(1); S5.InputName = K_PhiP.OutputName; S5.OutputName = sys5.InputName(1);
% S6 = sumblk("e \Phi dot = \delta \Phi cmd - \delta \Phi");
% 
% sys6 = connect(sys5, K_PhiP, S5, S6, ...
%     [S6.InputName(1); sys5.InputName(2)], sys5.OutputName);
% 
% % PI-controller
% integrator = tf(1, [1, 0]);
% integrator.InputName = "e \Phi dot";
% integrator.OutputName = "yI";
% 
% S5 = sumblk("\delta \xi input = yK \Phi P + yI");
% 
% sys7 = connect(sys5, K_PhiP, integrator, S5, S6, ...
%     [S6.InputName(1); sys5.InputName(2)], sys5.OutputName);


% Design 2:
% Only P-controller
% First, calculate gain margin
[Gm, Pm, Wcg, Wcp] = margin(-sys5('\delta \Phi', '\delta \xi input'));
K_PhiP = tf(-Gm / db2mag(6));  % gain margin of S (requirement: 6 dB gain margin)
%K_PhiP = tf(0.002);  % from root locus
%K_PhiP = tf(-0.025);  % from root locus
%K_PhiP = tf(-0.049);  % from root locus
K_PhiP.InputName = "e \Phi dot";
K_PhiP.OutputName = "yK \Phi P";

S5 = tf(1); S5.InputName = K_PhiP.OutputName; S5.OutputName = sys5.InputName(1);
S6 = sumblk("e \Phi dot = \delta \Phi cmd - \delta \Phi");

sys6 = connect(sys5, K_PhiP, S5, S6, ...
    [S6.InputName(1); sys5.InputName(2)], sys5.OutputName);

% PI-controller
%K_PhiP = -Gm / db2mag(6);
K_PhiP = Gm / db2mag(6);
%K_PhiI = 1;
K_PhiI = 10;
PI = (K_PhiP*s + K_PhiI) / s;
PI.InputName = "yK";
PI.OutputName = "\delta \xi input";

ol = connect(sys5, PI, ...
    [PI.InputName; sys5.InputName(2)], sys5.OutputName);

% sys7_2 = connect(sys5, PI, S6, ...
%     [S6.InputName(1); sys5.InputName(2)], sys5.OutputName);

% Required gain margin (6 dB can easily be achieved)
% Design to fulfil required phase margin (45°)
W = logspace(-3, 2, 100000);
[MAG, PHASE] = bode(-ol('\delta \Phi', 'yK'), W);
MAG = squeeze(MAG); PHASE = squeeze(PHASE);

[~, idx] = min(abs(PHASE - (45 - 180)));
mag = MAG(idx);
K = tf(-1/mag);
K.InputName = "e \Phi dot";
K.OutputName = "yK";

sys7 = connect(ol, K, S6, ...
    [S6.InputName(1); sys5.InputName(2)], ...
    sys5.OutputName);

% DONE.


%% Part 6) Feedforward controller - Roll command












%% Test sim
t = 0:0.01:100;
u_xi = [1; 0] * ones(size(t));  % step aileron only
u_zeta = [0; 1] * ones(size(t));  % step rudder only
%lsim(sys1, u, t); grid on; set(findall(gcf,'Type','Line'),'LineWidth',1.5,'MarkerSize',9)
