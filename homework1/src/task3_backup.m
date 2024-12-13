% Flight Control - TU Berlin
% WS 24/25
% HW 1 - Analysis of dynamic aircraft behavior
% and design of stability augmentation system.
%
% Task 3 - Design a base controller for decoupling lateral aircraft
% movement (SAS)
%
% Author: H. N. Tang
%
clear all; close all; clc

run("model_lat");


%% 1. Controller Design - Yaw damper(zetaDR = 0.7)

% Feedback: yaw rate -> rudder
kRange = 0.5:0.001:1.5;
%kRange = 0.5:0.001:1.65;
poles = rlocus(-sysLat('yaw rate', 'rudder'), kRange);
poleDR = poles(imag(poles) > 0);

% Compute damping of Dutch roll
zetaDR = -cos(angle(poleDR));

% Find the feedback gain for damping ratio of 0.7
[~, idx] = min(abs(zetaDR - 0.7));
%[~, idx] = min(abs(zetaDR - 0.98));
Krdr = ss(kRange(idx), 'InputName', 'yaw rate', 'OutputName', 'rudder');

% Closed loop system with yaw damper
sysLatYawDamper = feedback(sysLat, Krdr, 'name', +1);  % positive feedback


%% 2. Problem and solution of intended turn with yaw damper

% Washout filter
tauWashoutFilter = 4;
washoutFilter = generate_washout_filter(tauWashoutFilter);

% Feedback: yaw rate -> rudder
kRange = 0.5:0.001:1.5;
poles = rlocus(-washoutFilter * sysLat('yaw rate', 'rudder'), kRange);
poleDR = poles(imag(poles) > 0);

% Compute damping of Dutch roll
zetaDR = -cos(angle(poleDR));

% Find the feedback gain for damping ratio of 0.7
[~, idx] = min(abs(zetaDR - 0.7));
Krdr = ss(kRange(idx), 'InputName', 'washed out yaw rate', ...
    'OutputName', 'rudder');

% Closed loop system with yaw damper and washout filter
sysLatYawDamperWashout = feedback(sysLat, Krdr * washoutFilter, 'name', +1);

% Feedback: bank angle -> aileron
% Kphida = 0.2 (from root locus of sysLatYawDamperWashout)
%Kphida = 0.1;  % from root locus of sysLatYawDamperWashout
Kphida = ss(0.2, 'InputName', 'bank angle', 'OutputName', 'aileron');

% Closed loop system with positive feedback
sysLatClosedLoop = feedback(sysLatYawDamperWashout, Kphida, 'name', +1);

% Simulation initial conditions
% states: yaw rate, sideslip angle, roll rate, bank angle, washout state
x0 = [0; 0; 0; deg2rad(5); 0];  % phi0 = 5 deg


%% 3. Verification - Feedback sideslip angle -> rudder

% Redesign lateral-directional controller
% to achieve better coordinated turn

% Feedback: yaw rate -> rudder
kRange = 0.5:0.001:1.5;
poles = rlocus(-washoutFilter * sysLat('yaw rate', 'rudder'), kRange);
poleDR = poles(imag(poles) > 0);

% Compute damping of Dutch roll
zetaDR = -cos(angle(poleDR));

% Find the feedback gain for damping ratio of 0.9
[~, idx] = min(abs(zetaDR - 0.9));
Krdr = ss(kRange(idx), 'InputName', 'washed out yaw rate', ...
    'OutputName', 'rudder');

% Closed loop system with yaw damper and washout filter
sysLatYawDamperWashout = feedback(sysLat, Krdr * washoutFilter, 'name', +1);


% Feedback: bank angle -> aileron
% Kphida = 0.1 (from root locus of sysLatYawDamperWashout)
Kphida = ss(0.1, 'InputName', 'bank angle', 'OutputName', 'aileron');

% Closed loop system with positive feedback
sysLatClosedLoop = feedback(sysLatYawDamperWashout, Kphida, 'name', +1);


% Feedback: sideslipe angle -> rudder
kRange = 0.1:0.001:1.5;
poles = rlocus(sysLatClosedLoop('sideslip angle', 'rudder'), kRange);
poleDR = poles(imag(poles) > 0);

% Compute damping of Dutch roll
zetaDR = -cos(angle(poleDR));

% Find the feedback gain for damping ratio of 0.7
[~, idx] = min(abs(zetaDR - 0.7));
Kbetadr = ss(kRange(idx), 'InputName', 'sideslip angle', ...
    'OutputName', 'rudder');

% Closed loop system with all feedback controllers
sysLatCL = feedback(sysLatClosedLoop, Kbetadr, 'name', -1);


%% Simulation
figure(); step(deg2rad(20) * sysLatCL, 20); grid on;
set(findall(gcf,'Type','Line'),'LineWidth',1.5,'MarkerSize',9)

figure(); step(deg2rad(20) * sysLat, 20); grid on;
hold on; step(deg2rad(20) * sysLatYawDamper, 20);
step(deg2rad(20) * sysLatYawDamperWashout, 20);
step(deg2rad(20) * sysLatClosedLoop, 20);
step(deg2rad(20) * sysLatCL, 20);
legend('Lat', 'YawDamper', 'YDWashout', 'ClosedLoop', 'CL')
title('Step response')
set(findall(gcf,'Type','Line'),'LineWidth',1.5,'MarkerSize',9)

rad2deg(dcgain(deg2rad(20) * sysLatCL))

plot_doublet_response(sysLatCL, deg2rad(20), 20);


% Actuator
Krdr = 1.265; 
Kphida = 0.1;
Kbetadr = 0.901;

K = [0, 0, 0, -Kphida, 0; ...
     -Krdr, Kbetadr, 0, 0, Krdr];

% Augment the open loop system with washout filter
% Aaug = [Alat,                zeros(size(Alat, 1), 1); ...
%         zeros(1, size(Alat, 2)), -1/tauWashoutFilter];
% Baug = [Blat,               zeros(size(Blat, 1), 1); ...
%         zeros(1, size(Blat, 2)), 1/tauWashoutFilter];
% Caug = [Clat, zeros(size(Clat, 1), 1)];

sysLatAug = connect(sysLat, washoutFilter, ...
    sysLat.InputName, [sysLat.OutputName; {'washed out yaw rate'}]);