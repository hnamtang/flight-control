% Flight Control - TU Berlin
% WS 24/25
% HW 1 - Analysis of dynamic aircraft behavior
% and design of stability augmentation system.
%
% Task 3 - Design a base controller for decoupling lateral aircraft
% movement (SAS)
clear all; close all; clc

run("model_lat");

%% 1. Controller Design - Yaw damper(zetaDR = 0.7)

% Suitable feedback: r -> \delta_r
kRange = 0.5:0.001:1.5;
poles = rlocus(-sysLat(1, 2), kRange);
poleDR = poles(2, :);

% Compute damping of Dutch roll
zetaDR = -cos(angle(poleDR));

% Find the feedback gain for damping ratio of 0.7
[~, idx] = min(abs(zetaDR - 0.7));
Krdr = kRange(idx);

% Closed loop system with yaw damper
sysLatYawDamper = feedback(sysLat, Krdr, 2, 1, 1);

%% 2. Problem and solution of intended turn with yaw damper


% Washout filter
tauWashoutFilter = 5;
washoutFilter = generate_washout_filter(tauWashoutFilter);

% r-feedback to \delta_r
kRange = 0.5:0.001:1.5;
poles = rlocus(-washoutFilter * sysLat(1, 2), kRange);
poleDR = poles(2, :);

% Compute damping of Dutch roll
zetaDR = -cos(angle(poleDR));

% Find the feedback gain for damping ratio of 0.7
[~, idx] = min(abs(zetaDR - 0.7));
Krdr = kRange(idx);

% Closed loop system with yaw damper and washout filter
sysLatYawDamperWashout = feedback(sysLat, Krdr * washoutFilter, 2, 1, 1);

% phi-feedback to \delta_a
Kphida = 0.1;  % from root locus of sysLatYawDamperWashout

% Closed loop system with positive feedback
sysLatClosedLoop = feedback(sysLatYawDamperWashout, Kphida, 1, 4, 1);

[~, ~] = plot_step_response(sysLatClosedLoop, 50);

%% 3. Verification - Feedback \beta -> \delta_r

integrator = tf(1, [1, 0]);

Kbetadr = 0.125;  % from root locus of integrator * sysLatClosedLoop(2, 2)

% Closed loop system with negative feedback
sysCL = feedback(sysLatClosedLoop, Kbetadr * integrator, 2, 2, -1);

[~, ~] = plot_step_response(sysCL, 50);