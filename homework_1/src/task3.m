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


%% 1. Controller Design - Yaw Damper (zetaDR = 0.7)

% Feedback: yaw rate -> rudder
kRange = 0.5:0.001:1.5;
%kRange = 0.5:0.001:1.65;
poles = rlocus(-sysLat('yaw rate', 'rudder'), kRange);
poleDR = poles(imag(poles) > 0);

% Compute damping of Dutch roll
zetaDR = -cos(angle(poleDR));

% Find the feedback gain for damping ratio of 0.7
[~, idx] = min(abs(zetaDR - 0.7));
Krdr = ss(kRange(idx), 'InputName', 'yaw rate', 'OutputName', 'rudder');

% Closed loop system with yaw damper
sysLatYawDamper = feedback(sysLat, Krdr, 'name', +1);


%% 2. Problem and Solution in Case of Intended Turn with Yaw Damper

% Washout filter
tauWashoutFilter = 4;
washoutFilter = generate_washout_filter(tauWashoutFilter);

% Augment the aircraft state space model with washout filter
sysLatAug = connect(sysLat, washoutFilter, ...
    sysLat.InputName, [sysLat.OutputName; {'washed out yaw rate'}]);
sysLatAug.OutputUnit{end} = 'rad/s';

% Feedback: washed out yaw rate -> rudder
Krdr = get_feedback_gain(-sysLatAug('washed out yaw rate', 'rudder'), ...
    0.7, 'Dutch roll', 0.5, 1.5);

% Closed loop system with yaw damper and washout filter
sysLatAugYawDamper = feedback(sysLatAug, Krdr, 2, 5, +1);


% Feedback: bank angle -> aileron to stabilize spiral mode
Kphida = 0.2;  % positive feedback, from root locus of sysLatAugYawDamper

% Closed loop system with bank-angle feedback to aileron
sysLatAugClosedLoop = feedback(sysLatAugYawDamper, Kphida, 1, 4, +1);



%% 3. Verification

% Feedback: sideslipe angle -> rudder
Kbetadr = get_feedback_gain(sysLatAugClosedLoop('sideslip angle', 'rudder'), ...
    0.7, 'Dutch roll', 0.01, 1.0);  % negative feedback

% Closed loop system with all feedback controllers
sysLatAugCL = feedback(sysLatAugClosedLoop, Kbetadr, 2, 2, -1);

% Aileron and rudder deflection
Klat = [0, 0, 0, -Kphida, 0;
        -Krdr, Kbetadr, 0, 0, Krdr];

Acl = sysLatAug.A - sysLatAug.B * Klat;
Bcl = sysLatAug.B;
uDefl = ss(Acl, Bcl, -Klat, eye(2));
uDefl.StateName = sysLatAug.StateName;
uDefl.OutputName = sysLatAug.InputName;
uDefl.InputName = {'aileron cmd', 'rudder cmd'};
uDefl.InputUnit = {'rad', 'rad'};
uDefl.OutputUnit = {'rad', 'rad'};