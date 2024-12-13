%% FLIGHT CONTROL - TU Berlin
% Winter Semester 2024/2025
% Homework 1 - Task 2
% Gruppe 3

clc
clearvars
close all

% Define aircraft state space matrices
statename = {'q','\alpha','V','\theta'};
A_LON = [
    -0.9981, -2.5072, -3.518e-4, 0;
    0.9709, -0.9632, -0.0025, 0.0099;
    -0.1274, 4.6526, -0.0219, -9.7234;
    1, 0, 0, 0
];
inputname = {'\delta_t','\delta_e'};
B_LON = [
    0.1335, -5.6897;
    -0.0048, -0.1038;
    2.9885, -0.6188;
    0, 0.0518
];
outputname = {'q','\alpha','V','\theta'};
C_LON = eye(4);
D_LON = zeros(4,2);

%% Task 2.2 : Controller Design I

% State space for short-period mode approximation
A_SP = A_LON(1:2,1:2);
B_SP = B_LON(1:2,2);
C_SP = eye(2);
D_SP = zeros(2,1);
sys_SP = ss(A_SP,B_SP,C_SP,D_SP,'StateName',statename(1:2),'OutputName',outputname(1:2),'InputName',inputname(2));
% Transfer function of input 'delta_e' on output 'q'
TF_SP = tf(sys_SP(1,1));
% After calculation with hand of k_q
% Order of coefficient of polynomial [k_q^2 k_q 1]
% Getting the roots (eigenvalues) of the polynomial
k_q = roots([(5.69)^2, (4*0.49*5.22 - 2*1.961*5.69), ((1.961)^2-4*0.49*3.396)]);
% Check which k_q makes sense
for i = 1:length(k_q)
    D = (1.961-5.69*k_q(i))/(2*sqrt(3.396-5.22*k_q(i)));
end
% The first k_q resulted in negative damping, so we take the second k_q

%% Task 2.3 : Controller Design II

% Perform full state feeback so that the damping ratio is 0.7 and the
% undamped natural frequency is 0.4775 Hz

% Steps
% 1. Check for possibility for pole placement with controllability matrix
% Q_c
Q_c = [
    B_SP, A_SP*B_SP
];
rank(Q_c);

% 2. Build T with element of last row of inverse of Q_c
Q_c_inv = inv(Q_c);
T_inv = [
    Q_c_inv(end,:);...
    Q_c_inv(end,:)*A_SP
];
T = inv(T_inv);

% 3. Rewrite in CCT form : build A_T
A_T = T_inv*A_SP*T;

% 4. Get desired poles, in this case D = 0.7 and natural freq. = 0.4775 Hz
omega_n = 2*pi*0.4775;
D = 0.7;
% Coefficient of the char. polynom is the last row of A_T with added minus
char_coef = -A_T(end,:);
% The desired pole location (eigenvalues) on the root locus
sigma = -D*omega_n;
omega = omega_n*sqrt(1-D^2);
poles = [
    sigma+omega*1i, sigma-omega*1i
];
% Coefficient of polynomial of A_T
a = char_coef;
% Create polynomial of desired poles
% Flip because 'a' is ordered from smallest to highest exponent
a_d = flip(poly(poles));
% Compare the coefficient of both polynomial from A_T and desired poles
k_T = (a_d(1:2) - a);

% 5. Transform back the gain from CCT form to normal form
k = k_T*T_inv;

% 6. Simulate to check the result
A_SP_MOD = A_SP-B_SP*k;
damp(A_SP_MOD);

%% Task 2.3 : Verification I
% Step on elevator 'delta_e' with full state space
% With and without controller
% Pointing to 'delta_e'
input = 2;
% State spaces
% sys1 : without controller (original state space)
sys_based = ss(A_LON,B_LON,C_LON,D_LON,'StateName',statename,'OutputName',outputname,'InputName',inputname);
sys1 = sys_based;
% sys2 : with Controller Design I
s = tf('s');
sys_q_damper = tf(k_q(2));
sys_q_damper.InputName = {'q'};
sys_q_damper.OutputName = {'\delta_{e,qd}'};
delta_e_smblk = sumblk('\delta_e = \delta_{e,cmd} - \delta_{e,qd}');
sys2 = connect(sys_based,sys_q_damper,delta_e_smblk,{'\delta_t','\delta_{e,cmd}'},{'q','\alpha','V','\theta','\delta_t','\delta_e'});
% sys3 : with Controller Design II
sys_alpha_damper = tf(k(2));
sys_alpha_damper.InputName = {'\alpha'};
sys_alpha_damper.OutputName = {'\delta_{e,alphad}'};
delta_e_smblk = sumblk('\delta_e = \delta_{e,cmd} - \delta_{e,qd} - \delta_{e,alphad}');
sys3 = connect(sys_based,sys_q_damper,sys_alpha_damper,delta_e_smblk,{'\delta_t','\delta_{e,cmd}'},{'q','\alpha','V','\theta','\delta_t','\delta_e'});
% Plot : Short-term dynamics
t_short = 5;
figure()
step(sys1(:,input),t_short)
hold on
step(sys2(:,input),t_short)
step(sys3(:,input),t_short)
title('Short-term response on elevator step')
legend('undamped','q Feedback','q+{\alpha} Feedback','Location','bestoutside')
% Plot : Long-term dynamics
t_long = 100;
figure()
step(sys1(:,input),t_long)
hold on
step(sys2(:,input),t_long)
step(sys3(:,input),t_long)
title('Long-term response on elevator step')
legend('undamped','q Feedback','q+{\alpha} Feedback','Location','bestoutside')

%% Task 2.5 : Controller Design III
% Determine gain for phugoid mode by feeding back V to delta_t to achieve damping of 0.707
% Modified state space with full state feedback on SP mode
A_LON_NEW = [
    A_SP_MOD(1,1), A_SP_MOD(1,2), A_LON(1,3:4);
    A_SP_MOD(2,1), A_SP_MOD(2,2), A_LON(2,3:4);
    A_LON(3:4,:)
];
sys_LON_MOD = ss(A_LON_NEW,B_LON,C_LON,D_LON,'StateName',statename,'OutputName',outputname,'InputName',inputname);
% Determine gain manually from root locus to have a damping of 0.707
figure()
rlocus(sys_LON_MOD(3,1),(-2:0.00005:2))
grid on
axis equal
% Result -> gain of 0.0393

%% Task 2.6 : Verification II
% Steps in 'delta_e' on long term dynamics with phugoid damper
sys_V_damper = tf(0.0393);
sys_V_damper.InputName = {'V'};
sys_V_damper.OutputName = {'\delta_{t,Vd}'};
delta_e_smblk = sumblk('\delta_t = \delta_{t,cmd} - \delta_{t,Vd}');
sys5 = connect(sys_LON_MOD,sys_V_damper,delta_e_smblk,{'\delta_{t,cmd}','\delta_e'},{'q','\alpha','V','\theta','\delta_t','\delta_e'});
t_long = 100;
figure
step(sys_LON_MOD(:,input),t_long)
hold on
step(sys5(:,input),t_long)
legend('V undamped (modified SS)','V Feedback (modified SS)','Location','bestoutside')
