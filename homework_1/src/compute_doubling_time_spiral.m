function T2 = compute_doubling_time_spiral(system)
%compute_doubling_time_spiral Compute doubling time of unstable spiral mode.
%   Formula from lecture 05, page 14.
%
%   Author: H. N. Tang

g = 9.81;
Vtrim = 92.6;  % m/s

A = system.A;

Lr = A(3, 1);
Lbeta = A(3, 2);
Lp = A(3, 3);
Nr = A(1, 1);
Nbeta = A(1, 2);

lambdaS = (g / Vtrim) * ((Lbeta*Nr - Lr*Nbeta) / (Lp * Nbeta));

T2 = log(2) / lambdaS;

end