% Init
clear all
close all
addpath(genpath(cd));
load('system/parameters_scenarios.mat');
%% Todo
% Calculate If
% Add final cost for all MPC
%% E.g. execute simulation with LQR
% clear persisten variables of function controller_lqr
clear controller_lqr; 

T0_1 = [3; 1; 0] + [-21; 0.3; 7.32];
T0_2 = [-1; -0.3; -4.5] + [-21; 0.3; 7.32];
T0_3 = [12; 12; 12];

%T5
% % execute simulation starting from T0_1 using lqr controller with scenario 1
[T, p] = simulate_truck(T0_1, @controller_lqr, scen1);
 sgtitle('simulate truck(T0 1, @controller lqr, scen1)')
% % Controller works with constraints. Norm constraint from question checked
param = compute_controller_base_parameters;
assert(norm(param.T_sp-T(:, 31)) < 0.2*norm([3;1;0]))
figure

% %T7
% % % % execute simulation starting from T0_1 using lqr controller with scenario 1
[T, p] = simulate_truck(T0_2, @controller_lqr, scen1);
sgtitle('simulate truck(T0 2, @controller lqr, scen1)')
figure

% %T9
[T, p] = simulate_truck(T0_1, @controller_mpc_1, scen1);
sgtitle('simulate truck(T0 1, @controller mpc 1, scen1)')
figure

[T, p] = simulate_truck(T0_2, @controller_mpc_1, scen1);
sgtitle('simulate truck(T0 2, @controller mpc 1, scen1)')
figure

[T, p] = simulate_truck(T0_1, @controller_mpc_1_forces, scen1);
sgtitle('simulate truck(T0 2, @controller mpc 1 FORCES, scen1)')
figure


% T11
[T, p] = simulate_truck(T0_1, @controller_mpc_2, scen1);
sgtitle('simulate truck(T0 2, @controller mpc 2, scen1)')
figure

% % T15
% Todo: Choose infinite horizon cost so that the origin is an asymptotically
% stable equilibrium point for the resulting closed-loop system ???
simulate_truck(T0_1, @controller_mpc_3, scen1);
sgtitle('simulate truck(T0 1, @controller mpc 3, scen1)');
figure

simulate_truck(T0_2, @controller_mpc_3, scen1);
sgtitle('simulate truck(T0 2, @controller mpc 3, scen1)');
figure


% T17
simulate_truck(T0_3, @controller_mpc_3, scen1);
sgtitle('simulate truck(T0 3, @controller mpc 3, scen1)');
figure

% T18
simulate_truck(T0_3, @controller_mpc_4, scen1);
sgtitle('simulate truck(T0 3, @controller mpc 4, scen1)');
figure


% T19
simulate_truck(T0_2, @controller_mpc_4, scen1);
sgtitle('simulate truck(T0 2, @controller mpc 4, scen1)');
figure


% % T22
simulate_truck(T0_1, @controller_mpc_5, scen2);
sgtitle('simulate truck(T0 1, @controller mpc 5, scen2)');