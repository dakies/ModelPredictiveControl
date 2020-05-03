% Init
clear all
close all
addpath(genpath(cd));
load('system/parameters_scenarios.mat');

%% E.g. execute simulation with LQR
% clear persisten variables of function controller_lqr
clear controller_lqr; 

dev = [3; 1; 0];
T0_1 = dev + [-21; 0.3; 7.32];
% execute simulation starting from T0_1 using lqr controller with scenario 1
[T, p] = simulate_truck(T0_1, @controller_lqr, scen1);
% Controller works with constraints. Norm constraint from question checked
% t_30 = [-20.63; 0.5873; 7.477]
% norm(T_sp-t_30)<0.2*norm([3;1;0])