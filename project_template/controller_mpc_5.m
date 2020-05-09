% BRIEF:
%   Controller function template. This function can be freely modified but
%   input and output dimension MUST NOT be changed.
% INPUT:
%   T: Measured system temperatures, dimension (3,1)
% OUTPUT:
%   p: Cooling power, dimension (2,1)
function p = controller_mpc_5(T)
% controller variables
persistent param yalmip_optimizer d_hat x_hat

% initialize controller, if not done already
if isempty(param)
    [param, yalmip_optimizer] = init();
    
    %Initial state estimate
    x_hat = T;
    d_hat = param.d;
end

%% SP point computation
%Has to be recalculted due to varying disturbance
%Lec7 Slide30
tempor = [param.A-eye(size(param.A)) param.B;...
param.C_ref zeros(2,2)];

%Check for full Rank
assert(det(tempor) ~= 0);

xu = tempor\[-param.B_d*d_hat; param.b_ref]; 
T_sp = xu(1:3);
p_sp = xu(4:5);
x_aug = [x_hat; d_hat];

%% evaluate control action by solving MPC problem, e.g.
[u_mpc,errorcode] = yalmip_optimizer(x_aug, T_sp, p_sp);
if (errorcode ~= 0)
      warning('MPC infeasible');
end

p = u_mpc{1};

%% Estimator
est = param.A_aug*x_aug+param.B_aug*p+param.L*(T-param.C_aug*x_aug);
x_hat = est(1:3);
d_hat = est(4:6);
end

function [param, yalmip_optimizer] = init()
% initializes the controller on first call and returns parameters and
% Yalmip optimizer object

param = compute_controller_base_parameters; % get basic controller parameters

%% implement your MPC using Yalmip here, e.g.
%MPC data
N = 30;
Q = param.Q;
R = param.R;
[Ax,bx] = compute_X_LQR;

%Model data
A = param.A_aug;
B = param.B_aug;

%Number of states
nx = size(param.A_aug,1);
nu = size(param.B_aug,2);

%State -> constraints
Gu = [1 0; -1 0; 1 0; -1 0];
Gx = [1 0 0; 0 1 0; 0 -1 0];

%x:=delta_x, u:=delta_u
u = sdpvar(repmat(nu, 1, N-1), ones(1, N-1), 'full');
x = sdpvar(repmat(nx, 1, N), ones(1, N), 'full'); %x=[x;d_hat]

%Steady states. Recalculation required for variang disturbance estimate
x_s = sdpvar(nx/2,1,'full');
u_s = sdpvar(nu,1,'full');

%Init
objective = 0;
constraints =[];
for k = 1:N-1
  constraints = [constraints,  Gu*u{k} <= param.Ucons];
  constraints = [constraints,  Gx*x{k}(1:3) <= param.Xcons];
  constraints = [constraints, x{k+1} == A*x{k} + B*u{k}];
  objective = objective + (x{k}(1:3)-x_s)'*Q*(x{k}(1:3)-x_s) + (u{k}-u_s)'*R*(u{k}-u_s);
end

%Timestep N
objective = objective + (x{N}(1:3)-x_s)'*Q*(x{N}(1:3)-x_s);
constraints = [constraints, Gx*x{N}(1:3) <= param.Xcons];
constraints = [constraints, Ax*x{N}(1:3) <= bx];

ops = sdpsettings('verbose', 0, 'solver', 'quadprog');
fprintf('JMPC_dummy = %f', value(objective));
yalmip_optimizer = optimizer(constraints, objective, ops, {x{1,1}; x_s; u_s}, {u{1,1},objective});
end