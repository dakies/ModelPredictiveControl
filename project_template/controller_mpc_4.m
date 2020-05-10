% BRIEF:
%   Controller function template. This function can be freely modified but
%   input and output dimension MUST NOT be changed.
% INPUT:
%   T: Measured system temperatures, dimension (3,1)
% OUTPUT:
%   p: Cooling power, dimension (2,1)
function p = controller_mpc_4(T)
% controller variables
persistent param yalmip_optimizer

% initialize controller, if not done already
if isempty(param)
    [param, yalmip_optimizer] = init();
end

%% evaluate control action by solving MPC problem, e.g.
Tin = T - param.T_sp;
[u_mpc,errorcode] = yalmip_optimizer(Tin);
if (errorcode ~= 0)
      warning('MPC infeasible');
end
p = u_mpc{1}+param.p_sp;
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
A = param.A;
B = param.B;

%Number of states
nx = size(param.A,1);
nu = size(param.B,2);

%Soft constraint penelization
S = 10^2;
v = 10^2;

%State -> constraints
Gu = [1 0; -1 0; 1 0; -1 0];
Gx = [1 0 0; 0 1 0; 0 -1 0];

%x:=delta_x, u:=delta_u
u = sdpvar(repmat(nu,1,N-1), ones(1,N-1), 'full');
x = sdpvar(repmat(nx,1,N), ones(1,N), 'full');
Eps = sdpvar(repmat(size(Gx, 1), 1, N),ones(1,N),'full');

%Init
objective = 0;
constraints =[];
for k = 1:N-1
  %Input
  constraints = [constraints,  Gu*u{k} <= param.Ucons];
  % State
  constraints = [constraints,  Gx*x{k} <= param.Xcons];
  %System
  constraints = [constraints, x{k+1} == A*x{k}+B*u{k}, 0 <= Eps{k}];
  %Cost
  objective = objective + x{k}'*Q*x{k} + u{k}'*R*u{k} + Eps{k}'*S*Eps{k} + v*norm(Eps{k},1);
end

%Timestep N
objective = objective + x{N}'*Q*x{N} + Eps{N}'*S*Eps{N} + v*norm(Eps{N},1);
constraints = [constraints, Gx*x{N} <= param.Xcons + Eps{N}, 0 <= Eps{N}];
constraints = [constraints, Ax*x{N} <= bx];

ops = sdpsettings('verbose', 0, 'solver', 'quadprog');
fprintf('JMPC_dummy = %f', value(objective));
yalmip_optimizer = optimizer(constraints, objective, ops, x{1,1}, {u{1,1},objective});
end