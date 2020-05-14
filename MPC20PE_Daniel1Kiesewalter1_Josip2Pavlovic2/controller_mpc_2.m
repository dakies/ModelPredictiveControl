% BRIEF:
%   Controller function template. This function can be freely modified but
%   input and output dimension MUST NOT be changed.
% INPUT:
%   T: Measured system temperatures, dimension (3,1)
% OUTPUT:
%   p: Cooling power, dimension (2,1)
function p = controller_mpc_2(T)
% controller variables
persistent param yalmip_optimizer

% initialize controller, if not done already
if isempty(param)
    [param, yalmip_optimizer] = init();
end

%% evaluate control action by solving MPC problem, e.g.
Tin = T -param.T_sp;
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

%Model data
A = param.A;
B = param.B;

%Number of states/inputs
nx = size(param.A,1);
nu = size(param.B,2);

%State -> constraints
Gu = [1 0; -1 0; 1 0; -1 0];
Gx = [1 0 0; 0 1 0; 0 -1 0];

%x:=delta_x, u:=delta_u
u = sdpvar(repmat(nu,1,N-1), ones(1,N-1), 'full');
x = sdpvar(repmat(nx,1,N), ones(1,N), 'full');

objective = 0;
constraints =[];
for k = 1:N-1
  constraints = [constraints,  Gu*u{k} <= param.Ucons];
  constraints = [constraints,  Gx*x{k} <= param.Xcons];
  constraints = [constraints, x{k+1} == A*x{k}+B*u{k}];
  objective = objective + x{k}'*Q*x{k} + u{k}'*R*u{k};
end

%Timestep N
%No infinite horizon cost
%objective = objective + x{k}'*Q*x{N};
constraints = [constraints, Gx*x{N} <= param.Xcons];
constraints = [constraints, x{N} == [0; 0; 0]];

ops = sdpsettings('verbose', 0, 'solver', 'quadprog');
yalmip_optimizer = optimizer(constraints, objective, ops, x{1,1}, {u{1,1},objective});
end