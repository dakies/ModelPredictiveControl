% BRRIEF:
%   Template for explicit invariant set computation. You MUST NOT change
%   the output.
% OUTPUT:
%   A_x, b_x: Describes polytopic X_LQR = {x| A_x * x <= b_x}
function [A_x, b_x] = compute_X_LQR
    % get basic controller parameters
    param = compute_controller_base_parameters;
    %% Here you need to implement the X_LQR computation and assign the result.
    
    % computes a control invariant set for LTI system x^+ = A*x+B*u
    k_lqr = dlqr(param.A, param.B, param.Q, param.R,0);
    A = param.A - param.B*k_lqr 
    system = LTISystem('A', A);
%     system.x.min = [-Inf; param.Xcons(3); -Inf];
%     system.x.max = [param.Xcons(1); param.Xcons(2); Inf];
%     system.u.min = [param.Ucons(2); param.Ucons(4)];
%     system.u.max = [param.Ucons(1); param.Ucons(3)];
    system.x.min = [param.Tcons(1, 1); param.Tcons(2, 1); param.Tcons(3, 1)];
    system.x.max = [param.Tcons(1, 2); param.Tcons(2, 2); param.Tcons(3, 2)];
%     system.u.min = [param.Pcons(1, 1); param.Pcons(1, 1)];
%     system.u.max = [param.Pcons(1, 2); param.Pcons(1, 2)];
    InvSet = system.invariantSet('maxIterations', 150)
    InvSet.plot()
    A_x = []
    b_x = []
end

