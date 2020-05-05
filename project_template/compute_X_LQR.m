% BRRIEF:
%   Template for explicit invariant set computation. You MUST NOT change
%   the output.
% OUTPUT:
%   A_x, b_x: Describes polytopic X_LQR = {x| A_x * x <= b_x}
function [A_x, b_x] = compute_X_LQR
    % get basic controller parameters
    param = compute_controller_base_parameters;
    %% Here you need to implement the X_LQR computation and assign the result.

%     invariant_set_lqr.plot()
%     sgtitle('Set X_LQR');
    %Problem: Konvergiert immernoch nicht
    %Ideen: Teilweise Werte für delta_x und teilweise für x
    
%     system.x.min = [-Inf; param.Xcons(3); -Inf];
%     system.x.max = [param.Xcons(1); param.Xcons(2); Inf];
%     system.u.min = [param.Ucons(2); param.Ucons(4)];
%     system.u.max = [param.Ucons(1); param.Ucons(3)];
%   
% Controller
    k_lqr = -dlqr(param.A, param.B, param.Q, param.R);
   
    %System
    A = param.A;
    B = param.B;
    system = LTISystem('A', A+B*k_lqr);
    
    
    
    %Constrains1
%     Xcons = param.Tcons - [param.T_sp param.T_sp];
%     Ucons = param.Pcons - [param.p_sp param.p_sp];
%     poly = Polyhedron('A', [eye(3); -eye(3); k_lqr; -k_lqr], 'b', [Xcons(:,2); -Xcons(:,1); Ucons(:,2); -Ucons(:,1)]);

%     %Constrains2
    Gx = [1 0 0; 0 1 0; 0 -1 0];
    Gu = [1 0; -1 0; 0 1; 0 -1];
    poly = Polyhedron('A', [Gx; Gu*k_lqr], 'b', [param.Xcons;param.Ucons]);

    system.x.with('setConstraint');
    system.x.setConstraint = poly;

    figure(1)
    poly.plot()
    title("Contraints Polytope")
    
    %Calculate Invariant Set
    Set = system.invariantSet()
    figure(2)
    Set.plot()
    
    A_x = Set.A;
    b_x = Set.b;
end

