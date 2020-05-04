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
    
    %Problem: Konvergiert immernoch nicht
    %Ideen: Teilweise Werte für delta_x und teilweise für x
    
    % Define LQR Controller
    k_lqr = -dlqr(param.A, param.B, param.Q, param.R);
    systemLQR = LTISystem('A', param.A+param.B*k_lqr);
    
    % Define Constraints
%     system.x.min = [-Inf; param.Xcons(3); -Inf];
%     system.x.max = [param.Xcons(1); param.Xcons(2); Inf];
%     system.u.min = [param.Ucons(2); param.Ucons(4)];
%     system.u.max = [param.Ucons(1); param.Ucons(3)];
    
    %% Express constraints in polytopic state constraints
    %Reference recitation 5 code and solution recitation 6
    test = [1 0 0; 0 1 0; 0 -1 0];
    test1 = [-k_lqr; k_lqr];
    A = [test; test1];
    b = [param.Xcons(1,1); param.Xcons(2,1); param.Xcons(3,1); param.Ucons(1,1); param.Ucons(2,1); param.Ucons(3,1); param.Ucons(4,1)];
    Xp = Polyhedron('A', [test; test1], 'b', [param.Xcons(1,1); param.Xcons(2,1); param.Xcons(3,1); param.Ucons(1,1); param.Ucons(2,1); param.Ucons(3,1); param.Ucons(4,1)]);
    Xp.plot(), alpha(0.25)
    figure(1)
    systemLQR.x.with('setConstraint');
    systemLQR.x.setConstraint = Xp;

    %% Compute resulting control invariant set
    InvSetLQR = systemLQR.invariantSet();
    
    %% Plot
    figure (2)
    hold on
    InvSetLQR.plot(), alpha(0.25)
    
    A_x = Set.A;
    b_x = Set.b;
        
% %     Gx = [1 0 0; 0 1 0; 0 -1 0];
% %     Gu = [1 0; -1 0; 1 0; -1 0];
% %     poly = Polyhedron('A', [eye(3);-eye(3); -k_lqr; k_lqr], 'b', [param.Tcons(:,2);-param.Tcons(:,1);param.Pcons(:,2);-param.Pcons(:,1)]);
% %     system.x.with('setConstraint');
% %     system.x.setConstraint = poly;
%     
% %     %Reference
% %     system.x.with('reference')
% %     system.u.with('reference')
% %     system.x.reference = param.T_sp;
% %     system.u.reference = param.p_sp;
% %     
%     %LQR
%     Q = param.Q;
%     system.x.penalty = QuadFunction(Q);
%     R = param.R;
%     system.u.penalty = QuadFunction(R);
% %     
% %      
% %     system.x.min = [param.Tcons(1, 1); param.Tcons(2, 1); param.Tcons(3, 1)];
% %     system.x.max = [param.Tcons(1, 2); param.Tcons(2, 2); param.Tcons(3, 2)];
% %     system.u.min = [param.Pcons(1, 1); param.Pcons(1, 1)];
% %     system.u.max = [param.Pcons(1, 2); param.Pcons(1, 2)];
% % LQR controller lauft auf x und U.cons --> �bernehmen
% %     
% % %     Sanity checks
% %     display(system.LQRGain())
%     
%     system.x.min = [-Inf; param.Xcons(3); -Inf];
%     system.x.max = [param.Xcons(1); param.Xcons(2); Inf];
% %     system.u.min = [param.Ucons(2); param.Ucons(4)];
% %     system.u.max = [param.Ucons(1); param.Ucons(3)];
% 
% %     Set = system.invariantSet('maxIterations', 100)
% %     Set.plot()
% 
%     Set = system.LQRSet();
%     Set.plot()
end

