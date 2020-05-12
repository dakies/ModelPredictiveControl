function param = compute_controller_base_parameters
    %% load truck parameters
    load('system/parameters_truck');
    
    A_cont = [(-truck.a12-truck.a1o)/truck.m1 truck.a12/truck.m1 0;...
        truck.a12/truck.m2 (-truck.a12-truck.a23-truck.a2o)/truck.m2 truck.a23/truck.m2;...
        0 truck.a23/truck.m3 (-truck.a23-truck.a3o)/truck.m3];
    
    B_cont = [1/truck.m1 0;...
        0 1/truck.m2;...
        0 0]; %Input Matrix: Hier durch m dividiert D.K.
    
    d = truck.w + [truck.a1o*truck.To; truck.a2o*truck.To; truck.a3o*truck.To];
    B_d_cont = diag([1/truck.m1 1/truck.m2 1/truck.m3]); %Disturbance, mit Ts discretiziert --> keine Umrechnung n�tig, jedoch ist Jon sich nicht sicher
    
    
    %% (2) discretization
    Ts = 60;
    %Exact discretization
    A = expm(A_cont*Ts);
    B = A_cont\(A-eye(size(A_cont)))*B_cont;
    B_d = A_cont\(A-eye(size(A_cont)))*B_d_cont;
    
    %Check for same results
%     cont = ss(A_cont,B_cont,truck.C_ref,0);
%     disc = c2d(cont,Ts);
%     A2 = disc.A;
%     B2 = disc.B;
 
    
    %Lec7 Slide30
    tempor = [A-eye(size(A)) B;...
        truck.C_ref zeros(2,2)];
    
    %Check for full Rank
    assert(det(tempor) ~= 0);
    
    xu = tempor\[-B_d*d; truck.b_ref];
    
    %% (3) set point computation
    T_sp = xu(1:3);
    p_sp = xu(4:5);
    
    %% (4) system constraints
    Pcons = truck.InputConstraints;
    Tcons = truck.StateConstraints;
    
    %% (4) constraints for delta formulation
    Gu = [1 0; -1 0; 0 1; 0 -1];
    Ucons = [Pcons(1,2); -Pcons(1,1); Pcons(2,2); -Pcons(2,1)] - Gu * p_sp;
    
    Gx = [1 0 0; 0 1 0; 0 -1 0];
    Xcons = [Tcons(1,2); Tcons(2,2); -Tcons(2,1);] - Gx * T_sp;
    
    %% (5) LQR cost function
%     Q = diag([400; 400; 0]);
%     R = diag([0.007, 0.007]);
Q = diag([1; 1.5; 1])*10000;
    R = diag([0.4, 0.022]);
    
    [k_lqr, P, ~] = dlqr(A, B, Q, R);
    param.k_lqr = k_lqr;
    param.P = P;
    %% (6) Augumented System
    param.A_aug = [A B_d;...
        zeros(size(A)) eye(size(A))];
    param.B_aug = [B; zeros(size(A,1), size(B,2))];
    param.C_aug = [eye(3) zeros(3,3)]; %Y=[x1; x2; x3]
% %     param.L = [eye(3); -0.1*eye(3)];
    param.L = place(param.A_aug',param.C_aug',[0.03 -0.03 0.02  -0.02 0.01 -0.01])';
    param.d = d;
    param.C_ref = truck.C_ref;
    param.B_d = B_d;
    param.b_ref = truck.b_ref;
%     %Require stable error dynamics. Lec 7 Slide 28. Achieved through
%     %place()
    
    % For task 22, augmentation for disturbance estimation
%     param.A_aug = [A, B_d;zeros(3,3),eye(3)];
%     param.B_aug = [B;zeros(3,2)];
%     param.C_aug = [eye(3),zeros(3,3)];
%     param.L = place(param.A_aug',param.C_aug',[0.03 -0.03 0.02  -0.02 0.01 -0.01])';
    % Initialization of disturbance
%     param.dc = d;
    
    ev = eig(param.A_aug - param.L * param.C_aug); %Das sollte iegentlich ev = eig(param.A_aug + param.L * param.C_aug); sein!!!
%     for i = 1:length(ev)
%         assert(ev(i) <= 1)
%     end
    
    %% put everything together
    param.A = A;
    param.B = B;
    param.Q = Q;
    param.R = R;
    param.T_sp = T_sp;
    param.p_sp = p_sp;
    param.Ucons = Ucons;
    param.Xcons = Xcons;
    param.Tcons = Tcons;
    param.Pcons = Pcons;
end

