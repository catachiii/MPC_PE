function param = compute_controller_base_parameters
    % load truck parameters
    load('system/parameters_building');
    
    % Task 1: continuous time dynamics in state space form
    Ac = [[-(building.a_F1_VC + building.a_F2_VC + building.a_Env_VC), building.a_F1_VC, building.a_F2_VC] / building.m_VC;
         [building.a_F1_VC, -(building.a_F1_VC + building.a_F2_F1), building.a_F2_F1] / building.m_F1;
         [building.a_F2_VC, building.a_F2_F1, -(building.a_F2_VC + building.a_F2_F1)] / building.m_F2];
    Bc = [[building.b_11, building.b_12, building.b_13] / building.m_VC;
         [building.b_21, building.b_22, building.b_23] / building.m_F1;
         [building.b_31, building.b_32, building.b_33] / building.m_F2];
    Bdc = diag([1 / building.m_VC, 1 / building.m_F1, 1 / building.m_F2]);
    d = [building.a_Env_VC * building.T_Env + building.d_VC; building.d_F1; building.d_F2];
    
    % Task 2: discretization
    Ts = 60;
    A = eye(size(Ac)) + Ts * Ac;
    B = Ts * Bc;
    Bd = Ts * Bdc;
    
    % Task 3: set point computation
    b_ref = [25, -42, -18.5]';
    C_ref = eye(size(A));
    T_sp = C_ref \ b_ref;
    p_sp = B \ ((eye(size(A)) - A) * T_sp - Bd * d);
    
    % Task 4: constraints for delta formulation
    Pcons = building.InputConstraints;
    Tcons = building.StateConstraints;
    Ucons = Pcons - repmat(p_sp, 1, size(Pcons, 2));
    Xcons = Tcons - repmat(T_sp, 1, size(Tcons, 2));
    
    % put everything together
    param.A = A;
    param.B = B;
    param.Bd = Bd;
    param.d = d;
    param.b_ref = b_ref;
    param.C_ref = C_ref;
    param.T_sp = T_sp;
    param.p_sp = p_sp;
    param.Ucons = Ucons;
    param.Xcons = Xcons;
    param.Tcons = Tcons;
    param.Pcons = Pcons;
end
