%% Init
clear all
close all
addpath(genpath(cd));
rng(1234)
load('system/parameters_scenarios.mat')
T_sp = [25, -42, -18.5]';
% dT0_exmaple = ...
% T0_example = ...


%% Example
% figure; set(gcf, 'WindowStyle' ,'docked');
% clear persisten variables of function controller_example
% clear controller_example
% execute simulation
% [T,~,~,t] = simulate_building(T0_example, @controller_example);


%% Unconstrained optimal control
disp('Unconstraint optimal control');

% Uncontrolled system
% figure(1); set(gcf, 'WindowStyle' ,'docked');
%...

% Tuning of LQR on first initial condition
T0_1 = T_sp + [-2.25, 1.75, 0.75]';
T0_2 = T_sp + [1.5, 2.75, -0.25]';
T0 = T0_1;
[Q, R] = heuristic_LQR_tuning(2500, T0, T_sp, scen1);
clear controller_lqr
figure()
simulate_building(T0, @controller_lqr, Q, R, scen1);

% Inspect influence of Q and R
influence_inspection = false;

if influence_inspection
    figure()
    scale = [0.1, 1, 10];
    for i = 1:length(scale)
        clear controller_lqr
        simulate_building(T0, @controller_lqr, i * Q, R, scen1);
    end
    figure()
    for i = 1:length(scale)
        clear controller_lqr
        simulate_building(T0, @controller_lqr, Q, i * R, scen1);
    end
end

pause;

%% LQR feasible set


% [A_x, b_x] = compute_X_LQR(Q, R);
% S = Polyhedron('A', A_x, 'b', b_x);
% figure()
% hold on
% x0_1 = [-2.25, 1.75, 0.75];
% x0_2 = [1.5, 2.75, -0.25];
% x0 = [x0_1; x0_2];
% scatter3(x0(:,1),x0(:,2),x0(:,3), 'filled');
% text(x0(:,1),x0(:,2),x0(:,3), ["x0_1", "x0_2"]);
% S.plot('color', 'lightgreen', 'Alpha', 0.1);
% 
% pause;


%% LQR optimal cost


% [A_x, b_x] = compute_X_LQR(Q, R);
% S = Polyhedron('A', A_x, 'b', b_x);
% figure()
% S.plot('color', 'lightgreen', 'Alpha', 0.1);
% hold on
% 
% x_VC = -3:3;
% x_F1 = -3:3;
% x_F2 = -0.5:0.5:1.5;
% [X_VC, X_F1, X_F2] = meshgrid(x_VC, x_F1, x_F2);
% X_VC = reshape(X_VC, [numel(X_VC), 1]);
% X_F1 = reshape(X_F1, [numel(X_F1), 1]);
% X_F2 = reshape(X_F2, [numel(X_F2), 1]);
% J = [];
% X_VC_selected = [];
% X_F1_selected = [];
% X_F2_selected = [];
% for i = 1 : length(X_VC)
%     if S.isInside([X_VC(i); X_F1(i); X_F2(i)])
%         clear controller_lqr
%         [~, ~, J_opt] = simulate_building([X_VC(i); X_F1(i); X_F2(i)] + T_sp, @controller_lqr, Q, R, scen1, 0);
%         J = [J; sum(J_opt)];
%         X_VC_selected = [X_VC_selected; X_VC(i)];
%         X_F1_selected = [X_F1_selected; X_F1(i)];
%         X_F2_selected = [X_F2_selected; X_F2(i)];
%     end
% end
% scatter3(X_VC_selected, X_F1_selected, X_F2_selected, [], J, 'filled');
% xlabel('x_VC')
% ylabel('x_F1')
% zlabel('x_F2')
% cb = colorbar;
% cb.Label.String = 'Infinite horizon cost under the LQR control law';
% 
% pause;


%% From LQR to MPC
disp('First MPC'); 

T0 = T0_1;
clear controller_mpc_1
[~, ~, J_mpc_1] = simulate_building(T0, @controller_mpc_1, Q, R, scen1);
fprintf('The optimization cost for MPC 1 is %f \n',sum(J_mpc_1));
clear controller_mpc_2
[~, ~, J_mpc_2] = simulate_building(T0, @controller_mpc_2, Q, R, scen1);
fprintf('The optimization cost for MPC 2 is %f \n',sum(J_mpc_2));
clear controller_mpc_3
[~, ~, J_mpc_3] = simulate_building(T0, @controller_mpc_3, Q, R, scen1);
fprintf('The optimization cost for MPC 3 is %f \n',sum(J_mpc_3));

pause;


%% MPC with guarantees
disp('MPC with guarantees');


pause;


%% Soft-constrained MPC
disp('Soft-constrained MPC');

pause;


%% Offset-free MPC
disp('Offset-free MPC');

pause;


%% Comparison using forces
disp('MPC Implementation with FORCES Pro');
