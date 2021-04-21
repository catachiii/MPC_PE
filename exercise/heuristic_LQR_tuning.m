% BRRIEF:
%   Template for tuning of Q and R matrices for LQR controller as described 
%   in task 6.
% INPUT:
%   n_samples:  Number of samples considered for the tuning.
%   T0:         Initial condition
%   T_sp:       Set point
%   scen:       Disturbance scenario
% OUTPUT:
%   Q, R: Describes stage cost of LQR controller (x^T Q x + u^T R u)

function [Q, R] = heuristic_LQR_tuning(n_samples, T0, T_sp, scen)

figure(2); set(gcf, 'WindowStyle' ,'docked'); grid on; hold on
xlabel('Energy consumption [kWh]'); 
ylabel('Relative norm of steady state deviation');
R = eye(3);
flag = 0;
dT_relative_selected = 1;

for index = 1:n_samples
    Q_idx = diag([randi([1 10e6]), randi([1 10e6]), randi([1 10e6])]);
    clear controller_lqr
    [T, p, ~, ~, T_v, p_v] = simulate_building(T0, @controller_lqr, Q_idx, R, scen, 0);
    dT_relative = norm(T_sp - T(:, 15)) / norm(T_sp - T0);
    power_sum = sum(abs(p), 'all')/ 1000 / 60;
    if T_v
        RGB = 'r';
    elseif p_v
        RGB = 'b';
    else
        RGB = 'g';
    end
    scatter(power_sum, dT_relative, [], RGB);
    if RGB == 'g' && power_sum <= 16 && dT_relative < dT_relative_selected
        Q = Q_idx;
        dT_relative_selected = dT_relative;
        power_sum_selected = power_sum;
        RGB_selected = RGB;
        flag = 1;
    end
end

if flag == 1
    scatter(power_sum_selected, dT_relative_selected, [], RGB_selected, 'filled');
    text(power_sum_selected - 2e-2 * (max(xlim) - min(xlim)) , dT_relative_selected - 2e-2 * (max(ylim) - min(ylim)), "Q");
else
    warning("Expected Q not found!");
end

end
