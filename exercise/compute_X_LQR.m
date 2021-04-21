% BRRIEF:
%   Template for explicit invariant set computation. You MUST NOT change
%   the output.
% INPUT:
%   Q, R: State and input weighting matrix, dimension (3,3)
% OUTPUT:
%   A_x, b_x: Describes polytopic X_LQR = {x| A_x * x <= b_x}

function [A_x, b_x] = compute_X_LQR(Q, R)
    % get basic controller parameters
    param = compute_controller_base_parameters;
    system = LTISystem('A', param.A, 'B', param.B);
    system.x.min = param.Xcons(:, 1);
    system.x.max = param.Xcons(:, 2);
    system.u.min = param.Ucons(:, 1);
    system.u.max = param.Ucons(:, 2);
    system.x.penalty = QuadFunction(Q);
    system.u.penalty = QuadFunction(R);
    S = system.LQRSet();
    % implement the X_LQR computation and assign the result
    A_x = S.A;
    b_x = S.b;
end