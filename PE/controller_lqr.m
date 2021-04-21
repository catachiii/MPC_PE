% BRIEF:
%   Controller function template. Input and output dimension MUST NOT be
%   modified.
% INPUT:
%   Q: State weighting matrix, dimension (3,3)
%   R: Input weighting matrix, dimension (3,3)
%   T: Measured system temperatures, dimension (3,1)
% OUTPUT:
%   p: Heating and cooling power, dimension (3,1)

function p = controller_lqr(Q, R, T, ~, ~)
% controller variables
persistent param;

% initialize controller, if not done already
if isempty(param)
    param = init();
end

% compute control action
[Klqr, ~, ~] = dlqr(param.A, param.B, Q, R);
p = -Klqr * (T - param.T_sp) + param.p_sp;
end

function param = init()
% get basic controller parameters
param = compute_controller_base_parameters;
% add additional parameters if necessary, e.g.
% param.F = ...
end