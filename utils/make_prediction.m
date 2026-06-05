% function to make prediction using current measured string length, IMU
% angle, based on specified oeprating point index

function [W_star, p_star] = make_prediction(ref, l_str, theta)

% calculate l_str and theta delta
l_str_delta = l_str - ref.l_str;
theta_delta = theta - ref.q(4:6);

% extract operating point Hessian info
W_oper = ref.W_out;
As = ref.As;
Keff = ref.Keff;
q_oper = ref.q;

% define Cs and Jstr matrix
Cs = zeros(4, 5);
Cs(:, 2:end) = -eye(4);
Cs(:, 1) = -1;
Jstr = Cs * As;
Jstr_p = Jstr(:, 1:3);
Jstr_theta = Jstr(:, 4:6);

% compute augumented least square to find p_delta
lambda = 1e-3;
[~, m] = size(Jstr_p);
A = Jstr_p;
A_aug = [A; lambda * eye(m)];
b = l_str_delta - Jstr_theta * theta_delta;
b_aug = [b; zeros(m, 1)];
p_delta_star = A_aug \ b_aug;  % LS

% form p_star and q_delta_star
p_star = q_oper(1:3) + p_delta_star;
q_delta_star = [p_delta_star; theta_delta];

%compute output wrench 
W_delta_star = -Keff*q_delta_star;
W_star = W_oper + W_delta_star;

end