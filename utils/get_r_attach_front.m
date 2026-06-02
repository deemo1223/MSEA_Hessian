function [r_attach_front_B, r_attach_front] = get_r_attach_front(params, T_NB)

% extract variables
n = params.fixed.n;
phi = params.fixed.phi;
r_tube = params.fixed.r_tube;
l_rod = params.fixed.l_rod;

theta = deg2rad(90 * ones(1, n));

r_attach_front_B_all = generate_radial_points(n, theta, phi, r_tube * ones(1, n), 2*l_rod);
r_attach_front_B = r_attach_front_B_all(:, 1:n/2);
r_attach_front_B(1, :) = -r_attach_front_B(1, :);

% if specified T_NB, transform each point
if nargin < 2 || isempty(T_NB)
    r_attach_front = [];
else
    r_attach_front = zeros(3, n/2);
    for i = 1:n/2
        temp = T_NB * [r_attach_front_B(:, i); 1];
        r_attach_front(:, i) = temp(1:3);
    end
end
end

%[appendix]{"version":"1.0"}
%---
