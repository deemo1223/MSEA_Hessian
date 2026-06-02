function [r_attach_rear_B, r_attach_rear] = get_r_attach_rear(params, Lc, T_NB)

% extract variables
n = params.fixed.n;
phi = params.fixed.phi;
r_tube = params.fixed.r_tube;
l_rod = params.fixed.l_rod;
l_tube = params.fixed.l_tube;

theta = deg2rad(90 * ones(1, n));

% base yz layout
r_attach_rear_B_all = generate_radial_points(n, theta, phi, r_tube * ones(1, n), 1);
r_attach_rear_B = r_attach_rear_B_all(:, n/2+1:end);

% overwrite x using the actual compression geometry
for i = 1:n/2
    r_attach_rear_B(1, i) = -(l_rod + l_tube - Lc(1) - Lc(i+1));
end

% transform to N frame if T_NB is given
if nargin < 3 || isempty(T_NB)
    r_attach_rear = [];
else
    r_attach_rear = zeros(3, n/2);
    for i = 1:n/2
        temp = T_NB * [r_attach_rear_B(:, i); 1];
        r_attach_rear(:, i) = temp(1:3);
    end
end
end

%[appendix]{"version":"1.0"}
%---
