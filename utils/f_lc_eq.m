% function to compute Lc distance at initial pose
function f = f_lc_eq(z, params)
% z = [kc_rear_h; kc_rear_v; lc_front; lc_rear]
% extract parameters variables
n = params.fixed.n;
kc_front = params.fixed.kc_front;
r_anchor = params.fixed.r_anchor;
ke = params.fixed.ke;
le_0 = params.fixed.le_0;
l_tube = params.fixed.l_tube;
l_rod = params.fixed.l_rod;
phi = params.fixed.phi;
r_tube = params.fixed.r_tube;

% extract optimization variables
kc_rear_h = z(1);
kc_rear_v = z(2);
s_eq = z(3:end); % should be 1*2 given same rear compression distance

% define cost
f = 0;

% define initial tube axis 
u_hat0 = [1 0 0]';

% generate initial r_attach_front and _rear points in N frame, given same
% front and rear compression distance
theta = deg2rad(90 * ones(1, n));
r_attach_front = generate_radial_points(n, theta, phi, r_tube * ones(1, n), 2*l_rod);
r_attach_front(:, n/2+1:end) = [];
r_attach_rear = generate_radial_points(n, theta, phi, r_tube * ones(1, n), 1);
r_attach_rear = r_attach_rear(:, n/2+1:end);

% add the effect of lc in x axis 
r_attach_rear(1, :) = -0.5*(l_tube - s_eq(1) - s_eq(2));  % in N frame
r_attach_front(1, :) = 0.5*(l_tube - s_eq(1) - s_eq(2));

% compute horizontal Fs_front sum (odd number)
Fs_front_h = zeros(3,1);
for i = 1:2: n/2
    r  = r_anchor(:, i) - r_attach_front(:, i);   % 3x1
    lei = norm(r);
    u = r / lei;
    Fs_front_h = Fs_front_h + ke(i) * ((lei - le_0(i)) * u);
end

% compute vertical Fs_front sum (even number)
Fs_front_v = zeros(3,1);
for i = 2:2: n/2
    r  = r_anchor(:, i) - r_attach_front(:, i);   % 3x1
    lei = norm(r);
    u = r / lei;
    Fs_front_v = Fs_front_v + ke(i) * ((lei - le_0(i)) * u);
end
Fs_front = Fs_front_v + Fs_front_h;

% compute horizontal Fs_rear (odd number)
Fs_rear_h = zeros(3,1);
for i = (n/2+1) : 2 : n
    r  = r_anchor(:, i) - r_attach_rear(:, i - n/2);   % 3x1
    lei = norm(r);
    u = r / lei;
    Fs_rear_h = Fs_rear_h + ke(i) * ((lei - le_0(i)) * u);
end
% compute vertical Fs_rear (even number)
Fs_rear_v = zeros(3,1);
for i = (n/2+2) : 2: n
    r  = r_anchor(:, i) - r_attach_rear(:, i - n/2);   % 3x1
    lei = norm(r);
    u = r / lei;
    Fs_rear_v = Fs_rear_v + ke(i) * ((lei - le_0(i)) * u);
end
Fs_rear = Fs_rear_v + Fs_rear_h;

% front and rear extension equileibrium penalty 
f = f + (u_hat0.' * Fs_front_h + u_hat0.' * Fs_rear_h)^2;  % horizontal
f = f + (u_hat0.' * Fs_front_v + u_hat0.' * Fs_rear_v)^2;  % vertical

% rear compression and extension equileibrium penalty 
f = f + (2*kc_rear_v * s_eq(2) - u_hat0.' * Fs_rear_v)^2;
f = f + (2*kc_rear_h * s_eq(2) - u_hat0.' * Fs_rear_h)^2;

% front compression and extension equileibrium penalty 
f = f + (kc_front * s_eq(1) + u_hat0.' * Fs_front)^2;

% front and rear compression equileibrium penalty 
f = f + (2*(kc_rear_h + kc_rear_v) * s_eq(2) - u_hat0.' * Fs_rear)^2;

% front and rear compression length penalty 
f = f + 1e6 * (s_eq(1) - s_eq(2))^2;
end

%[appendix]{"version":"1.0"}
%---
