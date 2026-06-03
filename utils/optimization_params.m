function params = optimization_params(delta_q, configFile)


% read MSEA parameters in json file
text = fileread(configFile);
config = jsondecode(text);


% set tube delta pose
dx = delta_q(1);  
dy = delta_q(2);
dz = delta_q(3);
roll = delta_q(4);
pitch = delta_q(5);
yaw = delta_q(6);


% set the extension spring parameters from config 
params.fixed.n = 8;  % total number of extension springs
n = params.fixed.n;
ke_h = config.ke_h;  % horizontal extension spring constant
ke_v = config.ke_v;  % vertical extension spring constant
params.fixed.ke = repmat([ke_h ke_v], 1, n/2);  % extension spring constant 
params.fixed.le_0 =  config.le_0 * ones(1, n);  % extension spring rest length m
params.fixed.le_frame = config.le_frame * ones(1, n);  % length for anchor generation
e_ratio = config.e_ratio;  % extension ratio
params.fixed.le_range = zeros(2, n); % extension spring working range [min; max]
for i = 1: n
    params.fixed.le_range(:, i) = [params.fixed.le_0(i); params.fixed.le_0(i)*(1+e_ratio(i))];
end
params.slackIdx = zeros(1, n);  % slack indicator


% set the compression spring paramters from config 
params.fixed.kc_front = config.kc_front;  % front compression spring constant, scalar， 120mm long model
params.fixed.lc_0 = config.lc_0 * ones(1,5); %repmat(0.47*params.fixed.l_tube, 1, n/2 + 1);  % compression spring rest length
c_ratio = config.c_ratio;  % compression ratio [F, RH1, RV1, RH2, RV2]
params.fixed.lc_range = zeros(2, n/2 + 1);  % compression spring working range 
for i = 1:n/2+1
    params.fixed.lc_range(:, i) = [params.fixed.lc_0(i)*(1-c_ratio(i)) params.fixed.lc_0(i)];
end
%params.fixed.f_string = 2.5;  % constant force applied by the string encoder 


% set the geometry parameters from config
params.fixed.theta = deg2rad(config.theta * ones(1, n));  % spring sagital plane angle
params.fixed.l_rod = config.l_rod;
params.fixed.l_tube = config.l_middle + 2 * config.lc_0;
params.fixed.r_tube = config.r_tube;  % attach point radius to central axis
phi_half = linspace(0, 2*pi, n/2 + 1); phi_half(end) = []; params.fixed.phi = [phi_half, phi_half];  % spring frontal plane angle
params.fixed.r_anchor = generate_radial_points(n, params.fixed.theta, params.fixed.phi, params.fixed.le_frame, params.fixed.l_tube);
[params.fixed.r_attach_front_B, ~] = get_r_attach_front(params, []);


% optimization to find s_eq and required kc_rear to achieve same
% front/rear equilibrium compression distance, with all lc_rear identical
kc_rear_min = 50;
kc_rear_max = params.fixed.kc_front;
s_eq_min = 0;
s_eq_max = params.fixed.lc_range(2, 1) - params.fixed.lc_range(1, 1);
z_min = [kc_rear_min kc_rear_min s_eq_min s_eq_min];
z_max = [kc_rear_max kc_rear_max s_eq_max s_eq_max];  % set z min and max
z_init = 0.5 * (z_min + z_max);
obj = @(z) (f_lc_eq(z, params));
opts = optimoptions('fmincon','Algorithm','sqp','Display','off');
[z_star, fval] = fmincon(obj, z_init, [], [], [], [], z_min, z_max, [], opts);

kc_rear_star = z_star(1:2);
s_eq_star = z_star(3:4);  % [front rear] 1*2
params.fixed.kc_rear = [kc_rear_star(1), kc_rear_star(2), kc_rear_star(1), kc_rear_star(2)];  % write into params
params.initial.s_eq = [s_eq_star(1) s_eq_star(2) s_eq_star(2) s_eq_star(2) s_eq_star(2)];
params.initial.lc_eq = [params.fixed.lc_0(1)-s_eq_star(1), params.fixed.lc_0(2)-s_eq_star(2) ...
    params.fixed.lc_0(3)-s_eq_star(2), params.fixed.lc_0(4)-s_eq_star(2), params.fixed.lc_0(5)-s_eq_star(2)];


% define initial human attachment pose q_0
params.initial.q_0 = [0.5*(params.fixed.l_tube - s_eq_star(1) - s_eq_star(2))+params.fixed.l_rod 0 0 0 0 0]';
[params.initial.p_0, params.initial.R_NB_0, params.initial.T_NB_0, params.initial.u_hat0] = pose2matrix(params.initial.q_0);


% initial front attachment as a function of q_0 only
[params.initial.r_attach_front_B_eq, params.initial.r_attach_front_eq] = get_r_attach_front(params, params.initial.T_NB_0);
% initial front extension force
params.initial.Fs_front = zeros(3,1);
for i = 1 : n/2
    r  = params.fixed.r_anchor(:, i) - params.initial.r_attach_front_eq(:, i);   % 3x1
    lei = norm(r);
    params.initial.le_eq(i) = lei;
    if lei < params.fixed.le_range(1, i)  % ensure extension in range
        error('Extension spring with idx %d is initially slack.', i);
    elseif lei > params.fixed.le_range(2, i)
        error('Extension spring with idx %d is initially overstretched.', i);
    else
        ui = r / lei;
        params.initial.Fs_front = params.initial.Fs_front + params.fixed.ke(i) * (lei - params.fixed.le_0(i)) * ui;
    end
end


% initial rear attachment as a function of q_0 and lc_eq
[params.initial.r_attach_rear_B_eq, params.initial.r_attach_rear_eq] = get_r_attach_rear(params, params.initial.s_eq, params.initial.T_NB_0);
% inital rear extension force
params.initial.Fs_rear = zeros(3,1);
for i = n/2+1 : n
    r  = params.fixed.r_anchor(:, i) - params.initial.r_attach_rear_eq(:, i-n/2);   % 3x1
    lei = norm(r);
    params.initial.le_eq(i) = lei;
    if lei < params.fixed.le_range(1, i)  % ensure extension in range
        error('Extension spring with idx %d is initially slack.', i);
    elseif lei > params.fixed.le_range(2, i)
        error('Extension spring with idx %d is initially overstretched.', i);
    end
    ui = r / lei;
    params.initial.Fs_rear = params.initial.Fs_rear + params.fixed.ke(i) * (lei - params.fixed.le_0(i)) * ui;
end


% alter pose, add to q_0
params.q = params.initial.q_0 + [dx dy dz roll pitch yaw]';
[params.p, params.R_NB, params.T_NB, params.u_hat] = pose2matrix(params.q);


% calculate front extension force as a function of q only 
[~, params.r_attach_front] = get_r_attach_front(params, params.T_NB);% front extension force
params.Fs_front = zeros(3,1);
for i = 1 : n/2
    r  = params.fixed.r_anchor(:, i) - params.r_attach_front(:, i);   % 3x1
    lei = norm(r);
    params.le(i) = lei;
    if lei < params.fixed.le_range(1, i)  % ensure extension in range
        %error('Extension spring with idx %d is slack.', i);
        params.Fs_front = params.Fs_front;
        params.slackIdx(i) = 1;
    elseif lei > params.fixed.le_range(2, i)
            error('Extension spring with idx %d is overstretched.', i);
    else
        ui = r / lei;
        params.Fs_front = params.Fs_front + params.fixed.ke(i) * (lei - params.fixed.le_0(i)) * ui;
    end
end


% optimization to find new compression distance
s_min = 0 * ones(1, n/2 + 1);
s_max = params.fixed.lc_range(2, :) - params.fixed.lc_range(1, :);
s_init = 0.5 * (s_max + s_min);
obj = @(s) (f_lc(s, params));
opts = optimoptions('fmincon','Algorithm','sqp','Display','off');
[s_star, fval] = fmincon(obj, s_init, [], [], [], [], s_min, s_max, [], opts);
params.lc = params.fixed.lc_0 - s_star;
for i = 1: n/2 + 1 % ensure compression in range
    if params.lc(i) < params.fixed.lc_range(1, i)
        error('Compression spring with idx %d is over compressed.', i);
    elseif params.lc(i) > params.fixed.lc_range(2, i)
        error('Compression spring with idx %d is over stretched.', i);
    end
end
params.s = s_star;
if fval > 1e1
    error('lc optimization failed to find a solution. ')
end


% rear attachment as a function of q and lc
[params.r_attach_rear_B, params.r_attach_rear] = get_r_attach_rear(params, params.s, params.T_NB);
% rear extension force
params.Fs_rear = zeros(3,1);
for i = n/2+1 : n
    r  = params.fixed.r_anchor(:, i) - params.r_attach_rear(:, i-n/2);   % 3x1
    lei = norm(r);
    params.le(i) = lei;
    if lei < params.fixed.le_range(1, i)  % ensure extension in range
        %error('Extension spring with idx %d is slack.', i);
        params.Fs_rear = params.Fs_rear;
        params.slackIdx(i) = 1;
    elseif lei > params.fixed.le_range(2, i)
        error('Extension spring with idx %d is overstretched.', i);
    end
    ui = r / lei;
    params.Fs_rear = params.Fs_rear + params.fixed.ke(i) * (lei - params.fixed.le_0(i)) * ui;
end

% put front and rear r_attach together
params.r_attach = [params.r_attach_front params.r_attach_rear];
params.r_attach_B = [params.fixed.r_attach_front_B params.r_attach_rear_B];
params.initial.r_attach_eq = [params.initial.r_attach_front_eq params.initial.r_attach_rear_eq];


% compute extension spring unit directional vector, absolute length and
% pure extension
r = params.r_attach - params.fixed.r_anchor;
params.le = vecnorm(r, 2, 1);
params.u = r ./ vecnorm(r, 2, 1);
params.e = params.le - params.fixed.le_0;


% compute wrench
[params.W_out, ~] = compute_wrench(params);


% simulate known distance measured by the string encoder
params.l_str = zeros(n/2, 1);
for i = 1:n/2
    params.initial.l_str_eq(i) = params.fixed.l_rod + params.fixed.l_tube - params.initial.s_eq(1) - params.initial.s_eq(1+i);
    params.l_str(i) = params.fixed.l_rod + params.fixed.l_tube - params.s(1) - params.s(1+i);
end

end

%[appendix]{"version":"1.0"}
%---
