function [params, error_flag] = solve_msea_state(q, params)

n = params.fixed.n;
params.slackIdx = zeros(1, n); 
error_flag = 0;

% generate matrices according to the input q
params.q = q;
[params.p, params.R_NB, params.T_NB, params.u_hat] = pose2matrix(params.q);

% calculate front extension force as a function of q only 
[~, params.r_attach_front] = get_r_attach_front(params, params.T_NB);

params.Fs_front = zeros(3,1);
for i = 1 : n/2
    r  = params.fixed.r_anchor(:, i) - params.r_attach_front(:, i);   % 3x1
    lei = norm(r);
    params.le(i) = lei;
    ei = lei - params.fixed.le_0(i);

    % extension spring slack
    if ei <= 0 
        params.slackIdx(i) = 1;
     % extension spring overstrecthed
    elseif lei > params.fixed.le_range(2, i)
        error_flag = 1;
        return
    else
        ui = r / lei;
        params.Fs_front = params.Fs_front + params.fixed.ke(i) * ei * ui;
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

    % compression spring over compressed
    if params.lc(i) < params.fixed.lc_range(1, i)
        error_flag = 1;
        return
    % compression spring strecthed 
    elseif params.lc(i) > params.fixed.lc_range(2, i)
        error_flag = 1;
        return
    end
end
params.s = s_star;
if fval > 1e1
    %error('lc optimization failed to find a solution. ')
    error_flag = 1;
    return
end


% rear attachment as a function of q and lc
[params.r_attach_rear_B, params.r_attach_rear] = get_r_attach_rear(params, params.s, params.T_NB);
% rear extension force
params.Fs_rear = zeros(3,1);
for i = n/2+1 : n
    r  = params.fixed.r_anchor(:, i) - params.r_attach_rear(:, i-n/2);   % 3x1
    lei = norm(r);
    params.le(i) = lei;
    ei = lei - params.fixed.le_0(i);

    % extension spring slack
    if ei <= 0 
        params.slackIdx(i) = 1;

    % extension spring overstretched
    elseif lei > params.fixed.le_range(2, i)
        error_flag = 1;
        return
    else
        ui = r / lei;
        params.Fs_rear = params.Fs_rear + params.fixed.ke(i) * ei * ui;
    end
end

% put front and rear r_attach together
params.r_attach = [params.r_attach_front params.r_attach_rear];
params.r_attach_B = [params.fixed.r_attach_front_B params.r_attach_rear_B];
params.initial.r_attach_eq = [params.initial.r_attach_front_eq params.initial.r_attach_rear_eq];


% compute extension spring unit directional vector, absolute length and
% pure extension
r = params.fixed.r_anchor - params.r_attach;
params.le = vecnorm(r, 2, 1);
params.u = r ./ params.le;
params.e = params.le - params.fixed.le_0;
params.slackIdx = params.e <= 0;

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
