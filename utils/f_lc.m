% function to compute Lc distance with altered pose
function f = f_lc(lc_delta, params)

% extract variables
n = params.fixed.n;
kc_front = params.fixed.kc_front;
kc_rear = params.fixed.kc_rear;
r_anchor = params.fixed.r_anchor;
ks = params.fixed.ks;
ls_0 = params.fixed.ls_0;
T_NB = params.T_NB;
u_hat = params.u_hat;
f_string = params.fixed.f_string;

% define cost
f = 0;

% generate r_attach_rear points as a function of Lc and T_NB
[~, r_attach_rear] = get_r_attach_rear(params, lc_delta, T_NB);

% compute idx Fs_rear
Fs_rear = zeros(3,1);
for idx = n/2+1:n
    local_idx = idx - n/2;

    s = r_anchor(:, idx) - r_attach_rear(:, local_idx);
    li = norm(s);
    s_hat = s / li;

    Fs_rear_idx = ks(idx) * (li - ls_0(idx)) * s_hat;
    Fs_rear = Fs_rear + Fs_rear_idx;
    
    % force balance cost between idx Fs_rear and Fc_rear
    f = f + (kc_rear(local_idx) * lc_delta(local_idx+1) - u_hat.' * Fs_rear_idx )^2;
end

% force balance cost between total Fs_rear and Fc_front
f = f + (kc_front * lc_delta(1) - sum(kc_rear .* lc_delta(2:end)))^2;

end