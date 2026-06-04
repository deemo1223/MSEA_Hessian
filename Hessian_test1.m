clearvars -except params
addpath('utils/')


% use optimization to get ground truth
configFile = "config.json";
q_oper = [-0.04 0.01 0.01 0 0 0]'; % set operating point
p_oper = optimization_params(q_oper, configFile);
n = p_oper.fixed.n;  % no. of extension springs


% define Bis
Bis = zeros(3, 5, n);
for i = 1:n
    if i <= 4  % front 4 springs, shared slider
        Bis(1, 1, i) = 1;
    else  % rear 4 springs, individual sliders 
        Bis(1, i-3, i) = 1;
    end
end


% compute Ji and Jri
Jr = zeros(3, 11, n);
J = zeros(1, 11, n);

for i = 1: n

    % compute Jri and put into Jr
    roi = p_oper.R_NB * p_oper.r_attach_B(:, i);
    Jri = [-eye(3) skew(roi) -p_oper.R_NB*Bis(:, :, i)];
    Jr(:, :, i) = Jri;

    % compute Ji as the projection of Jri in the direction of ui
    ui = p_oper.u(:, i);
    Ji = [-ui' ui'*skew(roi) -ui'*p_oper.R_NB*Bis(:, :, i)];
    J(:, :, i) = Ji;
end


% compute HLi
HL = zeros(11, 11, n);
for i = 1: n
    ui = p_oper.u(:, i);
    lei = p_oper.le(i);
    HLi = Jr(:, :, i)' * ((eye(3) - ui*ui')/lei) * Jr(:, :, i);
    HL(:, :, i) = HLi;
end


% compute H
Kc = zeros(11, 11);
Kc(end-4:end, end-4:end) = diag([p_oper.fixed.kc_front p_oper.fixed.kc_rear]);
HLi_sum = zeros(11, 11);
for i = 1: n
    ei = p_oper.e(i);
    kei = p_oper.fixed.ke(i);
    Ji = J(:, :, i);
    HLi = HL(:, :, i);

    HLi_sum = HLi_sum + (kei*(Ji'*Ji) + kei*ei*HLi);
end
H = HLi_sum + Kc;


% extract Kqq Kqs Ksq and Kss from H
Kqq = H(1:6, 1:6);
Kss = H(end-4:end, end-4:end);
Kqs = H(1:6, end-4:end);
Ksq = H(end-4:end, 1:6);


% calcualte Keff
Keff = Kqq - Kqs * (Kss \ Ksq);
As = - Kss \ Ksq;

% test sensing
% set evaluation point, use optimization again
q_delta = [-0.001 0.002 -0.003 deg2rad(0.1) deg2rad(0.1) deg2rad(0.1)]'; 
q_eval = q_oper + q_delta %[output:26bf864c]
p_eval = optimization_params(q_eval, configFile);

% % compare output wrench
% H_wrench = -Keff*q_delta + p.W_out
% O_wrench = p_eval.W_out
% % compare compression distance
% H_s = As * q_delta + p.s'
% O_s = p_eval.s

% get the current measured string length, compute delta
l_str = p_eval.l_str;
l_str_delta = l_str - p_oper.l_str %[output:4858b3f9]

% get the current measure IMU angle, compute delta
theta = p_eval.q(4:6) %[output:0d2de57d]
theta_delta = theta - p_oper.q(4:6);

% define Cs and Jstr matrix
Cs = zeros(4, 5);
Cs(:, 2:end) = -eye(4);
Cs(:, 1) = -1;
Jstr = Cs * As;
Jstr_p = Jstr(:, 1:3);
Jstr_theta = Jstr(:, 4:6);

% compute Ls to find p_delta
lambda = 1e-3;
[~, m] = size(Jstr_p);
A_aug = [Jstr_p; lambda * eye(m)];
b = l_str_delta - Jstr_theta * theta_delta;
b_aug = [b; zeros(m, 1)];

p_delta = A_aug \ b_aug;
p_star = p_oper.q(1:3) + p_delta %[output:4b92167f]
p_old = p_oper.q(1:3) %[output:0a8f5f9c]
p_new = p_eval.q(1:3) %[output:0c7728b0]


%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":35.2}
%---
%[output:26bf864c]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"q_eval","rows":6,"type":"double","value":[["-0.0410"],["0.0120"],["0.0070"],["0.0017"],["0.0017"],["0.0017"]]}}
%---
%[output:4858b3f9]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"l_str_delta","rows":4,"type":"double","value":[["-0.0001"],["-0.0017"],["-0.0011"],["0.0002"]]}}
%---
%[output:0d2de57d]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"theta","rows":3,"type":"double","value":[["0.0017"],["0.0017"],["0.0017"]]}}
%---
%[output:4b92167f]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"p_star","rows":3,"type":"double","value":[["0.1711"],["0.0119"],["0.0071"]]}}
%---
%[output:0a8f5f9c]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"p_old","rows":3,"type":"double","value":[["0.1722"],["0.0100"],["0.0100"]]}}
%---
%[output:0c7728b0]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"p_new","rows":3,"type":"double","value":[["0.1712"],["0.0120"],["0.0070"]]}}
%---
