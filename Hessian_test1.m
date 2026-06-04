clearvars -except params
addpath('utils/')


% use optimization to get ground truth
configFile = "config.json";
q_oper = [-0.04 0.01 0.01 0 0 0]'; % set operating point
point_oper = optimization_params(q_oper, configFile);
n = point_oper.fixed.n;  % no. of extension springs


% define Bis, fixed
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
    roi = point_oper.R_NB * point_oper.r_attach_B(:, i);
    Jri = [-eye(3) skew(roi) -point_oper.R_NB*Bis(:, :, i)];
    Jr(:, :, i) = Jri;

    % compute Ji as the projection of Jri in the direction of ui
    ui = point_oper.u(:, i);
    Ji = [-ui' ui'*skew(roi) -ui'*point_oper.R_NB*Bis(:, :, i)];
    J(:, :, i) = Ji;
end


% compute HLi
HL = zeros(11, 11, n);
for i = 1: n
    ui = point_oper.u(:, i);
    lei = point_oper.le(i);
    HLi = Jr(:, :, i)' * ((eye(3) - ui*ui')/lei) * Jr(:, :, i);
    HL(:, :, i) = HLi;
end


% compute H
Kc = zeros(11, 11);
Kc(end-4:end, end-4:end) = diag([point_oper.fixed.kc_front point_oper.fixed.kc_rear]);
HLi_sum = zeros(11, 11);
for i = 1: n
    ei = point_oper.e(i);
    kei = point_oper.fixed.ke(i);
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
q_eval = q_oper + q_delta;
point_eval = optimization_params(q_eval, configFile);

% % compare output wrench
% H_wrench = -Keff*q_delta + p.W_out
% O_wrench = point_eval.W_out
% % compare compression distance
% H_s = As * q_delta + p.s'
% O_s = point_eval.s

% get the current measured string length, compute delta
l_str = point_eval.l_str;
l_str_delta = l_str - point_oper.l_str %[output:4858b3f9]

% get the current measure IMU angle, compute delta
theta = point_eval.q(4:6);
theta_delta = theta - point_oper.q(4:6);

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

% compare displacement
p_star = point_oper.q(1:3) + p_delta_star %[output:4b92167f]
p_operating = point_oper.q(1:3) %[output:0a8f5f9c]
p_evaluating = point_eval.q(1:3) %[output:0c7728b0]

% compute and compare output wrench 
q_delta_star = [p_delta_star ; theta_delta];
W_delta_star = -Keff*q_delta_star;
W_operating = point_oper.W_out;
W_star = W_operating + W_delta_star %[output:7a6c074d]

W_operating = point_oper.W_out %[output:3ab2ec52]
W_evaluating = point_eval.W_out %[output:444c4440]

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":33.7}
%---
%[output:4858b3f9]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"l_str_delta","rows":4,"type":"double","value":[["-0.0001"],["-0.0017"],["-0.0011"],["0.0002"]]}}
%---
%[output:4b92167f]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"p_star","rows":3,"type":"double","value":[["0.1711"],["0.0119"],["0.0071"]]}}
%---
%[output:0a8f5f9c]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"p_operating","rows":3,"type":"double","value":[["0.1722"],["0.0100"],["0.0100"]]}}
%---
%[output:0c7728b0]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"p_evaluating","rows":3,"type":"double","value":[["0.1712"],["0.0120"],["0.0070"]]}}
%---
%[output:7a6c074d]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"W_star","rows":6,"type":"double","value":[["93.7822"],["-27.8765"],["-50.1779"],["-0.0658"],["-8.6280"],["5.6969"]]}}
%---
%[output:3ab2ec52]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"W_operating","rows":6,"type":"double","value":[["91.9286"],["-24.1113"],["-67.1789"],["-0.0712"],["-11.5864"],["4.9473"]]}}
%---
%[output:444c4440]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"W_evaluating","rows":6,"type":"double","value":[["93.1619"],["-27.7517"],["-49.1309"],["-0.0469"],["-8.4889"],["5.7466"]]}}
%---
