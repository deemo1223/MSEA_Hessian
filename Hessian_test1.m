clearvars -except params
addpath('utils/')


% use optimization to get ground truth
configFile = "config.json";
q_oper = [-0.008 0 0 0 0 0]'; % set operating point
p = optimization_params(q_oper, configFile);
n = p.fixed.n;  % no. of extension springs


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
    roi = p.R_NB * p.r_attach_B(:, i);
    Jri = [-eye(3) skew(roi) -p.R_NB*Bis(:, :, i)];
    Jr(:, :, i) = Jri;

    % compute Ji as the projection of Jri in the direction of ui
    ui = p.u(:, i);
    Ji = [-ui' ui'*skew(roi) -ui'*p.R_NB*Bis(:, :, i)];
    J(:, :, i) = Ji;
end


% compute HLi
HL = zeros(11, 11, n);
for i = 1: n
    ui = p.u(:, i);
    lei = p.le(i);
    HLi = Jr(:, :, i)' * ((eye(3) - ui*ui')/lei) * Jr(:, :, i);
    HL(:, :, i) = HLi;
end


% compute H
Kc = zeros(11, 11);
Kc(end-4:end, end-4:end) = diag([p.fixed.kc_front p.fixed.kc_rear]);
HLi_sum = zeros(11, 11);
for i = 1: n
    ei = p.e(i);
    kei = p.fixed.ke(i);
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


% test sensing
% set evaluation point, use optimization again
q_delta = [-0.002 -0.002 -0.002 deg2rad(0.1) deg2rad(0.1) deg2rad(0.1)]'; 
q_eval = q_oper + q_delta %[output:26bf864c]
p_eval = optimization_params(q_eval, configFile);

% compare output wrench
H_wrench = -Keff*q_delta + p.W_out %[output:821fb446]
O_wrench = p_eval.W_out %[output:4d4cf3a6]

% compare compression distance
As = - Kss \ Ksq;
H_s = As * q_delta + p.s' %[output:9b45d45f]
O_s = p_eval.s %[output:28bca07e]

% get the current measured string length, compute delta
l_str = p_eval.l_str;
l_str_delta = l_str - p.l_str

% get the current measure IMU angle, compute delta
theta = p_eval.q(4:6)
theta_delta = theta - p.q(4:6);

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

x = A_aug \ b_aug


%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":45.3}
%---
%[output:26bf864c]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"q_eval","rows":6,"type":"double","value":[["-0.0100"],["-0.0020"],["-0.0020"],["0.0017"],["0.0017"],["0.0017"]]}}
%---
%[output:821fb446]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"H_wrench","rows":6,"type":"double","value":[["26.3808"],["5.9766"],["10.6468"],["-0.0017"],["1.9268"],["-1.2827"]]}}
%---
%[output:4d4cf3a6]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"O_wrench","rows":6,"type":"double","value":[["27.1217"],["5.9420"],["10.7704"],["-0.0201"],["1.9190"],["-1.2165"]]}}
%---
%[output:9b45d45f]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"H_s","rows":5,"type":"double","value":[["0.0333"],["0.0345"],["0.0340"],["0.0323"],["0.0328"]]}}
%---
%[output:28bca07e]
%   data: {"dataType":"matrix","outputData":{"columns":5,"name":"O_s","rows":1,"type":"double","value":[["0.0331","0.0343","0.0337","0.0320","0.0326"]]}}
%---
