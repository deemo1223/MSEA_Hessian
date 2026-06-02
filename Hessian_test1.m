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
    kei = p.fixed.ks(i);
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
q_delta = [-0.002 -0.002 -0.002 deg2rad(3) 0 0]'; 
q_eval = q_oper + q_delta %[output:118a6152]
p_eval = optimization_params(q_eval, configFile);

% compare output wrench
H_wrench = -Keff*q_delta + p.W_out %[output:624d671a]
O_wrench = p_eval.W_out %[output:18829d3d]

% compare compression distance
As = - Kss \ Ksq;
H_s = As * q_delta + p.lc_delta' %[output:63fcffc4]
O_s = p_eval.lc_delta %[output:3677850f]

% get the current measured string length, compute delta
l_str = p_eval.l_str;
l_str_delta = l_str - p.l_str %[output:1d345037]

% get the current measure IMU angle, compute delta
theta = p_eval.q(4:6) %[output:2c4f6f5e]
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

x = A_aug \ b_aug %[output:8ee9db2d]


%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":36.8}
%---
%[output:118a6152]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"q_eval","rows":6,"type":"double","value":[["-0.0100"],["-0.0020"],["-0.0020"],["0.0524"],["0"],["0"]]}}
%---
%[output:624d671a]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"H_wrench","rows":6,"type":"double","value":[["26.3808"],["5.0646"],["12.8122"],["-0.0518"],["2.4814"],["-1.0450"]]}}
%---
%[output:18829d3d]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"O_wrench","rows":6,"type":"double","value":[["27.0780"],["4.8493"],["13.5687"],["-0.3643"],["2.5150"],["-0.9845"]]}}
%---
%[output:63fcffc4]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"H_s","rows":5,"type":"double","value":[["0.0333"],["0.0343"],["0.0343"],["0.0326"],["0.0326"]]}}
%---
%[output:3677850f]
%   data: {"dataType":"matrix","outputData":{"columns":5,"name":"O_s","rows":1,"type":"double","value":[["0.0332","0.0340","0.0341","0.0324","0.0322"]]}}
%---
%[output:1d345037]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"l_str_delta","rows":4,"type":"double","value":[["-0.0021"],["-0.0022"],["-0.0005"],["-0.0004"]]}}
%---
%[output:2c4f6f5e]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"theta","rows":3,"type":"double","value":[["0.0524"],["0"],["0"]]}}
%---
%[output:8ee9db2d]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"x","rows":3,"type":"double","value":[["-0.0015"],["-0.0019"],["-0.0021"]]}}
%---
