clearvars -except params
addpath('utils/')


% use optimization to get ground truth
configFile = "config.json";
q_oper = [-0.0 0.0 0.0 0 0 0]'; % set operating point
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
Keff = Kqq - Kqs * (Kss \ Ksq) %[output:4858b3f9]
As = - Kss \ Ksq %[output:4b92167f]


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
l_str_delta = l_str - point_oper.l_str %[output:0a8f5f9c]

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
p_star = point_oper.q(1:3) + p_delta_star %[output:0c7728b0]
p_operating = point_oper.q(1:3) %[output:7a6c074d]
p_evaluating = point_eval.q(1:3) %[output:3ab2ec52]

% compute and compare output wrench 
q_delta_star = [p_delta_star ; theta_delta];
W_delta_star = -Keff*q_delta_star;
W_operating = point_oper.W_out;
W_star = W_operating + W_delta_star %[output:444c4440]

W_operating = point_oper.W_out %[output:66fbe8f9]
W_evaluating = point_eval.W_out %[output:34420747]

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":59.2}
%---
%[output:4858b3f9]
%   data: {"dataType":"matrix","outputData":{"columns":6,"exponent":"3","name":"Keff","rows":6,"type":"double","value":[["2.2828","-0.0000","-0.0000","-0.0000","-0.0000","0.0000"],["-0.0000","2.5717","0.0000","-0.0000","0.0000","-0.5273"],["-0.0000","0.0000","6.3418","0.0000","1.2523","-0.0000"],["-0.0000","-0.0000","0.0000","0.0010","0.0000","0.0000"],["-0.0000","0.0000","1.2523","0.0000","0.3282","-0.0000"],["0.0000","-0.5273","-0.0000","0.0000","-0.0000","0.1382"]]}}
%---
%[output:4b92167f]
%   data: {"dataType":"matrix","outputData":{"columns":6,"name":"As","rows":5,"type":"double","value":[["-0.4459","-0.0000","0","0.0000","0","0.0000"],["-0.4459","-0.4333","0","0","0","0.1477"],["-0.4459","-0.0000","-0.4333","0.0000","-0.1477","0.0000"],["-0.4459","0.4333","-0.0000","-0.0000","-0.0000","-0.1477"],["-0.4459","0.0000","0.4333","-0.0000","0.1477","-0.0000"]]}}
%---
%[output:0a8f5f9c]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"l_str_delta","rows":4,"type":"double","value":[["-0.0001"],["-0.0016"],["-0.0012"],["0.0004"]]}}
%---
%[output:0c7728b0]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"p_star","rows":3,"type":"double","value":[["0.2115"],["0.0019"],["-0.0029"]]}}
%---
%[output:7a6c074d]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"p_operating","rows":3,"type":"double","value":[["0.2122"],["0"],["0"]]}}
%---
%[output:3ab2ec52]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"p_evaluating","rows":3,"type":"double","value":[["0.2112"],["0.0020"],["-0.0030"]]}}
%---
%[output:444c4440]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"W_star","rows":6,"type":"double","value":[["1.5664"],["-3.9394"],["16.1201"],["-0.0018"],["3.0419"],["0.7552"]]}}
%---
%[output:66fbe8f9]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"-4","name":"W_operating","rows":6,"type":"double","value":[["0.2081"],["0"],["0"],["0"],["0"],["0"]]}}
%---
%[output:34420747]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"W_evaluating","rows":6,"type":"double","value":[["2.8668"],["-4.2261"],["16.9401"],["-0.0139"],["3.1232"],["0.8323"]]}}
%---
