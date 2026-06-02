clearvars -except params
addpath('utils/')


% use optimization to get ground truth
configFile = "config.json";
delta_q = [-0.07 0 0 0 0 0]'; % set operating point
p = optimization_params(delta_q, configFile);
n = p.fixed.n;  % no. of extension springs


% define Bis
Bis = zeros(3, 5, n);
for i = 1:n
    if i <= 4  % front 4 springs, shared slider
        Bis(1, 1, i) = -1;
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
Keff = Kqq - Kqs * (Kss \ Ksq) %[output:34c63e16]


% test Keff
test_q = [-0.005 0 0 0 0 0]';
Keff*test_q %[output:1ebe350b]


% test sensing


%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":48.9}
%---
%[output:34c63e16]
%   data: {"dataType":"matrix","outputData":{"columns":6,"exponent":"3","name":"Keff","rows":6,"type":"double","value":[["1.5218","0.0000","-0.0000","0.0000","-0.0000","-0.0000"],["0.0000","2.3847","0.0000","-0.0000","0.0000","-0.4934"],["-0.0000","0.0000","7.0143","-0.0000","1.1978","-0.0000"],["0.0000","-0.0000","-0.0000","0.0008","-0.0000","0.0000"],["-0.0000","0.0000","1.1978","-0.0000","0.2539","-0.0000"],["-0.0000","-0.4934","-0.0000","0.0000","-0.0000","0.1180"]]}}
%---
%[output:1ebe350b]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"ans","rows":6,"type":"double","value":[["-7.6092"],["-0.0000"],["0.0000"],["-0.0000"],["0.0000"],["0.0000"]]}}
%---
