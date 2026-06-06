clear
addpath('utils/')

% initialize MSEA model, .fixed and .initial
configFile = "config.json";
params = initialize_msea_model(configFile);

% extract x0 corresponding to config
x0 = params.initial.q_0(1);

% set operating point and corresponding index
q_oper = [-0.05 0.04 0.01 deg2rad(15) deg2rad(3) deg2rad(0)]';
q_oper(1) = q_oper(1) + x0;
index = 0;
index_soft = 1;

% use optimization to compute operating point states
[params_oper, errorFlag] = solve_msea_state(q_oper, params);
errorFlag %[output:7c7d100b]

% prepare operating point Hessian info
referenceFile = 'reference_info.csv';
prepare_info(referenceFile, params_oper, index);
prepare_info_soft(referenceFile, params_oper, index_soft, 'soft');

% define delta coordinate, set resultant evaluating point
q_delta = [0.001 -0.002 0.003 0.01 -0.01 0.01]'; 
q_eval = q_oper + q_delta;

% use optimization to compute evaluating point states
[params_eval, errorFlag] = solve_msea_state(q_eval, params);
errorFlag %[output:6e371460]

% extract operating point Hessian info
ref = load_info(referenceFile, index);
ref_soft = load_info(referenceFile, index_soft);

% get the current measured string length, compute delta
l_str = params_eval.l_str;

% get the current measure IMU angle
theta = params_eval.q(4:6);

% make prediction on q and W_out
[W_star, p_star] = make_prediction(ref, l_str, theta);
[W_star_soft, p_star_soft] = make_prediction(ref_soft, l_str, theta);

% compare with operating and evaluating
W_oper = params_oper.W_out %[output:0e462224]
p_oper= params_oper.q(1:3) %[output:409fc5e9]

W_star %[output:2f4c8f46]
p_star %[output:3c1beb1a]

W_star_soft %[output:5506ccb6]
p_star_soft %[output:7ccf35eb]

W_eval = params_eval.W_out %[output:44b7d04c]
p_eval= params_eval.q(1:3) %[output:942f0914]

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":70.5}
%---
%[output:7c7d100b]
%   data: {"dataType":"textualVariable","outputData":{"name":"errorFlag","value":"0"}}
%---
%[output:6e371460]
%   data: {"dataType":"textualVariable","outputData":{"name":"errorFlag","value":"0"}}
%---
%[output:0e462224]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"W_oper","rows":6,"type":"double","value":[["106.4025"],["-110.7575"],["-138.0352"],["-1.7412"],["-22.8312"],["21.7972"]]}}
%---
%[output:409fc5e9]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"p_oper","rows":3,"type":"double","value":[["0.1562"],["0.0400"],["0.0100"]]}}
%---
%[output:2f4c8f46]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"W_star","rows":6,"type":"double","value":[["103.2100"],["-93.9947"],["-137.4004"],["-1.8274"],["-22.2934"],["18.3530"]]}}
%---
%[output:3c1beb1a]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"p_star","rows":3,"type":"double","value":[["0.1578"],["0.0367"],["0.0116"]]}}
%---
%[output:5506ccb6]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"W_star_soft","rows":6,"type":"double","value":[["103.2365"],["-95.8623"],["-136.9162"],["-1.7964"],["-22.2380"],["18.5249"]]}}
%---
%[output:7ccf35eb]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"p_star_soft","rows":3,"type":"double","value":[["0.1581"],["0.0367"],["0.0116"]]}}
%---
%[output:44b7d04c]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"W_eval","rows":6,"type":"double","value":[["106.3118"],["-100.2903"],["-147.4547"],["-1.8372"],["-24.5145"],["19.9309"]]}}
%---
%[output:942f0914]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"p_eval","rows":3,"type":"double","value":[["0.1572"],["0.0380"],["0.0130"]]}}
%---
