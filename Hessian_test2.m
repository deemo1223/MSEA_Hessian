clear
addpath('utils/')

% initialize MSEA model, .fixed and .initial
configFile = "config.json";
params = initialize_msea_model(configFile);

% extract x0 corresponding to config
x0 = params.initial.q_0(1);

% set operating point and corresponding index
q_oper = [0 0.05 0 0 0 0]';
q_oper(1) = q_oper(1) + x0;
index = 0;
index_soft = 1;
beta = 1000;

% use optimization to compute operating point states
[params_oper, errorFlag] = solve_msea_state(q_oper, params);
errorFlag %[output:560ec8d7]

% prepare operating point Hessian info
referenceFile = 'reference_info.csv';
prepare_info(referenceFile, params_oper, index, 'normal');
prepare_info(referenceFile, params_oper, index_soft, 'soft', beta);

% define delta coordinate, set resultant evaluating point
q_delta = [0.001 -0.002 0.003 0.001 -0.002 0.003]'; 
q_eval = q_oper + q_delta;

% use optimization to compute evaluating point states
[params_eval, errorFlag] = solve_msea_state(q_eval, params);
errorFlag %[output:7689fc69]

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
W_oper = params_oper.W_out %[output:31ee8547]
p_oper= params_oper.q(1:3) %[output:29a9a0ce]

W_star %[output:559b030a]
p_star %[output:15f05c27]

W_star_soft %[output:95c1ee56]
p_star_soft %[output:3ab6a2ab]

W_eval = params_eval.W_out %[output:6c38c9ac]
p_eval= params_eval.q(1:3) %[output:6fffc7fa]

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":62.1}
%---
%[output:560ec8d7]
%   data: {"dataType":"textualVariable","outputData":{"name":"errorFlag","value":"0"}}
%---
%[output:7689fc69]
%   data: {"dataType":"textualVariable","outputData":{"name":"errorFlag","value":"0"}}
%---
%[output:31ee8547]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"W_oper","rows":6,"type":"double","value":[["-9.4123"],["-134.4175"],["0"],["0"],["0"],["27.1983"]]}}
%---
%[output:29a9a0ce]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"p_oper","rows":3,"type":"double","value":[["0.2122"],["0.0500"],["0"]]}}
%---
%[output:559b030a]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"W_star","rows":6,"type":"double","value":[["-12.1612"],["-125.1459"],["-15.3604"],["-0.0718"],["-2.9084"],["25.1846"]]}}
%---
%[output:15f05c27]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"p_star","rows":3,"type":"double","value":[["0.2133"],["0.0477"],["0.0029"]]}}
%---
%[output:95c1ee56]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"W_star_soft","rows":6,"type":"double","value":[["-12.1600"],["-126.0140"],["-15.4653"],["-0.0744"],["-2.9198"],["25.2488"]]}}
%---
%[output:3ab6a2ab]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"p_star_soft","rows":3,"type":"double","value":[["0.2135"],["0.0477"],["0.0029"]]}}
%---
%[output:6c38c9ac]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"W_eval","rows":6,"type":"double","value":[["-11.0856"],["-127.2703"],["-16.3635"],["-0.1247"],["-3.0355"],["25.6675"]]}}
%---
%[output:6fffc7fa]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"p_eval","rows":3,"type":"double","value":[["0.2132"],["0.0480"],["0.0030"]]}}
%---
