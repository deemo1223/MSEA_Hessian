clear
addpath('utils/')

% initialize MSEA model, .fixed and .initial
configFile = "config.json";
params = initialize_msea_model(configFile);

% extract x0 corresponding to config
x0 = params.initial.q_0(1);

% set operating point and corresponding index
q_oper = [-0.03 0.02 -0.01 deg2rad(3) deg2rad(3) deg2rad(5)]';
q_oper(1) = q_oper(1) + x0;
index = 0;

% use optimization to compute operating point states
[params_oper, errorFlag] = solve_msea_state(q_oper, params);
errorFlag %[output:7c7d100b]

% prepare operating point Hessian info
referenceFile = 'reference_info.csv';
prepare_info(referenceFile, params_oper, index);

% define delta coordinate, set resultant evaluating point
q_delta = [0.001 -0.002 0.003 0.01 -0.01 0.01]'; 
q_eval = q_oper + q_delta;

% use optimization to compute evaluating point states
[params_eval, errorFlag] = solve_msea_state(q_eval, params);
errorFlag %[output:6e371460]

% extract operating point Hessian info
ref = load_info(referenceFile, index);

% get the current measured string length, compute delta
l_str = params_eval.l_str;

% get the current measure IMU angle
theta = params_eval.q(4:6);

% make prediction on q and W_out
[W_star, p_star] = make_prediction(ref, l_str, theta);

% compare with operating and evaluating
W_oper = params_oper.W_out %[output:0e462224]
p_oper= params_oper.q(1:3) %[output:409fc5e9]

W_star %[output:2f4c8f46]
p_star %[output:3c1beb1a]

W_eval = params_eval.W_out %[output:5506ccb6]
p_eval= params_eval.q(1:3) %[output:7ccf35eb]

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":51.1}
%---
%[output:7c7d100b]
%   data: {"dataType":"textualVariable","outputData":{"name":"errorFlag","value":"0"}}
%---
%[output:6e371460]
%   data: {"dataType":"textualVariable","outputData":{"name":"errorFlag","value":"0"}}
%---
%[output:0e462224]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"W_oper","rows":6,"type":"double","value":[["69.7338"],["-5.3474"],["4.7363"],["-0.1808"],["-0.6495"],["1.4965"]]}}
%---
%[output:409fc5e9]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"p_oper","rows":3,"type":"double","value":[["0.1822"],["0.0200"],["-0.0100"]]}}
%---
%[output:2f4c8f46]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"W_star","rows":6,"type":"double","value":[["67.3084"],["6.3288"],["1.1485"],["-0.3390"],["-0.6179"],["-1.2254"]]}}
%---
%[output:3c1beb1a]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"p_star","rows":3,"type":"double","value":[["0.1832"],["0.0174"],["-0.0074"]]}}
%---
%[output:5506ccb6]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"W_eval","rows":6,"type":"double","value":[["67.4759"],["4.4850"],["-3.3173"],["-0.2299"],["-1.8293"],["-0.5124"]]}}
%---
%[output:7ccf35eb]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"p_eval","rows":3,"type":"double","value":[["0.1832"],["0.0180"],["-0.0070"]]}}
%---
