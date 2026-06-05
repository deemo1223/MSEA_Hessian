clear
addpath('utils/')

% define operating point, generate params
configFile = "config.json";
operating_index = 3;
q_oper = [-0.01 0.01 -0.014 deg2rad(0.5) deg2rad(0.4) deg2rad(0.3)]'; % set operating point,(does not include initial)
n = 8;
point_oper = optimization_params(q_oper, configFile);
q_0 = point_oper.initial.q_0;

% prepare operating point Hessian info
filename = 'reference_info.csv';
prepare_info(filename, point_oper, operating_index);


% define delta coordinate, set resultant evaluating point and generate
% params
q_delta = [-0.001 -0.001 0.001 0 0 0]'; 
q_eval = q_oper + q_delta;
point_eval = optimization_params(q_eval, configFile);


% extract operating point Hessian info
ref = load_info(filename, operating_index);

% get the current measured string length, compute delta
l_str = point_eval.l_str;

% get the current measure IMU angle
theta = point_eval.q(4:6);

% make prediction on q and W_out
[W_star, p_star] = make_prediction(ref, l_str, theta);
W_star %[output:9dff0306]
p_star %[output:9cd53807]

% compare with operating and evaluating
W_oper = point_oper.W_out %[output:873bdc4b]
p_oper= point_oper.q(1:3) %[output:36308a88]
W_eval = point_eval.W_out %[output:6a2290cc]
p_eval= point_eval.q(1:3) %[output:1fcc02e3]

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":41}
%---
%[output:9dff0306]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"W_star","rows":6,"type":"double","value":[["29.4559"],["-19.9158"],["75.0591"],["-0.0443"],["13.5541"],["4.0186"]]}}
%---
%[output:9cd53807]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"p_star","rows":3,"type":"double","value":[["0.2014"],["0.0090"],["-0.0130"]]}}
%---
%[output:873bdc4b]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"W_oper","rows":6,"type":"double","value":[["27.8702"],["-22.4161"],["81.3495"],["-0.0336"],["14.7777"],["4.5300"]]}}
%---
%[output:36308a88]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"p_oper","rows":3,"type":"double","value":[["0.2022"],["0.0100"],["-0.0140"]]}}
%---
%[output:6a2290cc]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"W_eval","rows":6,"type":"double","value":[["30.3519"],["-19.8685"],["75.1101"],["-0.0435"],["13.5902"],["4.0164"]]}}
%---
%[output:1fcc02e3]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"p_eval","rows":3,"type":"double","value":[["0.2012"],["0.0090"],["-0.0130"]]}}
%---
