clear
addpath('utils/')


% initialize MSEA model, .fixed and .initial
configFile = "config.json";
params = initialize_msea_model(configFile);

% extract x0
x0 = params.initial.q_0(1);

% set generalized coordinate with added x0
q = [-0.03 0.01 0.01 0 0 0]';
q(1) = q(1) + x0;

% use optimization to compute results
[params, errorFlag] = solve_msea_state(q, params);

errorFlag %[output:4cbd15f7]
params.q %[output:6e38b064]
params.W_out %[output:9c3b4337]

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":30.2}
%---
%[output:4cbd15f7]
%   data: {"dataType":"textualVariable","outputData":{"name":"errorFlag","value":"0"}}
%---
%[output:6e38b064]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"ans","rows":6,"type":"double","value":[["0.1822"],["0.0100"],["0.0100"],["0"],["0"],["0"]]}}
%---
%[output:9c3b4337]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"ans","rows":6,"type":"double","value":[["73.2325"],["-24.4134"],["-66.0931"],["-0.0690"],["-11.6621"],["5.0028"]]}}
%---
