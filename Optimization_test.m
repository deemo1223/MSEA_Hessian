clear
addpath('utils/')


% use optimization to get ground truth
configFile = "config.json";
q_oper = [-0.008 0 0 0 0 0]'; % set operating point
p = optimization_params(q_oper, configFile);
n = p.fixed.n;  % no. of extension springs

p.W_out %[output:8c3b4a0d]

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":45.2}
%---
%[output:8c3b4a0d]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"ans","rows":6,"type":"double","value":[["21.9140"],["0"],["0"],["0"],["0"],["0"]]}}
%---
