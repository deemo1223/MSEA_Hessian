clear
addpath('utils/')


% use optimization to get ground truth
configFile = "config.json";
q_oper = [0 0 0 0 0 0]'; % set operating point
p = optimization_params(q_oper, configFile);
n = p.fixed.n;  % no. of extension springs

folder = fullfile(pwd, 'reference_info');
if ~exist(folder, 'dir')
    mkdir(folder);
end

index = 0 %[output:0b2091c1]
prepare_info(p, index)

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":21.2}
%---
%[output:0b2091c1]
%   data: {"dataType":"textualVariable","outputData":{"name":"index","value":"0"}}
%---
