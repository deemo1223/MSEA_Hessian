% function to convert 6*1 pose to matrices 
function [p, R, T, u_hat] = pose2matrix(q)

p = [q(1), q(2), q(3)]';
R = rotz(q(6)) * roty(q(5)) * rotx(q(4));
u_hat = R * [1 0 0]';
T = [R p; 0 0 0 1];

end

%[appendix]{"version":"1.0"}
%---
