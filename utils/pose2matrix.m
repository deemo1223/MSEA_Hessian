% function to convert 6x1 pose to matrices
% q = [x y z roll pitch yaw]'
% angles are in radians
function [p, R, T, u_hat] = pose2matrix(q)

p = q(1:3);

roll  = q(4);
pitch = q(5);
yaw   = q(6);

Rx = [1, 0,          0;
      0, cos(roll), -sin(roll);
      0, sin(roll),  cos(roll)];

Ry = [ cos(pitch), 0, sin(pitch);
       0,          1, 0;
      -sin(pitch), 0, cos(pitch)];

Rz = [cos(yaw), -sin(yaw), 0;
      sin(yaw),  cos(yaw), 0;
      0,         0,        1];

% same order as your original code:
% R = rotz(yaw) * roty(pitch) * rotx(roll)
R = Rz * Ry * Rx;

u_hat = R * [1; 0; 0];

T = [R, p;
     0, 0, 0, 1];

end

%[appendix]{"version":"1.0"}
%---
