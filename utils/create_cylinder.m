function [Xr, Yr, Zr] = create_cylinder(tube_base, h, r, u_hat)

% plot the tube
n = 40;
d = u_hat;
[X,Y,Z] = cylinder(r, n);
Z = Z * h;
z = [0;0;1];
v = cross(z, d);
s = norm(v);
c = dot(z, d);
if s < 1e-8
    R = eye(3);
else
    V = [   0   -v(3)   v(2);
          v(3)     0   -v(1);
         -v(2)  v(1)     0  ];
    R = eye(3) + V + V*V * ((1-c)/(s^2));
end

% Rotate & translate tube
Psurf = R * [X(:)'; Y(:)'; Z(:)'];
Xr = reshape(Psurf(1,:), size(X)) + tube_base(1);
Yr = reshape(Psurf(2,:), size(Y)) + tube_base(2);
Zr = reshape(Psurf(3,:), size(Z)) + tube_base(3);

end

%[appendix]{"version":"1.0"}
%---
