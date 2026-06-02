function radial_points = generate_radial_points(n, theta, phi, arm_length, L_tube)

    if mod(n,2) ~= 0
        error('n must be even.');
    end

    half_n = n/2;
    radial_points = zeros(n, 3);

    for i = 1:half_n
        % -------- Front point --------
        x_front = L_tube/2 - arm_length(i) * cos(theta(i));
        R_front = arm_length(i) * sin(theta(i));
        y_front = R_front * cos(phi(i));
        z_front = R_front * sin(phi(i));

        radial_points(i, :) = [x_front, y_front, z_front];

        % -------- Rear point --------
        j = i + half_n;

        % Keep x symmetric with respect to x = 0
        x_rear = -x_front;

        % But allow its own theta / phi / arm_length for yz distribution
        R_rear = arm_length(j) * sin(theta(j));
        y_rear = R_rear * cos(phi(j));
        z_rear = R_rear * sin(phi(j));

        radial_points(j, :) = [x_rear, y_rear, z_rear];
    end

    radial_points = radial_points';
end

%[appendix]{"version":"1.0"}
%---
