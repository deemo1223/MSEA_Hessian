% function to visualize the MSEA
function visualize_MSEA(params, titleStr)

% extract variables
n = params.fixed.n;
R_tube = params.fixed.R_tube;
l_tube = params.fixed.l_tube;
l_rod = params.fixed.l_rod;
Lc = params.Lc;
Lc_range = params.fixed.Lc_range;
u_hat = params.u_hat;
r_anchor = params.fixed.r_anchor;
r_attach = params.r_attach;
r_attach_front = params.r_attach_front;
r_attach_rear = params.r_attach_rear;

% plot points
anchorRadius = 0.05;
attachRadius = 0.05;
[Xs,Ys,Zs] = sphere(20);      % unit sphere mesh
figure
hold on
for i = 1:size(r_anchor, 2)  % anchor points
    Xs_i = anchorRadius * Xs + r_anchor(1, i);
    Ys_i = anchorRadius * Ys + r_anchor(2, i);
    Zs_i = anchorRadius * Zs + r_anchor(3, i);
    surf(Xs_i, Ys_i, Zs_i, ...
        'FaceColor','r', 'EdgeColor','none');
    % ---- add label ----
    offset = 1.1*anchorRadius;   % adjustable range：1.0~2.0
    text(r_anchor(1, i) + offset, r_anchor(2, i) + offset, r_anchor(3, i) + offset, ...
        ['Anchor' num2str(i)], ...
        'FontSize',10,'FontWeight','bold', ...
        'Color','k', ...
        'HorizontalAlignment','left', ...
        'VerticalAlignment','bottom');
end
for i = 1: size(r_attach, 2)  % attach points
    Xs_i = attachRadius * Xs + r_attach(1, i);
    Ys_i = attachRadius * Ys + r_attach(2, i);
    Zs_i = attachRadius * Zs + r_attach(3, i);
    surf(Xs_i, Ys_i, Zs_i, ...
        'FaceColor','y', 'EdgeColor','none');
    % ---- add label ----
    offset = 1.1*anchorRadius;   % adjustable range：1.0~2.0
    text(r_attach(1, i) + offset, r_attach(2, i) + offset, r_attach(3, i) + offset, ...
        ['Attach' num2str(i)], ...
        'FontSize',10,'FontWeight','bold', ...
        'Color','k', ...
        'HorizontalAlignment','left', ...
        'VerticalAlignment','bottom');
end
view(3)
axis equal
grid on

% ---- add optional title ----
if nargin >= 2 && ~isempty(titleStr)
    title(titleStr);
end

% compute the front and rear attach point central point
r_attach_center_front = mean(r_attach_front, 2);


% plot the tube 
[Xr, Yr, Zr] = create_cylinder(r_attach_center_front, l_tube, R_tube, -u_hat);
surf(Xr, Yr, Zr, 'FaceColor', [0.3 0.8 0.3], 'EdgeColor', 'none');

% plot the rod
[Xr, Yr, Zr] = create_cylinder(r_attach_center_front, l_rod, 0.5*R_tube, u_hat);
surf(Xr, Yr, Zr, 'FaceColor', [0.8 0.8 0.3], 'EdgeColor', 'none');

% plot the compression springs
for i = 1: n/2
    [Xr, Yr, Zr] = create_cylinder(r_attach_rear(:, i), max(Lc_range) - Lc(i), attachRadius, u_hat);
    surf(Xr, Yr, Zr, 'FaceColor', [0.8 0 0], 'EdgeColor', 'none');
end


% plot the springs as lines
% Line style parameters
lineColor = [0.1 0 0.8];   % black
lineWidth = 1.5;

% First 4 springs (right side) -> Q(:,1)
for i = 1:n/2
    plot3([r_anchor(1, i), r_attach(1, i)], ...
          [r_anchor(2, i), r_attach(2, i)], ...
          [r_anchor(3, i), r_attach(3, i)], ...
          '-', 'Color', lineColor, 'LineWidth', lineWidth);
end

% Next 3 springs (left side) -> Q(:,2)
for i = (n/2+1) : n
    plot3([r_anchor(1, i), r_attach(1, i)], ...
          [r_anchor(2, i), r_attach(2, i)], ...
          [r_anchor(3, i), r_attach(3, i)], ...
          '-', 'Color', lineColor, 'LineWidth', lineWidth);
end
end

%[appendix]{"version":"1.0"}
%---
