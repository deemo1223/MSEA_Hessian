% function to visualize the MSEA
function visualize_double_MSEA(params, options, titleStr)

% extract variables
n = params.fixed.n;
r_tube = params.fixed.r_tube;
l_tube = params.fixed.l_tube;
l_rod = params.fixed.l_rod;
lc = params.lc;
lc_delta = params.lc_delta;
u_hat = params.u_hat;
r_anchor = params.fixed.r_anchor;
r_attach = params.r_attach;
r_attach_front = params.r_attach_front;
r_attach_rear = params.r_attach_rear;
slackIdx = params.slackIdx;

% extract initial variables
if nargin >= 2 && strcmp(options, 'initial')
    lc = params.initial.lc_eq;
    lc_delta = params.initial.lc_delta_eq;
    u_hat = params.initial.u_hat0;
    r_attach = params.initial.r_attach_eq;
    r_attach_front = params.initial.r_attach_front_eq;
    r_attach_rear = params.initial.r_attach_rear_eq;
    slackIdx = zeros(1, n);
end


% plot points
anchorRadius = 0.008;
attachRadius = 0.008;
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


% optional title
if nargin >= 2 && ~isempty(titleStr)
    title(titleStr);
end

% compute the front and rear attach point central point
r_attach_center_front = mean(r_attach_front, 2);  % take mean across columns

% plot the tube 
[Xr, Yr, Zr] = create_cylinder(r_attach_center_front, l_tube-lc_delta(1), r_tube, -u_hat);
surf(Xr, Yr, Zr, 'FaceColor', [0.3 0.8 0.3], 'EdgeColor', 'none');
[Xr, Yr, Zr] = create_cylinder(r_attach_center_front, lc_delta(1), r_tube, u_hat);
surf(Xr, Yr, Zr, 'FaceColor', [0.3 0.8 0.3], 'EdgeColor', 'none');

% plot the rod
[Xr, Yr, Zr] = create_cylinder(r_attach_center_front, l_rod, 0.5*r_tube, u_hat);
surf(Xr, Yr, Zr, 'FaceColor', [0.8 0.8 0.3], 'EdgeColor', 'none');

% plot the rear compression springs
for i = 1: n/2
    [Xr, Yr, Zr] = create_cylinder(r_attach_rear(:, i), lc(i+1), attachRadius, u_hat);
    surf(Xr, Yr, Zr, 'FaceColor', [0.8 0 0], 'EdgeColor', 'none');
end

% plot the front compression spring
[Xr, Yr, Zr] = create_cylinder(r_attach_center_front, lc(1), 1.05*r_tube, -u_hat);
surf(Xr, Yr, Zr, 'FaceColor', [0.8 0 0], 'EdgeColor', 'none');

% plot the springs as lines
% Line style parameters
nominalColor = [0.1 0 0.8];   % dark blue
slackColor = [1.0 0.4 0];  % orange 
lineWidth = 1.5;

% plot extension springs 
for i = 1:n
    if slackIdx(i) == 1
        lineColor = slackColor;
    else
        lineColor = nominalColor;
    end
    plot3([r_anchor(1, i), r_attach(1, i)], ...  % X
          [r_anchor(2, i), r_attach(2, i)], ...  % Y 
          [r_anchor(3, i), r_attach(3, i)], ...  % Z
          '-', 'Color', lineColor, 'LineWidth', lineWidth);
end


end

%[appendix]{"version":"1.0"}
%---
