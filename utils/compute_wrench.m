function [wrench, l] = compute_wrench(params)

% extract variables
n = params.fixed.n;
ks = params.fixed.ks;
ls_0 = params.fixed.ls_0;
r_anchor = params.fixed.r_anchor;
r_attach = params.r_attach;
r_attach_B = params.r_attach_B;
R_NB = params.R_NB;
slackIdx = params.slackIdx;

% preallocate
s = zeros(3, n);       % spring vector from attach to anchor
s_hat = zeros(3, n);   % normalized spring direction
l = zeros(1, n);       % spring length
a = zeros(3, n);       % moment arm related term
a_hat = zeros(3, n);   % normalized moment term

% compute spring geometry
for i = 1:n
    s(:, i) = r_anchor(:, i) - r_attach(:, i);
    l(i) = norm(s(:, i));

    % if spring is slack, skip all force/moment contribution
    if slackIdx(i) == 1
        continue;
    end

    % normal calculation
    if l(i) > 1e-12
        s_hat(:, i) = s(:, i) / l(i);
        a(:, i) = cross(R_NB * r_attach_B(:, i), s(:, i));
        a_hat(:, i) = a(:, i) / l(i);
    end
end

% form parameter vector
p = [ks, ks .* ls_0]';

% build A matrix
A = zeros(6, 2*n);

for i = 1:n
    % if slack, leave corresponding columns as zero
    if slackIdx(i) == 1
        continue;
    end

    A(1:3, i) = s(:, i);
    A(4:6, i) = a(:, i);

    A(1:3, n+i) = -s_hat(:, i);
    A(4:6, n+i) = -a_hat(:, i);
end

% compute wrench
wrench = A * p;

% remove tiny elements
wrench(abs(wrench) < 1e-5) = 0;

%[appendix]{"version":"1.0"}
%---
