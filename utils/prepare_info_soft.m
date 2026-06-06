function prepare_info_soft(filename, p, point_index, modelType, beta)
%PREPARE_INFO Prepare reference information for MSEA sensing.
%
% Usage:
%   prepare_info(filename, p, point_index)
%       Use normal extension spring Hessian.
%
%   prepare_info(filename, p, point_index, 'soft')
%       Use soft slack extension spring Hessian with beta = 1000.
%
%   prepare_info(filename, p, point_index, 'soft', beta)
%       Use soft slack extension spring Hessian with user-defined beta.
%
% In real implementation, l_str and theta should come directly from sensors.

% ---------- optional inputs ----------
if nargin < 4 || isempty(modelType)
    modelType = 'normal';
end

if nargin < 5 || isempty(beta)
    beta = 1000;
end

modelType = lower(string(modelType));

if modelType ~= "normal" && modelType ~= "soft"
    error("modelType must be either 'normal' or 'soft'.");
end

n = p.fixed.n;

% ---------- define Bis ----------
Bis = zeros(3, 5, n);
for i = 1:n
    if i <= 4  % front 4 springs, shared slider
        Bis(1, 1, i) = 1;
    else       % rear 4 springs, individual sliders
        Bis(1, i-3, i) = 1;
    end
end

% ---------- compute Ji and Jri ----------
Jr = zeros(3, 11, n);
J  = zeros(1, 11, n);

for i = 1:n

    % attach point relative to B origin, expressed in N frame
    roi = p.R_NB * p.r_attach_B(:, i);

    % vector Jacobian of spring vector r_i
    Jri = [-eye(3), skew(roi), -p.R_NB * Bis(:, :, i)];
    Jr(:, :, i) = Jri;

    % length Jacobian: Ji = ui^T * Jri
    ui = p.u(:, i);
    Ji = ui' * Jri;
    J(:, :, i) = Ji;

end

% ---------- compute HLi ----------
HL = zeros(11, 11, n);

for i = 1:n
    ui  = p.u(:, i);
    lei = p.le(i);

    HLi = Jr(:, :, i)' * ((eye(3) - ui * ui') / lei) * Jr(:, :, i);
    HL(:, :, i) = HLi;
end

% ---------- compute H ----------
Kc = zeros(11, 11);
kc = [p.fixed.kc_front; p.fixed.kc_rear(:)];
Kc(end-4:end, end-4:end) = diag(kc);

HLi_sum = zeros(11, 11);

for i = 1:n
    ei  = p.e(i);
    kei = p.fixed.ke(i);

    Ji  = J(:, :, i);
    HLi = HL(:, :, i);

    switch modelType

        case "normal"
            % Normal extension spring model:
            % Ki = k_i * (Ji'Ji + e_i HLi)
            Ki = kei * (Ji' * Ji + ei * HLi);

        case "soft"
            % Soft slack extension spring model:
            % U_i = 0.5 * k_i * softplus(e_i)^2
            %
            % Ki = k_i * (gamma_i Ji'Ji + eta_i HLi)
            %
            % eta_i   = phi_i * phi_i'
            % gamma_i = (phi_i')^2 + phi_i * phi_i''

            [phi, phi_prime, phi_second] = softplus_msea(ei, beta);

            eta   = phi * phi_prime;
            gamma = phi_prime^2 + phi * phi_second;

            Ki = kei * (gamma * (Ji' * Ji) + eta * HLi);

    end

    HLi_sum = HLi_sum + Ki;
end

H = HLi_sum + Kc;

% ---------- extract Kqq Kqs Ksq Kss ----------
Kqq = H(1:6, 1:6);
Kss = H(end-4:end, end-4:end);
Kqs = H(1:6, end-4:end);
Ksq = H(end-4:end, 1:6);

% ---------- calculate Keff and As ----------
Keff = Kqq - Kqs * (Kss \ Ksq);
As   = -Kss \ Ksq;

% ---------- store info in csv file ----------
W_out = p.W_out;
q     = p.q;
l_str = p.l_str;

% ---------- flatten variables ----------
row = [ ...
    point_index, ...
    l_str(:).', ...
    q(:).', ...
    W_out(:).', ...
    reshape(As.', 1, []), ...
    reshape(Keff.', 1, []) ...
];

% ---------- create headers ----------
names = {};

% point index
names{end+1} = 'point_index';

% l_str: 4x1
for i = 1:4
    names{end+1} = sprintf('l_str_%d', i);
end

% q: 6x1
for i = 1:6
    names{end+1} = sprintf('q_%d', i);
end

% W_out: 6x1
for i = 1:6
    names{end+1} = sprintf('W_out_%d', i);
end

% As: 5x6, row-major
for i = 1:5
    for j = 1:6
        names{end+1} = sprintf('As_%d_%d', i, j);
    end
end

% Keff: 6x6, row-major
for i = 1:6
    for j = 1:6
        names{end+1} = sprintf('Keff_%d_%d', i, j);
    end
end

% ---------- write to CSV ----------
T = array2table(row, 'VariableNames', names);

% add .csv to filename if needed
[~, ~, ext] = fileparts(filename);
if isempty(ext)
    filename = [char(filename), '.csv'];
end

if ~isfile(filename)
    writetable(T, filename);
else
    T_existing = readtable(filename);

    row_id = find(T_existing.point_index == point_index, 1);

    if isempty(row_id)
        % point_index does not exist, append new row
        T_existing = [T_existing; T];
    else
        % point_index exists, overwrite corresponding row
        T_existing(row_id, :) = T;
    end

    writetable(T_existing, filename);
end

end

%[appendix]{"version":"1.0"}
%---
