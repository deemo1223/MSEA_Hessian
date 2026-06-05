function prepare_info(filename, p, point_index)

% in real impelementation, l_str, theta should come from sensor output
% directly

n = p.fixed.n;

% define Bis, fixed
Bis = zeros(3, 5, n);
for i = 1:n
    if i <= 4  % front 4 springs, shared slider
        Bis(1, 1, i) = 1;
    else  % rear 4 springs, individual sliders 
        Bis(1, i-3, i) = 1;
    end
end


% compute Ji and Jri
Jr = zeros(3, 11, n);
J = zeros(1, 11, n);
for i = 1: n
    % compute Jri and put into Jr
    roi = p.R_NB * p.r_attach_B(:, i);
    Jri = [-eye(3) skew(roi) -p.R_NB*Bis(:, :, i)];
    Jr(:, :, i) = Jri;
    % compute Ji as the projection of Jri in the direction of ui
    ui = p.u(:, i);
    Ji = [-ui' ui'*skew(roi) -ui'*p.R_NB*Bis(:, :, i)];
    J(:, :, i) = Ji;
end

% compute HLi
HL = zeros(11, 11, n);
for i = 1: n
    ui = p.u(:, i);
    lei = p.le(i);
    HLi = Jr(:, :, i)' * ((eye(3) - ui*ui')/lei) * Jr(:, :, i);
    HL(:, :, i) = HLi;
end

% compute H
Kc = zeros(11, 11);
Kc(end-4:end, end-4:end) = diag([p.fixed.kc_front p.fixed.kc_rear]);
HLi_sum = zeros(11, 11);
for i = 1: n
    ei = p.e(i);
    kei = p.fixed.ke(i);
    Ji = J(:, :, i);
    HLi = HL(:, :, i);
    HLi_sum = HLi_sum + (kei*(Ji'*Ji) + kei*ei*HLi);
end
H = HLi_sum + Kc;

% extract Kqq Kqs Ksq and Kss from H
Kqq = H(1:6, 1:6);
Kss = H(end-4:end, end-4:end);
Kqs = H(1:6, end-4:end);
Ksq = H(end-4:end, 1:6);

% calcualte Keff
Keff = Kqq - Kqs * (Kss \ Ksq);
As = - Kss \ Ksq;


% store info in csv file
W_out = p.W_out;
q = p.q;
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

% l_str: 4×1
for i = 1:4
    names{end+1} = sprintf('l_str_%d', i);
end

% q: 6×1
for i = 1:6
    names{end+1} = sprintf('q_%d', i);
end

% W_out: 6×1
for i = 1:6
    names{end+1} = sprintf('W_out_%d', i);
end

% As: 5×6, row-major
for i = 1:5
    for j = 1:6
        names{end+1} = sprintf('As_%d_%d', i, j);
    end
end

% Keff: 6×6, row-major
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

% 如果文件不存在：创建文件并写入表头
% 如果文件已存在：
%   1. 若 point_index 已存在，覆盖对应行
%   2. 若 point_index 不存在，追加到下一行

if ~isfile(filename)
    writetable(T, filename);
else
    T_existing = readtable(filename);

    row_id = find(T_existing.point_index == point_index, 1);

    if isempty(row_id)
        % point_index 不存在，追加新行
        T_existing = [T_existing; T];
    else
        % point_index 已存在，覆盖对应行
        T_existing(row_id, :) = T;
    end

    % 重新写入整个表格
    writetable(T_existing, filename);
end
    

%[appendix]{"version":"1.0"}
%---
