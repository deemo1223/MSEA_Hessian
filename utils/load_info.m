function ref = load_info(filename, point_index)

    % 如果 filename 没有 .csv 后缀，自动补上
    [~, ~, ext] = fileparts(filename);
    if ext == ""
        filename = filename + ".csv";
    end

    % ---------- check file ----------
    if ~isfile(filename)
        error('load_info:FileNotFound', ...
              'Reference file not found: %s', filename);
    end

    % ---------- read CSV ----------
    T = readtable(filename);

    % ---------- find row by point_index ----------
    row_id = find(T.point_index == point_index, 1);

    if isempty(row_id)
        error('load_info:IndexNotFound', ...
              'point_index = %d was not found in file: %s', ...
              point_index, filename);
    end

    % 只取对应 index 的那一行
    row = T(row_id, :);

    % ---------- recover variables ----------
    ref.point_index = row.point_index;

    ref.l_str = [
        row.l_str_1;
        row.l_str_2;
        row.l_str_3;
        row.l_str_4
    ];

    ref.q = [
        row.q_1;
        row.q_2;
        row.q_3;
        row.q_4;
        row.q_5;
        row.q_6
    ];

    ref.W_out = [
        row.W_out_1;
        row.W_out_2;
        row.W_out_3;
        row.W_out_4;
        row.W_out_5;
        row.W_out_6
    ];

    % ---------- recover As: 5×6 ----------
    ref.As = zeros(5, 6);

    for i = 1:5
        for j = 1:6
            varName = sprintf('As_%d_%d', i, j);
            ref.As(i, j) = row.(varName);
        end
    end

    % ---------- recover Keff: 6×6 ----------
    ref.Keff = zeros(6, 6);

    for i = 1:6
        for j = 1:6
            varName = sprintf('Keff_%d_%d', i, j);
            ref.Keff(i, j) = row.(varName);
        end
    end

end

%[appendix]{"version":"1.0"}
%---
