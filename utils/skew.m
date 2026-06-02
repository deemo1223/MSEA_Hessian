function S = skew(v)

    v = v(:);   % 保证是 column vector

    if numel(v) ~= 3
        error('skew:InvalidInput', 'Input v must be a 3-element vector.');
    end

    S = [  0    -v(3)   v(2);
          v(3)   0    -v(1);
         -v(2)  v(1)   0   ];

end

%[appendix]{"version":"1.0"}
%---
