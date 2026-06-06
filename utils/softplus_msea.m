function [phi, phi_prime, phi_second] = softplus_msea(e, beta)
%SOFTPLUS Smooth approximation of max(0,e)
%
%   phi = softplus(e)
%   phi = softplus(e, beta)
%   [phi, phi_prime, phi_second] = softplus(e, beta)
%
%   phi        = log(1 + exp(beta*e)) / beta
%   phi_prime  = sigmoid(beta*e)
%   phi_second = beta * sigmoid(beta*e) * (1 - sigmoid(beta*e))
%
%   Default beta = 1000.
%
%   e can be scalar, vector, or matrix.

    if nargin < 2 || isempty(beta)
        beta = 1000;
    end

    z = beta .* e;

    phi = zeros(size(e));
    phi_prime = zeros(size(e));

    % stable regions
    idx_pos = z > 50;
    idx_neg = z < -50;
    idx_mid = ~(idx_pos | idx_neg);

    % for large positive z:
    % log(1 + exp(z)) / beta ≈ z / beta = e
    phi(idx_pos) = e(idx_pos);
    phi_prime(idx_pos) = 1;

    % for large negative z:
    % log(1 + exp(z)) / beta ≈ exp(z) / beta
    phi(idx_neg) = exp(z(idx_neg)) ./ beta;
    phi_prime(idx_neg) = exp(z(idx_neg));

    % normal region
    phi(idx_mid) = log(1 + exp(z(idx_mid))) ./ beta;
    phi_prime(idx_mid) = 1 ./ (1 + exp(-z(idx_mid)));

    % second derivative
    phi_second = beta .* phi_prime .* (1 - phi_prime);

end

%[appendix]{"version":"1.0"}
%---
