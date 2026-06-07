% function to compute a single extension springs 3D force, 1D tension,
% extension length and active indicator given r, k, initial length and
% spring force model

function [F, T, e, u, active] = extension_spring_force(r, k, le0, modelType, beta)

lei = norm(r);
e = lei - le0;

if lei < 1e-12
    error("Spring length too small.");
end

u = r / lei;

if nargin < 4 || isempty(modelType)
    modelType = 'hard';
end

if nargin < 5 || isempty(beta)
    beta = 1000;
end

switch lower(modelType)
    case 'normal'  
        T = k * e;
        active = true;

    case 'hard'
        if e > 0
            T = k * e;
            active = true;
        else
            T = 0;
            active = false;
        end

    case 'soft'
        [phi, phi_prime, ~] = softplus_msea(e, beta);
        T = k * phi * phi_prime;
        active = phi_prime > 0.5;  % softplus boundary

    otherwise
        error("Unknown modelType.");
end

F = T * u;

end

%[appendix]{"version":"1.0"}
%---
