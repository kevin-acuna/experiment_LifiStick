function d_hat = vlp_gls(nt, Praw, m, sigma2)
% VLP_GLS Generalized Least Squares estimator for VLP
% 
% Inputs:
%   nt      : 3×n  — orientaciones n_t^(i) (columnas)
%   Praw    : N×n  — muestras de potencia P_r^(k,i)
%   m       : esc. Lambertiano
%   sigma2  : varianza común de cada muestra
%
% Output:
%   d_hat   : dirección 3×1 del Tx al Rx (normada)

[N,n]  = size(Praw);
mu_hat = mean(Praw,1).';          % μ̂_i  (n×1)
mu_hat = min(max(mu_hat, 0.00000000001), 1000); % Limites minimos para la potencia media.

mu1    = mu_hat(1);
beta   = (mu_hat(2:end)/mu1).^(1/m);      % (n-1)×1

% ---------- 1. matriz de covarianza completa Σ_r ----------
diagVar = beta.^2 .* ( mu_hat(2:end).^(-2) + mu1^(-2) );
Sigma_r = diag(diagVar) + (beta*beta.')*mu1^(-2) ...
          - diag(beta.^2*mu1^(-2));       % tamaño (n-1)×(n-1)

% factor σ²/(N m²) es común → suprimir
% ---------- 2. matriz A ----------
A = zeros(3, n-1);
for i = 2:n
    A(:,i-1) = nt(:,i) - beta(i-1)*nt(:,1);
end

% ---------- 3. matriz de información M ----------
W   = inv(Sigma_r);                %  Σ_r^{-1}
M   = A * W * A.';                 %  3×3

% ---------- 4. autovector de menor autovalor ----------
[V,D]  = eig(M);
[~,ix] = min(diag(D));
d_hat  = V(:,ix) / norm(V(:,ix));  % unitario

% ---------- 5. signo coherente (hacia el receptor) ----------
if dot(d_hat, nt(:,1)) < 0
    d_hat = -d_hat;
end
end
