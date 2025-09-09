function [d_hat, beta_hat, w] = vlp_wls(nt, Praw, m)
% VLP_WLS_ROBUST Robust Weighted Least Squares estimator for VLP
% 
% Inputs:
%   nt   : 3×n orientaciones del LED     (columna)
%   Praw : N×n matriz de potencias crudas (filas = muestras k)
%   m    : orden Lambertiano
% 
% Outputs:
%   d_hat    : dirección 3×1 (unitaria) del transmisor al receptor
%   beta_hat : (n-1)×1 razones estimadas
%   w        : (n-1)×1 pesos finales
% 

[N, n] = size(Praw);

% ---- 1. medias μ̂_i
mu_hat = mean(Praw, 1) .';          % n×1

mu_hat = min(max(mu_hat, 0.00000000001), 1000); % Limites minimos para la potencia media.

mu1    = mu_hat(1);
beta_hat = (mu_hat(2:end) / mu1).^(1/m);   % (n-1)×1

% ---- 2. pesos según la fórmula simplificada
denom = beta_hat.^2 .* ( mu_hat(2:end).^(-2) + mu1^(-2) );
w     = 1 ./ denom;                 % (n-1)×1  (constante común omitida)

w = w/max(w);

% ---- 3. matriz M
M = zeros(3);
for i = 2:n
    ai = nt(:,i) - beta_hat(i-1)*nt(:,1);
    M  = M + w(i-1) * (ai*ai.');
end

% ---- 4. autovector de menor autovalor
[V,D]  = eig(M);
[~,ix] = min(diag(D));
d_hat  = V(:,ix) / norm(V(:,ix));

% Corregir signo (direccion Tx → Rx)
if dot(d_hat, nt(:,1)) < 0
    d_hat = -d_hat;                 
end
end
