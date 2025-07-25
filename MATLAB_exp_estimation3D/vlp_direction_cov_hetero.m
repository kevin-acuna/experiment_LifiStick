function d_hat = vlp_direction_cov_hetero(nt, Praw, m)
% nt   : 3×n   vectores de orientación (LED/Tx)
% Praw : N×n   potencias muestreadas (una columna por orientación)
% m    : orden Lambertiano
% Devuelve d_hat (3×1): dirección unit. del Tx al Rx

[N,n]  = size(Praw);

% --- medias y varianzas por orientación ---
mu_hat   = mean(Praw,1).';                    % μ̂_i  (n×1)
sigma2_i = var(Praw,0,1).';                   % σ̂_i² (n×1)  varianza muestral

mu1      = mu_hat(1);
sigma2_1 = sigma2_i(1);

beta     = (mu_hat(2:end)/mu1).^(1/m);        % (n-1)×1

% --- matriz de covarianza completa Σ_r (n-1 × n-1) ---
Sigma_r = zeros(n-1);

for i = 1:n-1
    mui   = mu_hat(i+1);
    var_b = sigma2_i(i+1)*beta(i)^2/(N*m^2) * (mui^-2 + mu1^-2);
    Sigma_r(i,i) = var_b;                     % Var[r_i]

    for j = i+1:n-1                           % Cov[r_i,r_j]
        cov_b = sigma2_1*beta(i)*beta(j)/(N*m^2) * mu1^-2;
        Sigma_r(i,j) = cov_b;
        Sigma_r(j,i) = cov_b;
    end
end

% --- construir A ---
A = zeros(3,n-1);
for i = 2:n
    A(:,i-1) = nt(:,i) - beta(i-1)*nt(:,1);
end

% --- matriz de información y autovector mínimo ---
M      = A / Sigma_r * A.';                   % usa backslash para estabilidad
[V,D]  = eig(M);
[~,ix] = min(diag(D));
d_hat  = V(:,ix) / norm(V(:,ix));

if dot(d_hat, nt(:,1)) < 0
    d_hat = -d_hat;                           % asegurar Tx→Rx
end
end
