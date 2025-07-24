%% Compare WLS Robust and GLS Estimators for VLP
% Este script compara los estimadores WLS y GLS para VLP,
% correspondientes a los casos 5 y 6 del archivo main_3D_withNoise.m
% Usa parámetros del sistema de analyze_PEB_vs_theta_half.m pero solo con theta_half=45°
%
% Author: Kevin Acuña
% Date: July 2025


% Escenarios con problemas:
% 1. Cuando tenemos N=9 el WLS presenta errores considerables.
% 2. El comportamiento comparativo del RMS del CRLB debe de considerarse.
%    - Quitar en la grafica el CDF del CRLB.
%    - Comparar el RMSE de CRLB.

close all;
clear variables;
clc;


% Seleccionar modo de posiciones del receptor
% Opciones: 
%   'fixed'  - Utiliza posiciones fijas en grid de testbed (como en analyze_PEB_vs_theta_half.m)
%   'random' - Utiliza 1000 posiciones aleatorias (como en main_3D_withNoise.m)
receiver_mode = 'fixed';  % Cambiar aquí para seleccionar el modo deseado

% Control para filtrado de valores imaginarios
% true: elimina todos los valores imaginarios para estadísticas más precisas
% false: mantiene todos los valores (comportamiento original)
filter_imaginary = false;  % Cambiar a false para mantener todos los valores

rng(42)
N_or = 9;  % Número de orientaciones

%% 1. System Parameters (from analyze_PEB_vs_theta_half.m with theta_half=45°)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Number of samples per orientation
N_samples=1000;

% LED Parameters
theta_half = 45;                        % Semi-ángulo a media potencia (45°)
P_t = 0.405;                           % Potencia óptica transmitida [W]
T = [0, 0, 2];                         % Posición de la fuente de luz (origen)
m_t = -log(2)./log(cosd(theta_half));  % Orden lambertiano

% Photodetector Parameters
p = 4.8e-3; q = 5.5e-3;               % Dimensiones del fotodiodo rectangular [m]
N_det = 1;                             % Número de fotodiodos
A_det = p*q*N_det;                     % Área sensible del fotoreceptor [m²]
R_pd = 0.63;                           % Fotosensibilidad del fotodiodo [A/W]
FOV = 85;                              % Campo de visión del fotoreceptor [°]
n_r = [0, 0, 1];                       % Vector normal del fotoreceptor
sigma2 = 30e6*10^(-21.0);     % Varianza AWGN [A²]
C = -P_t*(m_t+1)*A_det/(2*pi);         % Factor de normalización

% Use optimized orientations for K=5 from the CRLB analysis
% [theta1, rho1, theta2, rho2, ...] donde theta es elevación y rho es azimuth
% Configurations
orientations_K3 = [36.93, 56.20, 35.42, 176.85, 33.39, 296.52];
orientations_K4 = [36.87, 17.59, 41.59, 198.61, 42.40, 108.42, 39.37, 293.57];
orientations_K5 = [0.48, 294.81,57.57, 87.79, 57.71, 358.55, 57.17, 177.68, 55.72, 268.14];
% orientations_K5 = [0.48, 294.81,30, 87.79,30, 358.55, 30, 177.68, 30, 268.14];
% orientations_K5 = [0.48, 294.81,30.57, 87.79, 30.71, 358.55, 30.17, 177.68, 30.72, 268.14];
% orientations_K5 = [0.48, 294.81, 85, 87.79, 85, 358.55, 85, 177.68, 85, 268.14];
orientations_K6 = [53.23, 179.80, 58.97, 355.37, 48.42, 97.78, 49.58, 268.13, 19.80, 252.81, 25.95, 39.19];
orientations_K7 = [27.60, 355.20, 49.75, 182.12, 51.74, 280.40, 39.06, 251.04, 58.92, 352.88, 16.73, 71.81, 42.72, 104.45];
orientations_K8 = [32.76, 218.19, 28.47, 61.48, 51.87, 178.18, 35.72, 25.47, 51.63, 338.81, 57.74, 273.57, 49.66, 106.23, 18.14, 243.22];
%orientations_K9 = [11.75, 76.79, 26.09, 251.86, 64.05, 261.27, 60.74, 358.44, 57.22, 187.22, 63.10, 175.67, 44.54, 119.76, 58.20, 85.09, 25.17, 304.81];
orientations_K9 = [0, 76.79, 26.09, 251.86, 64.05, 261.27, 60.74, 358.44, 57.22, 187.22, 63.10, 175.67, 44.54, 119.76, 58.20, 85.09, 25.17, 304.81];
all_orientations = {orientations_K3, orientations_K4, orientations_K5, orientations_K6, orientations_K7, orientations_K8, orientations_K9};
K_values = [3, 4, 5, 6, 7, 8, 9];

% Convert spherical orientation angles to cartesian vectors
n_t = zeros(N_or, 3);
for i = 1:N_or
    theta_i = all_orientations{N_or-2}(2*i-1);  % elevation angle
    rho_i = all_orientations{N_or-2}(2*i);      % azimuth angle
    % Convert from spherical to cartesian coordinates
    n_t(i,1) = sind(theta_i) * cosd(rho_i);  % x component
    n_t(i,2) = sind(theta_i) * sind(rho_i);  % y component
    n_t(i,3) = -cosd(theta_i);               % z component (negative because pointing down)
end

%% 3. Generate Receiver Positions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Define common parameters
L = 3; % Length of room [m]
W = 3; % Width of room [m]
Hmax = 1.2; % Maximum height [m]

% Parametros para el estudio 
% step = 0.2; % step X,Y
% stepH = 0.2; % Step size [m]

% % Parametros para la Figura Comparacion de posiciones estimadas vs reales
step = 0.25; % step X,Y
stepH = 0.6; % Step size [m]

if strcmp(receiver_mode, 'fixed')
    % Opción 1: Posiciones fijas (testbed grid de analyze_PEB_vs_theta_half.m)
    % Generate 3D grid of positions
    [X, Y, Z] = meshgrid(-L/2:step:L/2, -W/2:step:W/2, 0:stepH:Hmax);
    
    % Convert to vector form
    X_r = X(:)';
    Y_r = Y(:)';
    Z_r = Z(:)';
    
    % Count the number of positions
    N_pos = length(X_r);
    
    fprintf('Usando %d posiciones fijas en grid (testbed)\n', N_pos);
else
    % Opción 2: Posiciones aleatorias (como en main_3D_withNoise.m)
    N_pos = 1000; % Number of random Rx positions simulated
    X_r = -L/2 + L.*rand(1,N_pos); % x-axis Rx coordinate
    Y_r = -W/2 + W.*rand(1,N_pos); % y-axis Rx coordinate
    Z_r = Hmax*rand(1,N_pos); % z-axis Rx coordinate (random altitudes)
    
    fprintf('Usando %d posiciones aleatorias\n', N_pos);
end

fprintf('Testing with %d receiver positions\n', N_pos);
fprintf('Position range: X ∈ [%.1f, %.1f], Y ∈ [%.1f, %.1f], Z ∈ [%.1f, %.1f]\n', ...
    min(X_r), max(X_r), ...
    min(Y_r), max(Y_r), ...
    min(Z_r), max(Z_r));

% Parameters for channel simulation
param_r = {A_det, n_r, FOV};           % Vector de parámetros del receptor

% 3. Simulation Core - Calculate Received Powers
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Initialize variables
P_r = cell(N_pos, N_or);               % Potencia real recibida [W]
P_r_noisy = cell(N_pos, N_or);         % Potencia con ruido [W]
v_tr = zeros(N_pos, 3);                % Vector unitario real desde Tx a Rx
d_tr = zeros(N_pos, 1);                % Distancia real entre Tx y Rx
SNR = cell(N_pos, N_or);               % SNR en dB

% Inicializar variables para estimadores
estPosWLS_Robust = zeros(N_pos, 3);    % Posiciones estimadas WLS
estPosGLS = zeros(N_pos, 3);           % Posiciones estimadas GLS

% Calculate received powers for each position and orientation
SNR_avg = [];
for i_pos = 1:N_pos
    x = X_r(i_pos); y = Y_r(i_pos); z = Z_r(i_pos);
    for i_dir = 1:N_or
        param_t = {T, n_t(i_dir,:), P_t, m_t};
        [~, P_r{i_pos,i_dir}, v_tr(i_pos,:), d_tr(i_pos,1)] = OWC_LOS_channel(x, y, z, param_t, param_r);
        
        % Add noise to received power - 1000 noise realizations
        P_r_noisy{i_pos,i_dir} = (P_r{i_pos,i_dir} + sqrt(sigma2).*randn(1,N_samples));
        
        % Calculate SNR
        SNR{i_pos,i_dir} = 10*log10((R_pd*P_r{i_pos,i_dir})^2/sigma2);
        SNR_avg = [SNR_avg, 10*log10((R_pd*P_r{i_pos,i_dir})^2/sigma2)];
    end
end

% Replace -Inf SNR values with -80 dB for averaging
pos_negInf = isinf(SNR_avg);
SNR_avg(pos_negInf) = -80;
average_SNR = mean(SNR_avg);
fprintf('Promedio SNR: %.2f dB\n', average_SNR);

%% 4. Position Estimation using WLS Robust and GLS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

time_WLS=[];
time_GLS=[];

for i_pos = 1:N_pos
    x_real = X_r(i_pos); y_real = Y_r(i_pos); z_real = Z_r(i_pos); % Posición real del receptor
    
    %---------------------------------------------------------------------------------------%
    % Case 5: WLS con pesos optimizados y distancia por beamsteering
    %---------------------------------------------------------------------------------------%
    % Obtener potencias crudas para cada orientación
    P_raw = zeros(N_samples, N_or); % Cada fila es una muestra, cada columna una orientación
    for i_dir = 1:N_or
        P_raw(:, i_dir) = P_r_noisy{i_pos, i_dir}; % N_samples muestras de potencia para cada orientación
    end

    % % =============================
    % %Por propocitos de debug (usar el breakpoint)
    % if(i_pos==1568)
    %     kevin=1; 
    % end
    % % =============================

    tic;
    % Obtener dirección usando el método WLS 
    [d_hat_robust, ~, ~] = vlp_wls(n_t', P_raw, m_t);
    
    % Calcular distancia usando el enfoque beamsteering (Tx-Rx)
    v_tr_est_WLS_Robust = d_hat_robust';
    param_t_axis = {T, v_tr_est_WLS_Robust, P_t, m_t};
    param_r_axis = {A_det, -v_tr_est_WLS_Robust, FOV}; % Vector de parámetros del receptor
    [~, P_r_axis_WLS_Robust, ~, ~] = OWC_LOS_channel(x_real, y_real, z_real, param_t_axis, param_r_axis);
    P_r_axis_noisy_WLS_Robust = (P_r_axis_WLS_Robust + sqrt(sigma2).*randn(1,N_samples)); % Potencia con ruido
    
    % Cálculo de la distancia con la fórmula del modelo lambertiano
    d_tr_est_WLS_Robust = sqrt(P_t*(m_t+1)*A_det/(2*pi*mean(P_r_axis_noisy_WLS_Robust)));
    
    % Posición final combinando dirección WLS robusta con distancia
    estPosWLS_Robust(i_pos,:) = T + (v_tr_est_WLS_Robust.*d_tr_est_WLS_Robust);
    tiempo_ejecucion = toc;
    time_WLS = [time_WLS; tiempo_ejecucion];

    %---------------------------------------------------------------------------------------%
    % Case 6: GLS (Generalized Least Squares) con matriz de covarianza completa
    %---------------------------------------------------------------------------------------%
    % Usar mismo P_raw definido antes
    tic;
    % Obtener dirección usando el método GLS (con matriz de covarianza completa)
    [d_hat_gls] = vlp_gls(n_t', P_raw, m_t, sigma2);
    
    % Calcular distancia usando el enfoque beamsteering (Tx-Rx)
    v_tr_est_GLS = d_hat_gls';
    param_t_axis = {T, v_tr_est_GLS, P_t, m_t};
    param_r_axis = {A_det, -v_tr_est_GLS, FOV}; % Vector de parámetros del receptor
    [~, P_r_axis_GLS, ~, ~] = OWC_LOS_channel(x_real, y_real, z_real, param_t_axis, param_r_axis);
    P_r_axis_noisy_GLS = P_r_axis_GLS + sqrt(sigma2).*randn(1,N_samples); % Potencia con ruido
    
    % Cálculo de la distancia con la fórmula del modelo lambertiano
    d_tr_est_GLS = sqrt(P_t*(m_t+1)*A_det/(2*pi*mean(P_r_axis_noisy_GLS)));
    
    % Posición final combinando dirección GLS con distancia por beamsteering
    estPosGLS(i_pos,:) = T + (v_tr_est_GLS.*d_tr_est_GLS);
    tiempo_ejecucion = toc;
    time_GLS = [time_GLS; tiempo_ejecucion];
end

%% 5. Error Calculation and CRLB Bound
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Prepare actual positions as matrix
realPos = [X_r ; Y_r ; Z_r]';

% Calculate error for WLS method
errorWLS_Robust = realPos - estPosWLS_Robust;
errorNormWLS_Robust = zeros(N_pos, 1);
valid_indices_WLS = true(N_pos, 1);  % Vector lógico para marcar índices válidos WLS
num_imaginarios_WLS = 0;

for i = 1:N_pos
    % Verificar si hay componentes imaginarias en la estimación WLS
    if ~isreal(estPosWLS_Robust(i,:))
        valid_indices_WLS(i) = false;  % Marcar como no válido
        num_imaginarios_WLS = num_imaginarios_WLS + 1;
        % Si no filtramos, usar magnitud como error
        if ~filter_imaginary
            errorNormWLS_Robust(i) = norm(abs(errorWLS_Robust(i,:)));
        end
    else
        errorNormWLS_Robust(i) = norm(errorWLS_Robust(i,:));
    end
end

% Procesar resultados WLS según configuración de filtrado
if filter_imaginary
    filtered_errorNormWLS_Robust = errorNormWLS_Robust(valid_indices_WLS);
    fprintf('Número total de valores WLS imaginarios filtrados: %d\n', num_imaginarios_WLS);
else
    filtered_errorNormWLS_Robust = errorNormWLS_Robust;
    fprintf('Número total de valores WLS imaginarios encontrados (no filtrados): %d\n', num_imaginarios_WLS);
end

% Calculate RMSE for WLS
rmseWLS_Robust = sqrt(mean(filtered_errorNormWLS_Robust.^2));

% Calculate error for GLS method
errorGLS = realPos - estPosGLS;
errorNormGLS = zeros(N_pos, 1);
valid_indices_GLS = true(N_pos, 1);  % Vector lógico para marcar índices válidos GLS
num_imaginarios_GLS = 0;

for i = 1:N_pos
    % Verificar si hay componentes imaginarias en la estimación GLS
    if ~isreal(estPosGLS(i,:))
        valid_indices_GLS(i) = false;  % Marcar como no válido
        num_imaginarios_GLS = num_imaginarios_GLS + 1;
        % Si no filtramos, usar magnitud como error
        if ~filter_imaginary
            errorNormGLS(i) = norm(abs(errorGLS(i,:)));
        end
    else
        errorNormGLS(i) = norm(errorGLS(i,:));
    end
end

if filter_imaginary
    % Filtrar valores imaginarios GLS
    filtered_errorNormGLS = errorNormGLS(valid_indices_GLS);
    fprintf('Número total de valores GLS imaginarios filtrados: %d\n', num_imaginarios_GLS);
else
    % Usar todos los valores, incluyendo magnitud de imaginarios
    filtered_errorNormGLS = errorNormGLS;
    fprintf('Número total de valores GLS imaginarios encontrados (no filtrados): %d\n', num_imaginarios_GLS);
end

% Calculate RMSE for GLS
rmseGLS = sqrt(mean(filtered_errorNormGLS.^2));

% Calculate CRLB (theoretical error bound)
tmp_errorNormCRLB = zeros(1, N_pos);
valid_indices_CRLB = true(1, N_pos);  % Vector lógico para marcar índices válidos CRLB
num_imaginarios_CRLB = 0;

for i = 1:N_pos
    % Parámetros para CRLB
    R_real = [X_r(i); Y_r(i); Z_r(i)];  % Posición del receptor
    K = P_t*(m_t+1)*A_det/(2*pi);       % Constante radiométrica P_t(m+1)/(2π)
    
    % Calcular CRLB para esta posición usando la función PEB_complete
    % PEB_complete(R, nt_orientations, T, Pt, m, A_det, theta_half, Psi_FOV, sigma2, N)
    % Convertimos los parámetros para la nueva función
    peb_value = PEB_complete(R_real, n_t', T', P_t, m_t, A_det, deg2rad(theta_half), deg2rad(FOV), sigma2, N_samples);
    
    % Verificar si el valor es imaginario
    if ~isreal(peb_value)
        valid_indices_CRLB(i) = false;  % Marcar como no válido
        num_imaginarios_CRLB = num_imaginarios_CRLB + 1;
        % Si no filtramos, usar magnitud como error
        if ~filter_imaginary
            tmp_errorNormCRLB(i) = abs(peb_value);  % Usar magnitud del valor complejo
        end
    else
        tmp_errorNormCRLB(i) = real(peb_value);  % Guardar solo la parte real
    end
end

if filter_imaginary
    % Filtrar valores imaginarios CRLB
    errorNormCRLB = tmp_errorNormCRLB(valid_indices_CRLB);
    fprintf('Número total de valores CRLB imaginarios filtrados: %d\n', num_imaginarios_CRLB);
else
    % Usar todos los valores, incluyendo magnitud de imaginarios
    errorNormCRLB = tmp_errorNormCRLB;
    fprintf('Número total de valores CRLB imaginarios encontrados (no filtrados): %d\n', num_imaginarios_CRLB);
end

% Calcular RMSE promedio para CRLB (valor teórico)
rmseCRLB = sqrt(mean(errorNormCRLB.^2));

% Calcular percentil 90 para los métodos (usando valores filtrados)
[f_RMS_WLS_Robust, x_RMS_WLS_Robust] = ecdf(filtered_errorNormWLS_Robust(:));
idx90_WLS_Robust = find(f_RMS_WLS_Robust<0.9, 1, 'last');
cdf90_RMS_WLS_Robust_cm = x_RMS_WLS_Robust(idx90_WLS_Robust)*100; % cm

[f_RMS_GLS, x_RMS_GLS] = ecdf(filtered_errorNormGLS(:));
idx90_GLS = find(f_RMS_GLS<0.9, 1, 'last');
cdf90_RMS_GLS_cm = x_RMS_GLS(idx90_GLS)*100; % cm

% Estadísticas para CRLB
[f_RMS_CRLB, x_RMS_CRLB] = ecdf(errorNormCRLB(:));
idx90_CRLB = find(f_RMS_CRLB<0.9, 1, 'last');
cdf90_RMS_CRLB_cm = x_RMS_CRLB(idx90_CRLB)*100; % cm

%% 6. Results Display
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Mostrar resultados de error
fprintf('\n==== RESUMEN DE ERRORES ====\n');
fprintf('\n-- Percentil 90 (CDF) --\n');
fprintf('WLS: %.2f cm\n', cdf90_RMS_WLS_Robust_cm);
fprintf('GLS: %.2f cm\n', cdf90_RMS_GLS_cm);
fprintf('CRLB (límite teórico): %.2f cm\n', cdf90_RMS_CRLB_cm);

fprintf('\n-- RMSE --\n');
fprintf('WLS: %.4f m (%.2f cm)\n', rmseWLS_Robust, rmseWLS_Robust*100);
fprintf('GLS: %.4f m (%.2f cm)\n', rmseGLS, rmseGLS*100);
fprintf('CRLB (límite teórico): %.4f m (%.2f cm)\n', rmseCRLB, rmseCRLB*100);

fprintf('\n-- Ratio respecto al CRLB --\n');
fprintf('WLS: %.2f veces CRLB\n', rmseWLS_Robust/rmseCRLB);
fprintf('GLS: %.2f veces CRLB\n', rmseGLS/rmseCRLB);



%%
% 3D scatter plot
figure(2)
plot3(realPos(:,1), realPos(:,2), realPos(:,3),'ko', 'MarkerSize', 2); hold on;

if filter_imaginary
    % Mostrar solo posiciones WLS reales (filtradas)
    plot3(estPosWLS_Robust(valid_indices_WLS,1), estPosWLS_Robust(valid_indices_WLS,2), estPosWLS_Robust(valid_indices_WLS,3), 'ms', 'MarkerSize', 2);
    
    % Mostrar solo posiciones GLS reales (filtradas)
    plot3(estPosGLS(valid_indices_GLS,1), estPosGLS(valid_indices_GLS,2), estPosGLS(valid_indices_GLS,3), 'cd', 'MarkerSize', 2);
    
else
    % Mostrar todas las posiciones (usando valores reales)
    plot3(estPosWLS_Robust(:,1), estPosWLS_Robust(:,2), estPosWLS_Robust(:,3), 'rs', 'MarkerSize', 2);
    plot3(estPosGLS(:,1), estPosGLS(:,2), estPosGLS(:,3), 'bd', 'MarkerSize', 2);
    
end

xlabel('X [m]','Interpreter','latex');
ylabel('Y [m]','Interpreter','latex');
zlabel('Z [m]','Interpreter','latex');
legend('Reference', 'WLS', 'GLS','Interpreter','latex');
axis([-L/2 L/2 -W/2 W/2 min(Z_r)-0.1 max(Z_r)+0.1])
grid on;
view(44.6,17.28);

% Tiempo de ejecución

fprintf('\nTiempo de ejecución: %.2f segundos\n', tiempo_ejecucion);


