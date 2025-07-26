% CALIBRATION - Calcula la constante K para el sistema VLP
% 
% Este script lee los datos de calibración del CSV en dataset/calibration
% y calcula la constante K = d^2 * Pr, donde:
% - d es la distancia entre transmisor (0,0,2) y receptor (x,y,z)
% - Pr es la potencia recibida corregida (medida - background)

clear; clc;

% Definir rutas
calibration_dir = '../dataset/calibration/';
calibration_file = 'data_0.000000_0.000000_1.000000.csv';
calibration_path = fullfile(calibration_dir, calibration_file);

% Verificar que el archivo existe
if ~exist(calibration_path, 'file')
    error('Archivo de calibración no encontrado: %s', calibration_path);
end

% Leer datos del CSV
fprintf('Leyendo datos de calibración de: %s\n', calibration_path);
data = readtable(calibration_path);

% Extraer coordenadas del nombre del archivo
[~, filename, ~] = fileparts(calibration_file);
coords_str = strrep(filename, 'data_', '');
coords = sscanf(coords_str, '%f_%f_%f');

if length(coords) ~= 3
    error('No se pudieron extraer las coordenadas del nombre del archivo');
end

x_recv = coords(1);
y_recv = coords(2);
z_recv = coords(3);

fprintf('Posición del receptor: (%.6f, %.6f, %.6f)\n', x_recv, y_recv, z_recv);

% Posición del transmisor (fija)
x_trans = 0;
y_trans = 0;
z_trans = 2;

% Calcular distancia entre transmisor y receptor
d = sqrt((x_recv - x_trans)^2 + (y_recv - y_trans)^2 + (z_recv - z_trans)^2);
fprintf('Distancia transmisor-receptor: %.6f m\n', d);

% Filtrar datos para inclinación=0 y azimuth=0 con stage="direction"
direction_mask = (data.inclinacion == 0) & (data.azimuth == 0) & strcmp(data.stage, 'direction');
direction_data = data(direction_mask, :);

% Filtrar datos de background
background_mask = strcmp(data.stage, 'background');
background_data = data(background_mask, :);

if isempty(direction_data)
    warning('No se encontraron datos de direction con inclinación=0 y azimuth=0');
    fprintf('Tipos de stage disponibles: %s\n', strjoin(unique(data.stage), ', '));
    fprintf('Valores únicos de inclinación: %s\n', num2str(unique(data.inclinacion)));
    fprintf('Valores únicos de azimuth: %s\n', num2str(unique(data.azimuth)));
    
    % Si no hay datos direction, usar solo background para demostración
    if ~isempty(background_data)
        background_mean = mean(background_data.medida_daq);
        fprintf('Solo datos de background disponibles. Media: %.6f\n', background_mean);
        K = d^2 * background_mean; % Esto es solo para demostración
        fprintf('K calculada con datos de background: %.6f\n', K);
        return;
    else
        error('No se encontraron datos válidos para calibración');
    end
end

if isempty(background_data)
    error('No se encontraron datos de background para la calibración');
end

% Calcular medias
direction_mean = mean(direction_data.medida_daq);
background_mean = mean(background_data.medida_daq);

% Calcular potencia recibida corregida
Pr = direction_mean - background_mean;

% Calcular constante K
K = d^2 * Pr;

% Mostrar resultados
fprintf('\n--- Resultados de Calibración ---\n');
fprintf('Número de medidas direction (0°,0°): %d\n', height(direction_data));
fprintf('Número de medidas background: %d\n', height(background_data));
fprintf('Media potencia direction: %.6f\n', direction_mean);
fprintf('Media potencia background: %.6f\n', background_mean);
fprintf('Potencia recibida corregida (Pr): %.6f\n', Pr);
fprintf('Distancia (d): %.6f m\n', d);
fprintf('Constante de calibración K = d² × Pr: %.6f\n', K);

% Guardar resultado en archivo
save('calibration_results.mat', 'K', 'd', 'Pr', 'direction_mean', 'background_mean', ...
     'x_recv', 'y_recv', 'z_recv');

fprintf('\nResultados guardados en calibration_results.mat\n');
fprintf('Variable K disponible en el workspace: %.6f\n', K);
