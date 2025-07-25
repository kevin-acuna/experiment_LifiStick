% Main script for reading CSV data and processing for direction estimation

% Configuración
clear;
close all;
clc;

% Constantes
m = 2; % Orden Lambertiano (ajustar según sea necesario)

% Archivo CSV a leer
% csv_file = '../dataset/data_0.000000_0.000000_1.000000.csv';
% csv_file = '../dataset/data_-0.600000_0.600000_1.200000.csv';
csv_file = '../dataset/data_-0.600000_0.400000_1.200000.csv';

% ======================================================================
% Vector de índices de orientaciones a utilizar (se ajustará después de identificar orientaciones)
% Por ejemplo, K = [1,6,7,8,9] seleccionará las orientaciones en esas posiciones
% K = [1,6,7,8,9]; % Por defecto, vacío significa usar todas las orientaciones
K = [1,2,3,4,5]; % Por defecto, vacío significa usar todas las orientaciones
% K = []
% ======================================================================

% Leer archivo CSV
disp(['Leyendo archivo CSV: ' csv_file]);
try
    data = readtable(csv_file);
    disp('CSV leído correctamente');
catch e
    error(['Error al leer CSV: ' e.message]);
end

% Mostrar estructura del CSV
disp('Estructura de los datos:');
disp(data(1:5,:));


% Identificar orientaciones únicas
disp('Identificando orientaciones únicas...');
orientaciones = unique(data(:, {'inclinacion', 'azimuth'}), 'rows');
num_orientaciones = height(orientaciones);
disp(['Se encontraron ' num2str(num_orientaciones) ' orientaciones diferentes.']);

% Mostrar las orientaciones disponibles
disp('Orientaciones disponibles:');
for i = 1:num_orientaciones
    disp([num2str(i) ': Inclinación = ' num2str(orientaciones.inclinacion(i)) ...
         '°, Azimuth = ' num2str(orientaciones.azimuth(i)) '°']);
end

% Solicitar al usuario las orientaciones a utilizar si no se han definido
if isempty(K)
    disp('Usando todas las orientaciones disponibles');
    K = 1:num_orientaciones;
else
    % Validar que los índices estén dentro del rango
    K = K(K >= 1 & K <= num_orientaciones);
    if isempty(K)
        warning('Los índices de orientación seleccionados no son válidos. Usando todas las orientaciones.');
        K = 1:num_orientaciones;
    else
        disp(['Usando ' num2str(length(K)) ' orientaciones seleccionadas de ' num2str(num_orientaciones) ' disponibles.']);
    end
end

% Separar datos de background
background_data = data(strcmp(data.stage, 'background'), :);
disp(['Datos de background: ' num2str(height(background_data)) ' muestras.']);

% Calcular la media de las medidas de background
if ~isempty(background_data)
    background_mean = mean(background_data.medida_daq);
    disp(['Media de las medidas de background: ' num2str(background_mean)]);
else
    warning('No se encontraron datos de background');
    background_mean = 0; % Valor por defecto si no hay background
end

% Crear matriz nt (vectores de orientación) para las orientaciones seleccionadas
nt = zeros(3, length(K));

% Crear matriz para almacenar medidas (Praw)
medidas_por_orientacion = {};
max_samples = 0;



% Procesar cada orientación seleccionada
for idx = 1:length(K)
    i = K(idx); % Índice de la orientación seleccionada
    
    % Obtener ángulos de la orientación actual
    inclinacion = orientaciones.inclinacion(i);
    azimuth = orientaciones.azimuth(i);
    
    % Convertir ángulos a vector de orientación (coordenadas cartesianas)
    % Inclinación desde la vertical (theta) y azimuth desde el eje X (phi)
    theta = deg2rad(inclinacion);
    phi = deg2rad(azimuth);
    
    % Convertir a vector unitario (coordenadas esféricas a cartesianas)
    nt(:, idx) = [sin(theta)*cos(phi); sin(theta)*sin(phi); -cos(theta)];
    
    % Filtrar datos para esta orientación
    orientacion_data = data(data.inclinacion == inclinacion & ...
                            data.azimuth == azimuth & ...
                            strcmp(data.stage, 'direction'), :);
    
    % Guardar medidas para esta orientación
    if ~isempty(orientacion_data)
        medidas_por_orientacion{idx} = orientacion_data.medida_daq;
        max_samples = max(max_samples, height(orientacion_data));
    else
        warning(['No se encontraron muestras para orientación: inclinación = ' ...
                 num2str(inclinacion) ', azimuth = ' num2str(azimuth)]);
        medidas_por_orientacion{idx} = [];
    end
    
    disp(['Orientación ' num2str(i) ' (índice ' num2str(idx) ' de ' num2str(length(K)) '): Inclinación = ' num2str(inclinacion) ...
         '°, Azimuth = ' num2str(azimuth) '° -> ' num2str(length(medidas_por_orientacion{idx})) ' muestras.']);
end


% Crear matriz Praw (N x n) para las orientaciones seleccionadas
Praw = zeros(max_samples, length(K));
for idx = 1:length(K)
    medidas = medidas_por_orientacion{idx};
    if ~isempty(medidas)
        % Llenar la columna correspondiente con las medidas disponibles
        n_samples = length(medidas);
        Praw(1:n_samples, idx) = -(medidas - background_mean);
        
        % Si hay menos muestras que el máximo, rellenar con el último valor
        if n_samples < max_samples
            Praw(n_samples+1:end, idx) = medidas(end);
        end
    else
        % Si no hay medidas para esta orientación, rellenar con ceros o NaN
        Praw(:, idx) = NaN;
    end
end


% Verificar si hay datos válidos para todas las orientaciones
valid_data = ~any(isnan(Praw), 1);
if ~all(valid_data)
    warning(['Faltan datos para ' num2str(sum(~valid_data)) ' orientaciones. Eliminando orientaciones sin datos.']);
    nt = nt(:, valid_data);
    Praw = Praw(:, valid_data);
    disp(['Procesando con ' num2str(size(nt, 2)) ' orientaciones válidas.']);
    
    % Verificar si hay suficientes orientaciones para continuar
    if size(nt, 2) < 2
        error('Se necesitan al menos 2 orientaciones para estimar la dirección.');
    end
end

% Mostrar información de las matrices
disp(['Dimensión de nt: ' num2str(size(nt, 1)) 'x' num2str(size(nt, 2))]);
disp(['Dimensión de Praw: ' num2str(size(Praw, 1)) 'x' num2str(size(Praw, 2))]);

% Visualizar los datos recibidos para cada orientación seleccionada
figure;
hold on;
colors = hsv(length(K)); % Colores diferentes para cada orientación
legendInfo = cell(length(K), 1);

for idx = 1:length(K)
        i=K(idx);
        
        % Graficar medidas de la orientación actual
        plot(1:length(Praw), Praw(:,idx), 'Color', colors(idx,:), 'MarkerSize', 2);
        
        % Calcular y graficar la media
        media_orientacion = mean(Praw(:,idx));
        % plot([1, length(Praw)], [media_orientacion, media_orientacion], '-', 'Color', colors(idx,:), 'LineWidth', 1);
        % 
        % Información para la leyenda
        legendInfo{idx} = ['Ori. ' num2str(i) ': Inc=' num2str(orientaciones.inclinacion(i)) ...
                           '\circ, Az=' num2str(orientaciones.azimuth(i)) '\circ (mean=' num2str(media_orientacion, '%.4f') ')'];
end

xlabel('Número de muestra');
ylabel('Valor medido');
title('Medidas por orientación seleccionada');
grid on;
legend(legendInfo, 'Location', 'best');
hold off;

% Estimar dirección usando vlp_direction_cov_hetero
disp('Estimando dirección del vector...');
try
    d_hat = vlp_direction_cov_hetero(nt, Praw, m);
    
    % Mostrar resultados
    disp('Vector de dirección estimado (del transmisor al receptor):');
    disp(d_hat);
    
    % Convertir a ángulos para mejor interpretación
    [azimuth, elevation] = cart2sph(d_hat(1), d_hat(2), d_hat(3));
    azimuth = rad2deg(azimuth);
    elevation = rad2deg(elevation);
    
    disp(['Elevation: ' num2str(elevation) '°', 'Azimuth: ' num2str(azimuth) '°']);
    
    % Visualización
    figure;
    quiver3(0, 0, 0, d_hat(1), d_hat(2), d_hat(3), 'LineWidth', 2, 'Color', 'r');
    hold on;
    
    % Visualizar orientaciones de los LEDs
    for i = 1:size(nt, 2)
        quiver3(0, 0, 0, nt(1,i), nt(2,i), nt(3,i), 'LineWidth', 1, 'Color', 'b');
    end
    
    grid on;
    axis equal;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('Estimación de dirección y orientaciones');
    legend('Dirección Estimada', 'Orientaciones de LED');
    
catch e
    error(['Error en la estimación de dirección: ' e.message]);
end