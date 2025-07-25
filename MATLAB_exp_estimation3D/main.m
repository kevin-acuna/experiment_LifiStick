% Main script for reading CSV data and processing for direction estimation

% Configuración
clear;
close all;
clc;

% Constantes
m = 3; % Orden Lambertiano (ajustar según sea necesario)

% Archivo CSV a leer
% csv_file = '../dataset/data_0.000000_0.000000_1.000000.csv';
% csv_file = '../dataset/data_-0.600000_0.600000_1.200000.csv';
% csv_file = '../dataset/data_-0.600000_0.000000_1.200000.csv';
csv_file = '../dataset/sabado26/data_-0.600000_0.400000_0.800000.csv';


% ======================================================================
% Vector de índices de orientaciones a utilizar (se ajustará después de identificar orientaciones)
% Por ejemplo, K = [1,6,7,8,9] seleccionará las orientaciones en esas posiciones
% K = [1,6,7,8,9]; % Por defecto, vacío significa usar todas las orientaciones
K = [1,2,3,4,5]; % Por defecto, vacío significa usar todas las orientaciones
% K = []
% ======================================================================

% Extraer posición del receptor del nombre del archivo CSV
[~, filename, ~] = fileparts(csv_file);
parts = split(filename, '_');
if length(parts) >= 4
    receiver_x = str2double(parts{2});
    receiver_y = str2double(parts{3});
    receiver_z = str2double(parts{4});
    receiver_pos = [receiver_x, receiver_y, receiver_z];
    transmitter_pos = [0, 0, 2]; % Posición fija del transmisor
    
    % Calcular vector unitario de dirección real (del transmisor al receptor)
    direction_vector = receiver_pos - transmitter_pos;
    direction_unit_vector = direction_vector / norm(direction_vector);
    
    disp(['Posición del receptor: [' num2str(receiver_x) ', ' num2str(receiver_y) ', ' num2str(receiver_z) ']']);
    disp(['Vector de dirección real (unitario): [' num2str(direction_unit_vector(1)) ', ' num2str(direction_unit_vector(2)) ', ' num2str(direction_unit_vector(3)) ']']);
else
    warning('No se pudo extraer la posición del receptor del nombre del archivo');
    direction_unit_vector = [];
end

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

% Calcular la media y varianza de las medidas de background
if ~isempty(background_data)
    background_values = background_data.medida_daq;
    background_mean = mean(background_values);
    background_var = var(background_values);
    disp(['Media de las medidas de background: ' num2str(background_mean)]);
    disp(['Varianza de las medidas de background: ' num2str(background_var)]);
    
    % Graficar la señal de background en una figura separada
    figure;
    plot(1:length(background_values), background_values, 'b', 'MarkerSize', 8);
    hold on;
    plot([1, length(background_values)], [background_mean, background_mean], 'r-', 'LineWidth', 2);
    xlabel('Número de muestra');
    ylabel('Valor medido');
    title('Señal de Background');
    axis([-inf inf 0 1])
    legend('Medidas de background', ['Media (' num2str(background_mean, '%.4f') ')']);
    grid on;
    hold off;
else
    warning('No se encontraron datos de background');
    background_mean = 0; % Valor por defecto si no hay background
    background_var = 0;
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

% Visualizar los datos procesados (Praw) para cada orientación seleccionada
figure;
hold on;
colors = hsv(length(K)); % Colores diferentes para cada orientación
legendInfo = cell(length(K), 1);

disp('\nVarianzas de las señales seleccionadas:');
disp('----------------------------------------');

for idx = 1:length(K)
    i = K(idx); % Índice original de la orientación
    
    % Extraer datos procesados de Praw para esta orientación
    praw_data = Praw(:, idx);
    valid_data = ~isnan(praw_data);
    praw_data = praw_data(valid_data); % Eliminar NaN
    
    if ~isempty(praw_data)
        % Calcular media y varianza de los datos procesados
        media_praw = mean(praw_data);
        var_praw = var(praw_data);
        
        % Mostrar varianza de Praw en la ventana de comandos
        disp(['Praw Orientación ' num2str(i) ': Varianza = ' num2str(var_praw, '%.6f')]);
        
        % Graficar datos procesados de la orientación actual
        plot(1:length(praw_data), praw_data, 'Color', colors(idx,:), 'MarkerSize', 4);
        
        % Información para la leyenda
        legendInfo{idx} = ['Ori. ' num2str(i) ': Inc=' num2str(orientaciones.inclinacion(i)) ...
                           '\circ, Az=' num2str(orientaciones.azimuth(i)) '\circ (media=' num2str(media_praw, '%.4f') ')'];
    else
        legendInfo{idx} = ['Ori. ' num2str(i) ': No hay datos'];
    end
end
disp('----------------------------------------');
xlabel('Número de muestra');
ylabel('Valor procesado (Praw)');
title('Datos procesados (Praw) por orientación seleccionada');
grid on;
legend(legendInfo, 'Location', 'best');
hold off;

% Estimar dirección usando vlp_direction_cov_hetero
disp('Estimando dirección del vector...');
try
    %d_hat = vlp_direction_cov_hetero(nt, Praw, m);
    d_hat = vlp_gls(nt, Praw, m);
    % Mostrar resultados
    disp('Vector de dirección estimado (del transmisor al receptor):');
    disp(d_hat);
    
    % Convertir a ángulos para mejor interpretación
    [azimuth, elevation] = cart2sph(d_hat(1), d_hat(2), d_hat(3));
    azimuth = rad2deg(azimuth);
    elevation = rad2deg(elevation);
    
    disp(['Elevation: ' num2str(elevation) '°', 'Azimuth: ' num2str(azimuth) '°']);
    
    % Comparar con vector real si está disponible
    if ~isempty(direction_unit_vector)
        disp('\nComparación con vector real:');
        disp(['Vector real (unitario): [' num2str(direction_unit_vector(1), '%.4f') ', ' num2str(direction_unit_vector(2), '%.4f') ', ' num2str(direction_unit_vector(3), '%.4f') ']']);
        disp(['Vector estimado: [' num2str(d_hat(1), '%.4f') ', ' num2str(d_hat(2), '%.4f') ', ' num2str(d_hat(3), '%.4f') ']']);
        
        % Calcular error angular
        dot_product = dot(direction_unit_vector, d_hat);
        % Asegurar que el producto punto esté en el rango [-1, 1] para evitar errores numéricos
        dot_product = max(-1, min(1, dot_product));
        angular_error = acos(dot_product);
        angular_error_deg = rad2deg(angular_error);
        
        disp(['Error angular: ' num2str(angular_error_deg, '%.2f') '°']);
        
        % Calcular error euclidiano
        euclidean_error = norm(direction_unit_vector - d_hat);
        disp(['Error euclidiano: ' num2str(euclidean_error, '%.4f')]);
    end
    
    % Visualización
    figure;
    quiver3(0, 0, 0, d_hat(1), d_hat(2), d_hat(3), 'LineWidth', 2, 'Color', 'r');
    hold on;
    
    % Añadir vector de dirección real si está disponible
    if ~isempty(direction_unit_vector)
        quiver3(0, 0, 0, direction_unit_vector(1), direction_unit_vector(2), direction_unit_vector(3), 'LineWidth', 2, 'Color', 'g');
    end
    
    % Visualizar orientaciones de los LEDs
    for i = 1:size(nt, 2)
        quiver3(0, 0, 0, nt(1,i), nt(2,i), nt(3,i), 'LineWidth', 1, 'Color', 'b');
    end
    
    grid on;
    axis equal;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('Comparación: Dirección Real vs Estimada');
    if ~isempty(direction_unit_vector)
        legend('Dirección Estimada', 'Dirección Real', 'Orientaciones de LED');
    else
        legend('Dirección Estimada', 'Orientaciones de LED');
    end
    axis([-1 1 -1 1 -1 0])
    
catch e
    error(['Error en la estimación de dirección: ' e.message]);
end