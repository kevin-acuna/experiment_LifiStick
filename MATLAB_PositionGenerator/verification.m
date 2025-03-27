%% Programa para procesar puntos de un archivo TXT
% Este script lee un archivo con datos (X, Y, etiqueta) y para cada punto con 
% etiqueta 0 calcula la distancia y el ángulo con respecto a una referencia (Xr, Yr).
% Si el punto cumple que su distancia está en el rango [d_min, d_max] y su ángulo 
% (con el eje Y) está en el rango [angle_min, angle_max], se le asigna una nueva etiqueta.
% Luego se grafica: puntos con etiqueta 0 en gris y puntos con etiqueta diferente de 0 
% en colores aleatorios (cada etiqueta única tendrá un color distinto).
clc, clear, close all

%% Parámetros
Xr = 0.6;            % Coordenada X de referencia (modificar según sea necesario)
Yr = -0.6;            % Coordenada Y de referencia (modificar según sea necesario)
newLabel = 9;      % Nueva etiqueta a asignar si se cumplen las condiciones

d_min = 0.4;         % Distancia mínima requerida
d_max = 0.8;         % Distancia máxima requerida
angle_min = 86;    % Ángulo mínimo en grados
angle_max = 184;    % Ángulo máximo en grados
%% Lectura de datos
filename = 'positions.txt';   % Nombre del archivo de entrada (asegúrate que esté en el path)
data = load(filename);      % Se asume que el archivo tiene tres columnas: [X, Y, etiqueta]

% Extraer columnas
X = data(:,1);
Y = data(:,2);
labels = data(:,3);

%% Procesamiento de los puntos
% Se evaluarán solo aquellos puntos con etiqueta 0
for i = 1:length(X)
    if labels(i) == 0
        % Calcular vector desde la referencia hasta el punto
        dx = X(i) - Xr;
        dy = Y(i) - Yr;
        dist = sqrt(dx^2 + dy^2);
        
        % Calcular el ángulo que forma el vector con el eje Y en grados
        % Si la distancia es 0, el ángulo se define como 0 para evitar división por cero.
        if dist == 0
            angulo = 0;
        else
            angulo = atan2(-dx,dy) * (180/pi);
        end
        
        % Evaluar condiciones de distancia y ángulo
        if (dist >= d_min) && (dist <= d_max) && (angulo >= angle_min) && (angulo <= angle_max)
            labels(i) = newLabel;  % Actualizar etiqueta
        end
    end
end

% Actualizar la matriz de datos con las etiquetas modificadas
data(:,3) = labels;

%% Graficación
figure;
hold on;

% Graficar puntos con etiqueta 0 (se muestran en gris)
idx0 = labels == 0;
scatter(X(idx0), Y(idx0), 36, [0.5 0.5 0.5], 'filled');

% Graficar puntos con etiqueta diferente de cero en colores aleatorios
uniqueLabels = unique(labels(labels~=0));
for k = 1:length(uniqueLabels)
    currentLabel = uniqueLabels(k);
    color = rand(1,3);  % Generar un color aleatorio
    idx = labels == currentLabel;
    scatter(X(idx), Y(idx), 36, color, 'filled');
end

xlabel('X');
ylabel('Y');
title('Distribución de Puntos con Etiquetas Actualizadas');
grid on;
hold off;

%% Guardar resultados (opcional)
% Se guarda la matriz actualizada en un nuevo archivo de texto.
save('positions.txt', 'data', '-ascii');
