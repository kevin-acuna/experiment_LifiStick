%% Programa para procesar puntos de un archivo TXT con 16 referencias
% Se lee un archivo con datos (X, Y, etiqueta). Luego, para cada uno de los 16
% pares de referencia (Xr, Yr) se evalúan todos los puntos con etiqueta 0.
% Si para una referencia el punto cumple que su distancia está en el rango [d_min, d_max]
% y el ángulo (calculado con respecto al eje Y) está entre [angle_min, angle_max],
% se asigna como etiqueta el número de iteración (1 a 16). 
% Finalmente se grafica:
% - Puntos sin etiquetar (etiqueta 0) como círculos grises.
% - Puntos etiquetados (condición cumplida) como asteriscos de colores aleatorios.

clc; clear; close all;

%% Parámetros de evaluación
d_min = 0.39;         % Distancia mínima requerida
d_max = 0.80;         % Distancia máxima requerida
angle_min = 90;      % Ángulo mínimo en grados
angle_max = 180;     % Ángulo máximo en grados

%% Cargar datos
filename = 'positions.txt';   % Archivo de entrada (asegúrate que esté en el path)
data = load(filename);          % Se espera que tenga tres columnas: [X, Y, etiqueta]
X = data(:,1);
Y = data(:,2);
labels = data(:,3);

%% Definir los 16 pares (Xr, Yr) de referencia
refs = [ -0.6, -0.6;
         -0.6,  0;
         -0.6,  0.6;
         -0.6,  1.2;
          0,    1.2;
          0,    0.6;
          0,    0;
          0,   -0.6;
          0.6, -0.6;
          0.6,  0;
          0.6,  0.6;
          0.6,  1.2;
          1.2,  1.2;
          1.2,  0.6;
          1.2,  0;
          1.2, -0.6];

numRefs = size(refs,1);  % Número de referencias (debe ser 16)

%% Procesamiento: Evaluar cada par (Xr, Yr) de forma secuencial
for i = 1:numRefs
    % Asignar la posición de referencia de la iteración actual
    Xr = refs(i,1);
    Yr = refs(i,2);
    
    % Para cada punto que aún no ha sido etiquetado (etiqueta == 0)
    for j = 1:length(X)
        if labels(j) == 0
            % Calcular vector desde la referencia hasta el punto
            dx = X(j) - Xr;
            dy = Y(j) - Yr;
            dist = sqrt(dx^2 + dy^2);
            
            % Calcular el ángulo que forma el vector con el eje Y (en grados)
            % Se usa atan2 para obtener el ángulo adecuado. Se multiplica -dx para 
            % ajustar la orientación respecto al eje Y.
            if dist == 0
                angulo = 0;
            else
                angulo = mod(atan2(-dx, dy) * (180/pi), 360);
                
            end
            
            % Evaluar condiciones de distancia y ángulo
            if (dist >= d_min) && (dist <= d_max) && (angulo >= angle_min) && (angulo <= angle_max)
                labels(j) = i;  % Asigna el número de iteración como etiqueta
            end
        end
    end
end

% Actualizar la matriz de datos con las etiquetas modificadas
data(:,3) = labels;

%% Graficación
figure;
hold on;

% Graficar puntos sin etiquetar (etiqueta 0) como círculos grises
idx0 = labels == 0;
scatter(X(idx0), Y(idx0), 36, [0.5 0.5 0.5], 'filled');

% Graficar puntos etiquetados (etiqueta diferente de 0) con asterisco y colores aleatorios
uniqueLabels = unique(labels(labels~=0));  % Etiquetas únicas asignadas (1 a 16)
for k = 1:length(uniqueLabels)
    currentLabel = uniqueLabels(k);
    color = rand(1,3);  % Genera un color aleatorio
    idx = labels == currentLabel;
    % Graficar con asterisco; se usa plot en vez de scatter para cambiar el marcador.
    plot(X(idx), Y(idx), '.', 'Color', color, 'MarkerSize', 40);
end

xlabel('X');
ylabel('Y');
title('Distribución de Puntos con Etiquetas Actualizadas');
grid on;
hold off;

%% Guardar resultados (opcional)
% Se guarda la matriz actualizada en un nuevo archivo de texto.
save('positions_actualizadas.txt', 'data', '-ascii');
