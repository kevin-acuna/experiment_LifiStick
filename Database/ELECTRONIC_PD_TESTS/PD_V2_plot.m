close all, clear all, clc

%% 0) Parámetros de muestreo y carga de datos
Fs   = 1000;           % Frecuencia de muestreo [Hz]
dur  = 4 * 60;         % Duración total [s]
N    = Fs * dur;       % Número de muestras
t    = (0:N-1) / Fs;   % Vector tiempo [s]

% Carga de CSV (una columna, sin cabecera)
voltaje = csvread('data_test_100cm_0degree.csv');  

voltaje = -voltaje;
umbral = 0;

if numel(voltaje) ~= N
    warning('Se esperaban %d muestras, pero se han cargado %d.', N, numel(voltaje));
end


%% 1) Media de todos los puntos > 1 V y trazo de recta horizontal
idx = voltaje > umbral;                % máscara de muestras mayores a 1 V
mu  = mean(voltaje(idx))         % media de esas muestras
std = std(voltaje(idx))

figure('Color','w','Position',[100 100 800 400]);
plot(t, voltaje, 'LineWidth',1);   hold on;
yline(mu, '--r', sprintf('Media >1V = %.3f V',mu), ...
      'LineWidth',2, 'LabelHorizontalAlignment','left');
xlabel('Tiempo (s)','FontSize',14,'FontWeight','bold');
ylabel('Voltaje (V)','FontSize',14,'FontWeight','bold');
title('Señal de Voltaje y Media de Puntos > 1 V','FontSize',16);
grid on; box on;
set(gca, 'FontName','Arial','FontSize',12,'TickDir','out','Layer','top');
xlim([0 dur]);

%% 1.1) Histograma de la distribución de puntos > 1 V
figure('Color','w','Position',[120 120 600 400]);
h = histogram(voltaje(idx), 'NumBins', 100, 'Normalization', 'count');
xlabel('Voltaje (V)',        'FontSize',14,'FontWeight','bold');
ylabel('Repetición',         'FontSize',14,'FontWeight','bold');
title('Distribución de Voltajes > 1 V','FontSize',16);
grid on; box on;
set(gca, ...
    'FontName','Arial', ...
    'FontSize',12, ...
    'LineWidth',1, ...
    'TickDir','out', ...
    'Layer','top' ...
);
% Ajuste de estilo del histograma
h.FaceColor = [0.2 0.6 0.8];
h.EdgeColor = [0.1 0.3 0.4];
h.LineWidth = 1;

%% 2) FFT de la señal original y espectro en dB
Y      = fft(voltaje);
nHalf  = floor(N/2);                   
P2     = abs(Y) / N;                   % espectro de dos lados normalizado
P1     = P2(1:nHalf+1);                % extraemos un lado
P1(2:end-1) = 2 * P1(2:end-1);         % doblamos amplitudes excepto DC y Nyquist

f      = Fs * (0:nHalf) / N;           % vector de frecuencias 0–Fs/2
P1dB   = 20*log10(P1);                 % conversión a decibelios

figure('Color','w','Position',[150 150 800 400]);
plot(f, P1dB, 'LineWidth', 1.5);
xlabel('Frecuencia (Hz)',  'FontSize', 14, 'FontWeight', 'bold');
ylabel('Amplitud (dB)',    'FontSize', 14, 'FontWeight', 'bold');
title('Espectro de Frecuencia (FFT) en dB','FontSize', 16);
grid on; box on;
set(gca, ...
    'FontName', 'Arial', ...
    'FontSize', 12, ...
    'LineWidth', 1, ...
    'TickDir', 'out', ...
    'Layer', 'top' ...
);
xlim([0, Fs/2]);

%% 3) Filtro digital pasabajo (Butterworth) y señal filtrada
fc     = 2;                      % frecuencia de corte [Hz]
orden  = 3;                        % orden del filtro
Wn     = fc/(Fs/2);                % frecuencia normalizada
[b,a]  = butter(orden, Wn, 'low'); % diseño del filtro
volt_f = filtfilt(b, a, voltaje);  % filtrado sin desfase


figure('Color','w','Position',[100 100 800 400]);
plot(t, voltaje, 'LineWidth',1);   hold on;
yline(mu, '--r', sprintf('Media >1V = %.3f V',mu), ...
      'LineWidth',2, 'LabelHorizontalAlignment','left');
xlabel('Tiempo (s)','FontSize',14,'FontWeight','bold');
ylabel('Voltaje (V)','FontSize',14,'FontWeight','bold');
title('Señal de Voltaje y Media de Puntos > 1 V','FontSize',16);
grid on; box on;
set(gca, 'FontName','Arial','FontSize',12,'TickDir','out','Layer','top');
xlim([0 dur]);

plot(t, volt_f, 'LineWidth',1.5,'Color','g');
xlabel('Tiempo (s)','FontSize',14,'FontWeight','bold');
ylabel('Voltaje filtrado (V)','FontSize',14,'FontWeight','bold');
title(sprintf('Señal Filtrada Pasabajo (%d Hz, Butterworth orden %d)',fc,orden),'FontSize',16);
grid on; box on;
set(gca, 'FontName','Arial','FontSize',12,'TickDir','out','Layer','top');
xlim([0 dur]);

%% 1.1) Histograma de la distribución de puntos > 1 V

idx_volt_f = volt_f > umbral; 

figure('Color','w','Position',[120 120 600 400]);
h = histogram(volt_f(idx_volt_f), 'NumBins', 1000, 'Normalization', 'count');
xlabel('Voltaje (V)',        'FontSize',14,'FontWeight','bold');
ylabel('Repetición',         'FontSize',14,'FontWeight','bold');
title('Distribución de Voltajes > 1 V','FontSize',16);
grid on; box on;
set(gca, ...
    'FontName','Arial', ...
    'FontSize',12, ...
    'LineWidth',1, ...
    'TickDir','out', ...
    'Layer','top' ...
);
% Ajuste de estilo del histograma
h.FaceColor = [0.2 0.6 0.8];
h.EdgeColor = [0.1 0.3 0.4];
h.LineWidth = 1;
