clc; clear; close all;
% =====================================================================
% route_coverage.m
% ---------------------------------------------------------------------
% Analiza, para un RECORRIDO especifico de posiciones del robot (las del
% menu de sub3_spatial.cpp), cuantos puntos del receptor (positions3D.txt)
% registrara CADA posicion del recorrido.
%
% Modela EXACTAMENTE la campana real:
%   - El robot visita las posiciones base en el ORDEN del recorrido.
%   - Cada punto se registra UNA sola vez, por la PRIMERA base del
%     recorrido que puede alcanzarlo (en sub3 los puntos alcanzados se
%     marcan "done" y las bases siguientes los saltan). Los puntos no
%     alcanzables por ninguna base quedan sin registrar.
%
% Cobertura del robot (tomada de src/server, coverage 3D):
%   - radio = distancia horizontal XY entre punto y base del robot.
%     (equivalente al pos_robot_vert del servidor: r=sqrt(dx^2+dy^2))
%   - angulo = mod(atan2(-dx,dy)*180/pi, 360)  (misma convencion que el
%     workspace del servidor y que reachability.m).
%   - radios DEPENDIENTES DE LA ALTURA z (config.WS_HEIGHT_RADII); si la
%     altura no esta en la tabla se usan los radios por defecto.
%   - limites angulares [WS_THETA_INIT, WS_THETA_FIN].
%
% Salidas:
%   1) Tabla: por cada paso del recorrido -> base, coords y #puntos.
%   2) Figura 3D: puntos coloreados segun la base que los registra.
%   3) Figura por altura: vista XY con el sector alcanzable de cada base.
%   4) Ficheros: route_coverage_table.csv y positions_labeled_route.txt
% =====================================================================


% =====================================================================
% CONFIGURACION
% =====================================================================
here    = fileparts(mfilename('fullpath'));
posFile = fullfile(here, 'positions3D.txt');   % X Y Z done (4 columnas)

% --- Posiciones base del robot disponibles en el menu (sub3_spatial.cpp)
%     PREDEFINED_POSITIONS[][2] en el marco global {G} [m].
MENU_POSITIONS = [ ...
   -1.5 -0.5;  -1.5  0.0;  -1.5  0.5;  -1.5  1.0;  -1.5  1.5;   % 1..5
   -1.0 -0.5;  -1.0  0.0;  -1.0  0.5;  -1.0  1.0;  -1.0  1.5;   % 1..5
   -0.5 -0.5;  -0.5  0.0;  -0.5  0.5;  -0.5  1.0;  -0.5  1.5;   % 1..5
    0.0 -0.5;   0.0  0.0;   0.0  0.5;   0.0  1.0;   0.0  1.5;   % 6..10
    0.5 -0.5;   0.5  0.0;   0.5  0.5;   0.5  1.0;   0.5  1.5;   % 11..15
    1.0 -0.5;   1.0  0.0;   1.0  0.5;   1.0  1.0;   1.0  1.5;   % 16..20
    1.5 -0.5;   1.5  0.0;   1.5  0.5;   1.5  1.0;   1.5  1.5];  % 21..25

% Offset de calibracion aplicado a la base comandada (experiment_config.h).
ROBOT_OFFSET = [-0.01, 0.012];   % [X Y]  (ROBOT_OFFSET_X, ROBOT_OFFSET_Y)
APPLY_OFFSET = true;             % true -> analiza la base real comandada

% --- RECORRIDO: secuencia de indices del menu (1-based) que vas a visitar.
%     Cambia este vector por el recorrido que quieras analizar.
%     Ejemplo pedido: 1, 2, 4, 5, ...
route = [11:14,16:19,21:24,26:29,1:4,6:9]

% (OPCIONAL) Recorrido con coordenadas personalizadas en vez de indices
% del menu. Si USE_CUSTOM_ROUTE=true se ignora 'route' y se usa esta matriz
% Nx2 de coords base [X Y] (por ejemplo posiciones "custom" de sub3).
USE_CUSTOM_ROUTE = false;
CUSTOM_ROUTE = [ ...
    0.0 0.0;
    0.5 0.5];

% --- Limites angulares del workspace (config.py) ---------------------
WS_THETA_INIT = 180;   % grados
WS_THETA_FIN  = 355;   % grados

% --- Radios dependientes de la altura (config.py WS_HEIGHT_RADII) -----
%     filas: [altura  inner  outer]
WS_HEIGHT_RADII = [ ...
    0.4  0.64  0.74;
    0.6  0.62  0.82;
    0.8  0.50  0.86;
    1.0  0.44  0.84;
    1.2  0.40  0.80];
WS_R_INNER_DEF = 0.65;   % inner por defecto (altura fuera de la tabla)
WS_R_OUTER_DEF = 0.85;   % outer por defecto

% Modo de seleccion de radios:
%   'exact'  -> replica el servidor: si z no esta en la tabla usa defecto.
%   'interp' -> interpola linealmente entre alturas de la tabla.
RADII_MODE = 'exact';


% =====================================================================
% CARGA DE DATOS
% =====================================================================
if ~isfile(posFile)
    error('No se encontro el fichero de posiciones: %s', posFile);
end
data = load(posFile);
X = data(:,1);  Y = data(:,2);  Z = data(:,3);
nPts = size(data,1);
labels = zeros(nPts,1);   % 0 = sin registrar; k = registrado por el paso k

% --- Construir las bases del recorrido -------------------------------
if USE_CUSTOM_ROUTE
    routeBaseSel = CUSTOM_ROUTE;
    menuIdx = nan(size(CUSTOM_ROUTE,1),1);
else
    if any(route < 1) || any(route > size(MENU_POSITIONS,1))
        error('route contiene indices fuera de rango [1..%d].', size(MENU_POSITIONS,1));
    end
    routeBaseSel = MENU_POSITIONS(route, :);   % base "logica" (menu)
    menuIdx = route(:);
end
nSteps = size(routeBaseSel,1);

routeBaseCmd = routeBaseSel;                   % base comandada (con offset)
if APPLY_OFFSET
    routeBaseCmd = routeBaseCmd + ROBOT_OFFSET;
end


% =====================================================================
% TEST DE ALCANCE SECUENCIAL (primera base que alcanza reclama el punto)
% =====================================================================
counts = zeros(nSteps,1);
for k = 1:nSteps
    Rx = routeBaseCmd(k,1);
    Ry = routeBaseCmd(k,2);
    for j = 1:nPts
        if labels(j) ~= 0
            continue;               % ya registrado por una base anterior
        end
        dx = X(j) - Rx;
        dy = Y(j) - Ry;
        r  = hypot(dx, dy);
        [rin, rout] = getRadii(Z(j), WS_HEIGHT_RADII, ...
                               WS_R_INNER_DEF, WS_R_OUTER_DEF, RADII_MODE);
        if r < rin || r > rout
            continue;
        end
        ang = mod(atan2(-dx, dy) * (180/pi), 360);
        if ang >= WS_THETA_INIT && ang <= WS_THETA_FIN
            labels(j) = k;
            counts(k) = counts(k) + 1;
        end
    end
end

nCovered = sum(labels ~= 0);
nMissing = sum(labels == 0);


% =====================================================================
% TABLA DE RESULTADOS
% =====================================================================
step        = (1:nSteps).';
baseX       = routeBaseSel(:,1);
baseY       = routeBaseSel(:,2);
cmdX        = routeBaseCmd(:,1);
cmdY        = routeBaseCmd(:,2);
newPoints   = counts;
cumPoints   = cumsum(counts);

T = table(step, menuIdx, baseX, baseY, cmdX, cmdY, newPoints, cumPoints, ...
    'VariableNames', {'Step','MenuIndex','BaseX','BaseY', ...
                      'CmdX','CmdY','NewPoints','CumPoints'});

fprintf('\n=====================================================\n');
fprintf(' COBERTURA DEL RECORRIDO\n');
fprintf('=====================================================\n');
fprintf(' Puntos totales en el fichero : %d\n', nPts);
fprintf(' Offset aplicado a la base    : %s (%.3f, %.3f)\n', ...
        ternary(APPLY_OFFSET,'SI','NO'), ROBOT_OFFSET(1), ROBOT_OFFSET(2));
fprintf(' Limites angulares            : [%g, %g] deg\n', WS_THETA_INIT, WS_THETA_FIN);
fprintf(' Modo de radios               : %s\n', RADII_MODE);
fprintf('-----------------------------------------------------\n');
disp(T);
fprintf('-----------------------------------------------------\n');
fprintf(' Registrados (cubiertos)      : %d (%.1f%%)\n', nCovered, 100*nCovered/nPts);
fprintf(' Sin registrar (nunca)        : %d (%.1f%%)\n', nMissing, 100*nMissing/nPts);
fprintf('=====================================================\n\n');

% Guardar tabla y posiciones etiquetadas
writetable(T, fullfile(here, 'route_coverage_table.csv'));
labeled = data;  labeled(:,4) = labels;
save(fullfile(here, 'positions_labeled_route.txt'), 'labeled', '-ascii');


% =====================================================================
% COLORES POR PASO DEL RECORRIDO
% =====================================================================
if nSteps <= 7
    colors = lines(max(nSteps,1));
else
    colors = jet(nSteps);
end


% =====================================================================
% FIGURA 1: DISPERSION 3D COLOREADA POR BASE QUE REGISTRA
% =====================================================================
f1 = figure('Name','Cobertura 3D por posicion del robot','Color','w');
hold on; grid on;

legEntries = {};
idx0 = labels == 0;
if any(idx0)
    scatter3(X(idx0), Y(idx0), Z(idx0), 12, [0.8 0.8 0.8], 'o');
    legEntries{end+1} = sprintf('sin registrar (%d)', nMissing);
end
for k = 1:nSteps
    idx = labels == k;
    if any(idx)
        scatter3(X(idx), Y(idx), Z(idx), 30, colors(k,:), 'filled');
        legEntries{end+1} = stepLabel(k, menuIdx, routeBaseSel, counts(k)); %#ok<SAGROW>
    end
end
% Bases del robot (marcadas en z=0)
for k = 1:nSteps
    plot3(routeBaseCmd(k,1), routeBaseCmd(k,2), 0, 'x', ...
          'Color', colors(k,:), 'MarkerSize', 14, 'LineWidth', 2.5);
end

xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('Puntos del receptor coloreados por la base del robot que los registra');
legend(legEntries, 'Location', 'eastoutside');
axis equal; view(35, 25);
hold off;


% =====================================================================
% FIGURA 2: VISTA XY POR ALTURA (con sector alcanzable de cada base)
% =====================================================================
heights = unique(Z);
heights = sort(heights, 'descend');
nH = numel(heights);
nRows = floor(sqrt(nH));
nCols = ceil(nH / nRows);

f2 = figure('Name','Cobertura por altura (vista XY)','Color','w');
for h = 1:nH
    zc = heights(h);
    subplot(nRows, nCols, h);
    hold on; axis equal; grid on;

    maskH = abs(Z - zc) < 1e-9;

    % Sector alcanzable de cada base a esta altura
    [rin, rout] = getRadii(zc, WS_HEIGHT_RADII, WS_R_INNER_DEF, WS_R_OUTER_DEF, RADII_MODE);
    for k = 1:nSteps
        drawSector(routeBaseCmd(k,1), routeBaseCmd(k,2), rin, rout, ...
                   WS_THETA_INIT, WS_THETA_FIN, colors(k,:));
    end

    % Puntos sin registrar a esta altura
    m0 = maskH & (labels == 0);
    if any(m0)
        scatter(X(m0), Y(m0), 18, [0.82 0.82 0.82], 'o');
    end
    % Puntos registrados coloreados por base
    for k = 1:nSteps
        mk = maskH & (labels == k);
        if any(mk)
            scatter(X(mk), Y(mk), 26, colors(k,:), 'filled');
        end
    end
    % Bases
    for k = 1:nSteps
        plot(routeBaseCmd(k,1), routeBaseCmd(k,2), 'x', ...
             'Color', colors(k,:), 'MarkerSize', 11, 'LineWidth', 2);
    end

    nRecH = sum(maskH & labels ~= 0);
    nTotH = sum(maskH);
    title(sprintf('z = %.2f m   (%d/%d)', zc, nRecH, nTotH));
    xlabel('X [m]'); ylabel('Y [m]');
    hold off;
end
sgtitle('Cobertura por altura: sector = zona alcanzable de cada base del recorrido');


% =====================================================================
% GUARDAR FIGURAS
% =====================================================================
saveFigure(f1, fullfile(here, 'route_coverage_3d.png'));
saveFigure(f2, fullfile(here, 'route_coverage_by_height.png'));


% =====================================================================
% FUNCIONES LOCALES
% =====================================================================
function [rin, rout] = getRadii(z, tbl, rinDef, routDef, mode)
    if strcmpi(mode, 'interp')
        h  = tbl(:,1);
        zc = min(max(z, min(h)), max(h));   % clamp al rango de la tabla
        rin  = interp1(h, tbl(:,2), zc);
        rout = interp1(h, tbl(:,3), zc);
    else   % 'exact' -> replica el servidor
        idx = find(abs(tbl(:,1) - z) < 1e-6, 1);
        if isempty(idx)
            rin = rinDef;  rout = routDef;
        else
            rin = tbl(idx,2);  rout = tbl(idx,3);
        end
    end
end

function drawSector(cx, cy, rin, rout, a0, a1, col)
    % Dibuja el sector anular alcanzable en el espacio XY real.
    % Convencion del servidor: theta = atan2(-dx, dy)  =>
    %   x = cx - r*sind(theta);  y = cy + r*cosd(theta).
    t  = linspace(a0, a1, 80);
    xo = cx - rout*sind(t);        yo = cy + rout*cosd(t);
    xi = cx - rin*sind(fliplr(t)); yi = cy + rin*cosd(fliplr(t));
    patch([xo, xi], [yo, yi], col, ...
          'FaceAlpha', 0.08, 'EdgeColor', col, 'LineWidth', 0.6);
end

function s = stepLabel(k, menuIdx, base, cnt)
    if isnan(menuIdx(k))
        s = sprintf('paso %d: (%.2f,%.2f)  [%d]', k, base(k,1), base(k,2), cnt);
    else
        s = sprintf('paso %d (menu %d): (%.2f,%.2f)  [%d]', ...
                    k, menuIdx(k), base(k,1), base(k,2), cnt);
    end
end

function out = ternary(cond, a, b)
    if cond, out = a; else, out = b; end
end

function saveFigure(fig, filename)
    try
        exportgraphics(fig, filename, 'Resolution', 150);
    catch
        saveas(fig, filename);
    end
end
