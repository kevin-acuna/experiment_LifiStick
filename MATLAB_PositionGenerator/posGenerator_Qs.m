clc, clear, close all

% Areas en 2D, h fijo y conocido
Q3 = [-1.5 -0.75 -0.75 0];
Q = [Q3];  % define areas, en este caso solo 1
NQ = size(Q,1);
gridstep = 0.25; % steps in the testbed

% Parameters to set 
% *******************************************************
z = [0.75, 0.75];
gridstep= 0.25;
% *******************************************************

% *******************************************************
graph = 1; % 1:yes, 0:no
save = 1; 
axis_pattern = 'x-axis';
% *******************************************************

for i_Q = 1:NQ

    [TbX,TbY] = meshgrid(Q(i_Q,1):gridstep:Q(i_Q,2), Q(i_Q,3):gridstep:Q(i_Q,4));
    % Invertir TbY

    
    if strcmp(axis_pattern,'y-axis')
        TbY(:,2:2:end) = TbY(end:-1:1,2:2:end);
    elseif strcmp(axis_pattern,'x-axis')
        TbX = TbX';
        TbX(:,2:2:end) = TbX(end:-1:1,2:2:end);
        TbY = TbY';
    else
        TbY(:,2:2:end) = TbY(end:-1:1,2:2:end);
    end
    
    x = reshape(TbX,[numel(TbX),1]);
    y = reshape(TbY,[numel(TbY),1]);
    
    
    data=[];
    ii=0;
    for k = z(2):-gridstep:z(1)
    
        % Agregar los "0" ya que aun no se han registrado.
        if(mod(ii,2)==0)
            data = [data; x, y, k*ones(size(x)), zeros(size(x))];
            ii=ii+1;
        else
            data = [data; x(end:-1:1), y(end:-1:1), k*ones(size(x)), zeros(size(x))];
            ii=ii+1;
        end
    
    end

end
%%

% Graficar
fprintf("Nro of points: %d \n",length(data))
if(graph)
    figure(1)
    axis([-2, 2, -2, 2, 0, 2])
    view(108,63)
    grid minor
    xlabel('x')
    ylabel('y')
    hold on
    
    for i=1: size(data,1)
        plot3(data(i,1),data(i,2),data(i,3),'ob','MarkerFaceColor','b','MarkerSize',4)
        pause(0.1)
    end
    
end

% Escribir los datos en un archivo de texto
if (save)
    dlmwrite('areas_Q.txt', data, 'delimiter', ' ');
    fprintf("Generacion de datos exitoso \n")
end