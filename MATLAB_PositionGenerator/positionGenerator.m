clc, clear, close all

% Parameters to set 
% *******************************************************
bed = [-1.2,1.2,-1.2,1.2];
step= 0.2;
% *******************************************************


% *******************************************************
graph = 0; % 1:yes, 0:no
save = 1; 
% *******************************************************


[TbX,TbY] = meshgrid(bed(1):step:bed(2), bed(3):step:bed(2) );

% Invertir TbY
TbY(:,2:2:end) = TbY(end:-1:1,2:2:end);

x = reshape(TbX,[numel(TbX),1]);
y = reshape(TbY,[numel(TbY),1]);

% Graficar
fprintf("Nro of points: %d \n",length(x))
if(graph)
    figure(1)
    axis([-2 2 -2 2])
    grid minor
    hold on
    for i=1:numel(TbX)
        plot(x(i),y(i),'ob','MarkerFaceColor','b','MarkerSize',4)
        pause(0.05)
    end
end


% Agregar los "0" ya que aun no se han registrado.
data = [x, y, zeros(length(x), 1)];

% Escribir los datos en un archivo de texto
if (save)
    dlmwrite('positions.txt', data, 'delimiter', ' ');
    fprintf("Generacion de datos exitoso \n")
end