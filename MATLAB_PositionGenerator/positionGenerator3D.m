clc, clear, close all

% Parameters to set 
% *******************************************************
bed = [-0.1,0.9,-0.1,0.9, 0.75, 0.75];
step= 0.25;
% *******************************************************

% *******************************************************
graph = 1; % 1:yes, 0:no
save = 1; 
axis_pattern = 0;   % 1: eje y
                    % 0: eje x
% *******************************************************


[TbX,TbY] = meshgrid(bed(1):step:bed(2), bed(3):step:bed(4));
% Invertir TbY
TbY(:,2:2:end) = TbY(end:-1:1,2:2:end);

if axis_pattern==1
    x = reshape(TbX,[numel(TbX),1]);
    y = reshape(TbY,[numel(TbY),1]);
else
    y = reshape(TbX,[numel(TbX),1]);
    x = reshape(TbY,[numel(TbY),1]);
end



data=[];
ii=0;
for k = bed(6):-step:bed(5)

    % Agregar los "0" ya que aun no se han registrado.
    if(mod(ii,2)==0)
        data = [data; x, y, k*ones(size(x)), zeros(size(x))];
        ii=ii+1;
    else
        data = [data; x(end:-1:1), y(end:-1:1), k*ones(size(x)), zeros(size(x))];
        ii=ii+1;
    end

end

%%

% Graficar
fprintf("Nro of points: %d \n",length(data))
if(graph)
    figure(1)
    axis([-2, 2, -2, 2, 0, 2])
    view(45,45)
    grid minor
    xlabel('x')
    ylabel('y')
    hold on
    
    for i=1: size(data,1)
        plot3(data(i,1),data(i,2),data(i,3),'ob','MarkerFaceColor','b','MarkerSize',4)
        pause(0.05)
    end
    
end



% Escribir los datos en un archivo de texto
if (save)
    dlmwrite('positions3D.txt', data, 'delimiter', ' ');
    fprintf("Generacion de datos exitoso \n")
end