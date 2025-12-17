clc, clear, close all
% =====================================================================
% Configuration
% =====================================================================
bed = [-1.4 , 1.4, -1.4, 1.4, 0.4, 1.2];
step= 0.2;

graph = 1; % 1:yes, 0:no
save = 1; 
axis_pattern = 0;   % 1: eje y
                    % 0: eje x
random_flag = 0;
random_points = 20;

number_repetition = 1;
% =====================================================================
% =====================================================================

if random_flag==0
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

else
    x_min_cm = bed(1)*100;
    x_max_cm = bed(2)*100;
    y_min_cm = bed(3)*100;
    y_max_cm = bed(4)*100;
    x = randi([x_min_cm, x_max_cm],random_points,1)/100; 
    y = randi([y_min_cm, y_max_cm],random_points,1)/100; 
end

data=[];
for i=1:numel(x)
    for j=1:number_repetition
        data = [data; x(i), y(i), bed(5), 0];
    end
end

% Funciona bien pero es para 3D con alturas distintas
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
%         pause(0.01)
    end
    
end



% Escribir los datos en un archivo de texto
if (save)
    dlmwrite('positions3D.txt', data, 'delimiter', ' ');
    fprintf("Generacion de datos exitoso \n")
end